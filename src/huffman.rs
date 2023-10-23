use crate::{bit_io::BitReader, lzss::EncBuffer};
use bitstream_io::{BitWrite, BitWriter};
use std::collections::HashMap;
use std::{collections::BTreeMap, io};

// Type should be `[u8; 288]` if `.concat()` could be used in `const` contexts
const FIXED_LITERAL_CODE_LENGTHS: [&[u8]; 4] = [&[8; 144], &[9; 112], &[7; 24], &[8; 8]];

const DYNAMIC_CODE_LENGTH_SYMBOLS: [u8; 19] = [
    16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15,
];

fn compute_heap_index(code: usize, code_len: usize) -> usize {
    code | (1 << code_len)
}

#[derive(Debug)]
pub struct HuffmanTree {
    /// The Huffman tree, encoded as an array-based heap.
    /// The root node is at index 1, and children are at 2n and 2n+1.
    tree: Vec<Option<u16>>,
    symbol_to_code: [[u16; 2]; 288], // [symbol][code, code_len]
}

impl HuffmanTree {
    pub fn from_code_lengths(code_lengths: &[u8]) -> Self {
        let code_length_counts = code_lengths.iter().filter(|&&length| length > 0).fold(
            <BTreeMap<_, u16>>::new(),
            |mut map, &length| {
                *map.entry(length).or_default() += 1;
                map
            },
        );

        let largest_code_length = code_length_counts
            .last_key_value()
            .map_or(0, |(&code_len, _)| code_len);

        let mut next_code = vec![0u16];
        let mut code = 0u16;

        for length in 1..=largest_code_length {
            let count = code_length_counts
                .get(&(length - 1))
                .copied()
                .unwrap_or_default();
            code = (code + count) << 1;
            next_code.push(code);
        }

        let mut tree = vec![None; 1 << (largest_code_length + 1)];
        let mut symbol_to_code = [[0; 2]; 288];
        for (symbol, &code_len) in code_lengths.iter().enumerate() {
            if code_len == 0 {
                continue;
            }

            let code_len = usize::from(code_len);
            let code: usize = next_code[code_len].try_into().unwrap();

            let heap_index = compute_heap_index(code, code_len);
            let heap_symbol: u16 = symbol.try_into().unwrap();
            tree[heap_index] = Some(heap_symbol);
            symbol_to_code[symbol] = [code.try_into().unwrap(), code_len.try_into().unwrap()];

            next_code[code_len] += 1;
        }

        Self {
            tree,
            symbol_to_code,
        }
    }

    pub fn fixed_literal() -> Self {
        Self::from_code_lengths(&FIXED_LITERAL_CODE_LENGTHS.concat())
    }

    pub fn dynamic_code_lengths(code_lengths_in_symbol_order: &[u8]) -> Self {
        assert!(code_lengths_in_symbol_order.len() <= DYNAMIC_CODE_LENGTH_SYMBOLS.len());

        let mut code_lengths = [0; DYNAMIC_CODE_LENGTH_SYMBOLS.len()];
        for (symbol, &code_length) in DYNAMIC_CODE_LENGTH_SYMBOLS
            .into_iter()
            .zip(code_lengths_in_symbol_order)
        {
            code_lengths[usize::from(symbol)] = code_length;
        }

        Self::from_code_lengths(&code_lengths)
    }

    pub fn decode<R>(&self, in_: &mut BitReader<R>) -> io::Result<u16>
    where
        R: io::Read,
    {
        let mut index = 1;
        loop {
            let bit = in_.read_bool()?;
            index = 2 * index + usize::from(bit);

            if index >= self.tree.len() {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!(
                        "invalid Huffman code, got index {index} which is >= {len}",
                        len = self.tree.len()
                    ),
                ));
            }

            if let Some(symbol) = self.tree[index] {
                return Ok(symbol);
            }
        }
    }

    pub fn encode<W, E>(
        &self,
        encoder: &mut EncBuffer,
        out: &mut BitWriter<W, E>,
        symbol: u16,
    ) -> io::Result<usize>
    where
        W: io::Write,
        E: bitstream_io::Endianness,
    {
        let mut index = 1;
        let mut length: u16 = 0;
        let code = self.symbol_to_code[usize::from(symbol)][0];
        let code_len = self.symbol_to_code[usize::from(symbol)][1];
        while length < code_len {
            let bit: bool = (code >> length) % 2 == 1;
            encoder.push_bit(out, bit);
            length += 1;
        }
        Ok(0)
    }

    pub fn decode_code_lengths<R>(
        &self,
        code_length_count: usize,
        in_: &mut BitReader<R>,
    ) -> io::Result<Self>
    where
        R: io::Read,
    {
        let mut code_lengths = vec![];
        let mut prev_code_length = None;

        while code_lengths.len() < code_length_count {
            let symbol: u8 = self.decode(in_)?.try_into().unwrap();
            match symbol {
                0..=15 => {
                    code_lengths.push(symbol);
                    prev_code_length = Some(symbol);
                }
                16 => {
                    let repeat = in_.read_u8_from_bits(2)? + 3;
                    let Some(prev_code_length) = prev_code_length else {
                        return Err(io::Error::new(
                            io::ErrorKind::InvalidData,
                            "saw symbol=16, but no previous code length found",
                        ));
                    };

                    code_lengths.resize(code_lengths.len() + usize::from(repeat), prev_code_length);
                }
                17 => {
                    let repeat = in_.read_u8_from_bits(3)? + 3;
                    code_lengths.resize(code_lengths.len() + usize::from(repeat), 0);

                    prev_code_length = Some(0);
                }
                18 => {
                    let repeat = in_.read_u8_from_bits(7)? + 11;
                    code_lengths.resize(code_lengths.len() + usize::from(repeat), 0);

                    prev_code_length = Some(0);
                }
                19.. => {
                    return Err(io::Error::new(
                        io::ErrorKind::InvalidData,
                        format!("symbol must be <= 18, got {symbol}"),
                    ))
                }
            }
        }

        if code_lengths.len() > code_length_count {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!(
                    "too many code lengths found; expected {code_length_count}, got {len}",
                    len = code_lengths.len(),
                ),
            ));
        }

        Ok(Self::from_code_lengths(&code_lengths))
    }
}

// TODO: Perhaps restrict `HuffmanTree` in `Dynamic` to u8
#[derive(Debug)]
pub enum DistanceEncoding {
    Fixed,
    Dynamic(HuffmanTree),
}

impl DistanceEncoding {
    pub fn decode<R>(&self, in_: &mut BitReader<R>) -> io::Result<u16>
    where
        R: io::Read,
    {
        match self {
            Self::Fixed => in_.read_u16_from_bits(5),
            Self::Dynamic(tree) => tree.decode(in_),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bitvec::prelude::*;

    fn literal_bits(literal: u16, bit_len: usize) -> BitReader<BitVec<u16, Lsb0>> {
        let mut vec = BitVec::from(&literal.view_bits::<Lsb0>()[..bit_len]);
        vec.reverse();

        // Pad to a multiple of 8 so `.read()` will return the last (possibly
        // partial) byte
        vec.resize(((bit_len - 1) / 8 + 1) * 8, false);

        BitReader::new(vec)
    }

    fn assert_decode(
        tree: &HuffmanTree,
        bit_len: usize,
        literals: impl Iterator<Item = u16>,
        symbols: impl Iterator<Item = u16>,
    ) {
        for (literal, symbol) in literals.zip(symbols) {
            assert_eq!(
                tree.decode(&mut literal_bits(literal, bit_len)).unwrap(),
                symbol,
            );
        }
    }

    #[test]
    fn test_fixed_literal_huffman() {
        let tree = HuffmanTree::fixed_literal();
        assert_decode(&tree, 8, 0b00110000..=0b10111111, 0..=143);
        assert_decode(&tree, 9, 0b110010000..=0b111111111, 144..=255);
        assert_decode(&tree, 7, 0b0000000..=0b0010111, 256..=279);
        assert_decode(&tree, 8, 0b11000000..=0b11000111, 280..=287);
    }
}
