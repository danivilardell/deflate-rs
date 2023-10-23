use crate::lzss;
use crate::{
    bit_io::BitReader,
    huffman::{DistanceEncoding, HuffmanTree},
    lzss::{EncBuffer, LzssEncoder, OutBuffer, Symbol},
};
use bitstream_io::{BigEndian, BitWriter};
use bitvec::prelude::*;
use std::io;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DeflateEncoding {
    NoCompression,
    FixedHuffman,
    DynamicHuffman,
}

impl TryFrom<&BitSlice<u8>> for DeflateEncoding {
    type Error = io::Error;

    fn try_from(slice: &BitSlice<u8>) -> io::Result<Self> {
        if slice.len() != 2 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("expected 2 encoding bits, got {}", slice.len()),
            ));
        }

        match slice.load_le::<u8>() {
            0b00 => Ok(Self::NoCompression),
            0b01 => Ok(Self::FixedHuffman),
            0b10 => Ok(Self::DynamicHuffman),
            0b11 => Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "0b11 is not a valid encoding",
            )),
            _ => unreachable!(),
        }
    }
}

impl From<DeflateEncoding> for BitVec<u8> {
    fn from(encoding: DeflateEncoding) -> Self {
        let bits: u8 = match encoding {
            DeflateEncoding::NoCompression => 0b00,
            DeflateEncoding::FixedHuffman => 0b01,
            DeflateEncoding::DynamicHuffman => 0b10,
        };
        Self::from_element(bits)
    }
}

fn parse_symbol<R>(
    length_huffman_tree: &HuffmanTree,
    distance_encoding: &DistanceEncoding,
    in_: &mut BitReader<R>,
) -> io::Result<Symbol>
where
    R: io::Read,
{
    let length_code = length_huffman_tree.decode(in_)?;

    match length_code {
        0..=255 => Ok(Symbol::Literal(length_code.try_into().unwrap())),
        256 => Ok(Symbol::EndOfBlock),
        257..=285 => {
            let length_code_minus_257: u8 = (length_code - 257).try_into().unwrap();
            let length_minus_three = match length_code_minus_257 {
                0..=7 => length_code_minus_257,
                8..=27 => {
                    let extra_bit_count = length_code_minus_257 / 4 - 1;
                    let extra_bits = in_.read_u8_from_bits(extra_bit_count.into())?;

                    (1 << (length_code_minus_257 / 4 + 1))
                        + (1 << (length_code_minus_257 / 4 - 1)) * (length_code_minus_257 % 4)
                        + extra_bits
                }
                28 => 255,
                29.. => unreachable!(),
            };

            let distance_code: u8 = distance_encoding.decode(in_)?.try_into().unwrap();
            let distance_minus_one = match distance_code {
                0..=3 => distance_code.into(),
                4..=29 => {
                    let extra_bit_count = distance_code / 2 - 1;
                    let extra_bits = in_.read_u16_from_bits(extra_bit_count.into())?;

                    (1 << (distance_code / 2))
                        + (1 << (distance_code / 2 - 1)) * u16::from(distance_code % 2)
                        + extra_bits
                }
                30.. => {
                    return Err(io::Error::new(
                        io::ErrorKind::InvalidData,
                        format!("distance code must be <= 29, got {distance_code}"),
                    ))
                }
            };

            Ok(Symbol::BackReference {
                length_minus_three,
                distance_minus_one,
            })
        }
        286.. => Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("length code must be <= 285, got {length_code}"),
        )),
    }
}

#[derive(Debug)]
enum DecodeStage {
    NewBlock,
    ParsedMode {
        is_final: bool,
        encoding: DeflateEncoding,
    },
    Complete,
}

#[derive(Debug)]
pub struct DeflateDecoder {
    /// Stores a 32k buffer when blocks are compressed
    out_buffer: OutBuffer,
    stage: DecodeStage,
}

impl DeflateDecoder {
    pub fn new() -> Self {
        Self {
            out_buffer: OutBuffer::new(),
            stage: DecodeStage::NewBlock,
        }
    }

    fn advance_stage<R, W>(&mut self, in_: &mut BitReader<R>, out: &mut W) -> io::Result<()>
    where
        R: io::Read,
        W: io::Write,
    {
        match self.stage {
            DecodeStage::NewBlock => {
                let is_final = in_.read_bool()?;

                let encoding_bits = bits![mut u8, Lsb0; 0; 2];
                in_.read_exact(encoding_bits)?;
                let encoding = (&*encoding_bits).try_into()?;

                self.stage = DecodeStage::ParsedMode { is_final, encoding };

                Ok(())
            }
            DecodeStage::ParsedMode { is_final, encoding } => {
                match encoding {
                    DeflateEncoding::NoCompression => {
                        in_.skip_to_byte_end();

                        let len = in_.read_u16()?;
                        let nlen = in_.read_u16()?;

                        if !len != nlen {
                            return Err(io::Error::new(
                                io::ErrorKind::InvalidData,
                                format!("len {len} does not match nlen {nlen}"),
                            ));
                        }

                        for _ in 0..len {
                            out.write_all(&[in_.read_u8()?])?;
                        }
                    }
                    DeflateEncoding::FixedHuffman => {
                        let literal_huffman_tree = HuffmanTree::fixed_literal();

                        self.decode_huffman_block(
                            in_,
                            out,
                            &literal_huffman_tree,
                            &DistanceEncoding::Fixed,
                        )?;
                    }
                    DeflateEncoding::DynamicHuffman => {
                        let literal_code_length_count = in_.read_u16_from_bits(5)? + 257;
                        let distance_code_length_count = in_.read_u8_from_bits(5)? + 1;
                        let code_length_symbol_count = in_.read_u8_from_bits(4)? + 4;

                        let mut code_lengths_in_symbol_order =
                            Vec::with_capacity(code_length_symbol_count.into());
                        for _ in 0..code_length_symbol_count {
                            let code_length = in_.read_u8_from_bits(3)?;
                            code_lengths_in_symbol_order.push(code_length);
                        }

                        let code_lengths_huffman_tree =
                            HuffmanTree::dynamic_code_lengths(&code_lengths_in_symbol_order);

                        let literal_huffman_tree = code_lengths_huffman_tree
                            .decode_code_lengths(literal_code_length_count.into(), in_)?;

                        let distance_huffman_tree = code_lengths_huffman_tree
                            .decode_code_lengths(distance_code_length_count.into(), in_)?;

                        self.decode_huffman_block(
                            in_,
                            out,
                            &literal_huffman_tree,
                            &DistanceEncoding::Dynamic(distance_huffman_tree),
                        )?;
                    }
                }

                if is_final {
                    in_.skip_to_byte_end();
                    out.flush()?;
                    self.stage = DecodeStage::Complete;
                } else {
                    self.stage = DecodeStage::NewBlock;
                }

                Ok(())
            }
            DecodeStage::Complete => Ok(()),
        }
    }

    fn decode_huffman_block<R, W>(
        &mut self,
        in_: &mut BitReader<R>,
        out: &mut W,
        literal_huffman_tree: &HuffmanTree,
        distance_encoding: &DistanceEncoding,
    ) -> io::Result<()>
    where
        R: io::Read,
        W: io::Write,
    {
        loop {
            let length_symbol = parse_symbol(literal_huffman_tree, distance_encoding, in_)?;

            match length_symbol {
                Symbol::Literal(literal) => {
                    out.write_all(&[literal])?;
                    self.out_buffer.push(literal);
                }
                Symbol::EndOfBlock => {
                    return Ok(());
                }
                Symbol::BackReference {
                    length_minus_three,
                    distance_minus_one,
                } => {
                    let length = u16::from(length_minus_three) + 3;
                    for _ in 0..length {
                        let byte =
                            self.out_buffer
                                .get(distance_minus_one.into())
                                .ok_or_else(|| {
                                    io::Error::new(
                                        io::ErrorKind::InvalidData,
                                        format!(
                                            "invalid backreference with distance {}",
                                            distance_minus_one + 1,
                                        ),
                                    )
                                })?;

                        out.write_all(&[byte])?;
                        self.out_buffer.push(byte);
                    }
                }
            }
        }
    }

    pub fn decode<R, W>(&mut self, in_: &mut BitReader<R>, out: &mut W) -> io::Result<()>
    where
        R: io::Read,
        W: io::Write,
    {
        while !matches!(self.stage, DecodeStage::Complete) {
            self.advance_stage(in_, out)?;
        }

        Ok(())
    }
}

#[derive(Debug)]
pub enum EncodeStage {
    NewBlock {
        is_final: bool,
        encoding: DeflateEncoding,
    },
    ParsedMode {
        encoding: DeflateEncoding,
    },
    Complete,
}

#[derive(Debug)]
pub struct DeflateEncoder {
    pub enc_buffer: EncBuffer,
    pub stage: EncodeStage,
}

impl DeflateEncoder {
    pub fn new(is_final: bool, encoding: DeflateEncoding) -> Self {
        Self {
            enc_buffer: EncBuffer::new(),
            stage: EncodeStage::NewBlock { is_final, encoding },
        }
    }

    fn advance_stage<R, W>(&mut self, in_: &mut R, out: &mut W) -> io::Result<()>
    where
        R: io::Read,
        W: io::Write,
    {
        let bit_writer = &mut BitWriter::new(out);
        match self.stage {
            EncodeStage::NewBlock { is_final, encoding } => {
                self.enc_buffer
                    .push_bit::<_, BigEndian>(bit_writer, is_final)?;
                match encoding {
                    DeflateEncoding::NoCompression => {
                        self.enc_buffer
                            .push_bits::<_, BigEndian>(bit_writer, &[0, 0])?;
                        self.enc_buffer
                            .push_bits::<_, BigEndian>(bit_writer, &[0; 5])?; //padding
                    }
                    DeflateEncoding::FixedHuffman => {
                        self.enc_buffer
                            .push_bits::<_, BigEndian>(bit_writer, &[0, 1])?;
                    }
                    DeflateEncoding::DynamicHuffman => {
                        self.enc_buffer
                            .push_bits::<_, BigEndian>(bit_writer, &[1, 0])?;
                    }
                }
                self.stage = EncodeStage::ParsedMode { encoding };
                Ok(())
            }
            EncodeStage::ParsedMode { encoding } => {
                const MAX_BYTES_PER_BLOCK: usize = u16::MAX as usize;
                let mut lzss_encoder = LzssEncoder::new();
                let mut is_eof = false;

                loop {
                    match in_.read(&mut lzss_encoder.in_buffer[0][lzss_encoder.in_len[0]..]) {
                        Ok(0) => {
                            is_eof = true;
                            break;
                        }
                        Ok(n) => {
                            lzss_encoder.in_len[0] += n;
                            if lzss_encoder.in_len[0] == MAX_BYTES_PER_BLOCK {
                                break;
                            }
                        }
                        Err(e) if matches!(e.kind(), io::ErrorKind::Interrupted) => continue,
                        Err(e) => return Err(e),
                    }
                }

                match encoding {
                    DeflateEncoding::NoCompression => {
                        self.enc_buffer.push_bytes(
                            bit_writer,
                            &lzss_encoder.in_buffer[0][..lzss_encoder.in_len[0]],
                        )?;
                    }
                    DeflateEncoding::FixedHuffman => {
                        let _ = lzss_encoder.encode(lzss_encoder.total_blocks);
                        let encoded_buf = lzss_encoder.encoded_buf;

                        let literal_huffman_tree = HuffmanTree::fixed_literal();

                        for block in &encoded_buf {
                            for symbol in block {
                                match symbol {
                                    Symbol::Literal(byte) => {
                                        literal_huffman_tree.encode(
                                            &mut self.enc_buffer,
                                            bit_writer,
                                            *byte as u16,
                                        )?;
                                    }
                                    Symbol::BackReference {
                                        length_minus_three,
                                        distance_minus_one,
                                    } => {
                                        literal_huffman_tree.encode(
                                            &mut self.enc_buffer,
                                            bit_writer,
                                            lzss::Symbol::back_reference_length_code(
                                                *length_minus_three,
                                            ),
                                        )?;
                                        let length_extra_bits = lzss::Symbol::back_reference_length_get_extra_bits(
                                            *length_minus_three,
                                        );
                                        let length_length_extra_bits = lzss::Symbol::back_reference_length_extra_bits(
                                            *length_minus_three,
                                        );
                                        for i in 0..length_length_extra_bits {
                                            let bit = (length_extra_bits >> (length_length_extra_bits - i)) & 1 == 1;
                                            self.enc_buffer.push_bit(bit_writer, bit)?;
                                        }
                                        for i in 0..5 {
                                            let bit = (distance_minus_one >> (4 - i)) & 1 == 1;
                                            self.enc_buffer.push_bit(bit_writer, bit)?;
                                        }
                                    }
                                    Symbol::EndOfBlock => {
                                        literal_huffman_tree.encode(
                                            &mut self.enc_buffer,
                                            bit_writer,
                                            256 as u16,
                                        )?;
                                        break;
                                    }
                                }

                                if let Symbol::EndOfBlock = symbol {
                                    break;
                                }
                            }
                        }
                    }
                    DeflateEncoding::DynamicHuffman => {}
                }

                if is_eof {
                    self.stage = EncodeStage::Complete;
                }

                bit_writer.flush()?;
                Ok(())
            }
            EncodeStage::Complete => Ok(()),
        }
    }

    pub fn encode<R, W>(&mut self, in_: &mut R, out: &mut W) -> io::Result<()>
    where
        R: io::Read,
        W: io::Write,
    {
        while !matches!(self.stage, EncodeStage::Complete) {
            self.advance_stage(in_, out)?;
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_nocompression_encode() {
        let mut encoder = DeflateEncoder::new(true, DeflateEncoding::NoCompression);
        let _ = encoder.encode(&mut io::stdin().lock(), &mut io::stdout().lock());
    }

    #[test]
    fn test_compression_fixed_huffman_encode() {
        let mut encoder = DeflateEncoder::new(true, DeflateEncoding::FixedHuffman);
        let _ = encoder.encode(&mut io::stdin().lock(), &mut io::stdout().lock());
    }
}
