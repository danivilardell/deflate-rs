use bitstream_io::{BitWrite, BitWriter};
use std::collections::VecDeque;
use std::io;
use std::io::Read;

use anyhow::Ok;

use crate::huffman::HuffmanTree;

const MAX_DISTANCE_BYTES: usize = 32768;
const MAX_BYTES_PER_BLOCK: usize = u16::MAX as usize;
const MAX_BLOCKS: usize = 1;

#[derive(Clone, Copy, Debug)]
pub enum Symbol {
    /// A literal byte
    Literal(u8),

    /// End of a block
    EndOfBlock,

    /// A back-reference
    BackReference {
        length_minus_three: u8,
        distance_minus_one: u16,
    },
}

#[derive(Clone, Copy, Debug)]
enum EncodeStage {
    NewBlock,
    Complete,
}

#[derive(Clone, Copy, Debug)]
pub struct LzssEncoder {
    stage: EncodeStage,
    pub in_buffer: [[u8; MAX_BYTES_PER_BLOCK]; MAX_BLOCKS],
    pub encoded_buf: [[Symbol; MAX_BYTES_PER_BLOCK]; MAX_BLOCKS],
    pub in_len: [usize; MAX_BLOCKS],
    pub enc_len: [usize; MAX_BLOCKS],
    pub total_blocks: usize,
}

#[derive(Clone, Copy, Debug)]
pub struct LzssDecoder {
    stage: EncodeStage,
    pub encoded_buf: [[Symbol; MAX_DISTANCE_BYTES]; MAX_BLOCKS],
    pub out_buf: [[u8; MAX_DISTANCE_BYTES]; MAX_BLOCKS],
    pub enc_len: [usize; MAX_BLOCKS],
    pub out_len: [usize; MAX_BLOCKS],
    pub total_blocks: usize,
}

impl LzssEncoder {
    pub fn new() -> Self {
        Self {
            stage: EncodeStage::NewBlock,
            in_buffer: [[0u8; MAX_BYTES_PER_BLOCK]; MAX_BLOCKS],
            encoded_buf: [[Symbol::Literal(0); MAX_BYTES_PER_BLOCK]; MAX_BLOCKS],
            in_len: [0; MAX_BLOCKS],
            enc_len: [0; MAX_BLOCKS],
            total_blocks: 0,
        }
    }

    fn find_longest_prefix_or_literal(
        &mut self,
        curr_position: usize,
        curr_block: usize,
    ) -> Symbol {
        if curr_position == self.in_len[curr_block] {
            return Symbol::EndOfBlock;
        }

        let mut long_prefix: Symbol = Symbol::Literal(self.in_buffer[curr_block][curr_position]);
        for position in 0..curr_position {
            let mut curr_matching_length = 0;
            while curr_position + curr_matching_length < self.in_len[curr_block]
                && self.in_buffer[curr_block][curr_position + curr_matching_length]
                    == self.in_buffer[curr_block][position as usize + curr_matching_length as usize]
                && curr_matching_length < 258
            {
                curr_matching_length += 1;
            }
            if curr_matching_length >= 3 {
                match long_prefix {
                    Symbol::Literal(_) => {
                        long_prefix = Symbol::BackReference {
                            length_minus_three: (curr_matching_length - 3) as u8,
                            distance_minus_one: (curr_position - position - 1) as u16,
                        };
                    }
                    Symbol::BackReference {
                        length_minus_three,
                        distance_minus_one: _,
                    } => {
                        if curr_matching_length > length_minus_three as usize + 3 {
                            long_prefix = Symbol::BackReference {
                                length_minus_three: (curr_matching_length - 3) as u8,
                                distance_minus_one: (curr_position - position - 1) as u16,
                            };
                        }
                    }
                    _ => (),
                }
            }
        }

        long_prefix
    }

    fn advance_stage(&mut self, curr_block: usize) -> io::Result<()> {
        match self.stage {
            EncodeStage::NewBlock => {
                let mut curr_position: usize = 0;
                loop {
                    match self.find_longest_prefix_or_literal(curr_position, curr_block) {
                        Symbol::Literal(b) => {
                            self.encoded_buf[curr_block][self.enc_len[curr_block]] =
                                Symbol::Literal(b);
                            self.enc_len[curr_block] += 1;
                            curr_position += 1;
                        }
                        Symbol::BackReference {
                            length_minus_three,
                            distance_minus_one,
                        } => {
                            self.encoded_buf[curr_block][self.enc_len[curr_block]] =
                                Symbol::BackReference {
                                    length_minus_three,
                                    distance_minus_one,
                                };
                            self.enc_len[curr_block] += 1;
                            curr_position += length_minus_three as usize + 3;
                        }
                        Symbol::EndOfBlock => {
                            self.encoded_buf[curr_block][self.enc_len[curr_block]] =
                                Symbol::EndOfBlock;
                            self.enc_len[curr_block] += 1;
                            break;
                        }
                    }
                }
            }
            EncodeStage::Complete => {}
        }
        //out.write_all(&self.in_buffer[..self.in_len])?;
        //out.write_all(&self.encoded_buf[..self.enc_len])?;
        io::Result::Ok(())
    }

    pub fn encode(&mut self, curr_block: usize) -> io::Result<()> {
        self.advance_stage(curr_block)?;
        self.total_blocks += 1;

        io::Result::Ok(())
    }
}

impl LzssDecoder {
    pub fn new() -> Self {
        Self {
            stage: EncodeStage::NewBlock,
            encoded_buf: [[Symbol::Literal(0); MAX_DISTANCE_BYTES]; MAX_BLOCKS],
            out_buf: [[0u8; MAX_DISTANCE_BYTES]; MAX_BLOCKS],
            enc_len: [0; MAX_BLOCKS],
            out_len: [0; MAX_BLOCKS],
            total_blocks: 0,
        }
    }
    //TODO Implement decoder
}

impl Symbol {
    pub fn length_code(&self) -> u16 {
        match self {
            Self::Literal(b) => (*b).into(),
            Self::EndOfBlock => 256,
            Self::BackReference {
                length_minus_three,
                distance_minus_one: _,
            } => Self::back_reference_length_code(*length_minus_three),
        }
    }

    pub fn back_reference_length_code(length_minus_three: u8) -> u16 {
        257 + u16::from(match length_minus_three {
            0..=7 => length_minus_three,
            8..=254 => {
                let log2: u8 = length_minus_three.ilog2().try_into().unwrap();
                4 * (log2 - 1) + (length_minus_three >> (log2 - 2) & 0b11)
            }
            255 => 28,
        })
    }

    pub fn back_reference_length_get_extra_bits(length_minus_three: u8) -> u8 {
        match length_minus_three {
            0..=7 => 0,
            8..=254 => {
                let log2: u8 = length_minus_three.ilog2().try_into().unwrap();
                let code = 4 * (log2 - 1) + (length_minus_three >> (log2 - 2) & 0b11);
                let min_val_with_code = ((code - 4 * (log2 - 1)) << (log2 - 2))
                    + ((length_minus_three >> log2) << log2);
                length_minus_three - min_val_with_code
            }
            255 => 0,
        }
    }

    pub fn back_reference_length_extra_bits(length_minus_three: u8) -> u8 {
        match length_minus_three {
            0..=7 => 0,
            8..=254 => u8::try_from(length_minus_three.ilog2()).unwrap() - 2,
            255 => 0,
        }
    }

    pub fn back_reference_distance_code(distance_minus_one: u16) -> u8 {
        match distance_minus_one {
            0..=3 => distance_minus_one.try_into().unwrap(),
            4..=32767 => {
                let log2: u8 = distance_minus_one.ilog2().try_into().unwrap();
                2 * log2 + u8::try_from(distance_minus_one >> (log2 - 1) & 1).unwrap()
            }
            32768.. => panic!("Distance cannot be more than {MAX_DISTANCE_BYTES}"),
        }
    }

    pub fn back_reference_distance_extra_bits(distance_minus_one: u16) -> u8 {
        match distance_minus_one {
            0..=3 => 0,
            4..=32767 => u8::try_from(distance_minus_one.ilog2()).unwrap() - 1,
            32768.. => panic!("Distance cannot be more than {MAX_DISTANCE_BYTES}"),
        }
    }
}

#[derive(Debug)]
pub struct OutBuffer(VecDeque<u8>);

impl OutBuffer {
    pub fn new() -> Self {
        Self(VecDeque::new())
    }

    pub fn push(&mut self, byte: u8) {
        if self.0.len() == MAX_DISTANCE_BYTES {
            self.0.pop_back();
        }

        self.0.push_front(byte);
    }

    pub fn get(&self, distance_minus_one: usize) -> Option<u8> {
        Some(*self.0.get(distance_minus_one)?)
    }
}

#[derive(Debug)]
pub struct EncBuffer();

impl EncBuffer {
    pub fn new() -> Self {
        Self()
    }

    pub fn push_bit<W, E>(&mut self, out: &mut BitWriter<W, E>, bit: bool) -> io::Result<()>
    where
        W: io::Write,
        E: bitstream_io::Endianness,
    {
        out.write_bit(bit)?;
        io::Result::Ok(())
    }

    pub fn push_bits<W, E>(&mut self, out: &mut BitWriter<W, E>, bits: &[u8]) -> io::Result<()>
    where
        W: io::Write,
        E: bitstream_io::Endianness,
    {
        for bit in bits {
            self.push_bit(out, *bit != 0)?;
        }
        io::Result::Ok(())
    }

    pub fn push_bytes<W, E>(&mut self, out: &mut BitWriter<W, E>, bytes: &[u8]) -> io::Result<()>
    where
        W: io::Write,
        E: bitstream_io::Endianness,
    {
        out.write_bytes(bytes)?;
        io::Result::Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    fn expected_lengths_by_code() -> HashMap<u16, Vec<u16>> {
        vec![
            (257, vec![3]),
            (258, vec![4]),
            (259, vec![5]),
            (260, vec![6]),
            (261, vec![7]),
            (262, vec![8]),
            (263, vec![9]),
            (264, vec![10]),
            (265, vec![11, 12]),
            (266, vec![13, 14]),
            (267, vec![15, 16]),
            (268, vec![17, 18]),
            (269, (19..=22).collect()),
            (270, (23..=26).collect()),
            (271, (27..=30).collect()),
            (272, (31..=34).collect()),
            (273, (35..=42).collect()),
            (274, (43..=50).collect()),
            (275, (51..=58).collect()),
            (276, (59..=66).collect()),
            (277, (67..=82).collect()),
            (278, (83..=98).collect()),
            (279, (99..=114).collect()),
            (280, (115..=130).collect()),
            (281, (131..=162).collect()),
            (282, (163..=194).collect()),
            (283, (195..=226).collect()),
            (284, (227..=257).collect()),
            (285, vec![258]),
        ]
        .into_iter()
        .collect::<HashMap<u16, Vec<u16>>>()
    }

    fn expected_distances_by_code() -> HashMap<u8, Vec<u16>> {
        vec![
            (0, vec![1]),
            (1, vec![2]),
            (2, vec![3]),
            (3, vec![4]),
            (4, vec![5, 6]),
            (5, vec![7, 8]),
            (6, (9..=12).collect()),
            (7, (13..=16).collect()),
            (8, (17..=24).collect()),
            (9, (25..=32).collect()),
            (10, (33..=48).collect()),
            (11, (49..=64).collect()),
            (12, (65..=96).collect()),
            (13, (97..=128).collect()),
            (14, (129..=192).collect()),
            (15, (193..=256).collect()),
            (16, (257..=384).collect()),
            (17, (385..=512).collect()),
            (18, (513..=768).collect()),
            (19, (769..=1024).collect()),
            (20, (1025..=1536).collect()),
            (21, (1537..=2048).collect()),
            (22, (2049..=3072).collect()),
            (23, (3073..=4096).collect()),
            (24, (4097..=6144).collect()),
            (25, (6145..=8192).collect()),
            (26, (8193..=12288).collect()),
            (27, (12289..=16384).collect()),
            (28, (16385..=24576).collect()),
            (29, (24577..=32768).collect()),
        ]
        .into_iter()
        .collect()
    }

    #[test]
    fn test_back_reference_length_codes() {
        let mut actual_lengths_by_code = <HashMap<u16, Vec<u16>>>::new();
        for length_minus_three in 0..=255 {
            let length_code = Symbol::back_reference_length_code(length_minus_three);
            let length = u16::from(length_minus_three) + 3;
            actual_lengths_by_code
                .entry(length_code)
                .or_default()
                .push(length);
        }

        assert_eq!(expected_lengths_by_code(), actual_lengths_by_code);
    }

    #[test]
    fn test_back_reference_distance_codes() {
        let mut actual_distances_by_code = <HashMap<u8, Vec<u16>>>::new();
        for distance_minus_one in 0..=32767 {
            let distance_code = Symbol::back_reference_distance_code(distance_minus_one);
            let distance = distance_minus_one + 1;
            actual_distances_by_code
                .entry(distance_code)
                .or_default()
                .push(distance);
        }

        assert_eq!(expected_distances_by_code(), actual_distances_by_code);
    }

    #[test]
    fn test_lzss() {
        let mut encoder = LzssEncoder::new();
        let mut is_eof = false;

        loop {
            loop {
                let in_ = &mut io::stdin().lock();
                match in_.read(
                    &mut encoder.in_buffer[encoder.total_blocks]
                        [encoder.in_len[encoder.total_blocks]..],
                ) {
                    std::result::Result::Ok(0) => {
                        print!("EOF\n");
                        is_eof = true;
                        break;
                    }
                    std::result::Result::Ok(n) => {
                        print!("READ {} BYTES\n", n);
                        encoder.in_len[encoder.total_blocks] += n;
                        if encoder.in_len[encoder.total_blocks] == MAX_BYTES_PER_BLOCK {
                            break;
                        }
                    }
                    Err(e) if matches!(e.kind(), io::ErrorKind::Interrupted) => continue,
                    Err(_) => panic!(),
                }
                let _ = encoder.encode(encoder.total_blocks);
            }

            let mut tot_in_length = 0;
            let mut tot_enc_length = 0;
            for i in 0..encoder.total_blocks {
                print!(
                    "BLOCK {}: {:?}\n",
                    i,
                    &encoder.in_buffer[i][..encoder.in_len[i]]
                );
                tot_in_length += encoder.in_len[i];
            }
            print!("####################\n");
            for i in 0..encoder.total_blocks {
                print!(
                    "ENCODED BLOCK {}: {:?}\n",
                    i,
                    &encoder.encoded_buf[i][..encoder.enc_len[i]]
                );
                tot_enc_length += encoder.enc_len[i];
            }
            print!("####################\n");
            print!("TOTAL INPUT LENGTH: {}\n", tot_in_length);
            print!("TOTAL ENCODED LENGTH: {}\n", tot_enc_length);

            if is_eof {
                encoder.stage = EncodeStage::Complete;
                break;
            }
        }
    }
}
