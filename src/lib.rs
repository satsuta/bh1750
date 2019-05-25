#![no_std]
extern crate embedded_hal;

use embedded_hal::blocking::delay;
use embedded_hal::blocking::i2c;

#[derive(Debug, Copy, Clone)]
#[allow(non_camel_case_types)]
pub enum Address {
    ADDR_H = 0b101_1100, // 0x5c
    ADDR_L = 0b010_0011, // 0x23
}

#[derive(Debug, Copy, Clone)]
#[allow(non_camel_case_types)]
enum Instruction {
    POWER_DOWN = 0b0000_0000,
    POWER_ON = 0b0000_0001,
    RESET = 0b0000_0111,
}

#[allow(non_camel_case_types)]
pub enum OneTimeMeasurement {
    HIHGT_RES = 0b0010_0000,  // 1    lx resolution
    HIHGT_RES2 = 0b0010_0001, // 0.5  lx resolution
    LOW_RES = 0b0010_0011,    // 4    lx resolution
}

#[allow(non_camel_case_types)]
pub enum ContinuesMeasurement {
    HIHGT_RES = 0b0001_0000,  // 1    lx resolution
    HIHGT_RES2 = 0b0001_0001, // 0.5  lx resolution
    LOW_RES = 0b0001_0011,    // 4    lx resolution
}
pub struct BH1750<I2C, DELAY> {
    com: I2C,
    delay: DELAY,
    address: Address,
}

impl<I2C, DELAY, E> BH1750<I2C, DELAY>
where
    I2C: i2c::WriteRead<Error = E>,
    DELAY: delay::DelayMs<u32>,
{
    /// Create new BH1750 driver
    pub fn new(i2c: I2C, delay: DELAY, address: Address) -> Result<Self, E> {
        let chip = BH1750 {
            com: i2c,
            delay,
            address,
        };
        Ok(chip)
    }

    pub fn light_one_shot(&mut self, mode: OneTimeMeasurement) -> u32 {
        let delay = match mode {
            OneTimeMeasurement::HIHGT_RES => 140,
            OneTimeMeasurement::HIHGT_RES2 => 160,
            OneTimeMeasurement::LOW_RES => 18,
        };
        let command = mode as u8;
        self.send_instruction(command);
        self.delay.delay_ms(delay);
        raw_to_lx(self.resive_answer(command))
    }

    pub fn start_measurement(&mut self, mode: ContinuesMeasurement) {
        let command = mode as u8;
        self.send_instruction(command);
    }

    pub fn reset(&mut self) {
        self.send_instruction(Instruction::RESET as u8);
    }

    pub fn power_down(&mut self) {
        self.send_instruction(Instruction::POWER_DOWN as u8);
    }

    pub fn power_on(&mut self) {
        self.send_instruction(Instruction::POWER_ON as u8);
    }

    pub fn get_measurement(&mut self, mode: ContinuesMeasurement) -> u32 {
        let delay = match mode {
            ContinuesMeasurement::HIHGT_RES => 120,
            ContinuesMeasurement::HIHGT_RES2 => 120,
            ContinuesMeasurement::LOW_RES => 16,
        };
        let command = mode as u8;
        self.delay.delay_ms(delay);
        raw_to_lx(self.resive_answer(command))
    }

    fn send_instruction(&mut self, instr: u8) {
        let mut buffer = [0];
        let _ = self
            .com
            .write_read(self.address as u8, &[instr], &mut buffer);
    }

    fn resive_answer(&mut self, instr: u8) -> u16 {
        let mut data: [u8; 2] = [0; 2];
        let _ = self.com.write_read(self.address as u8, &[instr], &mut data);
        let raw_answer: u16 = ((data[0] as u16) << 8) | data[1] as u16;
        raw_answer
    }
}

fn raw_to_lx(raw: u16) -> u32 {
    (raw as u32) * 12 / 10
}
