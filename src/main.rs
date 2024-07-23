#![no_std]
#![no_main]

const VERBOSE: bool = false;
const CLOCK_STRETCH_DELAY_US: u32 = 0;

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{Io, Level, OutputOpenDrain, Pull},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
};
use esp_println::println;

#[derive(Debug, PartialEq, PartialOrd, Copy, Clone)]
enum I2cState {
    Idle,
    Started,
    AddrBitSclLo,
    AddrBitSclHi,
    AddrAckSclLo,
    AddrAckSclHi,
    DataBitSclLo,
    DataBitSclLoSet,
    DataBitSclHi,
    DataAckSclLo,
    DataAckSclHi,
    AwaitStop,
}

macro_rules! do_clock_stretch {
    ($time:ident,$delay:ident,$scl:ident,$block:block) => {
        if $time > 0 {
            $scl.set_low();
            $block
            $delay.delay_micros($time);
            $scl.set_high();
        } else {
            $block
        }
    };
}

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let mut sda_pin = OutputOpenDrain::new(io.pins.gpio1, Level::High, Pull::Up);
    let mut scl_pin = OutputOpenDrain::new(io.pins.gpio2, Level::High, Pull::Up);

    let delay = Delay::new(&clocks);

    let mut state = I2cState::Idle;

    let mut addr_bit = 0;
    let mut addr: u8 = 0;
    let mut is_read = false;
    let mut sda_capture = false;
    let mut data_bit = 0;
    let mut rdata_index = 0;
    let mut read_data = [0u8; 255];
    let mut wdata_index = 0;
    let wdata = b"012345678901234567890123456789";

    let mut prev_scl = true;
    let mut prev_sda = true;

    loop {
        let sda = sda_pin.is_high();
        let scl = scl_pin.is_high();

        // detect RESTART condition
        if prev_scl && scl && prev_sda && !sda {
            state = I2cState::Idle;

            rdata_index = 0;
            read_data.iter_mut().for_each(|m| *m = 0);
            wdata_index = 0;
        }

        // detect STOP condition
        if prev_scl && scl && !prev_sda && sda {
            state = I2cState::Idle;

            if VERBOSE {
                println!("Stop");
                println!("Addr = {:x}", addr >> 1);
                println!("is_read = {}", addr & 1);
                if addr & 1 == 0 {
                    println!("Count = {}", rdata_index);
                    println!("Data = {:x?}", &read_data[..rdata_index as usize]);
                } else {
                    println!("Count {}", wdata_index);
                }
                println!();
            }

            rdata_index = 0;
            read_data.iter_mut().for_each(|m| *m = 0);
            wdata_index = 0;
        }
        prev_scl = scl;
        prev_sda = sda;

        state = match state {
            I2cState::Idle if !sda && !scl => I2cState::Started,
            I2cState::Started if !scl => {
                addr_bit = 0;
                addr = 0;
                data_bit = 0;
                rdata_index = 0;
                I2cState::AddrBitSclLo
            }

            // address
            I2cState::AddrBitSclLo if scl => {
                sda_capture = sda;
                I2cState::AddrBitSclHi
            }
            I2cState::AddrBitSclHi if !scl => {
                addr = addr | ((if sda_capture { 1 } else { 0 }) << (7 - addr_bit));
                addr_bit += 1;
                if addr_bit == 8 {
                    do_clock_stretch!(CLOCK_STRETCH_DELAY_US, delay, scl_pin, {
                        sda_pin.set_low(); // ACK
                    });
                    is_read = addr & 1 == 0;
                    I2cState::AddrAckSclLo
                } else {
                    do_clock_stretch!(CLOCK_STRETCH_DELAY_US, delay, scl_pin, {});
                    I2cState::AddrBitSclLo
                }
            }
            I2cState::AddrAckSclLo if scl => I2cState::AddrAckSclHi,
            I2cState::AddrAckSclHi if !scl => {
                sda_pin.set_high();
                I2cState::DataBitSclLo
            }

            // read data to device
            I2cState::DataBitSclLo if scl && is_read => {
                sda_capture = sda;
                I2cState::DataBitSclHi
            }
            I2cState::DataBitSclHi if !scl && is_read => {
                read_data[rdata_index] =
                    read_data[rdata_index] | ((if sda_capture { 1 } else { 0 }) << (7 - data_bit));
                data_bit += 1;
                if data_bit == 8 {
                    do_clock_stretch!(CLOCK_STRETCH_DELAY_US, delay, scl_pin, {
                        sda_pin.set_low(); // ACK
                    });
                    data_bit = 0;
                    rdata_index += 1;
                    I2cState::DataAckSclLo
                } else {
                    do_clock_stretch!(CLOCK_STRETCH_DELAY_US, delay, scl_pin, {});
                    I2cState::DataBitSclLo
                }
            }
            I2cState::DataAckSclLo if scl && is_read => I2cState::DataAckSclHi,
            I2cState::DataAckSclHi if !scl && is_read => {
                sda_pin.set_high();
                I2cState::DataBitSclLo
            }

            // write data from device
            I2cState::DataBitSclLo if !scl && !is_read => {
                let bit = (wdata[wdata_index] & (1 << (7 - data_bit))) != 0;
                do_clock_stretch!(CLOCK_STRETCH_DELAY_US, delay, scl_pin, {
                    if bit {
                        sda_pin.set_high();
                    } else {
                        sda_pin.set_low();
                    }
                });

                I2cState::DataBitSclLoSet
            }
            I2cState::DataBitSclLoSet if scl && !is_read => I2cState::DataBitSclHi,
            I2cState::DataBitSclHi if !scl && !is_read => {
                data_bit += 1;
                if data_bit == 8 {
                    sda_pin.set_high();
                    data_bit = 0;
                    wdata_index += 1;
                    I2cState::DataAckSclLo
                } else {
                    I2cState::DataBitSclLo
                }
            }
            I2cState::DataAckSclLo if scl && !is_read => I2cState::DataAckSclHi,
            I2cState::DataAckSclHi if !scl && !is_read => {
                // NACK = end
                if sda {
                    I2cState::AwaitStop
                } else {
                    I2cState::DataBitSclLo
                }
            }

            _ => state,
        };
    }
}
