#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl, delay::Delay, gpio::{Io, Level, Output}, i2c::{self, I2C}, peripherals::{Peripherals, I2C0}, prelude::*, system::SystemControl
};
use esp_println::println;

extern crate alloc;
use core::mem::MaybeUninit;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

use mpu6050::*;
use max7219::*;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let clocks = ClockControl::max(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);
    init_heap();
    let mut bz = Output::new(io.pins.gpio10, Level::High);
    esp_println::logger::init_logger_from_env();
    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio8,
        io.pins.gpio9,
        100u32.kHz(),
        &clocks,
        None,
    );
    let mut mpu = Mpu6050::new(i2c);
    let _ = mpu.init(&mut delay);
    let (clk,cs,din) = (Output::new(io.pins.gpio0, Level::High), Output::new(io.pins.gpio1, Level::High), Output::new(io.pins.gpio2, Level::High));
    let mut display = MAX7219::from_pins(1,din, cs, clk ).unwrap();
    display.power_on().unwrap();
    display.set_intensity(0, 0x01).unwrap();
    let data:[u8;8] = [1,2,3,4,5,6,7,8];
    display.write_raw(0b00000000,&data).unwrap();

    loop {
        log::info!("Hello world!");
        let acc = mpu.get_acc().unwrap();
        println!("acc: {:?}", acc[0]);
        if acc[0]<0.5{
            bz.set_high();
        }else {
            bz.set_low();
        }
        delay.delay(500.millis());
    }
}
