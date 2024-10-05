#![no_main]
#![no_std]
#![feature(abi_avr_interrupt)]
#![feature(panic_info_message)]

use attiny_hal::pac::cpu::OSCCAL;
use attiny_hal::prelude::*;
use attiny_hal::simple_pwm::IntoPwmPin;
use attiny_hal::simple_pwm::{Prescaler, Timer0Pwm};
use avr_device::asm::nop;
use avr_device::interrupt::{free, Mutex};
use core::cell::RefCell;
use panic_halt as _;

mod buffer;
// mod pwm;
mod uart;

use buffer::Buffer;
// use pwm::Pwm;
use uart::SoftwareUart;

// Define constants for baud rate and CPU frequency
const BAUD_RATE: u32 = 9600 * 2;
const CPU_FREQUENCY: u32 = 8_000_000; // Adjust this to match your ATtiny85's clock speed
const DEVICE_ID: u8 = 0x01;
const OSCCAL_ADJUSTMENT: i16 = -2;
// static UART: Mutex<RefCell<Option<SoftwareUart<BAUD_RATE, CPU_FREQUENCY>>>> =
//     Mutex::new(RefCell::new(None));
static UART: Mutex<RefCell<Option<SoftwareUart>>> = Mutex::new(RefCell::new(None));
static BUFFER: Mutex<RefCell<Buffer>> = Mutex::new(RefCell::new(Buffer::new()));

#[avr_device::entry]
fn main() -> ! {
    let dp = attiny_hal::Peripherals::take().unwrap();
    let pins: attiny_hal::Pins = attiny_hal::pins!(dp);

    // Starting and initializing the timer with prescaling 64.
    // it gives one clock count every 4 µs.
    // since the clock register size is 8 bits, the timer is full every
    // 1/(16e6/64)*2^8 ≈ 10 ms
    //

    // Read the current value of OSCCAL
    let current_value = dp.CPU.osccal.read().bits();
    // Calculate the new value to set
    let new_value = current_value as i16 + OSCCAL_ADJUSTMENT;
    // Write the new value to OSCCAL
    dp.CPU.osccal.write(|w| w.bits(new_value as u8));

    // We need to work with 50 Hz.
    // 8_000_000 / 128

    // let tc0 = dp.TC0;
    // tc0.tccr0a.write(|w| w.wgm0().ctc());
    // tc0.ocr0a.write(|w| w.bits(77));
    // let timer0 = Timer0Pwm::new(tc0, Prescaler::Prescale1024);

    // let mut led = pins.pb4.into_output_high();

    // Set up UART with the new constant generic parameters
    let mut uart: SoftwareUart = SoftwareUart::new(
        dp.TC1,
        pins.pb3.into_floating_input(),
        pins.pb4.into_output_high(),
    );

    // Send a test message
    uart.send(b'T').unwrap();
    uart.send(b'E').unwrap();
    uart.send(b'S').unwrap();
    uart.send(b'T').unwrap();
    uart.send(b'\n').unwrap();
    uart.send(b'\r').unwrap();

    // panic!("Test panic");

    free(|cs| UART.borrow(cs).replace(Some(uart)));

    // uart.send(b'1').unwrap();

    // Set up a timer to set pin high for 1 second, then low for 1 second
    // let mut timer = dp.TC0;
    // timer.tccr0a.write(|w| w.wgm0().ctc());
    // timer.ocr0a.write(|w| w.bits(125));
    // timer.tccr0b.write(|w| w.cs0().prescale_1024());

    // uart.send(b'2').unwrap();

    // // Set up PWM for servos and light
    // // let mut pwm = Pwm::new(dp.TC0, dp.TC1);
    // let mut base_servo = pins.pb0.into_output().into_pwm(&timer0);
    // let mut tilt_servo = pins.pb1.into_output().into_pwm(&timer0);
    // let _light = pins.pb2.into_output();

    // dp.USI.usicr.write(|w| w.usisie().set_bit());

    // Enable Pin Change Interrupt for PB3
    dp.EXINT.gimsk.modify(|_, w| w.pcie().set_bit());
    dp.EXINT.pcmsk.modify(|_, w| w.pcint3().set_bit());

    unsafe { avr_device::interrupt::enable() };

    // uart.send(b'3').unwrap();

    loop {
        // uart.send(b'4').unwrap();

        // Borrow uart and write that a sleep will happen
        // free(|cs| {
        //     if let Some(uart) = UART.borrow(cs).borrow_mut().as_mut() {
        //         uart.send(b'S').unwrap();
        //         uart.send(b'T').unwrap();
        //         uart.send(b'A').unwrap();
        //         uart.send(b'R').unwrap();
        //         uart.send(b'T').unwrap();
        //         uart.send(b'\n').unwrap();
        //         uart.send(b'\r').unwrap();
        //     }
        // });

        free(|cs| {
            // let mut buffer = BUFFER.borrow(cs).borrow_mut();

            // // If the buffer is greater than 1, pop and send the first byte
            // if buffer.len() > 1 {
            //     let byte = buffer.pop().unwrap();
            //     if let Some(uart) = UART.borrow(cs).borrow_mut().as_mut() {
            //         uart.send(byte).unwrap();
            //     }
            // }

            // if buffer.len() == 3 {
            //     let id_and_light = buffer.pop().unwrap();
            //     let rotation = buffer.pop().unwrap();
            //     let tilt = buffer.pop().unwrap();

            //     let _id = (id_and_light & 0xF0) >> 4;
            //     let light_state = id_and_light & 0x0F;

            //     // // Check if this message is for this device
            //     // if id == DEVICE_ID {
            //     //     // Process the message
            //     //     // base_servo.set_duty(rotation);
            //     //     // tilt_servo.set_duty(tilt);
            //     //     // Handle light state
            //     //     if light_state == 0x01 {
            //     //         // Turn light on
            //     //     } else {
            //     //         // Turn light off
            //     //     }
            //     // }

            //     // Echo back the received data

            //     if let Some(uart) = UART.borrow(cs).borrow_mut().as_mut() {
            //         uart.send(rotation).unwrap();
            //         uart.send(tilt).unwrap();
            //         uart.send(light_state).unwrap();
            //     }
            // }
        });

        // Borrow uart and write that a sleep will happen
        // free(|cs| {
        //     if let Some(uart) = UART.borrow(cs).borrow_mut().as_mut() {
        //         uart.send(b'S').unwrap();
        //         uart.send(b'L').unwrap();
        //         uart.send(b'E').unwrap();
        //         uart.send(b'E').unwrap();
        //         uart.send(b'P').unwrap();
        //         uart.send(b'\n').unwrap();
        //         uart.send(b'\r').unwrap();
        //     }
        // });

        // Sleep to save power
        avr_device::asm::sleep();
    }
}

// Change the interrupt handler to PCINT0
#[avr_device::interrupt(attiny85)]
fn PCINT0() {
    free(|cs| {
        if let Some(uart) = UART.borrow(cs).borrow_mut().as_mut() {
            match uart.receive() {
                Ok(byte) => {
                    BUFFER.borrow(cs).borrow_mut().push(byte);
                    uart.send(byte).unwrap(); // Echo received byte
                }
                Err(_) => {
                    // uart.send(b'E').unwrap(); // Send 'E' if error in receiving
                    // uart.send(b'R').unwrap();
                    // uart.send(b'R').unwrap();
                }
            }
            // Print HIT
            // uart.send(b'H').unwrap();
            // uart.send(b'I').unwrap();
            // uart.send(b'T').unwrap();
            // uart.send(b'\n').unwrap();
        }
    });
}

// - When recieving UART, need to throw away data if didn't get 4 in a certain
// amount of data
// - No ID needed, 3 bytes total
// - First test is get data, echo response
// - Make sure can flash to chip
// - Verify PWM math, test servo movement
// - Maybe calibrate oscillator
// - Wait isn't working
