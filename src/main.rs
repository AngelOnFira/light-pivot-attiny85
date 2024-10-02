#![no_main]
#![no_std]
#![feature(abi_avr_interrupt)]

use attiny_hal::prelude::*;
use attiny_hal::simple_pwm::IntoPwmPin;
use attiny_hal::simple_pwm::{Prescaler, Timer0Pwm};
use avr_device::interrupt::{free, Mutex};
use core::cell::RefCell;
use panic_halt as _;

// mod buffer;
// mod pwm;
mod uart;

// use buffer::Buffer;
// use pwm::Pwm;
use uart::SoftwareUart;

// Define constants for baud rate and CPU frequency
const BAUD_RATE: u32 = 9600 * 2;
const CPU_FREQUENCY: u32 = 8_000_000; // Adjust this to match your ATtiny85's clock speed
const DEVICE_ID: u8 = 0x01;
// static UART: Mutex<RefCell<Option<SoftwareUart<BAUD_RATE, CPU_FREQUENCY>>>> =
//     Mutex::new(RefCell::new(None));
static UART: Mutex<RefCell<Option<SoftwareUart>>> = Mutex::new(RefCell::new(None));
// static BUFFER: Mutex<RefCell<Buffer>> = Mutex::new(RefCell::new(Buffer::new()));

#[avr_device::entry]
fn main() -> ! {
    let dp = attiny_hal::Peripherals::take().unwrap();
    let pins: attiny_hal::Pins = attiny_hal::pins!(dp);

    // Starting and initializing the timer with prescaling 64.
    // it gives one clock count every 4 µs.
    // since the clock register size is 8 bits, the timer is full every
    // 1/(16e6/64)*2^8 ≈ 10 ms
    //

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
    // free(|cs| UART.borrow(cs).replace(Some(uart)));

    // Set up a timer to set pin high for 1 second, then low for 1 second
    let mut timer = dp.TC0;
    timer.tccr0a.write(|w| w.wgm0().ctc());
    timer.ocr0a.write(|w| w.bits(125));
    timer.tccr0b.write(|w| w.cs0().prescale_1024());

    loop {
        // Send test data
        // if let Some(uart) = UART.borrow(cs).borrow_mut().as_mut() {
        //     uart.send(DEVICE_ID).unwrap();
        //     uart.send(0x01).unwrap();
        //     uart.send(0x02).unwrap();
        // }

        // Send test data
        uart.send(b'H').unwrap();
        uart.send(b'e').unwrap();
        uart.send(b'l').unwrap();
        uart.send(b'l').unwrap();
        uart.send(b'o').unwrap();
        uart.send(b'\r').unwrap();
        uart.send(b'\n').unwrap();

        // // set the led high
        // led.set_high();

        // Wait for about 1 second
        // for _ in 0..1000000 {
        //     avr_device::asm::nop();
        // }


        // // Set pin high
        // led.set_high();

        // Wait for 1 second (64 timer overflows)
        for _ in 0..64 {
            while timer.tcnt0.read().bits() < 124 {
                avr_device::asm::sleep();
            }
            timer.tcnt0.write(|w| w.bits(0)); // Reset the counter
        }

        // // 8000000 / 1024 = 7812.5 / 125 = 62.5 per second

        // // Set pin low
        // led.set_low();

        // // Wait for another 1 second
        // for _ in 0..64 {
        //     while timer.tcnt0.read().bits() < 124 {
        //         avr_device::asm::sleep();
        //     }
        //     timer.tcnt0.write(|w| w.bits(0)); // Reset the counter
        // }

        // avr_device::asm::sleep();
    }

    // // Set up PWM for servos and light
    // // let mut pwm = Pwm::new(dp.TC0, dp.TC1);
    // let mut base_servo = pins.pb0.into_output().into_pwm(&timer0);
    // let mut tilt_servo = pins.pb1.into_output().into_pwm(&timer0);
    // let _light = pins.pb2.into_output();

    // unsafe { avr_device::interrupt::enable() };

    // loop {
    //     free(|cs| {
    //         let mut buffer = BUFFER.borrow(cs).borrow_mut();
    //         if buffer.len() == 3 {
    //             let id_and_light = buffer.pop().unwrap();
    //             let rotation = buffer.pop().unwrap();
    //             let tilt = buffer.pop().unwrap();

    //             let id = (id_and_light & 0xF0) >> 4;
    //             let light_state = id_and_light & 0x0F;

    //             // Check if this message is for this device
    //             if id == DEVICE_ID {
    //                 // Process the message
    //                 // base_servo.set_duty(rotation);
    //                 // tilt_servo.set_duty(tilt);
    //                 // Handle light state
    //                 if light_state == 0x01 {
    //                     // Turn light on
    //                 } else {
    //                     // Turn light off
    //                 }
    //             }

    //             // Echo back the received data

    //             if let Some(uart) = UART.borrow(cs).borrow_mut().as_mut() {
    //                 uart.send(rotation).unwrap();
    //                 uart.send(tilt).unwrap();
    //                 uart.send(light_state).unwrap();
    //             }
    //         }
    //     });

    //     // Sleep to save power
    //     avr_device::asm::sleep();
    // }
}

// #[avr_device::interrupt(attiny85)]
// fn USI_START() {
//     free(|cs| {
//         if let Some(uart) = UART.borrow(cs).borrow_mut().as_mut() {
//             if let Ok(byte) = uart.receive() {
//                 BUFFER.borrow(cs).borrow_mut().push(byte);
//             }
//         }
//     });
// }

// - When recieving UART, need to throw away data if didn't get 4 in a certain
// amount of data
// - No ID needed, 3 bytes total
// - First test is get data, echo response
// - Make sure can flash to chip
// - Verify PWM math, test servo movement
// - Maybe calibrate oscillator
// - Wait isn't working
