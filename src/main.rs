#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use attiny_hal::port::mode::Output;
use attiny_hal::port::{Pin, PB2};
use avr_device::interrupt::{free, Mutex};
use core::cell::RefCell;
use panic_halt as _;
use servo::{Servo, ServoEntry, ServoSequencer};

mod buffer;
mod servo;
mod uart;

use buffer::Buffer;
use uart::SoftwareUart;

// Constants
pub const SERVO_COUNT: usize = 2;
pub const MAX_SERVO_COUNT: usize = 5;
pub const TRIM_DURATION: u8 = 4;
pub const SERVO_MIN: u16 = 1000; // Minimum pulse width in microseconds
pub const SERVO_MAX: u16 = 2000; // Maximum pulse width in microseconds
pub const CYCLE_TICK_COUNT: u16 = 2500; // 20ms / 50Hz
pub const OSCCAL_ADJUSTMENT: i16 = -2;

// Static variables
static UART: Mutex<RefCell<Option<SoftwareUart>>> = Mutex::new(RefCell::new(None));
static BUFFER: Mutex<RefCell<Buffer>> = Mutex::new(RefCell::new(Buffer::new()));
static LIGHT: Mutex<RefCell<Option<Pin<Output, PB2>>>> = Mutex::new(RefCell::new(None));
static SEQUENCER: Mutex<RefCell<Option<ServoSequencer>>> = Mutex::new(RefCell::new(None));

#[avr_device::entry]
fn main() -> ! {
    let dp = attiny_hal::Peripherals::take().unwrap();
    let pins = attiny_hal::pins!(dp);

    // Adjust OSCCAL
    let current_value = dp.CPU.osccal.read().bits();
    let new_value = (current_value as i16 + OSCCAL_ADJUSTMENT).clamp(0, 255) as u8;
    dp.CPU.osccal.write(|w| w.bits(new_value));

    // Set up UART
    let mut uart = SoftwareUart::new(
        dp.TC1,
        pins.pb4.into_floating_input(),
        pins.pb3.into_output_high(),
    );
    // Send debug message
    uart.send_string("Hello, world!\r\n").unwrap();
    free(|cs| UART.borrow(cs).replace(Some(uart)));

    // Set up servo pins
    let base_servo = pins.pb0.into_output().downgrade();
    let tilt_servo = pins.pb1.into_output().downgrade();

    // Set up light pin
    let light = pins.pb2.into_output_high();
    free(|cs| LIGHT.borrow(cs).replace(Some(light)));

    // Enable Pin Change Interrupt for PB4
    dp.EXINT.gimsk.modify(|_, w| w.pcie().set_bit());
    dp.EXINT.pcmsk.modify(|_, w| w.pcint4().set_bit());

    // Set up Timer0 for PWM
    // dp.TC0.tccr0a.write(|w| w.wgm0().ctc());
    dp.TC0.tccr0b.write(|w| w.cs0().prescale_64());
    dp.TC0.ocr0a.write(|w| w.bits(255));
    dp.TC0.timsk.write(|w| w.ocie0a().set_bit());

    // Initialize ServoSequencer
    let mut sequencer = ServoSequencer::new(
        [
            ServoEntry {
                pulse_length_in_ticks: 255,
                enabled: true,
            },
            ServoEntry {
                pulse_length_in_ticks: 255,
                enabled: true,
            },
        ],
        [Some(base_servo), Some(tilt_servo)],
    );
    sequencer.set_servo_position(Servo::Base, 90);
    sequencer.set_servo_position(Servo::Tilt, 90);
    free(|cs| SEQUENCER.borrow(cs).replace(Some(sequencer)));

    // Enable global interrupts
    unsafe { avr_device::interrupt::enable() };

    // Main loop
    loop {
        free(|cs| {
            if let Some(sequencer) = SEQUENCER.borrow(cs).borrow_mut().as_mut() {
                let mut buffer = BUFFER.borrow(cs).borrow_mut();

                if buffer.len() >= 3 {
                    // if let Some(light) = LIGHT.borrow(cs).borrow_mut().as_mut() {
                    //     light.toggle();
                    // }
                    let result: Result<(), ()> = (|| {
                        let id_and_light = buffer.pop().ok_or(())?;
                        let rotation = buffer.pop().ok_or(())?.clamp(0, 180);
                        let tilt = buffer.pop().ok_or(())?.clamp(0, 180);
                        let _id = (id_and_light & 0xF0) >> 4;

                        // Light will turn on if any of the lower 4 bits are 1
                        // 0000 1111 & 0000 0001 = 0000 0001
                        let light_state = id_and_light & 0x0F != 0;

                        // Set servo positions
                        sequencer.set_servo_position(Servo::Base, rotation);
                        sequencer.set_servo_position(Servo::Tilt, tilt);

                        // Handle light state
                        // if let Some(light) = LIGHT.borrow(cs).borrow_mut().as_mut() {
                        //     if light_state {
                        //         light.set_high();
                        //     } else {
                        //         light.set_low();
                        //     }
                        // }
                        Ok(())
                    })();
                    if let Err(_error) = result {
                        buffer.clear();
                    }
                }
            }
        });

        // Sleep until next interrupt
        avr_device::asm::sleep();
    }
}

#[avr_device::interrupt(attiny85)]
fn PCINT0() {
    free(|cs| {
        if let Some(uart) = UART.borrow(cs).borrow_mut().as_mut() {
            if let Ok(byte) = uart.receive() {
                BUFFER.borrow(cs).borrow_mut().push(byte);

                // Echo the byte back
                uart.send(byte).unwrap();
            }
            // If there was an error, it likely came from an interrupt
            // being called a second time, so we can ignore it
        }

        // Toggle light
        if let Some(light) = LIGHT.borrow(cs).borrow_mut().as_mut() {
            light.toggle();
        }
    });
}

#[avr_device::interrupt(attiny85)]
fn TIMER0_COMPA() {
    // free(|cs| {
    //     if let Some(sequencer) = SEQUENCER.borrow(cs).borrow_mut().as_mut() {
    //         sequencer.update();
    //     }
    // });
}
