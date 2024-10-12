#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use attiny_hal::port::mode::Output;
use attiny_hal::port::{Dynamic, Pin, PB0, PB1, PB2};
use avr_device::interrupt::{free, Mutex};
use core::cell::{Cell, RefCell};
use panic_halt as _;

mod buffer;
mod uart;

use buffer::Buffer;
use uart::SoftwareUart;

// Constants
const MAX_SERVOS: usize = 2;
const TRIM_DURATION: u8 = 4;
const SERVO_MIN: u16 = 1000; // Minimum pulse width in microseconds
const SERVO_MAX: u16 = 2000; // Maximum pulse width in microseconds

// Servo Sequencer State
#[derive(Clone, Copy, PartialEq)]
enum SequencerState {
    WaitingToSetPinHigh,
    WaitingFor512Mark,
    WaitingToSetPinLow,
    WaitingFor2048Mark,
}

// Servo Entry
struct ServoEntry {
    pulse_length_in_ticks: u8,
    pin: u8,
    enabled: bool,
    slot_occupied: bool,
}

// Static variables
static UART: Mutex<RefCell<Option<SoftwareUart>>> = Mutex::new(RefCell::new(None));
static BUFFER: Mutex<RefCell<Buffer>> = Mutex::new(RefCell::new(Buffer::new()));
static STATE: Mutex<Cell<SequencerState>> =
    Mutex::new(Cell::new(SequencerState::WaitingToSetPinHigh));
static SERVO_INDEX: Mutex<Cell<u8>> = Mutex::new(Cell::new(0));
static SERVO_REGISTRY: Mutex<RefCell<[ServoEntry; MAX_SERVOS]>> = Mutex::new(RefCell::new([
    ServoEntry {
        pulse_length_in_ticks: 255,
        pin: 0,
        enabled: true,
        slot_occupied: true,
    },
    ServoEntry {
        pulse_length_in_ticks: 255,
        pin: 1,
        enabled: true,
        slot_occupied: true,
    },
]));
static SERVO_PINS: Mutex<RefCell<[Option<Pin<Output, Dynamic>>; MAX_SERVOS]>> =
    Mutex::new(RefCell::new([None, None]));
static LIGHT: Mutex<RefCell<Option<Pin<Output, PB2>>>> = Mutex::new(RefCell::new(None));

#[derive(Clone, Copy)]
enum Servo {
    Base,
    Tilt,
}

#[avr_device::entry]
fn main() -> ! {
    let dp = attiny_hal::Peripherals::take().unwrap();
    let pins = attiny_hal::pins!(dp);

    set_servo_position(Servo::Base, 90);
    set_servo_position(Servo::Tilt, 90);

    // // Adjust OSCCAL
    // let current_value = dp.CPU.osccal.read().bits();
    // let new_value = (current_value as i16 + OSCCAL_ADJUSTMENT).clamp(0, 255) as u8;
    // dp.CPU.osccal.write(|w| w.bits(new_value));

    // Set up UART
    let mut uart = SoftwareUart::new(
        dp.TC1,
        pins.pb4.into_floating_input(),
        pins.pb3.into_output_high(),
    );
    free(|cs| UART.borrow(cs).replace(Some(uart)));

    // // Set up servo pins
    let base_servo = pins.pb0.into_output().downgrade();
    let tilt_servo = pins.pb1.into_output().downgrade();
    free(|cs| {
        SERVO_PINS.borrow(cs).borrow_mut()[0] = Some(base_servo);
        SERVO_PINS.borrow(cs).borrow_mut()[1] = Some(tilt_servo);
    });

    // Set up light pin
    let light = pins.pb2.into_output();
    free(|cs| LIGHT.borrow(cs).replace(Some(light)));

    // Enable Pin Change Interrupt for PB3
    dp.EXINT.gimsk.modify(|_, w| w.pcie().set_bit());
    dp.EXINT.pcmsk.modify(|_, w| w.pcint3().set_bit());

    // Set up Timer0 for PWM
    // dp.TC0.tccr0a.write(|w| w.wgm0().ctc());
    dp.TC0.tccr0b.write(|w| w.cs0().prescale_64());
    dp.TC0.ocr0a.write(|w| w.bits(255));
    dp.TC0.timsk.write(|w| w.ocie0a().set_bit());



    // Enable global interrupts
    unsafe { avr_device::interrupt::enable() };

    // Main loop
    loop {
        // Your main loop logic here
        avr_device::asm::sleep();
    }
}

#[avr_device::interrupt(attiny85)]
fn PCINT0() {
    free(|cs| {
        if let Some(uart) = UART.borrow(cs).borrow_mut().as_mut() {
            if let Ok(byte) = uart.receive() {
                BUFFER.borrow(cs).borrow_mut().push(byte);
            }
            // If there was an error, it likely came from an interrupt
            // being called a second time, so we can ignore it
        }
    });
}

#[avr_device::interrupt(attiny85)]
fn TIMER0_COMPA() {
    free(|cs| {
        let mut state = STATE.borrow(cs).get();
        let mut servo_index = SERVO_INDEX.borrow(cs).get();
        let mut servo_registry = SERVO_REGISTRY.borrow(cs).borrow_mut();
        let mut servo_pins = SERVO_PINS.borrow(cs).borrow_mut();

        match state {
            SequencerState::WaitingToSetPinHigh => {
                servo_index = (servo_index + 1) % MAX_SERVOS as u8;
                if servo_registry[servo_index as usize].enabled {
                    if let Some(servo_pin) = &mut servo_pins[servo_index as usize] {
                        servo_pin.set_high();
                    }
                }
                unsafe { (*attiny_hal::pac::TC0::ptr()).tcnt0.write(|w| w.bits(0)) };
                unsafe {
                    (*attiny_hal::pac::TC0::ptr())
                        .ocr0a
                        .write(|w| w.bits(64u8.saturating_sub(TRIM_DURATION)))
                };
                state = SequencerState::WaitingFor512Mark;
            }
            SequencerState::WaitingFor512Mark => {
                unsafe {
                    (*attiny_hal::pac::TC0::ptr()).ocr0a.write(|w| {
                        w.bits(servo_registry[servo_index as usize].pulse_length_in_ticks)
                    })
                };
                state = SequencerState::WaitingToSetPinLow;
                unsafe { (*attiny_hal::pac::TC0::ptr()).tcnt0.write(|w| w.bits(0)) };
                if servo_registry[servo_index as usize].pulse_length_in_ticks == 0 {
                    unsafe { (*attiny_hal::pac::TC0::ptr()).tcnt0.write(|w| w.bits(0xFF)) };
                } else {
                    unsafe {
                        (*attiny_hal::pac::TC0::ptr())
                            .tifr
                            .write(|w| w.ocf0a().set_bit())
                    };
                }
            }
            SequencerState::WaitingToSetPinLow => {
                if servo_registry[servo_index as usize].enabled {
                    if let Some(servo_pin) = &mut servo_pins[servo_index as usize] {
                        servo_pin.set_low();
                    }
                }
                let total_ticks = 64u16.saturating_add(
                    servo_registry[servo_index as usize].pulse_length_in_ticks as u16,
                );
                if total_ticks > 255 {
                    state = SequencerState::WaitingToSetPinHigh;
                    unsafe {
                        (*attiny_hal::pac::TC0::ptr())
                            .ocr0a
                            .write(|w| w.bits((512u16.saturating_sub(total_ticks)) as u8))
                    };
                } else {
                    state = SequencerState::WaitingFor2048Mark;
                    unsafe {
                        (*attiny_hal::pac::TC0::ptr())
                            .ocr0a
                            .write(|w| w.bits((255u16.saturating_sub(total_ticks)) as u8))
                    };
                }
                unsafe { (*attiny_hal::pac::TC0::ptr()).tcnt0.write(|w| w.bits(0)) };
            }
            SequencerState::WaitingFor2048Mark => {
                state = SequencerState::WaitingToSetPinHigh;
                unsafe { (*attiny_hal::pac::TC0::ptr()).tcnt0.write(|w| w.bits(0)) };
                unsafe { (*attiny_hal::pac::TC0::ptr()).ocr0a.write(|w| w.bits(255)) };
            }
        }

        STATE.borrow(cs).set(state);
        SERVO_INDEX.borrow(cs).set(servo_index);
    });
}

fn set_servo_position(servo: Servo, degrees: u8) {
    let pulse = map_degrees_to_pulse(degrees);
    free(|cs| {
        let mut servo_registry = SERVO_REGISTRY.borrow(cs).borrow_mut();
        let index = match servo {
            Servo::Base => 0,
            Servo::Tilt => 1,
        };
        servo_registry[index].pulse_length_in_ticks = (pulse / 8) as u8;
    });
}

fn map_degrees_to_pulse(degrees: u8) -> u16 {
    map(degrees as u16, 0, 180, SERVO_MIN, SERVO_MAX)
}

fn map(x: u16, in_min: u16, in_max: u16, out_min: u16, out_max: u16) -> u16 {
    let numerator = (x as u32)
        .saturating_sub(in_min as u32)
        .saturating_mul(out_max as u32 - out_min as u32);
    let denominator = (in_max as u32).saturating_sub(in_min as u32);
    (numerator / denominator).saturating_add(out_min as u32) as u16
}
