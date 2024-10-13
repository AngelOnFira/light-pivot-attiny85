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
const SERVO_COUNT: usize = 2;
const MAX_SERVO_COUNT: usize = 5;
const TRIM_DURATION: u8 = 4;
const SERVO_MIN: u16 = 1000; // Minimum pulse width in microseconds
const SERVO_MAX: u16 = 2000; // Maximum pulse width in microseconds
const CYCLE_TICK_COUNT: u16 = 2500; // 20ms / 50Hz
const OSCCAL_ADJUSTMENT: i16 = -2;

struct ServoSequencer {
    state: SequencerState,
    servo_index: usize,
    servo_registry: [ServoEntry; SERVO_COUNT],
    servo_pins: [Option<Pin<Output, Dynamic>>; SERVO_COUNT],
    remaining_cycle_ticks: u16,
}

#[derive(Clone, Copy, PartialEq)]
enum SequencerState {
    SetPinHigh,
    WaitForPulseEnd,
    SetPinLow,
    WaitForNextPulse,
    WaitForCycleEnd,
}

// Servo Entry
struct ServoEntry {
    pulse_length_in_ticks: u8,
    enabled: bool,
}

// Static variables
static UART: Mutex<RefCell<Option<SoftwareUart>>> = Mutex::new(RefCell::new(None));
static BUFFER: Mutex<RefCell<Buffer>> = Mutex::new(RefCell::new(Buffer::new()));
static STATE: Mutex<Cell<SequencerState>> = Mutex::new(Cell::new(SequencerState::SetPinHigh));
static SERVO_INDEX: Mutex<Cell<u8>> = Mutex::new(Cell::new(0));
static SERVO_REGISTRY: Mutex<RefCell<[ServoEntry; SERVO_COUNT]>> = Mutex::new(RefCell::new([
    ServoEntry {
        pulse_length_in_ticks: 255,
        enabled: true,
    },
    ServoEntry {
        pulse_length_in_ticks: 255,
        enabled: true,
    },
]));
// static SERVO_PINS: Mutex<RefCell<[Option<Pin<Output, Dynamic>>; MAX_SERVOS]>> =
//     Mutex::new(RefCell::new([None, None]));
static LIGHT: Mutex<RefCell<Option<Pin<Output, PB2>>>> = Mutex::new(RefCell::new(None));
static SEQUENCER: Mutex<RefCell<Option<ServoSequencer>>> = Mutex::new(RefCell::new(None));

#[derive(Clone, Copy)]
enum Servo {
    Base,
    Tilt,
}

impl ServoSequencer {
    fn new(
        servo_registry: [ServoEntry; SERVO_COUNT],
        servo_pins: [Option<Pin<Output, Dynamic>>; SERVO_COUNT],
    ) -> Self {
        Self {
            state: SequencerState::SetPinHigh,
            servo_index: 0,
            servo_registry,
            servo_pins,
            remaining_cycle_ticks: CYCLE_TICK_COUNT,
        }
    }

    fn update(&mut self) {
        match self.state {
            SequencerState::SetPinHigh => self.set_pin_high(),
            SequencerState::WaitForPulseEnd => self.wait_for_pulse_min(),
            SequencerState::SetPinLow => self.set_pin_low(),
            SequencerState::WaitForNextPulse => self.wait_for_next_pulse(),
            SequencerState::WaitForCycleEnd => self.wait_for_cycle_end(),
        }
    }

    fn set_pin_high(&mut self) {
        if self.servo_registry[self.servo_index].enabled {
            if let Some(servo_pin) = &mut self.servo_pins[self.servo_index] {
                servo_pin.set_high();
            }
        }
        self.set_timer(64u8.saturating_sub(TRIM_DURATION));
        self.state = SequencerState::WaitForPulseEnd;
    }

    fn wait_for_pulse_min(&mut self) {
        let pulse_length = self.servo_registry[self.servo_index].pulse_length_in_ticks;
        self.set_timer(pulse_length);
        self.state = SequencerState::SetPinLow;
        if pulse_length == 0 {
            self.set_timer_to_max();
        } else {
            self.clear_timer_interrupt();
        }
    }

    fn set_pin_low(&mut self) {
        if self.servo_registry[self.servo_index].enabled {
            if let Some(servo_pin) = &mut self.servo_pins[self.servo_index] {
                servo_pin.set_low();
            }
        }

        let total_ticks = 64u16
            .saturating_add(self.servo_registry[self.servo_index].pulse_length_in_ticks as u16);

        // Update servo_index after finishing the pulse
        self.servo_index = (self.servo_index + 1) % SERVO_COUNT;
        self.remaining_cycle_ticks -= 500;

        // If we have no more motors after this one, wait for the cycle to end
        if self.servo_index == 0 {
            self.state = SequencerState::WaitForCycleEnd;
        } else if total_ticks > 255 {
            self.set_timer((512u16.saturating_sub(total_ticks)) as u8);
            self.state = SequencerState::SetPinHigh;
        } else {
            self.set_timer((255u16.saturating_sub(total_ticks)) as u8);
            self.state = SequencerState::WaitForNextPulse;
        }
    }

    fn wait_for_next_pulse(&mut self) {
        self.set_timer(255);
        self.state = SequencerState::SetPinHigh;
    }

    fn wait_for_cycle_end(&mut self) {
        // Set the timer for the max tick value, or however many ticks are left in the cycle
        if self.remaining_cycle_ticks > 255 {
            self.set_timer(255);
            self.remaining_cycle_ticks -= 255;
        } else {
            self.set_timer(self.remaining_cycle_ticks as u8);
            self.remaining_cycle_ticks = 0;
            self.state = SequencerState::SetPinHigh;
            self.remaining_cycle_ticks = CYCLE_TICK_COUNT;
        }
    }

    fn set_timer(&self, value: u8) {
        unsafe {
            (*attiny_hal::pac::TC0::ptr()).tcnt0.write(|w| w.bits(0));
            (*attiny_hal::pac::TC0::ptr())
                .ocr0a
                .write(|w| w.bits(value));
        }
    }

    fn set_timer_to_max(&self) {
        unsafe {
            (*attiny_hal::pac::TC0::ptr()).tcnt0.write(|w| w.bits(0xFF));
        }
    }

    fn clear_timer_interrupt(&self) {
        unsafe {
            (*attiny_hal::pac::TC0::ptr())
                .tifr
                .write(|w| w.ocf0a().set_bit());
        }
    }
}

#[avr_device::entry]
fn main() -> ! {
    let dp = attiny_hal::Peripherals::take().unwrap();
    let pins = attiny_hal::pins!(dp);

    set_servo_position(Servo::Base, 90);
    set_servo_position(Servo::Tilt, 90);

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
    free(|cs| UART.borrow(cs).replace(Some(uart)));

    // Set up servo pins
    let base_servo = pins.pb0.into_output().downgrade();
    let tilt_servo = pins.pb1.into_output().downgrade();
    // free(|cs| {
    //     SERVO_PINS.borrow(cs).borrow_mut()[0] = Some(base_servo);
    //     SERVO_PINS.borrow(cs).borrow_mut()[1] = Some(tilt_servo);
    // });

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

    // Initialize ServoSequencer
    let sequencer = ServoSequencer::new(
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
    free(|cs| SEQUENCER.borrow(cs).replace(Some(sequencer)));

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
        if let Some(sequencer) = SEQUENCER.borrow(cs).borrow_mut().as_mut() {
            sequencer.update();
        }
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
