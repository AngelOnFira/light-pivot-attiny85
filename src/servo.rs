use attiny_hal::port::mode::Output;
use attiny_hal::port::{Dynamic, Pin, PB0, PB1, PB2};
use avr_device::interrupt::{free, Mutex};
use core::cell::{Cell, RefCell};
use panic_halt as _;

use crate::{CYCLE_TICK_COUNT, SERVO_COUNT, SERVO_MAX, SERVO_MIN, TRIM_DURATION};

pub struct ServoSequencer {
    state: SequencerState,
    servo_index: usize,
    servo_registry: [ServoEntry; SERVO_COUNT],
    servo_pins: [Option<Pin<Output, Dynamic>>; SERVO_COUNT],
    remaining_cycle_ticks: u16,
}

#[derive(Clone, Copy, PartialEq)]
pub enum SequencerState {
    SetPinHigh,
    WaitForPulseEnd,
    SetPinLow,
    WaitForNextPulse,
    WaitForCycleEnd,
}

// Servo Entry
pub struct ServoEntry {
    pub pulse_length_in_ticks: u8,
    pub enabled: bool,
}

#[derive(Clone, Copy)]
pub enum Servo {
    Base,
    Tilt,
}

impl ServoSequencer {
    pub fn new(
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

    pub fn update(&mut self) {
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

    pub fn set_servo_position(&mut self, servo: Servo, degrees: u8) {
        let pulse = map_degrees_to_pulse(degrees);
        let index = match servo {
            Servo::Base => 0,
            Servo::Tilt => 1,
        };
        self.servo_registry[index].pulse_length_in_ticks = (pulse / 8) as u8;
    }
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
