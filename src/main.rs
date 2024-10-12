#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use attiny_hal::port::mode::Output;
use attiny_hal::port::{Dynamic, Pin, PB0, PB1, PB2};
use avr_device::interrupt::{free, Mutex};
use core::cell::{Cell, RefCell};
use core::marker::PhantomData;
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

// New type definitions
struct Motor0;
struct Motor1;
struct Idle;

trait Motor {
    const INDEX: usize;
    type Next: Motor;
}

impl Motor for Motor0 {
    const INDEX: usize = 0;
    type Next = Motor1;
}

impl Motor for Motor1 {
    const INDEX: usize = 1;
    type Next = Idle;
}

impl Motor for Idle {
    const INDEX: usize = 0; // This value doesn't matter for Idle
    type Next = Motor0;
}

struct WaitingToSetPinHigh<M: Motor>(PhantomData<M>);
struct WaitingFor512Mark<M: Motor>(PhantomData<M>);
struct WaitingToSetPinLow<M: Motor>(PhantomData<M>);
struct WaitingFor2048Mark<M: Motor>(PhantomData<M>);

struct ServoController<State = WaitingToSetPinHigh<Motor0>> {
    servos: [ServoEntry; MAX_SERVOS],
    state: State,
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
static SERVO_CONTROLLER: Mutex<RefCell<Option<ServoController>>> = Mutex::new(RefCell::new(None));
static SERVO_PINS: Mutex<RefCell<[Option<Pin<Output, Dynamic>>; MAX_SERVOS]>> =
    Mutex::new(RefCell::new([None, None]));
static LIGHT: Mutex<RefCell<Option<Pin<Output, PB2>>>> = Mutex::new(RefCell::new(None));

#[derive(Clone, Copy)]
enum Servo {
    Base,
    Tilt,
}

impl<State> ServoController<State> {
    fn transition<NewState>(self, new_state: NewState) -> ServoController<NewState> {
        ServoController {
            servos: self.servos,
            state: new_state,
        }
    }
}

impl<M: Motor> ServoController<WaitingToSetPinHigh<M>> {
    fn update_set_pin_high(
        mut self,
        servo_pins: &mut [Option<Pin<Output, Dynamic>>; MAX_SERVOS],
    ) -> ServoController<WaitingFor512Mark<M>> {
        if self.servos[M::INDEX].enabled {
            if let Some(servo_pin) = &mut servo_pins[M::INDEX] {
                servo_pin.set_high();
            }
        }
        unsafe {
            (*attiny_hal::pac::TC0::ptr()).tcnt0.write(|w| w.bits(0));
            (*attiny_hal::pac::TC0::ptr()).ocr0a.write(|w| w.bits(64u8.saturating_sub(TRIM_DURATION)));
        }
        self.transition(WaitingFor512Mark(PhantomData))
    }
}

impl<M: Motor> ServoController<WaitingFor512Mark<M>> {
    fn update_wait_512_mark(self) -> ServoController<WaitingToSetPinLow<M>> {
        unsafe {
            (*attiny_hal::pac::TC0::ptr()).ocr0a.write(|w| w.bits(self.servos[M::INDEX].pulse_length_in_ticks));
            (*attiny_hal::pac::TC0::ptr()).tcnt0.write(|w| w.bits(0));
            if self.servos[M::INDEX].pulse_length_in_ticks == 0 {
                (*attiny_hal::pac::TC0::ptr()).tcnt0.write(|w| w.bits(0xFF));
            } else {
                (*attiny_hal::pac::TC0::ptr()).tifr.write(|w| w.ocf0a().set_bit());
            }
        }
        self.transition(WaitingToSetPinLow(PhantomData))
    }
}

impl<M: Motor> ServoController<WaitingToSetPinLow<M>> {
    fn update_set_pin_low(
        self,
        servo_pins: &mut [Option<Pin<Output, Dynamic>>; MAX_SERVOS],
    ) -> ServoController<WaitingFor2048Mark<M>> {
        if self.servos[M::INDEX].enabled {
            if let Some(servo_pin) = &mut servo_pins[M::INDEX] {
                servo_pin.set_low();
            }
        }
        let total_ticks = 64u16.saturating_add(self.servos[M::INDEX].pulse_length_in_ticks as u16);
        unsafe {
            if total_ticks > 255 {
                (*attiny_hal::pac::TC0::ptr()).ocr0a.write(|w| w.bits((512u16.saturating_sub(total_ticks)) as u8));
            } else {
                (*attiny_hal::pac::TC0::ptr()).ocr0a.write(|w| w.bits((255u16.saturating_sub(total_ticks)) as u8));
            }
            (*attiny_hal::pac::TC0::ptr()).tcnt0.write(|w| w.bits(0));
        }
        self.transition(WaitingFor2048Mark(PhantomData))
    }
}

impl<M: Motor> ServoController<WaitingFor2048Mark<M>> {
    fn update_wait_2048_mark(self) -> ServoController<WaitingToSetPinHigh<M::Next>> {
        unsafe {
            (*attiny_hal::pac::TC0::ptr()).tcnt0.write(|w| w.bits(0));
            (*attiny_hal::pac::TC0::ptr()).ocr0a.write(|w| w.bits(255));
        }
        self.transition(WaitingToSetPinHigh(PhantomData))
    }
}

impl ServoController<WaitingToSetPinHigh<Idle>> {
    fn new() -> Self {
        Self {
            servos: [
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
            ],
            state: WaitingToSetPinHigh(PhantomData),
        }
    }

    fn update_idle(self) -> ServoController<WaitingToSetPinHigh<Motor0>> {
        // Here you can set a longer timer if needed
        unsafe {
            (*attiny_hal::pac::TC0::ptr()).tcnt0.write(|w| w.bits(0));
            (*attiny_hal::pac::TC0::ptr()).ocr0a.write(|w| w.bits(255)); // Adjust this value for a longer idle period
        }
        self.transition(WaitingToSetPinHigh(PhantomData))
    }

    fn set_servo_position(&mut self, servo: Servo, degrees: u8) {
        let pulse = map_degrees_to_pulse(degrees);
        let index = match servo {
            Servo::Base => 0,
            Servo::Tilt => 1,
        };
        self.servos[index].pulse_length_in_ticks = (pulse / 8) as u8;
    }
}

#[avr_device::entry]
fn main() -> ! {
    let dp = attiny_hal::Peripherals::take().unwrap();
    let pins = attiny_hal::pins!(dp);

    // Initialize ServoController
    free(|cs| {
        SERVO_CONTROLLER
            .borrow(cs)
            .replace(Some(ServoController::new()));
    });

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
        let mut servo_controller = SERVO_CONTROLLER.borrow(cs).borrow_mut();
        let mut servo_pins = SERVO_PINS.borrow(cs).borrow_mut();

        if let Some(controller) = servo_controller.take() {
            let new_controller = match controller.state {
                WaitingToSetPinHigh(_) => ServoController::update_set_pin_high(controller, &mut servo_pins),
                WaitingFor512Mark(_) => ServoController::update_wait_512_mark(controller),
                WaitingToSetPinLow(_) => ServoController::update_set_pin_low(controller, &mut servo_pins),
                WaitingFor2048Mark(_) => ServoController::update_wait_2048_mark(controller),
            };
            *servo_controller = Some(new_controller);
        }
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