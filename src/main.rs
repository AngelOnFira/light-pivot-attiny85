#![no_main]
#![no_std]
#![feature(abi_avr_interrupt)]

use attiny_hal::port::mode::Output;
use attiny_hal::port::{Pin, PB0, PB1, PB2};
use avr_device::interrupt::{free, Mutex};
use core::cell::{Cell, RefCell};
use panic_halt as _;

mod buffer;
mod uart;

use buffer::Buffer;
use uart::SoftwareUart;

// Define constants
const PWM_CYCLE: u16 = 2000; // 20ms cycle for 50Hz (2000 * 10μs)
const SERVO_MIN: u16 = 100; // 1ms (100 * 10μs)
const SERVO_MAX: u16 = 200; // 2ms (200 * 10μs)
const OSCCAL_ADJUSTMENT: i16 = -2;

// Define Servo enum
#[derive(Clone, Copy)]
enum Servo {
    Base,
    Tilt,
}

// Static variables
static UART: Mutex<RefCell<Option<SoftwareUart>>> = Mutex::new(RefCell::new(None));
static BUFFER: Mutex<RefCell<Buffer>> = Mutex::new(RefCell::new(Buffer::new()));
static COUNTER: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));
static SERVO_PULSES: Mutex<RefCell<[u16; 2]>> = Mutex::new(RefCell::new([SERVO_MIN; 2]));
static BASE_SERVO: Mutex<RefCell<Option<Pin<Output, PB0>>>> = Mutex::new(RefCell::new(None));
static TILT_SERVO: Mutex<RefCell<Option<Pin<Output, PB1>>>> = Mutex::new(RefCell::new(None));
static LIGHT: Mutex<RefCell<Option<Pin<Output, PB2>>>> = Mutex::new(RefCell::new(None));

// Add these new static variables
static DEBUG_BASE: Mutex<Cell<u8>> = Mutex::new(Cell::new(0));
static DEBUG_TILT: Mutex<Cell<u8>> = Mutex::new(Cell::new(0));
static DEBUG_LIGHT: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));

// Add this new static variable
static ALIVE_COUNTER: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));

// Add this new static variable
static LIGHT_STATE: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));

// Add these new static variables
static SERVO_STATE: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));

// Add this new static variable
static LIFETIME_COUNTER: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));

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

    // Write hello world to the UART
    uart.send_string("Hello, world!\r\n").unwrap();

    free(|cs| UART.borrow(cs).replace(Some(uart)));

    // Set up servo pins
    let base_servo = pins.pb0.into_output();
    let tilt_servo = pins.pb1.into_output();
    free(|cs| {
        BASE_SERVO.borrow(cs).replace(Some(base_servo));
        TILT_SERVO.borrow(cs).replace(Some(tilt_servo));
    });

    // Set up light pin
    let light = pins.pb2.into_output();
    free(|cs| LIGHT.borrow(cs).replace(Some(light)));

    // Enable Pin Change Interrupt for PB3
    dp.EXINT.gimsk.modify(|_, w| w.pcie().set_bit());
    dp.EXINT.pcmsk.modify(|_, w| w.pcint3().set_bit());

    // Set up Timer0 for PWM
    dp.TC0.tccr0a.write(|w| w.wgm0().ctc());
    dp.TC0.tccr0b.write(|w| w.cs0().direct());
    dp.TC0.ocr0a.write(|w| w.bits(79));

    // Timer0 Setup Math:
    // Target frequency: 100 kHz (10μs period)
    // ATtiny85 clock: 8 MHz
    // Prescaler: 8
    // OCR0A value: 79
    //
    // Timer frequency = (Clock frequency) / (Prescaler * (1 + OCR0A))
    //                 = 8,000,000 / (8 * (1 + 79))
    //                 = 8,000,000 / (8 * 80)
    //                 = 1,000,000 / 80
    //                 = 12,500 Hz
    //
    // This gives us a timer interrupt every 10μs (1/100,000 seconds)
    //
    // For 50 Hz PWM (20ms period):
    // 20ms / 10μs = 2000 steps per PWM cycle
    //
    // Servo pulse width range: 1ms to 2ms
    // 1ms / 10μs = 100 steps
    // 2ms / 10μs = 200 steps
    //
    // Total steps for servo range: 200 - 100 = 100 steps
    //
    // Degree precision:
    // 180 degrees / 100 steps = 1.8 degrees per step
    //
    // This setup allows for a precision of 1.8 degrees in servo positioning.

    // Initialize servos to a known position
    set_servo_position(Servo::Base, 90);
    set_servo_position(Servo::Tilt, 90);

    // Add a delay before starting the main loop
    for _ in 0..1000 {
        avr_device::asm::nop();
    }

    // Enable Timer0 Compare Match A interrupt
    dp.TC0.timsk.write(|w| w.ocie0a().set_bit());

    // Enable global interrupts
    unsafe { avr_device::interrupt::enable() };

    loop {
        free(|cs| {
            let mut buffer = BUFFER.borrow(cs).borrow_mut();

            if buffer.len() >= 3 {
                let result: Result<(), ()> = (|| {
                    let id_and_light = buffer.pop().ok_or(())?;
                    let rotation = buffer.pop().ok_or(())?.clamp(0, 180);
                    let tilt = buffer.pop().ok_or(())?.clamp(0, 180);

                    let _id = (id_and_light & 0xF0) >> 4;
                    let light_state = id_and_light & 0x0F != 0;

                    // Comment out servo position setting
                    // set_servo_position(Servo::Base, rotation);
                    // set_servo_position(Servo::Tilt, tilt);

                    // Handle light state
                    if let Some(light) = LIGHT.borrow(cs).borrow_mut().as_mut() {
                        if light_state {
                            light.set_high();
                        } else {
                            light.set_low();
                        }
                    }

                    // Update debug variables
                    DEBUG_BASE.borrow(cs).set(rotation);
                    DEBUG_TILT.borrow(cs).set(tilt);
                    DEBUG_LIGHT.borrow(cs).set(light_state);

                    Ok(())
                })();

                if let Err(_error) = result {
                    buffer.clear();
                }

            }

            // Check if 1 second has passed (100,000 * 10μs = 1 second)
            if ALIVE_COUNTER.borrow(cs).get() >= 100_000 {
                // Reset the counter
                ALIVE_COUNTER.borrow(cs).set(0);

                // Increment the lifetime counter
                let lifetime = LIFETIME_COUNTER.borrow(cs).get();
                LIFETIME_COUNTER.borrow(cs).set(lifetime + 1);

                // Toggle the light state
                let new_light_state = !LIGHT_STATE.borrow(cs).get();
                LIGHT_STATE.borrow(cs).set(new_light_state);

                // Toggle the servo state
                let new_servo_state = !SERVO_STATE.borrow(cs).get();
                SERVO_STATE.borrow(cs).set(new_servo_state);

                // Update the light
                if let Some(light) = LIGHT.borrow(cs).borrow_mut().as_mut() {
                    if new_light_state {
                        light.set_high();
                    } else {
                        light.set_low();
                    }
                }

                // Update servo positions
                if new_servo_state {
                    set_servo_position(Servo::Base, 120);
                    set_servo_position(Servo::Tilt, 120);
                } else {
                    set_servo_position(Servo::Base, 20);
                    set_servo_position(Servo::Tilt, 20);
                }

                // Send "alive" message over UART
                if let Some(uart) = UART.borrow(cs).borrow_mut().as_mut() {
                    uart.send_string("alive (").unwrap();
                    // Send the lifetime counter value
                    uart.send(LIFETIME_COUNTER.borrow(cs).get() as u8).unwrap();
                    uart.send_string("s)\r\n").unwrap();

                    uart.send_string(if new_light_state {
                        "Light ON\r\n"
                    } else {
                        "Light OFF\r\n"
                    })
                    .unwrap();
                    uart.send_string(if new_servo_state {
                        "Servos at 120 degrees\r\n"
                    } else {
                        "Servos at 20 degrees\r\n"
                    })
                    .unwrap();

                    // Send debug values
                    uart.send_string("Base: ").unwrap();
                    uart.send(DEBUG_BASE.borrow(cs).get()).unwrap();
                    uart.send_string(" Tilt: ").unwrap();
                    uart.send(DEBUG_TILT.borrow(cs).get()).unwrap();
                    uart.send_string(" Light: ").unwrap();
                    uart.send_string(if DEBUG_LIGHT.borrow(cs).get() {
                        "ON"
                    } else {
                        "OFF"
                    })
                    .unwrap();
                    uart.send_string("\r\n").unwrap();
                }
            }
        });

        // Sleep until interrupt
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
        let counter = COUNTER.borrow(cs).get();
        let servo_pulses = SERVO_PULSES.borrow(cs).borrow();

        if counter == 0 {
            // Start of cycle, set both servos high
            if let Some(base_servo) = BASE_SERVO.borrow(cs).borrow_mut().as_mut() {
                base_servo.set_high();
            }
            if let Some(tilt_servo) = TILT_SERVO.borrow(cs).borrow_mut().as_mut() {
                tilt_servo.set_high();
            }
        } else {
            // Check and set servos low if their pulse duration has passed
            if counter >= servo_pulses[0] {
                if let Some(base_servo) = BASE_SERVO.borrow(cs).borrow_mut().as_mut() {
                    base_servo.set_low();
                }
            }
            if counter >= servo_pulses[1] {
                if let Some(tilt_servo) = TILT_SERVO.borrow(cs).borrow_mut().as_mut() {
                    tilt_servo.set_low();
                }
            }
        }

        // Increment or reset counter
        if counter >= PWM_CYCLE - 1 {
            COUNTER.borrow(cs).set(0);
        } else {
            COUNTER.borrow(cs).set(counter + 1);
        }

        // Increment the ALIVE_COUNTER
        let alive_counter = ALIVE_COUNTER.borrow(cs).get();
        ALIVE_COUNTER.borrow(cs).set(alive_counter + 1);
    });
}

fn set_servo_position(servo: Servo, degrees: u8) {
    let pulse = map_degrees_to_pulse(degrees);
    free(|cs| {
        let mut servo_pulses = SERVO_PULSES.borrow(cs).borrow_mut();
        match servo {
            Servo::Base => servo_pulses[0] = pulse,
            Servo::Tilt => servo_pulses[1] = pulse,
        }
    });
}

fn map_degrees_to_pulse(degrees: u8) -> u16 {
    // First, calculate the range of pulse widths
    let pulse_range = SERVO_MAX - SERVO_MIN;

    // Then, calculate the proportion of the full range
    let proportion = (degrees as u16 * pulse_range) / 180;

    // Finally, add this to the minimum pulse width
    SERVO_MIN + proportion
}
// Proof of correctness for map_degrees_to_pulse:
// 1. When degrees = 0:
//    pulse_range = SERVO_MAX - SERVO_MIN
//    proportion = (0 * pulse_range) / 180 = 0
//    result = SERVO_MIN + 0 = SERVO_MIN
//    This correctly maps 0 degrees to the minimum pulse width.
//
// 2. When degrees = 180:
//    pulse_range = SERVO_MAX - SERVO_MIN
//    proportion = (180 * pulse_range) / 180 = pulse_range
//    result = SERVO_MIN + pulse_range = SERVO_MAX
//    This correctly maps 180 degrees to the maximum pulse width.
//
// 3. For any value between 0 and 180:
//    The function performs linear interpolation between SERVO_MIN and SERVO_MAX.
//    This ensures a proportional mapping of degrees to pulse width.
//
// 4. The use of u16 for the calculation prevents integer overflow:
//    SERVO_MAX (200) - SERVO_MIN (100) = 100
//    100 * 180 = 18000, which is well within the range of u16
//
// 5. The division by 180 is performed last in the proportion calculation:
//    This approach minimizes rounding errors in the integer division.
//
// 6. Degree to pulse counter translation:
//    - 0 degrees maps to SERVO_MIN (100 timer ticks = 1ms pulse)
//    - 180 degrees maps to SERVO_MAX (200 timer ticks = 2ms pulse)
//    - Any degree in between is linearly interpolated:
//      For example, 90 degrees would map to:
//      100 + (200 - 100) * 90 / 180 = 150 timer ticks = 1.5ms pulse
//
// Therefore, map_degrees_to_pulse correctly maps the full range of servo motion
// (0 to 180 degrees) to the appropriate pulse widths (SERVO_MIN to SERVO_MAX),
// with each degree corresponding to a specific number of timer ticks.