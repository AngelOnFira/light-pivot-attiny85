use attiny_hal::pac::{TC0, TC1};
use attiny_hal::port::mode::{Output, PwmOutput};
use attiny_hal::port::Pin;

pub struct Pwm {
    tc0: TC0,
    tc1: TC1,
}

impl Pwm {
    pub fn new(tc0: TC0, tc1: TC1) -> Self {
        // Configure TC0 for PWM
        tc0.tccr0a.write(|w| {
            w.wgm0()
                .pwm_fast()
                .com0a()
                .match_clear()
                .com0b()
                .match_clear()
        });
        tc0.tccr0b.write(|w| w.cs0().prescale_64());

        // Configure TC1 for PWM
        tc1.tccr1.write(|w| {
            w.cs1()
                .prescale_64()
                .pwm1a()
                .set_bit()
                .com1a()
                .match_clear()
        });

        Pwm { tc0, tc1 }
    }
    // pub fn get_output<PIN>(&mut self, pin: &Pin<Output, PIN>) -> PwmPin<PIN> {
    //     PwmPin { pin }
    // }
}

pub struct PwmPin<PIN> {
    pin: Pin<Output, PIN>,
}

impl<PIN> PwmPin<PIN> {
    // pub fn set_duty(&mut self, duty: u8) {
    //     self.pin.set_duty(duty);
    // }
}
