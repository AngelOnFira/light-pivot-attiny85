use attiny_hal::port::Pin;
use attiny_hal::port::mode::PwmOutput;
use avr_device::attiny85::{TC0, TC1};

pub struct Pwm {
    tc0: TC0,
    tc1: TC1,
}

impl Pwm {
    pub fn new(tc0: TC0, tc1: TC1) -> Self {
        // Configure TC0 for PWM
        tc0.tccr0a.write(|w| w
            .wgm0().pwm_fast()
            .com0a().match_clear()
            .com0b().match_clear()
        );
        tc0.tccr0b.write(|w| w.cs0().prescale_64());

        // Configure TC1 for PWM
        tc1.tccr1.write(|w| w
            .cs1().prescale_64()
            .pwm1a().set_bit()
            .com1a().match_clear()
        );

        Pwm { tc0, tc1 }
    }

    pub fn get_output<PIN>(&mut self, pin: &Pin<PwmOutput<TC0>, PIN>) -> PwmPin<PIN> {
        PwmPin { pin: *pin }
    }
}

pub struct PwmPin<PIN> {
    pin: Pin<PwmOutput<TC0>, PIN>,
}

impl<PIN> PwmPin<PIN> {
    pub fn set_duty(&mut self, duty: u8) {
        self.pin.set_duty(duty);
    }
}