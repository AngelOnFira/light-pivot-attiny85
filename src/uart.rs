use attiny_hal::port::Pin;
use attiny_hal::port::mode::{Input, Output};
use avr_device::attiny85::USI;

pub struct Uart {
    usi: USI,
    rx: Pin<Input, PB3>,
    tx: Pin<Output, PB4>,
}

impl Uart {
    pub fn new(rx: Pin<Input, PB3>, tx: Pin<Output, PB4>) -> Self {
        let usi = unsafe { &*USI::ptr() };
        
        // Configure USI for UART operation
        usi.usicr.write(|w| w
            .usiwm().bits(0b00) // Three-wire mode
            .usics().bits(0b10) // External clock source on PB2
            .usiclk().set_bit() // Clock strobe on positive edge
        );

        Uart { usi, rx, tx }
    }

    pub fn receive(&mut self) -> Option<u8> {
        if self.usi.usisr.read().usioif().bit_is_set() {
            let data = self.usi.usidr.read().bits();
            self.usi.usisr.modify(|_, w| w.usioif().set_bit());
            Some(data)
        } else {
            None
        }
    }
}