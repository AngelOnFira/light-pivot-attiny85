const BUFFER_SIZE: usize = 32;

pub struct Buffer {
    data: [u8; BUFFER_SIZE],
    read_idx: usize,
    write_idx: usize,
}

impl Buffer {
    pub const fn new() -> Self {
        Buffer {
            data: [0; BUFFER_SIZE],
            read_idx: 0,
            write_idx: 0,
        }
    }

    pub fn push(&mut self, byte: u8) {
        self.data[self.write_idx] = byte;
        self.write_idx = (self.write_idx + 1) % BUFFER_SIZE;
        if self.write_idx == self.read_idx {
            self.read_idx = (self.read_idx + 1) % BUFFER_SIZE;
        }
    }

    pub fn pop(&mut self) -> Option<u8> {
        if self.read_idx != self.write_idx {
            let byte = self.data[self.read_idx];
            self.read_idx = (self.read_idx + 1) % BUFFER_SIZE;
            Some(byte)
        } else {
            None
        }
    }

    pub fn len(&self) -> usize {
        if self.write_idx >= self.read_idx {
            self.write_idx - self.read_idx
        } else {
            BUFFER_SIZE - self.read_idx + self.write_idx
        }
    }
}