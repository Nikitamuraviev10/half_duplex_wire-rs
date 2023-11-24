#![no_std]
use core::mem::size_of;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::{InputPin, OutputPin};
const BUF_SIZE: usize = 8;

macro_rules! io_err {
    ( $i : expr ) => {
        $i.map_err(|_| Error::IO)
    };
}

#[derive(Debug)]
pub enum Error {
    Busy,
    Unavailable,
    IO,
    NoResponse,
}

pub trait ReadWrite {
    fn write(&mut self, data: u8, delay: &mut impl DelayMs<u8>) -> Result<(), Error>;
    fn read(&mut self, delay: &mut impl DelayMs<u8>) -> Result<u8, Error>;
}

pub struct HalfDuplexWire<F2, F1, I, O>
where
    F1: FnMut(O) -> I,
    F2: FnMut(I) -> O,
    I: InputPin,
    O: OutputPin,
{
    pin: Option<I>,
    into_input: F1,
    into_output: F2,
    delay: u8,
}

impl<F2, F1, I, O> ReadWrite for HalfDuplexWire<F2, F1, I, O>
where
    F1: Fn(O) -> I,
    F2: Fn(I) -> O,
    I: InputPin,
    O: OutputPin,
{
    fn write(&mut self, data: u8, delay: &mut impl DelayMs<u8>) -> Result<(), Error> {
        let pin = match self.pin.take() {
            Some(s) => s,
            None => return Err(Error::Unavailable),
        };

        if io_err!(pin.is_low())? {
            return Err(Error::Busy);
        }

        self.skip_phase(delay, 4);

        if io_err!(pin.is_low())? {
            return Err(Error::Busy);
        }

        let mut pin = (self.into_output)(pin);

        pin.set_low().ok();

        self.skip_phase(delay, 4);

        let mut mask = 0x80;
        for _ in 0..8 {
            if data & mask != 0 {
                pin.set_high().ok();
                self.skip_phase(delay, 4);
                pin.set_low().ok();
                self.skip_phase(delay, 4);
            } else {
                pin.set_high().ok();
                self.skip_phase(delay, 2);
                pin.set_low().ok();
                self.skip_phase(delay, 6);
            }

            mask >>= 1;
        }

        let pin = (self.into_input)(pin);
        self.pin = Some(pin);
        return Ok(());
    }

    fn read(&mut self, delay: &mut impl DelayMs<u8>) -> Result<u8, Error> {
        let pin = match self.pin.take() {
            Some(s) => s,
            None => return Err(Error::Unavailable),
        };

        let mut ed = EdgeDetector::new(pin);

        let mut data = 0u8;

        loop {
            if ed.risig_edge() {
                self.skip_phase(delay, 3);

                let tmp = io_err!(ed.is_high())?;

                self.skip_phase(delay, 3);

                if io_err!(ed.is_high())? {
                    break;
                } else {
                    data <<= 1;
                    data |= tmp as u8;
                }
            }
        }

        self.pin = Some(ed.release());
        return Ok(data);
    }
}

impl<F2, F1, I, O> HalfDuplexWire<F2, F1, I, O>
where
    F1: Fn(O) -> I,
    F2: Fn(I) -> O,
    I: InputPin,
    O: OutputPin,
{
    pub fn new(pin: I, into_output: F2, into_input: F1, delay: u8) -> Self {
        HalfDuplexWire {
            pin: Some(pin),
            into_input: into_input,
            into_output: into_output,
            delay: delay,
        }
    }

    pub fn skip_phase(&mut self, delay: &mut impl DelayMs<u8>, n: u8) {
        for _ in 0..n {
            delay.delay_ms(self.delay);
        }
    }

    pub fn stream_request(&mut self, delay: &mut impl DelayMs<u8>) -> Result<(), Error> {
        if let Some(pin) = &self.pin {
            if io_err!(pin.is_high())? {
                delay.delay_ms(self.delay);
                return Err(Error::NoResponse);
            } else {
                return Ok(());
            }
        }

        delay.delay_ms(self.delay);
        return Err(Error::Unavailable);
    }

    pub fn release(mut self) -> Result<I, Error> {
        let pin = match self.pin.take() {
            Some(s) => s,
            None => return Err(Error::Unavailable),
        };

        return Ok(pin);
    }

    pub fn get<T>(&mut self, delay: &mut impl DelayMs<u8>) -> Result<T, Error>
    where
        T: Copy,
    {
        let mut buf = [0u8; BUF_SIZE];
        let size = size_of::<T>();

        for i in 0..size {
            buf[i] = self.read(delay)?;
        }

        let tmp = unsafe { &*(buf[0..size].as_ptr() as *const T) };

        return Ok(*tmp);
    }
}

pub struct EdgeDetector<T> {
    pin: T,
    status: bool,
}

impl<T> EdgeDetector<T>
where
    T: InputPin,
{
    pub fn new(pin: T) -> Self {
        let status = pin.is_high().unwrap_or(false);
        return Self {
            pin: pin,
            status: status,
        };
    }

    pub fn risig_edge(&mut self) -> bool {
        let status = self.pin.is_high();
        if let Ok(status) = status {
            if status == self.status {
                return false;
            } else {
                self.status = status;
                return status;
            }
        } else {
            return false;
        }
    }

    pub fn release(self) -> T {
        return self.pin;
    }

    pub fn is_high(&mut self) -> Result<bool, T::Error> {
        return self.pin.is_high();
    }
}
