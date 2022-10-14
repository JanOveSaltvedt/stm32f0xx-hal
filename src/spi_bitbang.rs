//! API for using the SPI peripheral as a way to implement timing sensitive bitbanged protocols
//! Only the MOSI pin is used, the other pins can be used for other purposes
//! 

use core::marker::PhantomData;
use core::{ops::Deref, ptr};

pub use embedded_hal::spi::{Mode, Phase, Polarity};

use crate::pac::SPI1;

use crate::gpio::*;

use crate::rcc::{Clocks, Rcc};

use crate::time::Hertz;

/// Typestate for 8-bit transfer size
pub struct EightBit;

/// SPI error
#[non_exhaustive]
#[derive(Debug)]
pub enum Error {
    /// Overrun occurred
    Overrun,
    /// Mode fault occurred
    ModeFault,
    /// CRC error
    Crc,
}

/// SPI abstraction
pub struct Spi<SPI,MOSIPIN> {
    spi: SPI,
    pin: MOSIPIN,
}

pub trait MosiPin<SPI> {}

macro_rules! spi_pins {
    ($($SPI:ident => {
        mosi => [$($mosi:ty),+ $(,)*],
    })+) => {
        $(
            $(
                impl MosiPin<crate::pac::$SPI> for $mosi {}
            )+
        )+
    }
}

spi_pins! {
    SPI1 => {
        mosi => [gpioa::PA7<Alternate<AF0>>, gpiob::PB5<Alternate<AF0>>],
    }
}

macro_rules! spi {
    ($($SPI:ident: ($spi:ident, $spiXen:ident, $spiXrst:ident, $apbenr:ident, $apbrstr:ident),)+) => {
        $(
            impl<MOSIPIN> Spi<$SPI, MOSIPIN> {
                /// Creates a new spi instance
                pub fn $spi<F>(
                    spi: $SPI,
                    pin: MOSIPIN,
                    mode: Mode,
                    speed: F,
                    rcc: &mut Rcc,
                ) -> Self
                where
                    MOSIPIN: MosiPin<$SPI>,
                    F: Into<Hertz>,
                {
                    /* Enable clock for SPI */
                    rcc.regs.$apbenr.modify(|_, w| w.$spiXen().set_bit());

                    /* Reset SPI */
                    rcc.regs.$apbrstr.modify(|_, w| w.$spiXrst().set_bit());
                    rcc.regs.$apbrstr.modify(|_, w| w.$spiXrst().clear_bit());

                    Spi::<$SPI, MOSIPIN> { spi, pin }.spi_init(mode, speed, rcc.clocks).into_8bit_width()
                }
            }
        )+
    }
}

spi! {
    SPI1: (spi1, spi1en, spi1rst, apb2enr, apb2rstr),
}

// It's s needed for the impls, but rustc doesn't recognize that
#[allow(dead_code)]
type SpiRegisterBlock = crate::pac::spi1::RegisterBlock;

impl<SPI, MOSIPIN> Spi<SPI, MOSIPIN>
where
    SPI: Deref<Target = SpiRegisterBlock>,
{
    fn spi_init<F>(self, mode: Mode, speed: F, clocks: Clocks) -> Self
    where
        F: Into<Hertz>,
    {
        /* Make sure the SPI unit is disabled so we can configure it */
        self.spi.cr1.modify(|_, w| w.spe().clear_bit());

        let br = match clocks.pclk().0 / speed.into().0 {
            0 => unreachable!(),
            1..=2 => 0b000,
            3..=5 => 0b001,
            6..=11 => 0b010,
            12..=23 => 0b011,
            24..=47 => 0b100,
            48..=95 => 0b101,
            96..=191 => 0b110,
            _ => 0b111,
        };

        // mstr: master configuration
        // lsbfirst: MSB first
        // ssm: enable software slave management (NSS pin free for other uses)
        // ssi: set nss high = master mode
        // dff: 8 bit frames
        // bidimode: 2-line unidirectional
        // spe: enable the SPI bus
        self.spi.cr1.write(|w| {
            w.cpha()
                .bit(mode.phase == Phase::CaptureOnSecondTransition)
                .cpol()
                .bit(mode.polarity == Polarity::IdleHigh)
                .mstr()
                .set_bit()
                .br()
                .bits(br)
                .lsbfirst()
                .clear_bit()
                .ssm()
                .set_bit()
                .ssi()
                .set_bit()
                .rxonly()
                .clear_bit()
                .bidimode()
                .clear_bit()
                .spe()
                .set_bit()
        });

        self
    }

    pub fn into_8bit_width(self) -> Spi<SPI, MOSIPIN> {
        // FRXTH: 8-bit threshold on RX FIFO
        // DS: 8-bit data size
        // SSOE: cleared to disable SS output
        self.spi
            .cr2
            .write(|w| w.frxth().set_bit().ds().eight_bit().ssoe().clear_bit());

        Spi {
            spi: self.spi,
            pin: self.pin,
        }
    }

    fn set_send_only(&mut self) {
        self.spi
            .cr1
            .modify(|_, w| w.bidimode().set_bit().bidioe().set_bit());
    }

    fn send_buffer_size(&mut self) -> u8 {
        match self.spi.sr.read().ftlvl().bits() {
            // FIFO empty
            0 => 4,
            // FIFO 1/4 full
            1 => 3,
            // FIFO 1/2 full
            2 => 2,
            // FIFO full
            _ => 0,
        }
    }

    fn check_send(&mut self) -> nb::Result<(), Error> {
        let sr = self.spi.sr.read();

        Err(if sr.ovr().bit_is_set() {
            nb::Error::Other(Error::Overrun)
        } else if sr.modf().bit_is_set() {
            nb::Error::Other(Error::ModeFault)
        } else if sr.crcerr().bit_is_set() {
            nb::Error::Other(Error::Crc)
        } else if sr.txe().bit_is_set() && sr.bsy().bit_is_clear() {
            return Ok(());
        } else {
            nb::Error::WouldBlock
        })
    }


    fn send_u8(&mut self, byte: u8) {
        // NOTE(write_volatile) see note above
        unsafe { ptr::write_volatile(&self.spi.dr as *const _ as *mut u8, byte) }
    }

    pub fn release(self) -> (SPI, MOSIPIN) {
        (self.spi, self.pin)
    }
}

impl<SPI, MOSIPIN> ::embedded_hal::blocking::spi::Write<u8>
    for Spi<SPI, MOSIPIN>
where
    SPI: Deref<Target = SpiRegisterBlock>,
{
    type Error = Error;

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        let mut bufcap: u8 = 0;

        // We only want to send, so we don't need to worry about the receive buffer overflowing
        self.set_send_only();

        // Make sure we don't continue with an error condition
        nb::block!(self.check_send())?;

        // We have a 32 bit buffer to work with, so let's fill it before checking the status
        for word in words {
            // Loop as long as our send buffer is full
            while bufcap == 0 {
                bufcap = self.send_buffer_size();
            }

            self.send_u8(*word);
            bufcap -= 1;
        }

        // Do one last status register check before continuing
        nb::block!(self.check_send()).ok();
        Ok(())
    }
}

