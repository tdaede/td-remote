#![no_main]
#![no_std]

use panic_halt as _;

use bitvec::prelude::*;
use lpc8xx_hal::{
    cortex_m::interrupt as _interrupt, cortex_m_rt::entry, pac::interrupt as interrupt,  delay::Delay, gpio::Level, gpio::GpioPin, gpio::direction, prelude::*, CorePeripherals, pac::NVIC, pac::Interrupt,
    Peripherals
};

// tuned to create 38kHz modulation
const MODULATION_PERIOD: u16 = 18;

fn send_1ms_pulse<P>(delay: &mut Delay, led: &mut GpioPin<P, direction::Output>) where P: lpc8xx_hal::pins::Trait {
    for _ in 0..12 {
        led.set_high();
        delay.delay_us(MODULATION_PERIOD * 2 / 4);
        led.set_low();
        delay.delay_us(MODULATION_PERIOD * 2 / 4);
    }
    delay.delay_us(680 as u16);
}

fn send_2ms_pulse<P>(delay: &mut Delay, led: &mut GpioPin<P, direction::Output>) where P: lpc8xx_hal::pins::Trait {
    for _ in 0..12 {
        led.set_high();
        delay.delay_us(MODULATION_PERIOD * 2 / 4);
        led.set_low();
        delay.delay_us(MODULATION_PERIOD * 2 / 4);
    }
    delay.delay_us(1680_u16);
}

const CODE_ARRAY: [[u16; 4]; 6] = [
    [0b01000, 0b00000, 0b01101, 0b01110],
    [0b00000, 0b00010, 0b00001, 0b00110],
    [0b01111, 0b10000, 0b10001, 0b10010],
    [0b00000, 0b10011, 0b10100, 0b10101],
    [0b01001, 0b10110, 0b10111, 0b11000],
    [0b00100, 0b11001, 0b11010, 0b11011],
];

fn send_code<P>(code: u16, delay: &mut Delay, led: &mut GpioPin<P, direction::Output>) where P: lpc8xx_hal::pins::Trait  {
    // first code
    send_1ms_pulse(delay, led);
    send_1ms_pulse(delay, led);
    send_1ms_pulse(delay, led);
    for bit in code.view_bits::<Lsb0>().iter().take(9) {
        if *bit {
            send_2ms_pulse(delay, led);
        } else {
            send_1ms_pulse(delay, led);
        }
    }
    send_1ms_pulse(delay, led); // trailing pulse
    delay.delay_ms(40_u16);
    // second code
    send_1ms_pulse(delay, led);
    send_1ms_pulse(delay, led);
    send_1ms_pulse(delay, led);
    for bit in code.view_bits::<Lsb0>().iter().take(9) {
        if *bit {
            send_1ms_pulse(delay, led);
        } else {
            send_2ms_pulse(delay, led);
        }
    }
    send_1ms_pulse(delay, led); // trailing pulse
    delay.delay_ms(40_u16);
}

fn pin_interrupt_handler() {
}

#[interrupt]
fn PIN_INT0() {
    pin_interrupt_handler();
}
#[interrupt]
fn PIN_INT1() {
    pin_interrupt_handler();
}
#[interrupt]
fn PIN_INT2() {
    pin_interrupt_handler();
}
#[interrupt]
fn PIN_INT3() {
    pin_interrupt_handler();
}

#[entry]
fn main() -> ! {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();

    // Initialize the APIs of the peripherals we need.
    let mut delay = Delay::new(cp.SYST);

    let syscon = p.SYSCON.free();

    syscon.sysahbclkctrl.modify(|_, w| w.iocon().set_bit());

    let iocon = p.IOCON;
    // first two pins use "i2c" mode for open drain
    // it is absolutely critical for low power that pullups are disabled
    iocon.pio0_10.modify(|_, w| w.i2cmode().standarad_i2c());
    iocon.pio0_11.modify(|_, w| w.i2cmode().standarad_i2c());
    iocon.pio0_13.modify(|_, w| w.od().enabled().mode().inactive());
    iocon.pio0_14.modify(|_, w| w.od().enabled().mode().inactive());
    iocon.pio0_15.modify(|_, w| w.od().enabled().mode().inactive());
    iocon.pio0_17.modify(|_, w| w.od().enabled().mode().inactive());
    iocon.pio0_23.modify(|_, w| w.mode().inactive());
    iocon.pio0_3.modify(|_, w| w.mode().inactive()); // external pulldown is provided here

    let gpio = p.GPIO; // GPIO is initialized by default on LPC82x.

    let (led, token) = (p.pins.pio0_23, gpio.tokens.pio0_23);

    let pinint = p.PININT;
    unsafe {
        syscon.pintsel[0].write(|w| w.intpin().bits(1));
        syscon.pintsel[1].write(|w| w.intpin().bits(4));
        syscon.pintsel[2].write(|w| w.intpin().bits(8));
        syscon.pintsel[3].write(|w| w.intpin().bits(9));
    }
    //syscon.sysahbclkctrl.modify(|_, w| w.gpio().set_bit()); // this should already be enabled
    syscon.starterp0.modify(|_,w| w.pint0().enabled().pint1().enabled().pint2().enabled().pint3().enabled());
    unsafe {
        pinint.isel.write(|w| w.pmode().bits(0b00001111)); // level sensitive
        pinint.ienf.write(|w| w.enaf().bits(0b00000000)); // low level sensitive interrupt
        pinint.ienr.write(|w| w.enrl().bits(0b00001111)); // enable interrupt
    }

    let mut led = led.into_output_pin(token, Level::Low);

    let mut row0 = p.pins.pio0_10.into_output_pin(gpio.tokens.pio0_10, Level::High);
    let mut row1 = p.pins.pio0_11.into_output_pin(gpio.tokens.pio0_11, Level::High);
    let mut row2 = p.pins.pio0_13.into_output_pin(gpio.tokens.pio0_13, Level::High);
    let mut row3 = p.pins.pio0_14.into_output_pin(gpio.tokens.pio0_14, Level::High);
    let mut row4 = p.pins.pio0_15.into_output_pin(gpio.tokens.pio0_15, Level::High);
    let mut row5 = p.pins.pio0_17.into_output_pin(gpio.tokens.pio0_17, Level::High);

    let col0 = p.pins.pio0_1.into_input_pin(gpio.tokens.pio0_1);
    let col1 = p.pins.pio0_4.into_input_pin(gpio.tokens.pio0_4);
    let col2 = p.pins.pio0_8.into_input_pin(gpio.tokens.pio0_8);
    let col3 = p.pins.pio0_9.into_input_pin(gpio.tokens.pio0_9);

    let mut scb = cp.SCB;
    let mut pmu = p.PMU.split().handle;

    _interrupt::free(|_| {
        unsafe {
            NVIC::unmask(Interrupt::PIN_INT0);
            NVIC::unmask(Interrupt::PIN_INT1);
            NVIC::unmask(Interrupt::PIN_INT2);
            NVIC::unmask(Interrupt::PIN_INT3);
        }

        loop {
            for row in 0..6 {
                match row {
                    0 => row0.set_low(),
                    1 => row1.set_low(),
                    2 => row2.set_low(),
                    3 => row3.set_low(),
                    4 => row4.set_low(),
                    5 => row5.set_low(),
                    _ => unreachable!()
                };
                for col in 0..4 {
                    let state = match col {
                        0 => col0.is_low(),
                        1 => col1.is_low(),
                        2 => col2.is_low(),
                        3 => col3.is_low(),
                        _ => unreachable!()
                    };
                    if state {
                        send_code(CODE_ARRAY[row][col], &mut delay, &mut led);
                    }
                }
                match row {
                    0 => row0.set_high(),
                    1 => row1.set_high(),
                    2 => row2.set_high(),
                    3 => row3.set_high(),
                    4 => row4.set_high(),
                    5 => row5.set_high(),
                    _ => unreachable!()
                };
            }
            // now set all rows low for interrupt purposes
            row0.set_low();
            row1.set_low();
            row2.set_low();
            row3.set_low();
            row4.set_low();
            row5.set_low();
            NVIC::unpend(Interrupt::PIN_INT0);
            NVIC::unpend(Interrupt::PIN_INT1);
            NVIC::unpend(Interrupt::PIN_INT2);
            NVIC::unpend(Interrupt::PIN_INT3);
            unsafe { pmu.enter_power_down_mode(&mut scb) };
            row0.set_high();
            row1.set_high();
            row2.set_high();
            row3.set_high();
            row4.set_high();
            row5.set_high();
        }
    })
}
