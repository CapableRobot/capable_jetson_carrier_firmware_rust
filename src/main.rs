use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported

use std::thread;
use std::time::Duration;

use log::*;

use embedded_hal::digital::v2::OutputPin;

use esp_idf_hal::peripherals::Peripherals;

fn main() -> anyhow::Result<()> {
    // Temporary. Will disappear once ESP-IDF 4.4 is released, but for now it is necessary to call this function once,
    // or else some patches to the runtime implemented by esp-idf-sys might not link properly.
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    info!("booted");

    let peripherals = Peripherals::take().unwrap();
    let mut led = peripherals.pins.gpio19.into_output()?;
    
    info!("gpio configured");

    loop {
        led.set_high()?;
        thread::sleep(Duration::from_millis(1000));

        led.set_low()?;
        thread::sleep(Duration::from_millis(1000));
    }
}
