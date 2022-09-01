use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported

use std::thread;
use std::time::Duration;

use log::*;

use std::sync::Arc;
use core::sync::atomic::{AtomicU64, Ordering};

use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;

use esp_idf_hal::peripherals::Peripherals;


fn main() -> anyhow::Result<()> {
    // Temporary. Will disappear once ESP-IDF 4.4 is released, but for now it is necessary to call this function once,
    // or else some patches to the runtime implemented by esp-idf-sys might not link properly.
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    info!("booted");

    let peripherals = Peripherals::take().unwrap();

    info!("gpio configured");

    let led_interval = 1000;
    let led_default_on_time = 50;
    let led_on_time = Arc::new(AtomicU64::new(led_default_on_time));

    {
        let led_on_time = Arc::clone(&led_on_time);
        thread::spawn(move || {
            let mut led = peripherals.pins.gpio19.into_output().unwrap();
            
            info!("heartbeat thread start");

            loop {
                led.set_low().unwrap();
                thread::sleep(Duration::from_millis(led_on_time.load(Ordering::Relaxed)));

                led.set_high().unwrap();
                thread::sleep(Duration::from_millis(led_interval - led_on_time.load(Ordering::Relaxed)));
            }
        });
    }

    let button_interval = 100;
    let button_debounce = 1000;

    {
        let mut button_on_time = 0;
        let led_on_time = Arc::clone(&led_on_time);

        thread::spawn(move || {
            let button = peripherals.pins.gpio2.into_input().unwrap();

            info!("button monitor thread start");

            loop {
                if button.is_low().unwrap() { 
                    button_on_time += button_interval;
                } else {
                    button_on_time = 0;
                    led_on_time.store(led_default_on_time, Ordering::Relaxed)
                }

                if button_on_time >= button_debounce {
                    // Show that button has been pressed for a suffient time
                    led_on_time.store(500, Ordering::Relaxed)
                }

                thread::sleep(Duration::from_millis(button_interval));
            }
        });
    }

    info!("joining threads");

    loop {
        thread::sleep(Duration::from_millis(100));
    }
}
