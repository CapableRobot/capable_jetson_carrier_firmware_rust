use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported

use std::thread;
use std::time::Duration;

use log::*;

use std::sync::Arc;
use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;

use esp_idf_hal::peripherals::Peripherals;

use sfsm::*;

// State structs
#[derive(Debug)]
struct Booted {
    button_press: bool,
    system_signal: bool
}

#[derive(Debug)]
struct Suspending {}

#[derive(Debug)]
struct Suspended {}

#[derive(Debug)]
struct Starting {}

add_state_machine!(Logic,
    Booted,
    [Booted, Suspending, Suspended, Starting],
    [
        Booted => Suspending,
        Suspending => Suspended,
        Suspended => Starting,
        Starting => Booted,
    ]
);

// Event Structs
#[derive(Debug)]
struct ButtonPress {}

#[derive(Debug)]
struct SystemSignal {}

#[derive(Debug)]
struct TimerComplete {}

add_messages!(Logic,
    [
        ButtonPress -> Booted,
        SystemSignal -> Booted,
        SystemSignal -> Suspending,
        TimerComplete -> Suspending,
        TimerComplete -> Starting
    ]
);

impl State for Booted {
    fn entry(&mut self) {
        info!("booted: entry")
    }
}

impl State for Suspending {
    fn entry(&mut self) {
        info!("suspending: entry")
    }
}

impl State for Suspended {
    fn entry(&mut self) {
        info!("suspended: entry")
    }
}

impl State for Starting {
    fn entry(&mut self) {
        info!("starting: entry")
    }
}


impl ReceiveMessage<ButtonPress> for Booted {
    fn receive_message(&mut self, _message: ButtonPress) {
        info!("booted: got button press");
        self.button_press = true;
    }
}

impl ReceiveMessage<SystemSignal> for Booted {
    fn receive_message(&mut self, _message: SystemSignal) {
        info!("booted: got system signal");
        self.system_signal = true;
    }
}

impl ReceiveMessage<SystemSignal> for Suspending {
    fn receive_message(&mut self, _message: SystemSignal) {
        info!("suspending: got system signal");
    }
}

impl ReceiveMessage<TimerComplete> for Suspending {
    fn receive_message(&mut self, _message: TimerComplete) {
        info!("suspending: timer is complete");
    }
}

impl ReceiveMessage<TimerComplete> for Starting {
    fn receive_message(&mut self, _message: TimerComplete) {
        info!("starting: timer is complete");
    }
}


impl Into<Booted> for Starting {
    fn into(self) -> Booted {
        Booted {
            button_press: false,
            system_signal: false,
        }
    }
}

derive_transition_into!(Booted, Suspending);
derive_transition_into!(Suspending, Suspended);
derive_transition_into!(Suspended, Starting);


impl Transition<Suspending> for Booted {
    fn guard(&self) -> TransitGuard {
        if self.button_press || self.system_signal {
            TransitGuard::Transit
        } else {
            TransitGuard::Remain
        }
    }
}

derive_transition!(Suspending, Suspended, TransitGuard::Transit);
derive_transition!(Suspended, Starting, TransitGuard::Transit);
derive_transition!(Starting, Booted, TransitGuard::Transit);

#[sfsm_trace]
fn trace(_log: &str) {
    // info!("{}", log);
}


fn main() -> anyhow::Result<()> {
    // Temporary. Will disappear once ESP-IDF 4.4 is released, but for now it is necessary to call this function once,
    // or else some patches to the runtime implemented by esp-idf-sys might not link properly.
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    info!("booted");

    let peripherals = Peripherals::take().unwrap();

    info!("gpio configured");

    let mut fsm = Logic::new();

    let state = Booted {
        button_press: false,
        system_signal: false,
    };
    fsm.start(state).unwrap();


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

    let button_press_notify = Arc::new(AtomicBool::new(false));

    {
        let mut button_on_time = 0;    
        let led_on_time = Arc::clone(&led_on_time);
        let button_press_notify = Arc::clone(&button_press_notify);

        thread::spawn(move || {
            let button = peripherals.pins.gpio2.into_input().unwrap();

            info!("button monitor thread start");

            loop {
                if button.is_low().unwrap() { 
                    button_on_time += button_interval;
                } else {
                    button_on_time = 0;
                    button_press_notify.store(false, Ordering::Relaxed);
                    led_on_time.store(led_default_on_time, Ordering::Relaxed)
                }

                if button_on_time >= button_debounce {
                    // Show that button has been pressed for a suffient time
                    led_on_time.store(500, Ordering::Relaxed);

                    if button_press_notify.load(Ordering::Relaxed) == false {
                        button_press_notify.store(true, Ordering::Relaxed);
                    }
                }

                thread::sleep(Duration::from_millis(button_interval));
            }
        });
    }

    info!("joining threads");

    loop {
        if button_press_notify.load(Ordering::Relaxed) == true {
            if IsState::<Booted>::is_state(&fsm) {
                PushMessage::<Booted, ButtonPress>::push_message(&mut fsm, ButtonPress {}).unwrap();
            }
        }
        let _ = fsm.step().unwrap();
        
        thread::sleep(Duration::from_millis(100));
    }
}
