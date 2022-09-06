use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported

use std::thread;
use std::time::Duration;

use log::*;

use std::sync::Arc;
use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;

use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::gpio;
use esp_idf_sys as idf;

use mut_static::MutStatic;
use lazy_static::lazy_static;

mod helpers;
use crate::helpers::atomic_esp_system_time::AtomicSystemTime;

use sfsm::*;

static PWR_LED_OFF_TIME: AtomicU64 = AtomicU64::new(100);
static PWR_LED_ON_TIME: AtomicU64 = AtomicU64::new(0);

static HALTING_TIME : u64 = 15_000;
static BOOTING_TIME : u64 = 25_000;

#[derive(PartialEq)]
pub enum IO {
    On,
    Off
}


static BUFFER_PIN: i32 = 18;

struct IOPins {
    power_pin: gpio::Gpio10<gpio::Output>,
    sleep_pin: gpio::Gpio7<gpio::Output>,
    pwr_led_pin: gpio::Gpio4<gpio::Output>,
    dbg_led_pin: gpio::Gpio19<gpio::Output>,
    button_pin: gpio::Gpio2<gpio::Input>,
    signal_pin: gpio::Gpio9<gpio::Input>,
    sense_pin: gpio::Gpio8<gpio::Input>,
}

impl IOPins {
    fn new() -> IOPins {
        let peripherals = Peripherals::take().unwrap();
        let pins = peripherals.pins;

        // Changing from esp_idf_hal functions to lower level IDF unsafe
        // functions lowers the length of the high pulse on BUFFER_PIN from
        // 9 ms to around 1.4 us (6400 times shorter).  Unclear exactly what is
        // causing the 1.4 us pulse (as output level should) be zero before
        // the output is turned on, but it is short enought to not cause the
        // chip to self-reset (still allows control over the buffer control pin)
        unsafe { 
            idf::gpio_set_level(BUFFER_PIN, 0);
            idf::gpio_set_direction(BUFFER_PIN, idf::GPIO_MODE_DEF_OUTPUT); 
        }

        let sense_pin = pins.gpio8.into_input().unwrap();
        let host_on = sense_pin.is_high().unwrap();

        let mut power_pin = pins.gpio10.into_output().unwrap();
        let mut sleep_pin = pins.gpio7.into_output().unwrap();

        // If host is on (e.g. 3v3 rail is on) then keep power rail on
        if host_on {
            power_pin.set_high().unwrap();
            sleep_pin.set_high().unwrap();
            info!("iopins: host is on");
        } else {
            power_pin.set_low().unwrap();
            sleep_pin.set_low().unwrap();
            info!("iopins: host is off");
        }

        IOPins {
            power_pin: power_pin,
            sleep_pin: sleep_pin,
            pwr_led_pin: pins.gpio4.into_output().unwrap(),
            dbg_led_pin: pins.gpio19.into_output().unwrap(),
            button_pin: pins.gpio2.into_input().unwrap(),
            signal_pin: pins.gpio9.into_input().unwrap(),
            sense_pin: sense_pin,
        }
    }

    fn buffer_connect(&mut self) {
        unsafe {idf::gpio_set_level(BUFFER_PIN, 1);}
    }

    fn buffer_isolate(&mut self) {
        unsafe {idf::gpio_set_level(BUFFER_PIN, 0);}
    }

    fn host_sleep_assert(&mut self) {
        self.sleep_pin.set_low().unwrap();    
    }

    fn host_sleep_release(&mut self) {
        self.sleep_pin.set_high().unwrap();    
    }    

    fn host_power(&mut self, state:IO) {
        if state == IO::On {
            self.power_pin.set_high().unwrap();
        } else {
            self.power_pin.set_low().unwrap();    
        }
    }

    fn power_led(&mut self, state:IO) {
        if state == IO::On {
            self.pwr_led_pin.set_high().unwrap();
        } else {
            self.pwr_led_pin.set_low().unwrap();    
        }
    }

    fn debug_led(&mut self, state:IO) {
        if state == IO::On {
            self.dbg_led_pin.set_low().unwrap();
        } else {
            self.dbg_led_pin.set_high().unwrap();    
        }
    }

    fn is_signal_asserted(&self) -> bool {
        self.signal_pin.is_low().unwrap()
    }

    fn is_signal_released(&self) -> bool {
        self.signal_pin.is_high().unwrap()
    }

    fn is_button_pressed(&self) -> bool {
        self.button_pin.is_low().unwrap()
    }   

    fn is_button_released(&self) -> bool {
        self.button_pin.is_high().unwrap()
    }

    fn is_host_on(&self) -> bool {
        self.sense_pin.is_high().unwrap()
    }
}

lazy_static! {
    static ref IOPINS: MutStatic<IOPins> = {
        MutStatic::from(IOPins::new())
    };
}

#[derive(Debug, PartialEq)]
pub enum TimerState {
    Idle,
    Running,
    Complete,
}

// State structs
#[derive(Debug)]
struct Init {
    host_booted: bool,
}

#[derive(Debug)]
struct Booted {
    button_press: bool,
    system_signal: bool
}

#[derive(Debug)]
struct Suspending {
    system_signal: bool,
    started_at: u64,
}

#[derive(Debug)]
struct Suspended {
    button_press: bool
}

#[derive(Debug)]
struct Starting {
    started_at: u64,
}

add_state_machine!(Logic,
    Init,
    [Init, Booted, Suspending, Suspended, Starting],
    [
        Init => Booted,
        Init => Suspended,
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

add_messages!(Logic,
    [
        ButtonPress -> Booted,
        ButtonPress -> Suspended,
        SystemSignal -> Booted,
        SystemSignal -> Suspending,
    ]
);

derive_state!(Init);

impl State for Booted {
    fn entry(&mut self) {
        info!("booted: entry");
        PWR_LED_OFF_TIME.store(0, Ordering::Relaxed);
        PWR_LED_ON_TIME.store(100, Ordering::Relaxed);

        // Turn on isolation buffer
        IOPINS.write().unwrap().buffer_connect();
    }
}

impl State for Suspending {
    fn entry(&mut self) {
        info!("suspending: entry");
        PWR_LED_OFF_TIME.store(100, Ordering::Relaxed);
        PWR_LED_ON_TIME.store(100, Ordering::Relaxed);

        // Disable buffer, so that ESP continues to run when SOM shuts down
        IOPINS.write().unwrap().buffer_isolate();

        // Send power off signal to SOM
        IOPINS.write().unwrap().host_sleep_assert();
    }

    fn execute(&mut self) {
        // If we've gotten the system signal, and timer 
        // hasn't been started, start the shutdown timer
        if self.system_signal == true && self.started_at == 0 {
            info!("suspending: starting shutdown timer");
            self.started_at = AtomicSystemTime::now_millis();
        } 
    }

}

impl State for Suspended {
    fn entry(&mut self) {
        info!("suspended: entry");
        PWR_LED_ON_TIME.store(0, Ordering::Relaxed);

        // Turn off SOM regulators
        IOPINS.write().unwrap().host_power(IO::Off);

        // Release assertion on power signal
        IOPINS.write().unwrap().host_sleep_release();
    }
}

impl State for Starting {
    fn entry(&mut self) {
        info!("starting: entry");
        PWR_LED_OFF_TIME.store(100, Ordering::Relaxed);
        PWR_LED_ON_TIME.store(100, Ordering::Relaxed);

        // Turn on SOM regulators
        IOPINS.write().unwrap().host_power(IO::On);

        // Start booting timer
        self.started_at = AtomicSystemTime::now_millis();
    }
}


impl ReceiveMessage<ButtonPress> for Booted {
    fn receive_message(&mut self, _message: ButtonPress) {
        info!("booted: got button press");
        self.button_press = true;
    }
}

impl ReceiveMessage<ButtonPress> for Suspended {
    fn receive_message(&mut self, _message: ButtonPress) {
        info!("suspended: got button press");
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
        self.system_signal = true;
    }
}

impl Into<Booted> for Init {
    fn into(self) -> Booted {
        Booted {
            button_press: false,
            system_signal: false,
        }
    }
}

impl Into<Suspended> for Init {
    fn into(self) -> Suspended {
        Suspended {
            button_press: false,
        }
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

impl Into<Suspending> for Booted {
    fn into(self) -> Suspending {
        // System signal is passed here as if the host initiated the
        // shutdown, we don't need (or want) to wait for another signal
        // from the host.  We know it is going down.
        Suspending {
            system_signal: self.system_signal,
            started_at: 0,
        }
    }
}

impl Into<Suspended> for Suspending {
    fn into(self) -> Suspended {
        Suspended {
            button_press: false,
        }
    }
}

impl Into<Starting> for Suspended {
    fn into(self) -> Starting {
        Starting {
            started_at: 0,
        }
    }
}

impl Transition<Booted> for Init {
    fn guard(&self) -> TransitGuard {
        if self.host_booted {
            TransitGuard::Transit
        } else {
            TransitGuard::Remain
        }
    }
}

impl Transition<Suspended> for Init {
    fn guard(&self) -> TransitGuard {
        if self.host_booted {
            TransitGuard::Remain
        } else {
            TransitGuard::Transit
        }
    }
}


impl Transition<Suspending> for Booted {
    fn guard(&self) -> TransitGuard {
        if self.button_press || self.system_signal {
            TransitGuard::Transit
        } else {
            TransitGuard::Remain
        }
    }
}

impl Transition<Suspended> for Suspending {
    fn guard(&self) -> TransitGuard {
        let elapsed = AtomicSystemTime::now_millis() - self.started_at;

        if self.started_at > 0 && elapsed >= HALTING_TIME {
            TransitGuard::Transit
        } else {
            TransitGuard::Remain
        }
    }
}

impl Transition<Starting> for Suspended {
    fn guard(&self) -> TransitGuard {
        if self.button_press {
            TransitGuard::Transit
        } else {
            TransitGuard::Remain
        }
    }
}

impl Transition<Booted> for Starting {
    fn guard(&self) -> TransitGuard {
        let elapsed = AtomicSystemTime::now_millis() - self.started_at;

        if self.started_at > 0 && elapsed >= BOOTING_TIME {
            TransitGuard::Transit
        } else {
            TransitGuard::Remain
        }
    }
}

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

    // Setup default IO pin state
    IOPINS.write().unwrap();

    let mut fsm = Logic::new();

    let state = Init {
        host_booted: IOPINS.write().unwrap().is_host_on(),
    };
    fsm.start(state).unwrap();
     

    let dbg_led_interval = 1000;
    let dbg_led_default_on_time = 50;
    let dbg_led_on_time = Arc::new(AtomicU64::new(dbg_led_default_on_time));

    {
        
        let dbg_led_on_time = Arc::clone(&dbg_led_on_time);
        thread::spawn(move || {
            info!("heartbeat thread start");

            loop {
                IOPINS.write().unwrap().debug_led(IO::On);
                thread::sleep(Duration::from_millis(dbg_led_on_time.load(Ordering::Relaxed)));

                IOPINS.write().unwrap().debug_led(IO::Off);
                thread::sleep(Duration::from_millis(dbg_led_interval - dbg_led_on_time.load(Ordering::Relaxed)));
            }
        });
    }

    {
        thread::spawn(move || {
            info!("power LED thread start");

            loop {
                let off_time = PWR_LED_OFF_TIME.load(Ordering::Relaxed);
                let on_time = PWR_LED_ON_TIME.load(Ordering::Relaxed);

                if on_time > 0 {
                    // pwr_led.set_high().unwrap();
                    IOPINS.write().unwrap().power_led(IO::On);
                    thread::sleep(Duration::from_millis(on_time));
                }

                if off_time > 0 {
                    IOPINS.write().unwrap().power_led(IO::Off);
                    thread::sleep(Duration::from_millis(off_time));
                } 

                if off_time == 0 && on_time == 0 {
                    IOPINS.write().unwrap().power_led(IO::On);
                    thread::sleep(Duration::from_millis(100));  
                }
            }
        });
    }

    let button_interval = 100;
    let button_debounce = 2000;

    let button_press_notify = Arc::new(AtomicBool::new(false));
    let system_signal_notify = Arc::new(AtomicU64::new(0));

    {
        let mut button_on_time = 0;    
        let dbg_led_on_time = Arc::clone(&dbg_led_on_time);

        let button_press_notify = Arc::clone(&button_press_notify);
        let system_signal_notify = Arc::clone(&system_signal_notify);

        thread::spawn(move || {
            info!("input monitor thread starting");

            loop {
                if IOPINS.write().unwrap().is_button_pressed() { 
                    button_on_time += button_interval;
                } else {
                    button_on_time = 0;
                    button_press_notify.store(false, Ordering::Relaxed);
                    dbg_led_on_time.store(dbg_led_default_on_time, Ordering::Relaxed)
                }

                if button_on_time >= button_debounce {
                    // Show that button has been pressed for a suffient time
                    dbg_led_on_time.store(500, Ordering::Relaxed);

                    if button_press_notify.load(Ordering::Relaxed) == false {
                        button_press_notify.store(true, Ordering::Relaxed);
                    }
                }

                if IOPINS.write().unwrap().is_signal_asserted() { 
                    if system_signal_notify.load(Ordering::Relaxed) == 0 {
                        info!("pushing system signal");
                        system_signal_notify.store(1, Ordering::Relaxed);
                    }
                } else {
                    // Signal has been released, and has been passed on
                    // We can reset back to the default state
                    if system_signal_notify.load(Ordering::Relaxed) == 2 {
                        info!("clearing system signal");
                        system_signal_notify.store(0, Ordering::Relaxed);
                    }
                }

                thread::sleep(Duration::from_millis(button_interval));
            }
        });
    }

    info!("joining threads");

    loop {

        // Send button press notifications if they occured and if we're in the correct state
        if button_press_notify.load(Ordering::Relaxed) == true {
            if IsState::<Booted>::is_state(&fsm) {
                PushMessage::<Booted, ButtonPress>::push_message(&mut fsm, ButtonPress {}).unwrap();
            }

            if IsState::<Suspended>::is_state(&fsm) {
                PushMessage::<Suspended, ButtonPress>::push_message(&mut fsm, ButtonPress {}).unwrap();
            }
        }

        if system_signal_notify.load(Ordering::Relaxed) == 1 {
            // Signal that we've passed on the signal
            system_signal_notify.store(2, Ordering::Relaxed);
            
            // Used for host-initiated shutdowns
            if IsState::<Booted>::is_state(&fsm) {
                PushMessage::<Booted, SystemSignal>::push_message(&mut fsm, SystemSignal {}).unwrap();
            }

            // Used to signal that MCU-initiated shutdowns were received by the host
            if IsState::<Suspending>::is_state(&fsm) {
                PushMessage::<Suspending, SystemSignal>::push_message(&mut fsm, SystemSignal {}).unwrap();
            }

        }

        // Process the state machine
        let _ = fsm.step().unwrap();
        
        thread::sleep(Duration::from_millis(100));
    }
}
