#![allow(dead_code)]
use embedded_svc::sys_time::SystemTime;
use esp_idf_svc::systime::EspSystemTime;
use std::{
    sync::atomic::{AtomicU64, Ordering},
    time::Duration,
};

#[derive(Debug)]
pub enum Diff {
    /// The current System time is greater than the existing time
    HasPassed(Duration),

    /// The current System time is lesser than the existing time
    ToPass(Duration),
}

#[derive(Debug)]
pub struct AtomicSystemTime(AtomicU64);

impl AtomicSystemTime {
    pub fn now_duration() -> Duration {
        EspSystemTime {}.now()
    }

    pub fn now_millis() -> u64 {
        let current_system_time = Self::now_duration();
        current_system_time.as_millis() as u64
    }

    pub fn now() -> Self {
        let now_ms = Self::now_millis();
        Self(AtomicU64::new(now_ms as u64))
    }

    pub const fn from_millis(ms: u64) -> Self {
        Self(AtomicU64::new(ms as u64))
    }

    pub const fn from_duration(d: Duration) -> Self {
        Self(AtomicU64::new(d.as_millis() as u64))
    }

    pub fn as_millis(&self) -> u64 {
        self.0.load(Ordering::Relaxed)
    }

    pub fn as_duration(&self) -> Duration {
        let ms = self.as_millis();

        Duration::from_millis(ms)
    }

    pub fn since(&self) -> Diff {
        let now_ms = Self::now_millis();
        let existing = self.0.load(Ordering::Relaxed);

        if existing > now_ms {
            let diff = existing - now_ms;

            return Diff::ToPass(Duration::from_millis(diff as u64));
        }

        let diff = now_ms - existing;
        Diff::HasPassed(Duration::from_millis(diff as u64))
    }

    pub fn has_passed(&self) -> Option<Duration> {
        if let Diff::HasPassed(d) = self.since() {
            return Some(d);
        }

        None
    }

    pub fn to_pass(&self) -> Option<Duration> {
        if let Diff::ToPass(d) = self.since() {
            return Some(d);
        }

        None
    }

    pub fn set_now(&self) -> Self {
        let now_ms = Self::now_millis();

        self.0.store(now_ms, Ordering::Relaxed);

        Self::from_millis(now_ms)
    }

    pub fn add_duration_to_now(&self, d: Duration) -> Self {
        let now_d = Self::now_duration() + d;

        self.0.store(now_d.as_millis() as u64, Ordering::Relaxed);

        Self::from_duration(now_d)
    }

    pub fn add_millis_to_now(&self, millis: u64) -> Self {
        let now_ms = Self::now_millis() + millis;

        self.0.store(now_ms, Ordering::Relaxed);

        Self::from_millis(now_ms)
    }

    pub fn add_duration(&self, d: Duration) -> Self {
        let next = self.as_duration() + d;

        self.0.store(next.as_millis() as u64, Ordering::Relaxed);

        Self::from_duration(next)
    }

    pub fn add_millis(&self, millis: u64) -> Self {
        let next = self.as_millis() + millis;

        self.0.store(next, Ordering::Relaxed);

        Self::from_millis(next)
    }
}