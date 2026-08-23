//! Port of NavKit `StandingDetector` (location_guide/standing_detector.cpp).
//!
//! Provenance: maps-product `backend/mobile/libs/directions/guidance/location_guide/standing_detector.{h,cpp}`
//! and `guides/standing_guide/standing_guide_impl.cpp` (experiment defaults).
//!
//! Intentional deviation: vendor only *reports* standing when the current route
//! position is inside a standing segment annotation (traffic lights, stops);
//! `possibleStandingDetected` is logged but reported as `Moving`. UzRoute does
//! not emit standing-segment annotations yet, so UzNav reports standing
//! regardless of segment membership (the vendor `standing_guide_ignore_segments`
//! experiment behavior). When UzRoute gains standing segments this gate returns.

#[cfg(all(feature = "std", not(feature = "web-time")))]
use std::time::{Duration, SystemTime};
#[cfg(feature = "web-time")]
use web_time::{Duration, SystemTime};

use serde::{Deserialize, Serialize};

/// Standing detector state carried inside [`super::UzmatchState`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "uniffi", derive(uniffi::Record))]
pub struct StandingState {
    /// Vendor `oldestStandingSignalTimestamp_`.
    pub oldest_signal: Option<SystemTime>,
    /// Vendor `latestStandingSignalTimestamp_`.
    pub latest_signal: Option<SystemTime>,
    /// Vendor `currentStatus_ == Standing`.
    pub is_standing: bool,
}

impl Default for StandingState {
    fn default() -> Self {
        Self {
            oldest_signal: None,
            latest_signal: None,
            is_standing: false,
        }
    }
}

impl StandingState {
    /// Vendor `resetHistory`: any reason clears both timestamps and the status.
    pub fn reset(&mut self) {
        self.oldest_signal = None;
        self.latest_signal = None;
        self.is_standing = false;
    }

    /// Vendor query-time expiry (`tooLongWithoutUpdates`): if the latest signal
    /// is older than `signal_expiry`, history is reset before anything else.
    pub fn expire_if_stale(&mut self, now: SystemTime, signal_expiry: Duration) {
        if let Some(latest) = self.latest_signal {
            if now.duration_since(latest).map(|d| d > signal_expiry).unwrap_or(false) {
                self.reset();
            }
        }
    }

    /// Vendor `onRouteBoundLocation` for an accurate signal.
    ///
    /// - speed above `speed_threshold` -> moving signal -> reset.
    /// - speed at/below threshold -> standing signal; once the signal span
    ///   (`latest - oldest`) reaches `detection_period`, standing is detected.
    ///
    /// Inaccurate (coarse) and off-route signals are handled by the caller
    /// (they reset history, mirroring `onOffRouteSignal` / `coarse_signal`).
    pub fn on_accurate_signal(
        &mut self,
        speed_mps: Option<f64>,
        timestamp: SystemTime,
        speed_threshold: f64,
        detection_period: Duration,
    ) {
        if speed_mps.is_some_and(|speed| speed > speed_threshold) {
            self.reset();
            return;
        }

        if self.oldest_signal.is_none() {
            self.oldest_signal = Some(timestamp);
        }
        self.latest_signal = Some(timestamp);

        let standing_time = self
            .oldest_signal
            .and_then(|oldest| timestamp.duration_since(oldest).ok())
            .unwrap_or(Duration::ZERO);
        if standing_time >= detection_period {
            self.is_standing = true;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(secs: u64) -> SystemTime {
        SystemTime::UNIX_EPOCH + Duration::from_secs(secs)
    }

    const THRESHOLD: f64 = 0.5;
    const PERIOD: Duration = Duration::from_secs(7);
    const EXPIRY: Duration = Duration::from_secs(5);

    #[test]
    fn standing_after_seven_seconds_below_threshold() {
        let mut state = StandingState::default();
        for t in 0..7 {
            state.on_accurate_signal(Some(0.0), at(t), THRESHOLD, PERIOD);
            assert!(!state.is_standing, "must not be standing at t={t}");
        }
        state.on_accurate_signal(Some(0.3), at(7), THRESHOLD, PERIOD);
        assert!(state.is_standing);
    }

    #[test]
    fn moving_signal_resets_history() {
        let mut state = StandingState::default();
        state.on_accurate_signal(Some(0.0), at(0), THRESHOLD, PERIOD);
        state.on_accurate_signal(Some(0.0), at(6), THRESHOLD, PERIOD);
        state.on_accurate_signal(Some(2.0), at(7), THRESHOLD, PERIOD);
        assert!(!state.is_standing);
        assert_eq!(state.oldest_signal, None);
        // The 7 s window restarts from scratch.
        state.on_accurate_signal(Some(0.0), at(8), THRESHOLD, PERIOD);
        state.on_accurate_signal(Some(0.0), at(14), THRESHOLD, PERIOD);
        assert!(!state.is_standing);
        state.on_accurate_signal(Some(0.0), at(15), THRESHOLD, PERIOD);
        assert!(state.is_standing);
    }

    #[test]
    fn stale_signal_expires() {
        let mut state = StandingState::default();
        state.on_accurate_signal(Some(0.0), at(0), THRESHOLD, PERIOD);
        state.on_accurate_signal(Some(0.0), at(7), THRESHOLD, PERIOD);
        assert!(state.is_standing);
        // No update for more than the 5 s signal interval.
        state.expire_if_stale(at(13), EXPIRY);
        assert!(!state.is_standing);
        assert_eq!(state.latest_signal, None);
    }

    #[test]
    fn missing_speed_counts_as_standing_signal() {
        // Vendor compares `location.speed > threshold`; absent speed is not above
        // threshold, so it behaves as a standing signal.
        let mut state = StandingState::default();
        state.on_accurate_signal(None, at(0), THRESHOLD, PERIOD);
        state.on_accurate_signal(None, at(7), THRESHOLD, PERIOD);
        assert!(state.is_standing);
    }
}
