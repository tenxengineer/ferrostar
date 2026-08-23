//! Port of NavKit `LocationClassStateMachine` (location_guide/location_class_state_machine.cpp).
//!
//! Provenance: maps-product `backend/mobile/libs/directions/guidance/location_guide/`
//! `location_class_state_machine.{h,cpp}` and `location_guide_config.h` timeouts.

#[cfg(all(feature = "std", not(feature = "web-time")))]
use std::time::{Duration, SystemTime};
#[cfg(feature = "web-time")]
use web_time::{Duration, SystemTime};

use serde::{Deserialize, Serialize};

/// Vendor `EXACT_LOCATION_TIMEOUT` (location_guide_config.h).
pub const EXACT_LOCATION_TIMEOUT: Duration = Duration::from_secs(3);
/// Vendor `EXTRAPOLATED_LOCATION_TIMEOUT`.
pub const EXTRAPOLATED_LOCATION_TIMEOUT: Duration = Duration::from_secs(30);
/// Vendor `COARSE_LOCATION_TIMEOUT`.
pub const COARSE_LOCATION_TIMEOUT: Duration = Duration::from_secs(180);

/// Location accuracy class, mirroring vendor `LocationState`/`LocationClass`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "uniffi", derive(uniffi::Enum))]
pub enum UzLocationClass {
    /// Fresh accurate fix.
    Fine,
    /// No fine fix for [`EXACT_LOCATION_TIMEOUT`]; position is extrapolated.
    Extrapolated,
    /// No fine fix for [`EXTRAPOLATED_LOCATION_TIMEOUT`]; only coarse signal.
    Coarse,
    /// No usable signal for [`COARSE_LOCATION_TIMEOUT`]; location must not be shown.
    Outdated,
}

impl UzLocationClass {
    /// Vendor `isAccurate`: Fine and Extrapolated are accurate enough to bind to the route.
    pub fn is_accurate(&self) -> bool {
        matches!(self, Self::Fine | Self::Extrapolated)
    }
}

/// Pure state machine; only non-decreasing timestamps are supported (vendor contract).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "uniffi", derive(uniffi::Record))]
pub struct LocationClassState {
    pub class: UzLocationClass,
    pub since: Option<SystemTime>,
}

impl Default for LocationClassState {
    fn default() -> Self {
        Self {
            class: UzLocationClass::Outdated,
            since: None,
        }
    }
}

impl LocationClassState {
    /// Vendor `onFineLocation`.
    pub fn on_fine_location(&mut self, event: SystemTime) {
        self.set_state(UzLocationClass::Fine, event);
    }

    /// Vendor `onCoarseLocation`: only downgrades when not currently accurate.
    pub fn on_coarse_location(&mut self, event: SystemTime) {
        if !self.class.is_accurate() {
            self.set_state(UzLocationClass::Coarse, event);
        }
    }

    /// Vendor `onCoarseBoundLocation`: a coarse signal still bound to the route
    /// keeps the class at Extrapolated unless the signal is Fine.
    pub fn on_coarse_bound_location(&mut self, event: SystemTime) {
        if self.class != UzLocationClass::Fine {
            self.set_state(UzLocationClass::Extrapolated, event);
        }
    }

    /// Vendor `stateAt`: applies time-based transitions up to `now`.
    pub fn state_at(&mut self, now: SystemTime) -> UzLocationClass {
        loop {
            let Some(since) = self.since else {
                return self.class;
            };
            let (next, timeout) = match self.class {
                UzLocationClass::Fine => (UzLocationClass::Extrapolated, EXACT_LOCATION_TIMEOUT),
                UzLocationClass::Extrapolated => {
                    (UzLocationClass::Coarse, EXTRAPOLATED_LOCATION_TIMEOUT)
                }
                UzLocationClass::Coarse => (UzLocationClass::Outdated, COARSE_LOCATION_TIMEOUT),
                UzLocationClass::Outdated => return self.class,
            };
            let Some(next_at) = since.checked_add(timeout) else {
                return self.class;
            };
            if now >= next_at {
                self.set_state(next, next_at);
            } else {
                return self.class;
            }
        }
    }

    fn set_state(&mut self, class: UzLocationClass, timestamp: SystemTime) {
        // Vendor setState: ignore out-of-order events.
        if self.since.is_none_or(|since| timestamp >= since) {
            self.class = class;
            self.since = Some(timestamp);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(secs: u64) -> SystemTime {
        SystemTime::UNIX_EPOCH + Duration::from_secs(secs)
    }

    #[test]
    fn initial_state_is_outdated() {
        let mut sm = LocationClassState::default();
        assert_eq!(sm.state_at(at(0)), UzLocationClass::Outdated);
    }

    #[test]
    fn fine_degrades_through_vendor_timeouts() {
        let mut sm = LocationClassState::default();
        sm.on_fine_location(at(100));
        assert_eq!(sm.state_at(at(100)), UzLocationClass::Fine);
        assert_eq!(sm.state_at(at(102)), UzLocationClass::Fine);
        // EXACT_LOCATION_TIMEOUT = 3 s
        assert_eq!(sm.state_at(at(103)), UzLocationClass::Extrapolated);
        assert_eq!(sm.state_at(at(132)), UzLocationClass::Extrapolated);
        // EXTRAPOLATED_LOCATION_TIMEOUT = 30 s after entering Extrapolated
        assert_eq!(sm.state_at(at(133)), UzLocationClass::Coarse);
        assert_eq!(sm.state_at(at(312)), UzLocationClass::Coarse);
        // COARSE_LOCATION_TIMEOUT = 180 s after entering Coarse
        assert_eq!(sm.state_at(at(313)), UzLocationClass::Outdated);
    }

    #[test]
    fn coarse_signal_does_not_downgrade_fine() {
        let mut sm = LocationClassState::default();
        sm.on_fine_location(at(100));
        sm.on_coarse_location(at(101));
        assert_eq!(sm.state_at(at(101)), UzLocationClass::Fine);
    }

    #[test]
    fn coarse_bound_location_yields_extrapolated() {
        let mut sm = LocationClassState::default();
        sm.on_fine_location(at(0));
        sm.state_at(at(10)); // -> Extrapolated
        sm.on_coarse_bound_location(at(10));
        assert_eq!(sm.state_at(at(10)), UzLocationClass::Extrapolated);
        // ...but never upgrades a Fine signal.
        let mut sm2 = LocationClassState::default();
        sm2.on_fine_location(at(0));
        sm2.on_coarse_bound_location(at(1));
        assert_eq!(sm2.state_at(at(1)), UzLocationClass::Fine);
    }

    #[test]
    fn out_of_order_events_are_ignored() {
        let mut sm = LocationClassState::default();
        sm.on_fine_location(at(100));
        sm.on_fine_location(at(50));
        assert_eq!(sm.since, Some(at(100)));
    }
}
