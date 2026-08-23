//! UzNav matching core (P1a): standing detection, location classification,
//! speed filtering, heading-aware route snap and route-bound streaming.
//!
//! Ported from NavKit (`maps-product backend/mobile/libs/directions/guidance/location_guide/`)
//! into the Ferrostar downstream fork. Polyline-based (no road graph); the
//! graph HMM binder is P1b. Provenance and intentional deviations are recorded
//! in `docs/porting/ferrostar-navkit-guidance/PORTING.md`.

pub mod location_class;
#[cfg(feature = "std")]
pub mod motion;
pub mod snap;
pub mod speed_filter;
pub mod standing;

#[cfg(all(feature = "std", not(feature = "web-time")))]
use std::time::{Duration, SystemTime};
#[cfg(feature = "web-time")]
use web_time::{Duration, SystemTime};

use serde::{Deserialize, Serialize};

use crate::models::{Route, UserLocation};

pub use location_class::{LocationClassState, UzLocationClass};
#[cfg(feature = "std")]
pub use motion::{MotionPoint, OneDimensionalMotion, RouteBoundStreamer, StreamedPosition};
pub use snap::RoutePosition;
pub use speed_filter::SpeedSample;
pub use standing::StandingState;

/// Configuration for the UzNav matching core.
///
/// Defaults are the vendor constants; see each field for provenance.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "uniffi", derive(uniffi::Record))]
pub struct UzmatchConfig {
    /// Master switch. When false, the controller behaves exactly like upstream
    /// Ferrostar and [`TripState::Navigating::uzmatch`] stays `None`.
    ///
    /// [`TripState::Navigating::uzmatch`]: crate::navigation_controller::models::TripState::Navigating
    pub enabled: bool,
    /// Vendor `standing_guide_standing_speed` (0.5 m/s): speeds at or below
    /// this count as standing signals.
    pub standing_speed_threshold_mps: f64,
    /// Vendor `standing_guide_detection_period` (7000 ms): continuous
    /// standing-signal span required to report standing.
    pub standing_detection_period_ms: u64,
    /// Vendor `standing_guide_signal_interval` (5000 ms): standing history
    /// expires when no signal arrives within this interval.
    pub standing_signal_expiry_ms: u64,
    /// Vendor analyzer `GPS_POSITION_ERROR_STDDEV` (8.0 m): geometric emission sigma.
    pub snap_position_stddev_m: f64,
    /// Vendor analyzer `GPS_HEADING_ERROR_STDDEV` (6.0 deg): directional emission sigma.
    pub snap_heading_stddev_deg: f64,
    /// Vendor `IGNORE_HEADING_WHEN_SLOWER` mechanism; UzNav threshold 4.0 m/s
    /// (vendor default 0). Below this speed the heading term is ignored.
    pub snap_heading_min_speed_mps: f64,
    /// UzNav policy (no vendor equivalent extracted): a location with
    /// horizontal accuracy at or below this feeds the LCSM as a *fine* fix.
    pub fine_accuracy_threshold_m: f64,
}

impl Default for UzmatchConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            standing_speed_threshold_mps: 0.5,
            standing_detection_period_ms: 7000,
            standing_signal_expiry_ms: 5000,
            snap_position_stddev_m: 8.0,
            snap_heading_stddev_deg: 6.0,
            snap_heading_min_speed_mps: 4.0,
            fine_accuracy_threshold_m: 25.0,
        }
    }
}

/// Public snapshot of the matching core, attached to every
/// `TripState::Navigating` while uzmatch is enabled.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "uniffi", derive(uniffi::Record))]
pub struct UzmatchSnapshot {
    /// Route-global position from the heading-aware snap; `None` while the
    /// location class is not accurate (vendor does not bind coarse locations).
    pub route_position: Option<RoutePosition>,
    /// True when the user has been at/below the standing speed threshold for
    /// the detection period. Gates step advance and deviation recalculation.
    pub is_standing: bool,
    /// Current location class from the LCSM.
    pub location_class: UzLocationClass,
    /// Low-pass filtered speed in m/s (vendor `SpeedFilter`); `None` until at
    /// least two fresh samples exist.
    pub filtered_speed_mps: Option<f64>,
}

/// Internal evolving state of the matching core, packed into
/// [`NavState`](crate::navigation_controller::models::NavState) so the
/// controller remains functionally pure.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "uniffi", derive(uniffi::Record))]
pub struct UzmatchState {
    /// Standing detector state.
    pub standing: StandingState,
    /// Location class state machine state.
    pub location_class: LocationClassState,
    /// Speed filter sample window.
    pub speed_history: Vec<SpeedSample>,
    /// Latest route position (heading-aware snap).
    pub route_position: Option<RoutePosition>,
}

impl Default for UzmatchState {
    fn default() -> Self {
        Self {
            standing: StandingState::default(),
            location_class: LocationClassState::default(),
            speed_history: Vec::new(),
            route_position: None,
        }
    }
}

impl UzmatchState {
    /// Advance the matching core with a new raw location.
    ///
    /// Mirrors the vendor `LocationStreamer` signal flow:
    /// LCSM classification -> standing signals (coarse/off-route reset) ->
    /// speed filter append -> route snap (only when the class is accurate).
    pub fn update(&self, location: &UserLocation, route: &Route, config: &UzmatchConfig) -> Self {
        let timestamp = location.timestamp;
        let mut next = self.clone();

        // LCSM: fine vs coarse signal by accuracy (UzNav policy threshold).
        if location.horizontal_accuracy <= config.fine_accuracy_threshold_m {
            next.location_class.on_fine_location(timestamp);
        } else {
            next.location_class.on_coarse_location(timestamp);
        }
        let class = next.location_class.state_at(timestamp);

        // Standing: query-time expiry, then signal.
        next.standing
            .expire_if_stale(timestamp, Duration::from_millis(config.standing_signal_expiry_ms));
        if !class.is_accurate() {
            // Vendor: coarse signal resets standing history.
            next.standing.reset();
        } else {
            next.standing.on_accurate_signal(
                location.speed.map(|s| s.value),
                timestamp,
                config.standing_speed_threshold_mps,
                Duration::from_millis(config.standing_detection_period_ms),
            );
        }

        // Speed filter (vendor appends the bound location speed).
        if let Some(speed) = location.speed {
            speed_filter::append_speed(&mut next.speed_history, speed.value, timestamp);
        }

        // Heading-aware route snap; vendor does not bind coarse locations.
        next.route_position = if class.is_accurate() {
            let cum = snap::cumulative_lengths(&route.geometry);
            snap::snap_to_route(location, &route.geometry, &cum, config)
        } else {
            None
        };

        next
    }

    /// Public snapshot for `TripState::Navigating::uzmatch`.
    pub fn snapshot(&mut self, now: SystemTime) -> UzmatchSnapshot {
        UzmatchSnapshot {
            route_position: self.route_position,
            is_standing: self.standing.is_standing,
            location_class: self.location_class.state_at(now),
            filtered_speed_mps: speed_filter::filtered_speed(&mut self.speed_history, now),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::{BoundingBox, GeographicCoordinate, Speed};
    use crate::test_utils::make_user_location;
    use geo::coord;
    use std::time::Duration;

    fn at(secs: u64) -> SystemTime {
        SystemTime::UNIX_EPOCH + Duration::from_secs(secs)
    }

    fn test_route() -> Route {
        Route {
            geometry: vec![
                GeographicCoordinate { lat: 0.0, lng: 0.0 },
                GeographicCoordinate { lat: 0.0, lng: 0.001 },
            ],
            bbox: BoundingBox {
                sw: GeographicCoordinate { lat: 0.0, lng: 0.0 },
                ne: GeographicCoordinate { lat: 0.0, lng: 0.001 },
            },
            distance: 111.0,
            waypoints: vec![],
            steps: vec![],
        }
    }

    fn config() -> UzmatchConfig {
        UzmatchConfig {
            enabled: true,
            ..UzmatchConfig::default()
        }
    }

    #[test]
    fn disabled_by_default() {
        assert!(!UzmatchConfig::default().enabled);
    }

    #[test]
    fn update_produces_route_position_for_fine_location() {
        let route = test_route();
        let state = UzmatchState::default();
        let loc = UserLocation {
            timestamp: at(0),
            speed: Some(Speed {
                value: 10.0,
                accuracy: None,
            }),
            ..make_user_location(coord!(x: 0.0005, y: 0.0001), 5.0)
        };
        let mut next = state.update(&loc, &route, &config());
        let snapshot = next.snapshot(at(0));
        assert_eq!(snapshot.location_class, UzLocationClass::Fine);
        let position = snapshot.route_position.expect("fine location must snap");
        assert_eq!(position.segment_index, 0);
        assert!(!snapshot.is_standing);
    }

    #[test]
    fn coarse_location_is_not_bound_to_route() {
        let route = test_route();
        let state = UzmatchState::default();
        // Accuracy 100 m > threshold -> coarse signal.
        let loc = UserLocation {
            timestamp: at(0),
            ..make_user_location(coord!(x: 0.0005, y: 0.0001), 100.0)
        };
        let mut next = state.update(&loc, &route, &config());
        let snapshot = next.snapshot(at(0));
        assert_eq!(snapshot.location_class, UzLocationClass::Coarse);
        assert_eq!(snapshot.route_position, None);
    }

    #[test]
    fn standing_detected_and_gates_nothing_by_itself() {
        let route = test_route();
        let config = config();
        let mut state = UzmatchState::default();
        for t in 0..=7 {
            let loc = UserLocation {
                timestamp: at(t),
                speed: Some(Speed {
                    value: 0.0,
                    accuracy: None,
                }),
                ..make_user_location(coord!(x: 0.0005, y: 0.0001), 5.0)
            };
            state = state.update(&loc, &route, &config);
        }
        let mut state = state;
        let snapshot = state.snapshot(at(7));
        assert!(snapshot.is_standing);
        assert_eq!(snapshot.location_class, UzLocationClass::Fine);
    }

    #[test]
    fn filtered_speed_requires_two_samples() {
        let route = test_route();
        let config = config();
        let state = UzmatchState::default();
        let loc = UserLocation {
            timestamp: at(0),
            speed: Some(Speed {
                value: 9.0,
                accuracy: None,
            }),
            ..make_user_location(coord!(x: 0.0005, y: 0.0), 5.0)
        };
        let mut state = state.update(&loc, &route, &config);
        assert_eq!(state.snapshot(at(0)).filtered_speed_mps, None);
        let loc2 = UserLocation {
            timestamp: at(1),
            ..loc
        };
        let mut state = state.update(&loc2, &route, &config);
        let speed = state.snapshot(at(1)).filtered_speed_mps.unwrap();
        assert!((speed - 9.0).abs() < 1e-9);
    }
}
