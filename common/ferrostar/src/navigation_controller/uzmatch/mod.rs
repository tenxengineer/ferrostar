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
pub mod temporal;

#[cfg(test)]
mod evidence;

#[cfg(all(feature = "std", not(feature = "web-time")))]
use std::time::{Duration, SystemTime};
#[cfg(feature = "web-time")]
use web_time::{Duration, SystemTime};

use serde::{Deserialize, Serialize};

use crate::models::{Route, UserLocation};
use geo::{Distance, Haversine, Point};

pub use location_class::{LocationClassState, UzLocationClass};
#[cfg(feature = "std")]
pub use motion::{MotionPoint, OneDimensionalMotion, RouteBoundStreamer, StreamedPosition};
pub use snap::RoutePosition;
pub(crate) use snap::RouteSnapIndex;
pub use speed_filter::SpeedSample;
pub use standing::StandingState;
pub use temporal::{TemporalCandidate, TemporalMatchState};

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
    /// Vendor `Clinger` cling radius (guidance config `CLING_DISTANCE`,
    /// 33.25 m): a full route loss is held back until the fix is at least this
    /// far from the last on-route anchor. The vendor ships one automotive
    /// value; the pedestrian profile (UzNav's own number) lowers it.
    #[serde(default = "default_cling_distance_m")]
    #[cfg_attr(feature = "uniffi", uniffi(default = 33.25))]
    pub cling_distance_m: f64,
    /// Vendor `Clinger` cling window (guidance config `CLING_TIME`, 2000 ms).
    #[serde(default = "default_cling_time_ms")]
    #[cfg_attr(feature = "uniffi", uniffi(default = 2000))]
    pub cling_time_ms: u64,
    /// `UzNav` pedestrian route-loss detector (no vendor equivalent: the
    /// vendor's pedestrian guidance is a separate product outside the tree).
    /// When on, a walker whose credible course diverges from the route's
    /// forward direction at a route vertex for
    /// `heading_departure_confirmations` consecutive moving fixes is
    /// published as completely off route without waiting for the distance
    /// threshold or the cling radius.
    #[serde(default)]
    #[cfg_attr(feature = "uniffi", uniffi(default = false))]
    pub heading_departure_enabled: bool,
    /// Fixes slower than this (m/s) never count as a heading departure and
    /// reset the confirmation streak.
    #[serde(default = "default_heading_departure_min_speed_mps")]
    #[cfg_attr(feature = "uniffi", uniffi(default = 0.8))]
    pub heading_departure_min_speed_mps: f64,
    /// Course-vs-route divergence (degrees) above which a fix counts. A course
    /// whose reported accuracy is worse than this is treated as no course.
    #[serde(default = "default_heading_departure_tolerance_deg")]
    #[cfg_attr(feature = "uniffi", uniffi(default = 60.0))]
    pub heading_departure_tolerance_deg: f64,
    /// Consecutive diverging moving fixes required before publishing.
    #[serde(default = "default_heading_departure_confirmations")]
    #[cfg_attr(feature = "uniffi", uniffi(default = 3))]
    pub heading_departure_confirmations: u8,
}

fn default_cling_distance_m() -> f64 {
    CLING_DISTANCE_METERS
}

fn default_cling_time_ms() -> u64 {
    CLING_TIME_MS
}

fn default_heading_departure_min_speed_mps() -> f64 {
    0.8
}

fn default_heading_departure_tolerance_deg() -> f64 {
    60.0
}

fn default_heading_departure_confirmations() -> u8 {
    3
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
            cling_distance_m: default_cling_distance_m(),
            cling_time_ms: default_cling_time_ms(),
            heading_departure_enabled: false,
            heading_departure_min_speed_mps: default_heading_departure_min_speed_mps(),
            heading_departure_tolerance_deg: default_heading_departure_tolerance_deg(),
            heading_departure_confirmations: default_heading_departure_confirmations(),
        }
    }
}

/// Public snapshot of the matching core, attached to every
/// `TripState::Navigating` while uzmatch is enabled.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "uniffi", derive(uniffi::Record))]
pub struct UzmatchSnapshot {
    /// Route-global position from the heading-aware snap; `None` when the
    /// current raw fix is not eligible for downstream snapping. The persistent
    /// location class can remain Fine briefly after a coarse fix.
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
    /// Bounded candidate frontier used to preserve route continuity.
    #[serde(default)]
    #[cfg_attr(feature = "uniffi", uniffi(default))]
    pub temporal_match: TemporalMatchState,
    /// Route-cling state (vendor `Clinger`): route loss is stabilized, not per-fix.
    #[serde(default)]
    #[cfg_attr(feature = "uniffi", uniffi(default))]
    pub cling: ClingState,
    /// Consecutive moving fixes whose credible course diverged from the
    /// route's forward direction at a vertex (`UzNav` heading-departure
    /// detector). Saturates once the configured confirmations are reached.
    #[serde(default)]
    #[cfg_attr(feature = "uniffi", uniffi(default))]
    pub heading_departure_streak: u8,
}

/// Vendor `Clinger` cling window (guidance config `CLING_TIME`). Documented
/// default of [`UzmatchConfig::cling_time_ms`]; the controller reads the config.
pub(crate) const CLING_TIME_MS: u64 = 2000;
/// Vendor `Clinger` cling radius in meters (guidance config `CLING_DISTANCE`).
/// Documented default of [`UzmatchConfig::cling_distance_m`].
pub(crate) const CLING_DISTANCE_METERS: f64 = 33.25;

/// Last position and time the user was accepted as on-route, used to stabilize
/// route loss the way the vendor `Clinger` does: a full off-route deviation is
/// published only once the signal is far enough from this anchor in BOTH time
/// (`cling_time_ms`) and distance (`cling_distance_m`). Until then the user
/// keeps clinging to the route, so one or two bad urban-canyon fixes cannot
/// start a reroute.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
#[cfg_attr(feature = "uniffi", derive(uniffi::Record))]
pub struct ClingState {
    /// Coordinates of the last on-route acceptance (snapped when bound, else raw).
    #[serde(default)]
    #[cfg_attr(feature = "uniffi", uniffi(default))]
    pub anchor: Option<crate::models::GeographicCoordinate>,
    /// Timestamp of the last on-route acceptance.
    #[serde(default)]
    #[cfg_attr(feature = "uniffi", uniffi(default))]
    pub anchored_at: Option<SystemTime>,
}

impl ClingState {
    /// Record an on-route acceptance (any non-full deviation outcome).
    pub(crate) fn anchor_on_route(
        &mut self,
        coordinates: crate::models::GeographicCoordinate,
        at: SystemTime,
    ) {
        self.anchor = Some(coordinates);
        self.anchored_at = Some(at);
    }

    /// Whether a freshly computed full off-route deviation must still be held
    /// back. Vendor `Clinger::isFarEnough`: releasing requires BOTH the time
    /// and the distance thresholds to be exceeded. Once released the anchor is
    /// cleared so a later deviation after a reroute starts fresh.
    pub(crate) fn holds(&mut self, location: &UserLocation, config: &UzmatchConfig) -> bool {
        let (Some(anchor), Some(anchored_at)) = (self.anchor, self.anchored_at) else {
            return false;
        };
        let cling_time = Duration::from_millis(config.cling_time_ms);
        let time_elapsed = location
            .timestamp
            .duration_since(anchored_at)
            .map(|elapsed| elapsed >= cling_time)
            .unwrap_or(true);
        let distance = Haversine.distance(Point::from(anchor), Point::from(location.coordinates));
        if time_elapsed && distance >= config.cling_distance_m {
            self.anchor = None;
            self.anchored_at = None;
            false
        } else {
            true
        }
    }
}

impl Default for UzmatchState {
    fn default() -> Self {
        Self {
            standing: StandingState::default(),
            location_class: LocationClassState::default(),
            speed_history: Vec::new(),
            route_position: None,
            temporal_match: TemporalMatchState::default(),
            cling: ClingState::default(),
            heading_departure_streak: 0,
        }
    }
}

impl UzmatchState {
    fn copy_for_update(&self) -> Self {
        let temporal_match = if self.temporal_match.candidates.len() <= snap::MAX_SNAP_CANDIDATES {
            self.temporal_match.clone()
        } else {
            TemporalMatchState::default()
        };
        Self {
            standing: self.standing,
            location_class: self.location_class,
            speed_history: self.speed_history.clone(),
            route_position: self.route_position,
            temporal_match,
            cling: self.cling,
            heading_departure_streak: self.heading_departure_streak,
        }
    }

    /// Advance the matching core with a new raw location.
    ///
    /// Mirrors the vendor signal separation:
    /// LCSM classification -> standing signals (raw coarse/off-route reset) ->
    /// speed filter append -> route snap (only for an eligible raw fix).
    pub fn update(&self, location: &UserLocation, route: &Route, config: &UzmatchConfig) -> Self {
        let route_index = RouteSnapIndex::new(&route.geometry);
        self.update_with_index(location, &route_index, config)
    }

    pub(crate) fn update_with_index(
        &self,
        location: &UserLocation,
        route_index: &RouteSnapIndex,
        config: &UzmatchConfig,
    ) -> Self {
        let timestamp = location.timestamp;
        // Reject an oversized external temporal frontier before copying it.
        let mut next = self.copy_for_update();

        // Keep the raw signal quality separate from the persistent vendor
        // location class. `on_coarse_location` intentionally preserves a
        // recent Fine/Extrapolated class, but that must not make the raw coarse
        // fix eligible for downstream route snapping.
        let is_fine_signal = location.horizontal_accuracy.is_finite()
            && location.horizontal_accuracy >= 0.0
            && location.horizontal_accuracy <= config.fine_accuracy_threshold_m;
        if is_fine_signal {
            next.location_class.on_fine_location(timestamp);
        } else {
            next.location_class.on_coarse_location(timestamp);
        }
        next.location_class.state_at(timestamp);

        // Standing: query-time expiry, then signal.
        next.standing.expire_if_stale(
            timestamp,
            Duration::from_millis(config.standing_signal_expiry_ms),
        );
        if is_fine_signal {
            next.standing.on_accurate_signal(
                location.speed.map(|s| s.value),
                timestamp,
                config.standing_speed_threshold_mps,
                Duration::from_millis(config.standing_detection_period_ms),
            );
        } else {
            // Vendor: coarse signal resets standing history.
            next.standing.reset();
        }

        // Speed filter (vendor appends the bound location speed).
        if let Some(speed) = location.speed {
            speed_filter::append_speed(&mut next.speed_history, speed.value, timestamp);
        }

        // Heading-aware route snap. The public downstream contract keeps raw
        // coarse fixes unsnapped even while the persistent class is still Fine.
        next.route_position = if is_fine_signal {
            let candidate_anchor = next.temporal_match.candidate_anchor(location);
            let candidates = route_index.candidates_with_anchor(location, config, candidate_anchor);
            let (mut temporal_match, continued_previous_frontier) = next
                .temporal_match
                .update_with_status(location, &candidates, config);
            if candidate_anchor.is_some() && !continued_previous_frontier {
                let frame_local = route_index.candidates_with_anchor(location, config, None);
                temporal_match =
                    TemporalMatchState::default().update(location, &frame_local, config);
            }
            next.temporal_match = temporal_match;
            next.temporal_match.route_position()
        } else {
            next.temporal_match = TemporalMatchState::default();
            None
        };
        if is_fine_signal && next.route_position.is_none() {
            // Vendor `onOffRouteSignal`: a fine fix outside the route-binding
            // bias invalidates standing history just like a coarse signal.
            next.standing.reset();
        }

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
                GeographicCoordinate {
                    lat: 0.0,
                    lng: 0.001,
                },
            ],
            bbox: BoundingBox {
                sw: GeographicCoordinate { lat: 0.0, lng: 0.0 },
                ne: GeographicCoordinate {
                    lat: 0.0,
                    lng: 0.001,
                },
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
    fn serialized_pre_temporal_state_uses_an_empty_frontier() {
        let mut value = serde_json::to_value(UzmatchState::default()).unwrap();
        value.as_object_mut().unwrap().remove("temporal_match");

        let restored: UzmatchState = serde_json::from_value(value).unwrap();
        assert_eq!(restored, UzmatchState::default());
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
    fn temporal_match_stays_on_the_entered_branch_at_a_self_crossing() {
        let route = Route {
            geometry: vec![
                GeographicCoordinate {
                    lat: 0.0,
                    lng: -0.001,
                },
                GeographicCoordinate {
                    lat: 0.0,
                    lng: 0.001,
                },
                GeographicCoordinate {
                    lat: 0.001,
                    lng: 0.001,
                },
                GeographicCoordinate {
                    lat: -0.001,
                    lng: -0.001,
                },
            ],
            bbox: BoundingBox {
                sw: GeographicCoordinate {
                    lat: -0.001,
                    lng: -0.001,
                },
                ne: GeographicCoordinate {
                    lat: 0.001,
                    lng: 0.001,
                },
            },
            distance: 0.0,
            waypoints: vec![],
            steps: vec![],
        };
        let config = config();
        let route_index = RouteSnapIndex::new(&route.geometry);
        let moving_slowly = |seconds: u64, lng: f64, lat: f64| UserLocation {
            timestamp: at(seconds),
            coordinates: GeographicCoordinate { lat, lng },
            horizontal_accuracy: 5.0,
            speed: Some(Speed {
                value: 2.0,
                accuracy: None,
            }),
            course_over_ground: None,
        };

        let before_crossing = moving_slowly(0, 0.0002, 0.0002);
        let at_crossing = moving_slowly(1, 0.0, 0.0);
        let frame_local = route_index
            .snap_to_route(&at_crossing, &config)
            .expect("crossing must produce a frame-local candidate");
        assert_eq!(
            frame_local.segment_index, 0,
            "route-order tie break demonstrates the frame-local ambiguity"
        );

        let state =
            UzmatchState::default().update_with_index(&before_crossing, &route_index, &config);
        assert_eq!(
            state.route_position.map(|position| position.segment_index),
            Some(2)
        );

        let state = state.update_with_index(&at_crossing, &route_index, &config);
        assert_eq!(
            state.route_position.map(|position| position.segment_index),
            Some(2),
            "temporal continuity must keep the branch already being travelled"
        );
    }

    #[test]
    fn temporal_match_preserves_previous_branch_when_more_than_ten_segments_overlap() {
        let a = GeographicCoordinate {
            lat: 0.0,
            lng: -0.001,
        };
        let b = GeographicCoordinate {
            lat: 0.0,
            lng: 0.001,
        };
        let route = Route {
            geometry: (0..=20)
                .map(|index| if index % 2 == 0 { a } else { b })
                .collect(),
            bbox: BoundingBox { sw: a, ne: b },
            distance: 0.0,
            waypoints: vec![],
            steps: vec![],
        };
        let previous_location = UserLocation {
            timestamp: at(0),
            coordinates: GeographicCoordinate { lat: 0.0, lng: 0.0 },
            horizontal_accuracy: 5.0,
            speed: Some(Speed {
                value: 10.0,
                accuracy: None,
            }),
            course_over_ground: None,
        };
        let previous_route_position = RoutePosition {
            segment_index: 15,
            segment_offset_meters: 111.0,
            distance_along_route_meters: 3_447.0,
            coordinates: GeographicCoordinate { lat: 0.0, lng: 0.0 },
            course_over_ground: None,
        };
        let state = UzmatchState {
            route_position: Some(previous_route_position),
            temporal_match: TemporalMatchState {
                candidates: vec![TemporalCandidate {
                    route_position: previous_route_position,
                    accumulated_log_likelihood: 0.0,
                }],
                previous_location: Some(previous_location),
            },
            ..UzmatchState::default()
        };
        let current_location = UserLocation {
            timestamp: at(1),
            ..previous_location
        };

        let route_index = RouteSnapIndex::new(&route.geometry);
        let frame_local = route_index.candidates(&current_location, &config());
        assert_eq!(
            frame_local
                .first()
                .map(|candidate| candidate.route_position.segment_index),
            Some(0)
        );
        assert!(
            frame_local
                .iter()
                .all(|candidate| candidate.route_position.segment_index != 15),
            "the regression must demonstrate that frame-local top-K drops the established branch"
        );
        let anchored = route_index.candidates_with_anchor(
            &current_location,
            &config(),
            Some(previous_route_position),
        );
        assert!(anchored.len() <= snap::MAX_SNAP_CANDIDATES * 2);
        assert!(
            anchored
                .iter()
                .any(|candidate| candidate.route_position.segment_index == 15)
        );

        let state = state.update(&current_location, &route, &config());

        assert_eq!(
            state.route_position.map(|position| position.segment_index),
            Some(15),
            "the current projection of the previously selected branch must survive candidate truncation"
        );
    }

    #[test]
    fn unreachable_anchored_candidates_reset_to_frame_local_emissions() {
        let a = GeographicCoordinate {
            lat: 0.0,
            lng: -0.001,
        };
        let b = GeographicCoordinate {
            lat: 0.0,
            lng: 0.001,
        };
        let previous_coordinates = GeographicCoordinate { lat: 0.0, lng: 0.1 };
        let mut geometry = (0..=30)
            .map(|index| if index % 2 == 0 { a } else { b })
            .collect::<Vec<_>>();
        geometry.push(previous_coordinates);
        let route = Route {
            geometry,
            bbox: BoundingBox {
                sw: a,
                ne: previous_coordinates,
            },
            distance: 0.0,
            waypoints: vec![],
            steps: vec![],
        };
        let previous_location = UserLocation {
            timestamp: at(0),
            coordinates: previous_coordinates,
            horizontal_accuracy: 5.0,
            speed: Some(Speed {
                value: 10.0,
                accuracy: None,
            }),
            course_over_ground: None,
        };
        let previous_route_position = RoutePosition {
            segment_index: 30,
            segment_offset_meters: 11_000.0,
            distance_along_route_meters: 100_000.0,
            coordinates: previous_coordinates,
            course_over_ground: None,
        };
        let state = UzmatchState {
            route_position: Some(previous_route_position),
            temporal_match: TemporalMatchState {
                candidates: vec![TemporalCandidate {
                    route_position: previous_route_position,
                    accumulated_log_likelihood: 0.0,
                }],
                previous_location: Some(previous_location),
            },
            ..UzmatchState::default()
        };
        let current_location = UserLocation {
            timestamp: at(1),
            coordinates: GeographicCoordinate { lat: 0.0, lng: 0.0 },
            ..previous_location
        };

        let state = state.update(&current_location, &route, &config());

        assert_eq!(
            state.route_position.map(|position| position.segment_index),
            Some(0),
            "an unreachable anchored layer must restart from the strongest frame-local emissions"
        );
    }

    #[test]
    fn update_copy_rejects_oversized_temporal_frontier_before_clone() {
        let mut state = UzmatchState::default();
        state.temporal_match.candidates = (0..=snap::MAX_SNAP_CANDIDATES)
            .map(|segment_index| TemporalCandidate {
                route_position: RoutePosition {
                    segment_index: segment_index as u64,
                    segment_offset_meters: 0.0,
                    distance_along_route_meters: segment_index as f64,
                    coordinates: GeographicCoordinate { lat: 0.0, lng: 0.0 },
                    course_over_ground: None,
                },
                accumulated_log_likelihood: 0.0,
            })
            .collect();

        let copied = state.copy_for_update();

        assert_eq!(copied.temporal_match, TemporalMatchState::default());
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
        assert!(next.temporal_match.candidates.is_empty());
    }

    #[test]
    fn coarse_fix_after_fine_is_not_published_as_a_route_match() {
        let route = test_route();
        let config = config();
        let mut state = UzmatchState::default();
        for timestamp in 0..=7 {
            let fine = UserLocation {
                timestamp: at(timestamp),
                speed: Some(Speed {
                    value: 0.0,
                    accuracy: None,
                }),
                ..make_user_location(coord!(x: 0.0005, y: 0.0001), 5.0)
            };
            state = state.update(&fine, &route, &config);
        }
        assert!(state.route_position.is_some());
        assert!(state.standing.is_standing);

        let coarse = UserLocation {
            timestamp: at(8),
            ..make_user_location(coord!(x: 0.0005, y: 0.0001), 100.0)
        };

        let mut state = state.update(&coarse, &route, &config);
        let snapshot = state.snapshot(at(8));

        assert_eq!(
            snapshot.location_class,
            UzLocationClass::Fine,
            "vendor LCSM keeps the last fine class until its timeout"
        );
        assert_eq!(
            snapshot.route_position, None,
            "raw coarse fixes must not become downstream snapped positions"
        );
        assert!(state.temporal_match.candidates.is_empty());
        assert!(!snapshot.is_standing);
    }

    #[test]
    fn invalid_accuracy_fixes_are_not_published_as_route_matches() {
        let route = test_route();
        let config = config();

        for horizontal_accuracy in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -1.0] {
            let location = UserLocation {
                timestamp: at(0),
                ..make_user_location(coord!(x: 0.0005, y: 0.0001), horizontal_accuracy)
            };

            let state = UzmatchState::default().update(&location, &route, &config);

            assert_eq!(
                state.route_position, None,
                "invalid horizontal accuracy {horizontal_accuracy} must fail closed"
            );
            assert!(state.temporal_match.candidates.is_empty());
        }
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
    fn fine_off_route_fix_resets_previously_detected_standing() {
        let route = test_route();
        let config = config();
        let mut state = UzmatchState::default();
        for timestamp in 0..=7 {
            let location = UserLocation {
                timestamp: at(timestamp),
                speed: Some(Speed {
                    value: 0.0,
                    accuracy: None,
                }),
                ..make_user_location(coord!(x: 0.0005, y: 0.0001), 5.0)
            };
            state = state.update(&location, &route, &config);
        }
        assert!(state.standing.is_standing);

        let off_route = UserLocation {
            timestamp: at(8),
            speed: Some(Speed {
                value: 0.0,
                accuracy: None,
            }),
            ..make_user_location(coord!(x: 0.0005, y: 0.01), 5.0)
        };
        let mut state = state.update(&off_route, &route, &config);
        let snapshot = state.snapshot(at(8));

        assert_eq!(snapshot.route_position, None);
        assert!(!snapshot.is_standing);
        assert_eq!(state.standing.oldest_signal, None);
        assert_eq!(state.standing.latest_signal, None);
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
