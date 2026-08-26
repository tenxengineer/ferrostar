//! Bounded temporal route matching over frame-local snap candidates.
//!
//! This ports the central `NaviKit` binder invariant: a candidate is selected by
//! accumulated emission and transition likelihood, not by the current `GPS` fix
//! alone. `UzNav` currently has route geometry but no road graph, so transitions
//! use forward distance along the route polyline.

#[cfg(all(feature = "std", not(feature = "web-time")))]
use std::time::Duration;
#[cfg(feature = "web-time")]
use web_time::Duration;

use geo::{Coord, Distance, Haversine, Point};
use serde::{Deserialize, Serialize};

use crate::models::UserLocation;

use super::{RoutePosition, UzmatchConfig, snap::SnapCandidate};

/// Vendor `MAXIMAL_POSSIBLE_SPEED` (200 km/h), in meters per second.
const MAXIMAL_POSSIBLE_SPEED_MPS: f64 = 200.0 / 3.6;
/// Vendor `MINIMUM_RELIABLE_INTERVAL` used by the path-length envelope.
const MINIMUM_RELIABLE_INTERVAL: Duration = Duration::from_secs(2);
/// Bounded continuity window. A longer gap starts a new candidate sequence.
const MAX_SIGNAL_GAP: Duration = Duration::from_secs(5);
/// Vendor spatial-transition mean error (`0.05 * 40` meters).
const PATH_LENGTH_DIFFERENCE_MEAN_M: f64 = 2.0;
/// Vendor jump-likelihood normalization distance.
const JUMP_NORMALIZATION_M: f64 = 1.0;
/// Vendor lower probability bound for a discontinuous snap offset.
const JUMP_MIN_LIKELIHOOD: f64 = 0.000_130_031_437_976;
/// Vendor speed above which the jump penalty is fully applied.
const JUMP_GOOD_SPEED_MPS: f64 = 9.0;
/// Vendor accuracy at or below which the base normalization is used.
const JUMP_GOOD_ACCURACY_M: f64 = 6.5;

/// A candidate retained between GPS signals.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "uniffi", derive(uniffi::Record))]
pub struct TemporalCandidate {
    pub route_position: RoutePosition,
    /// Accumulated log likelihood, normalized so the frontier maximum is zero.
    pub accumulated_log_likelihood: f64,
}

/// The bounded Viterbi frontier for the most recent accurate signal.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
#[cfg_attr(feature = "uniffi", derive(uniffi::Record))]
pub struct TemporalMatchState {
    #[cfg_attr(feature = "uniffi", uniffi(default))]
    pub candidates: Vec<TemporalCandidate>,
    #[cfg_attr(feature = "uniffi", uniffi(default))]
    pub previous_location: Option<UserLocation>,
}

impl TemporalMatchState {
    pub(crate) fn candidate_anchor(&self, location: &UserLocation) -> Option<RoutePosition> {
        if self.candidates.len() > super::snap::MAX_SNAP_CANDIDATES
            || self.usable_interval(location).is_none()
        {
            return None;
        }

        self.candidates
            .iter()
            .max_by(|left, right| {
                left.accumulated_log_likelihood
                    .total_cmp(&right.accumulated_log_likelihood)
                    .then_with(|| {
                        right
                            .route_position
                            .segment_index
                            .cmp(&left.route_position.segment_index)
                    })
            })
            .map(|candidate| candidate.route_position)
    }

    pub(crate) fn update(
        &self,
        location: &UserLocation,
        emissions: &[SnapCandidate],
        config: &UzmatchConfig,
    ) -> Self {
        self.update_with_status(location, emissions, config).0
    }

    pub(crate) fn update_with_status(
        &self,
        location: &UserLocation,
        emissions: &[SnapCandidate],
        config: &UzmatchConfig,
    ) -> (Self, bool) {
        if emissions.is_empty() {
            return (Self::default(), false);
        }

        let usable_interval = self.usable_interval(location);

        // State can cross serialization and FFI boundaries. Reject an
        // oversized frontier before cloning or sorting it instead of trusting
        // every producer to preserve the bound. Valid anchored work stays at
        // most 2K x K; invalid state fails closed by seeding from current
        // emissions.
        let previous_candidates = if self.candidates.len() <= super::snap::MAX_SNAP_CANDIDATES {
            normalize_and_sort(self.candidates.clone())
        } else {
            Vec::new()
        };
        let (candidates, continued_previous_frontier) =
            match usable_interval.filter(|_| !previous_candidates.is_empty()) {
                Some((previous_location, interval)) => {
                    let transitioned = transition_frontier(
                        &previous_candidates,
                        &previous_location,
                        location,
                        interval,
                        emissions,
                        config,
                    );
                    if transitioned.is_empty() {
                        (seed_frontier(emissions), false)
                    } else {
                        (transitioned, true)
                    }
                }
                None => (seed_frontier(emissions), false),
            };

        (
            Self {
                candidates,
                previous_location: Some(*location),
            },
            continued_previous_frontier,
        )
    }

    pub(crate) fn route_position(&self) -> Option<RoutePosition> {
        self.candidates
            .first()
            .map(|candidate| candidate.route_position)
    }

    fn usable_interval(&self, location: &UserLocation) -> Option<(UserLocation, Duration)> {
        self.previous_location.and_then(|previous| {
            location
                .timestamp
                .duration_since(previous.timestamp)
                .ok()
                .filter(|interval| !interval.is_zero() && *interval <= MAX_SIGNAL_GAP)
                .map(|interval| (previous, interval))
        })
    }
}

fn seed_frontier(emissions: &[SnapCandidate]) -> Vec<TemporalCandidate> {
    normalize_and_sort(
        emissions
            .iter()
            .map(|candidate| TemporalCandidate {
                route_position: candidate.route_position,
                accumulated_log_likelihood: candidate.emission_log_likelihood,
            })
            .collect(),
    )
}

fn transition_frontier(
    previous_candidates: &[TemporalCandidate],
    previous_location: &UserLocation,
    location: &UserLocation,
    interval: Duration,
    emissions: &[SnapCandidate],
    config: &UzmatchConfig,
) -> Vec<TemporalCandidate> {
    let mut next = Vec::with_capacity(emissions.len());

    for emission in emissions {
        let best_score = previous_candidates
            .iter()
            .filter_map(|previous| {
                transition_log_likelihood(
                    previous.route_position,
                    emission.route_position,
                    previous_location,
                    location,
                    interval,
                    config,
                )
                .map(|transition| {
                    previous.accumulated_log_likelihood
                        + transition
                        + emission.emission_log_likelihood
                })
            })
            .max_by(f64::total_cmp);

        if let Some(accumulated_log_likelihood) = best_score {
            next.push(TemporalCandidate {
                route_position: emission.route_position,
                accumulated_log_likelihood,
            });
        }
    }

    normalize_and_sort(next)
}

fn transition_log_likelihood(
    previous: RoutePosition,
    current: RoutePosition,
    previous_location: &UserLocation,
    location: &UserLocation,
    interval: Duration,
    config: &UzmatchConfig,
) -> Option<f64> {
    let route_delta = current.distance_along_route_meters - previous.distance_along_route_meters;
    if !route_delta.is_finite() || route_delta < -config.snap_position_stddev_m {
        return None;
    }

    let path_distance = route_delta.max(0.0);
    let reliable_interval = interval.max(MINIMUM_RELIABLE_INTERVAL);
    let max_path_length = MAXIMAL_POSSIBLE_SPEED_MPS * 2.0 * reliable_interval.as_secs_f64();
    if path_distance > max_path_length {
        return None;
    }

    let raw_distance = Haversine.distance(
        Point::from(Coord::from(previous_location.coordinates)),
        Point::from(Coord::from(location.coordinates)),
    );
    if !raw_distance.is_finite() {
        return None;
    }
    let difference = (raw_distance - path_distance).abs();
    let jump_log_likelihood =
        jump_offset_log_likelihood(previous, current, previous_location, location)?;

    // The omitted `-ln(mean)` is constant for every transition in this layer
    // and therefore cannot change candidate ordering.
    Some(-difference / PATH_LENGTH_DIFFERENCE_MEAN_M + jump_log_likelihood)
}

fn jump_offset_log_likelihood(
    previous: RoutePosition,
    current: RoutePosition,
    previous_location: &UserLocation,
    location: &UserLocation,
) -> Option<f64> {
    let previous_offset = Coord {
        x: previous.coordinates.lng - previous_location.coordinates.lng,
        y: previous.coordinates.lat - previous_location.coordinates.lat,
    };
    let current_offset = Coord {
        x: current.coordinates.lng - location.coordinates.lng,
        y: current.coordinates.lat - location.coordinates.lat,
    };
    let offset_change_endpoint = Point::new(
        previous_location.coordinates.lng + previous_offset.x - current_offset.x,
        previous_location.coordinates.lat + previous_offset.y - current_offset.y,
    );
    let offset_change_m = Haversine.distance(
        Point::from(Coord::from(previous_location.coordinates)),
        offset_change_endpoint,
    );
    if !offset_change_m.is_finite() {
        return None;
    }

    if previous_location
        .speed
        .is_some_and(|speed| !speed.value.is_finite())
        || location.speed.is_some_and(|speed| !speed.value.is_finite())
    {
        return None;
    }
    let average_speed_mps = previous_location
        .speed
        .zip(location.speed)
        .map_or(0.0, |(previous, current)| {
            (previous.value + current.value).abs() / 2.0
        });
    if !previous_location.horizontal_accuracy.is_finite()
        || !location.horizontal_accuracy.is_finite()
        || !average_speed_mps.is_finite()
    {
        return None;
    }
    let accuracy_m = previous_location
        .horizontal_accuracy
        .max(location.horizontal_accuracy);
    let normalization_m = if accuracy_m > JUMP_GOOD_ACCURACY_M {
        JUMP_NORMALIZATION_M * accuracy_m / JUMP_GOOD_ACCURACY_M
    } else {
        JUMP_NORMALIZATION_M
    };
    let base = (-offset_change_m / normalization_m).exp();
    let corrected = if average_speed_mps < JUMP_GOOD_SPEED_MPS {
        base.powf(average_speed_mps / JUMP_GOOD_SPEED_MPS)
    } else {
        base
    };
    let likelihood = (1.0 - JUMP_MIN_LIKELIHOOD) * corrected + JUMP_MIN_LIKELIHOOD;
    likelihood.is_finite().then(|| likelihood.ln())
}

fn normalize_and_sort(mut candidates: Vec<TemporalCandidate>) -> Vec<TemporalCandidate> {
    candidates.sort_by(|left, right| {
        right
            .accumulated_log_likelihood
            .total_cmp(&left.accumulated_log_likelihood)
            .then_with(|| {
                left.route_position
                    .segment_index
                    .cmp(&right.route_position.segment_index)
            })
    });
    candidates.truncate(super::snap::MAX_SNAP_CANDIDATES);

    if let Some(maximum) = candidates
        .first()
        .map(|candidate| candidate.accumulated_log_likelihood)
    {
        for candidate in &mut candidates {
            candidate.accumulated_log_likelihood -= maximum;
        }
    }

    candidates
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::{GeographicCoordinate, Speed};
    use std::time::SystemTime;

    fn at(seconds: u64) -> SystemTime {
        SystemTime::UNIX_EPOCH + Duration::from_secs(seconds)
    }

    fn location(seconds: u64, longitude: f64) -> UserLocation {
        UserLocation {
            coordinates: GeographicCoordinate {
                lat: 0.0,
                lng: longitude,
            },
            horizontal_accuracy: 5.0,
            course_over_ground: None,
            timestamp: at(seconds),
            speed: Some(Speed {
                value: 10.0,
                accuracy: None,
            }),
        }
    }

    fn emission(distance: f64, segment: u64, score: f64) -> SnapCandidate {
        SnapCandidate {
            route_position: RoutePosition {
                segment_index: segment,
                segment_offset_meters: 0.0,
                distance_along_route_meters: distance,
                coordinates: GeographicCoordinate {
                    lat: 0.0,
                    lng: distance / 111_000.0,
                },
                course_over_ground: None,
            },
            emission_log_likelihood: score,
        }
    }

    fn emission_at(
        distance: f64,
        segment: u64,
        score: f64,
        longitude: f64,
        latitude: f64,
    ) -> SnapCandidate {
        let mut candidate = emission(distance, segment, score);
        candidate.route_position.coordinates = GeographicCoordinate {
            lat: latitude,
            lng: longitude,
        };
        candidate
    }

    #[test]
    fn continuity_beats_frame_local_winner_at_self_crossing() {
        let config = UzmatchConfig::default();
        let first = location(0, 100.0 / 111_000.0);
        let state = TemporalMatchState::default().update(
            &first,
            &[emission(100.0, 1, 0.0), emission(1_000.0, 9, -1.0)],
            &config,
        );
        let second = location(1, 110.0 / 111_000.0);
        let state = state.update(
            &second,
            &[emission(1_000.0, 9, 0.0), emission(110.0, 1, -1.0)],
            &config,
        );

        assert_eq!(
            state
                .route_position()
                .map(|position| position.segment_index),
            Some(1)
        );
    }

    #[test]
    fn stable_snap_offset_beats_a_stronger_frame_local_jump() {
        let config = UzmatchConfig::default();
        let previous = location(0, 0.0);
        let state = TemporalMatchState::default().update(
            &previous,
            &[emission_at(100.0, 0, 0.0, 0.0, 0.000_1)],
            &config,
        );
        let current = location(1, 10.0 / 111_000.0);
        let stable_offset = emission_at(110.0, 1, -1.0, current.coordinates.lng, 0.000_1);
        let offset_jump = emission_at(110.0, 2, 0.0, current.coordinates.lng, -0.000_1);

        let next = state.update(&current, &[offset_jump, stable_offset], &config);

        assert_eq!(
            next.route_position().map(|position| position.segment_index),
            Some(1)
        );
    }

    #[test]
    fn stopped_locations_do_not_apply_the_jump_penalty() {
        let config = UzmatchConfig::default();
        let mut previous = location(0, 0.0);
        previous.speed = Some(Speed {
            value: 0.0,
            accuracy: None,
        });
        let state = TemporalMatchState::default().update(
            &previous,
            &[emission_at(100.0, 0, 0.0, 0.0, 0.000_1)],
            &config,
        );
        let mut current = location(1, 10.0 / 111_000.0);
        current.speed = Some(Speed {
            value: 0.0,
            accuracy: None,
        });
        let stable_offset = emission_at(110.0, 1, -1.0, current.coordinates.lng, 0.000_1);
        let frame_winner = emission_at(110.0, 2, 0.0, current.coordinates.lng, -0.000_1);

        let next = state.update(&current, &[frame_winner, stable_offset], &config);

        assert_eq!(
            next.route_position().map(|position| position.segment_index),
            Some(2)
        );
    }

    #[test]
    fn poor_accuracy_relaxes_the_jump_penalty() {
        let config = UzmatchConfig::default();
        let mut previous = location(0, 0.0);
        previous.horizontal_accuracy = 100.0;
        let state = TemporalMatchState::default().update(
            &previous,
            &[emission_at(100.0, 0, 0.0, 0.0, 0.000_1)],
            &config,
        );
        let mut current = location(1, 10.0 / 111_000.0);
        current.horizontal_accuracy = 100.0;
        let stable_offset = emission_at(110.0, 1, -2.0, current.coordinates.lng, 0.000_1);
        let frame_winner = emission_at(110.0, 2, 0.0, current.coordinates.lng, -0.000_1);

        let next = state.update(&current, &[frame_winner, stable_offset], &config);

        assert_eq!(
            next.route_position().map(|position| position.segment_index),
            Some(2)
        );
    }

    #[test]
    fn invalid_jump_measurements_fail_closed() {
        let previous_position = emission_at(100.0, 0, 0.0, 0.0, 0.000_1).route_position;
        let current_position = emission_at(110.0, 1, 0.0, 0.000_1, -0.000_1).route_position;

        let mut invalid_accuracy_previous = location(0, 0.0);
        invalid_accuracy_previous.horizontal_accuracy = f64::NAN;
        let mut invalid_accuracy_current = location(1, 10.0 / 111_000.0);
        invalid_accuracy_current.horizontal_accuracy = f64::NAN;
        assert!(
            jump_offset_log_likelihood(
                previous_position,
                current_position,
                &invalid_accuracy_previous,
                &invalid_accuracy_current,
            )
            .is_none()
        );

        let valid_previous = location(0, 0.0);
        let mut invalid_speed_current = location(1, 10.0 / 111_000.0);
        invalid_speed_current.speed = Some(Speed {
            value: f64::NAN,
            accuracy: None,
        });
        assert!(
            jump_offset_log_likelihood(
                previous_position,
                current_position,
                &valid_previous,
                &invalid_speed_current,
            )
            .is_none()
        );

        let mut missing_speed_previous = location(0, 0.0);
        missing_speed_previous.speed = None;
        assert!(
            jump_offset_log_likelihood(
                previous_position,
                current_position,
                &missing_speed_previous,
                &invalid_speed_current,
            )
            .is_none()
        );
    }

    #[test]
    fn small_backward_projection_jitter_remains_reachable() {
        let config = UzmatchConfig::default();
        let first = location(0, 100.0 / 111_000.0);
        let state =
            TemporalMatchState::default().update(&first, &[emission(100.0, 1, 0.0)], &config);
        let second = location(1, 100.0 / 111_000.0);
        let state = state.update(&second, &[emission(95.0, 1, 0.0)], &config);

        assert_eq!(
            state
                .route_position()
                .map(|position| position.distance_along_route_meters),
            Some(95.0)
        );
    }

    #[test]
    fn impossible_backward_jump_starts_a_new_frontier() {
        let config = UzmatchConfig::default();
        let first = location(0, 100.0 / 111_000.0);
        let state =
            TemporalMatchState::default().update(&first, &[emission(100.0, 1, 0.0)], &config);
        let second = location(1, 50.0 / 111_000.0);
        let state = state.update(&second, &[emission(50.0, 0, -3.0)], &config);

        assert_eq!(state.candidates.len(), 1);
        assert_eq!(state.candidates[0].accumulated_log_likelihood, 0.0);
    }

    #[test]
    fn impossible_forward_jump_starts_a_new_frontier() {
        let config = UzmatchConfig::default();
        let first = location(0, 0.0);
        let state = TemporalMatchState::default().update(&first, &[emission(0.0, 0, 0.0)], &config);
        let second = location(1, 0.0);
        let state = state.update(&second, &[emission(500.0, 5, -3.0)], &config);

        assert_eq!(state.candidates.len(), 1);
        assert_eq!(state.candidates[0].accumulated_log_likelihood, 0.0);
        assert_eq!(
            state
                .route_position()
                .map(|position| position.segment_index),
            Some(5)
        );
    }

    #[test]
    fn stale_and_out_of_order_signals_start_new_frontiers() {
        let config = UzmatchConfig::default();
        let initial = location(10, 0.0);
        let state =
            TemporalMatchState::default().update(&initial, &[emission(0.0, 0, 0.0)], &config);

        for next in [location(16, 0.0), location(9, 0.0)] {
            let reset = state.update(&next, &[emission(500.0, 5, -4.0)], &config);
            assert_eq!(reset.candidates[0].accumulated_log_likelihood, 0.0);
            assert_eq!(
                reset
                    .route_position()
                    .map(|position| position.segment_index),
                Some(5)
            );
        }
    }

    #[test]
    fn oversized_deserialized_frontier_is_bounded_before_transitions() {
        let config = UzmatchConfig::default();
        let previous_location = location(0, 0.0);
        let mut candidates = (0..super::super::snap::MAX_SNAP_CANDIDATES)
            .map(|index| TemporalCandidate {
                route_position: emission(1_000.0, index as u64, 0.0).route_position,
                accumulated_log_likelihood: -(index as f64),
            })
            .collect::<Vec<_>>();
        // This eleventh candidate is the only reachable parent. An oversized
        // external frontier must be rejected before transition work; otherwise
        // its transition would make segment 2 win.
        candidates.push(TemporalCandidate {
            route_position: emission(0.0, 99, 0.0).route_position,
            accumulated_log_likelihood: -10.0,
        });
        let state = TemporalMatchState {
            candidates,
            previous_location: Some(previous_location),
        };

        let next = state.update(
            &location(1, 110.0 / 111_000.0),
            &[emission(100.0, 1, 0.0), emission(110.0, 2, -1.0)],
            &config,
        );

        assert_eq!(next.candidates.len(), 2);
        assert_eq!(
            next.route_position().map(|position| position.segment_index),
            Some(1)
        );
    }
}
