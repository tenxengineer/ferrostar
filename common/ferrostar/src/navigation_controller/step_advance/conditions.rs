use std::sync::Arc;

use super::{StepAdvanceCondition, StepAdvanceConditionSerializable, StepAdvanceResult};
use crate::{
    algorithms::{
        deviation_from_line, get_linestring, is_within_threshold_to_end_of_linestring,
        snap_user_location_to_line,
    },
    models::{GeographicCoordinate, ManeuverModifier, RouteStep, UserLocation},
    navigation_controller::models::TripState,
};
use geo::Point;
use serde::{Deserialize, Serialize};

#[cfg(all(feature = "std", not(feature = "web-time")))]
use std::time::SystemTime;

#[cfg(feature = "web-time")]
use web_time::SystemTime;

#[cfg(feature = "wasm-bindgen")]
use tsify::Tsify;

#[cfg(test)]
use proptest::prelude::*;

#[cfg(test)]
use crate::{
    deviation_detection::{DeviationKind, RouteDeviation},
    navigation_controller::test_helpers::get_navigating_trip_state,
    test_utils::{arb_coord, make_user_location},
};

use super::SerializableStepAdvanceCondition;

const UZMAP_EARTH_RADIUS_METRES: f64 = 6_371_000.0;

/// Never advances to the next step automatically;
/// requires calling [`NavigationController::advance_to_next_step`](super::NavigationController::advance_to_next_step).
///
/// You can use this to implement custom behaviors in external code.
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "uniffi", derive(uniffi::Object))]
pub struct ManualStepCondition;

impl StepAdvanceCondition for ManualStepCondition {
    #[allow(unused_variables)]
    fn should_advance_step(&self, trip_state: TripState) -> StepAdvanceResult {
        StepAdvanceResult::continue_with_state(Arc::new(ManualStepCondition))
    }

    fn new_instance(&self) -> Arc<dyn StepAdvanceCondition> {
        Arc::new(ManualStepCondition)
    }
}

impl StepAdvanceConditionSerializable for ManualStepCondition {
    fn to_js(&self) -> SerializableStepAdvanceCondition {
        SerializableStepAdvanceCondition::Manual
    }
}

// MARK: Basic Conditions

/// Automatically advances when the user's location is close enough to the end of the step.
///
/// This results in an eager advance where the user will jump to the next step when the
/// condition is met.
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "uniffi", derive(uniffi::Object))]
pub struct DistanceToEndOfStepCondition {
    /// Distance to the last waypoint in the step, measured in meters, at which to advance.
    pub distance: u16,
    /// The minimum required horizontal accuracy of the user location, in meters.
    /// Values larger than this cannot trigger a step advance.
    pub minimum_horizontal_accuracy: u16,
}

impl StepAdvanceCondition for DistanceToEndOfStepCondition {
    fn should_advance_step(&self, trip_state: TripState) -> StepAdvanceResult {
        self.should_advance_inner(&trip_state)
            .unwrap_or(StepAdvanceResult::continue_with_state(self.new_instance()))
    }

    fn new_instance(&self) -> Arc<dyn StepAdvanceCondition> {
        Arc::new(DistanceToEndOfStepCondition {
            distance: self.distance,
            minimum_horizontal_accuracy: self.minimum_horizontal_accuracy,
        })
    }
}

impl DistanceToEndOfStepCondition {
    fn should_advance_inner(&self, trip_state: &TripState) -> Option<StepAdvanceResult> {
        let user_location = trip_state.user_location()?;
        let current_step = trip_state.current_step()?;

        let should_advance =
            if user_location.horizontal_accuracy > self.minimum_horizontal_accuracy.into() {
                false
            } else {
                is_within_threshold_to_end_of_linestring(
                    &user_location.into(),
                    &current_step.get_linestring(),
                    f64::from(self.distance),
                )
            };

        let result = if should_advance {
            StepAdvanceResult::advance_to_new_instance(self)
        } else {
            StepAdvanceResult::continue_with_state(self.new_instance())
        };

        Some(result)
    }
}

impl StepAdvanceConditionSerializable for DistanceToEndOfStepCondition {
    fn to_js(&self) -> SerializableStepAdvanceCondition {
        SerializableStepAdvanceCondition::DistanceToEndOfStep {
            distance: self.distance,
            minimum_horizontal_accuracy: self.minimum_horizontal_accuracy,
        }
    }
}

/// Controls when a deviation-aware step-advance condition is allowed to evaluate,
/// based on the user's current
/// [`RouteDeviation`](crate::deviation_detection::RouteDeviation) status.
#[derive(Debug, Copy, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "uniffi", derive(uniffi::Enum))]
#[cfg_attr(feature = "wasm-bindgen", derive(Tsify))]
#[cfg_attr(feature = "wasm-bindgen", tsify(into_wasm_abi, from_wasm_abi))]
pub enum DeviationCalculationPolicy {
    /// Always evaluate, regardless of deviation status.
    Always,
    /// Evaluate while the user is still somewhere on the route polyline
    /// (current step or any future step),
    /// but suspend if the user is completely off the route.
    WhileOnRoute,
    /// Evaluate only while the user is within an acceptable deviation of the current step's polyline.
    ///
    /// Suspend on any deviation
    /// (whether off-step-but-on-route, or completely off-route).
    WhileOnCurrentStep,
}

/// Advances once the user is at least [`Self::distance`] meters from the current step's polyline.
///
/// This results in *delayed* advance,
/// but is more robust to spurious / unwanted step changes in scenarios including
/// self-intersecting routes (sudden jumps to the next step)
/// and pauses at intersections (advancing too soon before the maneuver is complete).
///
/// NOTE! This may be less robust to things like short steps, out-and-backs, and U-turns,
/// where this may eagerly exit a current step before the user has traversed it
/// if the start of the step is within range of the end.
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "uniffi", derive(uniffi::Object))]
pub struct DistanceFromStepCondition {
    /// The minimum distance, in meters,
    /// from the current step's polyline at which this condition advances.
    pub distance: u16,
    /// The minimum required horizontal accuracy of the user location, in meters.
    /// Locations with reported accuracy worse than this cannot trigger a step advance.
    pub minimum_horizontal_accuracy: u16,
    /// Controls when this condition is permitted to evaluate based on the user's current
    /// [`RouteDeviation`](crate::deviation_detection::RouteDeviation) status.
    pub calculation_policy: DeviationCalculationPolicy,
}

impl StepAdvanceCondition for DistanceFromStepCondition {
    fn should_advance_step(&self, trip_state: TripState) -> StepAdvanceResult {
        self.should_advance_inner(&trip_state)
            .unwrap_or(StepAdvanceResult::continue_with_state(self.new_instance()))
    }

    fn new_instance(&self) -> Arc<dyn StepAdvanceCondition> {
        // Simple Arc::new here; there is no internal state to reset for this condition.
        Arc::new(*self)
    }
}

impl DistanceFromStepCondition {
    fn should_advance_inner(&self, trip_state: &TripState) -> Option<StepAdvanceResult> {
        let deviation = trip_state.deviation()?;
        let user_location = trip_state.user_location()?;
        let current_step = trip_state.current_step()?;

        let permits_calculation = match self.calculation_policy {
            DeviationCalculationPolicy::Always => true,
            DeviationCalculationPolicy::WhileOnRoute => !deviation.is_completely_off_route(),
            DeviationCalculationPolicy::WhileOnCurrentStep => {
                !deviation.is_deviated_from_current_step()
            }
        };
        let location_too_inaccurate =
            user_location.horizontal_accuracy > self.minimum_horizontal_accuracy.into();

        let should_advance = if !permits_calculation || location_too_inaccurate {
            // Bail early
            false
        } else {
            let current_position: Point = user_location.into();
            let current_step_linestring = current_step.get_linestring();

            deviation_from_line(&current_position, &current_step_linestring)
                .is_some_and(|deviation| deviation > self.distance.into())
        };

        let result = if should_advance {
            StepAdvanceResult::advance_to_new_instance(self)
        } else {
            StepAdvanceResult::continue_with_state(self.new_instance())
        };

        Some(result)
    }
}

impl StepAdvanceConditionSerializable for DistanceFromStepCondition {
    fn to_js(&self) -> SerializableStepAdvanceCondition {
        SerializableStepAdvanceCondition::DistanceFromStep {
            distance: self.distance,
            minimum_horizontal_accuracy: self.minimum_horizontal_accuracy,
            calculation_policy: self.calculation_policy,
        }
    }
}

/// Advance if any of the conditions are met (OR).
///
/// This is ideal for short circuit type advance conditions.
///
/// E.g. you may have:
/// 1. A short circuit detecting if the user has exceeded a large distance from the current step.
/// 2. A default advance behavior.
#[derive(Clone)]
#[cfg_attr(feature = "uniffi", derive(uniffi::Object))]
pub struct OrAdvanceConditions {
    pub conditions: Vec<Arc<dyn StepAdvanceCondition>>,
}

impl StepAdvanceCondition for OrAdvanceConditions {
    fn should_advance_step(&self, trip_state: TripState) -> StepAdvanceResult {
        let mut should_advance = false;
        let mut next_conditions = Vec::with_capacity(self.conditions.len());

        for condition in &self.conditions {
            let result = condition.should_advance_step(trip_state.clone());
            should_advance = should_advance || result.should_advance;
            next_conditions.push(result.next_iteration);
        }

        StepAdvanceResult {
            should_advance,
            next_iteration: if should_advance {
                // When advancing, create fresh instances of all conditions to ensure state isolation
                self.new_instance()
            } else {
                // Preserve stateful progress when not advancing
                Arc::new(OrAdvanceConditions {
                    conditions: next_conditions,
                })
            },
        }
    }

    fn new_instance(&self) -> Arc<dyn StepAdvanceCondition> {
        Arc::new(OrAdvanceConditions {
            conditions: self
                .conditions
                .iter()
                .map(|condition| condition.new_instance())
                .collect(),
        })
    }
}

impl StepAdvanceConditionSerializable for OrAdvanceConditions {
    fn to_js(&self) -> SerializableStepAdvanceCondition {
        SerializableStepAdvanceCondition::OrAdvanceConditions {
            conditions: self.conditions.iter().map(|c| c.to_js()).collect(),
        }
    }
}

/// Advance if all of the conditions are met (AND).
#[derive(Clone)]
#[cfg_attr(feature = "uniffi", derive(uniffi::Object))]
pub struct AndAdvanceConditions {
    pub conditions: Vec<Arc<dyn StepAdvanceCondition>>,
}

impl StepAdvanceCondition for AndAdvanceConditions {
    fn should_advance_step(&self, trip_state: TripState) -> StepAdvanceResult {
        let mut should_advance = true;
        let mut next_conditions = Vec::with_capacity(self.conditions.len());

        for condition in &self.conditions {
            let result = condition.should_advance_step(trip_state.clone());
            should_advance = should_advance && result.should_advance;
            next_conditions.push(result.next_iteration);
        }

        StepAdvanceResult {
            should_advance,
            next_iteration: if should_advance {
                // When advancing, create fresh instances of all conditions to ensure state isolation
                self.new_instance()
            } else {
                // Preserve stateful progress when not advancing
                Arc::new(AndAdvanceConditions {
                    conditions: next_conditions,
                })
            },
        }
    }

    fn new_instance(&self) -> Arc<dyn StepAdvanceCondition> {
        Arc::new(AndAdvanceConditions {
            conditions: self
                .conditions
                .iter()
                .map(|condition| condition.new_instance())
                .collect(),
        })
    }
}

impl StepAdvanceConditionSerializable for AndAdvanceConditions {
    fn to_js(&self) -> SerializableStepAdvanceCondition {
        SerializableStepAdvanceCondition::AndAdvanceConditions {
            conditions: self.conditions.iter().map(|c| c.to_js()).collect(),
        }
    }
}

/// A stateful condition that requires the user to reach the end of the step then proceed past it to advance.
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "uniffi", derive(uniffi::Object))]
pub struct DistanceEntryAndExitCondition {
    /// Mark the arrival at the end of the step once the user is within this distance.
    pub(super) distance_to_end_of_step: u16,
    /// Advance only after the user has left the end of the step by at least this distance.
    ///
    /// This value should be small to avoid the user appearing stuck on the step when using
    /// visible location snapping.
    pub(super) distance_after_end_of_step: u16,
    /// The minimum required horizontal accuracy of the user location, in meters.
    /// Values larger than this cannot ever trigger a step advance.
    pub(super) minimum_horizontal_accuracy: u16,
    /// Internal state for tracking when the user is within `distance_to_end_of_step` meters from the end of the step.
    /// This allows for stateful advance only after entering a reasonable radius of the goal
    /// and then exiting the area by a separate trigger threshold.
    pub(super) has_reached_end_of_current_step: bool,
    // TODO: Do we want a speed multiplier
}

impl Default for DistanceEntryAndExitCondition {
    fn default() -> Self {
        Self {
            distance_to_end_of_step: 20,
            distance_after_end_of_step: 5,
            minimum_horizontal_accuracy: 25,
            has_reached_end_of_current_step: false,
        }
    }
}

#[cfg(test)]
impl DistanceEntryAndExitCondition {
    pub fn exact() -> Self {
        Self {
            distance_to_end_of_step: 0,
            distance_after_end_of_step: 0,
            minimum_horizontal_accuracy: 0,
            has_reached_end_of_current_step: false,
        }
    }
}

impl StepAdvanceCondition for DistanceEntryAndExitCondition {
    fn should_advance_step(&self, trip_state: TripState) -> StepAdvanceResult {
        if self.has_reached_end_of_current_step {
            // This inner check fires once the user is far enough from the current step's
            // polyline that we treat them as having moved past the end of the step.
            //
            // `WhileOnRoute` is the policy we want here:
            //   - On `OffStepOnRoute` (user has progressed onto a future step):
            //     evaluate. That is the normal success case for an exit check.
            //   - On `CompletelyOffRoute` (user is lost): suspend.
            //     The rerouter, not this condition, should drive what happens next.
            let distance_from_end = DistanceFromStepCondition {
                minimum_horizontal_accuracy: self.minimum_horizontal_accuracy,
                distance: self.distance_after_end_of_step,
                calculation_policy: DeviationCalculationPolicy::WhileOnRoute,
            };

            let should_advance = distance_from_end
                .should_advance_step(trip_state)
                .should_advance;

            if should_advance {
                StepAdvanceResult::advance_to_new_instance(self)
            } else {
                // The condition was not advanced. So we return a fresh iteration
                // where has_reached_end_of_current_step is still true to re-trigger this part 2 logic.
                StepAdvanceResult::continue_with_state(Arc::new(DistanceEntryAndExitCondition {
                    distance_to_end_of_step: self.distance_to_end_of_step,
                    distance_after_end_of_step: self.distance_after_end_of_step,
                    minimum_horizontal_accuracy: self.minimum_horizontal_accuracy,
                    has_reached_end_of_current_step: true,
                }))
            }
        } else {
            let distance_to_end = DistanceToEndOfStepCondition {
                minimum_horizontal_accuracy: self.minimum_horizontal_accuracy,
                distance: self.distance_to_end_of_step,
            };

            // Use the distance to end to determine if has_reached_end_of_current_step
            let next_iteration = DistanceEntryAndExitCondition {
                minimum_horizontal_accuracy: self.minimum_horizontal_accuracy,
                distance_to_end_of_step: self.distance_to_end_of_step,
                distance_after_end_of_step: self.distance_after_end_of_step,
                has_reached_end_of_current_step: distance_to_end
                    .should_advance_step(trip_state)
                    .should_advance,
            };

            StepAdvanceResult::continue_with_state(Arc::new(next_iteration))
        }
    }

    fn new_instance(&self) -> Arc<dyn StepAdvanceCondition> {
        Arc::new(DistanceEntryAndExitCondition {
            distance_to_end_of_step: self.distance_to_end_of_step,
            distance_after_end_of_step: self.distance_after_end_of_step,
            minimum_horizontal_accuracy: self.minimum_horizontal_accuracy,
            has_reached_end_of_current_step: false, // Always reset to initial state
        })
    }
}

impl StepAdvanceConditionSerializable for DistanceEntryAndExitCondition {
    fn to_js(&self) -> SerializableStepAdvanceCondition {
        SerializableStepAdvanceCondition::DistanceEntryExit {
            minimum_horizontal_accuracy: self.minimum_horizontal_accuracy,
            distance_to_end_of_step: self.distance_to_end_of_step,
            distance_after_end_step: self.distance_after_end_of_step,
            has_reached_end_of_current_step: self.has_reached_end_of_current_step,
        }
    }
}

/// A stateful condition that requires the user to reach the end of the step then proceed past it to advance.
///
/// This variant uses route snapping (snapping to the combined current+next step geometry) for the exit check,
/// making it more robust for pedestrian/hiking navigation where users may walk on the opposite side of the street
/// or wander around the optimal path. The route-snapped exit check prevents premature advancement while still
/// allowing natural pedestrian movement patterns.
/// The exit distance is measured from the current step to the route-snapped position.
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "uniffi", derive(uniffi::Object))]
pub struct DistanceEntryAndSnappedExitCondition {
    /// Mark the arrival at the end of the step once the user is within this distance.
    pub(super) distance_to_end_of_step: u16,
    /// Advance only after the route-snapped position has moved this distance from the current step.
    ///
    /// This uses a snapped position to the combined route (current+next steps) which provides
    /// better handling of pedestrian scenarios like walking on the opposite side of the street.
    /// Values of 2-5m work well for pedestrian navigation.
    pub(super) distance_after_end_of_step: u16,
    /// The minimum required horizontal accuracy of the user location, in meters.
    /// Values larger than this cannot ever trigger a step advance.
    pub(super) minimum_horizontal_accuracy: u16,
    /// Internal state for tracking when the user is within `distance_to_end_of_step` meters from the end of the step.
    pub(super) has_reached_end_of_current_step: bool,
}

impl Default for DistanceEntryAndSnappedExitCondition {
    fn default() -> Self {
        Self {
            distance_to_end_of_step: 20,
            distance_after_end_of_step: 2,
            minimum_horizontal_accuracy: 25,
            has_reached_end_of_current_step: false,
        }
    }
}

#[cfg(test)]
impl DistanceEntryAndSnappedExitCondition {
    pub fn exact() -> Self {
        Self {
            distance_to_end_of_step: 0,
            distance_after_end_of_step: 0,
            minimum_horizontal_accuracy: 0,
            has_reached_end_of_current_step: false,
        }
    }
}

impl StepAdvanceCondition for DistanceEntryAndSnappedExitCondition {
    #[allow(unused_variables)]
    fn should_advance_step(&self, trip_state: TripState) -> StepAdvanceResult {
        let result = if self.has_reached_end_of_current_step {
            // EXIT CHECK: Use existing distance to end logic
            self.check_exit_result(&trip_state)
        } else {
            // ENTRY CHECK: Use existing distance to end logic
            self.check_entry_result(&trip_state)
        };

        result.unwrap_or(StepAdvanceResult::continue_with_state(self.new_instance()))
    }

    fn new_instance(&self) -> Arc<dyn StepAdvanceCondition> {
        Arc::new(DistanceEntryAndSnappedExitCondition {
            has_reached_end_of_current_step: false, // Always reset this to the initial state
            ..*self
        })
    }
}

impl DistanceEntryAndSnappedExitCondition {
    fn check_exit_result(&self, trip_state: &TripState) -> Option<StepAdvanceResult> {
        let user_location = trip_state.user_location()?;
        let current_step = trip_state.current_step()?;
        let next_step = trip_state.next_step();

        let should_advance = if user_location.horizontal_accuracy
            > self.minimum_horizontal_accuracy.into()
        {
            false
        } else if let Some(next) = next_step {
            // Build combined linestring from current + N next steps
            // Accumulate steps until we have enough distance for meaningful exit check
            let mut combined_coords = current_step.geometry.clone();
            let mut accumulated_distance = next.distance;
            let mut step_index = 1;

            // Add first next step
            combined_coords.extend(next.geometry.clone());

            // Keep adding subsequent steps until we have sufficient distance
            // or run out of steps. Use 2x multiplier to increase chances of handling
            // U-turns and complex geometries in future steps.
            let target_distance = (self.distance_after_end_of_step as f64) * 2.0;
            while accumulated_distance < target_distance {
                if let Some(future_step) = trip_state.get_step(step_index + 1) {
                    combined_coords.extend(future_step.geometry.clone());
                    accumulated_distance += future_step.distance;
                    step_index += 1;
                } else {
                    break;
                }
            }

            let combined_linestring = get_linestring(&combined_coords);

            // Snap to the combined route
            let snapped_to_route = snap_user_location_to_line(user_location, &combined_linestring);

            // Measure distance from CURRENT step only
            let current_step_linestring = current_step.get_linestring();
            let deviation =
                deviation_from_line(&Point::from(snapped_to_route), &current_step_linestring)
                    .unwrap_or(0.0);

            // Use the minimum of configured exit distance and accumulated distance
            // to handle cases where there aren't enough future steps
            let effective_exit_distance =
                (self.distance_after_end_of_step as f64).min(accumulated_distance);

            deviation >= effective_exit_distance
        } else {
            // Advance because no next step.
            true
        };

        // EXIT CHECK: Use route-snapped position
        if should_advance {
            Some(StepAdvanceResult::advance_to_new_instance(self))
        } else {
            None
        }
    }

    fn check_entry_result(&self, trip_state: &TripState) -> Option<StepAdvanceResult> {
        let distance_to_end = DistanceToEndOfStepCondition {
            minimum_horizontal_accuracy: self.minimum_horizontal_accuracy,
            distance: self.distance_to_end_of_step,
        };

        let next_iteration = DistanceEntryAndSnappedExitCondition {
            has_reached_end_of_current_step: distance_to_end
                .should_advance_step(trip_state.clone())
                .should_advance,
            ..*self
        };

        let result = StepAdvanceResult::continue_with_state(Arc::new(next_iteration));
        Some(result)
    }
}

impl StepAdvanceConditionSerializable for DistanceEntryAndSnappedExitCondition {
    fn to_js(&self) -> SerializableStepAdvanceCondition {
        SerializableStepAdvanceCondition::DistanceEntryAndSnappedExit {
            minimum_horizontal_accuracy: self.minimum_horizontal_accuracy,
            distance_to_end_of_step: self.distance_to_end_of_step,
            distance_after_end_step: self.distance_after_end_of_step,
            has_reached_end_of_current_step: self.has_reached_end_of_current_step,
        }
    }
}

#[derive(Clone)]
#[cfg_attr(feature = "uniffi", derive(uniffi::Object))]
pub struct DistanceEntryAndExitWithUTurnConfirmationCondition {
    pub(super) distance_to_end_of_step: u16,
    pub(super) distance_after_end_of_step: u16,
    pub(super) minimum_horizontal_accuracy: u16,
    pub(super) minimum_significant_movement: u16,
    pub(super) maximum_plausible_speed: u16,
    pub(super) plausibility_distance_allowance: u16,
    pub(super) required_confirmations: u8,
    pub(super) uturn_confirmation_enabled: bool,
    pub(super) candidate_is_uturn: Option<bool>,
    pub(super) candidate_successor: Option<Arc<dyn StepAdvanceCondition>>,
    pub(super) confirmation_active: bool,
    pub(super) movement_anchor: Option<UserLocation>,
    pub(super) confirmation_count: u8,
    pub(super) last_evaluated_timestamp: Option<SystemTime>,
}

impl DistanceEntryAndExitWithUTurnConfirmationCondition {
    fn fresh_candidate(&self, is_uturn: bool) -> Arc<dyn StepAdvanceCondition> {
        if is_uturn {
            Arc::new(DistanceEntryAndSnappedExitCondition {
                distance_to_end_of_step: self.distance_to_end_of_step,
                distance_after_end_of_step: self.distance_after_end_of_step,
                minimum_horizontal_accuracy: self.minimum_horizontal_accuracy,
                has_reached_end_of_current_step: false,
            })
        } else {
            Arc::new(DistanceEntryAndExitCondition {
                distance_to_end_of_step: self.distance_to_end_of_step,
                distance_after_end_of_step: self.distance_after_end_of_step,
                minimum_horizontal_accuracy: self.minimum_horizontal_accuracy,
                has_reached_end_of_current_step: false,
            })
        }
    }

    fn candidate_for_tick(&self, is_uturn: bool) -> Arc<dyn StepAdvanceCondition> {
        match (&self.candidate_successor, self.candidate_is_uturn) {
            (Some(candidate), Some(stored_kind)) if stored_kind == is_uturn => {
                Arc::clone(candidate)
            }
            _ => self.fresh_candidate(is_uturn),
        }
    }

    fn is_valid_fix(&self, location: UserLocation) -> bool {
        is_valid_coordinate(location.coordinates)
            && location.horizontal_accuracy.is_finite()
            && location.horizontal_accuracy >= 0.0
            && location.horizontal_accuracy <= f64::from(self.minimum_horizontal_accuracy)
    }

    fn cleared_movement_progress(&self) -> Self {
        Self {
            movement_anchor: None,
            confirmation_count: 0,
            ..self.clone()
        }
    }

    fn continue_with(next: Self) -> StepAdvanceResult {
        StepAdvanceResult::continue_with_state(Arc::new(next))
    }

    fn evaluate_confirmation(&self, trip_state: &TripState) -> StepAdvanceResult {
        if self.required_confirmations == 0 {
            return StepAdvanceResult::continue_with_state(self.new_instance());
        }

        let Some(current_location) = trip_state.user_location() else {
            return Self::continue_with(self.cleared_movement_progress());
        };

        let Some(anchor) = self.movement_anchor else {
            let mut next = self.cleared_movement_progress();
            if self.is_valid_fix(current_location) {
                next.movement_anchor = Some(current_location);
            }
            return Self::continue_with(next);
        };

        if !self.is_valid_fix(anchor) || !self.is_valid_fix(current_location) {
            return Self::continue_with(self.cleared_movement_progress());
        }

        let Ok(elapsed) = current_location.timestamp.duration_since(anchor.timestamp) else {
            return Self::continue_with(self.cleared_movement_progress());
        };
        if elapsed.is_zero() {
            return Self::continue_with(self.cleared_movement_progress());
        }

        let distance =
            uzmap_canonical_distance_metres(anchor.coordinates, current_location.coordinates);
        let plausible_distance = f64::from(self.maximum_plausible_speed) * elapsed.as_secs_f64()
            + f64::from(self.plausibility_distance_allowance);
        if !distance.is_finite() || !plausible_distance.is_finite() || distance > plausible_distance
        {
            return Self::continue_with(self.cleared_movement_progress());
        }

        if distance < f64::from(self.minimum_significant_movement) {
            return Self::continue_with(self.clone());
        }

        let Some(current_step) = trip_state.current_step() else {
            return Self::continue_with(self.cleared_movement_progress());
        };
        let Some(next_step) = trip_state.next_step() else {
            return Self::continue_with(self.cleared_movement_progress());
        };
        let Some((current_origin, current_destination)) = last_valid_segment(&current_step) else {
            return Self::continue_with(self.cleared_movement_progress());
        };
        let Some((next_origin, next_destination)) = first_valid_segment(&next_step) else {
            return Self::continue_with(self.cleared_movement_progress());
        };

        let Some(movement_bearing) = normalized_bearing(
            Point::from(anchor.coordinates),
            Point::from(current_location.coordinates),
        ) else {
            return Self::continue_with(self.cleared_movement_progress());
        };
        let Some(current_bearing) = normalized_bearing(current_origin, current_destination) else {
            return Self::continue_with(self.cleared_movement_progress());
        };
        let Some(next_bearing) = normalized_bearing(next_origin, next_destination) else {
            return Self::continue_with(self.cleared_movement_progress());
        };

        let current_delta = angular_distance(movement_bearing, current_bearing);
        let next_delta = angular_distance(movement_bearing, next_bearing);
        let mut next = self.clone();
        next.movement_anchor = Some(current_location);

        if next_delta < current_delta {
            next.confirmation_count = next.confirmation_count.saturating_add(1);
            if next.confirmation_count >= next.required_confirmations {
                return StepAdvanceResult::advance_to_new_instance(self);
            }
        } else if current_delta < next_delta {
            next.confirmation_count = 0;
        }

        Self::continue_with(next)
    }
}

fn is_uturn(trip_state: &TripState) -> bool {
    matches!(
        trip_state,
        TripState::Navigating {
            visual_instruction: Some(instruction),
            ..
        } if instruction.primary_content.maneuver_modifier == Some(ManeuverModifier::UTurn)
    )
}

fn is_valid_coordinate(coordinate: GeographicCoordinate) -> bool {
    coordinate.lat.is_finite()
        && (-90.0..=90.0).contains(&coordinate.lat)
        && coordinate.lng.is_finite()
        && (-180.0..=180.0).contains(&coordinate.lng)
}

fn uzmap_canonical_distance_metres(
    origin: GeographicCoordinate,
    destination: GeographicCoordinate,
) -> f64 {
    let d_lat = (destination.lat - origin.lat).to_radians();
    let d_lng = (destination.lng - origin.lng).to_radians();
    let half_lat_sine = (d_lat / 2.0).sin();
    let half_lng_sine = (d_lng / 2.0).sin();
    let a = half_lat_sine * half_lat_sine
        + origin.lat.to_radians().cos()
            * destination.lat.to_radians().cos()
            * half_lng_sine
            * half_lng_sine;
    UZMAP_EARTH_RADIUS_METRES * 2.0 * a.sqrt().atan2((1.0 - a).sqrt())
}

fn valid_segment(pair: &[GeographicCoordinate]) -> Option<(Point, Point)> {
    let [origin, destination] = pair else {
        return None;
    };
    if !is_valid_coordinate(*origin) || !is_valid_coordinate(*destination) {
        return None;
    }

    let distance = uzmap_canonical_distance_metres(*origin, *destination);
    (distance.is_finite() && distance > 0.0)
        .then_some((Point::from(*origin), Point::from(*destination)))
}

fn last_valid_segment(step: &RouteStep) -> Option<(Point, Point)> {
    step.geometry.windows(2).filter_map(valid_segment).last()
}

fn first_valid_segment(step: &RouteStep) -> Option<(Point, Point)> {
    step.geometry.windows(2).find_map(valid_segment)
}

fn normalized_bearing(origin: Point, destination: Point) -> Option<f64> {
    let start_latitude_radians = origin.y().to_radians();
    let end_latitude_radians = destination.y().to_radians();
    let longitude_delta_radians = (destination.x() - origin.x()).to_radians();
    let y = longitude_delta_radians.sin() * end_latitude_radians.cos();
    let x = start_latitude_radians.cos() * end_latitude_radians.sin()
        - start_latitude_radians.sin() * end_latitude_radians.cos() * longitude_delta_radians.cos();
    let bearing = y.atan2(x).to_degrees();
    bearing.is_finite().then(|| (bearing + 360.0) % 360.0)
}

fn angular_distance(a: f64, b: f64) -> f64 {
    (((a - b + 540.0) % 360.0) - 180.0).abs()
}

impl StepAdvanceCondition for DistanceEntryAndExitWithUTurnConfirmationCondition {
    fn should_advance_step(&self, trip_state: TripState) -> StepAdvanceResult {
        let current_timestamp = trip_state.user_location().map(|fix| fix.timestamp);
        if current_timestamp.is_some() && current_timestamp == self.last_evaluated_timestamp {
            return Self::continue_with(self.clone());
        }

        let mut evaluated = self.clone();
        evaluated.last_evaluated_timestamp = current_timestamp;

        if evaluated.required_confirmations == 0 {
            return StepAdvanceResult::continue_with_state(evaluated.new_instance());
        }
        if evaluated.confirmation_active {
            return evaluated.evaluate_confirmation(&trip_state);
        }

        let selected_is_uturn = evaluated.uturn_confirmation_enabled && is_uturn(&trip_state);
        let candidate = evaluated.candidate_for_tick(selected_is_uturn);
        let candidate_result = candidate.should_advance_step(trip_state.clone());
        let candidate_should_advance = candidate_result.should_advance();

        let mut next = evaluated.clone();
        next.candidate_is_uturn = Some(selected_is_uturn);
        next.candidate_successor = Some(candidate_result.next_iteration);
        next.confirmation_active = false;
        next.movement_anchor = None;
        next.confirmation_count = 0;

        if candidate_should_advance {
            if !selected_is_uturn {
                return StepAdvanceResult::advance_to_new_instance(&evaluated);
            }

            next.confirmation_active = true;
            next.movement_anchor = trip_state
                .user_location()
                .filter(|location| evaluated.is_valid_fix(*location));
        }

        Self::continue_with(next)
    }

    fn new_instance(&self) -> Arc<dyn StepAdvanceCondition> {
        Arc::new(Self {
            distance_to_end_of_step: self.distance_to_end_of_step,
            distance_after_end_of_step: self.distance_after_end_of_step,
            minimum_horizontal_accuracy: self.minimum_horizontal_accuracy,
            minimum_significant_movement: self.minimum_significant_movement,
            maximum_plausible_speed: self.maximum_plausible_speed,
            plausibility_distance_allowance: self.plausibility_distance_allowance,
            required_confirmations: self.required_confirmations,
            uturn_confirmation_enabled: self.uturn_confirmation_enabled,
            candidate_is_uturn: None,
            candidate_successor: None,
            confirmation_active: false,
            movement_anchor: None,
            confirmation_count: 0,
            last_evaluated_timestamp: self.last_evaluated_timestamp,
        })
    }
}

impl StepAdvanceConditionSerializable for DistanceEntryAndExitWithUTurnConfirmationCondition {
    fn to_js(&self) -> SerializableStepAdvanceCondition {
        SerializableStepAdvanceCondition::DistanceEntryAndExitWithUTurnConfirmation {
            distance_to_end_of_step: self.distance_to_end_of_step,
            distance_after_end_step: self.distance_after_end_of_step,
            minimum_horizontal_accuracy: self.minimum_horizontal_accuracy,
            minimum_significant_movement: self.minimum_significant_movement,
            maximum_plausible_speed: self.maximum_plausible_speed,
            plausibility_distance_allowance: self.plausibility_distance_allowance,
            required_confirmations: self.required_confirmations,
            uturn_confirmation_enabled: self.uturn_confirmation_enabled,
            candidate_is_uturn: self.candidate_is_uturn,
            candidate_successor: self
                .candidate_successor
                .iter()
                .map(|candidate| candidate.to_js())
                .collect(),
            confirmation_active: self.confirmation_active,
            movement_anchor: self.movement_anchor,
            confirmation_count: self.confirmation_count,
            last_evaluated_timestamp: self.last_evaluated_timestamp,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::{
        ManeuverModifier, RouteStep, UserLocation, VisualInstruction, VisualInstructionContent,
    };
    use crate::navigation_controller::step_advance::step_advance_distance_entry_and_exit_with_uturn_confirmation;
    use crate::navigation_controller::test_helpers::{
        gen_route_step_with_coords, get_navigating_trip_state,
    };
    use crate::test_utils::make_user_location;
    use geo::{Bearing, Distance, coord};
    use std::sync::LazyLock;
    use std::time::{Duration, SystemTime, UNIX_EPOCH};

    const KAN_69_T0_SECONDS: u64 = 1_785_319_200;

    static STRAIGHT_LINE_SHORT_ROUTE_STEP: LazyLock<RouteStep> = LazyLock::new(|| {
        gen_route_step_with_coords(vec![
            coord!(x: 0.0, y: 0.0),   // Origin
            coord!(x: 0.001, y: 0.0), // 111 meters east at the equator
        ])
    });

    static LOCATION_NEAR_START_OF_STEP: LazyLock<UserLocation> =
        LazyLock::new(|| make_user_location(coord!(x: 0.0001, y: 0.0), 5.0));
    static LOCATION_NEAR_END_OF_STEP: LazyLock<UserLocation> =
        LazyLock::new(|| make_user_location(coord!(x: 0.00099, y: 0.0), 5.0));

    fn kan_69_current_step() -> RouteStep {
        gen_route_step_with_coords(vec![
            coord!(x: 69.00000, y: 41.00000),
            coord!(x: 69.00000, y: 41.00100),
        ])
    }

    fn kan_69_next_step() -> RouteStep {
        gen_route_step_with_coords(vec![
            coord!(x: 69.00000, y: 41.00100),
            coord!(x: 69.000155, y: 41.00094),
            coord!(x: 69.000155, y: 41.00000),
        ])
    }

    fn kan_69_location(lat: f64, lng: f64, second_offset: u64, accuracy: f64) -> UserLocation {
        let mut location = make_user_location(coord!(x: lng, y: lat), accuracy);
        location.timestamp = UNIX_EPOCH + Duration::from_secs(KAN_69_T0_SECONDS + second_offset);
        location
    }

    fn kan_69_instruction(modifier: ManeuverModifier) -> VisualInstruction {
        VisualInstruction {
            primary_content: VisualInstructionContent {
                text: "U-turn".into(),
                maneuver_type: None,
                maneuver_modifier: Some(modifier),
                roundabout_exit_degrees: None,
                lane_info: None,
                exit_numbers: vec![],
            },
            secondary_content: None,
            sub_content: None,
            trigger_distance_before_maneuver: 30.0,
        }
    }

    fn kan_69_state(location: UserLocation, modifier: Option<ManeuverModifier>) -> TripState {
        let mut state = get_navigating_trip_state(
            location,
            vec![
                kan_69_current_step(),
                kan_69_next_step(),
                kan_69_next_step(),
            ],
            vec![],
            RouteDeviation::NoDeviation,
        );
        if let TripState::Navigating {
            visual_instruction, ..
        } = &mut state
        {
            *visual_instruction = modifier.map(kan_69_instruction);
        }
        state
    }

    fn kan_69_coordinate(lat: f64, lng: f64) -> GeographicCoordinate {
        GeographicCoordinate { lat, lng }
    }

    fn kan_69_equatorial_current() -> Vec<GeographicCoordinate> {
        vec![kan_69_coordinate(-0.001, 0.0), kan_69_coordinate(0.0, 0.0)]
    }

    fn kan_69_equatorial_next() -> Vec<GeographicCoordinate> {
        vec![kan_69_coordinate(0.0, 0.0), kan_69_coordinate(-0.001, 0.0)]
    }

    fn kan_69_state_with_geometry(
        location: UserLocation,
        current_geometry: Vec<GeographicCoordinate>,
        next_geometry: Vec<GeographicCoordinate>,
    ) -> TripState {
        let mut current_step = kan_69_current_step();
        current_step.geometry = current_geometry;
        let mut next_step = kan_69_next_step();
        next_step.geometry = next_geometry;
        get_navigating_trip_state(
            location,
            vec![current_step, next_step.clone(), next_step],
            vec![],
            RouteDeviation::NoDeviation,
        )
    }

    fn active_kan_69_condition(
        required_confirmations: u8,
        confirmation_count: u8,
        movement_anchor: Option<UserLocation>,
    ) -> Arc<dyn StepAdvanceCondition> {
        let last_evaluated_timestamp = movement_anchor.map(|fix| fix.timestamp);
        active_kan_69_condition_with_watermark(
            required_confirmations,
            confirmation_count,
            movement_anchor,
            last_evaluated_timestamp,
        )
    }

    fn active_kan_69_condition_with_watermark(
        required_confirmations: u8,
        confirmation_count: u8,
        movement_anchor: Option<UserLocation>,
        last_evaluated_timestamp: Option<SystemTime>,
    ) -> Arc<dyn StepAdvanceCondition> {
        Arc::new(DistanceEntryAndExitWithUTurnConfirmationCondition {
            distance_to_end_of_step: 30,
            distance_after_end_of_step: 5,
            minimum_horizontal_accuracy: 32,
            minimum_significant_movement: 5,
            maximum_plausible_speed: 70,
            plausibility_distance_allowance: 10,
            required_confirmations,
            uturn_confirmation_enabled: true,
            candidate_is_uturn: Some(true),
            candidate_successor: Some(Arc::new(DistanceEntryAndSnappedExitCondition {
                distance_to_end_of_step: 30,
                distance_after_end_of_step: 5,
                minimum_horizontal_accuracy: 32,
                has_reached_end_of_current_step: false,
            })),
            confirmation_active: true,
            movement_anchor,
            confirmation_count,
            last_evaluated_timestamp,
        })
    }

    fn expected_active_kan_69(
        required_confirmations: u8,
        movement_anchor: Option<UserLocation>,
        confirmation_count: u8,
        last_evaluated_timestamp: Option<SystemTime>,
    ) -> SerializableStepAdvanceCondition {
        expected_kan_69_wrapper(
            required_confirmations,
            Some(true),
            vec![snapped_successor(false)],
            true,
            movement_anchor,
            confirmation_count,
            last_evaluated_timestamp,
        )
    }

    fn expected_reset_kan_69(
        required_confirmations: u8,
        last_evaluated_timestamp: Option<SystemTime>,
    ) -> SerializableStepAdvanceCondition {
        expected_kan_69_wrapper(
            required_confirmations,
            None,
            vec![],
            false,
            None,
            0,
            last_evaluated_timestamp,
        )
    }

    fn assert_kan_69_result(
        result: StepAdvanceResult,
        should_advance: bool,
        expected: SerializableStepAdvanceCondition,
    ) -> StepAdvanceResult {
        assert_eq!(result.should_advance(), should_advance);
        assert_serialized_condition(result.next_iteration.to_js(), expected);
        result
    }

    fn evaluate_active_kan_69(
        location: UserLocation,
        current_geometry: Vec<GeographicCoordinate>,
        next_geometry: Vec<GeographicCoordinate>,
        required_confirmations: u8,
        confirmation_count: u8,
        anchor: UserLocation,
    ) -> StepAdvanceResult {
        active_kan_69_condition(required_confirmations, confirmation_count, Some(anchor))
            .should_advance_step(kan_69_state_with_geometry(
                location,
                current_geometry,
                next_geometry,
            ))
    }

    fn ordinary_successor(
        has_reached_end_of_current_step: bool,
    ) -> SerializableStepAdvanceCondition {
        SerializableStepAdvanceCondition::DistanceEntryExit {
            distance_to_end_of_step: 30,
            distance_after_end_step: 5,
            minimum_horizontal_accuracy: 32,
            has_reached_end_of_current_step,
        }
    }

    fn snapped_successor(
        has_reached_end_of_current_step: bool,
    ) -> SerializableStepAdvanceCondition {
        SerializableStepAdvanceCondition::DistanceEntryAndSnappedExit {
            distance_to_end_of_step: 30,
            distance_after_end_step: 5,
            minimum_horizontal_accuracy: 32,
            has_reached_end_of_current_step,
        }
    }

    fn expected_kan_69_wrapper(
        required_confirmations: u8,
        candidate_is_uturn: Option<bool>,
        candidate_successor: Vec<SerializableStepAdvanceCondition>,
        confirmation_active: bool,
        movement_anchor: Option<UserLocation>,
        confirmation_count: u8,
        last_evaluated_timestamp: Option<SystemTime>,
    ) -> SerializableStepAdvanceCondition {
        expected_kan_69_wrapper_with_mode(
            true,
            required_confirmations,
            candidate_is_uturn,
            candidate_successor,
            confirmation_active,
            movement_anchor,
            confirmation_count,
            last_evaluated_timestamp,
        )
    }

    fn expected_kan_69_wrapper_with_mode(
        uturn_confirmation_enabled: bool,
        required_confirmations: u8,
        candidate_is_uturn: Option<bool>,
        candidate_successor: Vec<SerializableStepAdvanceCondition>,
        confirmation_active: bool,
        movement_anchor: Option<UserLocation>,
        confirmation_count: u8,
        last_evaluated_timestamp: Option<SystemTime>,
    ) -> SerializableStepAdvanceCondition {
        SerializableStepAdvanceCondition::DistanceEntryAndExitWithUTurnConfirmation {
            distance_to_end_of_step: 30,
            distance_after_end_step: 5,
            minimum_horizontal_accuracy: 32,
            minimum_significant_movement: 5,
            maximum_plausible_speed: 70,
            plausibility_distance_allowance: 10,
            required_confirmations,
            uturn_confirmation_enabled,
            candidate_is_uturn,
            candidate_successor,
            confirmation_active,
            movement_anchor,
            confirmation_count,
            last_evaluated_timestamp,
        }
    }

    fn assert_serialized_condition(
        actual: SerializableStepAdvanceCondition,
        expected: SerializableStepAdvanceCondition,
    ) {
        match (actual, expected) {
            (
                SerializableStepAdvanceCondition::DistanceEntryAndExitWithUTurnConfirmation {
                    distance_to_end_of_step: actual_end,
                    distance_after_end_step: actual_after,
                    minimum_horizontal_accuracy: actual_accuracy,
                    minimum_significant_movement: actual_movement,
                    maximum_plausible_speed: actual_speed,
                    plausibility_distance_allowance: actual_allowance,
                    required_confirmations: actual_required,
                    uturn_confirmation_enabled: actual_enabled,
                    candidate_is_uturn: actual_kind,
                    candidate_successor: actual_successor,
                    confirmation_active: actual_active,
                    movement_anchor: actual_anchor,
                    confirmation_count: actual_count,
                    last_evaluated_timestamp: actual_timestamp,
                },
                SerializableStepAdvanceCondition::DistanceEntryAndExitWithUTurnConfirmation {
                    distance_to_end_of_step: expected_end,
                    distance_after_end_step: expected_after,
                    minimum_horizontal_accuracy: expected_accuracy,
                    minimum_significant_movement: expected_movement,
                    maximum_plausible_speed: expected_speed,
                    plausibility_distance_allowance: expected_allowance,
                    required_confirmations: expected_required,
                    uturn_confirmation_enabled: expected_enabled,
                    candidate_is_uturn: expected_kind,
                    candidate_successor: expected_successor,
                    confirmation_active: expected_active,
                    movement_anchor: expected_anchor,
                    confirmation_count: expected_count,
                    last_evaluated_timestamp: expected_timestamp,
                },
            ) => {
                assert_eq!(
                    (
                        actual_end,
                        actual_after,
                        actual_accuracy,
                        actual_movement,
                        actual_speed,
                        actual_allowance,
                        actual_required,
                        actual_enabled,
                    ),
                    (
                        expected_end,
                        expected_after,
                        expected_accuracy,
                        expected_movement,
                        expected_speed,
                        expected_allowance,
                        expected_required,
                        expected_enabled,
                    ),
                );
                assert_eq!(actual_kind, expected_kind);
                assert_eq!(
                    serde_json::to_value(actual_successor).unwrap(),
                    serde_json::to_value(expected_successor).unwrap(),
                );
                assert_eq!(actual_active, expected_active);
                assert_eq!(actual_anchor, expected_anchor);
                assert_eq!(actual_count, expected_count);
                assert_eq!(actual_timestamp, expected_timestamp);
            }
            (actual, expected) => {
                panic!(
                    "expected KAN-69 wrapper pair, got actual={actual:?}, \
                     expected={expected:?}"
                );
            }
        }
    }

    fn canonical_distance_metres(a: GeographicCoordinate, b: GeographicCoordinate) -> f64 {
        let d_lat = (b.lat - a.lat).to_radians();
        let d_lng = (b.lng - a.lng).to_radians();
        let value = (d_lat / 2.0).sin().powi(2)
            + a.lat.to_radians().cos() * b.lat.to_radians().cos() * (d_lng / 2.0).sin().powi(2);
        6_371_000.0 * 2.0 * value.sqrt().atan2((1.0 - value).sqrt())
    }

    fn canonical_bearing_degrees(a: GeographicCoordinate, b: GeographicCoordinate) -> f64 {
        let start_lat = a.lat.to_radians();
        let end_lat = b.lat.to_radians();
        let delta_lng = (b.lng - a.lng).to_radians();
        let y = delta_lng.sin() * end_lat.cos();
        let x = start_lat.cos() * end_lat.sin() - start_lat.sin() * end_lat.cos() * delta_lng.cos();
        (y.atan2(x).to_degrees() + 360.0) % 360.0
    }

    fn legacy_swift_bearing_degrees(a: GeographicCoordinate, b: GeographicCoordinate) -> f64 {
        let start_lat = a.lat * std::f64::consts::PI / 180.0;
        let end_lat = b.lat * std::f64::consts::PI / 180.0;
        let delta_lng = (b.lng - a.lng) * std::f64::consts::PI / 180.0;
        let y = delta_lng.sin() * end_lat.cos();
        let x = start_lat.cos() * end_lat.sin() - start_lat.sin() * end_lat.cos() * delta_lng.cos();
        (y.atan2(x) * 180.0 / std::f64::consts::PI + 360.0) % 360.0
    }

    fn canonical_angular_distance(a: f64, b: f64) -> f64 {
        (((a - b + 540.0) % 360.0) - 180.0).abs()
    }

    #[test]
    fn kan_69_baseline_snapped_candidate_advances_before_motion_confirmation() {
        let condition: Arc<dyn StepAdvanceCondition> =
            Arc::new(DistanceEntryAndSnappedExitCondition {
                distance_to_end_of_step: 30,
                distance_after_end_of_step: 5,
                minimum_horizontal_accuracy: 32,
                has_reached_end_of_current_step: false,
            });
        let armed_fix = kan_69_location(41.00100, 69.00000, 0, 5.0);
        let armed =
            condition.should_advance_step(kan_69_state(armed_fix, Some(ManeuverModifier::UTurn)));
        assert!(!armed.should_advance());
        let premature = armed.next_iteration.should_advance_step(kan_69_state(
            kan_69_location(41.00094, 69.000155, 1, 5.0),
            Some(ManeuverModifier::UTurn),
        ));
        assert!(premature.should_advance());
    }

    #[test]
    fn kan_69_uturn_requires_two_raw_movement_confirmations() {
        let condition = step_advance_distance_entry_and_exit_with_uturn_confirmation(
            30, 5, 32, 5, 70, 10, 2, true,
        );
        let entry_fix = kan_69_location(41.00100, 69.00000, 0, 5.0);
        let entry =
            condition.should_advance_step(kan_69_state(entry_fix, Some(ManeuverModifier::UTurn)));
        assert!(!entry.should_advance());
        assert_serialized_condition(
            entry.next_iteration.to_js(),
            expected_kan_69_wrapper(
                2,
                Some(true),
                vec![snapped_successor(true)],
                false,
                None,
                0,
                Some(entry_fix.timestamp),
            ),
        );

        let candidate_fix = kan_69_location(41.00094, 69.000155, 1, 5.0);
        let candidate = entry
            .next_iteration
            .should_advance_step(kan_69_state(candidate_fix, Some(ManeuverModifier::UTurn)));
        assert!(!candidate.should_advance());
        assert_serialized_condition(
            candidate.next_iteration.to_js(),
            expected_kan_69_wrapper(
                2,
                Some(true),
                vec![snapped_successor(false)],
                true,
                Some(candidate_fix),
                0,
                Some(candidate_fix.timestamp),
            ),
        );

        let first_fix = kan_69_location(41.00088, 69.000155, 2, 5.0);
        let first = candidate
            .next_iteration
            .should_advance_step(kan_69_state(first_fix, None));
        assert!(!first.should_advance());
        assert_serialized_condition(
            first.next_iteration.to_js(),
            expected_kan_69_wrapper(
                2,
                Some(true),
                vec![snapped_successor(false)],
                true,
                Some(first_fix),
                1,
                Some(first_fix.timestamp),
            ),
        );

        let second_fix = kan_69_location(41.00082, 69.000155, 3, 5.0);
        let second = first
            .next_iteration
            .should_advance_step(kan_69_state(second_fix, None));
        assert!(second.should_advance());
        assert_serialized_condition(
            second.next_iteration.to_js(),
            expected_kan_69_wrapper(2, None, vec![], false, None, 0, Some(second_fix.timestamp)),
        );
    }

    #[test]
    fn kan_69_modifier_switch_before_latch_reselects_candidate() {
        let condition = step_advance_distance_entry_and_exit_with_uturn_confirmation(
            30, 5, 32, 5, 70, 10, 2, true,
        );
        let uturn_fix = kan_69_location(41.001, 69.0, 0, 5.0);
        let uturn =
            condition.should_advance_step(kan_69_state(uturn_fix, Some(ManeuverModifier::UTurn)));
        let uturn = assert_kan_69_result(
            uturn,
            false,
            expected_kan_69_wrapper(
                2,
                Some(true),
                vec![snapped_successor(true)],
                false,
                None,
                0,
                Some(uturn_fix.timestamp),
            ),
        );

        let ordinary_fix = kan_69_location(41.001, 69.0, 1, 5.0);
        let ordinary = uturn
            .next_iteration
            .should_advance_step(kan_69_state(ordinary_fix, None));
        assert_kan_69_result(
            ordinary,
            false,
            expected_kan_69_wrapper(
                2,
                Some(false),
                vec![ordinary_successor(true)],
                false,
                None,
                0,
                Some(ordinary_fix.timestamp),
            ),
        );
    }

    #[test]
    fn kan_69_outbound_interval_resets_confirmation() {
        let candidate_fix = kan_69_location(41.00094, 69.000155, 1, 5.0);
        let return_fix = kan_69_location(41.00088, 69.000155, 2, 5.0);
        let returned = active_kan_69_condition(2, 0, Some(candidate_fix))
            .should_advance_step(kan_69_state(return_fix, None));
        let returned = assert_kan_69_result(
            returned,
            false,
            expected_active_kan_69(2, Some(return_fix), 1, Some(return_fix.timestamp)),
        );

        let outbound_fix = kan_69_location(41.00094, 69.000155, 3, 5.0);
        let outbound = returned
            .next_iteration
            .should_advance_step(kan_69_state(outbound_fix, None));
        assert_kan_69_result(
            outbound,
            false,
            expected_active_kan_69(2, Some(outbound_fix), 0, Some(outbound_fix.timestamp)),
        );
    }

    #[test]
    fn kan_69_duplicate_timestamp_is_noop_and_distinct_fix_progresses() {
        let candidate_fix = kan_69_location(41.00094, 69.000155, 1, 5.0);
        let repeated_timestamp_fix = kan_69_location(41.00088, 69.000155, 1, 5.0);
        let repeated = active_kan_69_condition(2, 1, Some(candidate_fix))
            .should_advance_step(kan_69_state(repeated_timestamp_fix, None));
        let repeated = assert_kan_69_result(
            repeated,
            false,
            expected_active_kan_69(2, Some(candidate_fix), 1, Some(candidate_fix.timestamp)),
        );

        let later_fix = kan_69_location(41.00082, 69.000155, 2, 5.0);
        let advanced = repeated
            .next_iteration
            .should_advance_step(kan_69_state(later_fix, None));
        let advanced = assert_kan_69_result(
            advanced,
            true,
            expected_reset_kan_69(2, Some(later_fix.timestamp)),
        );

        let duplicate_after_advance = kan_69_location(41.00076, 69.000155, 2, 5.0);
        let still_reset = advanced
            .next_iteration
            .should_advance_step(kan_69_state(duplicate_after_advance, None));
        let still_reset = assert_kan_69_result(
            still_reset,
            false,
            expected_reset_kan_69(2, Some(later_fix.timestamp)),
        );

        let post_reset_fix = kan_69_location(41.00082, 69.000155, 3, 5.0);
        let post_reset = still_reset
            .next_iteration
            .should_advance_step(kan_69_state(post_reset_fix, None));
        assert_kan_69_result(
            post_reset,
            false,
            expected_kan_69_wrapper(
                2,
                Some(false),
                vec![ordinary_successor(true)],
                false,
                None,
                0,
                Some(post_reset_fix.timestamp),
            ),
        );
    }

    #[test]
    fn kan_69_new_instance_resets_complete_transient_state() {
        let active_fix = kan_69_location(41.00088, 69.000155, 2, 5.0);
        assert_serialized_condition(
            active_kan_69_condition(2, 1, Some(active_fix))
                .new_instance()
                .to_js(),
            expected_reset_kan_69(2, Some(active_fix.timestamp)),
        );
    }

    #[test]
    fn kan_69_numeric_value_domain_cases() {
        let anchor = kan_69_location(0.0, 0.0, 0, 5.0);
        let current = kan_69_equatorial_current();
        let next = kan_69_equatorial_next();

        for lat in [
            f64::NAN,
            f64::NEG_INFINITY,
            f64::INFINITY,
            -90.0000001,
            90.0000001,
        ] {
            let result = evaluate_active_kan_69(
                kan_69_location(lat, 0.0, 1, 5.0),
                current.clone(),
                next.clone(),
                2,
                1,
                anchor,
            );
            assert_kan_69_result(
                result,
                false,
                expected_active_kan_69(
                    2,
                    None,
                    0,
                    Some(UNIX_EPOCH + Duration::from_secs(KAN_69_T0_SECONDS + 1)),
                ),
            );
        }

        for lat in [-90.0, 90.0] {
            assert!(is_valid_coordinate(kan_69_coordinate(lat, 0.0)));
            let result = evaluate_active_kan_69(
                kan_69_location(lat, 0.0, 1, 5.0),
                current.clone(),
                next.clone(),
                2,
                1,
                anchor,
            );
            assert_kan_69_result(
                result,
                false,
                expected_active_kan_69(
                    2,
                    None,
                    0,
                    Some(UNIX_EPOCH + Duration::from_secs(KAN_69_T0_SECONDS + 1)),
                ),
            );
        }

        for lng in [
            f64::NAN,
            f64::NEG_INFINITY,
            f64::INFINITY,
            -180.0000001,
            180.0000001,
        ] {
            let result = evaluate_active_kan_69(
                kan_69_location(0.0, lng, 1, 5.0),
                current.clone(),
                next.clone(),
                2,
                1,
                anchor,
            );
            assert_kan_69_result(
                result,
                false,
                expected_active_kan_69(
                    2,
                    None,
                    0,
                    Some(UNIX_EPOCH + Duration::from_secs(KAN_69_T0_SECONDS + 1)),
                ),
            );
        }

        for lng in [-180.0, 180.0] {
            assert!(is_valid_coordinate(kan_69_coordinate(0.0, lng)));
            let result = evaluate_active_kan_69(
                kan_69_location(0.0, lng, 1, 5.0),
                current.clone(),
                next.clone(),
                2,
                1,
                anchor,
            );
            assert_kan_69_result(
                result,
                false,
                expected_active_kan_69(
                    2,
                    None,
                    0,
                    Some(UNIX_EPOCH + Duration::from_secs(KAN_69_T0_SECONDS + 1)),
                ),
            );
        }

        for accuracy in [-1.0, 33.0, f64::NAN, f64::NEG_INFINITY, f64::INFINITY] {
            let result = evaluate_active_kan_69(
                kan_69_location(0.0, 0.0, 1, accuracy),
                current.clone(),
                next.clone(),
                2,
                1,
                anchor,
            );
            assert_kan_69_result(
                result,
                false,
                expected_active_kan_69(
                    2,
                    None,
                    0,
                    Some(UNIX_EPOCH + Duration::from_secs(KAN_69_T0_SECONDS + 1)),
                ),
            );
        }

        for accuracy in [-0.0, 0.0, 32.0] {
            let location = kan_69_location(0.0, 0.0, 1, accuracy);
            assert!(
                DistanceEntryAndExitWithUTurnConfirmationCondition {
                    distance_to_end_of_step: 30,
                    distance_after_end_of_step: 5,
                    minimum_horizontal_accuracy: 32,
                    minimum_significant_movement: 5,
                    maximum_plausible_speed: 70,
                    plausibility_distance_allowance: 10,
                    required_confirmations: 2,
                    uturn_confirmation_enabled: true,
                    candidate_is_uturn: None,
                    candidate_successor: None,
                    confirmation_active: false,
                    movement_anchor: None,
                    confirmation_count: 0,
                    last_evaluated_timestamp: None,
                }
                .is_valid_fix(location)
            );
            let result =
                evaluate_active_kan_69(location, current.clone(), next.clone(), 2, 1, anchor);
            assert_kan_69_result(
                result,
                false,
                expected_active_kan_69(2, Some(anchor), 1, Some(location.timestamp)),
            );
        }

        let equal_time = kan_69_location(0.0, 0.0, 0, 5.0);
        let mut backward_time = equal_time;
        backward_time.timestamp = backward_time
            .timestamp
            .checked_sub(Duration::from_millis(1))
            .unwrap();
        let equal_result =
            evaluate_active_kan_69(equal_time, current.clone(), next.clone(), 2, 1, anchor);
        assert_kan_69_result(
            equal_result,
            false,
            expected_active_kan_69(2, Some(anchor), 1, Some(anchor.timestamp)),
        );

        let distinct_prior_watermark = anchor
            .timestamp
            .checked_add(Duration::from_secs(1))
            .unwrap();
        let zero_duration_result = active_kan_69_condition_with_watermark(
            2,
            1,
            Some(anchor),
            Some(distinct_prior_watermark),
        )
        .should_advance_step(kan_69_state_with_geometry(
            equal_time,
            current.clone(),
            next.clone(),
        ));
        assert_kan_69_result(
            zero_duration_result,
            false,
            expected_active_kan_69(2, None, 0, Some(equal_time.timestamp)),
        );

        let backward_result =
            evaluate_active_kan_69(backward_time, current.clone(), next.clone(), 2, 1, anchor);
        assert_kan_69_result(
            backward_result,
            false,
            expected_active_kan_69(2, None, 0, Some(backward_time.timestamp)),
        );

        let plausible = kan_69_location(0.00071945728473498446, 0.0, 1, 5.0);
        let plausible_result =
            evaluate_active_kan_69(plausible, current.clone(), next.clone(), 2, 1, anchor);
        assert_kan_69_result(
            plausible_result,
            false,
            expected_active_kan_69(2, Some(plausible), 0, Some(plausible.timestamp)),
        );

        let implausible = kan_69_location(0.00071946627795104368, 0.0, 1, 5.0);
        let implausible_result =
            evaluate_active_kan_69(implausible, current.clone(), next.clone(), 2, 1, anchor);
        assert_kan_69_result(
            implausible_result,
            false,
            expected_active_kan_69(2, None, 0, Some(implausible.timestamp)),
        );

        let threshold_fix = kan_69_location(0.000044966080295936529, 0.0, 1, 5.0);
        for (bad_current, bad_next) in [
            (vec![], vec![]),
            (
                vec![kan_69_coordinate(0.0, 0.0), kan_69_coordinate(0.0, 0.0)],
                vec![kan_69_coordinate(0.0, 0.0), kan_69_coordinate(0.0, 0.0)],
            ),
        ] {
            let result = evaluate_active_kan_69(threshold_fix, bad_current, bad_next, 2, 1, anchor);
            assert_kan_69_result(
                result,
                false,
                expected_active_kan_69(2, None, 0, Some(threshold_fix.timestamp)),
            );
        }

        let return_fix = kan_69_location(-0.000044966080295936529, 0.0, 1, 5.0);
        let prefix = evaluate_active_kan_69(
            return_fix,
            vec![
                kan_69_coordinate(f64::NAN, 0.0),
                kan_69_coordinate(-0.001, 0.0),
                kan_69_coordinate(0.0, 0.0),
            ],
            next.clone(),
            2,
            1,
            anchor,
        );
        assert_kan_69_result(
            prefix,
            true,
            expected_reset_kan_69(2, Some(return_fix.timestamp)),
        );

        let suffix = evaluate_active_kan_69(
            return_fix,
            vec![
                kan_69_coordinate(-0.001, 0.0),
                kan_69_coordinate(0.0, 0.0),
                kan_69_coordinate(f64::NAN, 0.0),
            ],
            vec![
                kan_69_coordinate(0.0, 0.0),
                kan_69_coordinate(-0.001, 0.0),
                kan_69_coordinate(f64::NAN, 0.0),
            ],
            2,
            1,
            anchor,
        );
        assert_kan_69_result(
            suffix,
            true,
            expected_reset_kan_69(2, Some(return_fix.timestamp)),
        );

        let zero_fix = kan_69_location(0.0, 0.0, 1, 5.0);
        let zero = step_advance_distance_entry_and_exit_with_uturn_confirmation(
            30, 5, 32, 5, 70, 10, 0, true,
        )
        .should_advance_step(kan_69_state_with_geometry(
            zero_fix,
            current.clone(),
            next.clone(),
        ));
        assert_kan_69_result(
            zero,
            false,
            expected_reset_kan_69(0, Some(zero_fix.timestamp)),
        );

        let saturated = evaluate_active_kan_69(return_fix, current, next, u8::MAX, u8::MAX, anchor);
        assert_kan_69_result(
            saturated,
            true,
            expected_reset_kan_69(u8::MAX, Some(return_fix.timestamp)),
        );
    }

    #[test]
    fn kan_69_distance_uses_canonical_radius() {
        let anchor = kan_69_location(0.0, 0.0, 0, 5.0);
        let boundary = kan_69_location(0.000044966053316288351, 0.0, 1, 5.0);
        let canonical_distance =
            canonical_distance_metres(anchor.coordinates, boundary.coordinates);
        let default_radius_distance = canonical_distance * 6_371_008.8 / 6_371_000.0;
        assert!((canonical_distance - 4.999997).abs() < 1e-12);
        assert!((default_radius_distance - 5.000003906290001).abs() < 1e-12);
        assert!(canonical_distance < 5.0);
        assert!(default_radius_distance > 5.0);

        let result = evaluate_active_kan_69(
            boundary,
            kan_69_equatorial_current(),
            kan_69_equatorial_next(),
            2,
            1,
            anchor,
        );
        assert_kan_69_result(
            result,
            false,
            expected_active_kan_69(2, Some(anchor), 1, Some(boundary.timestamp)),
        );
    }

    #[test]
    fn kan_69_distance_uses_canonical_atan2_at_plausibility_boundary() {
        let anchor = kan_69_location(0.0, 0.0, 0, 5.0);
        let elapsed = Duration::new(1, 123_456_789);
        let mut boundary = kan_69_location(f64::from_bits(4_560_491_878_112_033_671), 0.0, 0, 5.0);
        boundary.timestamp = anchor.timestamp.checked_add(elapsed).unwrap();

        let canonical_distance =
            canonical_distance_metres(anchor.coordinates, boundary.coordinates);
        let geo_0_33_1_distance = geo::HaversineMeasure::new(UZMAP_EARTH_RADIUS_METRES).distance(
            Point::from(anchor.coordinates),
            Point::from(boundary.coordinates),
        );
        let plausible_distance = 70.0 * elapsed.as_secs_f64() + 10.0;

        assert_eq!(canonical_distance.to_bits(), 4_635_938_041_415_232_588);
        assert_eq!(geo_0_33_1_distance.to_bits(), 4_635_938_041_415_232_587);
        assert_eq!(plausible_distance.to_bits(), 4_635_938_041_415_232_587);
        assert!(canonical_distance > plausible_distance);
        assert_eq!(geo_0_33_1_distance, plausible_distance);

        let result = evaluate_active_kan_69(
            boundary,
            kan_69_equatorial_current(),
            kan_69_equatorial_next(),
            2,
            1,
            anchor,
        );
        assert_kan_69_result(
            result,
            false,
            expected_active_kan_69(2, None, 0, Some(boundary.timestamp)),
        );
    }

    #[test]
    fn kan_69_bearing_uses_canonical_subtraction_order_at_decision_boundary() {
        let anchor = kan_69_location(41.0, 170.0, 0, 5.0);
        let current_fix = kan_69_location(41.001, 170.001, 2, 5.0);
        let current_end = kan_69_coordinate(41.002, 170.0002);
        let next_end = kan_69_coordinate(41.004, 170.00739120406232);
        let current_geometry = vec![anchor.coordinates, current_end];
        let next_geometry = vec![current_end, next_end];

        let canonical_movement =
            canonical_bearing_degrees(anchor.coordinates, current_fix.coordinates);
        let canonical_current = canonical_bearing_degrees(anchor.coordinates, current_end);
        let canonical_next = canonical_bearing_degrees(current_end, next_end);
        let canonical_current_distance =
            canonical_angular_distance(canonical_movement, canonical_current);
        let canonical_next_distance =
            canonical_angular_distance(canonical_movement, canonical_next);

        let geo_bearing = |origin: GeographicCoordinate, destination: GeographicCoordinate| {
            geo::HaversineMeasure::new(UZMAP_EARTH_RADIUS_METRES)
                .bearing(Point::from(origin), Point::from(destination))
                .rem_euclid(360.0)
        };
        let geo_movement = geo_bearing(anchor.coordinates, current_fix.coordinates);
        let geo_current = geo_bearing(anchor.coordinates, current_end);
        let geo_next = geo_bearing(current_end, next_end);
        let geo_current_distance = canonical_angular_distance(geo_movement, geo_current);
        let geo_next_distance = canonical_angular_distance(geo_movement, geo_next);
        let legacy_swift_movement =
            legacy_swift_bearing_degrees(anchor.coordinates, current_fix.coordinates);
        let legacy_swift_current = legacy_swift_bearing_degrees(anchor.coordinates, current_end);
        let legacy_swift_next = legacy_swift_bearing_degrees(current_end, next_end);
        let legacy_swift_current_distance =
            canonical_angular_distance(legacy_swift_movement, legacy_swift_current);
        let legacy_swift_next_distance =
            canonical_angular_distance(legacy_swift_movement, legacy_swift_next);

        assert!(canonical_distance_metres(anchor.coordinates, current_fix.coordinates) <= 150.0);
        assert!(canonical_next_distance < canonical_current_distance);
        assert!(geo_next_distance > geo_current_distance);
        assert!(legacy_swift_next_distance > legacy_swift_current_distance);

        let result =
            evaluate_active_kan_69(current_fix, current_geometry, next_geometry, 2, 1, anchor);
        assert_kan_69_result(
            result,
            true,
            expected_reset_kan_69(2, Some(current_fix.timestamp)),
        );
    }

    #[test]
    fn kan_69_sub_5_movement_preserves_anchor() {
        let anchor = kan_69_location(0.0, 0.0, 0, 5.0);
        let below = kan_69_location(0.000044957087079877343, 0.0, 1, 5.0);
        let threshold = kan_69_location(0.000044966080295936529, 0.0, 1, 5.0);
        assert!(
            (canonical_distance_metres(anchor.coordinates, below.coordinates) - 4.999).abs()
                < 1e-12
        );
        assert!(
            (canonical_distance_metres(anchor.coordinates, threshold.coordinates) - 5.0).abs()
                < 1e-12
        );

        let below_result = evaluate_active_kan_69(
            below,
            kan_69_equatorial_current(),
            kan_69_equatorial_next(),
            2,
            1,
            anchor,
        );
        assert_kan_69_result(
            below_result,
            false,
            expected_active_kan_69(2, Some(anchor), 1, Some(below.timestamp)),
        );

        let threshold_result = evaluate_active_kan_69(
            threshold,
            kan_69_equatorial_current(),
            kan_69_equatorial_next(),
            2,
            1,
            anchor,
        );
        assert_kan_69_result(
            threshold_result,
            false,
            expected_active_kan_69(2, Some(threshold), 0, Some(threshold.timestamp)),
        );
    }

    #[test]
    fn kan_69_bearing_strict_and_equal_branches() {
        let candidate_fix = kan_69_location(41.00094, 69.000155, 1, 5.0);
        let return_fix = kan_69_location(41.00088, 69.000155, 2, 5.0);
        let next_result = active_kan_69_condition(2, 1, Some(candidate_fix))
            .should_advance_step(kan_69_state(return_fix, None));
        assert_kan_69_result(
            next_result,
            true,
            expected_reset_kan_69(2, Some(return_fix.timestamp)),
        );

        let outbound_fix = kan_69_location(41.00094, 69.000155, 3, 5.0);
        let current_result = active_kan_69_condition(2, 1, Some(return_fix))
            .should_advance_step(kan_69_state(outbound_fix, None));
        assert_kan_69_result(
            current_result,
            false,
            expected_active_kan_69(2, Some(outbound_fix), 0, Some(outbound_fix.timestamp)),
        );

        let equal_anchor = kan_69_location(0.0, 0.0, 0, 5.0);
        let equal_fix = kan_69_location(0.0, 0.000071945728473498446, 1, 5.0);
        let current_bearing =
            canonical_bearing_degrees(kan_69_coordinate(-0.001, 0.0), kan_69_coordinate(0.0, 0.0));
        let next_bearing =
            canonical_bearing_degrees(kan_69_coordinate(0.0, 0.0), kan_69_coordinate(-0.001, 0.0));
        let movement_bearing =
            canonical_bearing_degrees(equal_anchor.coordinates, equal_fix.coordinates);
        assert!((current_bearing - 0.0).abs() < 1e-12);
        assert!((next_bearing - 180.0).abs() < 1e-12);
        assert!((movement_bearing - 90.0).abs() < 1e-12);
        assert!(
            (canonical_angular_distance(movement_bearing, current_bearing)
                - canonical_angular_distance(movement_bearing, next_bearing))
            .abs()
                < 1e-12
        );

        let equal_result = evaluate_active_kan_69(
            equal_fix,
            kan_69_equatorial_current(),
            kan_69_equatorial_next(),
            2,
            1,
            equal_anchor,
        );
        assert_kan_69_result(
            equal_result,
            false,
            expected_active_kan_69(2, Some(equal_fix), 1, Some(equal_fix.timestamp)),
        );
    }

    #[test]
    fn kan_69_reuses_returned_candidate_successor_between_ticks() {
        let condition = step_advance_distance_entry_and_exit_with_uturn_confirmation(
            30, 5, 32, 5, 70, 10, 2, true,
        );
        let armed_fix = kan_69_location(41.00100, 69.00000, 0, 5.0);
        let armed = condition.should_advance_step(kan_69_state(armed_fix, None));
        assert!(!armed.should_advance());
        assert_serialized_condition(
            armed.next_iteration.to_js(),
            expected_kan_69_wrapper(
                2,
                Some(false),
                vec![ordinary_successor(true)],
                false,
                None,
                0,
                Some(armed_fix.timestamp),
            ),
        );

        let advanced_fix = kan_69_location(41.00094, 69.000155, 1, 5.0);
        let advanced = armed
            .next_iteration
            .should_advance_step(kan_69_state(advanced_fix, None));
        assert!(advanced.should_advance());
        assert_serialized_condition(
            advanced.next_iteration.to_js(),
            expected_kan_69_wrapper(
                2,
                None,
                vec![],
                false,
                None,
                0,
                Some(advanced_fix.timestamp),
            ),
        );
    }

    #[test]
    fn kan_69_disabled_confirmation_uses_ordinary_candidate_for_uturn() {
        let condition = step_advance_distance_entry_and_exit_with_uturn_confirmation(
            30, 5, 32, 5, 70, 10, 2, false,
        );
        let armed_fix = kan_69_location(41.00100, 69.00000, 0, 5.0);
        let armed =
            condition.should_advance_step(kan_69_state(armed_fix, Some(ManeuverModifier::UTurn)));
        let armed = assert_kan_69_result(
            armed,
            false,
            expected_kan_69_wrapper_with_mode(
                false,
                2,
                Some(false),
                vec![ordinary_successor(true)],
                false,
                None,
                0,
                Some(armed_fix.timestamp),
            ),
        );

        let advanced_fix = kan_69_location(41.00094, 69.000155, 1, 5.0);
        assert_kan_69_result(
            armed
                .next_iteration
                .should_advance_step(kan_69_state(advanced_fix, Some(ManeuverModifier::UTurn))),
            true,
            expected_kan_69_wrapper_with_mode(
                false,
                2,
                None,
                vec![],
                false,
                None,
                0,
                Some(advanced_fix.timestamp),
            ),
        );
    }

    fn assert_condition_serializable_round_trip(condition: &Arc<dyn StepAdvanceCondition>) {
        let serialized = condition.to_js();
        let restored: Arc<dyn StepAdvanceCondition> = serialized.clone().into();
        assert_eq!(
            serde_json::to_value(restored.to_js()).unwrap(),
            serde_json::to_value(serialized).unwrap(),
        );
    }

    #[test]
    fn kan_69_serializes_complete_ordinary_candidate_successor() {
        let condition = step_advance_distance_entry_and_exit_with_uturn_confirmation(
            30, 5, 32, 5, 70, 10, 2, true,
        );
        let armed_fix = kan_69_location(41.00100, 69.00000, 0, 5.0);
        let armed = condition.should_advance_step(kan_69_state(armed_fix, None));
        assert_serialized_condition(
            armed.next_iteration.to_js(),
            expected_kan_69_wrapper(
                2,
                Some(false),
                vec![ordinary_successor(true)],
                false,
                None,
                0,
                Some(armed_fix.timestamp),
            ),
        );
        assert_condition_serializable_round_trip(&armed.next_iteration);
    }

    #[test]
    fn kan_69_serializes_complete_active_confirmation_successor() {
        let condition = step_advance_distance_entry_and_exit_with_uturn_confirmation(
            30, 5, 32, 5, 70, 10, 2, true,
        );
        let entry = condition.should_advance_step(kan_69_state(
            kan_69_location(41.00100, 69.00000, 0, 5.0),
            Some(ManeuverModifier::UTurn),
        ));
        let candidate = entry.next_iteration.should_advance_step(kan_69_state(
            kan_69_location(41.00094, 69.000155, 1, 5.0),
            Some(ManeuverModifier::UTurn),
        ));
        let confirmed_once = candidate.next_iteration.should_advance_step(kan_69_state(
            kan_69_location(41.00088, 69.000155, 2, 5.0),
            Some(ManeuverModifier::UTurn),
        ));
        let expected_anchor = kan_69_location(41.00088, 69.000155, 2, 5.0);
        assert_serialized_condition(
            confirmed_once.next_iteration.to_js(),
            expected_kan_69_wrapper(
                2,
                Some(true),
                vec![snapped_successor(false)],
                true,
                Some(expected_anchor),
                1,
                Some(expected_anchor.timestamp),
            ),
        );
        assert_condition_serializable_round_trip(&confirmed_once.next_iteration);
    }

    fn assert_transient_state_is_clean(
        value: SerializableStepAdvanceCondition,
        expected_uturn_confirmation_enabled: bool,
        expected_last_evaluated_timestamp: Option<SystemTime>,
    ) {
        match value {
            SerializableStepAdvanceCondition::DistanceEntryAndExitWithUTurnConfirmation {
                distance_to_end_of_step,
                distance_after_end_step,
                minimum_horizontal_accuracy,
                minimum_significant_movement,
                maximum_plausible_speed,
                plausibility_distance_allowance,
                required_confirmations,
                uturn_confirmation_enabled,
                candidate_is_uturn,
                candidate_successor,
                confirmation_active,
                movement_anchor,
                confirmation_count,
                last_evaluated_timestamp,
            } => {
                assert_eq!(
                    (
                        distance_to_end_of_step,
                        distance_after_end_step,
                        minimum_horizontal_accuracy,
                        minimum_significant_movement,
                        maximum_plausible_speed,
                        plausibility_distance_allowance,
                        required_confirmations,
                        uturn_confirmation_enabled,
                    ),
                    (30, 5, 32, 5, 70, 10, 2, expected_uturn_confirmation_enabled),
                );
                assert_eq!(candidate_is_uturn, None);
                assert!(candidate_successor.is_empty());
                assert!(!confirmation_active);
                assert_eq!(movement_anchor, None);
                assert_eq!(confirmation_count, 0);
                assert_eq!(last_evaluated_timestamp, expected_last_evaluated_timestamp);
            }
            other => panic!("expected KAN-69 condition, got {other:?}"),
        }
    }

    #[test]
    fn kan_69_malformed_serialized_successors_fail_closed() {
        let anchor = Some(kan_69_location(41.00094, 69.000155, 1, 5.0));
        let last_evaluated_timestamp = anchor.map(|fix| fix.timestamp);
        let malformed = [
            (
                expected_kan_69_wrapper(
                    2,
                    Some(true),
                    vec![snapped_successor(true), snapped_successor(false)],
                    true,
                    anchor,
                    1,
                    last_evaluated_timestamp,
                ),
                true,
            ),
            (
                expected_kan_69_wrapper(
                    2,
                    Some(true),
                    vec![ordinary_successor(true)],
                    true,
                    anchor,
                    1,
                    last_evaluated_timestamp,
                ),
                true,
            ),
            (
                expected_kan_69_wrapper(
                    2,
                    Some(true),
                    vec![],
                    true,
                    anchor,
                    1,
                    last_evaluated_timestamp,
                ),
                true,
            ),
            (
                expected_kan_69_wrapper(
                    2,
                    Some(false),
                    vec![snapped_successor(false)],
                    false,
                    None,
                    0,
                    last_evaluated_timestamp,
                ),
                true,
            ),
            (
                expected_kan_69_wrapper(
                    2,
                    Some(false),
                    vec![ordinary_successor(true)],
                    true,
                    anchor,
                    1,
                    last_evaluated_timestamp,
                ),
                true,
            ),
            (
                expected_kan_69_wrapper(
                    2,
                    None,
                    vec![ordinary_successor(false)],
                    false,
                    None,
                    0,
                    last_evaluated_timestamp,
                ),
                true,
            ),
            (
                expected_kan_69_wrapper(
                    2,
                    Some(true),
                    vec![
                        SerializableStepAdvanceCondition::DistanceEntryAndSnappedExit {
                            distance_to_end_of_step: 31,
                            distance_after_end_step: 5,
                            minimum_horizontal_accuracy: 32,
                            has_reached_end_of_current_step: false,
                        },
                    ],
                    false,
                    None,
                    0,
                    last_evaluated_timestamp,
                ),
                true,
            ),
            (
                expected_kan_69_wrapper_with_mode(
                    false,
                    2,
                    Some(true),
                    vec![snapped_successor(false)],
                    true,
                    anchor,
                    1,
                    last_evaluated_timestamp,
                ),
                false,
            ),
        ];

        for (value, expected_uturn_confirmation_enabled) in malformed {
            let normalized: Arc<dyn StepAdvanceCondition> = value.into();
            assert_transient_state_is_clean(
                normalized.to_js(),
                expected_uturn_confirmation_enabled,
                last_evaluated_timestamp,
            );
        }
    }

    #[test]
    fn test_manual_step_advance() {
        let condition = ManualStepCondition;

        let trip_state = get_navigating_trip_state(
            *LOCATION_NEAR_START_OF_STEP,
            vec![STRAIGHT_LINE_SHORT_ROUTE_STEP.clone()],
            vec![],
            RouteDeviation::NoDeviation,
        );

        // Test the condition - we should NOT advance since we're far from the end
        let result = condition.should_advance_step(trip_state);

        // We should never advance to the next step in manual mode,
        // so the list should always be empty.
        assert!(
            !result.should_advance,
            "Should not advance with the manual condition"
        );
    }

    #[test]
    fn test_distance_to_end_of_step_doesnt_advance() {
        // Set up the condition with a distance threshold of 20 meters
        let condition = DistanceToEndOfStepCondition {
            minimum_horizontal_accuracy: 10,
            distance: 20, // Must be within 20 meters of the end to advance
        };

        let trip_state = get_navigating_trip_state(
            *LOCATION_NEAR_START_OF_STEP,
            vec![STRAIGHT_LINE_SHORT_ROUTE_STEP.clone()],
            vec![],
            RouteDeviation::NoDeviation,
        );

        // Test the condition - we should NOT advance since we're far from the end
        let result = condition.should_advance_step(trip_state);

        assert!(
            !result.should_advance,
            "Should not advance when far from the end of the step"
        );
    }

    #[test]
    fn test_distance_to_end_of_step_advance() {
        // Set up the condition with a distance threshold of 20 meters
        let condition = DistanceToEndOfStepCondition {
            minimum_horizontal_accuracy: 10,
            distance: 20, // Must be within 20 meters of the end to advance
        };

        let trip_state = get_navigating_trip_state(
            *LOCATION_NEAR_END_OF_STEP,
            vec![STRAIGHT_LINE_SHORT_ROUTE_STEP.clone()],
            vec![],
            RouteDeviation::NoDeviation,
        );

        // Test the condition - we SHOULD advance since we're close to the end
        let result = condition.should_advance_step(trip_state);

        assert!(
            result.should_advance,
            "Should advance when close to the end of the step"
        );
    }

    #[test]
    fn test_distance_from_step_advance_with_deviation() {
        // Create a location that's far from the route (500+ meters north)
        // At the equator, 0.005° latitude is approximately 555 meters
        let user_location = make_user_location(coord!(x: 0.005, y: 0.0005), 5.0);

        // Set up the condition with a minimum deviation of 100 meters to advance
        let condition = DistanceFromStepCondition {
            minimum_horizontal_accuracy: 10,
            distance: 100, // Must be at least 100 meters from route to advance
            calculation_policy: DeviationCalculationPolicy::Always,
        };

        let trip_state = get_navigating_trip_state(
            user_location,
            vec![STRAIGHT_LINE_SHORT_ROUTE_STEP.clone()],
            vec![],
            RouteDeviation::Deviation {
                kind: DeviationKind::CompletelyOffRoute {
                    deviation_from_route_line: 10.0,
                },
            },
        );

        // Test the condition - we SHOULD advance since we're far from the route
        let result = condition.should_advance_step(trip_state);

        assert!(
            result.should_advance,
            "Should advance when far from the route"
        );
    }

    #[test]
    fn test_distance_from_step_no_advance_when_completely_off_route() {
        // Create a location that's far from the route (500+ meters north)
        // At the equator, 0.005° latitude is approximately 555 meters
        let user_location = make_user_location(coord!(x: 0.005, y: 0.0005), 5.0);

        // Set up the condition with a minimum deviation of 100 meters to advance
        let condition = DistanceFromStepCondition {
            minimum_horizontal_accuracy: 10,
            distance: 100, // Must be at least 100 meters from route to advance
            calculation_policy: DeviationCalculationPolicy::WhileOnRoute,
        };

        let trip_state = get_navigating_trip_state(
            user_location,
            vec![STRAIGHT_LINE_SHORT_ROUTE_STEP.clone()],
            vec![],
            RouteDeviation::Deviation {
                kind: DeviationKind::CompletelyOffRoute {
                    deviation_from_route_line: 10.0,
                },
            },
        );

        // Test the condition - we SHOULD advance since we're far from the route
        let result = condition.should_advance_step(trip_state);

        assert!(
            !result.should_advance,
            "WhileOnRoute should not advance when the user is completely off the route"
        );
    }

    #[test]
    fn test_distance_from_step_advance() {
        // Create a location that's far from the route (500+ meters north)
        // At the equator, 0.005° latitude is approximately 555 meters
        let user_location = make_user_location(coord!(x: 0.005, y: 0.0005), 5.0);

        // Set up the condition with a minimum deviation of 100 meters to advance
        let condition = DistanceFromStepCondition {
            minimum_horizontal_accuracy: 10,
            distance: 100, // Must be at least 100 meters from route to advance
            calculation_policy: DeviationCalculationPolicy::WhileOnRoute,
        };

        let trip_state = get_navigating_trip_state(
            user_location,
            vec![STRAIGHT_LINE_SHORT_ROUTE_STEP.clone()],
            vec![],
            RouteDeviation::NoDeviation,
        );

        // Test the condition - we SHOULD advance since we're far from the route
        let result = condition.should_advance_step(trip_state);

        assert!(
            result.should_advance,
            "Should advance when far from the route"
        );
    }

    // Combination Rules

    #[test]
    fn test_or_condition_doesnt_advance() {
        // Create two false conditions - both manual step advance
        let manual_condition1 = ManualStepCondition;
        let manual_condition2 = ManualStepCondition;

        // Create an OR condition - should only advance when at least one condition is true
        let or_condition = OrAdvanceConditions {
            conditions: vec![Arc::new(manual_condition1), Arc::new(manual_condition2)],
        };

        let trip_state = get_navigating_trip_state(
            *LOCATION_NEAR_START_OF_STEP,
            vec![STRAIGHT_LINE_SHORT_ROUTE_STEP.clone()],
            vec![],
            RouteDeviation::NoDeviation,
        );

        // Test the condition - we should NOT advance since both conditions are false
        let result = or_condition.should_advance_step(trip_state);

        assert!(
            !result.should_advance,
            "Should not advance when all OR conditions are false"
        );
    }

    #[test]
    fn test_or_condition_advance() {
        // Create a false condition
        let manual_condition = ManualStepCondition;

        // Create a true condition - distance to end of step
        let distance_condition = DistanceToEndOfStepCondition {
            minimum_horizontal_accuracy: 10,
            distance: 20, // Must be within 20 meters of the end to advance
        };

        // Create an OR condition - should advance when any condition is true
        let or_condition = OrAdvanceConditions {
            conditions: vec![Arc::new(manual_condition), Arc::new(distance_condition)],
        };

        let trip_state = get_navigating_trip_state(
            *LOCATION_NEAR_END_OF_STEP,
            vec![STRAIGHT_LINE_SHORT_ROUTE_STEP.clone()],
            vec![],
            RouteDeviation::NoDeviation,
        );

        // Test the condition - we SHOULD advance since one condition is true
        let result = or_condition.should_advance_step(trip_state);

        assert!(
            result.should_advance,
            "Should advance when at least one OR condition is true"
        );
    }

    #[test]
    fn test_and_condition_doesnt_advance() {
        // Create a false condition
        let manual_condition = ManualStepCondition;

        // Create a true condition - distance to end of step
        let distance_condition = DistanceToEndOfStepCondition {
            minimum_horizontal_accuracy: 10,
            distance: 20, // Must be within 20 meters of the end to advance
        };

        // Create an AND condition - should only advance when all conditions are true
        let and_condition = AndAdvanceConditions {
            conditions: vec![
                Arc::new(manual_condition),   // This will always be false
                Arc::new(distance_condition), // This will be true
            ],
        };

        let trip_state = get_navigating_trip_state(
            *LOCATION_NEAR_END_OF_STEP,
            vec![STRAIGHT_LINE_SHORT_ROUTE_STEP.clone()],
            vec![],
            RouteDeviation::NoDeviation,
        );

        // Test the condition - we should NOT advance since one condition is false
        let result = and_condition.should_advance_step(trip_state);

        assert!(
            !result.should_advance,
            "Should not advance when at least one AND condition is false"
        );
    }

    #[test]
    fn test_and_condition_advance() {
        // Create two true conditions - both distance to end of step but with different thresholds
        let distance_condition1 = DistanceToEndOfStepCondition {
            minimum_horizontal_accuracy: 10,
            distance: 30, // Must be within 30 meters of the end to advance
        };

        let distance_condition2 = DistanceToEndOfStepCondition {
            minimum_horizontal_accuracy: 10,
            distance: 20, // Must be within 20 meters of the end to advance
        };

        // Create an AND condition - should only advance when all conditions are true
        let and_condition = AndAdvanceConditions {
            conditions: vec![Arc::new(distance_condition1), Arc::new(distance_condition2)],
        };

        let trip_state = get_navigating_trip_state(
            *LOCATION_NEAR_END_OF_STEP,
            vec![STRAIGHT_LINE_SHORT_ROUTE_STEP.clone()],
            vec![],
            RouteDeviation::NoDeviation,
        );

        // Test the condition - we SHOULD advance since both conditions are true
        let result = and_condition.should_advance_step(trip_state);

        assert!(
            result.should_advance,
            "Should advance when all AND conditions are true"
        );
    }

    // Stateful Conditions

    #[test]
    fn test_entry_and_exit_condition_doesnt_advance() {
        // Create a condition that requires proximity to end followed by distance from step
        let condition = DistanceEntryAndExitCondition {
            distance_to_end_of_step: 10,
            distance_after_end_of_step: 20,
            minimum_horizontal_accuracy: 5,
            has_reached_end_of_current_step: false,
        };

        let trip_state = get_navigating_trip_state(
            *LOCATION_NEAR_END_OF_STEP,
            vec![STRAIGHT_LINE_SHORT_ROUTE_STEP.clone()],
            vec![],
            RouteDeviation::NoDeviation,
        );

        // First update: User is close to the end of the step...
        // Should not advance yet, but should update internal state
        let result1 = condition.should_advance_step(trip_state);

        assert!(
            !result1.should_advance,
            "Should not advance on first update even when near end of step"
        );

        // Second update: User has moved but is still too close to the route
        // This is only 2 meters from the end point, not the required 5 meters
        let user_location_still_close = make_user_location(coord!(x: 0.001, y: 0.00002), 5.0);

        // Get the next iteration from the first result
        let next_condition = result1.next_iteration;

        let trip_state2 = get_navigating_trip_state(
            user_location_still_close,
            vec![STRAIGHT_LINE_SHORT_ROUTE_STEP.clone()],
            vec![],
            RouteDeviation::NoDeviation,
        );

        // Should still not advance because we haven't moved far enough away
        let result2 = next_condition.should_advance_step(trip_state2);

        assert!(
            !result2.should_advance,
            "Should not advance when user hasn't moved far enough from the route"
        );
    }

    #[test]
    fn test_entry_and_exit_condition_advance() {
        // Create a condition that requires proximity to end followed by distance from step
        let condition = DistanceEntryAndExitCondition {
            distance_to_end_of_step: 10,
            distance_after_end_of_step: 20,
            minimum_horizontal_accuracy: 5,
            has_reached_end_of_current_step: false,
        };

        let trip_state = get_navigating_trip_state(
            *LOCATION_NEAR_END_OF_STEP,
            vec![STRAIGHT_LINE_SHORT_ROUTE_STEP.clone()],
            vec![],
            RouteDeviation::NoDeviation,
        );

        // First update: User is close to the end of the step...
        // Should not advance yet, but should update internal state
        let result1 = condition.should_advance_step(trip_state);

        assert!(
            !result1.should_advance,
            "Should not advance on first update even when near end of step"
        );

        // Get the next iteration with updated internal state
        let next_condition = result1.next_iteration;

        // Second update: User has moved far enough from the route (> 5 meters)
        // ~55 meters north of the route (0.0005 degrees latitude)
        let user_location_far = make_user_location(coord!(x: 0.001, y: 0.0005), 5.0);

        let trip_state2 = get_navigating_trip_state(
            user_location_far,
            vec![STRAIGHT_LINE_SHORT_ROUTE_STEP.clone()],
            vec![],
            RouteDeviation::NoDeviation,
        );

        // Now should advance because we've satisfied both conditions sequentially
        let result2 = next_condition.should_advance_step(trip_state2);

        assert!(
            result2.should_advance,
            "Should advance when user has first reached end of step and then moved away"
        );
    }

    #[test]
    fn test_and_condition_preserves_state() {
        // Create a stateful condition that we can track
        let entry_exit_condition = DistanceEntryAndExitCondition {
            distance_to_end_of_step: 10,
            distance_after_end_of_step: 5,
            minimum_horizontal_accuracy: 5,
            has_reached_end_of_current_step: false,
        };

        // Create an AND condition with just this one condition for simplicity
        let and_condition = AndAdvanceConditions {
            conditions: vec![Arc::new(entry_exit_condition)],
        };

        let trip_state = get_navigating_trip_state(
            *LOCATION_NEAR_END_OF_STEP,
            vec![STRAIGHT_LINE_SHORT_ROUTE_STEP.clone()],
            vec![],
            RouteDeviation::NoDeviation,
        );

        // First update: User is close to the end of the step
        let result1 = and_condition.should_advance_step(trip_state);

        assert!(
            !result1.should_advance,
            "Should not advance on first update"
        );

        // Get the next iteration and cast it back to check the internal state
        let next_and_condition = result1.next_iteration;

        // Second update: User moves far away - should advance now
        let user_location_far = make_user_location(coord!(x: 0.001, y: 0.0005), 5.0);

        let trip_state2 = get_navigating_trip_state(
            user_location_far,
            vec![STRAIGHT_LINE_SHORT_ROUTE_STEP.clone()],
            vec![],
            RouteDeviation::NoDeviation,
        );

        let result2 = next_and_condition.should_advance_step(trip_state2);

        assert!(
            result2.should_advance,
            "Should advance when stateful condition completes in AND"
        );
    }

    #[test]
    fn test_entry_and_exit_condition_in_and_composite_advance() {
        // Create a condition that requires proximity to end followed by distance from step
        let entry_exit_condition = DistanceEntryAndExitCondition {
            distance_to_end_of_step: 10,
            distance_after_end_of_step: 5,
            minimum_horizontal_accuracy: 5,
            has_reached_end_of_current_step: false,
        };

        // Create a simple condition that's always true when near the end
        let distance_condition = DistanceToEndOfStepCondition {
            minimum_horizontal_accuracy: 10,
            distance: 100, // Increased to 100 meters to account for the test location
        };

        // Create an AND condition combining both
        let and_condition = AndAdvanceConditions {
            conditions: vec![Arc::new(entry_exit_condition), Arc::new(distance_condition)],
        };

        let trip_state = get_navigating_trip_state(
            *LOCATION_NEAR_END_OF_STEP,
            vec![STRAIGHT_LINE_SHORT_ROUTE_STEP.clone()],
            vec![],
            RouteDeviation::NoDeviation,
        );

        // First update: User is close to the end of the step
        // Should not advance yet, but should update internal state of the entry/exit condition
        let result1 = and_condition.should_advance_step(trip_state);

        assert!(
            !result1.should_advance,
            "Should not advance on first update even when near end of step"
        );

        // Get the next iteration with updated internal state
        let next_condition = result1.next_iteration;

        // Second update: User has moved far enough from the route (> 20 meters)
        // ~55 meters north of the route (0.0005 degrees latitude)
        let user_location_far = make_user_location(coord!(x: 0.001, y: 0.0005), 5.0);

        let trip_state2 = get_navigating_trip_state(
            user_location_far,
            vec![STRAIGHT_LINE_SHORT_ROUTE_STEP.clone()],
            vec![],
            RouteDeviation::NoDeviation,
        );

        // Now should advance because the entry/exit condition has maintained its state
        // through the AND composite condition
        let result2 = next_condition.should_advance_step(trip_state2);

        assert!(
            result2.should_advance,
            "Should advance when stateful condition completes within AND composite"
        );
    }

    #[test]
    fn test_entry_and_exit_condition_in_or_composite_advance() {
        // Create a condition that requires proximity to end followed by distance from step
        let entry_exit_condition = DistanceEntryAndExitCondition {
            distance_to_end_of_step: 10,
            distance_after_end_of_step: 5,
            minimum_horizontal_accuracy: 5,
            has_reached_end_of_current_step: false,
        };

        // Create a condition that will never be true (manual)
        let manual_condition = ManualStepCondition;

        // Create an OR condition combining both
        let or_condition = OrAdvanceConditions {
            conditions: vec![Arc::new(entry_exit_condition), Arc::new(manual_condition)],
        };

        let trip_state = get_navigating_trip_state(
            *LOCATION_NEAR_END_OF_STEP,
            vec![STRAIGHT_LINE_SHORT_ROUTE_STEP.clone()],
            vec![],
            RouteDeviation::NoDeviation,
        );

        // First update: User is close to the end of the step
        // Should not advance yet, but should update internal state of the entry/exit condition
        let result1 = or_condition.should_advance_step(trip_state);

        assert!(
            !result1.should_advance,
            "Should not advance on first update even when near end of step"
        );

        // Get the next iteration with updated internal state
        let next_condition = result1.next_iteration;

        // Second update: User has moved far enough from the route (> 20 meters)
        // ~55 meters north of the route (0.0005 degrees latitude)
        let user_location_far = make_user_location(coord!(x: 0.001, y: 0.0005), 5.0);

        let trip_state2 = get_navigating_trip_state(
            user_location_far,
            vec![STRAIGHT_LINE_SHORT_ROUTE_STEP.clone()],
            vec![],
            RouteDeviation::NoDeviation,
        );

        // Now should advance because the entry/exit condition has maintained its state
        // through the OR composite condition
        let result2 = next_condition.should_advance_step(trip_state2);

        assert!(
            result2.should_advance,
            "Should advance when stateful condition completes within OR composite"
        );
    }

    #[test]
    fn test_entry_and_exit_condition_resets_in_and_composite_when_advancing() {
        // Create a condition that requires proximity to end followed by distance from step
        let entry_exit_condition = DistanceEntryAndExitCondition {
            distance_to_end_of_step: 10,
            distance_after_end_of_step: 5,
            minimum_horizontal_accuracy: 5,
            has_reached_end_of_current_step: false,
        };

        // Create a condition that's true for both near and far locations
        let distance_condition = DistanceToEndOfStepCondition {
            minimum_horizontal_accuracy: 10,
            distance: 100, // True for both test locations
        };

        // Create an AND condition combining both
        let and_condition = AndAdvanceConditions {
            conditions: vec![Arc::new(entry_exit_condition), Arc::new(distance_condition)],
        };

        let trip_state = get_navigating_trip_state(
            *LOCATION_NEAR_END_OF_STEP,
            vec![STRAIGHT_LINE_SHORT_ROUTE_STEP.clone()],
            vec![],
            RouteDeviation::NoDeviation,
        );

        // First update: User is close to the end of the step
        // Should not advance yet, but should update internal state of the entry/exit condition
        let result1 = and_condition.should_advance_step(trip_state);

        assert!(
            !result1.should_advance,
            "Should not advance on first update even when near end of step"
        );

        // Get the next iteration with updated internal state
        let next_condition = result1.next_iteration;

        // Second update: User has moved far enough from the route (> 5 meters)
        // ~55 meters north of the route (0.0005 degrees latitude)
        let user_location_far = make_user_location(coord!(x: 0.001, y: 0.0005), 5.0);

        let trip_state2 = get_navigating_trip_state(
            user_location_far,
            vec![STRAIGHT_LINE_SHORT_ROUTE_STEP.clone()],
            vec![],
            RouteDeviation::NoDeviation,
        );

        // Now should advance because the entry/exit condition has maintained its state
        // through the AND composite condition
        let result2 = next_condition.should_advance_step(trip_state2);

        assert!(
            result2.should_advance,
            "Should advance when stateful condition completes within AND composite"
        );

        // The key test: verify that the next iteration after advancing has reset conditions
        let reset_condition = result2.next_iteration;

        let trip_state3 = get_navigating_trip_state(
            *LOCATION_NEAR_END_OF_STEP,
            vec![STRAIGHT_LINE_SHORT_ROUTE_STEP.clone()],
            vec![],
            RouteDeviation::NoDeviation,
        );

        // Third update: User is near the end again, but the entry/exit condition should be reset
        // Since the entry/exit condition is reset, it should start over even though user is at end
        let result3 = reset_condition.should_advance_step(trip_state3);

        assert!(
            !result3.should_advance,
            "Should not advance immediately after reset - entry/exit condition should restart its two-phase process"
        );
    }

    #[test]
    fn test_route_snapped_entry_and_exit_condition_advance() {
        // Create a straight route with two steps
        let step1 = gen_route_step_with_coords(vec![
            coord!(x: 0.0, y: 0.0),   // Start
            coord!(x: 0.001, y: 0.0), // 111m east (end of step 1)
        ]);

        let step2 = gen_route_step_with_coords(vec![
            coord!(x: 0.001, y: 0.0),   // Start of step 2 (same as end of step 1)
            coord!(x: 0.001, y: 0.001), // 111m north
        ]);

        let condition = DistanceEntryAndSnappedExitCondition {
            distance_to_end_of_step: 10,
            distance_after_end_of_step: 5,
            minimum_horizontal_accuracy: 10,
            has_reached_end_of_current_step: false,
        };

        // User near end of step 1
        let location_near_end = make_user_location(coord!(x: 0.00099, y: 0.0), 5.0);
        let trip_state = get_navigating_trip_state(
            location_near_end,
            vec![step1],
            vec![],
            RouteDeviation::NoDeviation,
        );

        // First update: Should enter the zone but not advance
        let result1 = condition.should_advance_step(trip_state);

        assert!(
            !result1.should_advance,
            "Should not advance on first update when entering end zone"
        );

        // Second update: User has turned onto step 2 (10m north of the corner)
        let location_on_step2 = make_user_location(coord!(x: 0.001, y: 0.0001), 5.0);
        let next_condition = result1.next_iteration;
        let trip_state2 = get_navigating_trip_state(
            location_on_step2,
            vec![step2],
            vec![],
            RouteDeviation::NoDeviation,
        );

        let result2 = next_condition.should_advance_step(trip_state2);

        assert!(
            result2.should_advance,
            "Should advance when route-snapped position has moved onto next step"
        );
    }

    #[test]
    fn test_route_snapped_entry_and_exit_with_short_next_step() {
        // Create a route where step 1 is normal length, but step 2 and 3 are short
        let step1 = gen_route_step_with_coords(vec![
            coord!(x: 0.0, y: 0.0),   // Start
            coord!(x: 0.001, y: 0.0), // 111m east (end of step 1)
        ]);

        // Step 2 is short - ~3 meters long
        let step2 = gen_route_step_with_coords(vec![
            coord!(x: 0.001, y: 0.0),      // Start of step 2
            coord!(x: 0.001, y: 0.000027), // ~3m north
        ]);

        // Step 3 is also short - ~3 meters long
        // Total accumulated distance: 3 + 3 = 6m, less than configured 10m
        let step3 = gen_route_step_with_coords(vec![
            coord!(x: 0.001, y: 0.000027), // Start of step 3
            coord!(x: 0.001, y: 0.000054), // ~3m north
        ]);

        // Configure with 10m exit distance, but step 2 + step 3 = only 6m total
        // The algorithm should cap the effective exit distance to min(10, 6) = 6m
        let condition = DistanceEntryAndSnappedExitCondition {
            distance_to_end_of_step: 10,
            distance_after_end_of_step: 10, // Larger than accumulated distance!
            minimum_horizontal_accuracy: 10,
            has_reached_end_of_current_step: false,
        };

        // User near end of step 1
        let location_near_end = make_user_location(coord!(x: 0.00099, y: 0.0), 5.0);
        let trip_state = get_navigating_trip_state(
            location_near_end,
            vec![step1, step2.clone(), step3.clone()],
            vec![],
            RouteDeviation::NoDeviation,
        );

        // First update: Enter the end zone
        let result1 = condition.should_advance_step(trip_state);

        assert!(
            !result1.should_advance,
            "Should not advance on first update when entering end zone"
        );

        // Second update: User has moved 8m north from the turn (well past the short steps)
        // With correct code using min(10, 6), effective_exit_distance is 6m
        // The perpendicular deviation from step1 should exceed 6m, so should advance
        let location_on_step2 = make_user_location(coord!(x: 0.001, y: 0.000072), 5.0);
        let next_condition = result1.next_iteration;
        let trip_state2 = get_navigating_trip_state(
            location_on_step2,
            vec![step2, step3],
            vec![],
            RouteDeviation::NoDeviation,
        );

        let result2 = next_condition.should_advance_step(trip_state2);

        assert!(
            result2.should_advance,
            "Should advance with short next steps using min(configured, accumulated) distance"
        );
    }

    #[test]
    fn test_route_snapped_entry_and_exit_with_zero_length_via_waypoint() {
        // Create a route with a normal step, a 0-length via waypoint step, and another normal step
        let step1 = gen_route_step_with_coords(vec![
            coord!(x: 0.0, y: 0.0),   // Start
            coord!(x: 0.001, y: 0.0), // 111m east (end of step 1)
        ]);

        // Step 2 is a via waypoint with 0 distance (same start and end point)
        let mut step2 = gen_route_step_with_coords(vec![
            coord!(x: 0.001, y: 0.0), // Via waypoint location (duplicated)
            coord!(x: 0.001, y: 0.0), // Same point
        ]);
        step2.distance = 0.0; // Explicitly set to 0 distance

        // Step 3 is a normal step continuing from the via waypoint
        let step3 = gen_route_step_with_coords(vec![
            coord!(x: 0.001, y: 0.0),   // Start (via waypoint)
            coord!(x: 0.001, y: 0.001), // 111m north
        ]);

        let condition = DistanceEntryAndSnappedExitCondition {
            distance_to_end_of_step: 10,
            distance_after_end_of_step: 5,
            minimum_horizontal_accuracy: 10,
            has_reached_end_of_current_step: false,
        };

        // User near end of step 1
        let location_near_end = make_user_location(coord!(x: 0.00099, y: 0.0), 5.0);
        let trip_state = get_navigating_trip_state(
            location_near_end,
            vec![step1, step2.clone(), step3.clone()],
            vec![],
            RouteDeviation::NoDeviation,
        );

        // First update: Should enter the zone but not advance
        let result1 = condition.should_advance_step(trip_state);

        assert!(
            !result1.should_advance,
            "Should not advance on first update when entering end zone"
        );

        // Second update: User has moved onto step 3 (past the 0-length via waypoint)
        // The algorithm should accumulate geometry from step2 (0m) + step3 (111m) to have sufficient distance
        let location_on_step3 = make_user_location(coord!(x: 0.001, y: 0.0001), 5.0);
        let next_condition = result1.next_iteration;
        let trip_state2 = get_navigating_trip_state(
            location_on_step3,
            vec![step2, step3],
            vec![],
            RouteDeviation::NoDeviation,
        );

        let result2 = next_condition.should_advance_step(trip_state2);

        assert!(
            result2.should_advance,
            "Should advance when route-snapped position has moved onto step after 0-length via waypoint"
        );
    }

    #[test]
    fn test_entry_and_exit_condition_resets_in_or_composite_when_advancing() {
        // Create a condition that requires proximity to end followed by distance from step
        let entry_exit_condition = DistanceEntryAndExitCondition {
            distance_to_end_of_step: 10,
            distance_after_end_of_step: 5,
            minimum_horizontal_accuracy: 5,
            has_reached_end_of_current_step: false,
        };

        // Create a condition that will never be true (manual)
        let manual_condition = ManualStepCondition;

        // Create an OR condition combining both
        let or_condition = OrAdvanceConditions {
            conditions: vec![Arc::new(entry_exit_condition), Arc::new(manual_condition)],
        };

        let trip_state = get_navigating_trip_state(
            *LOCATION_NEAR_END_OF_STEP,
            vec![STRAIGHT_LINE_SHORT_ROUTE_STEP.clone()],
            vec![],
            RouteDeviation::NoDeviation,
        );

        // First update: User is close to the end of the step
        // Should not advance yet, but should update internal state of the entry/exit condition
        let result1 = or_condition.should_advance_step(trip_state);

        assert!(
            !result1.should_advance,
            "Should not advance on first update even when near end of step"
        );

        // Get the next iteration with updated internal state
        let next_condition = result1.next_iteration;

        // Second update: User has moved far enough from the route (> 5 meters)
        // ~55 meters north of the route (0.0005 degrees latitude)
        let user_location_far = make_user_location(coord!(x: 0.001, y: 0.0005), 5.0);

        let trip_state2 = get_navigating_trip_state(
            user_location_far,
            vec![STRAIGHT_LINE_SHORT_ROUTE_STEP.clone()],
            vec![],
            RouteDeviation::NoDeviation,
        );

        // Now should advance because the entry/exit condition has maintained its state
        let result2 = next_condition.should_advance_step(trip_state2);

        assert!(
            result2.should_advance,
            "Should advance when stateful condition completes within OR composite"
        );

        // The key test: verify that the next iteration after advancing has reset conditions
        let reset_condition = result2.next_iteration;

        let trip_state3 = get_navigating_trip_state(
            *LOCATION_NEAR_END_OF_STEP,
            vec![STRAIGHT_LINE_SHORT_ROUTE_STEP.clone()],
            vec![],
            RouteDeviation::NoDeviation,
        );

        // Third update: User is near the end again, but the entry/exit condition should be reset
        // Since the entry/exit condition is reset, it should start over even though user is at end
        let result3 = reset_condition.should_advance_step(trip_state3);

        assert!(
            !result3.should_advance,
            "Should not advance immediately after reset - entry/exit condition should restart its two-phase process"
        );
    }
}

#[cfg(test)]
proptest! {
    #[test]
    fn manual_step_never_advances(
        c1 in arb_coord(),
        c2 in arb_coord(),
        accuracy: f64
    ) {
        // Create a straight line random route step
        let route_step =
            crate::navigation_controller::test_helpers::gen_route_step_with_coords(vec![c1, c2]);

        // User location exactly at the end of the step
        let user_location = make_user_location(c2, accuracy);

        let condition = ManualStepCondition;

        let trip_state = get_navigating_trip_state(
            user_location,
            vec![route_step],
            vec![],
            RouteDeviation::NoDeviation,
        );

        // Test the condition - we should NOT advance since we're far from the end
        let result = condition.should_advance_step(trip_state);

        // We should never advance to the next step in manual mode,
        // so the list should always be empty.
        prop_assert!(
            !result.should_advance,
            "Should not advance with the manual condition"
        );
    }

    #[test]
    fn entry_and_exit_never_advances_on_zero_movement(
        c1 in arb_coord(),
        c2 in arb_coord(),
    ) {
        // Create a straight line random route step
        let route_step =
            crate::navigation_controller::test_helpers::gen_route_step_with_coords(vec![c1, c2]);

        // User location exactly at the end of the step
        let user_location = make_user_location(c2, 5.0);

        // Create a condition that requires proximity to end followed by distance from step
        let condition = DistanceEntryAndExitCondition {
            distance_to_end_of_step: 10,
            distance_after_end_of_step: 20,
            minimum_horizontal_accuracy: 5,
            has_reached_end_of_current_step: false,
        };

        let trip_state = get_navigating_trip_state(
            user_location,
            vec![route_step.clone()],
            vec![],
            RouteDeviation::NoDeviation,
        );

        // First update: User is close to the end of the step...
        // Should not advance yet, but should update internal state
        let result1 = condition.should_advance_step(trip_state);

        prop_assert!(
            !result1.should_advance,
            "Should not advance on first update even when at the end of the step"
        );

        // Second update: User has not moved

        // Get the next iteration from the first result
        let next_condition = result1.next_iteration;

        let trip_state2 = get_navigating_trip_state(
            user_location,
            vec![route_step],
            vec![],
            RouteDeviation::NoDeviation,
        );

        // Should still not advance because we haven't moved far enough away
        let result2 = next_condition.should_advance_step(trip_state2);

        prop_assert!(
            !result2.should_advance,
            "Should not advance when user hasn't moved far enough from the route"
        );
    }

    // TODO: handling of accuracy parameter for "always" advance

    // TODO: "or" advance with one condition that's always trivially true and another which is always false

    // TODO: enter+exit with two updates: one exact, and another that exceeds the distance threshold (could generate such a coordinate with polar formulas; probably a crate for that if our existing ones can't do it)

    // TODO: Similar to the above, but with, say, 5 random updates where the user is *always* within the distance threshold, so they never advance to the next step
}

#[cfg(test)]
mod off_step_tests {
    use super::*;
    use crate::models::UserLocation;
    use crate::navigation_controller::test_helpers::get_navigating_trip_state;
    use crate::test_utils::make_user_location;
    use geo::coord;
    use std::sync::LazyLock;

    use crate::models::RouteStep;
    use crate::navigation_controller::test_helpers::gen_route_step_with_coords;

    static STRAIGHT_LINE_STEP: LazyLock<RouteStep> = LazyLock::new(|| {
        gen_route_step_with_coords(vec![
            coord!(x: 0.0, y: 0.0),
            coord!(x: 0.001, y: 0.0), // 111 meters east
        ])
    });

    static LOCATION_FAR_FROM_STEP: LazyLock<UserLocation> =
        LazyLock::new(|| make_user_location(coord!(x: 0.005, y: 0.0005), 5.0));

    #[test]
    fn test_always_policy_advances_when_off_step_on_route() {
        let condition = DistanceFromStepCondition {
            minimum_horizontal_accuracy: 10,
            distance: 100,
            calculation_policy: DeviationCalculationPolicy::Always,
        };

        let trip_state = get_navigating_trip_state(
            *LOCATION_FAR_FROM_STEP,
            vec![STRAIGHT_LINE_STEP.clone()],
            vec![],
            RouteDeviation::Deviation {
                kind: DeviationKind::OffStepOnRoute {
                    deviation_from_step_line: 50.0,
                },
            },
        );

        let result = condition.should_advance_step(trip_state);
        assert!(
            result.should_advance,
            "Always should advance regardless of deviation status"
        );
    }

    #[test]
    fn test_while_on_current_step_policy_blocks_advance_when_off_step_on_route() {
        let condition = DistanceFromStepCondition {
            minimum_horizontal_accuracy: 10,
            distance: 100,
            calculation_policy: DeviationCalculationPolicy::WhileOnCurrentStep,
        };

        let trip_state = get_navigating_trip_state(
            *LOCATION_FAR_FROM_STEP,
            vec![STRAIGHT_LINE_STEP.clone()],
            vec![],
            RouteDeviation::Deviation {
                kind: DeviationKind::OffStepOnRoute {
                    deviation_from_step_line: 50.0,
                },
            },
        );

        let result = condition.should_advance_step(trip_state);
        assert!(
            !result.should_advance,
            "WhileOnCurrentStep should block advance once the user is off the current step"
        );
    }

    #[test]
    fn test_while_on_route_policy_advances_when_off_step_on_route() {
        // WhileOnRoute should permit advance when the user is off the current step
        // but still on a future step on the route polyline.
        let condition = DistanceFromStepCondition {
            minimum_horizontal_accuracy: 10,
            distance: 100,
            calculation_policy: DeviationCalculationPolicy::WhileOnRoute,
        };

        let trip_state = get_navigating_trip_state(
            *LOCATION_FAR_FROM_STEP,
            vec![STRAIGHT_LINE_STEP.clone()],
            vec![],
            RouteDeviation::Deviation {
                kind: DeviationKind::OffStepOnRoute {
                    deviation_from_step_line: 50.0,
                },
            },
        );

        let result = condition.should_advance_step(trip_state);
        assert!(
            result.should_advance,
            "WhileOnRoute should not block advancement when only off the current step"
        );
    }

    #[test]
    fn test_while_on_current_step_policy_blocks_advance_when_completely_off_route() {
        let condition = DistanceFromStepCondition {
            minimum_horizontal_accuracy: 10,
            distance: 100,
            // `WhileOnCurrentStep` blocks advancement on *any*
            // deviation, including `CompletelyOffRoute`.
            // Being completely off the route is itself a (stronger) form of being off the current step.
            calculation_policy: DeviationCalculationPolicy::WhileOnCurrentStep,
        };

        let trip_state = get_navigating_trip_state(
            *LOCATION_FAR_FROM_STEP,
            vec![STRAIGHT_LINE_STEP.clone()],
            vec![],
            RouteDeviation::Deviation {
                kind: DeviationKind::CompletelyOffRoute {
                    deviation_from_route_line: 200.0,
                },
            },
        );

        let result = condition.should_advance_step(trip_state);
        assert!(
            !result.should_advance,
            "WhileOnCurrentStep must block advancement even when CompletelyOffRoute, \
             since being off the route is itself being off the current step"
        );
    }

    #[test]
    fn test_while_on_route_policy_blocks_advance_when_completely_off_route() {
        let condition = DistanceFromStepCondition {
            minimum_horizontal_accuracy: 10,
            distance: 100,
            calculation_policy: DeviationCalculationPolicy::WhileOnRoute,
        };

        let trip_state = get_navigating_trip_state(
            *LOCATION_FAR_FROM_STEP,
            vec![STRAIGHT_LINE_STEP.clone()],
            vec![],
            RouteDeviation::Deviation {
                kind: DeviationKind::CompletelyOffRoute {
                    deviation_from_route_line: 200.0,
                },
            },
        );

        let result = condition.should_advance_step(trip_state);
        assert!(
            !result.should_advance,
            "WhileOnRoute should block advancement when the user is completely off the route"
        );
    }

    #[test]
    fn test_while_on_current_step_policy_handles_all_deviation_states() {
        let condition = DistanceFromStepCondition {
            minimum_horizontal_accuracy: 10,
            distance: 100,
            calculation_policy: DeviationCalculationPolicy::WhileOnCurrentStep,
        };

        // OffStepOnRoute: blocked
        let trip_state_off_step = get_navigating_trip_state(
            *LOCATION_FAR_FROM_STEP,
            vec![STRAIGHT_LINE_STEP.clone()],
            vec![],
            RouteDeviation::Deviation {
                kind: DeviationKind::OffStepOnRoute {
                    deviation_from_step_line: 50.0,
                },
            },
        );
        let result = condition.should_advance_step(trip_state_off_step);
        assert!(
            !result.should_advance,
            "WhileOnCurrentStep should block on OffStepOnRoute"
        );

        // CompletelyOffRoute: blocked
        let trip_state_off_route = get_navigating_trip_state(
            *LOCATION_FAR_FROM_STEP,
            vec![STRAIGHT_LINE_STEP.clone()],
            vec![],
            RouteDeviation::Deviation {
                kind: DeviationKind::CompletelyOffRoute {
                    deviation_from_route_line: 200.0,
                },
            },
        );
        let result = condition.should_advance_step(trip_state_off_route);
        assert!(
            !result.should_advance,
            "WhileOnCurrentStep should block on CompletelyOffRoute"
        );

        // NoDeviation: allowed
        let trip_state_on_route = get_navigating_trip_state(
            *LOCATION_FAR_FROM_STEP,
            vec![STRAIGHT_LINE_STEP.clone()],
            vec![],
            RouteDeviation::NoDeviation,
        );
        let result = condition.should_advance_step(trip_state_on_route);
        assert!(
            result.should_advance,
            "WhileOnCurrentStep should still allow advance when NoDeviation"
        );
    }
}
