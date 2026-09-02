//! The navigation state machine.

pub mod models;
pub mod step_advance;
mod step_progress;
pub mod uzmatch;
pub mod waypoint_advance;

#[cfg(test)]
pub(crate) mod test_helpers;

#[cfg(feature = "wasm-bindgen")]
use crate::navigation_controller::models::{
    SerializableNavState, SerializableNavigationControllerConfig,
};
use crate::{
    algorithms::{
        advance_step, apply_snapped_course, calculate_trip_progress,
        calculate_trip_progress_from_distance, index_of_closest_segment_origin,
        snap_user_location_to_line,
    },
    deviation_detection::RouteDeviation,
    models::{Route, RouteStep, UserLocation, Waypoint},
    navigation_controller::{
        models::TripSummary,
        waypoint_advance::{WaypointAdvanceChecker, WaypointAdvanceResult, WaypointCheckEvent},
    },
    navigation_session::{NavigationObserver, NavigationSession, recording::NavigationRecorder},
};
use chrono::Utc;
use geo::geometry::LineString;
use geo::{Distance, Haversine, Point};
use models::{NavState, NavigationControllerConfig, StepAdvanceStatus, TripState};
use std::clone::Clone;
use std::sync::Arc;
use step_progress::StepProgressIndex;
use uzmatch::{RouteSnapIndex, UzmatchSnapshot, UzmatchState};
#[cfg(feature = "wasm-bindgen")]
use wasm_bindgen::{JsValue, prelude::wasm_bindgen};

/// Core interface for navigation functionalities.
///
/// This trait defines the essential operations for a navigation state manager.
/// This lets us build additional layers (e.g. event logging)
/// around [`NavigationController`] in a composable manner.
#[cfg_attr(feature = "uniffi", uniffi::export)]
pub trait Navigator: Send + Sync {
    fn route(&self) -> Route;
    fn get_initial_state(&self, location: UserLocation) -> NavState;
    fn advance_to_next_step(&self, state: NavState) -> NavState;
    fn update_user_location(&self, location: UserLocation, state: NavState) -> NavState;
}

/// Creates a new navigation controller for the given route and configuration.
///
/// It returns an Arc-wrapped trait object implementing `Navigator`.
/// If `should_record` is true, it creates a controller with event recording enabled.
#[cfg_attr(feature = "uniffi", uniffi::export)]
pub fn create_navigator(
    route: Route,
    config: NavigationControllerConfig,
    should_record: bool,
) -> Arc<dyn Navigator> {
    let observers: Vec<Arc<dyn NavigationObserver>> = if should_record {
        vec![Arc::new(NavigationRecorder::new(
            route.clone(),
            config.clone(),
        ))]
    } else {
        vec![]
    };

    // Creates a normal navigation controller.
    Arc::new(NavigationSession::new(
        Arc::new(NavigationController::new(route, config)),
        observers,
    ))
}

/// Manages the navigation lifecycle through a route,
/// returning an updated state given inputs like user location.
///
/// Notes for implementing a new platform:
/// - A controller is bound to a single route; if you want recalculation, create a new instance.
/// - This is a pure type (no interior mutability), so a core function of your platform code is responsibly managing mutable state.
#[cfg_attr(feature = "uniffi", derive(uniffi::Object))]
pub struct NavigationController {
    route: Route,
    config: NavigationControllerConfig,
    route_snap_index: Option<RouteSnapIndex>,
    step_progress_index: Option<StepProgressIndex>,
}

#[cfg_attr(feature = "uniffi", uniffi::export)]
impl NavigationController {
    #[cfg_attr(feature = "uniffi", uniffi::constructor)]
    /// Create a navigation controller for a route and configuration.
    pub fn new(route: Route, config: NavigationControllerConfig) -> Self {
        let route_snap_index = config
            .uzmatch
            .enabled
            .then(|| RouteSnapIndex::new(&route.geometry));
        let step_progress_index = config
            .uzmatch
            .enabled
            .then(|| StepProgressIndex::new(&route))
            .flatten();
        Self {
            route,
            config,
            route_snap_index,
            step_progress_index,
        }
    }
}

impl Navigator for NavigationController {
    /// The route associated with this controller.
    fn route(&self) -> Route {
        self.route.clone()
    }

    /// Returns initial trip state as if the user had just started the route with no progress.
    fn get_initial_state(&self, location: UserLocation) -> NavState {
        let remaining_steps = self.route.steps.clone();

        let initial_summary = TripSummary {
            distance_traveled: 0.0,
            snapped_distance_traveled: 0.0,
            started_at: Utc::now(),
            ended_at: None,
        };

        let Some(current_route_step) = remaining_steps.first() else {
            // Bail early; if we don't have any steps, this is a useless route
            return NavState::complete(location, initial_summary);
        };

        // UzNav matching core: seed from the first location when enabled.
        let mut uzmatch_state = if let Some(route_index) = &self.route_snap_index {
            UzmatchState::default().update_with_index(&location, route_index, &self.config.uzmatch)
        } else {
            UzmatchState::default()
        };
        let uzmatch_snapshot = if self.config.uzmatch.enabled {
            Some(uzmatch_state.snapshot(location.timestamp))
        } else {
            None
        };

        let (current_step_geometry_index, snapped_user_location, progress) = self.step_state(
            location,
            current_route_step,
            &remaining_steps,
            uzmatch_snapshot.as_ref(),
        );

        let visual_instruction = current_route_step
            .get_active_visual_instruction(progress.distance_to_next_maneuver)
            .cloned();
        let spoken_instruction = current_route_step
            .get_current_spoken_instruction(progress.distance_to_next_maneuver)
            .cloned();

        let annotation_json = current_step_geometry_index
            .and_then(|index| current_route_step.get_annotation_at_current_index(index));

        let initial_trip_state = TripState::Navigating {
            current_step_geometry_index,
            user_location: location,
            snapped_user_location,
            remaining_steps,
            // Skip the first waypoint, as it is the current one
            remaining_waypoints: self.route.waypoints.iter().skip(1).cloned().collect(),
            progress,
            summary: initial_summary,
            deviation: RouteDeviation::NoDeviation,
            visual_instruction,
            spoken_instruction,
            annotation_json,
            uzmatch: uzmatch_snapshot,
        };

        let deviation = self
            .config
            .route_deviation_tracking
            .check_route_deviation(&self.route, &initial_trip_state);

        let trip_state = if let TripState::Navigating {
            current_step_geometry_index,
            user_location,
            snapped_user_location,
            remaining_steps,
            remaining_waypoints,
            progress,
            summary,
            visual_instruction,
            spoken_instruction,
            annotation_json,
            uzmatch,
            ..
        } = initial_trip_state
        {
            // If the user starts completely off the route, suppress instructions for the
            // same reason as in `create_intermediate_trip_state`: the snap-derived distance
            // to the next maneuver is geometrically unsound, so any countdown surfaced
            // from it would mislead the user. `OffStepOnRoute` is intentionally not
            // suppressed here — the user is still on the route polyline (just on a future
            // step), and the step-advance flow will reconcile shortly.
            let (visual_instruction, spoken_instruction) = if deviation.is_completely_off_route() {
                (None, None)
            } else {
                (visual_instruction, spoken_instruction)
            };
            TripState::Navigating {
                current_step_geometry_index,
                user_location,
                snapped_user_location,
                remaining_steps,
                remaining_waypoints,
                progress,
                summary,
                deviation, // Use the newly calculated deviation
                visual_instruction,
                spoken_instruction,
                annotation_json,
                uzmatch,
            }
        } else {
            unreachable!("initial_trip_state should always be Navigating variant")
        };

        let next_advance = Arc::clone(&self.config.step_advance_condition);
        NavState::new(trip_state, next_advance, uzmatch_state)
    }

    /// Advances navigation to the next step (or finishes the route).
    ///
    /// Depending on the advancement strategy, this may be automatic.
    /// For other cases, it is desirable to advance to the next step manually (ex: walking in an
    /// urban tunnel). We leave this decision to the app developer and provide this as a convenience.
    ///
    /// This method takes the intermediate state (e.g., from `update_user_location`) and advances if necessary,
    /// and does not handle anything like snapping.
    fn advance_to_next_step(&self, state: NavState) -> NavState {
        match state.trip_state() {
            TripState::Navigating {
                user_location,
                ref remaining_steps,
                ref remaining_waypoints,
                deviation,
                summary,
                uzmatch,
                ..
            } => {
                let update = advance_step(remaining_steps);
                match update {
                    StepAdvanceStatus::Advanced { step: current_step } => {
                        // Trim the remaining waypoints if needed.
                        let waypoints_result = self.get_new_waypoints(
                            &state.trip_state(),
                            WaypointCheckEvent::StepAdvanced(current_step.clone()),
                        );
                        let remaining_waypoints = match waypoints_result {
                            WaypointAdvanceResult::Unchanged => remaining_waypoints.clone(),
                            WaypointAdvanceResult::Changed(new_waypoints) => new_waypoints,
                        };

                        // Apply the updates
                        let mut remaining_steps = remaining_steps.clone();
                        remaining_steps.remove(0);

                        // Create a new trip state with the updated current_step
                        // and remaining_steps
                        let trip_state = self.create_intermediate_trip_state(
                            state.trip_state(),
                            user_location,
                            current_step,
                            remaining_steps,
                            remaining_waypoints,
                            deviation,
                            uzmatch,
                        );

                        // Reset condition state on every step advance. Auto-advance gets a fresh
                        // condition via `should_advance_step` returning `advance_to_new_instance`,
                        // but manual advance bypasses that — without this reset, stateful latches
                        // would leak from the previous step into the next one.
                        NavState::new(
                            trip_state,
                            state.step_advance_condition().new_instance(),
                            state.uzmatch_state(),
                        )
                    }
                    StepAdvanceStatus::EndOfRoute => NavState::complete(user_location, summary),
                }
            }
            // Pass through
            TripState::Idle { .. } | TripState::Complete { .. } => state.clone(),
        }
    }

    /// Updates the user's current location and updates the navigation state accordingly.
    ///
    /// # Panics
    ///
    /// If there is no current step ([`TripState::Navigating`] has an empty `remainingSteps` value),
    /// this function will panic.
    fn update_user_location(&self, location: UserLocation, state: NavState) -> NavState {
        match state.trip_state() {
            TripState::Navigating {
                remaining_steps,
                ref remaining_waypoints,
                summary,
                ..
            } => {
                // Remaining steps is empty, the route is finished.
                let Some(current_step) = remaining_steps.first().cloned() else {
                    return NavState::complete(location, summary);
                };

                // Trim the remaining waypoints if needed.
                let waypoints_result = self
                    .get_new_waypoints(&state.trip_state(), WaypointCheckEvent::LocationUpdated);
                let remaining_waypoints = match waypoints_result {
                    WaypointAdvanceResult::Unchanged => remaining_waypoints.clone(),
                    WaypointAdvanceResult::Changed(new_waypoints) => new_waypoints,
                };

                // UzNav matching core update. Runs before deviation so the
                // standing gate can suppress both deviation recalculation and
                // step advance (vendor: no step skips from GPS jitter while
                // standing at traffic lights).
                let previous_deviation = state
                    .trip_state()
                    .deviation()
                    .unwrap_or(RouteDeviation::NoDeviation);
                let (mut uzmatch_state, uzmatch_snapshot) =
                    if let Some(route_index) = &self.route_snap_index {
                        let mut updated = state.uzmatch_state().update_with_index(
                            &location,
                            route_index,
                            &self.config.uzmatch,
                        );
                        let snapshot = updated.snapshot(location.timestamp);
                        (updated, Some(snapshot))
                    } else {
                        (state.uzmatch_state(), None)
                    };
                let is_standing = uzmatch_snapshot.as_ref().is_some_and(|s| s.is_standing);
                let uzmatch_ran = uzmatch_snapshot.is_some();
                let matched_route_position =
                    uzmatch_snapshot.as_ref().and_then(|s| s.route_position);

                let is_arriving = remaining_steps.len() <= 2;
                let mut intermediate_trip_state = self.create_intermediate_trip_state(
                    state.trip_state(),
                    location,
                    current_step,
                    remaining_steps,
                    remaining_waypoints,
                    previous_deviation,
                    uzmatch_snapshot,
                );

                if !is_standing {
                    let deviation = self
                        .config
                        .route_deviation_tracking
                        .check_route_deviation(&self.route, &intermediate_trip_state);
                    let deviation = self.cling_stabilized_deviation(
                        &mut uzmatch_state,
                        uzmatch_ran,
                        matched_route_position,
                        location,
                        deviation,
                    );
                    Self::apply_deviation_and_instruction_policy(
                        &mut intermediate_trip_state,
                        deviation,
                    );
                }

                // Get the step advance condition result.
                //
                // While standing, the regular (non-arrival) condition is not
                // evaluated at all: the snapped position is frozen, so any
                // advance would be jitter-induced. Arrival conditions keep
                // running so a stopped vehicle can still finish the route.
                let (should_advance, next_condition) = if is_standing && !is_arriving {
                    (false, state.step_advance_condition())
                } else {
                    let step_advance_result = if is_arriving {
                        self.config
                            .arrival_step_advance_condition
                            .should_advance_step(intermediate_trip_state.clone())
                    } else {
                        state
                            .step_advance_condition()
                            .should_advance_step(intermediate_trip_state.clone())
                    };
                    (
                        step_advance_result.should_advance(),
                        step_advance_result.next_iteration,
                    )
                };

                let intermediate_nav_state =
                    NavState::new(intermediate_trip_state, next_condition, uzmatch_state);

                if should_advance {
                    // Advance to the next step
                    let updated_state = self.advance_to_next_step(intermediate_nav_state);

                    return if is_arriving {
                        updated_state
                    } else {
                        // Recurse ("speed run" behavior)
                        self.update_user_location(location, updated_state)
                    };
                }

                intermediate_nav_state
            }
            // Pass through
            TripState::Idle { .. } | TripState::Complete { .. } => state.clone(),
        }
    }
}

// Shared functionality for the navigation controller that is not exported by `UniFFI`.
impl NavigationController {
    /// Vendor `Clinger` route-loss stabilization (polyline port, no graph),
    /// plus the `UzNav` heading-departure detector.
    ///
    /// A freshly computed `CompletelyOffRoute` is held back until the raw fix
    /// is far enough from the last on-route acceptance in BOTH time
    /// (`cling_time_ms`) and distance (`cling_distance_m`) — one or two bad
    /// urban-canyon fixes therefore cannot escalate into a reroute. On-route
    /// outcomes (including the soft `OffStepOnRoute`) re-anchor the cling.
    ///
    /// When `heading_departure_enabled`, a credible course that keeps
    /// diverging from the route's forward direction at a vertex publishes the
    /// loss BYPASSING the cling: direction proves intent before distance can.
    /// Inactive when uzmatch is off so the upstream deviation contract is
    /// unchanged for plain Ferrostar users.
    fn cling_stabilized_deviation(
        &self,
        uzmatch_state: &mut UzmatchState,
        uzmatch_ran: bool,
        matched_route_position: Option<uzmatch::RoutePosition>,
        location: UserLocation,
        deviation: RouteDeviation,
    ) -> RouteDeviation {
        use crate::deviation_detection::DeviationKind;

        if !uzmatch_ran {
            return deviation;
        }
        if self.heading_departure_confirmed(uzmatch_state, matched_route_position, &location) {
            let deviation_from_route_line = match deviation {
                RouteDeviation::Deviation {
                    kind:
                        DeviationKind::CompletelyOffRoute {
                            deviation_from_route_line,
                        },
                } => deviation_from_route_line,
                _ => matched_route_position.map_or(0.0, |position| {
                    Haversine.distance(
                        Point::from(position.coordinates),
                        Point::from(location.coordinates),
                    )
                }),
            };
            return RouteDeviation::Deviation {
                kind: DeviationKind::CompletelyOffRoute {
                    deviation_from_route_line,
                },
            };
        }
        let completely_off = matches!(
            deviation,
            RouteDeviation::Deviation {
                kind: DeviationKind::CompletelyOffRoute { .. }
            }
        );
        if completely_off {
            if uzmatch_state.cling.holds(&location, &self.config.uzmatch) {
                RouteDeviation::NoDeviation
            } else {
                deviation
            }
        } else {
            let anchor = matched_route_position
                .map(|position| position.coordinates)
                .unwrap_or(location.coordinates);
            uzmatch_state
                .cling
                .anchor_on_route(anchor, location.timestamp);
            deviation
        }
    }

    /// `UzNav` heading-departure detector (design D2 of
    /// `uznav-pedestrian-route-loss`). A fix counts toward the confirmation
    /// streak only when ALL hold: the detector is enabled, the fix is bound to
    /// the route, it carries a course whose accuracy is within the tolerance,
    /// it is moving at or above `heading_departure_min_speed_mps`, the matched
    /// position is pinned at a route vertex ("after the maneuver point" — a
    /// mid-segment crossing of the avenue must not count), and the course
    /// diverges from the route's forward direction by more than the
    /// tolerance. Any other fix resets the streak. Returns true once the
    /// streak reaches the configured confirmations; it saturates there so
    /// every further diverging fix keeps publishing.
    fn heading_departure_confirmed(
        &self,
        uzmatch_state: &mut UzmatchState,
        matched_route_position: Option<uzmatch::RoutePosition>,
        location: &UserLocation,
    ) -> bool {
        let config = &self.config.uzmatch;
        if !config.heading_departure_enabled {
            uzmatch_state.heading_departure_streak = 0;
            return false;
        }
        let diverging = (|| {
            let position = matched_route_position?;
            let index = self.route_snap_index.as_ref()?;
            let course = location.course_over_ground?;
            if course.accuracy.is_some_and(|accuracy| {
                f64::from(accuracy) > config.heading_departure_tolerance_deg
            }) {
                return None;
            }
            let speed = location.speed?.value;
            if speed.is_nan() || speed < config.heading_departure_min_speed_mps {
                return None;
            }
            if !index.is_pinned_at_vertex(&position) {
                return None;
            }
            let ahead = index.bearing_ahead(&position)?;
            Some(
                uzmatch::snap::heading_difference(f64::from(course.degrees), ahead)
                    > config.heading_departure_tolerance_deg,
            )
        })();
        if diverging == Some(true) {
            uzmatch_state.heading_departure_streak = uzmatch_state
                .heading_departure_streak
                .saturating_add(1)
                .min(config.heading_departure_confirmations.max(1));
            uzmatch_state.heading_departure_streak >= config.heading_departure_confirmations
        } else {
            uzmatch_state.heading_departure_streak = 0;
            false
        }
    }

    fn step_state(
        &self,
        location: UserLocation,
        current_step: &RouteStep,
        remaining_steps: &[RouteStep],
        uzmatch: Option<&UzmatchSnapshot>,
    ) -> (Option<u64>, UserLocation, models::TripProgress) {
        self.step_state_from_match(location, remaining_steps, uzmatch)
            .unwrap_or_else(|| {
                // The independently supplied route and step geometries may not align.
                // Preserve the legacy current-step projection for that public input.
                let current_step_linestring = current_step.get_linestring();
                let (current_step_geometry_index, snapped_user_location) =
                    self.snap_user_to_match_or_line(location, &current_step_linestring, uzmatch);
                let progress = calculate_trip_progress(
                    &snapped_user_location.into(),
                    &current_step_linestring,
                    remaining_steps,
                );
                (current_step_geometry_index, snapped_user_location, progress)
            })
    }

    fn step_state_from_match(
        &self,
        location: UserLocation,
        remaining_steps: &[RouteStep],
        uzmatch: Option<&UzmatchSnapshot>,
    ) -> Option<(Option<u64>, UserLocation, models::TripProgress)> {
        // `NavState` crosses FFI and can be restored or constructed outside this
        // controller. Do not combine a route-global match with a foreign current
        // step merely because the remaining-step count happens to match.
        let current_step_index = self.route.steps.len().checked_sub(remaining_steps.len())?;
        let route_step_geometry = &self.route.steps.get(current_step_index)?.geometry;
        if route_step_geometry != &remaining_steps.first()?.geometry {
            return None;
        }

        let route_position = uzmatch?.route_position?;
        let indexed_position = self
            .step_progress_index
            .as_ref()?
            .locate(route_position, remaining_steps.len())?;
        let snapped_user_location = self.location_from_route_position(location, route_position);
        let progress = calculate_trip_progress_from_distance(
            indexed_position.distance_to_next_maneuver,
            remaining_steps,
        );
        Some((
            Some(indexed_position.current_step_geometry_index),
            snapped_user_location,
            progress,
        ))
    }

    fn location_from_route_position(
        &self,
        location: UserLocation,
        route_position: uzmatch::RoutePosition,
    ) -> UserLocation {
        let mut snapped_user_location = UserLocation {
            coordinates: route_position.coordinates,
            ..location
        };
        if matches!(
            self.config.snapped_location_course_filtering,
            models::CourseFiltering::SnapToRoute
        ) {
            snapped_user_location.course_over_ground = route_position.course_over_ground;
        }
        snapped_user_location
    }

    /// Create an intermediate trip state with updated values,
    /// but does _not_ advance to the next step or handle arrival.
    ///
    /// Parameters:
    /// - `trip_state`: The existing/last trip state.
    /// - `location`: The user's current location.
    /// - `current_step`: The current route step.
    /// - `remaining_steps`: The remaining route steps.
    /// - `remaining_waypoints`: The remaining waypoints.
    ///
    /// Returns:
    /// - `TripState`: The intermediate trip state.
    fn create_intermediate_trip_state(
        &self,
        trip_state: TripState,
        current_user_location: UserLocation,
        current_step: RouteStep,
        remaining_steps: Vec<RouteStep>,
        remaining_waypoints: Vec<Waypoint>,
        deviation: RouteDeviation,
        uzmatch: Option<UzmatchSnapshot>,
    ) -> TripState {
        match trip_state {
            TripState::Navigating {
                user_location: previous_user_location,
                snapped_user_location: previous_snapped_user_location,
                summary: previous_summary,
                ..
            } => {
                // Use the validated route-global position when it belongs to the
                // current step. The legacy projection remains the disabled,
                // off-route and non-aligning route fallback.
                let (current_step_geometry_index, snapped_user_location, progress) = self
                    .step_state(
                        current_user_location,
                        &current_step,
                        &remaining_steps,
                        uzmatch.as_ref(),
                    );

                // Update trip summary with accumulated distance
                let updated_summary = previous_summary.update(
                    &previous_user_location,
                    &current_user_location,
                    &previous_snapped_user_location,
                    &snapped_user_location,
                );

                let annotation_json = current_step_geometry_index
                    .and_then(|index| current_step.get_annotation_at_current_index(index));

                let mut intermediate_trip_state = TripState::Navigating {
                    current_step_geometry_index,
                    user_location: current_user_location,
                    snapped_user_location,
                    remaining_steps,
                    remaining_waypoints,
                    progress,
                    summary: updated_summary,
                    deviation,
                    visual_instruction: None,
                    spoken_instruction: None,
                    annotation_json,
                    uzmatch,
                };
                Self::apply_deviation_and_instruction_policy(
                    &mut intermediate_trip_state,
                    deviation,
                );
                intermediate_trip_state
            }
            // Pass through
            TripState::Idle { .. } | TripState::Complete { .. } => trip_state,
        }
    }

    /// Apply route deviation and derive guidance from the same updated trip state.
    ///
    /// A completely off-route location must not emit instructions paced from its
    /// phantom snapped point. `OffStepOnRoute` remains eligible because the user is
    /// still on the route and step advance will reconcile it.
    fn apply_deviation_and_instruction_policy(
        trip_state: &mut TripState,
        new_deviation: RouteDeviation,
    ) {
        let TripState::Navigating {
            remaining_steps,
            progress,
            deviation,
            visual_instruction,
            spoken_instruction,
            ..
        } = trip_state
        else {
            return;
        };

        *deviation = new_deviation;
        let Some(current_step) = remaining_steps.first() else {
            *visual_instruction = None;
            *spoken_instruction = None;
            return;
        };

        if new_deviation.is_completely_off_route() {
            *visual_instruction = None;
            *spoken_instruction = None;
        } else {
            *visual_instruction = current_step
                .get_active_visual_instruction(progress.distance_to_next_maneuver)
                .cloned();
            *spoken_instruction = current_step
                .get_current_spoken_instruction(progress.distance_to_next_maneuver)
                .cloned();
        }
    }

    /// Snaps the user's location to the route line and updates the user's course if necessary.
    ///
    /// This bundles all work related to snapping the user's location to the route line and is not intended to be exported.
    ///
    /// Returns the index of the closest segment origin to the snapped user location as well as the snapped user location.
    fn snap_user_to_line(
        &self,
        location: UserLocation,
        line: &LineString,
    ) -> (Option<u64>, UserLocation) {
        // Snap the user's latitude and longitude to the line.
        let snapped_user_location = snap_user_location_to_line(location, line);

        // Get the index of the closest segment origin to the snapped user location.
        let current_step_geometry_index =
            index_of_closest_segment_origin(snapped_user_location, line);

        // Snap the user's course to the line if the configuration specifies it.
        let snapped_with_course: UserLocation = match &self.config.snapped_location_course_filtering
        {
            models::CourseFiltering::SnapToRoute => {
                apply_snapped_course(snapped_user_location, current_step_geometry_index, line)
            }
            models::CourseFiltering::Raw => snapped_user_location,
        };

        (current_step_geometry_index, snapped_with_course)
    }

    /// Use the route-global Uzmatch projection when the current fix was bound.
    ///
    /// Calculating only the current-step index from the selected coordinate is
    /// intentional. Re-projecting the raw fix here would allow the controller
    /// and Uzmatch to disagree at loops and nearby parallel segments.
    fn snap_user_to_match_or_line(
        &self,
        location: UserLocation,
        line: &LineString,
        uzmatch: Option<&UzmatchSnapshot>,
    ) -> (Option<u64>, UserLocation) {
        let Some(route_position) = uzmatch.and_then(|snapshot| snapshot.route_position) else {
            return self.snap_user_to_line(location, line);
        };

        let snapped_user_location = self.location_from_route_position(location, route_position);

        let current_step_geometry_index =
            index_of_closest_segment_origin(snapped_user_location, line);
        (current_step_geometry_index, snapped_user_location)
    }

    /// Process waypoint advance
    fn get_new_waypoints(
        &self,
        state: &TripState,
        event: WaypointCheckEvent,
    ) -> WaypointAdvanceResult {
        let checker = WaypointAdvanceChecker {
            mode: self.config.waypoint_advance,
        };
        checker.get_new_waypoints(state, event)
    }
}

/// JavaScript wrapper for `NavigationController`.
/// This wrapper is required because `NavigationController` cannot be directly converted to a JavaScript object
/// and requires serialization/deserialization of its methods' inputs and outputs.
#[cfg(feature = "wasm-bindgen")]
#[wasm_bindgen(js_name = NavigationController)]
pub struct JsNavigationController(Arc<dyn Navigator>);

#[cfg(feature = "wasm-bindgen")]
#[wasm_bindgen(js_class = NavigationController)]
impl JsNavigationController {
    #[wasm_bindgen(constructor)]
    pub fn new(
        route: JsValue,
        config: JsValue,
        should_record: JsValue,
    ) -> Result<JsNavigationController, JsValue> {
        let route: Route = serde_wasm_bindgen::from_value(route)?;
        let config: SerializableNavigationControllerConfig =
            serde_wasm_bindgen::from_value(config)?;
        let should_record: bool = serde_wasm_bindgen::from_value(should_record)?;

        Ok(JsNavigationController(create_navigator(
            route,
            config.into(),
            should_record,
        )))
    }

    #[wasm_bindgen(js_name = getInitialState)]
    pub fn get_initial_state(&self, location: JsValue) -> Result<JsValue, JsValue> {
        let location: UserLocation = serde_wasm_bindgen::from_value(location)?;
        let nav_state = self.0.get_initial_state(location);
        let result: SerializableNavState = nav_state.into();

        serde_wasm_bindgen::to_value(&result).map_err(|e| JsValue::from_str(&format!("{:?}", e)))
    }

    #[wasm_bindgen(js_name = advanceToNextStep)]
    pub fn advance_to_next_step(&self, state: JsValue) -> Result<JsValue, JsValue> {
        let state: SerializableNavState = serde_wasm_bindgen::from_value(state)?;
        let new_state = self.0.advance_to_next_step(state.into());

        serde_wasm_bindgen::to_value(&SerializableNavState::from(new_state))
            .map_err(|e| JsValue::from_str(&format!("{:?}", e)))
    }

    #[wasm_bindgen(js_name = updateUserLocation)]
    pub fn update_user_location(
        &self,
        location: JsValue,
        state: JsValue,
    ) -> Result<JsValue, JsValue> {
        let location: UserLocation = serde_wasm_bindgen::from_value(location)?;
        let state: SerializableNavState = serde_wasm_bindgen::from_value(state)?;
        let new_state = self.0.update_user_location(location, state.into());

        serde_wasm_bindgen::to_value(&SerializableNavState::from(new_state))
            .map_err(|e| JsValue::from_str(&format!("{:?}", e)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::ManeuverModifier;
    use crate::navigation_controller::step_advance::conditions::{
        DistanceEntryAndExitCondition, DistanceToEndOfStepCondition,
    };
    use crate::navigation_controller::step_advance::{
        SerializableStepAdvanceCondition, StepAdvanceCondition, StepAdvanceResult,
        kan_69_test_condition_with_candidate,
    };
    use crate::navigation_controller::test_helpers::{
        gen_dummy_route_step, gen_route_from_steps, get_test_navigation_controller_config,
        nav_controller_insta_settings,
    };
    use crate::navigation_controller::uzmatch::{UzmatchConfig, UzmatchState};
    use crate::routing_adapters::osrm::models::OsrmWaypointProperties;
    use crate::simulation::{
        LocationBias, advance_location_simulation, location_simulation_from_route,
    };
    use crate::test_utils::{TestRoute, make_user_location, redact_properties};
    use geo::coord;
    use std::sync::{
        Arc, Mutex,
        atomic::{AtomicUsize, Ordering},
    };
    use std::time::{Duration, SystemTime, UNIX_EPOCH};

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum RecordingRole {
        Regular,
        Arrival,
    }

    #[derive(Debug, Clone, PartialEq)]
    enum RecordingEvent {
        Evaluate {
            role: RecordingRole,
            instance_id: usize,
            user_location: UserLocation,
            remaining_steps: usize,
        },
        NewInstance {
            role: RecordingRole,
            from_instance_id: usize,
            to_instance_id: usize,
        },
    }

    #[derive(Clone)]
    struct RecordingStepCondition {
        role: RecordingRole,
        instance_id: usize,
        next_instance_id: Arc<AtomicUsize>,
        events: Arc<Mutex<Vec<RecordingEvent>>>,
        should_advance: bool,
    }

    impl step_advance::StepAdvanceConditionSerializable for RecordingStepCondition {
        fn to_js(&self) -> SerializableStepAdvanceCondition {
            SerializableStepAdvanceCondition::Manual
        }
    }

    impl StepAdvanceCondition for RecordingStepCondition {
        fn should_advance_step(&self, trip_state: TripState) -> StepAdvanceResult {
            let remaining_steps = match &trip_state {
                TripState::Navigating {
                    remaining_steps, ..
                } => remaining_steps.len(),
                other => panic!("expected Navigating, got {other:?}"),
            };
            self.events.lock().unwrap().push(RecordingEvent::Evaluate {
                role: self.role,
                instance_id: self.instance_id,
                user_location: trip_state.user_location().unwrap(),
                remaining_steps,
            });
            if self.should_advance {
                StepAdvanceResult::advance_to_new_instance(self)
            } else {
                StepAdvanceResult::continue_with_state(self.new_instance())
            }
        }

        fn new_instance(&self) -> Arc<dyn StepAdvanceCondition> {
            let to_instance_id = self.next_instance_id.fetch_add(1, Ordering::SeqCst);
            self.events
                .lock()
                .unwrap()
                .push(RecordingEvent::NewInstance {
                    role: self.role,
                    from_instance_id: self.instance_id,
                    to_instance_id,
                });
            Arc::new(Self {
                instance_id: to_instance_id,
                ..self.clone()
            })
        }
    }

    #[derive(Clone)]
    struct CountingAdvanceCondition {
        evaluations: Arc<AtomicUsize>,
    }

    impl step_advance::StepAdvanceConditionSerializable for CountingAdvanceCondition {
        fn to_js(&self) -> SerializableStepAdvanceCondition {
            SerializableStepAdvanceCondition::Manual
        }
    }

    impl StepAdvanceCondition for CountingAdvanceCondition {
        fn should_advance_step(&self, _trip_state: TripState) -> StepAdvanceResult {
            self.evaluations.fetch_add(1, Ordering::SeqCst);
            StepAdvanceResult::advance_to_new_instance(self)
        }

        fn new_instance(&self) -> Arc<dyn StepAdvanceCondition> {
            Arc::new(self.clone())
        }
    }

    fn remaining_step_count(state: &NavState) -> usize {
        match state.trip_state() {
            TripState::Navigating {
                remaining_steps, ..
            } => remaining_steps.len(),
            other => panic!("expected Navigating, got {other:?}"),
        }
    }

    fn assert_regular_cadence_state(
        state: &NavState,
        expected_candidate_is_uturn: Option<bool>,
        expected_candidate_successor: Vec<SerializableStepAdvanceCondition>,
        expected_timestamp: SystemTime,
    ) {
        match state.step_advance_condition().to_js() {
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
                    (30, 5, 32, 5, 70, 10, 2, false),
                );
                assert_eq!(candidate_is_uturn, expected_candidate_is_uturn);
                assert_eq!(
                    serde_json::to_value(candidate_successor).unwrap(),
                    serde_json::to_value(expected_candidate_successor).unwrap(),
                );
                assert!(!confirmation_active);
                assert_eq!(movement_anchor, None);
                assert_eq!(confirmation_count, 0);
                assert_eq!(last_evaluated_timestamp, Some(expected_timestamp));
            }
            other => panic!("expected KAN-69 wrapper, got {other:?}"),
        }
    }

    fn assert_current_instruction_is_not_uturn(state: &NavState) {
        match state.trip_state() {
            TripState::Navigating {
                visual_instruction, ..
            } => assert!(!matches!(
                visual_instruction
                    .as_ref()
                    .and_then(|instruction| { instruction.primary_content.maneuver_modifier }),
                Some(ManeuverModifier::UTurn),
            )),
            other => panic!("expected Navigating, got {other:?}"),
        }
    }

    #[test]
    fn kan_69_disabled_confirmation_four_to_three_same_fix_is_suppressed() {
        let mut route = TestRoute::Valhalla.first_route();
        assert_eq!(route.steps.len(), 23);
        let selected_steps = route.steps[5..9].to_vec();
        assert_eq!(
            selected_steps
                .iter()
                .map(|step| step.distance)
                .collect::<Vec<_>>(),
            vec![7.0, 70.0, 46.0, 131.0],
        );
        let start = selected_steps[0].geometry[0];
        route.steps = selected_steps;
        assert_eq!(route.steps.len(), 4);

        let mut submitted_fix = make_user_location(coord!(x: start.lng, y: start.lat), 5.0);
        submitted_fix.timestamp = UNIX_EPOCH + Duration::from_secs(1_785_319_200);
        let evaluations = Arc::new(AtomicUsize::new(0));
        let regular = kan_69_test_condition_with_candidate(Arc::new(CountingAdvanceCondition {
            evaluations: Arc::clone(&evaluations),
        }));
        let controller =
            create_navigator(route, get_test_navigation_controller_config(regular), false);
        let state = controller.get_initial_state(submitted_fix);
        assert_eq!(remaining_step_count(&state), 4);
        assert_current_instruction_is_not_uturn(&state);

        let after_same_fix = controller.update_user_location(submitted_fix, state);
        assert_eq!(remaining_step_count(&after_same_fix), 3);
        assert_eq!(evaluations.load(Ordering::SeqCst), 1);
        assert_regular_cadence_state(&after_same_fix, None, vec![], submitted_fix.timestamp);
        assert_current_instruction_is_not_uturn(&after_same_fix);

        let (next_start, next_distance) = match after_same_fix.trip_state() {
            TripState::Navigating {
                remaining_steps, ..
            } => (remaining_steps[0].geometry[0], remaining_steps[0].distance),
            other => panic!("expected Navigating, got {other:?}"),
        };
        assert_eq!(next_distance, 70.0);
        let mut distinct_fix =
            make_user_location(coord!(x: next_start.lng, y: next_start.lat), 5.0);
        distinct_fix.timestamp = submitted_fix.timestamp + Duration::from_secs(1);
        let after_distinct = controller.update_user_location(distinct_fix, after_same_fix);

        assert_eq!(remaining_step_count(&after_distinct), 3);
        assert_eq!(evaluations.load(Ordering::SeqCst), 1);
        assert_regular_cadence_state(
            &after_distinct,
            Some(false),
            vec![SerializableStepAdvanceCondition::DistanceEntryExit {
                distance_to_end_of_step: 30,
                distance_after_end_step: 5,
                minimum_horizontal_accuracy: 32,
                has_reached_end_of_current_step: false,
            }],
            distinct_fix.timestamp,
        );
    }

    fn recording_condition(
        role: RecordingRole,
        should_advance: bool,
        next_instance_id: Arc<AtomicUsize>,
        events: Arc<Mutex<Vec<RecordingEvent>>>,
    ) -> Arc<dyn StepAdvanceCondition> {
        Arc::new(RecordingStepCondition {
            role,
            instance_id: next_instance_id.fetch_add(1, Ordering::SeqCst),
            next_instance_id,
            events,
            should_advance,
        })
    }

    fn run_regular_to_arrival_cadence(
        arrival_should_advance: bool,
    ) -> (Vec<RecordingEvent>, NavState, UserLocation) {
        let mut route = TestRoute::Valhalla.first_route();
        let start = route.geometry[0];
        assert_eq!(route.steps.len(), 23);
        assert_eq!((start.lat, start.lng), (59.442643, 24.765368));
        route.steps.truncate(3);
        assert_eq!(route.steps.len(), 3);
        let mut submitted_fix = make_user_location(coord!(x: start.lng, y: start.lat), 5.0);
        submitted_fix.timestamp = UNIX_EPOCH + Duration::from_secs(1_785_319_200);
        let events = Arc::new(Mutex::new(Vec::new()));
        let next_instance_id = Arc::new(AtomicUsize::new(1));
        let regular = recording_condition(
            RecordingRole::Regular,
            true,
            Arc::clone(&next_instance_id),
            Arc::clone(&events),
        );
        let arrival = recording_condition(
            RecordingRole::Arrival,
            arrival_should_advance,
            Arc::clone(&next_instance_id),
            Arc::clone(&events),
        );
        let mut config = get_test_navigation_controller_config(regular);
        config.arrival_step_advance_condition = arrival;
        let controller = create_navigator(route, config, false);
        let state = controller.get_initial_state(submitted_fix);
        assert_eq!(remaining_step_count(&state), 3);
        events.lock().unwrap().clear();

        let final_state = controller.update_user_location(submitted_fix, state);
        let recorded = events.lock().unwrap().clone();
        (recorded, final_state, submitted_fix)
    }

    fn evaluation(event: &RecordingEvent) -> (RecordingRole, usize, UserLocation, usize) {
        match event {
            RecordingEvent::Evaluate {
                role,
                instance_id,
                user_location,
                remaining_steps,
            } => (*role, *instance_id, *user_location, *remaining_steps),
            other => panic!("expected Evaluate, got {other:?}"),
        }
    }

    fn reset(event: &RecordingEvent) -> (RecordingRole, usize, usize) {
        match event {
            RecordingEvent::NewInstance {
                role,
                from_instance_id,
                to_instance_id,
            } => (*role, *from_instance_id, *to_instance_id),
            other => panic!("expected NewInstance, got {other:?}"),
        }
    }

    fn assert_same_fix_event_order(
        events: &[RecordingEvent],
        submitted_fix: UserLocation,
        arrival_advances: bool,
    ) {
        assert_eq!(events.len(), if arrival_advances { 6 } else { 5 });
        let (regular_role, regular_0, regular_fix, regular_steps) = evaluation(&events[0]);
        let (regular_reset_role_1, regular_from_0, regular_1) = reset(&events[1]);
        let (regular_reset_role_2, regular_from_1, regular_2) = reset(&events[2]);
        let (arrival_role, arrival_0, arrival_fix, arrival_steps) = evaluation(&events[3]);
        let (arrival_reset_role_1, arrival_from_0, arrival_1) = reset(&events[4]);

        assert_eq!(regular_role, RecordingRole::Regular);
        assert_eq!(regular_reset_role_1, RecordingRole::Regular);
        assert_eq!(regular_reset_role_2, RecordingRole::Regular);
        assert_eq!(arrival_role, RecordingRole::Arrival);
        assert_eq!(arrival_reset_role_1, RecordingRole::Arrival);
        assert_eq!(regular_fix, submitted_fix);
        assert_eq!(arrival_fix, submitted_fix);
        assert_eq!((regular_steps, arrival_steps), (3, 2));
        assert_eq!((regular_0, regular_1, regular_2), (1, 3, 4));
        assert_eq!((arrival_0, arrival_1), (2, 5));
        assert_eq!(regular_0, regular_from_0);
        assert_eq!(regular_1, regular_from_1);
        assert_ne!(regular_0, regular_1);
        assert_ne!(regular_1, regular_2);
        assert_eq!(arrival_0, arrival_from_0);
        assert_ne!(arrival_0, arrival_1);
        assert_ne!(regular_0, arrival_0);
        assert!(!events.iter().skip(3).any(|event| matches!(
            event,
            RecordingEvent::Evaluate {
                role: RecordingRole::Regular,
                ..
            }
        )));

        if arrival_advances {
            let (role, from, to) = reset(&events[5]);
            assert_eq!(role, RecordingRole::Arrival);
            assert_eq!(from, arrival_1);
            assert_eq!(to, 6);
        }
    }

    #[test]
    fn kan_69_regular_to_arrival_same_fix_cadence_arrival_holds() {
        let (events, final_state, submitted_fix) = run_regular_to_arrival_cadence(false);
        assert_eq!(remaining_step_count(&final_state), 2);
        assert_same_fix_event_order(&events, submitted_fix, false);
    }

    #[test]
    fn kan_69_regular_to_arrival_same_fix_cadence_arrival_advances() {
        let (events, final_state, submitted_fix) = run_regular_to_arrival_cadence(true);
        assert_eq!(remaining_step_count(&final_state), 1);
        assert_same_fix_event_order(&events, submitted_fix, true);
    }

    fn test_full_route_state_snapshot(
        route: Route,
        step_advance_condition: Arc<dyn StepAdvanceCondition>,
        should_record: bool,
    ) -> (Arc<dyn Navigator>, Vec<NavState>) {
        let mut simulation_state =
            location_simulation_from_route(&route, Some(10.0), LocationBias::None)
                .expect("Unable to create simulation");

        let controller = create_navigator(
            route,
            get_test_navigation_controller_config(step_advance_condition),
            should_record,
        );

        let mut state = controller.get_initial_state(simulation_state.current_location);
        let mut states = vec![state.clone()];
        loop {
            let new_simulation_state = advance_location_simulation(&simulation_state);
            let new_state =
                controller.update_user_location(new_simulation_state.current_location, state);

            match new_state.trip_state() {
                TripState::Idle { .. } => {}
                TripState::Navigating {
                    current_step_geometry_index,
                    ref remaining_steps,
                    ref deviation,
                    ..
                } => {
                    if let Some(index) = current_step_geometry_index {
                        let geom_length = remaining_steps[0].geometry.len() as u64;
                        // Regression test that the geometry index is valid
                        assert!(
                            index < geom_length,
                            "index = {index}, geom_length = {geom_length}"
                        );
                    }

                    // Regression test that we are never marked as completely off the route.
                    // We used to encounter this with relative step advance on self-intersecting
                    // routes, for example. OffStepOnRoute is acceptable
                    // (on the route, just a different step).
                    assert!(
                        !deviation.is_completely_off_route(),
                        "User should never be completely off route during simulation, got: {deviation:?}"
                    );
                }
                TripState::Complete { .. } => {
                    states.push(new_state);
                    break;
                }
            }

            simulation_state = new_simulation_state;
            state = new_state.clone();
            states.push(new_state);
        }

        (controller, states)
    }

    // Full simulations for several routes with different settings

    #[test]
    fn test_extended_exact_distance() {
        nav_controller_insta_settings().bind(|| {
            let (_, states) = test_full_route_state_snapshot(
                TestRoute::ValhallaExtended.first_route(),
                Arc::new(DistanceToEndOfStepCondition {
                    distance: 0,
                    minimum_horizontal_accuracy: 0,
                }),
                false,
            );
            insta::assert_yaml_snapshot!(states
                .into_iter()
                .map(|state| state.trip_state())
                .collect::<Vec<_>>(), {
                    ".**.remainingWaypoints[].properties" => insta::dynamic_redaction(redact_properties::<OsrmWaypointProperties>),
                    ".**.remaining_waypoints[].properties" => insta::dynamic_redaction(redact_properties::<OsrmWaypointProperties>),
                });
        });
    }

    #[test]
    fn test_extended_relative_linestring() {
        nav_controller_insta_settings().bind(|| {
            let (_, states) = test_full_route_state_snapshot(
                TestRoute::ValhallaExtended.first_route(),
                Arc::new(DistanceEntryAndExitCondition::exact()),
                false,
            );
            insta::assert_yaml_snapshot!(states
                .into_iter()
                .map(|state| state.trip_state())
                .collect::<Vec<_>>(), {
                    ".**.remainingWaypoints[].properties" => insta::dynamic_redaction(redact_properties::<OsrmWaypointProperties>),
                    ".**.remaining_waypoints[].properties" => insta::dynamic_redaction(redact_properties::<OsrmWaypointProperties>),
                });
        });
    }

    #[test]
    fn test_self_intersecting_exact_distance() {
        nav_controller_insta_settings().bind(|| {
            let (_, states) = test_full_route_state_snapshot(
                TestRoute::ValhallaSelfIntersecting.first_route(),
                Arc::new(DistanceToEndOfStepCondition {
                    distance: 0,
                    minimum_horizontal_accuracy: 0,
                }),
                false,
            );
            insta::assert_yaml_snapshot!(states
                .into_iter()
                .map(|state| state.trip_state())
                .collect::<Vec<_>>(), {
                    ".**.remainingWaypoints[].properties" => insta::dynamic_redaction(redact_properties::<OsrmWaypointProperties>),
                    ".**.remaining_waypoints[].properties" => insta::dynamic_redaction(redact_properties::<OsrmWaypointProperties>),
                });
        });
    }

    #[test]
    fn test_self_intersecting_relative_linestring() {
        nav_controller_insta_settings().bind(|| {
            let (_, states) = test_full_route_state_snapshot(
                TestRoute::ValhallaSelfIntersecting.first_route(),
                Arc::new(DistanceEntryAndExitCondition::exact()),
                false,
            );
            insta::assert_yaml_snapshot!(states
                .into_iter()
                .map(|state| state.trip_state())
                .collect::<Vec<_>>(), {
                    ".**.remainingWaypoints[].properties" => insta::dynamic_redaction(redact_properties::<OsrmWaypointProperties>),
                    ".**.remaining_waypoints[].properties" => insta::dynamic_redaction(redact_properties::<OsrmWaypointProperties>),
                });
        });
    }

    #[test]
    fn test_self_intersecting_relative_linestring_min_line_distance() {
        nav_controller_insta_settings().bind(|| {
            let (_, states) = test_full_route_state_snapshot(
                TestRoute::ValhallaSelfIntersecting.first_route(),
                Arc::new(DistanceToEndOfStepCondition {
                    distance: 0,
                    minimum_horizontal_accuracy: 0,
                }),
                false,
            );
            insta::assert_yaml_snapshot!(states
                .into_iter()
                .map(|state| state.trip_state())
                .collect::<Vec<_>>(), {
                    ".**.remainingWaypoints[].properties" => insta::dynamic_redaction(redact_properties::<OsrmWaypointProperties>),
                    ".**.remaining_waypoints[].properties" => insta::dynamic_redaction(redact_properties::<OsrmWaypointProperties>),
                });
        });
    }

    #[test]
    fn test_roundabout_exact_distance() {
        nav_controller_insta_settings().bind(|| {
            let (_, states) = test_full_route_state_snapshot(
                TestRoute::ValhallaWithRoundabouts.first_route(),
                Arc::new(DistanceToEndOfStepCondition {
                    distance: 0,
                    minimum_horizontal_accuracy: 0,
                }),
                false,
            );
            insta::assert_yaml_snapshot!(states
                .into_iter()
                .map(|state| state.trip_state())
                .collect::<Vec<_>>(), {
                    ".**.remainingWaypoints[].properties" => insta::dynamic_redaction(redact_properties::<OsrmWaypointProperties>),
                    ".**.remaining_waypoints[].properties" => insta::dynamic_redaction(redact_properties::<OsrmWaypointProperties>),
                });
        });
    }

    /// Off-route should suppress visual + spoken instructions, because they are derived
    /// from the snapped distance to the next maneuver — which is geometrically unsound
    /// when the user is laterally far from the route. Returning to the route should
    /// resume normal emission.
    #[test]
    fn test_off_route_suppresses_instructions() {
        use crate::deviation_detection::RouteDeviationTracking;
        use crate::navigation_controller::models::{CourseFiltering, WaypointAdvanceMode};
        use crate::navigation_controller::step_advance::conditions::ManualStepCondition;
        use crate::test_utils::make_user_location;
        use geo::coord;

        let route = TestRoute::Valhalla.first_route();
        let start_coord = route.geometry[0];

        // ManualStepCondition keeps the test focused on instruction emission — no automatic
        // step advancement can perturb the current step across calls.
        // StaticThreshold with tight thresholds: any noticeable lateral offset trips deviation.
        let config = NavigationControllerConfig {
            waypoint_advance: WaypointAdvanceMode::WaypointWithinRange(100.0),
            route_deviation_tracking: RouteDeviationTracking::StaticThreshold {
                minimum_horizontal_accuracy: 20,
                max_acceptable_deviation: 30.0,
            },
            snapped_location_course_filtering: CourseFiltering::Raw,
            step_advance_condition: Arc::new(ManualStepCondition),
            arrival_step_advance_condition: Arc::new(ManualStepCondition),
            uzmatch: UzmatchConfig::default(),
        };

        let controller = create_navigator(route, config, false);

        let on_route_loc = make_user_location(coord!(x: start_coord.lng, y: start_coord.lat), 5.0);

        // 1. Initial state at the route start: on-route, instructions populated.
        let initial = controller.get_initial_state(on_route_loc.clone());
        let (initial_visual, initial_spoken) = match initial.trip_state() {
            TripState::Navigating {
                deviation,
                visual_instruction,
                spoken_instruction,
                ..
            } => {
                assert_eq!(
                    deviation,
                    RouteDeviation::NoDeviation,
                    "initial state at the route start should be on-route"
                );
                assert!(
                    visual_instruction.is_some() || spoken_instruction.is_some(),
                    "Valhalla fixture's first step has voice + banner instructions; \
                     at least one should be populated when on-route at the start"
                );
                (visual_instruction, spoken_instruction)
            }
            other => panic!("expected Navigating, got {other:?}"),
        };

        // 2. Update with a wildly off-route location.
        // Offset by ~0.5° (~55 km) so the location is unambiguously far from every step
        // in the route, regardless of where the route winds. The deviation check scans all
        // remaining steps and returns NoDeviation if the user is close to any of them.
        let off_route_loc = make_user_location(
            coord!(x: start_coord.lng + 0.5, y: start_coord.lat + 0.5),
            5.0,
        );
        let off_state = controller.update_user_location(off_route_loc, initial);
        match off_state.trip_state() {
            TripState::Navigating {
                deviation,
                visual_instruction,
                spoken_instruction,
                ..
            } => {
                assert!(
                    deviation.is_completely_off_route(),
                    "expected CompletelyOffRoute on the current off-route tick, got {deviation:?}"
                );
                assert!(
                    visual_instruction.is_none(),
                    "visual_instruction must be None while completely off-route, got {visual_instruction:?}"
                );
                assert!(
                    spoken_instruction.is_none(),
                    "spoken_instruction must be None while completely off-route, got {spoken_instruction:?}"
                );
            }
            other => panic!("expected Navigating, got {other:?}"),
        };

        // 3. A single current on-route fix clears deviation and resumes instructions.
        let recovered = controller.update_user_location(on_route_loc, off_state);
        match recovered.trip_state() {
            TripState::Navigating {
                deviation,
                visual_instruction,
                spoken_instruction,
                ..
            } => {
                assert_eq!(
                    deviation,
                    RouteDeviation::NoDeviation,
                    "expected to be back on route at the same starting coordinate"
                );
                assert_eq!(
                    visual_instruction, initial_visual,
                    "visual_instruction should resume emission on return to route"
                );
                assert_eq!(
                    spoken_instruction, initial_spoken,
                    "spoken_instruction should resume emission on return to route"
                );
            }
            other => panic!("expected Navigating, got {other:?}"),
        };
    }

    /// Starting a route with an off-route initial location should also suppress instructions —
    /// covers the `get_initial_state` path in addition to the `update_user_location` path.
    #[test]
    fn test_off_route_initial_state_suppresses_instructions() {
        use crate::deviation_detection::RouteDeviationTracking;
        use crate::navigation_controller::models::{CourseFiltering, WaypointAdvanceMode};
        use crate::navigation_controller::step_advance::conditions::ManualStepCondition;
        use crate::test_utils::make_user_location;
        use geo::coord;

        let route = TestRoute::Valhalla.first_route();
        let start_coord = route.geometry[0];

        let config = NavigationControllerConfig {
            waypoint_advance: WaypointAdvanceMode::WaypointWithinRange(100.0),
            route_deviation_tracking: RouteDeviationTracking::StaticThreshold {
                minimum_horizontal_accuracy: 20,
                max_acceptable_deviation: 30.0,
            },
            snapped_location_course_filtering: CourseFiltering::Raw,
            step_advance_condition: Arc::new(ManualStepCondition),
            arrival_step_advance_condition: Arc::new(ManualStepCondition),
            uzmatch: UzmatchConfig::default(),
        };

        let controller = create_navigator(route, config, false);

        // User opens the app already off-route from the planned start.
        // Offset by ~0.5° (~55 km) so the location is unambiguously far from every step
        // in the route, regardless of where the route winds. The deviation check scans all
        // remaining steps and returns NoDeviation if the user is close to any of them.
        let off_route_loc = make_user_location(
            coord!(x: start_coord.lng + 0.5, y: start_coord.lat + 0.5),
            5.0,
        );
        let initial = controller.get_initial_state(off_route_loc);
        match initial.trip_state() {
            TripState::Navigating {
                deviation,
                visual_instruction,
                spoken_instruction,
                ..
            } => {
                assert!(
                    deviation.is_completely_off_route(),
                    "expected CompletelyOffRoute on initial state at off-route location, got {deviation:?}"
                );
                assert!(
                    visual_instruction.is_none(),
                    "initial visual_instruction must be None when starting off-route"
                );
                assert!(
                    spoken_instruction.is_none(),
                    "initial spoken_instruction must be None when starting off-route"
                );
            }
            other => panic!("expected Navigating, got {other:?}"),
        };
    }

    /// Stateful step advance conditions must have their per-step state reset whenever
    /// a step advances, regardless of how the advance was triggered. Auto-advance is
    /// reset upstream by `should_advance_step` returning `advance_to_new_instance`,
    /// but manual advance — direct calls to `Navigator::advance_to_next_step` — must
    /// also reset, otherwise per-step latches leak into the next step.
    #[test]
    fn test_manual_advance_resets_condition_state() {
        use crate::deviation_detection::RouteDeviationTracking;
        use crate::navigation_controller::models::{CourseFiltering, WaypointAdvanceMode};
        use crate::navigation_controller::step_advance::SerializableStepAdvanceCondition;
        use crate::test_utils::make_user_location;
        use geo::coord;

        let route = TestRoute::Valhalla.first_route();
        let start_coord = route.geometry[0];

        // Build a pre-latched condition via the public serializable form, since the
        // concrete struct's fields are crate-private.
        let pre_latched: Arc<dyn StepAdvanceCondition> =
            SerializableStepAdvanceCondition::DistanceEntryExit {
                distance_to_end_of_step: 20,
                distance_after_end_step: 5,
                minimum_horizontal_accuracy: 25,
                has_reached_end_of_current_step: true,
            }
            .into();

        let config = NavigationControllerConfig {
            waypoint_advance: WaypointAdvanceMode::WaypointWithinRange(100.0),
            route_deviation_tracking: RouteDeviationTracking::None,
            snapped_location_course_filtering: CourseFiltering::Raw,
            step_advance_condition: Arc::clone(&pre_latched),
            arrival_step_advance_condition: Arc::clone(&pre_latched),
            uzmatch: UzmatchConfig::default(),
        };

        let controller = create_navigator(route, config, false);
        let initial = controller.get_initial_state(make_user_location(
            coord!(x: start_coord.lng, y: start_coord.lat),
            5.0,
        ));

        // Replace the initial state's condition with the pre-latched one to simulate
        // having reached the end of the current step on a previous tick.
        let state_with_latch =
            NavState::new(initial.trip_state(), pre_latched, UzmatchState::default());

        // Manual advance bypasses `should_advance_step` and `advance_to_new_instance`.
        let advanced = controller.advance_to_next_step(state_with_latch);

        match advanced.step_advance_condition().to_js() {
            SerializableStepAdvanceCondition::DistanceEntryExit {
                has_reached_end_of_current_step,
                ..
            } => assert!(
                !has_reached_end_of_current_step,
                "Manual advance must reset the carried condition's latch state"
            ),
            other => panic!("expected DistanceEntryExit, got {other:?}"),
        }
    }

    /// UzNav P1a: while the matching core reports standing, the regular step
    /// advance condition is not evaluated, so GPS jitter across the step
    /// boundary cannot skip the step (vendor: no step skips at traffic lights).
    #[test]
    fn uzmatch_standing_gates_step_advance() {
        use crate::deviation_detection::RouteDeviationTracking;
        use crate::models::{GeographicCoordinate, Speed};
        use crate::navigation_controller::models::{CourseFiltering, WaypointAdvanceMode};
        use crate::navigation_controller::step_advance::conditions::{
            DistanceToEndOfStepCondition, ManualStepCondition,
        };

        // Three ~111 m steps (east, east, north) so that `is_arriving`
        // (remaining <= 2) stays false and the regular condition is exercised.
        let step1 = gen_dummy_route_step(0.0, 0.0, 0.001, 0.0);
        let step2 = gen_dummy_route_step(0.001, 0.0, 0.002, 0.0);
        let step3 = gen_dummy_route_step(0.002, 0.0, 0.002, 0.001);
        let route = gen_route_from_steps(vec![step1, step2, step3]);

        let config = NavigationControllerConfig {
            waypoint_advance: WaypointAdvanceMode::WaypointWithinRange(100.0),
            route_deviation_tracking: RouteDeviationTracking::None,
            snapped_location_course_filtering: CourseFiltering::Raw,
            // Advances whenever the user is within 20 m of the step end.
            step_advance_condition: Arc::new(DistanceToEndOfStepCondition {
                distance: 20,
                minimum_horizontal_accuracy: 10,
            }),
            arrival_step_advance_condition: Arc::new(ManualStepCondition),
            uzmatch: UzmatchConfig {
                enabled: true,
                ..UzmatchConfig::default()
            },
        };
        let controller = NavigationController::new(route, config);

        let t0 = SystemTime::UNIX_EPOCH + Duration::from_secs(1_000_000);
        let loc = |t: u64, lng: f64, speed: f64| UserLocation {
            coordinates: GeographicCoordinate { lng, lat: 0.0 },
            horizontal_accuracy: 5.0,
            course_over_ground: None,
            timestamp: t0 + Duration::from_secs(t),
            speed: Some(Speed {
                value: speed,
                accuracy: None,
            }),
        };

        let remaining = |state: &NavState| match state.trip_state() {
            TripState::Navigating {
                remaining_steps, ..
            } => remaining_steps.len(),
            other => panic!("expected Navigating, got {other:?}"),
        };

        // Approach and stop ~22 m before the step end (outside the 20 m
        // threshold): the standing span reaches 7 s at t=8.
        let mut state = controller.get_initial_state(loc(0, 0.0008, 4.0));
        for t in 1..=8 {
            state = controller.update_user_location(loc(t, 0.0008, 0.0), state);
            assert_eq!(
                remaining(&state),
                3,
                "must not advance while parked at t={t}"
            );
        }

        // Standing is established; the snapshot must say so.
        match state.trip_state() {
            TripState::Navigating { uzmatch, .. } => {
                assert_eq!(
                    uzmatch.map(|s| s.is_standing),
                    Some(true),
                    "standing must be reported after 7 s below threshold"
                );
            }
            other => panic!("expected Navigating, got {other:?}"),
        }

        // Jitter across the 20 m threshold while standing: must NOT advance.
        for t in 9..=11 {
            state = controller.update_user_location(loc(t, 0.00085, 0.0), state);
            assert_eq!(
                remaining(&state),
                3,
                "standing gate must swallow jitter across the step boundary at t={t}"
            );
        }

        // Movement resumes: the gate lifts and the condition advances normally.
        state = controller.update_user_location(loc(12, 0.00085, 8.0), state);
        assert_eq!(
            remaining(&state),
            2,
            "step must advance once the vehicle is moving again"
        );
    }

    #[test]
    fn uzmatch_projection_is_the_controller_snap_source() {
        use crate::deviation_detection::RouteDeviationTracking;
        use crate::models::{CourseOverGround, GeographicCoordinate, Speed};
        use crate::navigation_controller::models::{CourseFiltering, WaypointAdvanceMode};
        use crate::navigation_controller::step_advance::conditions::ManualStepCondition;

        let eastbound = gen_dummy_route_step(0.0, 0.0, 0.01, 0.0);
        let connector = gen_dummy_route_step(0.01, 0.0, 0.01, 0.0001);
        let westbound = gen_dummy_route_step(0.01, 0.0001, 0.0, 0.0001);
        let route = gen_route_from_steps(vec![eastbound, connector, westbound]);
        let config = NavigationControllerConfig {
            waypoint_advance: WaypointAdvanceMode::WaypointWithinRange(100.0),
            route_deviation_tracking: RouteDeviationTracking::None,
            snapped_location_course_filtering: CourseFiltering::SnapToRoute,
            step_advance_condition: Arc::new(ManualStepCondition),
            arrival_step_advance_condition: Arc::new(ManualStepCondition),
            uzmatch: UzmatchConfig {
                enabled: true,
                ..UzmatchConfig::default()
            },
        };
        let controller = NavigationController::new(route, config);
        let location = UserLocation {
            coordinates: GeographicCoordinate {
                lat: 0.00008,
                lng: 0.005,
            },
            horizontal_accuracy: 5.0,
            course_over_ground: Some(CourseOverGround::new(270.0, None)),
            timestamp: UNIX_EPOCH + Duration::from_secs(10),
            speed: Some(Speed {
                value: 10.0,
                accuracy: None,
            }),
        };

        let state = controller.get_initial_state(location);
        let TripState::Navigating {
            snapped_user_location,
            uzmatch: Some(snapshot),
            ..
        } = state.trip_state()
        else {
            panic!("expected navigating state with Uzmatch snapshot");
        };
        let matched = snapshot
            .route_position
            .expect("accurate location must bind to the route");

        assert_eq!(matched.segment_index, 4);
        assert_eq!(snapped_user_location.coordinates, matched.coordinates);
        assert_eq!(
            snapped_user_location.course_over_ground,
            matched.course_over_ground
        );
        assert!(snapped_user_location.coordinates.lat > 0.00009);
    }

    /// Vendor `Clinger` polyline port: a brief off-route excursion is held
    /// back, and release requires BOTH thresholds; returning to the route
    /// re-anchors the cling.
    #[test]
    fn uzmatch_cling_holds_then_releases_route_loss() {
        use crate::deviation_detection::{DeviationKind, RouteDeviationTracking};
        use crate::models::{GeographicCoordinate, Speed};
        use crate::navigation_controller::models::{CourseFiltering, WaypointAdvanceMode};
        use crate::navigation_controller::step_advance::conditions::ManualStepCondition;

        let step1 = gen_dummy_route_step(0.0, 0.0, 0.001, 0.0);
        let step2 = gen_dummy_route_step(0.001, 0.0, 0.002, 0.0);
        let step3 = gen_dummy_route_step(0.002, 0.0, 0.002, 0.001);
        let route = gen_route_from_steps(vec![step1, step2, step3]);

        let config = NavigationControllerConfig {
            waypoint_advance: WaypointAdvanceMode::WaypointWithinRange(100.0),
            route_deviation_tracking: RouteDeviationTracking::StaticThreshold {
                minimum_horizontal_accuracy: 32,
                max_acceptable_deviation: 50.0,
            },
            snapped_location_course_filtering: CourseFiltering::Raw,
            step_advance_condition: Arc::new(ManualStepCondition),
            arrival_step_advance_condition: Arc::new(ManualStepCondition),
            uzmatch: UzmatchConfig {
                enabled: true,
                ..UzmatchConfig::default()
            },
        };
        let controller = NavigationController::new(route, config);

        let t0 = SystemTime::UNIX_EPOCH + Duration::from_secs(1_000_000);
        // Moving fixes (speed 10 m/s) so the standing gate never engages.
        let loc = |t_ms: u64, lng: f64, lat: f64| UserLocation {
            coordinates: GeographicCoordinate { lng, lat },
            horizontal_accuracy: 5.0,
            course_over_ground: None,
            timestamp: t0 + Duration::from_millis(t_ms),
            speed: Some(Speed {
                value: 10.0,
                accuracy: None,
            }),
        };
        let deviation = |state: &NavState| match state.trip_state() {
            TripState::Navigating { deviation, .. } => deviation,
            other => panic!("expected Navigating, got {other:?}"),
        };

        // On-route anchor at t=0.
        let mut state = controller.get_initial_state(loc(0, 0.0008, 0.0));
        state = controller.update_user_location(loc(500, 0.00085, 0.0), state);
        assert_eq!(deviation(&state), RouteDeviation::NoDeviation);

        // ~67 m off the line one second after the anchor: the raw deviation
        // exceeds the 50 m threshold, but the cling window (2 s) holds it.
        state = controller.update_user_location(loc(1_500, 0.00085, 0.0006), state);
        assert_eq!(
            deviation(&state),
            RouteDeviation::NoDeviation,
            "a brief excursion must cling to the route"
        );

        // Still off at t=3.5 s and ~67 m from the anchor: both vendor
        // thresholds are exceeded, the loss is finally published.
        state = controller.update_user_location(loc(3_500, 0.0009, 0.0006), state);
        assert!(
            matches!(
                deviation(&state),
                RouteDeviation::Deviation {
                    kind: DeviationKind::CompletelyOffRoute { .. }
                }
            ),
            "sustained loss must release the cling"
        );

        // Back on the route: re-anchored, and a fresh brief excursion is
        // held again instead of escalating immediately.
        state = controller.update_user_location(loc(4_500, 0.001, 0.0), state);
        assert_eq!(deviation(&state), RouteDeviation::NoDeviation);
        state = controller.update_user_location(loc(5_000, 0.00105, 0.0006), state);
        assert_eq!(
            deviation(&state),
            RouteDeviation::NoDeviation,
            "returning to the route must re-arm the cling"
        );
    }

    /// Vendor guide model (`guide_impl.cpp`: `drivingRoute()->setPosition`):
    /// a bound fix advances maneuvers by its route-global position. A raw GPS
    /// track offset ~40 m laterally must still advance the step once the
    /// MATCHED position passes the maneuver — under the raw-proximity entry
    /// check it never could, because the raw fix never enters the 30 m radius.
    #[test]
    fn uzmatch_bound_position_advances_offset_raw_track() {
        use crate::deviation_detection::RouteDeviationTracking;
        use crate::models::{GeographicCoordinate, Speed};
        use crate::navigation_controller::models::{CourseFiltering, WaypointAdvanceMode};
        use crate::navigation_controller::step_advance::conditions::ManualStepCondition;
        use crate::navigation_controller::step_advance::step_advance_distance_entry_and_exit;

        let step1 = gen_dummy_route_step(0.0, 0.0, 0.001, 0.0);
        let step2 = gen_dummy_route_step(0.001, 0.0, 0.002, 0.0);
        let step3 = gen_dummy_route_step(0.002, 0.0, 0.002, 0.001);
        let route = gen_route_from_steps(vec![step1, step2, step3]);

        let config = NavigationControllerConfig {
            waypoint_advance: WaypointAdvanceMode::WaypointWithinRange(100.0),
            route_deviation_tracking: RouteDeviationTracking::StaticThreshold {
                minimum_horizontal_accuracy: 32,
                max_acceptable_deviation: 50.0,
            },
            snapped_location_course_filtering: CourseFiltering::Raw,
            step_advance_condition: step_advance_distance_entry_and_exit(30, 5, 25),
            arrival_step_advance_condition: Arc::new(ManualStepCondition),
            uzmatch: UzmatchConfig {
                enabled: true,
                ..UzmatchConfig::default()
            },
        };
        let controller = NavigationController::new(route, config);

        let t0 = SystemTime::UNIX_EPOCH + Duration::from_secs(1_000_000);
        // Raw fixes ride ~40 m north of the route line (lat 0.00036) while
        // progressing east; the matcher binds them to the line below.
        let loc = |t: u64, lng: f64| UserLocation {
            coordinates: GeographicCoordinate { lng, lat: 0.00036 },
            horizontal_accuracy: 5.0,
            course_over_ground: None,
            timestamp: t0 + Duration::from_secs(t),
            speed: Some(Speed {
                value: 10.0,
                accuracy: None,
            }),
        };
        let remaining = |state: &NavState| match state.trip_state() {
            TripState::Navigating {
                remaining_steps, ..
            } => remaining_steps.len(),
            other => panic!("expected Navigating, got {other:?}"),
        };

        let mut state = controller.get_initial_state(loc(0, 0.0));
        assert_eq!(remaining(&state), 3);
        // Drive past the first maneuver (step 1 ends at lng 0.001 ≈ 111 m).
        for (i, lng) in [
            0.0002, 0.0004, 0.0006, 0.0008, 0.00095, 0.00105, 0.0012, 0.0013,
        ]
        .iter()
        .enumerate()
        {
            state = controller.update_user_location(loc(i as u64 + 1, *lng), state);
        }
        assert_eq!(
            remaining(&state),
            2,
            "the matched route position passed the maneuver, so the step must advance"
        );
    }

    /// UzNav P1a: while standing, route deviation is not recalculated, so
    /// jitter off the route line does not flap the deviation flag.
    #[test]
    fn uzmatch_standing_freezes_deviation() {
        use crate::deviation_detection::RouteDeviationTracking;
        use crate::models::{GeographicCoordinate, Speed};
        use crate::navigation_controller::models::{CourseFiltering, WaypointAdvanceMode};
        use crate::navigation_controller::step_advance::conditions::ManualStepCondition;

        let step1 = gen_dummy_route_step(0.0, 0.0, 0.001, 0.0);
        let step2 = gen_dummy_route_step(0.001, 0.0, 0.001, 0.001);
        let route = gen_route_from_steps(vec![step1, step2]);

        let config = NavigationControllerConfig {
            waypoint_advance: WaypointAdvanceMode::WaypointWithinRange(100.0),
            route_deviation_tracking: RouteDeviationTracking::StaticThreshold {
                minimum_horizontal_accuracy: 10,
                max_acceptable_deviation: 10.0,
            },
            snapped_location_course_filtering: CourseFiltering::Raw,
            step_advance_condition: Arc::new(ManualStepCondition),
            arrival_step_advance_condition: Arc::new(ManualStepCondition),
            uzmatch: UzmatchConfig {
                enabled: true,
                ..UzmatchConfig::default()
            },
        };
        let controller = NavigationController::new(route, config);

        let t0 = SystemTime::UNIX_EPOCH + Duration::from_secs(2_000_000);
        let loc = |t: u64, lat: f64, speed: f64| UserLocation {
            coordinates: GeographicCoordinate { lng: 0.0005, lat },
            horizontal_accuracy: 5.0,
            course_over_ground: None,
            timestamp: t0 + Duration::from_secs(t),
            speed: Some(Speed {
                value: speed,
                accuracy: None,
            }),
        };

        let deviation_of = |state: &NavState| state.trip_state().deviation();

        // On route, then stop. The standing span reaches 7 s at t=8.
        let mut state = controller.get_initial_state(loc(0, 0.0, 4.0));
        for t in 1..=8 {
            state = controller.update_user_location(loc(t, 0.0, 0.0), state);
        }
        assert_eq!(deviation_of(&state), Some(RouteDeviation::NoDeviation));

        // While standing, jitter 50 m off the route line: deviation stays frozen.
        for t in 9..=11 {
            state = controller.update_user_location(loc(t, 0.0005, 0.0), state);
            assert_eq!(
                deviation_of(&state),
                Some(RouteDeviation::NoDeviation),
                "deviation must stay frozen while standing at t={t}"
            );
        }

        // Moving again off the route: deviation detection resumes.
        state = controller.update_user_location(loc(12, 0.0005, 8.0), state);
        state = controller.update_user_location(loc(13, 0.0005, 8.0), state);
        assert!(
            matches!(deviation_of(&state), Some(RouteDeviation::Deviation { .. })),
            "deviation must fire once moving off-route, got {:?}",
            deviation_of(&state)
        );
    }
    // ---------------------------------------------------------------------
    // UzNav pedestrian route-loss profile (openspec uznav-pedestrian-route-loss)
    // ---------------------------------------------------------------------

    /// Route: 111 m east, then a LEFT turn north (111 m), then east again.
    fn left_turn_route() -> Route {
        let step1 = gen_dummy_route_step(0.0, 0.0, 0.001, 0.0);
        let step2 = gen_dummy_route_step(0.001, 0.0, 0.001, 0.001);
        let step3 = gen_dummy_route_step(0.001, 0.001, 0.002, 0.001);
        gen_route_from_steps(vec![step1, step2, step3])
    }

    /// Pedestrian profile from design D3 (numbers are UzNav's own; the vendor
    /// ships one automotive constant set and no pedestrian Clinger).
    fn walk_uzmatch_config() -> UzmatchConfig {
        UzmatchConfig {
            enabled: true,
            snap_heading_min_speed_mps: 0.8,
            cling_distance_m: 12.0,
            cling_time_ms: 2000,
            heading_departure_enabled: true,
            heading_departure_min_speed_mps: 0.8,
            heading_departure_tolerance_deg: 60.0,
            heading_departure_confirmations: 3,
            ..UzmatchConfig::default()
        }
    }

    fn loss_config(
        max_acceptable_deviation: f64,
        uzmatch: UzmatchConfig,
    ) -> NavigationControllerConfig {
        use crate::deviation_detection::RouteDeviationTracking;
        use crate::navigation_controller::models::{CourseFiltering, WaypointAdvanceMode};
        use crate::navigation_controller::step_advance::conditions::ManualStepCondition;
        NavigationControllerConfig {
            waypoint_advance: WaypointAdvanceMode::WaypointWithinRange(100.0),
            route_deviation_tracking: RouteDeviationTracking::StaticThreshold {
                minimum_horizontal_accuracy: 32,
                max_acceptable_deviation,
            },
            snapped_location_course_filtering: CourseFiltering::Raw,
            step_advance_condition: Arc::new(ManualStepCondition),
            arrival_step_advance_condition: Arc::new(ManualStepCondition),
            uzmatch,
        }
    }

    /// Degrees of longitude/latitude per metre near the equator.
    const DEG_PER_METER: f64 = 1.0 / 111_320.0;
    const WALK_MPS: f64 = 1.4;

    fn walker_fix(
        t0: SystemTime,
        second: u64,
        lng: f64,
        lat: f64,
        course_deg: Option<f64>,
        speed_mps: f64,
    ) -> UserLocation {
        use crate::models::{CourseOverGround, GeographicCoordinate, Speed};
        UserLocation {
            coordinates: GeographicCoordinate { lng, lat },
            horizontal_accuracy: 5.0,
            course_over_ground: course_deg.map(|deg| CourseOverGround::new(deg, Some(10))),
            timestamp: t0 + Duration::from_secs(second),
            speed: Some(Speed {
                value: speed_mps,
                accuracy: None,
            }),
        }
    }

    fn is_off_route(state: &NavState) -> bool {
        state
            .trip_state()
            .deviation()
            .is_some_and(|deviation| deviation.is_completely_off_route())
    }

    /// Replay of the `uzmatch_cling_holds_then_releases_route_loss` track under
    /// the DEFAULT config: the drive profile is the vendor constants and the
    /// published deviation sequence is byte-identical to the pre-profile core.
    #[test]
    fn drive_profile_matches_vendor_constants() {
        use crate::models::{GeographicCoordinate, Speed};

        let defaults = UzmatchConfig::default();
        assert_eq!(defaults.cling_distance_m, 33.25);
        assert_eq!(defaults.cling_time_ms, 2000);
        assert_eq!(defaults.snap_heading_min_speed_mps, 4.0);
        assert!(!defaults.heading_departure_enabled);

        let step1 = gen_dummy_route_step(0.0, 0.0, 0.001, 0.0);
        let step2 = gen_dummy_route_step(0.001, 0.0, 0.002, 0.0);
        let step3 = gen_dummy_route_step(0.002, 0.0, 0.002, 0.001);
        let route = gen_route_from_steps(vec![step1, step2, step3]);
        let controller = NavigationController::new(
            route,
            loss_config(
                50.0,
                UzmatchConfig {
                    enabled: true,
                    ..UzmatchConfig::default()
                },
            ),
        );
        let t0 = SystemTime::UNIX_EPOCH + Duration::from_secs(1_000_000);
        let loc = |t_ms: u64, lng: f64, lat: f64| UserLocation {
            coordinates: GeographicCoordinate { lng, lat },
            horizontal_accuracy: 5.0,
            course_over_ground: None,
            timestamp: t0 + Duration::from_millis(t_ms),
            speed: Some(Speed {
                value: 10.0,
                accuracy: None,
            }),
        };
        let track = [
            (500, 0.00085, 0.0),
            (1_500, 0.00085, 0.0006),
            (3_500, 0.0009, 0.0006),
            (4_500, 0.001, 0.0),
            (5_000, 0.00105, 0.0006),
        ];
        let mut state = controller.get_initial_state(loc(0, 0.0008, 0.0));
        let published: Vec<bool> = track
            .iter()
            .map(|(t_ms, lng, lat)| {
                state = controller.update_user_location(loc(*t_ms, *lng, *lat), state.clone());
                is_off_route(&state)
            })
            .collect();
        assert_eq!(
            published,
            [false, false, true, false, false],
            "the drive profile must publish exactly the vendor Clinger sequence"
        );
    }

    /// With the pedestrian cling radius (12 m) a sustained loss is released
    /// ~22 m from the anchor, where the vendor radius (33.25 m) still holds.
    #[test]
    fn walk_cling_radius_releases_earlier() {
        use crate::models::{GeographicCoordinate, Speed};

        let route = left_turn_route();
        let t0 = SystemTime::UNIX_EPOCH + Duration::from_secs(1_000_000);
        let loc = |t_ms: u64, lng: f64, lat: f64| UserLocation {
            coordinates: GeographicCoordinate { lng, lat },
            horizontal_accuracy: 5.0,
            course_over_ground: None,
            timestamp: t0 + Duration::from_millis(t_ms),
            speed: Some(Speed {
                value: 1.4,
                accuracy: None,
            }),
        };
        // Anchor on the first step, then a fix ~22 m north of the line, 3 s later.
        let replay = |uzmatch: UzmatchConfig| {
            let controller = NavigationController::new(route.clone(), loss_config(10.0, uzmatch));
            let state = controller.get_initial_state(loc(0, 0.0005, 0.0));
            let state = controller.update_user_location(loc(1_000, 0.00051, 0.0), state);
            assert!(!is_off_route(&state));
            let state = controller.update_user_location(loc(4_000, 0.00052, 0.0002), state);
            is_off_route(&state)
        };

        assert!(
            !replay(UzmatchConfig {
                enabled: true,
                ..UzmatchConfig::default()
            }),
            "22 m from the anchor is inside the vendor cling radius"
        );
        assert!(
            replay(UzmatchConfig {
                enabled: true,
                cling_distance_m: 12.0,
                ..UzmatchConfig::default()
            }),
            "22 m from the anchor is outside the pedestrian cling radius"
        );
    }

    /// Walk the first step east at 1.4 m/s with course, then keep going
    /// straight past the LEFT turn. Returns the number of fixes past the turn
    /// vertex before the first full route loss, if any.
    fn fixes_past_turn_until_loss(course: impl Fn(f64) -> Option<f64>) -> Option<usize> {
        let controller =
            NavigationController::new(left_turn_route(), loss_config(25.0, walk_uzmatch_config()));
        let t0 = SystemTime::UNIX_EPOCH + Duration::from_secs(1_000_000);
        let step = WALK_MPS * DEG_PER_METER;
        let start_lng = 0.001 - 20.0 * DEG_PER_METER;
        let mut state = controller.get_initial_state(walker_fix(
            t0,
            0,
            start_lng,
            0.0,
            course(start_lng),
            WALK_MPS,
        ));
        let mut past_turn = 0usize;
        for second in 1..=40u64 {
            let lng = start_lng + step * second as f64;
            state = controller.update_user_location(
                walker_fix(t0, second, lng, 0.0, course(lng), WALK_MPS),
                state,
            );
            if lng > 0.001 {
                past_turn += 1;
            }
            if is_off_route(&state) {
                assert!(
                    past_turn > 0,
                    "route loss before the turn at second {second}"
                );
                return Some(past_turn);
            }
        }
        None
    }

    /// Spec "Straight past a left turn": with a credible course the loss is
    /// published on the 3rd moving fix past the maneuver (≈ +3 s), not when
    /// the distance threshold is finally crossed.
    #[test]
    fn heading_departure_straight_past_turn_publishes_within_confirmations() {
        assert_eq!(fixes_past_turn_until_loss(|_| Some(90.0)), Some(3));
    }

    /// Spec "Fixes without a course fall back to distance": no heading term,
    /// the pedestrian distance profile (25 m / 12 m cling) publishes on its
    /// own, well within +25 s.
    #[test]
    fn no_course_falls_back_to_distance() {
        let fixes = fixes_past_turn_until_loss(|_| None).expect("distance fallback must publish");
        assert!(
            fixes > 3,
            "without a course nothing may publish early: {fixes}"
        );
        assert!(
            fixes <= 25,
            "distance fallback must publish within 25 s: {fixes}"
        );
    }

    /// A course whose accuracy is worse than the tolerance is not credible and
    /// behaves like no course at all.
    #[test]
    fn incredible_course_falls_back_to_distance() {
        use crate::models::CourseOverGround;
        let controller =
            NavigationController::new(left_turn_route(), loss_config(25.0, walk_uzmatch_config()));
        let t0 = SystemTime::UNIX_EPOCH + Duration::from_secs(1_000_000);
        let step = WALK_MPS * DEG_PER_METER;
        let start_lng = 0.001 - 10.0 * DEG_PER_METER;
        let fix = |second: u64| UserLocation {
            course_over_ground: Some(CourseOverGround::new(90.0, Some(120))),
            ..walker_fix(
                t0,
                second,
                start_lng + step * second as f64,
                0.0,
                None,
                WALK_MPS,
            )
        };
        let mut state = controller.get_initial_state(fix(0));
        for second in 1..=12 {
            state = controller.update_user_location(fix(second), state);
            assert!(
                !is_off_route(&state),
                "an incredible course published at second {second}"
            );
        }
    }

    /// Spec "Wrong turn at the maneuver": the route turns LEFT (north), the
    /// walker turns RIGHT (south) at the vertex.
    #[test]
    fn heading_departure_wrong_turn() {
        let controller =
            NavigationController::new(left_turn_route(), loss_config(25.0, walk_uzmatch_config()));
        let t0 = SystemTime::UNIX_EPOCH + Duration::from_secs(1_000_000);
        let step = WALK_MPS * DEG_PER_METER;
        let mut state = controller.get_initial_state(walker_fix(
            t0,
            0,
            0.001 - 4.0 * step,
            0.0,
            Some(90.0),
            WALK_MPS,
        ));
        for second in 1..=3u64 {
            let lng = 0.001 - (4.0 - second as f64) * step;
            state = controller.update_user_location(
                walker_fix(t0, second, lng, 0.0, Some(90.0), WALK_MPS),
                state,
            );
            assert!(
                !is_off_route(&state),
                "approach fix {second} must stay on route"
            );
        }
        // At the vertex, turn right and walk south.
        let mut published_at = None;
        for n in 1..=6u64 {
            let lat = -(step * n as f64);
            state = controller.update_user_location(
                walker_fix(t0, 4 + n, 0.001, lat, Some(180.0), WALK_MPS),
                state,
            );
            if is_off_route(&state) {
                published_at = Some(n);
                break;
            }
        }
        assert_eq!(
            published_at,
            Some(3),
            "wrong turn must publish on the 3rd diverging fix"
        );
    }

    /// Spec "Brief heading noise while on route": the course lags the turn by
    /// two fixes (typical GPS behaviour when a walker rounds a corner), then
    /// agrees again. Nothing may publish.
    #[test]
    fn heading_noise_two_fixes_does_not_publish() {
        let controller =
            NavigationController::new(left_turn_route(), loss_config(25.0, walk_uzmatch_config()));
        let t0 = SystemTime::UNIX_EPOCH + Duration::from_secs(1_000_000);
        let step = WALK_MPS * DEG_PER_METER;
        let mut state = controller.get_initial_state(walker_fix(
            t0,
            0,
            0.001 - 3.0 * step,
            0.0,
            Some(90.0),
            WALK_MPS,
        ));
        state = controller.update_user_location(
            walker_fix(t0, 1, 0.001 - 2.0 * step, 0.0, Some(90.0), WALK_MPS),
            state,
        );
        state = controller.update_user_location(
            walker_fix(t0, 2, 0.001 - step, 0.0, Some(90.0), WALK_MPS),
            state,
        );
        // Two fixes at/just past the vertex still report the OLD course (90°)
        // although the walker is already turning north (route ahead 0°).
        state = controller
            .update_user_location(walker_fix(t0, 3, 0.001, 0.0, Some(90.0), WALK_MPS), state);
        assert!(!is_off_route(&state));
        state = controller.update_user_location(
            walker_fix(t0, 4, 0.001, 0.4 * step, Some(90.0), WALK_MPS),
            state,
        );
        assert!(!is_off_route(&state), "two noisy courses must not publish");
        // Course catches up; walker proceeds up the north leg.
        for n in 1..=8u64 {
            state = controller.update_user_location(
                walker_fix(t0, 4 + n, 0.001, step * n as f64, Some(0.0), WALK_MPS),
                state,
            );
            assert!(
                !is_off_route(&state),
                "on-route fix {n} after noise must not publish"
            );
        }
    }

    /// Spec "Far sidewalk of a wide avenue": 300 m parallel to the route at an
    /// 18 m offset with the course matching. The polyline has a straight
    /// shape vertex in the middle so a pinned projection at a non-turn vertex
    /// is exercised too.
    #[test]
    fn parallel_sidewalk_18m_300m_stays_on_route() {
        let step1 = gen_dummy_route_step(0.0, 0.0, 0.0015, 0.0);
        let step2 = gen_dummy_route_step(0.0015, 0.0, 0.003, 0.0);
        let step3 = gen_dummy_route_step(0.003, 0.0, 0.003, 0.001);
        let route = gen_route_from_steps(vec![step1, step2, step3]);
        let controller = NavigationController::new(route, loss_config(25.0, walk_uzmatch_config()));
        let t0 = SystemTime::UNIX_EPOCH + Duration::from_secs(1_000_000);
        let step = WALK_MPS * DEG_PER_METER;
        let lat = 18.0 * DEG_PER_METER;
        let mut state =
            controller.get_initial_state(walker_fix(t0, 0, 0.0, lat, Some(90.0), WALK_MPS));
        let seconds = (300.0 / WALK_MPS) as u64;
        for second in 1..=seconds {
            // ±25° course wobble, always inside the 30° the spec allows.
            let course = 90.0 + if second % 2 == 0 { 25.0 } else { -25.0 };
            state = controller.update_user_location(
                walker_fix(
                    t0,
                    second,
                    step * second as f64,
                    lat,
                    Some(course),
                    WALK_MPS,
                ),
                state,
            );
            assert!(
                !is_off_route(&state),
                "parallel sidewalk published a route loss at second {second}"
            );
        }
    }

    /// Spec "Standing at a crossing": below the standing speed the fixes may
    /// drift up to 20 m with an arbitrary course; nothing publishes, before
    /// and after the standing gate engages.
    #[test]
    fn standing_drift_does_not_publish() {
        let controller =
            NavigationController::new(left_turn_route(), loss_config(25.0, walk_uzmatch_config()));
        let t0 = SystemTime::UNIX_EPOCH + Duration::from_secs(1_000_000);
        let mut state =
            controller.get_initial_state(walker_fix(t0, 0, 0.001, 0.0, Some(90.0), WALK_MPS));
        let drift = [
            (12.0, 0.0, 210.0),
            (0.0, 15.0, 30.0),
            (-14.0, 8.0, 300.0),
            (19.0, -5.0, 120.0),
            (5.0, 19.0, 250.0),
            (-18.0, -2.0, 90.0),
            (0.0, 0.0, 180.0),
            (16.0, 11.0, 45.0),
            (-9.0, -16.0, 330.0),
            (19.0, 0.0, 200.0),
        ];
        for (second, (east_m, north_m, course)) in drift.iter().enumerate() {
            state = controller.update_user_location(
                walker_fix(
                    t0,
                    second as u64 + 1,
                    0.001 + east_m * DEG_PER_METER,
                    north_m * DEG_PER_METER,
                    Some(*course),
                    0.3,
                ),
                state,
            );
            assert!(
                !is_off_route(&state),
                "standing drift published at fix {second}"
            );
        }
    }

    /// Not in the design table but implied by the sidewalk requirement: a
    /// walker crossing the avenue perpendicular to a route that runs along it
    /// holds a 90° course for many seconds while staying inside 25 m. The
    /// heading detector only counts fixes pinned at a route vertex ("after
    /// the maneuver point"), so a mid-segment crossing never publishes.
    #[test]
    fn perpendicular_crossing_mid_segment_does_not_publish() {
        // A long east step so the north leg (445 m ahead) is outside the snap
        // bias: the crossing fix can only bind to the segment being crossed.
        let step1 = gen_dummy_route_step(0.0, 0.0, 0.004, 0.0);
        let step2 = gen_dummy_route_step(0.004, 0.0, 0.004, 0.001);
        let route = gen_route_from_steps(vec![step1, step2]);
        let controller = NavigationController::new(route, loss_config(25.0, walk_uzmatch_config()));
        let t0 = SystemTime::UNIX_EPOCH + Duration::from_secs(1_000_000);
        let step = WALK_MPS * DEG_PER_METER;
        let mut state =
            controller.get_initial_state(walker_fix(t0, 0, 0.0004, 0.0, Some(90.0), WALK_MPS));
        state = controller.update_user_location(
            walker_fix(t0, 1, 0.0004 + step, 0.0, Some(90.0), WALK_MPS),
            state,
        );
        // Cross north for 10 s (14 m), then walk east on the far side.
        for n in 1..=10u64 {
            state = controller.update_user_location(
                walker_fix(
                    t0,
                    1 + n,
                    0.0004 + step,
                    step * n as f64,
                    Some(0.0),
                    WALK_MPS,
                ),
                state,
            );
            assert!(!is_off_route(&state), "crossing published at fix {n}");
        }
        for n in 1..=10u64 {
            state = controller.update_user_location(
                walker_fix(
                    t0,
                    11 + n,
                    0.0004 + step * (n + 1) as f64,
                    10.0 * step,
                    Some(90.0),
                    WALK_MPS,
                ),
                state,
            );
            assert!(!is_off_route(&state), "far side published at fix {n}");
        }
    }

    /// Spec "Reroute installs a new route": a fresh controller/state starts
    /// clean — a single noisy fix does not publish, a second real departure
    /// publishes on the same 3-fix budget as the first.
    #[test]
    fn return_to_route_rearms_detection() {
        assert_eq!(fixes_past_turn_until_loss(|_| Some(90.0)), Some(3));

        // The reroute: a new route from the walker's position east, turning north later.
        let step1 = gen_dummy_route_step(0.00105, 0.0, 0.002, 0.0);
        let step2 = gen_dummy_route_step(0.002, 0.0, 0.002, 0.001);
        let route = gen_route_from_steps(vec![step1, step2]);
        let controller = NavigationController::new(route, loss_config(25.0, walk_uzmatch_config()));
        let t0 = SystemTime::UNIX_EPOCH + Duration::from_secs(2_000_000);
        let step = WALK_MPS * DEG_PER_METER;
        // First fix on the new route is at its start vertex with a noisy course.
        let mut state =
            controller.get_initial_state(walker_fix(t0, 0, 0.00105, 0.0, Some(200.0), WALK_MPS));
        assert!(
            !is_off_route(&state),
            "a single noisy fix on the new route must not publish"
        );
        let mut past_turn = 0usize;
        let mut published = None;
        for second in 1..=90u64 {
            let lng = 0.00105 + step * second as f64;
            state = controller.update_user_location(
                walker_fix(t0, second, lng, 0.0, Some(90.0), WALK_MPS),
                state,
            );
            if lng > 0.002 {
                past_turn += 1;
            }
            if is_off_route(&state) {
                published = Some(past_turn);
                break;
            }
        }
        assert_eq!(
            published,
            Some(3),
            "the second departure must use the same budget"
        );
    }
}
