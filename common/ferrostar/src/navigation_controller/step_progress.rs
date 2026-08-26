use crate::{
    models::Route,
    navigation_controller::uzmatch::{RoutePosition, snap::cumulative_lengths},
};
use geo::{Distance, Haversine, Point};

#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct IndexedStepPosition {
    pub(super) current_step_geometry_index: u64,
    pub(super) distance_to_next_maneuver: f64,
}

#[derive(Debug, Clone, Copy)]
struct IndexedStep {
    first_route_segment: usize,
    end_route_segment: usize,
    distance_at_end: f64,
}

/// Validated route-lifetime mapping from route-global segments to step-local progress.
///
/// This is unavailable when the independently supplied route and step geometries
/// do not form one exact contiguous segment sequence. The controller then keeps
/// its legacy current-step projection and progress scan.
pub(super) struct StepProgressIndex {
    route_vertex_distances: Vec<f64>,
    route_segment_lengths: Vec<f64>,
    steps: Vec<IndexedStep>,
}

impl StepProgressIndex {
    pub(super) fn new(route: &Route) -> Option<Self> {
        if route.steps.is_empty() || route.geometry.is_empty() {
            return None;
        }

        let route_vertex_distances = cumulative_lengths(&route.geometry);
        let route_segment_lengths = route
            .geometry
            .windows(2)
            .map(|segment| Haversine.distance(Point::from(segment[0]), Point::from(segment[1])))
            .collect();
        let mut first_route_segment = 0usize;
        let mut steps = Vec::with_capacity(route.steps.len());

        for step in &route.steps {
            let step_segment_count = step.geometry.len().checked_sub(1)?;
            let end_route_segment = first_route_segment.checked_add(step_segment_count)?;
            let route_slice = route
                .geometry
                .get(first_route_segment..=end_route_segment)?;
            if route_slice != step.geometry.as_slice() {
                return None;
            }

            steps.push(IndexedStep {
                first_route_segment,
                end_route_segment,
                distance_at_end: *route_vertex_distances.get(end_route_segment)?,
            });
            first_route_segment = end_route_segment;
        }

        if first_route_segment + 1 != route.geometry.len() {
            return None;
        }

        Some(Self {
            route_vertex_distances,
            route_segment_lengths,
            steps,
        })
    }

    pub(super) fn locate(
        &self,
        route_position: RoutePosition,
        remaining_steps_len: usize,
    ) -> Option<IndexedStepPosition> {
        let current_step_index = self.steps.len().checked_sub(remaining_steps_len)?;
        let step = *self.steps.get(current_step_index)?;
        let route_segment = usize::try_from(route_position.segment_index).ok()?;
        if route_segment < step.first_route_segment {
            return None;
        }
        if route_segment >= step.end_route_segment {
            // Vendor model (NaviKit `IndexedRoute`): the maneuver is a fixed
            // route-global position, so a match that already passed it is zero
            // distance ahead. Falling back to the frame-local step projection
            // here re-projected the passed coordinate onto the step geometry,
            // which reinflated the distance and rewound the geometry index at
            // hairpins and nearby parallel legs.
            if route_segment >= self.route_segment_lengths.len() {
                return None;
            }
            return Some(IndexedStepPosition {
                current_step_geometry_index: (step.end_route_segment - step.first_route_segment)
                    .saturating_sub(1) as u64,
                distance_to_next_maneuver: 0.0,
            });
        }

        let distance_at_segment_start = *self.route_vertex_distances.get(route_segment)?;
        let segment_length = *self.route_segment_lengths.get(route_segment)?;
        let segment_offset = route_position.segment_offset_meters;
        if !segment_offset.is_finite() || segment_offset < 0.0 || segment_offset > segment_length {
            return None;
        }

        let distance_to_next_maneuver =
            step.distance_at_end - distance_at_segment_start - segment_offset;
        if !distance_to_next_maneuver.is_finite() || distance_to_next_maneuver < 0.0 {
            return None;
        }

        Some(IndexedStepPosition {
            current_step_geometry_index: (route_segment - step.first_route_segment) as u64,
            distance_to_next_maneuver,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::StepProgressIndex;
    use crate::{
        algorithms::calculate_trip_progress,
        models::{GeographicCoordinate, Route, UserLocation},
        navigation_controller::{
            NavigationController, Navigator,
            models::TripState,
            step_advance::conditions::ManualStepCondition,
            test_helpers::get_test_navigation_controller_config,
            test_helpers::{gen_route_from_steps, gen_route_step_with_coords},
            uzmatch::{RoutePosition, UzLocationClass, UzmatchConfig, UzmatchSnapshot},
        },
    };
    use geo::{Distance, Haversine, Point, coord};
    use std::{sync::Arc, time::SystemTime};

    fn aligned_route(step_coordinates: Vec<Vec<geo::Coord>>) -> Route {
        let steps = step_coordinates
            .into_iter()
            .map(gen_route_step_with_coords)
            .collect::<Vec<_>>();
        let geometry = steps
            .iter()
            .enumerate()
            .flat_map(|(step_index, step)| {
                step.geometry
                    .iter()
                    .skip(usize::from(step_index > 0))
                    .copied()
            })
            .collect();
        Route {
            geometry,
            ..gen_route_from_steps(steps)
        }
    }

    fn route_position(segment_index: u64, segment_offset_meters: f64) -> RoutePosition {
        RoutePosition {
            segment_index,
            segment_offset_meters,
            distance_along_route_meters: 0.0,
            coordinates: GeographicCoordinate { lat: 0.0, lng: 0.0 },
            course_over_ground: None,
        }
    }

    #[test]
    fn maps_route_segment_to_the_current_step() {
        let route = aligned_route(vec![
            vec![
                coord!(x: 0.0, y: 0.0),
                coord!(x: 0.00001, y: 0.0),
                coord!(x: 0.00002, y: 0.0),
            ],
            vec![
                coord!(x: 0.00002, y: 0.0),
                coord!(x: 0.00003, y: 0.0),
                coord!(x: 0.00004, y: 0.0),
            ],
        ]);
        let index = StepProgressIndex::new(&route).expect("aligned route must be indexed");

        let position = index
            .locate(route_position(3, 0.0), 1)
            .expect("global segment 3 belongs to the second current step");

        assert_eq!(position.current_step_geometry_index, 1);
        assert!((position.distance_to_next_maneuver - 1.11195).abs() < 0.01);
    }

    #[test]
    fn repeated_coordinates_are_resolved_by_segment_identity() {
        let route = aligned_route(vec![
            vec![
                coord!(x: 0.0, y: 0.0),
                coord!(x: 0.00001, y: 0.0),
                coord!(x: 0.0, y: 0.0),
            ],
            vec![coord!(x: 0.0, y: 0.0), coord!(x: 0.0, y: 0.00001)],
        ]);
        let index = StepProgressIndex::new(&route).expect("aligned loop must be indexed");

        let position = index
            .locate(route_position(2, 0.0), 1)
            .expect("segment identity must select the second step");

        assert_eq!(position.current_step_geometry_index, 0);
    }

    #[test]
    fn rejects_route_candidate_gaps_instead_of_guessing() {
        let steps = vec![
            gen_route_step_with_coords(vec![coord!(x: 0.0, y: 0.0), coord!(x: 0.00001, y: 0.0)]),
            gen_route_step_with_coords(vec![
                coord!(x: 0.00002, y: 0.0),
                coord!(x: 0.00003, y: 0.0),
            ]),
        ];
        let mut route = gen_route_from_steps(steps);
        route.geometry = vec![
            GeographicCoordinate { lat: 0.0, lng: 0.0 },
            GeographicCoordinate {
                lat: 0.0,
                lng: 0.00001,
            },
            GeographicCoordinate {
                lat: 0.0,
                lng: 0.00002,
            },
            GeographicCoordinate {
                lat: 0.0,
                lng: 0.00003,
            },
        ];

        assert!(StepProgressIndex::new(&route).is_none());
    }

    #[test]
    fn match_past_step_end_clamps_to_the_maneuver() {
        let route = aligned_route(vec![
            vec![coord!(x: 0.0, y: 0.0), coord!(x: 0.00001, y: 0.0)],
            vec![coord!(x: 0.00001, y: 0.0), coord!(x: 0.00002, y: 0.0)],
        ]);
        let index = StepProgressIndex::new(&route).expect("aligned route must be indexed");

        let position = index
            .locate(route_position(1, 0.5), 2)
            .expect("a match past the current step's end is the maneuver, not a fallback");

        assert_eq!(position.current_step_geometry_index, 0);
        assert_eq!(position.distance_to_next_maneuver, 0.0);
    }

    #[test]
    fn hairpin_match_past_step_end_does_not_reinflate_distance() {
        // Step 0 is a hairpin that returns next to its own origin; step 1 passes
        // within ~1 m of step 0's first segment. A fix matched onto step 1 while
        // step 0 is still current previously fell back to the frame-local
        // step projection, which snapped to the early hairpin leg and reported
        // ~90 m to a maneuver the user had already passed.
        let route = aligned_route(vec![
            vec![
                coord!(x: 0.0, y: 0.0),
                coord!(x: 0.0004, y: 0.0),
                coord!(x: 0.0004, y: 0.00003),
                coord!(x: 0.00001, y: 0.00003),
            ],
            vec![coord!(x: 0.00001, y: 0.00003), coord!(x: 0.00001, y: 0.0)],
        ]);
        let mut config = get_test_navigation_controller_config(Arc::new(ManualStepCondition));
        config.uzmatch.enabled = true;
        let controller = NavigationController::new(route.clone(), config);
        assert!(controller.step_progress_index.is_some());

        let match_coordinates = GeographicCoordinate {
            lat: 0.00001,
            lng: 0.00001,
        };
        let location = UserLocation {
            coordinates: match_coordinates,
            horizontal_accuracy: 5.0,
            course_over_ground: None,
            timestamp: SystemTime::UNIX_EPOCH,
            speed: None,
        };
        let snapshot = UzmatchSnapshot {
            route_position: Some(RoutePosition {
                segment_index: 3,
                segment_offset_meters: 2.2,
                distance_along_route_meters: 0.0,
                coordinates: match_coordinates,
                course_over_ground: None,
            }),
            is_standing: false,
            location_class: UzLocationClass::Fine,
            filtered_speed_mps: None,
        };

        let (current_step_geometry_index, snapped_user_location, progress) = controller.step_state(
            location,
            &route.steps[0],
            &route.steps,
            Some(&snapshot),
        );

        assert_eq!(current_step_geometry_index, Some(2));
        assert_eq!(snapped_user_location.coordinates, match_coordinates);
        assert_eq!(progress.distance_to_next_maneuver, 0.0);
        assert!(
            (progress.distance_remaining - route.steps[1].distance).abs() < 1e-9,
            "past the maneuver only the next step's distance remains"
        );
    }

    #[test]
    fn does_not_map_a_match_from_another_step() {
        let route = aligned_route(vec![
            vec![coord!(x: 0.0, y: 0.0), coord!(x: 0.00001, y: 0.0)],
            vec![coord!(x: 0.00001, y: 0.0), coord!(x: 0.00002, y: 0.0)],
        ]);
        let index = StepProgressIndex::new(&route).expect("aligned route must be indexed");

        assert!(index.locate(route_position(0, 0.0), 1).is_none());
    }

    #[test]
    fn accepts_a_segment_endpoint_after_a_long_prefix() {
        let coordinates = (0..10_001)
            .map(|point_index| coord!(x: point_index as f64 * 0.00001, y: 0.0))
            .collect::<Vec<_>>();
        let route = aligned_route(vec![coordinates]);
        let index = StepProgressIndex::new(&route).expect("aligned route must be indexed");
        let segment_index = 4_999usize;
        let segment_offset = Haversine.distance(
            Point::from(route.geometry[segment_index]),
            Point::from(route.geometry[segment_index + 1]),
        );

        assert!(
            index
                .locate(route_position(segment_index as u64, segment_offset), 1)
                .is_some()
        );
    }

    #[test]
    fn controller_fast_path_matches_legacy_progress() {
        let route = aligned_route(vec![vec![
            coord!(x: 0.0, y: 0.0),
            coord!(x: 0.00001, y: 0.0),
            coord!(x: 0.00002, y: 0.0),
            coord!(x: 0.00003, y: 0.0),
        ]]);
        let mut config = get_test_navigation_controller_config(Arc::new(ManualStepCondition));
        config.uzmatch = UzmatchConfig {
            enabled: true,
            ..UzmatchConfig::default()
        };
        let controller = NavigationController::new(route.clone(), config);
        assert!(controller.step_progress_index.is_some());

        let state = controller.get_initial_state(UserLocation {
            coordinates: GeographicCoordinate {
                lat: 0.0,
                lng: 0.000015,
            },
            horizontal_accuracy: 5.0,
            course_over_ground: None,
            timestamp: SystemTime::UNIX_EPOCH,
            speed: None,
        });
        let TripState::Navigating {
            current_step_geometry_index,
            snapped_user_location,
            remaining_steps,
            progress,
            ..
        } = state.trip_state()
        else {
            panic!("expected navigating state");
        };
        let current_step_linestring = remaining_steps[0].get_linestring();
        let legacy_progress = calculate_trip_progress(
            &Point::from(snapped_user_location),
            &current_step_linestring,
            &remaining_steps,
        );

        assert_eq!(current_step_geometry_index, Some(1));
        assert!(
            (progress.distance_to_next_maneuver - legacy_progress.distance_to_next_maneuver).abs()
                < 1e-9
        );
        assert!((progress.distance_remaining - legacy_progress.distance_remaining).abs() < 1e-9);
        assert!((progress.duration_remaining - legacy_progress.duration_remaining).abs() < 1e-9);
    }

    #[test]
    fn one_point_arrival_step_does_not_disable_prior_steps() {
        let mut route = aligned_route(vec![vec![
            coord!(x: 0.0, y: 0.0),
            coord!(x: 0.00001, y: 0.0),
            coord!(x: 0.00002, y: 0.0),
        ]]);
        let mut arrival = route.steps[0].clone();
        arrival.geometry = vec![*route.geometry.last().expect("route has an endpoint")];
        arrival.distance = 0.0;
        arrival.duration = 0.0;
        route.steps.push(arrival);

        let index = StepProgressIndex::new(&route)
            .expect("terminal point step must not disable preceding step index");

        assert!(index.locate(route_position(0, 0.0), 2).is_some());
        assert!(
            index.locate(route_position(1, 0.0), 1).is_none(),
            "a zero-segment arrival step must use the legacy fallback"
        );
    }

    #[test]
    fn controller_rejects_fast_path_for_foreign_current_step() {
        let route = aligned_route(vec![vec![
            coord!(x: 0.0, y: 0.0),
            coord!(x: 0.00001, y: 0.0),
            coord!(x: 0.00002, y: 0.0),
        ]]);
        let foreign_route = aligned_route(vec![vec![
            coord!(x: 0.0, y: 0.0),
            coord!(x: 0.0, y: 0.00001),
            coord!(x: 0.0, y: 0.00002),
        ]]);
        let mut config = get_test_navigation_controller_config(Arc::new(ManualStepCondition));
        config.uzmatch.enabled = true;
        let controller = NavigationController::new(route, config);
        let location = UserLocation {
            coordinates: GeographicCoordinate {
                lat: 0.0,
                lng: 0.000015,
            },
            horizontal_accuracy: 5.0,
            course_over_ground: None,
            timestamp: SystemTime::UNIX_EPOCH,
            speed: None,
        };
        let snapshot = UzmatchSnapshot {
            route_position: Some(RoutePosition {
                segment_index: 1,
                segment_offset_meters: 0.5,
                distance_along_route_meters: 1.5,
                coordinates: location.coordinates,
                course_over_ground: None,
            }),
            is_standing: false,
            location_class: UzLocationClass::Fine,
            filtered_speed_mps: None,
        };

        assert!(
            controller
                .step_state_from_match(location, &foreign_route.steps, Some(&snapshot))
                .is_none()
        );
    }
}
