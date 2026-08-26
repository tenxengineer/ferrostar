//! Manual host-local replay and performance evidence for the temporal matcher.
//!
//! The vendor matcher debug surface records each signal layer, candidate parent,
//! and likelihood components. This runner keeps the same boundary separation:
//! route-index construction, matcher-core updates, and whole-controller updates
//! are measured independently so controller state cloning is not attributed to
//! the bounded candidate recurrence.

use std::{
    hint::black_box,
    sync::Arc,
    time::{Duration, Instant, SystemTime},
};

use geo::{Coord, Point};

use super::{RouteSnapIndex, UzmatchState, snap::MAX_SNAP_CANDIDATES};
use crate::{
    algorithms::{calculate_trip_progress, index_of_closest_segment_origin},
    models::{CourseOverGround, GeographicCoordinate, Speed, UserLocation},
    navigation_controller::{
        NavigationController, Navigator,
        models::TripState,
        step_advance::conditions::ManualStepCondition,
        test_helpers::{
            gen_route_from_steps, gen_route_step_with_coords, get_test_navigation_controller_config,
        },
    },
};

const SAMPLE_COUNT: u64 = 20;

fn straight_route(point_count: usize) -> crate::models::Route {
    let coordinates = (0..point_count)
        .map(|index| Coord {
            x: index as f64 * 0.000_01,
            y: 0.0,
        })
        .collect();
    gen_route_from_steps(vec![gen_route_step_with_coords(coordinates)])
}

fn segmented_route(step_count: usize, points_per_step: usize) -> crate::models::Route {
    let steps = (0..step_count)
        .map(|step_index| {
            let first_point = step_index * (points_per_step - 1);
            let coordinates = (0..points_per_step)
                .map(|point_index| Coord {
                    x: (first_point + point_index) as f64 * 0.000_01,
                    y: 0.0,
                })
                .collect();
            gen_route_step_with_coords(coordinates)
        })
        .collect();
    gen_route_from_steps(steps)
}

fn location_at(lng: f64, tick: u64) -> UserLocation {
    UserLocation {
        coordinates: GeographicCoordinate { lat: 0.0, lng },
        horizontal_accuracy: 5.0,
        course_over_ground: Some(CourseOverGround::new(90.0, None)),
        timestamp: SystemTime::UNIX_EPOCH + Duration::from_secs(tick),
        speed: Some(Speed {
            value: 10.0,
            accuracy: None,
        }),
    }
}

fn location(point_count: usize, tick: u64) -> UserLocation {
    location_at((point_count / 2) as f64 * 0.000_01, tick)
}

fn route_location(route: &crate::models::Route, tick: u64) -> UserLocation {
    let current_step = &route.steps[0];
    location_at(
        current_step.geometry[current_step.geometry.len() / 2].lng,
        tick,
    )
}

fn percentile_micros(samples: &[Duration], percentile: usize) -> u128 {
    let mut micros = samples.iter().map(Duration::as_micros).collect::<Vec<_>>();
    micros.sort_unstable();
    let index = (micros.len() - 1) * percentile / 100;
    micros[index]
}

fn duration_samples<T>(mut operation: impl FnMut() -> T) -> Vec<Duration> {
    let mut samples = Vec::with_capacity(SAMPLE_COUNT as usize);
    for _ in 0..SAMPLE_COUNT {
        let started = Instant::now();
        black_box(operation());
        samples.push(started.elapsed());
    }
    samples
}

fn component_medians(
    route: crate::models::Route,
    fix: UserLocation,
    config: super::UzmatchConfig,
) -> [u128; 5] {
    let mut controller_config =
        get_test_navigation_controller_config(Arc::new(ManualStepCondition));
    controller_config.uzmatch = config;
    let controller = NavigationController::new(route, controller_config);
    let state = controller.get_initial_state(fix);
    let trip_state = state.trip_state();
    let TripState::Navigating {
        remaining_steps, ..
    } = &trip_state
    else {
        panic!("initial controller state must be navigating");
    };
    let current_step = &remaining_steps[0];
    let current_step_linestring = current_step.get_linestring();
    let snapped_point = Point::from(fix);

    let state_clone = duration_samples(|| state.trip_state());
    let step_clone = duration_samples(|| current_step.clone());
    let linestring_build = duration_samples(|| current_step.get_linestring());
    let closest_segment =
        duration_samples(|| index_of_closest_segment_origin(fix, &current_step_linestring));
    let progress = duration_samples(|| {
        calculate_trip_progress(&snapped_point, &current_step_linestring, remaining_steps)
    });

    [
        percentile_micros(&state_clone, 50),
        percentile_micros(&step_clone, 50),
        percentile_micros(&linestring_build, 50),
        percentile_micros(&closest_segment, 50),
        percentile_micros(&progress, 50),
    ]
}

fn core_samples(
    point_count: usize,
    route_index: &RouteSnapIndex,
    config: &super::UzmatchConfig,
) -> Vec<Duration> {
    let mut state =
        UzmatchState::default().update_with_index(&location(point_count, 0), route_index, config);
    let mut samples = Vec::with_capacity(SAMPLE_COUNT as usize);
    for tick in 1..=SAMPLE_COUNT {
        let fix = location(point_count, tick);
        let started = Instant::now();
        state = black_box(state.update_with_index(&fix, route_index, config));
        samples.push(started.elapsed());
    }
    assert!(state.route_position.is_some());
    assert!(state.temporal_match.candidates.len() <= MAX_SNAP_CANDIDATES);
    samples
}

fn controller_samples(
    location_lng: f64,
    route: crate::models::Route,
    config: super::UzmatchConfig,
) -> Vec<Duration> {
    let mut controller_config =
        get_test_navigation_controller_config(Arc::new(ManualStepCondition));
    controller_config.uzmatch = config;
    let controller = NavigationController::new(route, controller_config);
    let mut state = controller.get_initial_state(location_at(location_lng, 0));
    let mut samples = Vec::with_capacity(SAMPLE_COUNT as usize);
    for tick in 1..=SAMPLE_COUNT {
        let started = Instant::now();
        state = black_box(controller.update_user_location(location_at(location_lng, tick), state));
        samples.push(started.elapsed());
    }
    assert!(matches!(
        state.trip_state(),
        TripState::Navigating {
            uzmatch: Some(_),
            ..
        }
    ));
    samples
}

#[test]
#[ignore = "manual host-local performance evidence; no CI timing threshold"]
fn long_route_matcher_and_controller_timings() {
    let config = super::UzmatchConfig {
        enabled: true,
        ..super::UzmatchConfig::default()
    };

    eprintln!(
        "temporal_evidence,points,index_build_us,core_median_us,core_p95_us,controller_median_us,controller_p95_us"
    );
    for point_count in [1_001, 10_001, 50_001] {
        let route = straight_route(point_count);
        let index_started = Instant::now();
        let route_index = RouteSnapIndex::new(&route.geometry);
        let index_build_us = index_started.elapsed().as_micros();
        let core = core_samples(point_count, &route_index, &config);
        let controller =
            controller_samples((point_count / 2) as f64 * 0.000_01, route, config.clone());

        eprintln!(
            "temporal_evidence,{point_count},{index_build_us},{},{},{},{}",
            percentile_micros(&core, 50),
            percentile_micros(&core, 95),
            percentile_micros(&controller, 50),
            percentile_micros(&controller, 95),
        );
    }

    eprintln!(
        "temporal_components,points,state_clone_median_us,current_step_clone_median_us,linestring_build_median_us,closest_segment_median_us,progress_median_us"
    );
    for point_count in [1_001, 10_001, 50_001] {
        let route = straight_route(point_count);
        let medians = component_medians(route, location(point_count, 0), config.clone());
        eprintln!(
            "temporal_components,{point_count},{},{},{},{},{}",
            medians[0], medians[1], medians[2], medians[3], medians[4]
        );
    }

    eprintln!(
        "temporal_shape,steps,total_step_points,current_step_points,state_clone_median_us,controller_median_us"
    );
    for (step_count, points_per_step) in [(1, 50_001), (100, 501), (1_000, 51)] {
        let route = segmented_route(step_count, points_per_step);
        let fix = route_location(&route, 0);
        let state_clone_median = component_medians(route.clone(), fix, config.clone())[0];
        let controller = controller_samples(fix.coordinates.lng, route, config.clone());
        eprintln!(
            "temporal_shape,{step_count},{},{points_per_step},{state_clone_median},{}",
            step_count * points_per_step,
            percentile_micros(&controller, 50)
        );
    }
}
