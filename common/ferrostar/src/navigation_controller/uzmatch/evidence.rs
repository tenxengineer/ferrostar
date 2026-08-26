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

use geo::Coord;

use super::{RouteSnapIndex, UzmatchState, snap::MAX_SNAP_CANDIDATES};
use crate::{
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

fn location(point_count: usize, tick: u64) -> UserLocation {
    UserLocation {
        coordinates: GeographicCoordinate {
            lat: 0.0,
            lng: (point_count / 2) as f64 * 0.000_01,
        },
        horizontal_accuracy: 5.0,
        course_over_ground: Some(CourseOverGround::new(90.0, None)),
        timestamp: SystemTime::UNIX_EPOCH + Duration::from_secs(tick),
        speed: Some(Speed {
            value: 10.0,
            accuracy: None,
        }),
    }
}

fn percentile_micros(samples: &[Duration], percentile: usize) -> u128 {
    let mut micros = samples.iter().map(Duration::as_micros).collect::<Vec<_>>();
    micros.sort_unstable();
    let index = (micros.len() - 1) * percentile / 100;
    micros[index]
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
    point_count: usize,
    route: crate::models::Route,
    config: super::UzmatchConfig,
) -> Vec<Duration> {
    let mut controller_config =
        get_test_navigation_controller_config(Arc::new(ManualStepCondition));
    controller_config.uzmatch = config;
    let controller = NavigationController::new(route, controller_config);
    let mut state = controller.get_initial_state(location(point_count, 0));
    let mut samples = Vec::with_capacity(SAMPLE_COUNT as usize);
    for tick in 1..=SAMPLE_COUNT {
        let started = Instant::now();
        state = black_box(controller.update_user_location(location(point_count, tick), state));
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
        let controller = controller_samples(point_count, route, config.clone());

        eprintln!(
            "temporal_evidence,{point_count},{index_build_us},{},{},{},{}",
            percentile_micros(&core, 50),
            percentile_micros(&core, 95),
            percentile_micros(&controller, 50),
            percentile_micros(&controller, 95),
        );
    }
}
