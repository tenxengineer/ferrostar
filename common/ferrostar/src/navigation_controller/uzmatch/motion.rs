//! Port of NavKit route-bound location streaming (location_guide/location_streamer/).
//!
//! Provenance: maps-product `backend/mobile/libs/directions/guidance/location_guide/`
//! `location_streamer/{bound_motion.h,route_bound_motion.h,one_dimensional_motion.{h,cpp},cubic_bezier_curve.h}`
//! and `location_streamer.cpp` (advance/reset policy), `location_guide_config.h` (constants).
//!
//! The streamer renders a continuous 20 Hz position on the route polyline
//! between discrete (typically 1 Hz) location fixes, using a cubic-Bezier
//! one-dimensional motion along the route's distance axis.

#[cfg(all(feature = "std", not(feature = "web-time")))]
use std::time::{Duration, SystemTime};
#[cfg(feature = "web-time")]
use web_time::{Duration, SystemTime};

use std::sync::Mutex;

use serde::{Deserialize, Serialize};

use crate::models::{CourseOverGround, GeographicCoordinate, Route, UserLocation};

use super::snap::{cumulative_lengths, point_at_distance};

/// Vendor `PATH_CURVATURE` (one_dimensional_motion.cpp); must be in (0, 1).
const PATH_CURVATURE: f64 = 0.5;
/// Vendor `PATH_POINT_NUMBER`: Bezier sampling resolution.
const PATH_POINT_NUMBER: usize = 50;
/// Vendor `MAXIMAL_POSSIBLE_SPEED` (200 km/h) in m/s.
const MAXIMAL_POSSIBLE_SPEED: f64 = 200.0 / 3.6;
/// Vendor `MAXIMAL_MOTION_LOCATION_DELAY`: a new fix arriving later than this
/// after the current rendered position makes the motion unavailable.
const MAXIMAL_MOTION_LOCATION_DELAY: Duration = Duration::from_secs(2);
/// Vendor `JUMP_DISTANCE_THRESHOLD`: a backward jump larger than this kills the motion.
const JUMP_DISTANCE_THRESHOLD: f64 = 10.0;
/// Vendor `MAX_REASONABLE_ADVANCEMENT` (location_streamer.cpp): longer gaps
/// between ticks reset the streamer instead of animating.
const MAX_REASONABLE_ADVANCEMENT: Duration = Duration::from_secs(10);

/// Cubic Bezier curve over 2D points (vendor `CubicBezierCurve`).
#[derive(Debug, Clone, Copy)]
struct CubicBezierCurve {
    start: [f64; 2],
    start_control: [f64; 2],
    finish_control: [f64; 2],
    finish: [f64; 2],
}

impl CubicBezierCurve {
    fn point(&self, position: f64) -> [f64; 2] {
        let t = position;
        let s = 1.0 - t;
        let mut out = [0.0; 2];
        for i in 0..2 {
            out[i] = s * s * s * self.start[i]
                + 3.0 * s * s * t * self.start_control[i]
                + 3.0 * s * t * t * self.finish_control[i]
                + t * t * t * self.finish[i];
        }
        out
    }

    fn derivative(&self, position: f64) -> [f64; 2] {
        let t = position;
        let s = 1.0 - t;
        let mut out = [0.0; 2];
        for i in 0..2 {
            out[i] = 3.0 * s * s * (self.start_control[i] - self.start[i])
                + 6.0 * s * t * (self.finish_control[i] - self.start_control[i])
                + 3.0 * t * t * (self.finish[i] - self.finish_control[i]);
        }
        out
    }
}

/// A sampled point of the one-dimensional motion (vendor `OneDimensionalMotion::Point`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "uniffi", derive(uniffi::Record))]
pub struct MotionPoint {
    /// Seconds since the motion started.
    pub time: f64,
    /// Distance covered since the motion started, in meters.
    pub distance: f64,
    /// Instantaneous speed at this point, in m/s.
    pub speed: f64,
}

/// Continuous 1D motion with known initial/final speed, duration and distance.
/// Port of vendor `OneDimensionalMotion`: the time→distance mapping is a cubic
/// Bezier curve whose control points guarantee monotonicity in both axes.
#[derive(Debug, Clone)]
pub struct OneDimensionalMotion {
    points: Vec<MotionPoint>,
}

impl Default for OneDimensionalMotion {
    /// Vendor zero-distance motion.
    fn default() -> Self {
        Self {
            points: vec![MotionPoint {
                time: 0.0,
                distance: 0.0,
                speed: 0.0,
            }],
        }
    }
}

impl OneDimensionalMotion {
    /// Vendor constructor: Bezier control points at `duration * PATH_CURVATURE`,
    /// clamped so control-point distances never overshoot the total distance.
    pub fn new(initial_speed: f64, final_speed: f64, duration: f64, distance: f64) -> Self {
        debug_assert!(duration > 0.0);
        debug_assert!(distance >= 0.0);
        if !(duration > 0.0)
            || distance < 0.0
            || !initial_speed.is_finite()
            || !final_speed.is_finite()
        {
            return Self::default();
        }

        let mut control_point_time = duration * PATH_CURVATURE;
        if initial_speed * control_point_time > distance {
            control_point_time = distance / initial_speed;
        }
        if final_speed * control_point_time > distance {
            control_point_time = distance / final_speed;
        }

        let curve = CubicBezierCurve {
            start: [0.0, 0.0],
            start_control: [
                control_point_time,
                (control_point_time * initial_speed).min(distance),
            ],
            finish_control: [
                duration - control_point_time,
                distance - (control_point_time * final_speed).min(distance),
            ],
            finish: [duration, distance],
        };

        let mut points: Vec<MotionPoint> = Vec::with_capacity(PATH_POINT_NUMBER);
        for index in 0..PATH_POINT_NUMBER {
            let curve_position = (index as f64 / (PATH_POINT_NUMBER - 1) as f64).min(1.0);
            let curve_point = curve.point(curve_position);
            let curve_derivative = curve.derivative(curve_position);

            // Fight double imprecision exactly like the vendor: clamp each axis
            // to be non-decreasing against the previous sample.
            let (min_time, min_distance) = points
                .last()
                .map(|p: &MotionPoint| (p.time, p.distance))
                .unwrap_or((0.0, 0.0));
            let time = curve_point[0].clamp(min_time, duration);
            let distance = curve_point[1].clamp(min_distance, distance);
            let speed = if curve_derivative[1] != 0.0 && curve_derivative[0] > 0.0 {
                (curve_derivative[1] / curve_derivative[0]).abs()
            } else {
                0.0
            };
            points.push(MotionPoint {
                time,
                distance,
                speed,
            });
        }
        Self { points }
    }

    /// Vendor `point(time)`: linear interpolation between samples; past the end
    /// of the motion, extrapolate with the final sample's speed.
    pub fn point(&self, time: f64) -> MotionPoint {
        if !time.is_finite() || time < 0.0 {
            return self.points[0];
        }
        let next_index = self.points.iter().position(|p| p.time >= time);

        match next_index {
            Some(0) => self.points[0],
            None => {
                let mut last = *self.points.last().unwrap_or(&MotionPoint {
                    time: 0.0,
                    distance: 0.0,
                    speed: 0.0,
                });
                last.distance += (time - last.time) * last.speed;
                last.time = time;
                last
            }
            Some(next_index) => {
                let next = self.points[next_index];
                let previous = self.points[next_index - 1];
                let span = next.time - previous.time;
                let factor = if span > 0.0 {
                    (time - previous.time) / span
                } else {
                    0.0
                };
                MotionPoint {
                    time,
                    distance: previous.distance + (next.distance - previous.distance) * factor,
                    speed: previous.speed + (next.speed - previous.speed) * factor,
                }
            }
        }
    }
}

/// A streamed position on the route, produced at display frame rate.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "uniffi", derive(uniffi::Record))]
pub struct StreamedPosition {
    /// Interpolated coordinate on the route polyline.
    pub coordinates: GeographicCoordinate,
    /// Route bearing at the position.
    pub course_over_ground: Option<CourseOverGround>,
    /// Absolute route distance of the rendered position.
    pub distance_along_route_meters: f64,
    /// Instantaneous speed of the motion, in m/s.
    pub speed_mps: Option<f64>,
}

/// Internal bound-motion state (vendor `BoundMotion<RouteBoundLocation>`).
#[derive(Debug, Clone)]
struct BoundMotionState {
    /// Currently rendered route distance (vendor `location_.routePosition`).
    current_distance: f64,
    /// Timestamp of the rendered position (vendor `location_.timestamp`).
    current_timestamp: SystemTime,
    /// Current 1D motion toward the latest fix.
    motion: OneDimensionalMotion,
    /// Current point inside `motion`.
    point: MotionPoint,
    /// Vendor `isMotionAvailable_`.
    available: bool,
}

#[derive(Debug)]
struct StreamerInner {
    coords: Vec<GeographicCoordinate>,
    cum: Vec<f64>,
    motion: Option<BoundMotionState>,
    last_tick: Option<SystemTime>,
}

/// Streams a continuous position bound to the route polyline.
///
/// Feed every route-bound fix via [`Self::on_route_bound_location`] and drive
/// rendering with [`Self::advance_to`] at the display frame rate (vendor
/// `LOCATION_PUBLISHING_FRAME_RATE` = 20 Hz).
#[cfg_attr(feature = "uniffi", derive(uniffi::Object))]
pub struct RouteBoundStreamer {
    inner: Mutex<StreamerInner>,
}

#[cfg_attr(feature = "uniffi", uniffi::export)]
impl RouteBoundStreamer {
    /// Create a streamer for a route. Recreate on route change (vendor `setRoute`).
    #[cfg_attr(feature = "uniffi", uniffi::constructor)]
    pub fn new(route: &Route) -> Self {
        Self {
            inner: Mutex::new(StreamerInner {
                cum: cumulative_lengths(&route.geometry),
                coords: route.geometry.clone(),
                motion: None,
                last_tick: None,
            }),
        }
    }

    /// Supply a fresh route-bound location (vendor `BoundMotion::supplyLocation`
    /// with the constructor delay check folded in for the first fix).
    pub fn on_route_bound_location(
        &self,
        location: UserLocation,
        distance_along_route_meters: f64,
    ) {
        let mut inner = match self.inner.lock() {
            Ok(guard) => guard,
            Err(poisoned) => poisoned.into_inner(),
        };

        match inner.motion.take() {
            Some(mut state) if state.available => {
                supply_location(&mut state, location, distance_along_route_meters);
                inner.motion = Some(state);
            }
            None => {
                // Seed the first fix or recover after a rejected interpolation. The next
                // route-bound fix is a new trustworthy anchor; without reseeding, one GPS
                // gap would disable smoothing for the rest of the route.
                inner.motion = Some(initial_motion_state(location, distance_along_route_meters));
                inner.last_tick = None;
            }
            Some(state) if location.timestamp > state.current_timestamp => {
                inner.motion = Some(initial_motion_state(location, distance_along_route_meters));
                inner.last_tick = None;
            }
            Some(state) => {
                // Keep the newest rejected state's timestamp. Otherwise a second delayed
                // fix could reseed motion from an even older route position.
                inner.motion = Some(state);
            }
        }
    }

    /// Advance the rendered position to `now` (vendor `BoundMotion::advanceTo`
    /// plus the `MAX_REASONABLE_ADVANCEMENT` reset from `LocationStreamer::location`).
    pub fn advance_to(&self, now: SystemTime) -> Option<StreamedPosition> {
        let mut guard = match self.inner.lock() {
            Ok(guard) => guard,
            Err(poisoned) => poisoned.into_inner(),
        };
        let StreamerInner {
            coords,
            cum,
            motion,
            last_tick,
        } = &mut *guard;

        let advancement = last_tick
            .as_ref()
            .and_then(|last| now.duration_since(*last).ok())
            .unwrap_or(Duration::ZERO);
        if advancement > MAX_REASONABLE_ADVANCEMENT {
            *motion = None;
            *last_tick = None;
            return None;
        }
        *last_tick = Some(now);

        let state = motion.as_mut()?;
        if state.available {
            let previous_point = state.point;
            state.point = state
                .motion
                .point(state.point.time + advancement.as_secs_f64());
            let distance = (state.point.distance - previous_point.distance).max(0.0);
            state.current_distance += distance;
            state.current_timestamp += advancement;
        }

        let (coordinates, bearing) = point_at_distance(coords, cum, state.current_distance)?;
        Some(StreamedPosition {
            coordinates,
            course_over_ground: bearing.map(|b| CourseOverGround::new(b, None)),
            distance_along_route_meters: state.current_distance,
            speed_mps: state.available.then_some(state.point.speed),
        })
    }

    /// Non-mutating preview of the rendered position at `now`.
    ///
    /// UzMap display policy: the Android puck animates between location updates
    /// over a fixed window, so the consumer needs the motion's position at
    /// animation end WITHOUT advancing the streamer's tick clock. Call order per
    /// fix: [`Self::advance_to`] (catch the motion clock up), then
    /// [`Self::on_route_bound_location`], then `preview_at(now + window)`.
    pub fn preview_at(&self, now: SystemTime) -> Option<StreamedPosition> {
        let guard = match self.inner.lock() {
            Ok(guard) => guard,
            Err(poisoned) => poisoned.into_inner(),
        };
        let inner = &*guard;
        let state = inner.motion.as_ref()?;

        let advancement = inner
            .last_tick
            .as_ref()
            .and_then(|last| now.duration_since(*last).ok())
            .unwrap_or(Duration::ZERO);
        if advancement > MAX_REASONABLE_ADVANCEMENT {
            return None;
        }

        let (distance, speed) = if state.available {
            let point = state
                .motion
                .point(state.point.time + advancement.as_secs_f64());
            (
                state.current_distance + (point.distance - state.point.distance).max(0.0),
                Some(point.speed),
            )
        } else {
            (state.current_distance, None)
        };

        let (coordinates, bearing) = point_at_distance(&inner.coords, &inner.cum, distance)?;
        Some(StreamedPosition {
            coordinates,
            course_over_ground: bearing.map(|b| CourseOverGround::new(b, None)),
            distance_along_route_meters: distance,
            speed_mps: speed,
        })
    }

    /// Whether the motion is currently available (vendor `isMotionAvailable`).
    pub fn is_available(&self) -> bool {
        self.inner
            .lock()
            .map(|inner| inner.motion.as_ref().is_some_and(|m| m.available))
            .unwrap_or(false)
    }

    /// Vendor `reset`/`doResetMotion`.
    pub fn reset(&self) {
        if let Ok(mut inner) = self.inner.lock() {
            inner.motion = None;
            inner.last_tick = None;
        }
    }
}

fn initial_motion_state(location: UserLocation, route_distance: f64) -> BoundMotionState {
    BoundMotionState {
        current_distance: route_distance,
        current_timestamp: location.timestamp,
        motion: OneDimensionalMotion::default(),
        point: MotionPoint {
            time: 0.0,
            distance: 0.0,
            speed: location.speed.map(|s| s.value).unwrap_or(0.0),
        },
        available: true,
    }
}

/// Vendor `supplyLocation` on an existing motion.
fn supply_location(state: &mut BoundMotionState, location: UserLocation, route_distance: f64) {
    if location.timestamp <= state.current_timestamp {
        // Vendor: "Unexpected location order".
        state.available = false;
        return;
    }

    let path = route_distance - state.current_distance;
    let path = if path < 0.0 {
        if -path > JUMP_DISTANCE_THRESHOLD {
            state.available = false;
        } else {
            // Small backward jitter: vendor resets the 1D motion and skips this supply.
            state.motion = OneDimensionalMotion::default();
            state.point = state.motion.point(0.0);
        }
        return;
    } else {
        path
    };

    let interval = match location.timestamp.duration_since(state.current_timestamp) {
        Ok(interval) => interval,
        Err(_) => {
            state.available = false;
            return;
        }
    };
    if interval > MAXIMAL_MOTION_LOCATION_DELAY {
        state.available = false;
        return;
    }
    if path > MAXIMAL_POSSIBLE_SPEED * interval.as_secs_f64() {
        // Vendor: "Travelling distance is too large".
        state.available = false;
        return;
    }

    let new_speed = location.speed.map(|s| s.value).unwrap_or(0.0);
    state.motion =
        OneDimensionalMotion::new(state.point.speed, new_speed, interval.as_secs_f64(), path);
    state.point = state.motion.point(0.0);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_utils::make_user_location;
    use geo::coord;

    fn at(secs: u64) -> SystemTime {
        SystemTime::UNIX_EPOCH + Duration::from_secs(secs)
    }

    fn test_route() -> Route {
        // ~11.1 m per segment at the equator; total ~111 m.
        let geometry: Vec<GeographicCoordinate> = (0..=10)
            .map(|i| GeographicCoordinate {
                lat: 0.0,
                lng: i as f64 * 0.0001,
            })
            .collect();
        Route {
            bbox: crate::models::BoundingBox {
                sw: GeographicCoordinate { lat: 0.0, lng: 0.0 },
                ne: GeographicCoordinate {
                    lat: 0.0,
                    lng: 0.001,
                },
            },
            distance: 111.0,
            waypoints: vec![],
            steps: vec![],
            geometry,
        }
    }

    #[test]
    fn bezier_motion_is_monotonic_and_covers_distance() {
        let motion = OneDimensionalMotion::new(5.0, 8.0, 2.0, 14.0);
        let mut last = motion.point(0.0);
        for i in 1..=100 {
            let t = i as f64 * 0.02;
            let p = motion.point(t);
            assert!(p.time >= last.time);
            assert!(
                p.distance >= last.distance - 1e-9,
                "distance must be non-decreasing"
            );
            last = p;
        }
        let end = motion.point(2.0);
        assert!((end.distance - 14.0).abs() < 1e-6);
    }

    #[test]
    fn zero_distance_motion_is_safe() {
        let motion = OneDimensionalMotion::new(0.0, 0.0, 1.0, 0.0);
        assert_eq!(motion.point(0.5).distance, 0.0);
        assert_eq!(motion.point(2.0).distance, 0.0);
    }

    #[test]
    fn extrapolates_beyond_motion_end_with_last_speed() {
        let motion = OneDimensionalMotion::new(10.0, 10.0, 1.0, 10.0);
        let p = motion.point(2.0);
        assert!(p.distance > 10.0, "must extrapolate past the end");
        assert!((p.speed - 10.0).abs() < 1e-6);
    }

    #[test]
    fn streamer_advances_along_route_between_fixes() {
        let route = test_route();
        let total = cumulative_lengths(&route.geometry).last().copied().unwrap();
        let streamer = RouteBoundStreamer::new(&route);

        let loc0 = make_user_location(coord!(x: 0.0, y: 0.0), 5.0);
        let loc0 = UserLocation {
            timestamp: at(0),
            ..loc0
        };
        streamer.on_route_bound_location(loc0, 0.0);

        let loc1 = UserLocation {
            timestamp: at(1),
            ..make_user_location(coord!(x: 0.001, y: 0.0), 5.0)
        };
        let seg_len = total / 10.0;
        streamer.on_route_bound_location(loc1, seg_len);

        // Half a second after the second fix the rendered position must be
        // between the two fixes.
        let rendered = streamer.advance_to(at(1)).unwrap();
        assert!(rendered.distance_along_route_meters >= 0.0);
        let halfway = streamer
            .advance_to(at(1) + Duration::from_millis(500))
            .unwrap();
        assert!(halfway.distance_along_route_meters > rendered.distance_along_route_meters);
        assert!(halfway.distance_along_route_meters <= seg_len + 1.0);
    }

    #[test]
    fn streamer_rejects_late_fix() {
        let route = test_route();
        let streamer = RouteBoundStreamer::new(&route);
        streamer.on_route_bound_location(
            UserLocation {
                timestamp: at(0),
                ..make_user_location(coord!(x: 0.0, y: 0.0), 5.0)
            },
            0.0,
        );
        // Fix arrives 3 s after the rendered position (> MAXIMAL_MOTION_LOCATION_DELAY).
        streamer.on_route_bound_location(
            UserLocation {
                timestamp: at(3),
                ..make_user_location(coord!(x: 0.001, y: 0.0), 5.0)
            },
            100.0,
        );
        assert!(!streamer.is_available());

        // The rejected interpolation must not poison the route lifetime. A
        // subsequent route-bound fix becomes a fresh anchor.
        streamer.on_route_bound_location(
            UserLocation {
                timestamp: at(4),
                ..make_user_location(coord!(x: 0.0011, y: 0.0), 5.0)
            },
            110.0,
        );
        assert!(streamer.is_available());
        let recovered = streamer
            .advance_to(at(4))
            .expect("fresh fix must recover motion");
        assert!((recovered.distance_along_route_meters - 110.0).abs() < 1e-6);
    }

    #[test]
    fn streamer_does_not_recover_from_a_second_out_of_order_fix() {
        let route = test_route();
        let streamer = RouteBoundStreamer::new(&route);
        streamer.on_route_bound_location(
            UserLocation {
                timestamp: at(10),
                ..make_user_location(coord!(x: 0.001, y: 0.0), 5.0)
            },
            100.0,
        );
        streamer.on_route_bound_location(
            UserLocation {
                timestamp: at(8),
                ..make_user_location(coord!(x: 0.0008, y: 0.0), 5.0)
            },
            80.0,
        );
        assert!(!streamer.is_available());

        streamer.on_route_bound_location(
            UserLocation {
                timestamp: at(9),
                ..make_user_location(coord!(x: 0.0009, y: 0.0), 5.0)
            },
            90.0,
        );

        assert!(!streamer.is_available());
        let position = streamer
            .advance_to(at(10))
            .expect("the last reliable anchor remains renderable");
        assert!((position.distance_along_route_meters - 100.0).abs() < 1e-6);
    }

    #[test]
    fn streamer_rejects_implausibly_fast_fix() {
        let route = test_route();
        let streamer = RouteBoundStreamer::new(&route);
        streamer.on_route_bound_location(
            UserLocation {
                timestamp: at(0),
                ..make_user_location(coord!(x: 0.0, y: 0.0), 5.0)
            },
            0.0,
        );
        // 1000 m in 1 s exceeds MAXIMAL_POSSIBLE_SPEED.
        streamer.on_route_bound_location(
            UserLocation {
                timestamp: at(1),
                ..make_user_location(coord!(x: 0.005, y: 0.0), 5.0)
            },
            1000.0,
        );
        assert!(!streamer.is_available());
    }

    #[test]
    fn streamer_resets_after_long_tick_gap() {
        let route = test_route();
        let streamer = RouteBoundStreamer::new(&route);
        streamer.on_route_bound_location(
            UserLocation {
                timestamp: at(0),
                ..make_user_location(coord!(x: 0.0, y: 0.0), 5.0)
            },
            0.0,
        );
        assert!(streamer.advance_to(at(1)).is_some());
        // 11 s without a tick (> MAX_REASONABLE_ADVANCEMENT) resets the motion.
        assert!(streamer.advance_to(at(12)).is_none());
        assert!(!streamer.is_available());
    }

    #[test]
    fn preview_does_not_mutate_tick_state() {
        let route = test_route();
        let total = cumulative_lengths(&route.geometry).last().copied().unwrap();
        let seg_len = total / 10.0;
        let streamer = RouteBoundStreamer::new(&route);

        streamer.on_route_bound_location(
            UserLocation {
                timestamp: at(0),
                ..make_user_location(coord!(x: 0.0, y: 0.0), 5.0)
            },
            0.0,
        );
        streamer.on_route_bound_location(
            UserLocation {
                timestamp: at(1),
                ..make_user_location(coord!(x: 0.0001, y: 0.0), 5.0)
            },
            seg_len,
        );
        // Establish the tick clock.
        let _ = streamer.advance_to(at(1)).unwrap();

        // Preview one second ahead: the motion's end (the second fix distance).
        let preview = streamer.preview_at(at(2)).unwrap();
        assert!(
            (preview.distance_along_route_meters - seg_len).abs() < 1.0,
            "preview must land at the motion end, got {}",
            preview.distance_along_route_meters
        );

        // The preview must not have advanced the rendered position: a real tick
        // at the same instant still animates from the motion start.
        let rendered = streamer
            .advance_to(at(1) + Duration::from_millis(500))
            .unwrap();
        assert!(
            rendered.distance_along_route_meters < seg_len,
            "tick after preview must still be mid-motion, got {}",
            rendered.distance_along_route_meters
        );
    }

    #[test]
    fn backward_jump_over_threshold_kills_motion() {
        let route = test_route();
        let total = cumulative_lengths(&route.geometry).last().copied().unwrap();
        let streamer = RouteBoundStreamer::new(&route);
        streamer.on_route_bound_location(
            UserLocation {
                timestamp: at(0),
                ..make_user_location(coord!(x: 0.005, y: 0.0), 5.0)
            },
            total / 2.0,
        );
        streamer.on_route_bound_location(
            UserLocation {
                timestamp: at(1),
                ..make_user_location(coord!(x: 0.0, y: 0.0), 5.0)
            },
            total / 2.0 - 50.0,
        );
        assert!(!streamer.is_available());
    }
}
