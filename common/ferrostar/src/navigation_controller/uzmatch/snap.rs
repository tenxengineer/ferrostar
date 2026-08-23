//! Heading-aware snap of a raw location to the route polyline.
//!
//! Provenance: emission sigmas from maps-product
//! `backend/analyzer/libs/guidance/include/config.h`
//! (`GPS_POSITION_ERROR_STDDEV = 8.0`, `GPS_HEADING_ERROR_STDDEV = 6.0`),
//! applied to the route polyline (P1a is polyline-based; graph binding is P1b).
//! Heading is ignored below `snap_heading_min_speed_mps` (vendor
//! `IGNORE_HEADING_WHEN_SLOWER` mechanism; UzNav default 4.0 m/s).

use geo::{Bearing, Coord, Distance, Geodesic, Haversine, Point};
use serde::{Deserialize, Serialize};

use crate::models::{CourseOverGround, GeographicCoordinate, UserLocation};

use super::UzmatchConfig;

/// The user's position on the route polyline, route-global.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "uniffi", derive(uniffi::Record))]
pub struct RoutePosition {
    /// Index of the route geometry segment (segment `i` runs from vertex `i` to `i + 1`).
    pub segment_index: u64,
    /// Distance in meters from the segment's first vertex to the projected point.
    pub segment_offset_meters: f64,
    /// Absolute distance in meters from the start of the route.
    pub distance_along_route_meters: f64,
    /// The snapped coordinate on the route polyline.
    pub coordinates: GeographicCoordinate,
    /// The route's bearing at the snapped position.
    pub course_over_ground: Option<CourseOverGround>,
}

/// Cumulative route length in meters at each vertex; `result[0] == 0`.
pub(crate) fn cumulative_lengths(coords: &[GeographicCoordinate]) -> Vec<f64> {
    let mut cum = Vec::with_capacity(coords.len());
    cum.push(0.0);
    for pair in coords.windows(2) {
        let segment = Haversine.distance(
            Point::from(Coord::from(pair[0])),
            Point::from(Coord::from(pair[1])),
        );
        cum.push(cum.last().copied().unwrap_or(0.0) + segment);
    }
    cum
}

/// Smallest absolute angular difference between two headings, in degrees.
pub(crate) fn heading_difference(a: f64, b: f64) -> f64 {
    let diff = (a - b).rem_euclid(360.0);
    if diff > 180.0 { 360.0 - diff } else { diff }
}

struct SegmentCandidate {
    score: f64,
    segment_index: usize,
    offset_meters: f64,
    distance_along_route: f64,
    snapped: GeographicCoordinate,
    bearing: f64,
}

/// Snap `location` to the route polyline, scoring each segment by a Gaussian
/// geometric emission (sigma = `snap_position_stddev_m`) multiplied by a
/// Gaussian heading emission (sigma = `snap_heading_stddev_deg`) when the
/// user's speed is at least `snap_heading_min_speed_mps` and a course is
/// available.
///
/// Returns `None` when the route has fewer than two vertices.
pub(crate) fn snap_to_route(
    location: &UserLocation,
    coords: &[GeographicCoordinate],
    cum: &[f64],
    config: &UzmatchConfig,
) -> Option<RoutePosition> {
    if coords.len() < 2 {
        return None;
    }

    let user_point = Point::from(Coord::from(location.coordinates));
    let user_course = location.course_over_ground.map(|c| c.degrees as f64);
    let use_heading = location
        .speed
        .map(|s| s.value >= config.snap_heading_min_speed_mps)
        .unwrap_or(false)
        && user_course.is_some();

    let mut best: Option<SegmentCandidate> = None;

    for (index, pair) in coords.windows(2).enumerate() {
        let a = Coord::from(pair[0]);
        let b = Coord::from(pair[1]);
        let ab_x = b.x - a.x;
        let ab_y = b.y - a.y;
        let len_sq = ab_x * ab_x + ab_y * ab_y;

        // Planar projection fraction along the segment (degrees space; the
        // metric correction happens via haversine below).
        let t = if len_sq > f64::EPSILON {
            let ap_x = user_point.x() - a.x;
            let ap_y = user_point.y() - a.y;
            ((ap_x * ab_x + ap_y * ab_y) / len_sq).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let proj = Coord {
            x: a.x + t * ab_x,
            y: a.y + t * ab_y,
        };
        let proj_point = Point::from(proj);
        let distance_m = Haversine.distance(user_point, proj_point);

        let mut score = -0.5 * (distance_m / config.snap_position_stddev_m).powi(2);

        let bearing = Geodesic.bearing(Point::from(a), Point::from(b)).rem_euclid(360.0);
        if use_heading {
            let delta = heading_difference(user_course.unwrap_or(0.0), bearing);
            score += -0.5 * (delta / config.snap_heading_stddev_deg).powi(2);
        }

        let segment_length = Haversine.distance(Point::from(a), Point::from(b));
        let offset_meters = t * segment_length;
        let candidate = SegmentCandidate {
            score,
            segment_index: index,
            offset_meters,
            distance_along_route: cum.get(index).copied().unwrap_or(0.0) + offset_meters,
            snapped: GeographicCoordinate {
                lat: proj.y,
                lng: proj.x,
            },
            bearing,
        };

        if best.as_ref().is_none_or(|b| candidate.score > b.score) {
            best = Some(candidate);
        }
    }

    best.map(|c| RoutePosition {
        segment_index: c.segment_index as u64,
        segment_offset_meters: c.offset_meters,
        distance_along_route_meters: c.distance_along_route,
        coordinates: c.snapped,
        course_over_ground: Some(CourseOverGround::new(c.bearing, None)),
    })
}

/// Interpolate the coordinate and bearing at an absolute route distance.
/// Used by the streamer to render positions between location updates.
pub(crate) fn point_at_distance(
    coords: &[GeographicCoordinate],
    cum: &[f64],
    distance: f64,
) -> Option<(GeographicCoordinate, Option<f64>)> {
    if coords.len() < 2 {
        return None;
    }
    let clamped = distance.clamp(0.0, cum.last().copied().unwrap_or(0.0));
    // Binary search for the segment containing `clamped`.
    let index = match cum.binary_search_by(|v| v.partial_cmp(&clamped).unwrap_or(std::cmp::Ordering::Equal)) {
        Ok(i) => i.min(coords.len() - 2),
        Err(i) => i.saturating_sub(1).min(coords.len() - 2),
    };
    let a = Coord::from(coords[index]);
    let b = Coord::from(coords[index + 1]);
    let seg_start = cum[index];
    let seg_len = (cum[index + 1] - seg_start).max(f64::EPSILON);
    let t = ((clamped - seg_start) / seg_len).clamp(0.0, 1.0);
    let point = GeographicCoordinate {
        lat: a.y + t * (b.y - a.y),
        lng: a.x + t * (b.x - a.x),
    };
    let bearing = Geodesic.bearing(Point::from(a), Point::from(b)).rem_euclid(360.0);
    Some((point, Some(bearing)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::Speed;
    use crate::test_utils::make_user_location;
    use geo::coord;

    fn test_config() -> UzmatchConfig {
        UzmatchConfig::default()
    }

    fn route_coords() -> Vec<GeographicCoordinate> {
        // Straight east-west route along lat 0.
        vec![
            GeographicCoordinate { lat: 0.0, lng: 0.0 },
            GeographicCoordinate { lat: 0.0, lng: 0.01 },
        ]
    }

    #[test]
    fn snaps_to_closest_point_without_heading() {
        let coords = route_coords();
        let cum = cumulative_lengths(&coords);
        let loc = UserLocation {
            coordinates: GeographicCoordinate {
                lat: 0.0002,
                lng: 0.005,
            },
            speed: None,
            course_over_ground: None,
            ..make_user_location(coord!(x: 0.0, y: 0.0), 5.0)
        };
        let position = snap_to_route(&loc, &coords, &cum, &test_config()).unwrap();
        assert_eq!(position.segment_index, 0);
        assert!((position.coordinates.lat).abs() < 1e-9);
        assert!((position.coordinates.lng - 0.005).abs() < 1e-9);
        // ~0.005 deg lng at equator ~= 555 m; offset should be roughly half the route.
        let total = *cum.last().unwrap();
        assert!((position.distance_along_route_meters - total / 2.0).abs() < total * 0.02);
    }

    #[test]
    fn heading_disambiguates_parallel_carriageway() {
        // Route that goes east along lat=0 and returns west along lat=0.0001
        // (~11 m apart): a self-overlapping parallel carriageway.
        let coords = vec![
            GeographicCoordinate { lat: 0.0, lng: 0.0 },
            GeographicCoordinate { lat: 0.0, lng: 0.01 },
            GeographicCoordinate {
                lat: 0.0001,
                lng: 0.01,
            },
            GeographicCoordinate {
                lat: 0.0001,
                lng: 0.0,
            },
        ];
        let cum = cumulative_lengths(&coords);
        let config = test_config();

        // User sits exactly between the carriageways, moving EAST (90 deg).
        // Geometry alone cannot pick; heading must select segment 0.
        let loc = UserLocation {
            coordinates: GeographicCoordinate {
                lat: 0.00005,
                lng: 0.005,
            },
            speed: Some(Speed {
                value: 10.0,
                accuracy: None,
            }),
            course_over_ground: Some(CourseOverGround::new(90.0, None)),
            ..make_user_location(coord!(x: 0.0, y: 0.0), 5.0)
        };
        let position = snap_to_route(&loc, &coords, &cum, &config).unwrap();
        assert_eq!(position.segment_index, 0, "eastbound heading must pick the eastbound carriageway");

        // Same geometry, moving WEST (270 deg) -> segment 2.
        let loc_west = UserLocation {
            course_over_ground: Some(CourseOverGround::new(270.0, None)),
            ..loc
        };
        let position = snap_to_route(&loc_west, &coords, &cum, &config).unwrap();
        assert_eq!(position.segment_index, 2, "westbound heading must pick the westbound carriageway");
    }

    #[test]
    fn heading_ignored_below_min_speed() {
        let coords = vec![
            GeographicCoordinate { lat: 0.0, lng: 0.0 },
            GeographicCoordinate { lat: 0.0, lng: 0.01 },
            GeographicCoordinate {
                lat: 0.0001,
                lng: 0.01,
            },
            GeographicCoordinate {
                lat: 0.0001,
                lng: 0.0,
            },
        ];
        let cum = cumulative_lengths(&coords);
        let config = test_config();
        // Slow user (2 m/s < 4.0) slightly closer to the westbound carriageway
        // but heading east: heading must be ignored, geometry wins.
        let loc = UserLocation {
            coordinates: GeographicCoordinate {
                lat: 0.00008,
                lng: 0.005,
            },
            speed: Some(Speed {
                value: 2.0,
                accuracy: None,
            }),
            course_over_ground: Some(CourseOverGround::new(90.0, None)),
            ..make_user_location(coord!(x: 0.0, y: 0.0), 5.0)
        };
        let position = snap_to_route(&loc, &coords, &cum, &config).unwrap();
        assert_eq!(position.segment_index, 2);
    }

    #[test]
    fn heading_difference_wraps() {
        assert_eq!(heading_difference(350.0, 10.0), 20.0);
        assert_eq!(heading_difference(10.0, 350.0), 20.0);
        assert_eq!(heading_difference(90.0, 270.0), 180.0);
    }

    #[test]
    fn point_at_distance_interpolates() {
        let coords = route_coords();
        let cum = cumulative_lengths(&coords);
        let total = *cum.last().unwrap();
        let (point, bearing) = point_at_distance(&coords, &cum, total / 2.0).unwrap();
        assert!((point.lng - 0.005).abs() < 1e-6);
        assert!((bearing.unwrap() - 90.0).abs() < 0.5);
        // Clamps past the end.
        let (end, _) = point_at_distance(&coords, &cum, total + 100.0).unwrap();
        assert!((end.lng - 0.01).abs() < 1e-9);
    }
}
