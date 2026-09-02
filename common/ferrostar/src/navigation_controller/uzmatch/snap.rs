//! Heading-aware snap of a raw location to the route polyline.
//!
//! Provenance: emission sigmas from maps-product
//! `backend/analyzer/libs/guidance/include/config.h`
//! (`GPS_POSITION_ERROR_STDDEV = 8.0`, `GPS_HEADING_ERROR_STDDEV = 6.0`),
//! applied to the route polyline (P1a is polyline-based; graph binding is P1b).
//! Heading is ignored below `snap_heading_min_speed_mps` (vendor
//! `IGNORE_HEADING_WHEN_SLOWER` mechanism; `UzNav` default 4.0 m/s).

use core::cmp::Ordering;
use geo::{Bearing, Coord, Distance, Geodesic, Haversine, Point};
use rstar::{AABB, RTree, RTreeObject};
use serde::{Deserialize, Serialize};

use crate::models::{CourseOverGround, GeographicCoordinate, UserLocation};

use super::UzmatchConfig;

/// Vendor `MAX_ROUTE_LOCATION_BIAS`: segments outside this distance cannot
/// become route-binding candidates.
const MAX_ROUTE_LOCATION_BIAS_METERS: f64 = 200.0;

/// Offsets within this many metres of a segment end count as pinned at the vertex.
const VERTEX_PIN_EPSILON_METERS: f64 = 0.01;
/// Conservative lower bound for meters per degree used only to build a query
/// envelope. Exact candidate distances are still measured with Haversine.
const METERS_PER_DEGREE: f64 = 110_000.0;

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

/// Maximum number of candidates retained in one temporal frontier.
///
/// With no usable previous position, the snap index returns at most this many
/// frame-local emission candidates. With a usable previous position, it keeps
/// this many route candidates on each side before temporal scoring.
pub(super) const MAX_SNAP_CANDIDATES: usize = 10;

/// One route projection and its frame-local emission log likelihood.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct SnapCandidate {
    pub route_position: RoutePosition,
    pub emission_log_likelihood: f64,
}

fn compare_candidates(left: &SnapCandidate, right: &SnapCandidate) -> Ordering {
    right
        .emission_log_likelihood
        .total_cmp(&left.emission_log_likelihood)
        .then_with(|| {
            left.route_position
                .segment_index
                .cmp(&right.route_position.segment_index)
        })
}

#[derive(Debug, Clone, Copy)]
struct IndexedSegment {
    start: Coord,
    delta_x: f64,
    delta_y: f64,
    length_squared: f64,
    length_meters: f64,
    distance_from_route_start: f64,
    bearing: f64,
}

#[derive(Debug, Clone, Copy)]
struct SegmentEnvelope {
    index: usize,
    envelope: AABB<[f64; 2]>,
}

impl RTreeObject for SegmentEnvelope {
    type Envelope = AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        self.envelope
    }
}

/// Route-lifetime spatial index for heading-aware snapping.
///
/// `NaviKit`'s `IndexedRoute` owns the equivalent route index. Keeping it on the
/// controller avoids rebuilding cumulative lengths, segment bearings and the
/// spatial search tree for every GPS fix.
pub(crate) struct RouteSnapIndex {
    segments: Vec<IndexedSegment>,
    tree: RTree<SegmentEnvelope>,
}

impl RouteSnapIndex {
    pub(crate) fn new(coords: &[GeographicCoordinate]) -> Self {
        let mut distance_from_route_start = 0.0;
        let mut segments = Vec::with_capacity(coords.len().saturating_sub(1));
        let mut envelopes = Vec::with_capacity(coords.len().saturating_sub(1));

        for (index, pair) in coords.windows(2).enumerate() {
            let start = Coord::from(pair[0]);
            let end = Coord::from(pair[1]);
            let delta_x = end.x - start.x;
            let delta_y = end.y - start.y;
            let length_meters = Haversine.distance(Point::from(start), Point::from(end));
            let bearing = Geodesic
                .bearing(Point::from(start), Point::from(end))
                .rem_euclid(360.0);

            segments.push(IndexedSegment {
                start,
                delta_x,
                delta_y,
                length_squared: delta_x * delta_x + delta_y * delta_y,
                length_meters,
                distance_from_route_start,
                bearing,
            });
            envelopes.push(SegmentEnvelope {
                index,
                envelope: AABB::from_corners(
                    [start.x.min(end.x), start.y.min(end.y)],
                    [start.x.max(end.x), start.y.max(end.y)],
                ),
            });
            distance_from_route_start += length_meters;
        }

        Self {
            segments,
            tree: RTree::bulk_load(envelopes),
        }
    }

    #[cfg(test)]
    pub(crate) fn snap_to_route(
        &self,
        location: &UserLocation,
        config: &UzmatchConfig,
    ) -> Option<RoutePosition> {
        self.candidates(location, config)
            .first()
            .map(|candidate| candidate.route_position)
    }

    /// Return the strongest frame-local route projections.
    #[cfg(test)]
    pub(crate) fn candidates(
        &self,
        location: &UserLocation,
        config: &UzmatchConfig,
    ) -> Vec<SnapCandidate> {
        self.candidates_with_anchor(location, config, None)
    }

    /// Return bounded local route projections relative to a previous match.
    pub(crate) fn candidates_with_anchor(
        &self,
        location: &UserLocation,
        config: &UzmatchConfig,
        previous_route_position: Option<RoutePosition>,
    ) -> Vec<SnapCandidate> {
        if self.segments.is_empty()
            || !location.coordinates.lat.is_finite()
            || !location.coordinates.lng.is_finite()
        {
            return Vec::new();
        }

        let user_point = Point::from(Coord::from(location.coordinates));
        let user_course = location.course_over_ground.map(|c| f64::from(c.degrees));
        let use_heading = location
            .speed
            .is_some_and(|s| s.value >= config.snap_heading_min_speed_mps)
            && user_course.is_some();

        let mut candidate_indices = self.candidate_indices(location.coordinates);
        // R-tree iteration order is intentionally unspecified. Route order makes
        // equal-score selection deterministic and preserves the old first-win rule.
        candidate_indices.sort_unstable();

        let mut frame_candidates = Vec::with_capacity(candidate_indices.len().min(
            if previous_route_position.is_some() {
                MAX_SNAP_CANDIDATES * 2
            } else {
                MAX_SNAP_CANDIDATES
            },
        ));
        let mut route_candidates_ahead = Vec::with_capacity(MAX_SNAP_CANDIDATES);
        let mut route_candidates_behind = Vec::with_capacity(MAX_SNAP_CANDIDATES);
        for index in candidate_indices {
            let segment = self.segments[index];
            let t = if segment.length_squared > f64::EPSILON {
                let ap_x = user_point.x() - segment.start.x;
                let ap_y = user_point.y() - segment.start.y;
                ((ap_x * segment.delta_x + ap_y * segment.delta_y) / segment.length_squared)
                    .clamp(0.0, 1.0)
            } else {
                0.0
            };
            let projection = Coord {
                x: segment.start.x + t * segment.delta_x,
                y: segment.start.y + t * segment.delta_y,
            };
            let distance_meters = Haversine.distance(user_point, Point::from(projection));
            if !distance_meters.is_finite() || distance_meters > MAX_ROUTE_LOCATION_BIAS_METERS {
                continue;
            }

            let mut score = -0.5 * (distance_meters / config.snap_position_stddev_m).powi(2);
            if use_heading {
                let delta = heading_difference(user_course.unwrap_or(0.0), segment.bearing);
                score += -0.5 * (delta / config.snap_heading_stddev_deg).powi(2);
            }
            if !score.is_finite() {
                continue;
            }

            let offset_meters = t * segment.length_meters;
            let candidate = SnapCandidate {
                emission_log_likelihood: score,
                route_position: RoutePosition {
                    segment_index: index as u64,
                    segment_offset_meters: offset_meters,
                    distance_along_route_meters: segment.distance_from_route_start + offset_meters,
                    coordinates: GeographicCoordinate {
                        lat: projection.y,
                        lng: projection.x,
                    },
                    course_over_ground: Some(CourseOverGround::new(segment.bearing, None)),
                },
            };

            if let Some(previous) = previous_route_position {
                if compare_route_positions(&candidate.route_position, &previous) == Ordering::Less {
                    insert_bounded(
                        &mut route_candidates_behind,
                        candidate,
                        compare_route_candidates_behind,
                    );
                } else {
                    insert_bounded(
                        &mut route_candidates_ahead,
                        candidate,
                        compare_route_candidates_ahead,
                    );
                }
            } else {
                insert_bounded(&mut frame_candidates, candidate, compare_candidates);
            }
        }

        if previous_route_position.is_some() {
            frame_candidates.extend(route_candidates_ahead);
            frame_candidates.extend(route_candidates_behind);
        }
        frame_candidates
    }

    /// The direction the route continues in from a matched position. Inside a
    /// segment that is the segment's own bearing; at a segment's end vertex it
    /// is the bearing of the next non-degenerate segment, so a position pinned
    /// at a maneuver reports the post-maneuver direction. `None` past the end
    /// of the route.
    pub(crate) fn bearing_ahead(&self, position: &RoutePosition) -> Option<f64> {
        let index = usize::try_from(position.segment_index).ok()?;
        let segment = self.segments.get(index)?;
        if position.segment_offset_meters < segment.length_meters - VERTEX_PIN_EPSILON_METERS {
            return Some(segment.bearing);
        }
        self.segments
            .iter()
            .skip(index + 1)
            .find(|next| next.length_meters > VERTEX_PIN_EPSILON_METERS)
            .map(|next| next.bearing)
    }

    fn candidate_indices(&self, point: GeographicCoordinate) -> Vec<usize> {
        let latitude_delta = MAX_ROUTE_LOCATION_BIAS_METERS / METERS_PER_DEGREE;
        let longitude_scale = point.lat.to_radians().cos().abs();
        let longitude_delta = if longitude_scale <= f64::EPSILON {
            180.0
        } else {
            (MAX_ROUTE_LOCATION_BIAS_METERS / (METERS_PER_DEGREE * longitude_scale)).min(180.0)
        };

        // A Cartesian longitude envelope cannot wrap around the antimeridian.
        // Falling back to all segments there is rare and preserves correctness.
        if point.lng - longitude_delta < -180.0 || point.lng + longitude_delta > 180.0 {
            return (0..self.segments.len()).collect();
        }

        let envelope = AABB::from_corners(
            [point.lng - longitude_delta, point.lat - latitude_delta],
            [point.lng + longitude_delta, point.lat + latitude_delta],
        );
        self.tree
            .locate_in_envelope_intersecting(&envelope)
            .map(|segment| segment.index)
            .collect()
    }
}

fn compare_route_candidates_ahead(left: &SnapCandidate, right: &SnapCandidate) -> Ordering {
    compare_route_positions(&left.route_position, &right.route_position)
}

fn compare_route_candidates_behind(left: &SnapCandidate, right: &SnapCandidate) -> Ordering {
    compare_route_positions(&right.route_position, &left.route_position)
}

fn compare_route_positions(left: &RoutePosition, right: &RoutePosition) -> Ordering {
    left.segment_index.cmp(&right.segment_index).then_with(|| {
        left.segment_offset_meters
            .total_cmp(&right.segment_offset_meters)
    })
}

fn insert_bounded(
    candidates: &mut Vec<SnapCandidate>,
    candidate: SnapCandidate,
    compare: fn(&SnapCandidate, &SnapCandidate) -> Ordering,
) {
    let insertion_index = candidates
        .binary_search_by(|existing| compare(existing, &candidate))
        .unwrap_or_else(|index| index);
    if insertion_index < MAX_SNAP_CANDIDATES {
        candidates.insert(insertion_index, candidate);
        candidates.truncate(MAX_SNAP_CANDIDATES);
    }
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
    let index = match cum.binary_search_by(|v| v.partial_cmp(&clamped).unwrap_or(Ordering::Equal)) {
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
    let bearing = Geodesic
        .bearing(Point::from(a), Point::from(b))
        .rem_euclid(360.0);
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

    fn snap_to_route(
        location: &UserLocation,
        coords: &[GeographicCoordinate],
        _cumulative_lengths: &[f64],
        config: &UzmatchConfig,
    ) -> Option<RoutePosition> {
        RouteSnapIndex::new(coords).snap_to_route(location, config)
    }

    fn route_coords() -> Vec<GeographicCoordinate> {
        // Straight east-west route along lat 0.
        vec![
            GeographicCoordinate { lat: 0.0, lng: 0.0 },
            GeographicCoordinate {
                lat: 0.0,
                lng: 0.01,
            },
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
            GeographicCoordinate {
                lat: 0.0,
                lng: 0.01,
            },
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
        assert_eq!(
            position.segment_index, 0,
            "eastbound heading must pick the eastbound carriageway"
        );

        // Same geometry, moving WEST (270 deg) -> segment 2.
        let loc_west = UserLocation {
            course_over_ground: Some(CourseOverGround::new(270.0, None)),
            ..loc
        };
        let position = snap_to_route(&loc_west, &coords, &cum, &config).unwrap();
        assert_eq!(
            position.segment_index, 2,
            "westbound heading must pick the westbound carriageway"
        );
    }

    #[test]
    fn heading_ignored_below_min_speed() {
        let coords = vec![
            GeographicCoordinate { lat: 0.0, lng: 0.0 },
            GeographicCoordinate {
                lat: 0.0,
                lng: 0.01,
            },
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
    fn route_index_rejects_locations_outside_vendor_bias() {
        let coords = route_coords();
        let location = UserLocation {
            coordinates: GeographicCoordinate {
                lat: 0.01,
                lng: 0.005,
            },
            ..make_user_location(coord!(x: 0.0, y: 0.0), 5.0)
        };

        assert!(
            RouteSnapIndex::new(&coords)
                .snap_to_route(&location, &test_config())
                .is_none()
        );
    }

    #[test]
    fn invalid_emission_scale_fails_closed() {
        let coords = route_coords();
        let location = UserLocation {
            coordinates: GeographicCoordinate {
                lat: 0.0,
                lng: 0.005,
            },
            ..make_user_location(coord!(x: 0.0, y: 0.0), 5.0)
        };
        let config = UzmatchConfig {
            snap_position_stddev_m: 0.0,
            ..test_config()
        };

        assert!(
            RouteSnapIndex::new(&coords)
                .candidates(&location, &config)
                .is_empty()
        );
    }

    #[test]
    fn route_index_prunes_a_long_route_to_local_segments() {
        let coords: Vec<_> = (0..=10_000)
            .map(|index| GeographicCoordinate {
                lat: 0.0,
                lng: index as f64 * 0.0001,
            })
            .collect();
        let route_index = RouteSnapIndex::new(&coords);
        let location = GeographicCoordinate { lat: 0.0, lng: 0.5 };
        let candidates = route_index.candidate_indices(location);

        assert!(candidates.contains(&4_999));
        assert!(candidates.contains(&5_000));
        assert!(
            candidates.len() < 100,
            "expected a local candidate set, got {} of {} segments",
            candidates.len(),
            route_index.segments.len()
        );

        let location = UserLocation {
            coordinates: location,
            ..make_user_location(coord!(x: 0.0, y: 0.0), 5.0)
        };
        assert!(route_index.candidates(&location, &test_config()).len() <= MAX_SNAP_CANDIDATES);
    }

    #[test]
    fn anchored_candidate_sides_use_route_position_at_duplicate_vertices() {
        let point = GeographicCoordinate { lat: 0.0, lng: 0.0 };
        let coords = vec![point; 26];
        let route_index = RouteSnapIndex::new(&coords);
        let location = UserLocation {
            coordinates: point,
            ..make_user_location(coord!(x: 0.0, y: 0.0), 5.0)
        };
        let anchor = RoutePosition {
            segment_index: 15,
            segment_offset_meters: 0.0,
            distance_along_route_meters: 0.0,
            coordinates: point,
            course_over_ground: None,
        };

        let candidates =
            route_index.candidates_with_anchor(&location, &test_config(), Some(anchor));
        let segment_indices = candidates
            .iter()
            .map(|candidate| candidate.route_position.segment_index)
            .collect::<Vec<_>>();

        assert_eq!(segment_indices.len(), MAX_SNAP_CANDIDATES * 2);
        assert_eq!(&segment_indices[..10], &(15_u64..25).collect::<Vec<_>>());
        assert_eq!(
            &segment_indices[10..],
            &(5_u64..15).rev().collect::<Vec<_>>()
        );
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
    /// Design D2: past the end of a step the route direction is the NEXT
    /// segment's bearing. A walker straight past a left turn projects onto
    /// the pre-turn segment's end vertex, whose own bearing equals their
    /// heading — using it would never show a divergence.
    #[test]
    fn heading_departure_past_step_end_uses_next_step_bearing() {
        let coords = vec![
            GeographicCoordinate { lat: 0.0, lng: 0.0 },
            GeographicCoordinate {
                lat: 0.0,
                lng: 0.001,
            },
            GeographicCoordinate {
                lat: 0.0,
                lng: 0.001,
            },
            GeographicCoordinate {
                lat: 0.001,
                lng: 0.001,
            },
        ];
        let index = RouteSnapIndex::new(&coords);
        let east_length = index.segments[0].length_meters;
        let position = |segment_index: u64, offset: f64| RoutePosition {
            segment_index,
            segment_offset_meters: offset,
            distance_along_route_meters: 0.0,
            coordinates: GeographicCoordinate { lat: 0.0, lng: 0.0 },
            course_over_ground: None,
        };

        let interior = position(0, east_length / 2.0);
        assert!((index.bearing_ahead(&interior).unwrap() - 90.0).abs() < 0.5);

        // Pinned at the end of the east segment: skip the zero-length segment,
        // report the north leg.
        let at_end = position(0, east_length);
        assert!(index.bearing_ahead(&at_end).unwrap().abs() < 0.5);

        // Pinned at the start of the north leg: its own bearing.
        let at_start = position(2, 0.0);
        assert!(index.bearing_ahead(&at_start).unwrap().abs() < 0.5);

        // Past the route end there is no direction ahead.
        let route_end = position(2, index.segments[2].length_meters);
        assert_eq!(index.bearing_ahead(&route_end), None);
    }
}
