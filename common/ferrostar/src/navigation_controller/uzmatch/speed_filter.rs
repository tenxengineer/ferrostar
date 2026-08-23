//! Port of NavKit `SpeedFilter` (location_guide/speed_filter.cpp).
//!
//! Low-pass filter over recent speed samples with vendor weight tables.
//! Provenance: maps-product `backend/mobile/libs/directions/guidance/location_guide/speed_filter.{h,cpp}`.

#[cfg(all(feature = "std", not(feature = "web-time")))]
use std::time::{Duration, SystemTime};
#[cfg(feature = "web-time")]
use web_time::{Duration, SystemTime};

use serde::{Deserialize, Serialize};

/// Extra half second beyond the nominal 4 s window to smooth out time measurement errors.
pub const SPEED_OUTDATING: Duration = Duration::from_millis(4500);
/// Minimum interval between two samples for the pair to be usable.
pub const MINIMAL_LOCATION_TIME_DIFFERENCE: Duration = Duration::from_millis(100);
/// Maximum number of samples kept in the window.
pub const MAX_SAVED_SPEEDS: usize = 5;

/// Vendor low-pass filter weights, indexed by sample count (2..=5).
/// Source: `lowPassFilterWeights` in speed_filter.cpp.
pub fn low_pass_filter_weights(size: usize) -> Option<&'static [f64]> {
    match size {
        2 => Some(&[0.5, 0.5]),
        3 => Some(&[0.225388, 0.549223, 0.225388]),
        4 => Some(&[0.146436, 0.353564, 0.353564, 0.146436]),
        5 => Some(&[0.10292, 0.242081, 0.309998, 0.242081, 0.10292]),
        _ => None,
    }
}

/// A speed sample with its observation time.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "uniffi", derive(uniffi::Record))]
pub struct SpeedSample {
    /// Speed in meters per second.
    pub speed_mps: f64,
    /// When the sample was observed.
    pub timestamp: SystemTime,
}

/// Returns true when the interval between two samples is usable for filtering.
pub fn is_interval_suitable(interval: Duration) -> bool {
    interval >= MINIMAL_LOCATION_TIME_DIFFERENCE && interval <= SPEED_OUTDATING
}

/// Append a speed sample to the history, applying vendor cleanup and gap filling.
///
/// Mirrors `SpeedFilter::appendSpeed`: out-of-order timestamps clear the history;
/// entries older than [`SPEED_OUTDATING`] or beyond [`MAX_SAVED_SPEEDS`] are dropped;
/// gaps between consecutive samples are filled with linearly interpolated
/// one-second-spaced samples.
pub fn append_speed(history: &mut Vec<SpeedSample>, speed_mps: f64, timestamp: SystemTime) {
    clean_up_speed_history(history, timestamp);
    let sample = SpeedSample {
        speed_mps,
        timestamp,
    };
    if let Some(last) = history.last().copied() {
        fill_gap(history, last, sample);
    }
    history.push(sample);
}

/// Current filtered speed, or `None` when fewer than two samples are in the window
/// (vendor: "Use at least two speeds to suppress noise").
pub fn filtered_speed(history: &mut Vec<SpeedSample>, timestamp: SystemTime) -> Option<f64> {
    clean_up_speed_history(history, timestamp);
    let weights = low_pass_filter_weights(history.len())?;
    Some(
        history
            .iter()
            .zip(weights.iter())
            .map(|(sample, weight)| sample.speed_mps * weight)
            .sum(),
    )
}

fn clean_up_speed_history(history: &mut Vec<SpeedSample>, timestamp: SystemTime) {
    if let Some(last) = history.last() {
        if timestamp < last.timestamp {
            history.clear();
            return;
        }
    }
    while let Some(front) = history.first() {
        let too_many = history.len() > MAX_SAVED_SPEEDS;
        let too_old = front
            .timestamp
            .checked_add(SPEED_OUTDATING)
            .map(|expiry| expiry < timestamp)
            .unwrap_or(true);
        if too_many || too_old {
            history.remove(0);
        } else {
            break;
        }
    }
}

/// Fill the gap between two samples with one-second-spaced lerped samples.
///
/// Port of `SpeedFilter::fillGap`: the time difference is rounded to whole
/// seconds and that many equal intervals are interpolated.
fn fill_gap(history: &mut Vec<SpeedSample>, from: SpeedSample, to: SpeedSample) {
    let Ok(diff) = to.timestamp.duration_since(from.timestamp) else {
        return;
    };
    let number_of_intervals = diff.as_secs_f64().round() as u64;
    for i in 1..number_of_intervals {
        let t = i as f64 / number_of_intervals as f64;
        let interpolated_speed = from.speed_mps + t * (to.speed_mps - from.speed_mps);
        let interpolated_timestamp = from.timestamp + diff.mul_f64(t);
        history.push(SpeedSample {
            speed_mps: interpolated_speed,
            timestamp: interpolated_timestamp,
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(secs: u64) -> SystemTime {
        SystemTime::UNIX_EPOCH + Duration::from_secs(secs)
    }

    #[test]
    fn requires_at_least_two_samples() {
        let mut history = Vec::new();
        append_speed(&mut history, 10.0, at(0));
        assert_eq!(filtered_speed(&mut history, at(0)), None);
        append_speed(&mut history, 12.0, at(1));
        assert_eq!(filtered_speed(&mut history, at(1)), Some(11.0));
    }

    #[test]
    fn vendor_weights_three_samples() {
        let mut history = Vec::new();
        append_speed(&mut history, 10.0, at(0));
        append_speed(&mut history, 20.0, at(1));
        append_speed(&mut history, 30.0, at(2));
        let speed = filtered_speed(&mut history, at(2)).unwrap();
        let expected = 0.225388 * 10.0 + 0.549223 * 20.0 + 0.225388 * 30.0;
        assert!((speed - expected).abs() < 1e-9);
    }

    #[test]
    fn evicts_samples_older_than_window() {
        let mut history = Vec::new();
        append_speed(&mut history, 50.0, at(0));
        append_speed(&mut history, 10.0, at(10));
        append_speed(&mut history, 10.0, at(11));
        // The 50 mps sample at t=0 is older than 4.5 s relative to t=11.
        assert_eq!(filtered_speed(&mut history, at(11)), Some(10.0));
    }

    #[test]
    fn caps_history_at_max_saved_speeds() {
        let mut history = Vec::new();
        for i in 0..8 {
            append_speed(&mut history, i as f64, at(i));
        }
        assert!(history.len() <= MAX_SAVED_SPEEDS);
    }

    #[test]
    fn out_of_order_timestamp_clears_history() {
        let mut history = Vec::new();
        append_speed(&mut history, 10.0, at(10));
        append_speed(&mut history, 12.0, at(11));
        append_speed(&mut history, 8.0, at(5));
        assert_eq!(history.len(), 1);
        assert_eq!(filtered_speed(&mut history, at(5)), None);
    }

    #[test]
    fn fills_multi_second_gaps_with_lerped_samples() {
        let mut history = Vec::new();
        append_speed(&mut history, 10.0, at(0));
        append_speed(&mut history, 13.0, at(3));
        // Gap of 3 s -> two interpolated samples at t=1 (11.0) and t=2 (12.0).
        assert_eq!(history.len(), 4);
        assert!((history[1].speed_mps - 11.0).abs() < 1e-9);
        assert!((history[2].speed_mps - 12.0).abs() < 1e-9);
        assert_eq!(history[1].timestamp, at(1));
        assert_eq!(history[2].timestamp, at(2));
    }

    #[test]
    fn interval_suitability_matches_vendor_bounds() {
        assert!(!is_interval_suitable(Duration::from_millis(99)));
        assert!(is_interval_suitable(Duration::from_millis(100)));
        assert!(is_interval_suitable(Duration::from_millis(4500)));
        assert!(!is_interval_suitable(Duration::from_millis(4501)));
    }
}
