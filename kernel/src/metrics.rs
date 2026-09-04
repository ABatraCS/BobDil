//! Loop health, measured rather than assumed.
//!
//! Step time and realtime factor are first-class signals in BobDil, not
//! diagnostics: they are published in every state frame and shown in the UI
//! (architecture.md 1.8). A driver has to be able to see when the rig stopped
//! keeping up, because otherwise their verdict on a setup silently encodes the
//! stutter instead.
//!
//! Recording a sample must be allocation-free and branch-light, because it
//! happens inside every step. A fixed-bucket histogram gives exact-enough
//! percentiles for a microsecond-resolution budget with a single array write.

/// Fixed-bucket histogram of step times.
#[derive(Debug, Clone)]
pub struct StepTimeHistogram {
    buckets: Vec<u32>,
    bucket_ns: u64,
    overflow: u64,
    count: u64,
    sum_ns: u128,
    max_ns: u64,
}

impl StepTimeHistogram {
    /// `range_ns` is the largest step time resolved exactly; anything above it
    /// lands in the overflow bucket and still counts toward the percentiles.
    pub fn new(range_ns: u64, bucket_ns: u64) -> Self {
        let bucket_ns = bucket_ns.max(1);
        let buckets = (range_ns / bucket_ns).max(1) as usize;
        Self {
            buckets: vec![0; buckets],
            bucket_ns,
            overflow: 0,
            count: 0,
            sum_ns: 0,
            max_ns: 0,
        }
    }

    /// One array increment. No allocation, no branch on data size.
    pub fn record(&mut self, elapsed_ns: u64) {
        let index = (elapsed_ns / self.bucket_ns) as usize;
        if index < self.buckets.len() {
            self.buckets[index] += 1;
        } else {
            self.overflow += 1;
        }
        self.count += 1;
        self.sum_ns += elapsed_ns as u128;
        if elapsed_ns > self.max_ns {
            self.max_ns = elapsed_ns;
        }
    }

    pub fn count(&self) -> u64 {
        self.count
    }

    pub fn max_ns(&self) -> u64 {
        self.max_ns
    }

    pub fn mean_ns(&self) -> f64 {
        if self.count == 0 {
            0.0
        } else {
            self.sum_ns as f64 / self.count as f64
        }
    }

    /// Upper edge of the bucket holding the requested quantile.
    ///
    /// Overflow samples are attributed to the true maximum, so a percentile can
    /// never under-report a tail that ran off the end of the histogram.
    pub fn percentile_ns(&self, quantile: f64) -> u64 {
        if self.count == 0 {
            return 0;
        }
        let target = (quantile.clamp(0.0, 1.0) * self.count as f64).ceil() as u64;
        let mut cumulative = 0u64;
        for (index, hits) in self.buckets.iter().enumerate() {
            cumulative += *hits as u64;
            if cumulative >= target {
                return (index as u64 + 1) * self.bucket_ns;
            }
        }
        self.max_ns
    }

    pub fn reset(&mut self) {
        self.buckets.iter_mut().for_each(|bucket| *bucket = 0);
        self.overflow = 0;
        self.count = 0;
        self.sum_ns = 0;
        self.max_ns = 0;
    }

    /// One line for a report or a log.
    pub fn summary_us(&self) -> String {
        format!(
            "n={} mean={:.1} p50={:.1} p99={:.1} p99.9={:.1} max={:.1} us",
            self.count,
            self.mean_ns() / 1000.0,
            self.percentile_ns(0.50) as f64 / 1000.0,
            self.percentile_ns(0.99) as f64 / 1000.0,
            self.percentile_ns(0.999) as f64 / 1000.0,
            self.max_ns as f64 / 1000.0,
        )
    }
}

/// Realtime factor over a sliding window.
///
/// 1.0 means the plant is advancing exactly as fast as wall clock. Below 1.0
/// the driver is in slow motion, which they will not necessarily notice and
/// will absolutely misattribute.
#[derive(Debug, Clone)]
pub struct RealtimeFactor {
    window_ns: u64,
    window_start_ns: u64,
    sim_at_window_start: f64,
    current: f64,
}

impl RealtimeFactor {
    pub fn new(window_ns: u64) -> Self {
        Self {
            window_ns,
            window_start_ns: 0,
            sim_at_window_start: 0.0,
            current: 1.0,
        }
    }

    pub fn start(&mut self, now_ns: u64, sim_time: f64) {
        self.window_start_ns = now_ns;
        self.sim_at_window_start = sim_time;
        self.current = 1.0;
    }

    pub fn update(&mut self, now_ns: u64, sim_time: f64) -> f64 {
        let elapsed = now_ns.saturating_sub(self.window_start_ns);
        if elapsed >= self.window_ns {
            let advanced = sim_time - self.sim_at_window_start;
            self.current = advanced / (elapsed as f64 * 1e-9);
            self.window_start_ns = now_ns;
            self.sim_at_window_start = sim_time;
        }
        self.current
    }

    pub fn value(&self) -> f64 {
        self.current
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn percentiles_track_a_known_distribution() {
        let mut histogram = StepTimeHistogram::new(2_000_000, 1_000);
        for _ in 0..990 {
            histogram.record(100_000); // 100 us
        }
        for _ in 0..10 {
            histogram.record(900_000); // 900 us
        }
        assert_eq!(histogram.count(), 1000);
        assert!(histogram.percentile_ns(0.50) <= 101_000);
        assert!(histogram.percentile_ns(0.999) >= 900_000);
        assert_eq!(histogram.max_ns(), 900_000);
    }

    /// A tail that runs off the end of the histogram must never be reported as
    /// smaller than it was. Under-reporting the tail is the one failure mode
    /// that would make the timing gate meaningless.
    #[test]
    fn a_tail_past_the_histogram_range_is_not_under_reported() {
        let mut histogram = StepTimeHistogram::new(500_000, 1_000);
        for _ in 0..999 {
            histogram.record(50_000);
        }
        histogram.record(40_000_000); // a 40 ms stall
        assert_eq!(histogram.max_ns(), 40_000_000);
        assert_eq!(
            histogram.percentile_ns(1.0),
            40_000_000,
            "the worst case must survive falling outside the histogram"
        );
    }

    #[test]
    fn an_empty_histogram_reports_zero_rather_than_dividing_by_zero() {
        let histogram = StepTimeHistogram::new(1_000_000, 1_000);
        assert_eq!(histogram.percentile_ns(0.99), 0);
        assert_eq!(histogram.mean_ns(), 0.0);
    }

    #[test]
    fn realtime_factor_detects_a_loop_running_at_half_speed() {
        let mut rtf = RealtimeFactor::new(1_000_000_000);
        rtf.start(0, 0.0);
        // One second of wall clock in which the plant advanced half a second.
        let value = rtf.update(1_000_000_000, 0.5);
        assert!((value - 0.5).abs() < 1e-9, "expected 0.5, got {value}");
    }

    #[test]
    fn realtime_factor_is_one_when_the_loop_keeps_up() {
        let mut rtf = RealtimeFactor::new(1_000_000_000);
        rtf.start(0, 0.0);
        let value = rtf.update(1_000_000_000, 1.0);
        assert!((value - 1.0).abs() < 1e-9);
    }
}
