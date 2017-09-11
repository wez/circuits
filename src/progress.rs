use indicatif::{ProgressBar, ProgressBarIter, ProgressStyle};
use std::time::{Duration, Instant};
use std::fmt;

/// Convenience wrapper around a progress bar that includes the total time
/// in the final message it prints out, and does so automatically when it
/// is dropped.
pub struct Progress {
    start: Instant,
    bar: ProgressBar,
    msg: String,
    use_finish_message: bool,
    finish_style: ProgressStyle,
}

struct PreciseFormattedDuration(pub Duration);

impl fmt::Display for PreciseFormattedDuration {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let secs = self.0.as_secs();
        let alt = f.alternate();
        macro_rules! try_unit {
            ($secs:expr, $sg:expr, $pl:expr, $s:expr) => {
                let cnt = secs / $secs;
                if cnt == 1 {
                    if alt {
                        return write!(f, "{}{}", cnt, $s);
                    } else {
                        return write!(f, "{} {}", cnt, $sg);
                    }
                } else if cnt > 1 {
                    if alt {
                        return write!(f, "{}{}", cnt, $s);
                    } else {
                        return write!(f, "{} {}", cnt, $pl);
                    }
                }
            }
        }

        try_unit!(365 * 24 * 60 * 60, "year", "years", "y");
        try_unit!(7 * 24 * 60 * 60, "week", "weeks", "w");
        try_unit!(24 * 60 * 60, "day", "days", "d");
        try_unit!(60 * 60, "hour", "hours", "h");
        try_unit!(60, "minute", "minutes", "m");
        try_unit!(1, "second", "seconds", "s");

        // Small enough that we want to look at subsecond values
        let nanos = self.0.subsec_nanos();

        if nanos > 1_000_000 {
            return write!(f, "{}ms", nanos / 1_000_000);
        }

        if nanos > 1000 {
            return write!(f, "{}us", nanos / 1000);
        }

        write!(f, "{}ns", nanos)
    }
}

impl Drop for Progress {
    fn drop(&mut self) {
        if self.use_finish_message {
            self.bar.set_style(self.finish_style.clone());
            self.bar.finish_with_message(&format!(
                "{} done in {:#}",
                self.msg,
                PreciseFormattedDuration(self.start.elapsed())
            ));
        } else {
            self.bar.finish_and_clear();
        }
    }
}

impl Progress {
    pub fn new(msg: &str, target: usize) -> Progress {
        let pb = ProgressBar::new(target as u64);
        pb.set_style(ProgressStyle::default_bar().template(
            "{spinner:.bold} [{elapsed}] {msg:<50} \
             {bar} {pos}/{len} ({eta})",
        ));
        pb.set_message(msg);
        pb.enable_steady_tick(200);
        Progress {
            start: Instant::now(),
            bar: pb,
            msg: msg.to_owned(),
            use_finish_message: true,
            finish_style: ProgressStyle::default_bar().template("{msg:<50} {bar} {pos}/{len}"),
        }
    }

    pub fn disable_finish_message(&mut self) {
        self.use_finish_message = false;
    }

    pub fn spinner(msg: &str) -> Progress {
        let pb = ProgressBar::new_spinner();
        pb.set_style(
            ProgressStyle::default_spinner()
                .template("{spinner:.bold} [{elapsed}] {msg:<50} ({pos})"),
        );
        pb.set_message(msg);
        pb.enable_steady_tick(200);
        Progress {
            start: Instant::now(),
            bar: pb,
            msg: msg.to_owned(),
            use_finish_message: true,
            finish_style: ProgressStyle::default_spinner().template("{msg:<50}"),
        }
    }

    pub fn inc(&self) {
        self.bar.inc(1);
    }

    pub fn wrap_iter<It: Iterator>(&self, it: It) -> ProgressBarIter<It> {
        self.bar.wrap_iter(it)
    }
}
