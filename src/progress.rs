use indicatif::{HumanDuration, ProgressBar, ProgressBarIter, ProgressStyle};
use std::time::Instant;

/// Convenience wrapper around a progress bar that includes the total time
/// in the final message it prints out, and does so automatically when it
/// is dropped.
pub struct Progress {
    start: Instant,
    bar: ProgressBar,
    msg: String,
}

impl Drop for Progress {
    fn drop(&mut self) {
        self.bar.finish_with_message(&format!(
            "{} done in {}",
            self.msg,
            HumanDuration(self.start.elapsed())
        ));
    }
}

impl Progress {
    pub fn new(msg: &str, target: usize) -> Progress {
        let pb = ProgressBar::new(target as u64);
        pb.enable_steady_tick(200);
        pb.set_message(msg);
        pb.set_style(ProgressStyle::default_bar().template(
            "{spinner:.bold} [{elapsed_precise}] {msg:40} \
             {bar} {pos}/{len} ({eta})",
        ));
        Progress {
            start: Instant::now(),
            bar: pb,
            msg: msg.to_owned(),
        }
    }

    pub fn spinner(msg: &str) -> Progress {
        let pb = ProgressBar::new_spinner();
        pb.enable_steady_tick(200);
        pb.set_message(msg);
        pb.set_style(
            ProgressStyle::default_spinner()
                .template("{spinner:.bold} [{elapsed_precise}] {msg} ({pos})"),
        );
        Progress {
            start: Instant::now(),
            bar: pb,
            msg: msg.to_owned(),
        }
    }

    pub fn inc(&self) {
        self.bar.inc(1);
    }

    pub fn wrap_iter<It: Iterator>(&self, it: It) -> ProgressBarIter<It> {
        self.bar.wrap_iter(it)
    }
}
