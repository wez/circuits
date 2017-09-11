use indicatif::{HumanDuration, ProgressBar, ProgressBarIter, ProgressStyle};
use std::time::Instant;

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

impl Drop for Progress {
    fn drop(&mut self) {
        if self.use_finish_message {
            self.bar.set_style(self.finish_style.clone());
            self.bar.finish_with_message(&format!(
                "{} done in {}",
                self.msg,
                HumanDuration(self.start.elapsed())
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
            "{spinner:.bold} [{elapsed_precise}] {msg:>50} \
             {bar} {pos}/{len} ({eta_precise})",
        ));
        pb.set_message(msg);
        pb.enable_steady_tick(200);
        Progress {
            start: Instant::now(),
            bar: pb,
            msg: msg.to_owned(),
            use_finish_message: true,
            finish_style: ProgressStyle::default_bar().template(
                "{spinner:.bold} [{elapsed_precise}] {msg:>50} \
                 {bar} {pos}/{len}",
            ),
        }
    }

    pub fn disable_finish_message(&mut self) {
        self.use_finish_message = false;
    }

    pub fn spinner(msg: &str) -> Progress {
        let pb = ProgressBar::new_spinner();
        pb.set_style(
            ProgressStyle::default_spinner()
                .template("{spinner:.bold} [{elapsed_precise}] {msg:>50} ({pos})"),
        );
        pb.set_message(msg);
        pb.enable_steady_tick(200);
        Progress {
            start: Instant::now(),
            bar: pb,
            msg: msg.to_owned(),
            use_finish_message: true,
            finish_style: ProgressStyle::default_spinner()
                .template("{spinner:.bold} [{elapsed_precise}] {msg:>50}"),
        }
    }

    pub fn inc(&self) {
        self.bar.inc(1);
    }

    pub fn wrap_iter<It: Iterator>(&self, it: It) -> ProgressBarIter<It> {
        self.bar.wrap_iter(it)
    }
}
