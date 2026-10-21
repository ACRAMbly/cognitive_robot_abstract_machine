# Preserve scheduled observations when playback falls behind

Synthetic playback provides a reproducible observation sequence for later
annotators. Emit every scheduled observation in chronological order and preserve
its scheduled capture time, even when runner ticks arrive late. This preserves
the sequence at the cost of falling behind real time when the runner cannot
keep up.
