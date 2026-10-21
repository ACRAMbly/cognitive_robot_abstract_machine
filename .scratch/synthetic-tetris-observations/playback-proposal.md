# Illustrative playback defaults

Q10 and Q11 are accepted. The canonical camera, poses, and timing are now in
[the plan](../../synthetic_data_analysis_engine.md#illustrative-sequence-defaults).
The user revised the total duration to thirty seconds; stage durations are scaled
proportionally while retaining ten observations per second.

The user clarified that only one sequence is needed. The plan now ends after
reappearance and excludes looping and extra return motion. Q12's fixed clock is
test infrastructure only; ordinary playback starts capture timestamps at the
current clock time. The design interview is complete and ready for `to-spec`.

Read-only projection checks of the supplied meshes found both blocks fully in
frame and separated at all endpoints and 10,001 samples along each motion path.
These checks establish feasibility; they do not replace acceptance tests.
