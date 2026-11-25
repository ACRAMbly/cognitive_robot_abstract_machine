# Scheduled passes

Status: resolved
Type: task
Blocked by: 01

Passes 0 … 299 at 10 Hz from the first request. A pass is due once its
scheduled time has come; a late caller still gets every pass in order with its
scheduled time (ADR-0001); nothing after the last pass. Time comes from an
injectable clock.

Done when tests with a controlled clock show waiting, catching up and the end.

## Answer

`synthetic_observations/playback.py` (`PassSchedule`, `Playback`, `Clock`, `SystemClock`), `NoPassDue` in `exceptions.py`. Covered by `test_synthetic_playback.py`.

## Comments
