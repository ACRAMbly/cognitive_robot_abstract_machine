# Apply belief updates only between plans, at perception points

Coraplex holds back incoming world updates (`defer_incoming_updates=True`) and
applies them only between two plans. The demo script splits each task into plans
so that every perception point falls between two of them. There it triggers one
capture and waits until each part the next plan needs has an update captured
after the previous plan ended. Then it applies the updates and builds the next
plan from the updated world. The world never changes under a running plan, and
the `coraplex/` package needs no change.

## Considered Options

- **Apply updates as they arrive.** The world would change under a running plan.
- **Apply whatever has arrived at the perception point, without waiting.** The
  update may come from a capture taken before the robot moved the part, or while
  the arm or camera was still moving.
- **A `coraplex/` plan node that waits and applies inside a plan.** It needs a
  `coraplex/` change. It is kept only as the fallback for a perception point that
  has to sit in the middle of one plan's motion.

## Consequences

- A plan acts on the pose from the capture taken before it started, and nothing
  newer.
- Retries (a timed-out observation, a failed grasp that needs another look) move
  from the plan into the demo script.
- The wait needs the capture time of each accepted observation (ADR 0004),
  and it needs a timeout, because an occluded part never produces a fresh
  observation.
- A part the robot holds across a plan boundary is still overwritten by
  RoboKudo's next accepted pose when the updates are applied.
