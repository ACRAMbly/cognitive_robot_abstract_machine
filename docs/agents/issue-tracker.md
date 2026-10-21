# Issue tracker: Local Markdown

Issues and specs live under `.scratch/`.

## Conventions

- One feature per directory: `.scratch/<feature-slug>/`.
- Specs: `.scratch/<feature-slug>/spec.md`.
- Implementation tickets: `.scratch/<feature-slug>/issues/<NN>-<slug>.md`,
  numbered from `01`, with one file per ticket.
- Record triage state in a `Status:` line near the top.
  Use the role strings in `triage-labels.md`.
- Append conversation under `## Comments`.

## Publishing and fetching

To publish, create the appropriate spec or ticket file.
To fetch a ticket, read its referenced file.

## Wayfinding operations

- Map: `.scratch/<effort>/map.md`, containing Notes,
  Decisions-so-far, and Fog.
- Child tickets: `.scratch/<effort>/issues/NN-<slug>.md`.
- Record ticket type in `Type:`:
  `research`, `prototype`, `grilling`, or `task`.
- Record dependencies in `Blocked by: NN, NN`.
  A ticket is unblocked when all dependencies are resolved.
- Select the first numbered open, unblocked, unclaimed ticket.
- Before work, save `Status: claimed`.
- To resolve, append an `## Answer`, save `Status: resolved`,
  and add a summary and link to the map's Decisions-so-far.
