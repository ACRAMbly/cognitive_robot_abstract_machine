# Issue tracker: Local Markdown

Issues and specs for this repo live as Markdown files in `.scratch/`.

## Conventions

- One feature per directory: `.scratch/<feature>/`.
- The spec is `.scratch/<feature>/spec.md`.
- Implementation issues are one file per ticket at `.scratch/<feature>/issues/<number>-<slug>.md`, numbered from `01`. Keep tickets in separate files.
- Record triage state as a `Status:` line near the top of each issue file. Use the role strings in [triage-labels.md](triage-labels.md).
- Append comments and conversation history under a `## Comments` heading at the bottom of the file.

## When a skill says "publish to the issue tracker"

Write the spec or individual ticket files at the paths above, creating directories as needed.

## When a skill says "fetch the relevant ticket"

Read the referenced file. Resolve an issue number within the feature directory; ask for the feature if the number is ambiguous.

## Wayfinding operations

For `/wayfinder`, the map is one file with a separate file for each child ticket.

- **Map**: `.scratch/<effort>/map.md`, holding the notes, decisions so far, and unresolved questions.
- **Child ticket**: `.scratch/<effort>/issues/<number>-<slug>.md`, numbered from `01`, with the question in the body. A `Type:` line records `research`, `prototype`, `grilling`, or `task`. A `Status:` line records `open`, `claimed`, or `resolved`; new tickets start `open`.
- **Blocking**: a `Blocked by: <number>, <number>` line near the top references other tickets in the same effort. A ticket is unblocked when every referenced ticket is `resolved`.
- **Frontier**: scan the effort's issue files for tickets that are `open` and unblocked; first by number wins.
- **Claim**: set `Status: claimed` and save before beginning work.
- **Resolve**: append the answer under `## Answer`, set `Status: resolved`, and append a brief finding with a link to the map's decisions so far.
