# Domain docs

This repo uses a single shared domain context across its workspace packages.

## Before exploring

- Read `GLOSSARY.md` at the repo root.
- Read architecture decision records under the root `docs/adr/` that concern the area you are exploring.
- If these files or directories do not exist, proceed silently. Do not suggest creating them upfront. `/domain-modeling` creates documentation when terms or decisions are resolved.

## Layout

| Location, relative to the repo root | Contents |
| --- | --- |
| `GLOSSARY.md` | Shared domain terms. |
| `docs/adr/` | Architecture decision records. |

## Use the glossary's vocabulary

When naming a domain concept in an issue, proposal, hypothesis, or test, use the term defined in `GLOSSARY.md`.

If the concept is missing, reconsider whether it belongs to the project's vocabulary. Record a genuine terminology gap for `/domain-modeling`.

## Surface decision conflicts

If a proposal contradicts an existing architecture decision record, identify that record and explain why the decision should be reconsidered.
