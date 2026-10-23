# Domain docs

## Layout and reading rules

Use a single-context layout:

- `GLOSSARY.md` at the repository root.
- Architecture decision records under `docs/adr/`.

Before exploring, read the glossary and ADRs relevant to the work.
If they are absent, proceed silently. The domain-modeling skill
creates them as terms and decisions are resolved.

## Vocabulary

Use glossary terms when naming domain concepts in code, tests,
issues, and proposals. Reconsider unfamiliar terms; record real
vocabulary gaps for domain modeling.

## Decision conflicts

Explicitly identify any proposal that contradicts an existing ADR,
including the decision reference and why it should be reconsidered.
