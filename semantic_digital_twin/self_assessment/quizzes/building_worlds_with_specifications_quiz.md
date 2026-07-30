---
jupytext:
    formats: md:myst
    text_representation:
        extension: .md
        format_name: myst
kernelspec:
    display_name: Python 3 (ipykernel)
    language: python
    name: python3
---

(building-worlds-with-specifications-quiz)=
# Building Worlds with Specifications Quiz

This page provides a self-check quiz for the tutorial: [](building-worlds-with-specifications).  
Source: Jupyter quiz. $ $

% NOTE: The lone `$ $` above ensures some math is rendered before the quiz,
% which fixes a known math-rendering quirk inside the quiz widget.

```{code-cell} ipython3
:tags: [remove-input]
from jupyterquiz import display_quiz

questions = [
    {
      "question": "What distinguishes a specification from the entity it describes?",
      "type": "multiple_choice",
      "answers": [
        {"answer": "It is a reusable, world-independent recipe that is materialized later", "correct": True},
        {"answer": "It is a lightweight proxy that mirrors the entity's live state", "correct": False},
        {"answer": "It is the serialized form of an entity that already exists in a world", "correct": False},
        {"answer": "It is a read-only view on the entity's geometry", "correct": False}
      ],
    },
    {
      "question": "What does spawn(world) do for a body specification?",
      "type": "multiple_choice",
      "answers": [
        {"answer": "Materializes the body, attaches it to a parent, and spawns its children in one modification block", "correct": True},
        {"answer": "Only creates the body; connections must be added manually afterwards", "correct": False},
        {"answer": "Registers the specification in the world for lazy construction", "correct": False},
        {"answer": "Returns a copy of the specification bound to the world", "correct": False}
      ],
    },
    {
      "question": "Which connection attaches a spawned entity when its specification's connection_specification is left unset?",
      "type": "multiple_choice",
      "answers": [
        {"answer": "A FixedConnection", "correct": True},
        {"answer": "A Connection6DoF", "correct": False},
        {"answer": "A PrismaticConnection", "correct": False},
        {"answer": "No connection; the entity floats unattached", "correct": False}
      ],
    },
    {
      "question": "Why does connect require the child while spawn does not take one?",
      "type": "multiple_choice",
      "answers": [
        {"answer": "A connection joins two pre-existing entities, so there is nothing to materialize", "correct": True},
        {"answer": "connect is a legacy method kept for backwards compatibility", "correct": False},
        {"answer": "The child is optional; the world root is used when omitted", "correct": False},
        {"answer": "spawn infers the child from the specification's name", "correct": False}
      ],
    },
    {
      "question": "How do get_specification and get_default_root_specification work together?",
      "type": "multiple_choice",
      "answers": [
        {"answer": "get_default_root_specification builds the root geometry; get_specification wraps it into the annotation specification", "correct": True},
        {"answer": "get_specification builds the geometry; get_default_root_specification names it", "correct": False},
        {"answer": "They are alternatives: each returns a complete annotation specification", "correct": False},
        {"answer": "get_default_root_specification spawns the root; get_specification registers the annotation", "correct": False}
      ],
    },
    {
      "question": "When are the keys of part_specifications validated?",
      "type": "multiple_choice",
      "answers": [
        {"answer": "At specification construction time, before any world is mutated", "correct": True},
        {"answer": "At spawn time, when the parts are mounted", "correct": False},
        {"answer": "Only when the world is saved or serialized", "correct": False},
        {"answer": "Never; unknown keys are silently ignored", "correct": False}
      ],
    },
    {
      "question": "What does WorldSpecification.to_domain_object return on repeated calls?",
      "type": "multiple_choice",
      "answers": [
        {"answer": "A fresh, independent world each time; the stored environment is deep-copied", "correct": True},
        {"answer": "The same world instance, cached after the first call", "correct": False},
        {"answer": "A new world that shares bodies with the previous one", "correct": False},
        {"answer": "It fails on the second call because the environment was consumed", "correct": False}
      ],
    }
]

import json
json_str = json.dumps(questions)
json.loads(json_str) 

display_quiz(questions)
```
