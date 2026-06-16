---
jupytext:
  text_representation:
    extension: .md
    format_name: myst
    format_version: 0.13
    jupytext_version: 1.19.3
kernelspec:
  display_name: Python 3
  language: python
  name: python3
---

# Role Pattern — User Guide

The Role pattern lets an existing object take on a new semantic context — adding context-specific
attributes and behaviour — while **remaining the same entity semantically**. A role and its role taker share
identity: they compare equal and have the same hash (not same in memory ID/address).

This guide uses a university ontology as its running example. A `Person` is a persistent entity
with a name. Over time that person may become a `CEO` or a `Professor`. These are roles: they
add context-specific data, but the person's identity never changes.

## Defining a Role Taker

A role taker is any dataclass you define.

```{code-cell} ipython3
from __future__ import annotations

from dataclasses import dataclass, field
from typing_extensions import List

from krrood.entity_query_language.predicate import Symbol


@dataclass(eq=False)
class Person(Symbol):
    """A persistent human entity with a unique name."""

    name: str

    def __hash__(self) -> int:
        return hash(self.name)


@dataclass(eq=False)
class Company(Symbol):
    """An organisation that people can head or work for."""

    name: str

    def __hash__(self) -> int:
        return hash(self.name)


@dataclass(eq=False)
class Course(Symbol):
    """An academic course that a professor can teach."""

    name: str

    def __hash__(self) -> int:
        return hash(self.name)


alice = Person(name="Alice")
print(alice)
```

## Defining a Role

A role is a dataclass that inherits from `Role[T]`, where `T` is the type of its role taker.
The role taker is stored in a single field that you mark with `role_taker_field()`.

```{code-cell} ipython3
from krrood.patterns.role import Role, role_taker_field


@dataclass(eq=False)
class CEO(Role[Person]):
    """The role of being the chief executive of a company."""

    person: Person = role_taker_field()
    head_of: Company = None
```

Notice that `CEO` inherits from `Role[Person]` — it does **not** inherit from
`Person`. The role and its taker are entirely separate classes connected only through the
`role_taker_field`.

## Constructing a Role

Role construction is explicit. You pass the role taker as a keyword argument to the role's
constructor, just like any other dataclass field.

```{code-cell} ipython3
acme = Company(name="ACME")
ceo = CEO(person=alice, head_of=acme)
print(ceo)
```

The role registers itself with the `SymbolGraph` during `__post_init__`, which is what enables role lookup and
reasoning.

## Identity Sharing

A role and its role taker are considered the same entity. They compare equal and share the same
hash. This means they occupy the same slot in sets and dictionaries.

```{code-cell} ipython3
print("alice == ceo:", alice == ceo)
print("hash(alice) == hash(ceo):", hash(alice) == hash(ceo))
print("set size:", len({alice, ceo}))
```

## Attribute Delegation

Attributes that are not declared on the role are automatically delegated to the role taker via
`__getattr__`. You can read and write role-taker attributes through the role as if they were
declared there.

```{code-cell} ipython3
# Reading a role-taker attribute through the role
print("ceo.name:", ceo.name)

# Writing a role-taker attribute through the role modifies the taker
ceo_person_before = alice.name
ceo.name = "Alicia"
print("alice.name after writing through ceo:", alice.name)

# Restore the original name
alice.name = ceo_person_before

# Role-native attributes live on the role, not the taker
print("ceo.head_of:", ceo.head_of)
print("hasattr(alice, 'head_of'):", hasattr(alice, "head_of"))
```

## Querying Roles

`Role` provides class methods for checking whether a role taker has a role and for retrieving
role instances.

```{code-cell} ipython3
# Check whether alice has a CEO role
print("has CEO role:", Role.has_role(alice, CEO))

# Retrieve all CEO roles for alice
ceos = Role.roles_for(alice, CEO)
print("CEO roles:", ceos)

# Retrieve all roles of any type
all_roles = Role.roles_for(alice)
print("all roles:", all_roles)
```

## Parallel Roles

A single role taker can hold multiple roles simultaneously. Each role is a separate object, but
all of them are equal to the taker (and to each other).

```{code-cell} ipython3
@dataclass(eq=False)
class Professor(Role[Person], Symbol):
    """The role of being an academic who teaches courses."""

    person: Person = role_taker_field()
    teaches: List[Course] = field(default_factory=list, kw_only=True)


cs_101 = Course(name="CS 101")
professor = Professor(person=alice, teaches=[cs_101])

print("ceo == professor:", ceo == professor)
print("professor == alice:", professor == alice)
print("set size:", len({alice, ceo, professor}))

# Each role preserves its own context-specific data
print("CEO heads:", ceo.head_of)
print("Professor teaches:", professor.teaches)

# The taker knows about every role it currently holds
print("all roles of alice:", Role.roles_for(alice))
```

## Role Chaining

A role's role taker can itself be a role. This lets you layer multiple levels of semantic
context onto the same entity.

```{code-cell} ipython3
@dataclass(eq=False)
class Representative(Role[CEO]):
    """The role of a CEO acting as a representative of their company."""

    ceo: CEO = role_taker_field()
    represents: Company = None


rep = Representative(ceo=ceo, represents=acme)

print("rep == ceo:", rep == ceo)
print("rep == alice:", rep == alice)
print("set size:", len({alice, ceo, professor, rep}))
```

The chain is transparent. Querying from any point in the chain resolves all the way back to
the root entity.

```{code-cell} ipython3
# Check roles from the perspective of the root taker
print("alice has Representative role:", Role.has_role(alice, Representative))
print(
    "Representative roles of alice:",
    Role.roles_for(alice, Representative),
)

# Navigate the chain programmatically
print("rep.role_taker:", rep.role_taker)
print("rep.root_persistent_entity:", rep.root_persistent_entity)
print("root type:", Representative.get_root_role_taker_type())
```

## The `from_role_taker` Factory

When you do not have extra role-specific data to supply at construction time, you can use the
`from_role_taker` class method. It constructs the role by passing only the role taker.

```{code-cell} ipython3
# Equivalent to CEO(person=alice) when no role-native fields need values up front
new_ceo = CEO.from_role_taker(alice)
print("new_ceo:", new_ceo)
```

## Available Features

| Feature | How to use it |
|---|---|
| Define a role class | Inherit from `Role[T]`, mark the taker field with `role_taker_field()` |
| Construct a role | `MyRole(taker_field=taker_instance, ...)` — always explicit |
| Construct without extra data | `MyRole.from_role_taker(taker_instance)` |
| Check whether a taker has a role | `Role.has_role(taker, MyRole)` |
| Retrieve roles of a type | `Role.roles_for(taker, MyRole)` |
| Retrieve all roles | `Role.roles_for(taker)` |
| Access the immediate role taker | `role.role_taker` |
| Access the root non-role entity | `role.root_persistent_entity` |
| Find the root role-taker type | `MyRole.get_root_role_taker_type()` |
| Read/write taker attributes via role | Use the role instance directly — delegation is automatic |

## When to Use the Role Pattern

- An object needs context-specific attributes that do not logically belong to the original type.
- The additional context is temporary or situational — the entity may gain and lose the role
  over its lifetime.
- The same entity can play multiple semantic roles simultaneously.
- Any part of the system must be able to ask "does this entity have role X?" without scanning
  all instances.
- The role and the original object can be considered the same entity throughout the system.

## When Not to Use the Role Pattern

- The new class changes the permanent, identifying properties of the original. If a `Fruit`
  cannot exist without being an `Apple`, subclassing `Apple` from `Fruit` is the right choice.
- The new class has its own independent persistent identity.
- You do not need identity sharing between the two objects.

---

## Decision Guide

```{mermaid}
%%{init: {'theme': 'base', 'themeVariables': {'fontSize': '20px', 'lineColor': '#333333', 'primaryColor': '#ddeeff', 'primaryTextColor': '#111111', 'primaryBorderColor': '#3a7abf', 'edgeLabelBackground': '#ffffff'}, 'flowchart': {'nodeSpacing': 60, 'rankSpacing': 80, 'padding': 20}}}%%
flowchart TD
    Q1{"Should the wrapper and<br/>the original object be<br/>the same entity?"}
    Q2{"Is this a temporary<br/>contextual role, or does it<br/>change persistent identity?"}
    ROLE["<b>Role[T]</b><br/>Identity-sharing, role registry,<br/>optional chaining."]
    SUBCLASS["<b>Subclass</b><br/>The new class has its own<br/>persistent identifying properties."]
    ASSOCIATION["<b>Association</b><br/>The new class is a different kind<br/>of thing from the original."]

    Q1 -->|Yes| Q2
    Q1 -->|No| ASSOCIATION
    Q2 -->|Temporary / contextual| ROLE
    Q2 -->|Persistent identity change| SUBCLASS
```

---

## Learn More

- **Developer guide**: {doc}`developer/role` — design decisions, architecture, and extension points.
- **API reference**: `krrood.patterns.role` — `Role`, `role_taker_field`, `HasRoleTaker`,
  `RoleTakerFieldNotFound`.
- **Source**: `krrood/src/krrood/patterns/role.py`
- **Tests**: `test/krrood_test/test_patterns/test_role.py` and the dataset under
  `test/krrood_test/dataset/role_and_ontology/`.
