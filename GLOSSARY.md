# Cognitive Robot Abstract Machine

Domain language for robot perception and its synthetic observations.

## Language

**Scripted observation**:
A synthetic camera observation whose scene is specified by a scenario, including
an observation in which no blocks are present.

**Observation sequence**:
A finite, time-ordered series of scripted observations describing one scenario.

**Object hypothesis**:
A claim about an object in one observation, including its classification and
geometry. It does not by itself establish that object's identity across observations.

**Detection identifier**:
The identifier of one detection within one observation. It does not establish a
physical object's identity across observations.

**Model identifier**:
The label of an object kind, shared by observations of that kind.

**Motion pause**:
An interval in a scenario during which block poses remain unchanged and
observations continue.

**Foreground mask**:
An image marking which pixels belong to one observed block, excluding surrounding
background and gaps in the block's image footprint.

**Visual geometry**:
The shape representing an object's appearance in the world.

**Collision geometry**:
The shape representing an object's occupied volume for collision checks.
