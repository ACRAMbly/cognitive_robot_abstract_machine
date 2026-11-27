# Cognitive Robot Abstract Machine

Domain language for robot perception and synthetic observations.

## Language

**Pass**:
One run of a perception pipeline over one observation, ending with what it
perceived.

**Object hypothesis**:
A claim one pass makes about one object: what kind of object it is and where it
is. It does not by itself establish that object's identity across passes.

**Perception belief**:
What perception holds true about an object in the shared world, updated from the
object hypotheses of successive passes.

**Scripted observation**:
The object hypotheses of one pass and the time they were captured, specified by
a scenario instead of perceived from sensor data.

**Observation sequence**:
A finite, time-ordered series of scripted observations describing one scenario.

**Shared world**:
The one world that perception and the plan both read and change, kept the same
in every process that joins it.

**Aircraft**:
The Battat toy aircraft that the robot assembles from its aircraft parts.
_Avoid_: Plane, airplane

**Aircraft part**:
One of the distinct pieces the aircraft is assembled from, each with its own
shape, such as the top wing or the left rear wheel.
_Avoid_: Plane part, component

**Tetris block**:
One of the toy blocks built from unit cubes, in a straight or a
three-dimensional shape.
_Avoid_: Cube, child cube
