# Cognitive Robot Abstract Machine

Shared language for robot perception and its results.

## Language

**Analysis engine**:
A Robokudo perception pipeline composed of annotators.

**Annotator**:
A perception stage that contributes observations or estimates to a scene.

**6D pose**:
The position and orientation of a rigid object relative to a reference coordinate frame.
_Avoid_: Position, when orientation is also part of the result

**Object model**:
The geometry of a rigid target object, shared by physical copies of that object.

**CAD frame**:
The coordinate frame in which an object's CAD geometry is defined. Its origin need
not coincide with the object's geometric center.
