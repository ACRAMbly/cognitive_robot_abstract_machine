# Cognitive Robot Abstract Machine

Shared language for robot perception and its results.

## Language

**Analysis engine**:
A Robokudo perception pipeline composed of annotators.

**Annotator**:
A perception stage that contributes observations or estimates to a scene.

**Scripted observation**:
A synthetic perception observation with specified visible objects and poses,
independent of camera capture or inference.

**6D pose**:
The position and orientation of a rigid object relative to a reference coordinate frame.
_Avoid_: Position, when orientation is also part of the result

**Object model**:
The geometry of a rigid target object, shared by physical copies of that object.

**CAD frame**:
The coordinate frame in which an object's CAD geometry is defined. Its origin need
not coincide with the object's geometric center.


**Object belief**:
A persistent representation of an observed physical object, retaining its identity
and last accepted pose when the object is temporarily unobserved.

**Receiving world**:
A separate world model that receives accepted changes from the perception world.
