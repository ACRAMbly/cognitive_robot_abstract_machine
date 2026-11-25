# Cognitive Robot Abstract Machine

Domain language for robot perception, synthetic observations, and assembly geometry.

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

**Object belief**:
A persistent account of one physical object, retaining its identity and last
accepted state across observations, including observations in which it is absent.
_Avoid_: Object hypothesis, detection

**Local belief world**:
The world of object beliefs RoboKudo maintains on its own from accepted
observations, not shared with other processes.

**Shared robot world**:
The robot's world that Giskard, Coraplex and RoboKudo each hold a copy of, kept
identical by synchronization, with one identity per part.
_Avoid_: Local belief world

**Capture time**:
The moment the camera took an observation, not the moment RoboKudo accepted it.
_Avoid_: Timestamp, acceptance time

**Accepted capture**:
A capture from which RoboKudo accepted at least one observation into the shared
robot world.

**Perception point**:
A point in a task where the next motion needs a part's current pose, such as
before a grasp.

**Settle time**:
The moment the motion before a perception point has ended, so that a capture
taken afterwards shows the robot and camera at rest.

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

**Aircraft part**:
A physical component of the model aircraft, such as a wing, wheel, or body half.
_Avoid_: Plane (when the component, rather than the whole aircraft, is intended)

**Assembly path**:
The sequence of part positions and orientations through which components move
when joining or separating an assembly.

**Assembly opening**:
A passage or cavity that admits a mating component along an assembly path.

**Mating clearance**:
The separation available between mating surfaces at a particular relative pose.

**Rigid assembly part**:
A component whose shape remains unchanged throughout assembly and disassembly.

**Mating contact**:
Touching between mating surfaces without penetration of the rigid parts.

**Assembly reference**:
An agreed set of mating relationships, part poses, and assembly paths against
which preservation of assembly fit is assessed.
