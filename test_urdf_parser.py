import sys
import os
from semantic_digital_twin.adapters.urdf import URDFParser

urdf_path = os.path.abspath("pycram/resources/objects/box.urdf")
print("URDF path:", urdf_path)
parser = URDFParser.from_file(urdf_path, prefix="box_A")
urdf_world = parser.parse()
print("World name:", urdf_world.name)
print("Root:", urdf_world.root)
if urdf_world.root:
    print("Root name:", urdf_world.root.name)
    print("Root name name:", urdf_world.root.name.name)
else:
    print("Bodies:", [b.name for b in urdf_world.bodies])
    print("Kinematic structure entities:", [kse.name for kse in urdf_world.kinematic_structure_entities])
