from dataclasses import dataclass

from semantic_digital_twin.world_description.world_entity import Body


@dataclass
class ManipulabilityMaximizer:
    tip: Body
    root: Body

    def muh(self):
        pass
