from coraplex.alternative_motion_mapping import AlternativeMotion
from coraplex.alternative_motion_mappings.hsrb_motion_mapping import HSRBMoveMotion
from coraplex.alternative_motion_mappings.stretch_motion_mapping import (
    StretchClose,
    StretchMoveReal,
    StretchMoveSim,
    StretchMoveToolCenterPoint,
)
from coraplex.alternative_motion_mappings.tiago_motion_mapping import TiagoMoveSim

# %% discovery


def test_discover_all_finds_every_robots_alternative_motion():
    assert set(AlternativeMotion.discover_all()) == {
        HSRBMoveMotion,
        StretchMoveToolCenterPoint,
        StretchMoveSim,
        StretchMoveReal,
        StretchClose,
        TiagoMoveSim,
    }
