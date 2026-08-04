import pytest

from experiments.experiment_definitions import InvalidVolumeBoundError, VolumeBound

# %% VolumeBound


def test_volume_bound_string_representation_rounds_to_four_decimals():
    bound = VolumeBound(lower=1.23456, upper=7.89123)

    assert str(bound) == "[1.2346, 7.8912]"


def test_volume_bound_accepts_equal_lower_and_upper():
    bound = VolumeBound(lower=2.0, upper=2.0)

    assert bound.lower == bound.upper == 2.0


def test_volume_bound_rejects_lower_greater_than_upper():
    with pytest.raises(InvalidVolumeBoundError):
        VolumeBound(lower=5.0, upper=1.0)
