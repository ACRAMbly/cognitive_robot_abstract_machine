import pytest

from experiments.experiment_definitions import (
    InvalidBoundError,
    PercentageBound,
    VolumeBound,
)

# %% VolumeBound


def test_volume_bound_string_representation_rounds_to_four_decimals():
    bound = VolumeBound(lower=1.23456, upper=7.89123)

    assert str(bound) == "[1.2346, 7.8912]"


def test_volume_bound_accepts_equal_lower_and_upper():
    bound = VolumeBound(lower=2.0, upper=2.0)

    assert bound.lower == bound.upper == 2.0


def test_volume_bound_rejects_lower_greater_than_upper():
    with pytest.raises(InvalidBoundError):
        VolumeBound(lower=5.0, upper=1.0)


# %% PercentageBound


def test_percentage_bound_string_representation_rounds_to_two_decimals():
    bound = PercentageBound(lower=41.2345, upper=43.6789)

    assert str(bound) == "[41.23%, 43.68%]"


def test_percentage_bound_rejects_lower_greater_than_upper():
    with pytest.raises(InvalidBoundError):
        PercentageBound(lower=50.0, upper=10.0)


def test_ratio_of_pairs_worst_case_ends():
    numerator = VolumeBound(lower=8.0, upper=10.0)
    denominator = VolumeBound(lower=20.0, upper=40.0)

    bound = PercentageBound.ratio_of(numerator, denominator)

    # lower: smallest numerator over largest denominator; upper: largest numerator
    # over smallest denominator.
    assert bound.lower == pytest.approx(100.0 * 8.0 / 40.0)
    assert bound.upper == pytest.approx(100.0 * 10.0 / 20.0)


def test_ratio_of_clips_at_one_hundred_percent():
    numerator = VolumeBound(lower=9.0, upper=10.0)
    denominator = VolumeBound(lower=9.0, upper=10.0)

    bound = PercentageBound.ratio_of(numerator, denominator)

    assert bound.upper == 100.0


def test_ratio_of_a_fully_covered_exact_match_is_exactly_one_hundred_percent():
    exact = VolumeBound(lower=5.0, upper=5.0)

    bound = PercentageBound.ratio_of(exact, exact)

    assert bound.lower == pytest.approx(100.0)
    assert bound.upper == pytest.approx(100.0)
