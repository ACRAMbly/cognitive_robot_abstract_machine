import pytest

from krrood.entity_query_language.factories import an
from krrood.entity_query_language.operators.causal import cause
from krrood.patterns.exceptions import KeywordNamesNoFactoryParameter
from krrood.patterns.factory_and_kwargs import HasFactoryAndKwargs

from ..dataset.example_classes import KRROODPosition


def record_keywords(**keywords: float) -> dict[str, float]:
    """
    A factory that accepts arbitrary keywords.

    :return: The keywords it was called with.
    """
    return keywords


# %% keywords naming no parameter of the factory


def test_keyword_naming_no_parameter_is_refused():
    factory_and_kwargs = HasFactoryAndKwargs(
        KRROODPosition, _kwargs_={"x": 1.0, "y": 2.0, "z": 3.0, "w": 4.0}
    )
    with pytest.raises(KeywordNamesNoFactoryParameter) as error:
        factory_and_kwargs.construct_instance()
    assert error.value.factory is KRROODPosition
    assert error.value.keyword == "w"


def test_factory_accepting_arbitrary_keywords_receives_every_keyword():
    factory_and_kwargs = HasFactoryAndKwargs(record_keywords, _kwargs_={"w": 4.0})
    assert factory_and_kwargs.construct_instance() == {"w": 4.0}


def test_misspelled_match_keyword_is_refused_at_construction():
    position = an(KRROODPosition)(x=1.0, y=2.0, zz=3.0)
    with pytest.raises(KeywordNamesNoFactoryParameter) as error:
        position.construct_instance()
    assert error.value.keyword == "zz"


def test_match_keyword_marked_as_a_causal_role_is_left_out_of_construction():
    position = an(KRROODPosition)(x=1.0, y=2.0, z=3.0, distance_to_origin=cause)
    assert position.construct_instance() == KRROODPosition(1.0, 2.0, 3.0)
