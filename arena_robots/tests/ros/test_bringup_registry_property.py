"""Hypothesis property tests for ClassRegistry invariants."""

from __future__ import annotations

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from arena_rclpy_mixins.registry import ClassRegistry


@given(
    kinds=st.lists(
        st.text(
            alphabet=st.characters(whitelist_categories=("Ll", "Lu", "Nd"), min_codepoint=65),
            min_size=1,
            max_size=20,
        ),
        min_size=1,
        max_size=10,
        unique=True,
    ).map(lambda ks: [f"__prop_{k}__" for k in ks])
)
@settings(max_examples=30)
def test_all_registered_kinds_are_retrievable(kinds: list[str]) -> None:
    """After registering N unique kinds, all are retrievable."""
    reg: ClassRegistry = ClassRegistry()
    classes = {}
    for kind in kinds:

        class _Cls:
            pass

        _Cls.__name__ = kind
        classes[kind] = _Cls
        reg.register(kind)(lambda c=_Cls: c)

    for kind in kinds:
        result = reg.get(kind)
        assert result is classes[kind]


@given(
    kind=st.text(
        alphabet=st.characters(whitelist_categories=("Ll", "Lu", "Nd"), min_codepoint=65),
        min_size=1,
        max_size=20,
    ).map(lambda k: f"__propdup_{k}__")
)
@settings(max_examples=20)
def test_duplicate_registration_always_raises(kind: str) -> None:
    """Registering the same kind twice always raises."""
    reg: ClassRegistry = ClassRegistry()
    reg.register(kind)(lambda: object)
    with pytest.raises(Exception):
        reg.register(kind)(lambda: object)


@given(
    kind=st.text(
        alphabet=st.characters(whitelist_categories=("Ll",), min_codepoint=97),
        min_size=3,
        max_size=20,
    ).map(lambda k: f"__propget_{k}__")
)
@settings(max_examples=20)
def test_get_unregistered_always_raises(kind: str) -> None:
    """Getting an unregistered kind always raises KeyError."""
    reg: ClassRegistry = ClassRegistry()
    with pytest.raises(KeyError):
        reg.get(kind)
