"""Randomized trial card-mask sampling tests."""

import random

import pytest

from phy_data_collection.runner.scenario import _sample_combination_mask


@pytest.mark.parametrize(
    ("rail_count", "min_card_count"),
    ((5, 2), (5, 5), (2, 2)),
)
def test_sample_combination_mask_respects_minimum_card_count(
    rail_count: int,
    min_card_count: int,
) -> None:
    rng = random.Random(42)

    masks = {
        _sample_combination_mask(
            rng,
            rail_count=rail_count,
            min_card_count=min_card_count,
        )
        for _ in range(1_000)
    }

    assert masks
    assert all(mask.bit_count() >= min_card_count for mask in masks)
    assert all(mask < 1 << rail_count for mask in masks)


@pytest.mark.parametrize(
    ("rail_count", "min_card_count"),
    ((0, 1), (5, 0), (5, 6)),
)
def test_sample_combination_mask_rejects_invalid_card_count(
    rail_count: int,
    min_card_count: int,
) -> None:
    with pytest.raises(ValueError, match="between 1 and rail_count"):
        _sample_combination_mask(
            random.Random(42),
            rail_count=rail_count,
            min_card_count=min_card_count,
        )
