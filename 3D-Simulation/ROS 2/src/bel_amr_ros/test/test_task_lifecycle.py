"""A task completes only after both travel legs and station handling."""

import pytest

from bel_amr_ros.task_lifecycle import TaskLifecycle, normalize_task


def test_delivery_required_for_completion():
    task = normalize_task({'task_id': 'a', 'pickup_x': 1, 'pickup_y': 2,
                           'delivery_x': 3, 'delivery_y': 4}, (0, 0))
    lifecycle = TaskLifecycle(task, 0)
    lifecycle.start()
    lifecycle.arrived(2)
    assert lifecycle.state == 'PICKING'
    assert not lifecycle.tick(2.9)
    assert lifecycle.tick(3)
    assert lifecycle.state == 'TO_DELIVERY'
    assert not lifecycle.tick(100)
    lifecycle.arrived(101)
    assert lifecycle.state == 'DROPPING'
    assert lifecycle.tick(102)
    assert lifecycle.state == 'COMPLETED'


def test_legacy_pickup_returns_to_home():
    task = normalize_task({'task_id': 'a', 'pickup_x': 1, 'pickup_y': 2}, (8, 9))
    assert (task['delivery_x'], task['delivery_y']) == (8, 9)


@pytest.mark.parametrize('extra', [
    {'delivery_x': 3}, {'pickup_x': float('nan')}, {'drop_seconds': -1},
])
def test_invalid_task_rejected(extra):
    with pytest.raises(ValueError):
        normalize_task(dict(task_id='a', pickup_x=1, pickup_y=2) | extra, (0, 0))
