"""Actual parameter training/aggregation checks, not printed claims."""
import math
import pytest

from bel_amr_ros.edge_learning import CostModel, bootstrap, fedavg


def test_local_training_changes_parameters_and_reduces_loss():
    model = CostModel()
    before = model.digest()
    metrics = model.train(bootstrap('amr_01'))
    assert model.digest() != before
    assert metrics['loss_after'] < metrics['loss_before']


def test_fedavg_is_weighted_by_samples():
    model = fedavg([{'weights': [1.0]*7, 'samples': 1},
                    {'weights': [3.0]*7, 'samples': 3}])
    assert model.weights == [2.5]*7


def test_three_independent_peers_converge_to_same_parameters():
    peers = [CostModel() for _ in range(3)]
    for _ in range(3):
        updates = []
        for i, model in enumerate(peers):
            rows = bootstrap(f'amr_{i+1:02d}', count=16+i*8)
            model.train(rows)
            updates.append({'weights': model.weights, 'samples': len(rows)})
        peers = [fedavg(updates) for _ in peers]
        assert len({peer.digest() for peer in peers}) == 1


@pytest.mark.parametrize('update', [
    {'weights': [0.0]*7, 'samples': 0},
    {'weights': [math.nan]*7, 'samples': 3},
    {'weights': [0.0]*6, 'samples': 3},
])
def test_rejects_invalid_updates(update):
    with pytest.raises(ValueError):
        fedavg([update])
