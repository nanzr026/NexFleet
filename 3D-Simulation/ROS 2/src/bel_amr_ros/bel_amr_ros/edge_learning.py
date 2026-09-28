"""Small trainable completion-cost model and validated sample-weighted FedAvg."""
import hashlib
import json
import math
import random

FEATURES = ('battery', 'distance', 'workload', 'obstacles', 'conflicts', 'congestion')


def features(battery, distance, workload=0, obstacles=0, conflicts=0, congestion=0):
    values = (battery/100, distance/50, workload, obstacles/10, conflicts/10, congestion/3)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Non-finite model feature')
    return [max(0.0, min(1.0, value)) for value in values]


class CostModel:
    """Seven-parameter sigmoid regressor, trained with MSE gradient descent."""

    def __init__(self, weights=None):
        self.weights = [0.0] * 7 if weights is None else list(weights)
        if len(self.weights) != 7 or not all(math.isfinite(v) and abs(v) <= 100 for v in self.weights):
            raise ValueError('Invalid model weights')

    def predict(self, x):
        if len(x) != 6 or not all(math.isfinite(v) and 0 <= v <= 1 for v in x):
            raise ValueError('Invalid model feature vector')
        z = self.weights[-1] + sum(w*v for w, v in zip(self.weights, x))
        return 1 / (1 + math.exp(-max(-40, min(40, z))))

    def loss(self, samples):
        return sum((self.predict(row['x'])-row['y'])**2 for row in samples) / len(samples)

    def train(self, samples, epochs=60, learning_rate=0.4):
        if not samples:
            raise ValueError('Cannot train without local samples')
        for row in samples:
            self.predict(row['x'])
            if not math.isfinite(row['y']) or not 0 <= row['y'] <= 1:
                raise ValueError('Invalid target')
        before = self.loss(samples)
        for _ in range(epochs):
            gradient = [0.0]*7
            for row in samples:
                p = self.predict(row['x'])
                error = 2*(p-row['y'])*p*(1-p)
                for i, value in enumerate(row['x']+[1.0]):
                    gradient[i] += error*value/len(samples)
            self.weights = [w-learning_rate*g for w, g in zip(self.weights, gradient)]
        return {'loss_before': before, 'loss_after': self.loss(samples), 'samples': len(samples)}

    def digest(self):
        return hashlib.sha256(json.dumps(self.weights, separators=(',', ':')).encode()).hexdigest()


def fedavg(updates):
    """Validate and aggregate only parameters, weighted by local sample counts."""
    if not updates:
        raise ValueError('No updates')
    total = 0
    sums = [0.0]*7
    for update in updates:
        n = update['samples']
        if isinstance(n, bool) or not isinstance(n, int) or not 0 < n <= 100000:
            raise ValueError('Invalid sample count')
        weights = CostModel(update['weights']).weights
        total += n
        sums = [a+n*w for a, w in zip(sums, weights)]
    return CostModel([value/total for value in sums])


def bootstrap(robot_id, count=32):
    """Explicitly synthetic local data; not a warehouse-performance dataset."""
    rng = random.Random(robot_id)
    result = []
    for _ in range(count):
        x = [rng.random() for _ in FEATURES]
        y = min(1.0, max(0.0, 0.1 + 0.55*x[1] + 0.1*(1-x[0]) + 0.08*x[3] + 0.08*x[5]))
        result.append({'x': x, 'y': y, 'source': 'synthetic_bootstrap'})
    return result
