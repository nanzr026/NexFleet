"""Small CPU message-passing model. JSON parameters, no pickle or PyG."""
import math
import time
import torch
from .fleet_graph import IDS, SCHEMA, digest

torch.set_num_threads(1)
ARCH = SCHEMA + ":mp7x12x12x2:remaining180:exposure2.2m10s:v1"

class GraphModel(torch.nn.Module):
    def __init__(self):
        super().__init__()
        self.embed = torch.nn.Linear(7, 12)
        self.message = torch.nn.Linear(15, 12)
        self.combine = torch.nn.Linear(24, 12)
        self.head = torch.nn.Linear(12, 2)

    def forward(self, x, edge, mask):
        h = torch.relu(self.embed(x))
        # edge[b,target,source] feeds actual neighbor messages into each node.
        source = h[:, None, :, :].expand(-1, x.shape[1], -1, -1)
        messages = torch.relu(self.message(torch.cat((source, edge), -1)))
        aggregate = (messages*mask[..., None]).sum(2)/mask.sum(2).clamp(min=1)[..., None]
        return torch.sigmoid(self.head(torch.relu(self.combine(torch.cat((h, aggregate), -1)))))

def initial_model():
    with torch.random.fork_rng():
        torch.manual_seed(17)
        return GraphModel()

def tensors(graphs, ablate=False):
    x = torch.tensor([[g["nodes"][r]["features"] for r in IDS] for g in graphs],
                     dtype=torch.float32)
    edge = torch.zeros(len(graphs), 3, 3, 3)
    mask = torch.zeros(len(graphs), 3, 3)
    for i, g in enumerate(graphs):
        if g["schema"] != SCHEMA:
            raise ValueError("Graph schema mismatch")
        for e in g["edges"]:
            target, source = IDS.index(e["target"]), IDS.index(e["source"])
            edge[i, target, source] = torch.tensor(e["features"])
            if not ablate:
                mask[i, target, source] = 1
    return x, edge, mask

def weights(model):
    return {k: v.detach().flatten().tolist() for k, v in model.state_dict().items()}

def load(model, values):
    expected = model.state_dict()
    if values.keys() != expected.keys():
        raise ValueError("Parameter names mismatch")
    state = {}
    for k, value in values.items():
        tensor = torch.tensor(value, dtype=torch.float32)
        if tensor.numel() != expected[k].numel() or not torch.isfinite(tensor).all():
            raise ValueError("Invalid parameters")
        state[k] = tensor.reshape(expected[k].shape)
    model.load_state_dict(state)

def train(model, samples, epochs=70):
    started=time.perf_counter()
    if not samples:
        raise ValueError("No measured local samples")
    x, e, m = tensors([s["graph"] for s in samples])
    ego = torch.tensor([IDS.index(s["robot_id"]) for s in samples])
    rows = torch.arange(len(samples))
    target = torch.tensor([s["target"] for s in samples], dtype=torch.float32)
    optimizer = torch.optim.Adam(model.parameters(), lr=.01)
    def loss():
        return ((model(x,e,m)[rows,ego]-target)**2).mean()
    before = float(loss().detach())
    for _ in range(epochs):
        optimizer.zero_grad()
        error = loss()
        error.backward()
        optimizer.step()
    return dict(samples=len(samples), before=before, after=float(loss().detach()),
                duration_seconds=time.perf_counter()-started)

def predict(model, graph, rid, ablate=False):
    with torch.no_grad():
        return model(*tensors([graph], ablate))[0, IDS.index(rid)].tolist()

def fedavg(updates, parent, version, run):
    if len(updates) != 3 or {u["robot_id"] for u in updates} != set(IDS):
        raise ValueError("Incomplete cohort")
    reference = initial_model()
    for u in updates:
        if (u["architecture"] != ARCH or u["parent"] != parent or
                u["version"] != version or u["run"] != run or
                not isinstance(u["samples"], int) or not 0 < u["samples"] <= 1000000):
            raise ValueError("Invalid or stale update")
        load(reference, u["weights"])
    updates = sorted(updates, key=lambda u: u["robot_id"])
    count = sum(u["samples"] for u in updates)
    return {k: [sum(u["samples"]*u["weights"][k][i] for u in updates)/count
                for i in range(len(updates[0]["weights"][k]))]
            for k in updates[0]["weights"]}
