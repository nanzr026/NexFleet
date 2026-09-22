const BASE_URL = 'http://localhost:8000';
const WS_URL = 'ws://localhost:8000/ws/fleet';

export const api = {
  async get(path) {
    const res = await fetch(`${BASE_URL}${path}`);
    if (!res.ok) throw new Error(`API error: ${res.status}`);
    return res.json();
  },
  async post(path, body = {}) {
    const res = await fetch(`${BASE_URL}${path}`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    });
    if (!res.ok) throw new Error(`API error: ${res.status}`);
    return res.json();
  },
  async delete(path, body = {}) {
    const res = await fetch(`${BASE_URL}${path}`, {
      method: 'DELETE',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    });
    return res.json();
  },
  health: () => api.get('/api/health'),
  getState: () => api.get('/api/state'),
  getRobots: () => api.get('/api/robots'),
  getTasks: () => api.get('/api/tasks'),
  getMetrics: () => api.get('/api/metrics'),
  getEvents: (limit = 50) => api.get(`/api/events?limit=${limit}`),
  getAIStatus: () => api.get('/api/ai/status'),
  getNetwork: () => api.get('/api/network'),
  getFederatedStatus: () => api.get('/api/federated/status'),
  getAuction: () => api.get('/api/auction'),
  getAuctionHistory: (limit = 20) => api.get(`/api/auction/history?limit=${limit}`),
  getInteractionGraph: () => api.get('/api/ai/interaction-graph'),

  startSimulation: (body) => api.post('/api/simulation/start', body),
  pauseSimulation: () => api.post('/api/simulation/pause'),
  resetSimulation: (body) => api.post('/api/simulation/reset', body),
  createTask: (body) => api.post('/api/tasks', body),
  addObstacle: (body) => api.post('/api/obstacles', body),
  removeObstacle: (body) => api.delete('/api/obstacles', body),
  failRobot: (id) => api.post(`/api/robots/${id}/fail`),
  recoverRobot: (id) => api.post(`/api/robots/${id}/recover`),
  setNetworkConfig: (body) => api.post('/api/network/config', body),
  runComparison: (body) => api.post('/api/experiments/comparison', body),
  trainGNN: (body) => api.post('/api/ai/gnn/train', body),
  trainMARL: (body) => api.post('/api/ai/marl/train', body),
  trainFederated: (body) => api.post('/api/federated/train', body),
};

export { WS_URL };
