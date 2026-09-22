import React, { useState, useEffect } from 'react';
import { Share2, Play } from 'lucide-react';
import { api } from '../services/api';

export default function FederatedLearning() {
  const [status, setStatus] = useState(null);
  const [training, setTraining] = useState(false);
  const [rounds, setRounds] = useState(5);

  const load = () => api.getFederatedStatus().then(setStatus).catch(() => {});
  useEffect(() => { load(); const t = setInterval(load, 3000); return () => clearInterval(t); }, []);

  const handleTrain = async () => {
    setTraining(true);
    try { await api.trainFederated({ rounds, episodes: 15 }); }
    catch {} finally { setTimeout(() => setTraining(false), 2000); }
  };

  const robotIds = ['AMR-01', 'AMR-02', 'AMR-03'];
  const ROBOT_COLORS = { 'AMR-01': '#3b82f6', 'AMR-02': '#10b981', 'AMR-03': '#f59e0b' };

  return (
    <div className="page">
      <div className="page-header">
        <h2>Federated Learning</h2>
        <p>Distributed GNN training preserving trajectory data privacy</p>
      </div>

      <div className="card mb-4">
        <div className="flex gap-3 items-center">
          <label style={{ fontSize:12, color:'var(--text-secondary)' }}>FL Rounds:
            <input type="number" value={rounds} onChange={e => setRounds(+e.target.value)} className="ctrl"
              min={1} max={20} style={{ width:60, marginLeft:6 }} />
          </label>
          <button className="btn btn-primary" onClick={handleTrain} disabled={training}>
            <Play size={14} />{training ? 'TRAINING IN BACKGROUND...' : 'START FEDERATED TRAINING'}
          </button>
          {training && <span className="badge badge-yellow">Processing distributed updates...</span>}
        </div>
      </div>

      {/* FL Architecture Diagram */}
      <div className="card mb-4">
        <div className="card-title"><Share2 size={14} />Federation Architecture (FedAvg)</div>
        <div className="fl-diagram">
          <div className="fl-node fl-node-global" style={{ minWidth:180, padding: 16 }}>
            <div style={{ fontSize: 16 }}>🧠 GLOBAL GNN</div>
            <div style={{ fontSize:11, marginTop:6, color:'var(--accent-blue)' }}>
              Version: {status?.model_version || 0}
            </div>
          </div>
          <div className="fl-arrow" style={{ padding: 12 }}>↓ parameters ↓</div>
          <div className="fl-row">
            {robotIds.map(rid => (
              <div key={rid} className="fl-node" style={{ borderColor: ROBOT_COLORS[rid] }}>
                <div style={{ color: ROBOT_COLORS[rid], fontWeight:800, fontSize: 13 }}>{rid}</div>
                <div style={{ fontSize:10, color:'var(--text-muted)', marginTop:6 }}>Local Partitions</div>
                <div style={{ fontSize:10, marginTop:4, color: 'var(--text-primary)' }}>• Private Trajectories</div>
                <div style={{ fontSize:10, marginTop:2, color: 'var(--text-primary)' }}>• Local Gradients</div>
              </div>
            ))}
          </div>
          <div className="fl-arrow" style={{ padding: 12 }}>↓ local model updates ↓</div>
          <div className="fl-node" style={{ borderColor:'var(--accent-purple)', background:'rgba(139,92,246,0.08)', padding: 16 }}>
            <div style={{ color:'var(--accent-purple)', fontWeight:800, fontSize: 14 }}>Secure Aggregator</div>
            <div style={{ fontSize:11, color:'var(--text-muted)', marginTop:6, fontFamily: 'var(--font-mono)' }}>Σ(n_k × w_k) / Σn_k</div>
          </div>
          <div className="fl-arrow" style={{ padding: 12 }}>↓ updated global model ↓</div>
        </div>
      </div>

      {/* Status Metrics */}
      <div className="card">
        <div className="card-title">Training Metrics & History</div>
        <div className="grid-3 mb-4">
          <div className="stat-card">
            <div className="stat-label">Model Version</div>
            <div className="stat-value text-accent-blue">{status?.model_version || 0}</div>
          </div>
          <div className="stat-card">
            <div className="stat-label">Rounds Completed</div>
            <div className="stat-value">{status?.n_rounds_completed || 0}</div>
          </div>
          <div className="stat-card">
            <div className="stat-label">Federated Clients</div>
            <div className="stat-value">{(status?.clients || robotIds).length}</div>
          </div>
        </div>
        
        {status?.history && Object.keys(status.history).length > 0 ? (
          <table className="data-table">
            <thead>
              <tr><th>Round</th><th>Val Loss</th><th>Val Accuracy</th><th>Val F1 Score</th></tr>
            </thead>
            <tbody>
              {Object.entries(status.history).map(([r, metrics]) => (
                <tr key={r}>
                  <td>{r}</td>
                  <td style={{ color: 'var(--accent-cyan)' }}>{metrics.val_loss?.toFixed(4)}</td>
                  <td>{(metrics.val_acc * 100).toFixed(1)}%</td>
                  <td style={{ color: 'var(--accent-green)' }}>{metrics.val_f1?.toFixed(4)}</td>
                </tr>
              ))}
            </tbody>
          </table>
        ) : (
          <div style={{ fontSize: 12, color: 'var(--text-muted)', textAlign: 'center', padding: 20 }}>
            No training history available. Run federated training to generate metrics.
          </div>
        )}
      </div>
    </div>
  );
}
