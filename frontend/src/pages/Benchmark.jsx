import React, { useState } from 'react';
import { BarChart3, Play, CheckCircle, XCircle } from 'lucide-react';
import { BarChart, Bar, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer, Legend } from 'recharts';
import { api } from '../services/api';

export default function Benchmark() {
  const [results, setResults] = useState(null);
  const [loading, setLoading] = useState(false);
  const [scenario, setScenario] = useState('baseline');
  const [nTasks, setNTasks] = useState(10);

  const runComparison = async () => {
    setLoading(true);
    try {
      const r = await api.runComparison({ scenario, seed: 42, max_ticks: 1000, n_tasks: nTasks });
      setResults(r);
    } catch (e) { alert(e.message); }
    finally { setLoading(false); }
  };

  const metrics = [
    { key: 'makespan_ticks', label: 'Makespan (ticks)', lower_better: true },
    { key: 'avg_task_completion_time', label: 'Avg Task Time', lower_better: true },
    { key: 'total_waiting_ticks', label: 'Total Wait (ticks)', lower_better: true },
    { key: 'total_distance', label: 'Total Distance', lower_better: false },
    { key: 'conflicts_detected', label: 'Conflicts', lower_better: true },
    { key: 'deadlocks_detected', label: 'Deadlocks', lower_better: true },
    { key: 'reroutes', label: 'Reroutes', lower_better: false },
    { key: 'collisions', label: 'Collisions', lower_better: true },
  ];

  const chartData = results ? metrics.map(m => ({
    name: m.label,
    Traditional: results.traditional?.[m.key] || 0,
    NexFleet: results.nexfleet?.[m.key] || 0,
  })) : [];

  return (
    <div className="page">
      <div className="page-header">
        <h2>Performance Benchmark</h2>
        <p>Head-to-head evaluation: Traditional Stop-and-Wait vs Decentralized NEXFLEET</p>
      </div>

      <div className="card mb-4">
        <div className="flex gap-3 items-center flex-wrap">
          <select value={scenario} onChange={e => setScenario(e.target.value)} className="ctrl" style={{ minWidth: 160 }}>
            {['baseline','choke_point','high_congestion','blocked_aisle','robot_failure'].map(s => (
              <option key={s} value={s}>{s.replace('_',' ').toUpperCase()}</option>
            ))}
          </select>
          <label style={{ fontSize:12, color:'var(--text-secondary)' }}>Task Count:
            <input type="number" value={nTasks} onChange={e => setNTasks(+e.target.value)}
              min={2} max={50} className="ctrl" style={{ width:60, marginLeft:6 }} />
          </label>
          <button className="btn btn-primary" onClick={runComparison} disabled={loading}>
            <Play size={14} />{loading ? 'RUNNING EXPERIMENT...' : 'RUN FAIR COMPARISON'}
          </button>
          <span style={{ fontSize:10, color:'var(--text-muted)' }}>Identical seeds, layout, and task injections are used.</span>
        </div>
      </div>

      {results && (
        <>
          {/* Target result panel */}
          <div className="card mb-4" style={{ textAlign:'center', padding: '32px 16px', borderColor: results.target_achieved ? 'var(--accent-green)' : 'var(--accent-yellow)', background: results.target_achieved ? 'rgba(16,185,129,0.04)' : 'rgba(245,158,11,0.04)' }}>
            <div style={{ display:'flex', alignItems:'center', justifyContent:'center', gap:12, marginBottom:12 }}>
              {results.target_achieved ? <CheckCircle size={32} color="var(--accent-green)" /> : <XCircle size={32} color="var(--accent-yellow)" />}
              <div style={{ fontSize:28, fontWeight:900, color: results.target_achieved ? 'var(--accent-green)' : 'var(--accent-yellow)', fontFamily:'var(--font-mono)' }}>
                {results.target_message}
              </div>
            </div>
            <div style={{ fontSize:42, fontWeight:900, color:'var(--text-primary)', fontFamily:'var(--font-mono)' }}>
              {results.improvement_percent > 0 ? '+' : ''}{results.improvement_percent}%
            </div>
            <div style={{ fontSize:14, color:'var(--text-secondary)', marginTop:8 }}>Makespan improvement vs baseline • Zero collisions: {results.zero_collisions ? 'VERIFIED ✅' : 'FAILED ❌'}</div>
          </div>

          {/* Side-by-side */}
          <div className="comparison-grid mb-4">
            <div className="card">
              <div className="card-title" style={{ color:'var(--text-muted)' }}>Traditional Baseline</div>
              {metrics.map(m => (
                <div key={m.key} className="metric-row">
                  <span className="metric-label">{m.label}</span>
                  <span className="metric-value">{(results.traditional?.[m.key] || 0).toFixed ? (results.traditional?.[m.key] || 0).toFixed(2) : results.traditional?.[m.key]}</span>
                </div>
              ))}
            </div>
            <div className="vs-divider">VS</div>
            <div className="card">
              <div className="card-title" style={{ color:'var(--accent-blue)' }}>NEXFLEET EdgeFleet-AI</div>
              {metrics.map(m => {
                const tval = results.traditional?.[m.key] || 0;
                const nval = results.nexfleet?.[m.key] || 0;
                const better = m.lower_better ? nval < tval : nval > tval;
                return (
                  <div key={m.key} className="metric-row">
                    <span className="metric-label">{m.label}</span>
                    <span className="metric-value" style={{ color: better ? 'var(--accent-green)' : 'var(--accent-red)' }}>
                      {(nval.toFixed ? nval.toFixed(2) : nval)} {better ? '▼' : '▲'}
                    </span>
                  </div>
                );
              })}
            </div>
          </div>

          {/* Chart */}
          <div className="card">
            <div className="card-title"><BarChart3 size={14} />Metrics Visualization (Lower is Better for Time/Conflicts)</div>
            <div style={{ width: '100%', height: 320, marginTop: 16 }}>
              <ResponsiveContainer width="100%" height="100%">
                <BarChart data={chartData.slice(0, 5)} margin={{ top: 5, right: 20, bottom: 20, left: 0 }}>
                  <CartesianGrid strokeDasharray="3 3" stroke="var(--border)" vertical={false} />
                  <XAxis dataKey="name" tick={{ fill: 'var(--text-muted)', fontSize: 11 }} axisLine={false} tickLine={false} />
                  <YAxis tick={{ fill: 'var(--text-muted)', fontSize: 11 }} axisLine={false} tickLine={false} />
                  <Tooltip cursor={{ fill: 'rgba(255,255,255,0.02)' }} contentStyle={{ background: 'var(--bg-secondary)', border: '1px solid var(--border)', borderRadius: 8, fontSize: 12 }} />
                  <Legend wrapperStyle={{ fontSize: 12, paddingTop: 12 }} />
                  <Bar dataKey="Traditional" fill="#4a5568" radius={[4,4,0,0]} barSize={40} />
                  <Bar dataKey="NexFleet" fill="#3b82f6" radius={[4,4,0,0]} barSize={40} />
                </BarChart>
              </ResponsiveContainer>
            </div>
          </div>
        </>
      )}
    </div>
  );
}
