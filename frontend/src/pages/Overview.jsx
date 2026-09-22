import React, { useEffect, useState } from 'react';
import { Activity, Bot, ClipboardList, Zap, AlertTriangle, CheckCircle } from 'lucide-react';
import { api } from '../services/api';

const STATUS_COLOR = {
  IDLE: 'gray', MOVING: 'blue', WAITING: 'yellow',
  REROUTING: 'purple', BLOCKED: 'red', CHARGING: 'cyan', FAILED: 'red',
};

export default function Overview({ fleetState }) {
  const [health, setHealth] = useState(null);
  const [aiStatus, setAiStatus] = useState(null);

  useEffect(() => {
    api.health().then(setHealth).catch(() => setHealth({ status: 'unreachable' }));
    api.getAIStatus().then(setAiStatus).catch(() => {});
  }, []);

  const metrics = fleetState?.metrics || {};
  const robots = fleetState?.robots || [];
  const tasks = fleetState?.tasks || [];
  const activeRobots = robots.filter(r => r.status !== 'IDLE' && r.status !== 'FAILED').length;
  const completedTasks = tasks.filter(t => t.status === 'COMPLETED').length;

  const stats = [
    { label: 'Active Robots', value: `${activeRobots}/${robots.length}`, icon: Bot, color: 'var(--accent-blue)' },
    { label: 'Tasks Completed', value: completedTasks, icon: CheckCircle, color: 'var(--accent-green)' },
    { label: 'Conflicts Detected', value: metrics.conflicts_detected || 0, icon: AlertTriangle, color: 'var(--accent-yellow)' },
    { label: 'Collisions', value: metrics.collisions || 0, icon: Zap, color: metrics.collisions > 0 ? 'var(--accent-red)' : 'var(--accent-green)' },
  ];

  return (
    <div className="page">
      <div className="page-header">
        <h2>System Overview</h2>
        <p>NEXFLEET EdgeFleet-AI — Real-time fleet coordination dashboard</p>
      </div>

      {/* Health indicator */}
      <div className="card mb-4" style={{ display:'flex', alignItems:'center', gap: 12 }}>
        <span className={`status-dot ${health?.status === 'ok' ? 'dot-green' : 'dot-red'}`} />
        <span style={{ fontSize: 13, fontWeight: 600 }}>
          Backend: {health?.status === 'ok' ? 'Connected' : 'Disconnected'}
        </span>
        <span style={{ color: 'var(--text-muted)', fontSize: 12 }}>|</span>
        <span style={{ fontSize: 12, color: 'var(--text-secondary)' }}>
          Risk Engine: <strong style={{ color: aiStatus?.risk_engine === 'GNN' ? 'var(--accent-green)' : 'var(--accent-yellow)' }}>{aiStatus?.risk_engine || '...'}</strong>
        </span>
        <span style={{ color: 'var(--text-muted)', fontSize: 12 }}>|</span>
        <span style={{ fontSize: 12, color: 'var(--text-secondary)' }}>
          Decision Engine: <strong style={{ color: aiStatus?.decision_engine === 'MARL' ? 'var(--accent-green)' : 'var(--accent-yellow)' }}>{aiStatus?.decision_engine || '...'}</strong>
        </span>
        <span style={{ color: 'var(--text-muted)', fontSize: 12 }}>|</span>
        <span style={{ fontSize: 12, color: 'var(--text-secondary)' }}>
          Transport: <strong style={{ color: 'var(--accent-cyan)' }}>{health?.transport || 'IN-PROCESS'}</strong>
        </span>
      </div>

      {/* KPI Grid */}
      <div className="grid-4 mb-4">
        {stats.map(({ label, value, icon: Icon, color }) => (
          <div key={label} className="stat-card">
            <div className="flex items-center justify-between mb-4">
              <div className="stat-label">{label}</div>
              <Icon size={16} style={{ color }} />
            </div>
            <div className="stat-value" style={{ color }}>{value}</div>
          </div>
        ))}
      </div>

      {/* Metrics */}
      <div className="grid-3 mb-4">
        <div className="card">
          <div className="card-title"><Activity size={14} />Throughput</div>
          <div className="stat-value">{((metrics.throughput_tasks_per_tick || 0) * 100).toFixed(2)}<span className="stat-unit">%/tick</span></div>
          <div className="stat-sub">Makespan: {metrics.makespan_ticks || 0} ticks</div>
        </div>
        <div className="card">
          <div className="card-title">Distance Travelled</div>
          <div className="stat-value">{(metrics.total_distance || 0).toFixed(0)}<span className="stat-unit">cells</span></div>
          <div className="stat-sub">Reroutes: {metrics.reroutes || 0}</div>
        </div>
        <div className="card">
          <div className="card-title">Safety</div>
          <div className="stat-value" style={{ color: (metrics.collisions || 0) === 0 ? 'var(--accent-green)' : 'var(--accent-red)' }}>
            {(metrics.collisions || 0) === 0 ? 'SAFE' : `${metrics.collisions} COL`}
          </div>
          <div className="stat-sub">Deadlocks: {metrics.deadlocks_detected || 0}</div>
        </div>
      </div>

      {/* Robot status table */}
      <div className="card">
        <div className="card-title"><Bot size={14} />Fleet Status</div>
        <table className="data-table">
          <thead>
            <tr>
              <th>Robot</th><th>Status</th><th>Position</th>
              <th>Battery</th><th>Task</th><th>Completed</th><th>Conflicts</th>
            </tr>
          </thead>
          <tbody>
            {robots.map(r => (
              <tr key={r.robot_id}>
                <td style={{ fontWeight: 700 }}>{r.robot_id}</td>
                <td>
                  <span className={`badge badge-${STATUS_COLOR[r.status] || 'gray'}`}>{r.status}</span>
                </td>
                <td>({r.row},{r.col})</td>
                <td>
                  <div className="flex items-center gap-2">
                    <div className="progress-bar" style={{ flex: 1 }}>
                      <div className="progress-fill"
                        style={{ width: `${r.battery}%`, background: r.battery > 50 ? 'var(--accent-green)' : r.battery > 20 ? 'var(--accent-yellow)' : 'var(--accent-red)' }}
                      />
                    </div>
                    <span style={{ color: 'var(--text-secondary)', minWidth: 32 }}>{r.battery?.toFixed(0)}%</span>
                  </div>
                </td>
                <td style={{ color: 'var(--text-muted)' }}>{r.current_task_id || '—'}</td>
                <td>{r.completed_tasks || 0}</td>
                <td>{r.conflict_count || 0}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  );
}
