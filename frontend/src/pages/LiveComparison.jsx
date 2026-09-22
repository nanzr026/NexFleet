import React, { useState } from 'react';
import { api } from '../services/api';
import WarehouseMap from '../components/WarehouseMap';

export default function LiveComparison({ fleetState }) {
  const [newTask, setNewTask] = useState({ pickup_station: 'P1', delivery_station: 'D1' });
  const [obstacleMode, setObstacleMode] = useState(false);

  const traditional = fleetState?.traditional || {};
  const nexfleet = fleetState?.nexfleet || {};
  const comp = fleetState?.comparison || {};

  const handleStart = async () => await api.startSimulation({});
  const handlePause = async () => await api.pauseSimulation();
  const handleReset = async () => await api.resetSimulation({});
  
  const handleQueueTask = async () => {
    try { await api.createTask(newTask); } catch (e) { alert(e.message); }
  };
  
  const handleCellClick = async (r, c) => {
    if (!obstacleMode) return;
    try { await api.addObstacle({ row: r, col: c }); } catch {}
  };

  const MetricRow = ({ label, vT, vN, lowerBetter }) => {
    const isNum = typeof vT === 'number' && typeof vN === 'number';
    let better = null;
    if (isNum && vT !== vN) better = lowerBetter ? (vN < vT) : (vN > vT);
    return (
      <div className="metric-row">
        <span className="metric-label">{label}</span>
        <span className="metric-value">{typeof vT === 'number' ? vT.toFixed(1) : vT}</span>
        <span className="metric-value" style={{ color: better === true ? 'var(--accent-green)' : better === false ? 'var(--accent-red)' : 'inherit' }}>
          {typeof vN === 'number' ? vN.toFixed(1) : vN} {better === true ? (lowerBetter ? '▼' : '▲') : ''}
        </span>
      </div>
    );
  };

  return (
    <div className="ops-container">
      {/* HEADER & CONTROLS */}
      <div className="ops-header" style={{ flexDirection: 'column', gap: 12 }}>
        <div style={{ display: 'flex', justifyContent: 'space-between' }}>
          <div className="ops-title">NEXFLEET / LIVE COMPARISON</div>
          <div className="ops-status" style={{ color: comp.is_running ? 'var(--accent-green)' : 'var(--text-dim)' }}>
            {comp.is_running ? 'SIMULATION RUNNING' : 'SYSTEM IDLE'}
          </div>
        </div>
        
        <div className="ops-actions" style={{ padding: '8px 0', borderTop: '1px solid var(--border-dim)' }}>
          <div style={{ display:'flex', gap: 8, alignItems:'center' }}>
            <span style={{ fontSize:11, color:'var(--text-dim)', textTransform:'uppercase' }}>Queue Task:</span>
            <select value={newTask.pickup_station} onChange={e => setNewTask(t => ({...t, pickup_station: e.target.value}))} className="ctrl-select">
              {['P1','P2','P3','P4'].map(s => <option key={s} value={s}>{s}</option>)}
            </select>
            <span style={{color: '#666'}}>→</span>
            <select value={newTask.delivery_station} onChange={e => setNewTask(t => ({...t, delivery_station: e.target.value}))} className="ctrl-select">
              {['D1','D2','D3','D4'].map(s => <option key={s} value={s}>{s}</option>)}
            </select>
            <button className="ctrl-btn bg-blue" onClick={handleQueueTask}>+ QUEUE</button>
            <span style={{ fontSize:10, color:'var(--accent-blue)', marginLeft: 8 }}>{comp.queued_tasks} TASKS QUEUED</span>
          </div>

          <div className="divider"></div>
          
          <button className="ctrl-btn bg-green" onClick={handleStart} disabled={comp.is_running}>RUN COMPARISON</button>
          <button className="ctrl-btn bg-gray" onClick={handlePause} disabled={!comp.is_running}>PAUSE</button>
          <button className="ctrl-btn bg-gray" onClick={handleReset}>RESET SCENARIO</button>
          
          <div className="divider"></div>
          
          <button className={`ctrl-btn ${obstacleMode ? 'bg-orange' : 'bg-gray'}`} onClick={() => setObstacleMode(!obstacleMode)}>
            {obstacleMode ? 'CLICK MAP TO ADD OBSTACLE' : 'ADD OBSTACLE'}
          </button>
        </div>
      </div>

      {/* DUAL WAREHOUSE LAYOUT */}
      <div className="flex" style={{ flex: 1, overflow: 'hidden' }}>
        
        {/* LEFT: TRADITIONAL */}
        <div style={{ flex: 1, display: 'flex', flexDirection: 'column', borderRight: '1px solid var(--border-dim)' }}>
          <div style={{ padding: '8px 16px', background: 'var(--bg-panel)', borderBottom: '1px solid var(--border-dim)', fontWeight: 600, color: 'var(--text-dim)', textAlign: 'center', letterSpacing: '0.1em' }}>
            TRADITIONAL — STOP & WAIT
          </div>
          <div className="ops-workspace">
            <WarehouseMap warehouse={traditional.warehouse} robots={traditional.robots} onCellClick={obstacleMode ? handleCellClick : null} />
          </div>
          <div className="ops-amr-bar" style={{ minHeight: 120 }}>
            {(traditional.robots || []).map(r => (
              <div key={r.robot_id} className="amr-status-row" style={{ gridTemplateColumns: '60px 70px 80px 100px 1fr 60px' }}>
                <div className="amr-id" style={{color: r.robot_id === 'AMR-01' ? '#2563eb' : r.robot_id === 'AMR-02' ? '#059669' : '#ea580c'}}>{r.robot_id}</div>
                <div className={`amr-state state-${r.status.toLowerCase()}`}>{r.status}</div>
                <div className="amr-bat" style={{ color: r.battery > 50 ? 'var(--text-main)' : r.battery > 20 ? 'var(--accent-yellow)' : 'var(--accent-red)'}}>
                  BAT: {r.battery?.toFixed(0)}%
                </div>
                <div className="amr-task">T: {r.current_task_id || 'NONE'}</div>
                <div className="amr-target" style={{color: 'var(--text-dim)'}}>POS: ({r.row},{r.col})</div>
                <div className="amr-target" style={{color: 'var(--text-dim)'}}>W: {r.waiting_ticks}</div>
              </div>
            ))}
          </div>
        </div>

        {/* RIGHT: NEXFLEET */}
        <div style={{ flex: 1, display: 'flex', flexDirection: 'column' }}>
          <div style={{ padding: '8px 16px', background: 'var(--bg-panel)', borderBottom: '1px solid var(--border-dim)', fontWeight: 600, color: 'var(--accent-blue)', textAlign: 'center', letterSpacing: '0.1em' }}>
            NEXFLEET — DECENTRALIZED
          </div>
          <div className="ops-workspace">
            <WarehouseMap warehouse={nexfleet.warehouse} robots={nexfleet.robots} onCellClick={obstacleMode ? handleCellClick : null} />
          </div>
          <div className="ops-amr-bar" style={{ minHeight: 120 }}>
            {(nexfleet.robots || []).map(r => (
              <div key={r.robot_id} className="amr-status-row" style={{ gridTemplateColumns: '60px 70px 80px 100px 1fr 60px' }}>
                <div className="amr-id" style={{color: r.robot_id === 'AMR-01' ? '#2563eb' : r.robot_id === 'AMR-02' ? '#059669' : '#ea580c'}}>{r.robot_id}</div>
                <div className={`amr-state state-${r.status.toLowerCase()}`}>{r.status}</div>
                <div className="amr-bat" style={{ color: r.battery > 50 ? 'var(--text-main)' : r.battery > 20 ? 'var(--accent-yellow)' : 'var(--accent-red)'}}>
                  BAT: {r.battery?.toFixed(0)}%
                </div>
                <div className="amr-task">T: {r.current_task_id || 'NONE'}</div>
                <div className="amr-target" style={{color: 'var(--text-dim)'}}>POS: ({r.row},{r.col})</div>
                <div className="amr-target" style={{color: 'var(--text-dim)'}}>W: {r.waiting_ticks}</div>
              </div>
            ))}
          </div>
        </div>

      </div>

      {/* METRICS & TARGET PANEL */}
      <div style={{ display: 'flex', borderTop: '1px solid var(--border-dim)', background: 'var(--bg-panel)' }}>
        
        {/* KPI Panel */}
        <div style={{ flex: 1, padding: 24, borderRight: '1px solid var(--border-dim)', display: 'flex', flexDirection: 'column', justifyContent: 'center', alignItems: 'center' }}>
          <div style={{ fontSize: 11, color: 'var(--text-dim)', letterSpacing: '0.1em', marginBottom: 12 }}>TASK COMPLETION TIME REDUCTION</div>
          <div style={{ fontSize: 48, fontWeight: 800, fontFamily: 'var(--font-mono)', color: comp.target_achieved ? 'var(--accent-green)' : (comp.completion_time_reduction_pct > 0 ? 'var(--accent-blue)' : 'var(--accent-yellow)') }}>
            {comp.completion_time_reduction_pct > 0 ? '↓ ' : (comp.completion_time_reduction_pct < 0 ? '↑ ' : '')}
            {Math.abs(comp.completion_time_reduction_pct || 0).toFixed(1)}%
          </div>
          <div style={{ marginTop: 12, padding: '4px 12px', border: `1px solid ${comp.target_achieved ? 'var(--accent-green)' : 'var(--accent-yellow)'}`, color: comp.target_achieved ? 'var(--accent-green)' : 'var(--accent-yellow)', fontSize: 11, fontWeight: 600, letterSpacing: '0.05em' }}>
            {comp.target_achieved ? 'TARGET ACHIEVED (≥20% & 0 COLLISIONS)' : 'TARGET NOT YET ACHIEVED'}
          </div>
        </div>

        {/* Live Metrics Table */}
        <div style={{ flex: 2, padding: 16 }}>
          <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 24 }}>
            <div>
              <div style={{ fontSize: 10, color: 'var(--text-dim)', borderBottom: '1px solid var(--border-dim)', paddingBottom: 6, marginBottom: 8, display: 'flex', justifyContent: 'space-between' }}>
                <span>EFFICIENCY METRICS</span>
                <span>TRADITIONAL &nbsp;|&nbsp; <span style={{color: 'var(--accent-blue)'}}>NEXFLEET</span></span>
              </div>
              <MetricRow label="Makespan (ticks)" vT={traditional.metrics?.makespan_ticks} vN={nexfleet.metrics?.makespan_ticks} lowerBetter={true} />
              <MetricRow label="Tasks Completed" vT={`${traditional.metrics?.completed_tasks || 0}/${traditional.metrics?.total_tasks || 0}`} vN={`${nexfleet.metrics?.completed_tasks || 0}/${nexfleet.metrics?.total_tasks || 0}`} />
              <MetricRow label="Total Wait Ticks" vT={traditional.metrics?.total_waiting_ticks} vN={nexfleet.metrics?.total_waiting_ticks} lowerBetter={true} />
              <MetricRow label="Total Distance" vT={traditional.metrics?.total_distance} vN={nexfleet.metrics?.total_distance} lowerBetter={true} />
            </div>
            <div>
              <div style={{ fontSize: 10, color: 'var(--text-dim)', borderBottom: '1px solid var(--border-dim)', paddingBottom: 6, marginBottom: 8, display: 'flex', justifyContent: 'space-between' }}>
                <span>SAFETY METRICS</span>
                <span>TRADITIONAL &nbsp;|&nbsp; <span style={{color: 'var(--accent-blue)'}}>NEXFLEET</span></span>
              </div>
              <MetricRow label="Collisions" vT={traditional.metrics?.collisions} vN={nexfleet.metrics?.collisions} lowerBetter={true} />
              <MetricRow label="Conflicts Detected" vT={traditional.metrics?.conflicts_detected} vN={nexfleet.metrics?.conflicts_detected} lowerBetter={true} />
              <MetricRow label="Deadlocks" vT={traditional.metrics?.deadlocks_detected} vN={nexfleet.metrics?.deadlocks_detected} lowerBetter={true} />
              <MetricRow label="Dynamic Reroutes" vT={traditional.metrics?.reroutes} vN={nexfleet.metrics?.reroutes} lowerBetter={true} />
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
