import React, { useState } from 'react';
import { api } from '../services/api';
import WarehouseMap from '../components/WarehouseMap';

export default function LiveOperations({ fleetState }) {
  const [loading, setLoading] = useState(false);
  const [newTask, setNewTask] = useState({ pickup_station: 'P1', delivery_station: 'D1', priority: 1 });
  const [obstacleMode, setObstacleMode] = useState(false);

  const handleStart = async () => {
    setLoading(true);
    try { await api.startSimulation({ mode: fleetState?.mode || 'nexfleet', seed: 42, tick_interval: 0.15 }); }
    catch (e) { alert(e.message); } finally { setLoading(false); }
  };
  const handlePause = async () => { try { await api.pauseSimulation(); } catch {} };
  const handleReset = async () => { try { await api.resetSimulation({ mode: fleetState?.mode || 'nexfleet', seed: 42 }); } catch {} };
  const handleCreateTask = async () => {
    try { await api.createTask(newTask); } catch (e) { alert(e.message); }
  };
  const handleCellClick = async (r, c) => {
    if (!obstacleMode) return;
    try { await api.addObstacle({ row: r, col: c }); } catch {}
  };
  const setMode = async (m) => {
    try { await api.resetSimulation({ mode: m, seed: 42 }); } catch {}
  };

  const state = fleetState?.state || 'stopped';
  const mode = fleetState?.mode || 'nexfleet';
  const robots = fleetState?.robots || [];
  const events = fleetState?.events || [];

  return (
    <div className="ops-container">
      {/* HEADER SECTION */}
      <div className="ops-header">
        <div className="ops-title">NEXFLEET / LIVE OPERATIONS</div>
        <div className="ops-status">SYSTEM {state.toUpperCase()}</div>
      </div>

      {/* CONTROL BAR */}
      <div className="ops-controls">
        <div className="segmented-control">
          <button 
            className={`seg-btn ${mode === 'traditional' ? 'active' : ''}`}
            onClick={() => setMode('traditional')}
          >
            TRADITIONAL — STOP & WAIT
          </button>
          <button 
            className={`seg-btn ${mode === 'nexfleet' ? 'active' : ''}`}
            onClick={() => setMode('nexfleet')}
          >
            NEXFLEET — DECENTRALIZED
          </button>
        </div>
        
        <div className="ops-actions">
          <button className="ctrl-btn bg-green" onClick={handleStart} disabled={loading || state === 'running'}>START</button>
          <button className="ctrl-btn bg-gray" onClick={handlePause}>PAUSE</button>
          <button className="ctrl-btn bg-gray" onClick={handleReset}>RESET</button>
          
          <div className="divider"></div>
          
          <select value={newTask.pickup_station} onChange={e => setNewTask(t => ({...t, pickup_station: e.target.value}))} className="ctrl-select">
            {['P1','P2','P3','P4'].map(s => <option key={s} value={s}>{s}</option>)}
          </select>
          <span style={{color: '#666'}}>→</span>
          <select value={newTask.delivery_station} onChange={e => setNewTask(t => ({...t, delivery_station: e.target.value}))} className="ctrl-select">
            {['D1','D2','D3','D4'].map(s => <option key={s} value={s}>{s}</option>)}
          </select>
          <button className="ctrl-btn bg-blue" onClick={handleCreateTask}>CREATE TASK</button>
          
          <div className="divider"></div>
          
          <button className={`ctrl-btn ${obstacleMode ? 'bg-orange' : 'bg-gray'}`} onClick={() => setObstacleMode(!obstacleMode)}>
            {obstacleMode ? 'CLICK MAP TO ADD OBSTACLE' : 'ADD OBSTACLE'}
          </button>
        </div>
      </div>

      {/* WAREHOUSE MAP (Dominant Workspace) */}
      <div className="ops-workspace">
        <WarehouseMap warehouse={fleetState?.warehouse} robots={robots} onCellClick={obstacleMode ? handleCellClick : null} />
      </div>

      {/* AMR STATUS BAR */}
      <div className="ops-amr-bar">
        {robots.map(r => (
          <div key={r.robot_id} className="amr-status-row">
            <div className="amr-id" style={{color: r.robot_id === 'AMR-01' ? '#3b82f6' : r.robot_id === 'AMR-02' ? '#10b981' : '#f59e0b'}}>{r.robot_id}</div>
            <div className={`amr-state state-${r.status.toLowerCase()}`}>{r.status}</div>
            <div className="amr-bat">BAT: {r.battery?.toFixed(0)}%</div>
            <div className="amr-task">TASK: {r.current_task_id || 'NONE'}</div>
            <div className="amr-target">TARGET: {r.current_task_id ? `(${r.current_path?.[r.current_path.length-1]?.[0] || '?'}, ${r.current_path?.[r.current_path.length-1]?.[1] || '?'})` : '—'}</div>
            <div className="amr-fail-ctrl">
              <button 
                className={`ctrl-btn ${r.status === 'FAILED' ? 'bg-green' : 'bg-red'}`} 
                onClick={() => r.status === 'FAILED' ? api.recoverRobot(r.robot_id) : api.failRobot(r.robot_id)}
              >
                {r.status === 'FAILED' ? 'RECOVER' : 'FAIL ROBOT'}
              </button>
            </div>
          </div>
        ))}
      </div>

      {/* EVENT STREAM */}
      <div className="ops-events">
        <div className="events-title">EVENT STREAM / SYSTEM STATUS</div>
        <div className="events-list">
          {[...events].reverse().slice(0, 15).map((ev, i) => (
            <div key={i} className="ev-row">
              <span className="ev-tick">[{ev.tick?.toString().padStart(4, '0')}]</span>
              <span className="ev-type">{ev.type}</span>
              <span className="ev-detail">{JSON.stringify(ev)}</span>
            </div>
          ))}
        </div>
      </div>
    </div>
  );
}
