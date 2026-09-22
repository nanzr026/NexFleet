import React from 'react';
import { Bot } from 'lucide-react';

const STATUS_COLOR = { IDLE:'gray', MOVING:'blue', WAITING:'yellow', REROUTING:'purple', BLOCKED:'red', CHARGING:'cyan', FAILED:'red' };
const ROBOT_COLORS = { 'AMR-01': '#3b82f6', 'AMR-02': '#10b981', 'AMR-03': '#f59e0b' };

export default function Fleet({ fleetState }) {
  const robots = fleetState?.robots || [];
  return (
    <div className="page">
      <div className="page-header"><h2>Fleet Telemetry</h2><p>Individual AMR agent status and metrics</p></div>
      <div className="grid-3">
        {robots.map(r => (
          <div key={r.robot_id} className="card">
            <div className="flex justify-between items-center mb-4">
              <div style={{ display:'flex', alignItems:'center', gap:10 }}>
                <div style={{ width:14, height:14, borderRadius:'50%', background: ROBOT_COLORS[r.robot_id] || '#fff' }} />
                <span style={{ fontWeight:800, fontSize:18 }}>{r.robot_id}</span>
              </div>
              <span className={`badge badge-${STATUS_COLOR[r.status] || 'gray'}`}>{r.status}</span>
            </div>
            <div className="flex flex-col gap-1">
              {[
                ['Position (R,C)', `(${r.row}, ${r.col})`],
                ['Heading', r.direction],
                ['Velocity', r.velocity],
                ['Battery Level', `${r.battery?.toFixed(1)}%`],
                ['Assigned Task', r.current_task_id || 'None'],
                ['Completed Tasks', r.completed_tasks],
                ['Path Conflicts', r.conflict_count],
                ['Reroutes', r.reroute_count],
                ['Congestion Wait Ticks', r.waiting_ticks],
                ['Distance Travelled', r.distance_travelled?.toFixed(0) + ' cells'],
              ].map(([k, v]) => (
                <div key={k} className="metric-row">
                  <span className="metric-label">{k}</span>
                  <span className="metric-value">{v}</span>
                </div>
              ))}
            </div>
          </div>
        ))}
      </div>
    </div>
  );
}
