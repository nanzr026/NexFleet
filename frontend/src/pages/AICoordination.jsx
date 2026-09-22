import React, { useEffect, useState } from 'react';
import { Brain, Shield, Cpu, Activity } from 'lucide-react';
import { api } from '../services/api';

const RISK_COLORS = { LOW: 'var(--accent-green)', MEDIUM: 'var(--accent-yellow)', HIGH: 'var(--accent-red)' };
const ROBOT_COLORS = { 'AMR-01': '#3b82f6', 'AMR-02': '#10b981', 'AMR-03': '#f59e0b' };

export default function AICoordination({ fleetState }) {
  const [graphData, setGraphData] = useState(null);
  const [aiStatus, setAiStatus] = useState(null);

  useEffect(() => {
    const load = () => {
      api.getInteractionGraph().then(setGraphData).catch(() => {});
      api.getAIStatus().then(setAiStatus).catch(() => {});
    };
    load();
    const t = setInterval(load, 2000); // Polling for the graph UI since it requires heavier backend compute
    return () => clearInterval(t);
  }, []);

  const robots = fleetState?.robots || [];
  const events = (fleetState?.events || []).filter(e => e.type === 'RISK_ASSESSMENT').slice(-10).reverse();

  // SVG interaction graph dimensions
  const svgW = 320, svgH = 200;
  const robotPos = { 'AMR-01': [80, 100], 'AMR-02': [160, 40], 'AMR-03': [240, 100] };
  const pairs = graphData?.pairs || [];

  return (
    <div className="page">
      <div className="page-header">
        <h2>AI Coordination</h2>
        <p>Graph Neural Network risk prediction • MARL decision engine • Safety verification</p>
      </div>

      {/* Engine status */}
      <div className="grid-4 mb-4">
        {[
          { label: 'Risk Engine', value: aiStatus?.risk_engine || '...', good: aiStatus?.risk_engine === 'GNN', icon: Brain },
          { label: 'Decision Engine', value: aiStatus?.decision_engine || '...', good: aiStatus?.decision_engine === 'MARL', icon: Cpu },
          { label: 'LaCAM+PIBT', value: aiStatus?.lacam_pibt?.lacam_pibt_status || '...', good: false, icon: Shield },
          { label: 'Safety Layer', value: 'ACTIVE', good: true, icon: Activity },
        ].map(({ label, value, good, icon: Icon }) => (
          <div key={label} className="card">
            <div className="card-title"><Icon size={14} />{label}</div>
            <span className={`badge badge-${good ? 'green' : 'yellow'}`}>{value}</span>
          </div>
        ))}
      </div>

      <div className="grid-2">
        {/* Interaction Graph SVG */}
        <div className="card">
          <div className="card-title"><Brain size={14} />Interaction Graph (Live)</div>
          <svg viewBox={`0 0 ${svgW} ${svgH}`} style={{ width:'100%', background:'var(--bg-secondary)', borderRadius:8 }}>
            {/* Edges */}
            {pairs.map(({ robot_a, robot_b, risk, risk_level }) => {
              const [x1,y1] = robotPos[robot_a] || [0,0];
              const [x2,y2] = robotPos[robot_b] || [0,0];
              const col = RISK_COLORS[risk_level] || 'var(--text-muted)';
              return (
                <line key={`${robot_a}-${robot_b}`}
                  x1={x1} y1={y1} x2={x2} y2={y2}
                  stroke={col} strokeWidth={1.5 + risk * 4}
                  opacity={0.7} strokeDasharray="4,3"
                />
              );
            })}
            {/* Risk labels on edges */}
            {pairs.map(({ robot_a, robot_b, risk }) => {
              const [x1,y1] = robotPos[robot_a] || [0,0];
              const [x2,y2] = robotPos[robot_b] || [0,0];
              return (
                <text key={`label-${robot_a}-${robot_b}`}
                  x={(x1+x2)/2} y={(y1+y2)/2 - 8}
                  textAnchor="middle" fontSize={10} fill="var(--text-primary)" fontWeight="bold">
                  {risk.toFixed(3)}
                </text>
              );
            })}
            {/* Nodes */}
            {['AMR-01', 'AMR-02', 'AMR-03'].map(r => {
              const [cx,cy] = robotPos[r] || [0,0];
              const col = ROBOT_COLORS[r] || '#fff';
              const agentState = robots.find(a => a.robot_id === r);
              return (
                <g key={r}>
                  <circle cx={cx} cy={cy} r={22} fill={col} opacity={0.15} />
                  <circle cx={cx} cy={cy} r={16} fill={col} opacity={0.9} />
                  <text x={cx} y={cy+3} textAnchor="middle" fontSize={9} fill="white" fontWeight="bold">
                    {r.replace('AMR-','')}
                  </text>
                  <text x={cx} y={cy+28} textAnchor="middle" fontSize={8} fill={col}>
                    {agentState?.status || 'UNKNOWN'}
                  </text>
                </g>
              );
            })}
          </svg>
          {/* Pair risk table */}
          <table className="data-table" style={{ marginTop:12 }}>
            <thead><tr><th>Pair</th><th>Risk Prediction</th><th>Level</th></tr></thead>
            <tbody>
              {pairs.map(({ robot_a, robot_b, risk, risk_level }) => {
                return (
                  <tr key={`${robot_a}-${robot_b}`}>
                    <td>{robot_a} ↔ {robot_b}</td>
                    <td style={{ color: RISK_COLORS[risk_level] }}>{risk.toFixed(4)}</td>
                    <td><span className={`badge badge-${risk_level==='LOW'?'green':risk_level==='MEDIUM'?'yellow':'red'}`}>{risk_level}</span></td>
                  </tr>
                );
              })}
              {pairs.length === 0 && <tr><td colSpan={3} style={{ textAlign: 'center', color: 'var(--text-muted)' }}>Waiting for active simulation...</td></tr>}
            </tbody>
          </table>
        </div>

        {/* Safety Layer */}
        <div className="flex flex-col gap-3">
          <div className="card">
            <div className="card-title"><Shield size={14} />Safety Layer Modules</div>
            <div style={{ fontSize:12 }}>
              {[
                { label: 'A* Path Planning', status: 'ACTIVE', color: 'green' },
                { label: 'Reservation Table', status: 'ACTIVE', color: 'green' },
                { label: 'Conflict Detector', status: 'ACTIVE', color: 'green' },
                { label: 'Deadlock Resolver', status: 'ACTIVE', color: 'green' },
                { label: 'LaCAM+PIBT', status: aiStatus?.lacam_pibt?.lacam_pibt_status || 'PENDING', color: 'yellow' },
              ].map(({ label, status, color }) => (
                <div key={label} className="flex justify-between items-center" style={{ padding:'6px 0', borderBottom:'1px solid var(--border)' }}>
                  <span style={{ color:'var(--text-secondary)' }}>{label}</span>
                  <span className={`badge badge-${color}`}>{status}</span>
                </div>
              ))}
            </div>
          </div>
          <div className="card">
            <div className="card-title">Recent Risk & MARL Events</div>
            <div className="event-feed">
              {events.map((ev, i) => (
                <div key={i} className="event-item">
                  <span className="event-time">t={ev.tick}</span>
                  <span className="event-type" style={{ color: ROBOT_COLORS[ev.robot_id] }}>{ev.robot_id}</span>
                  <span className="event-detail">Risk: {ev.risk?.toFixed(3)} → Action: <strong>{ev.action}</strong></span>
                </div>
              ))}
              {events.length === 0 && <div style={{ color:'var(--text-muted)', fontSize:11 }}>No risk assessment events yet. Start simulation.</div>}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
