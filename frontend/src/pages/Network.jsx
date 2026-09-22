import React, { useEffect, useState } from 'react';
import { Network } from 'lucide-react';
import { api } from '../services/api';

const PROFILES = ['NORMAL', 'HIGH_LATENCY', 'PACKET_LOSS', 'DEAD_ZONE'];

export default function P2PNetwork() {
  const [network, setNetwork] = useState(null);
  const [profile, setProfile] = useState('NORMAL');

  const load = () => api.getNetwork().then(setNetwork).catch(() => {});
  useEffect(() => { load(); const t = setInterval(load, 2000); return () => clearInterval(t); }, []);

  const handleProfileChange = async (p) => {
    setProfile(p);
    const cfg = { profile: p, latency_ms: 0, packet_loss_rate: 0 };
    if (p === 'HIGH_LATENCY') cfg.latency_ms = 500;
    if (p === 'PACKET_LOSS') cfg.packet_loss_rate = 0.2;
    if (p === 'DEAD_ZONE') cfg.packet_loss_rate = 1.0;
    
    await api.setNetworkConfig(cfg).catch(() => {});
    load();
  };

  const robots = ['AMR-01', 'AMR-02', 'AMR-03'];
  const ROBOT_COLORS = { 'AMR-01': '#3b82f6', 'AMR-02': '#10b981', 'AMR-03': '#f59e0b' };
  const peers = [['AMR-01','AMR-02'],['AMR-02','AMR-03'],['AMR-01','AMR-03']];

  return (
    <div className="page">
      <div className="page-header">
        <h2>P2P Network Diagnostics</h2>
        <p>Peer-to-peer DDS/Transport communication status and impairment simulation</p>
      </div>
      
      <div className="card mb-4">
        <div className="card-title">Network Impairment Profile</div>
        <div className="flex gap-2">
          {PROFILES.map(p => (
            <button key={p} className={`btn ${profile===p?'btn-primary':'btn-ghost'}`} onClick={() => handleProfileChange(p)} style={{ minWidth: 120 }}>{p.replace('_', ' ')}</button>
          ))}
        </div>
      </div>
      
      <div className="grid-2">
        <div className="card">
          <div className="card-title"><Network size={14} />Peer Topology</div>
          <svg viewBox="0 0 300 200" style={{ width:'100%', background:'var(--bg-secondary)', borderRadius:8 }}>
            {peers.map(([a,b]) => {
              const pos = {'AMR-01':[60,100],'AMR-02':[150,40],'AMR-03':[240,100]};
              const [x1,y1]=pos[a],[x2,y2]=pos[b];
              const isDisconnected = profile === 'DEAD_ZONE';
              return (
                <line key={`${a}-${b}`} x1={x1} y1={y1} x2={x2} y2={y2} 
                  stroke={isDisconnected ? "var(--accent-red)" : "var(--accent-blue)"} 
                  strokeWidth={2} strokeDasharray="4,3" 
                  className={isDisconnected ? "" : "topology-edge"} opacity={isDisconnected ? 0.3 : 0.8} />
              );
            })}
            {robots.map(r => {
              const pos = {'AMR-01':[60,100],'AMR-02':[150,40],'AMR-03':[240,100]};
              const [cx,cy]=pos[r];
              return (
                <g key={r}>
                  <circle cx={cx} cy={cy} r={20} fill={ROBOT_COLORS[r]} opacity={0.2} />
                  <circle cx={cx} cy={cy} r={14} fill={ROBOT_COLORS[r]} opacity={0.9} />
                  <text x={cx} y={cy+3} textAnchor="middle" fontSize={8} fill="white" fontWeight="bold">{r.replace('AMR-','')}</text>
                </g>
              );
            })}
          </svg>
        </div>
        
        <div className="card">
          <div className="card-title">Transport Statistics</div>
          <div className="flex flex-col gap-1">
            {network && Object.entries(network).map(([k,v]) => (
              <div key={k} className="metric-row">
                <span className="metric-label">{k.replace(/_/g,' ').toUpperCase()}</span>
                <span className="metric-value">{typeof v === 'number' ? v.toFixed ? v.toFixed(2) : v : String(v)}</span>
              </div>
            ))}
            {!network && <div style={{ color: 'var(--text-muted)', fontSize: 12 }}>Loading transport stats...</div>}
          </div>
        </div>
      </div>
    </div>
  );
}
