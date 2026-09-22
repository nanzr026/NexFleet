import React, { useState, useEffect } from 'react';
import { ClipboardList, Award } from 'lucide-react';
import { api } from '../services/api';

const STATUS_COLOR = { PENDING:'gray', ASSIGNED:'blue', TO_PICKUP:'cyan', TO_DELIVERY:'yellow', COMPLETED:'green', REASSIGNING:'purple', FAILED:'red' };

export default function Tasks({ fleetState }) {
  const [auction, setAuction] = useState(null);
  
  // Only refresh auction if a task is actually assigned this tick to avoid excessive polling
  useEffect(() => { 
    api.getAuction().then(setAuction).catch(() => {}); 
  }, [fleetState?.tick]);
  
  const tasks = fleetState?.tasks || [];
  
  return (
    <div className="page">
      <div className="page-header"><h2>Task Management</h2><p>Task lifecycle and Contract-Net distributed auction results</p></div>
      
      {auction && auction.task_id && (
        <div className="card mb-4" style={{ borderColor: 'var(--accent-blue)' }}>
          <div className="card-title" style={{ color: 'var(--accent-blue)' }}><Award size={14} />Latest Distributed Auction</div>
          <div style={{ marginBottom: 16, fontSize: 13 }}>
            <strong>Task ID:</strong> {auction.task_id} &nbsp;|&nbsp; 
            <strong>Winner:</strong> <span style={{ color: 'var(--accent-green)', fontWeight: 'bold' }}>{auction.winner}</span>
          </div>
          <div className="grid-3">
            {(auction.bids || []).map(b => (
              <div key={b.robot_id} className="stat-card" style={{ borderColor: b.robot_id === auction.winner ? 'var(--accent-green)' : 'var(--border)', background: b.robot_id === auction.winner ? 'rgba(16,185,129,0.05)' : 'transparent' }}>
                <div style={{ fontWeight:800, marginBottom:8, fontSize: 14 }}>
                  {b.robot_id} {b.robot_id === auction.winner && <span style={{ color: 'var(--accent-green)' }}>WINNER ✓</span>}
                </div>
                <div className="metric-row"><span className="metric-label">Computed Bid</span><span className="metric-value">{b.bid_value?.toFixed(2)}</span></div>
                <div className="metric-row"><span className="metric-label">Dist to Pickup</span><span className="metric-value">{b.distance_to_pickup?.toFixed(1)}</span></div>
                <div className="metric-row"><span className="metric-label">Battery Level</span><span className="metric-value">{b.battery?.toFixed(0)}%</span></div>
              </div>
            ))}
          </div>
        </div>
      )}
      
      <div className="card">
        <div className="card-title"><ClipboardList size={14} />All Tasks</div>
        <table className="data-table">
          <thead><tr><th>Task ID</th><th>Pickup</th><th>Delivery</th><th>Priority</th><th>Status</th><th>Robot</th><th>Completion Tick</th></tr></thead>
          <tbody>
            {tasks.map(t => (
              <tr key={t.task_id}>
                <td>{t.task_id}</td>
                <td>{t.pickup_station || `(${t.pickup[0]},${t.pickup[1]})`}</td>
                <td>{t.delivery_station || `(${t.delivery[0]},${t.delivery[1]})`}</td>
                <td>{t.priority}</td>
                <td><span className={`badge badge-${STATUS_COLOR[t.status] || 'gray'}`}>{t.status}</span></td>
                <td style={{ fontWeight: 'bold', color: t.assigned_robot ? 'var(--accent-blue)' : 'inherit' }}>{t.assigned_robot || '—'}</td>
                <td>{t.completion_time ? t.completion_time : '—'}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  );
}
