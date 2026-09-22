import React, { useState } from 'react';
import Sidebar from './components/Sidebar';
import { useWebSocket } from './hooks/useWebSocket';
import Overview from './pages/Overview';
import LiveComparison from './pages/LiveComparison';
import Fleet from './pages/Fleet';
import Tasks from './pages/Tasks';
import AICoordination from './pages/AICoordination';
import P2PNetwork from './pages/Network';
import FederatedLearning from './pages/FederatedLearning';
import Benchmark from './pages/Benchmark';
import Architecture from './pages/Architecture';

const PAGES = {
  overview: Overview,
  comparison: LiveComparison,
  fleet: Fleet,
  tasks: Tasks,
  ai: AICoordination,
  network: P2PNetwork,
  federated: FederatedLearning,
  benchmark: Benchmark,
  architecture: Architecture,
};

export default function App() {
  const [activePage, setActivePage] = useState('overview');
  const { connected, fleetState } = useWebSocket();
  const PageComponent = PAGES[activePage] || Overview;

  // Since payload is dual, default single-page components to nexfleet data if needed.
  // We'll pass the whole dual payload down, components handle it or we proxy.
  const proxyState = fleetState?.nexfleet || fleetState || {};
  // but keep fleetState for comparison which needs both.

  return (
    <div className="app-layout">
      <Sidebar activePage={activePage} onNavigate={setActivePage} />
      <div className="main-content">
        <PageComponent fleetState={activePage === 'comparison' ? fleetState : proxyState} />
      </div>
      <div className="connection-bar">
        <span className={`status-dot ${connected ? 'dot-green' : 'dot-red'}`} />
        <span>WebSocket: {connected ? 'CONNECTED' : 'DISCONNECTED'}</span>
        <span style={{ color:'var(--text-dim)' }}>|</span>
        <span>Tick: {fleetState?.nexfleet?.tick || 0}</span>
        <span style={{ color:'var(--text-dim)' }}>|</span>
        <span>State: {(fleetState?.nexfleet?.state || 'STOPPED').toUpperCase()}</span>
      </div>
    </div>
  );
}
