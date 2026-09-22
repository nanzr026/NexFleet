import React from 'react';
import {
  LayoutDashboard, SplitSquareHorizontal, Bot, ClipboardList,
  Brain, Network, Share2, BarChart3, GitBranch
} from 'lucide-react';

const navItems = [
  { id: 'overview', label: 'Overview', icon: LayoutDashboard },
  { id: 'comparison', label: 'Live Comparison', icon: SplitSquareHorizontal },
  { id: 'fleet', label: 'Fleet Analysis', icon: Bot },
  { id: 'tasks', label: 'Task Analysis', icon: ClipboardList },
  { id: 'ai', label: 'AI Coordination', icon: Brain },
  { id: 'federated', label: 'Federated Learning', icon: Share2 },
  { id: 'network', label: 'P2P Network', icon: Network },
  { id: 'architecture', label: 'Architecture', icon: GitBranch },
];

export default function Sidebar({ activePage, onNavigate }) {
  return (
    <nav className="sidebar">
      <div className="sidebar-logo">
        <h1>NEXFLEET</h1>
        <div className="tagline">EdgeFleet-AI • SIH 2025</div>
      </div>
      <div className="sidebar-nav">
        {navItems.map(({ id, label, icon: Icon }) => (
          <div
            key={id}
            className={`nav-item${activePage === id ? ' active' : ''}`}
            onClick={() => onNavigate(id)}
          >
            <Icon size={16} />
            <span>{label}</span>
          </div>
        ))}
      </div>
    </nav>
  );
}
