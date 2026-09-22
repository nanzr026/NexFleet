import React, { useMemo } from 'react';

const CELL_SIZE = 20;
const ROBOT_COLORS = { 'AMR-01': '#2563eb', 'AMR-02': '#059669', 'AMR-03': '#ea580c' };

const CELL_COLORS = {
  EMPTY: null,
  WALL: '#111',
  RACK: '#222',
  PICKUP: '#059669',
  DELIVERY: '#d97706',
  CHARGING: '#2563eb',
  OBSTACLE: '#dc2626',
};

export default function WarehouseMap({ warehouse, robots = [], onCellClick }) {
  if (!warehouse) return <div style={{ color: 'var(--text-dim)' }}>No warehouse data</div>;

  const { rows, cols, grid, stations = {}, dynamic_obstacles = [] } = warehouse;
  const obstacleSet = new Set(dynamic_obstacles.map(o => `${o[0]},${o[1]}`));

  const svgWidth = cols * CELL_SIZE;
  const svgHeight = rows * CELL_SIZE;

  const cells = useMemo(() => {
    if (!grid) return [];
    const items = [];
    for (let r = 0; r < rows; r++) {
      for (let c = 0; c < cols; c++) {
        const ct = obstacleSet.has(`${r},${c}`) ? 'OBSTACLE' : (grid[r]?.[c] || 'EMPTY');
        const color = CELL_COLORS[ct];
        if (color) {
          items.push({ r, c, ct, color });
        }
      }
    }
    return items;
  }, [grid, rows, cols, obstacleSet]);

  return (
    <div className="warehouse-container">
      <svg
        viewBox={`0 0 ${svgWidth} ${svgHeight}`}
        style={{ width: '100%', height: '100%', maxHeight: '600px', cursor: onCellClick ? 'crosshair' : 'default' }}
        onClick={e => {
          if (!onCellClick) return;
          const rect = e.currentTarget.getBoundingClientRect();
          const sx = (e.clientX - rect.left) / rect.width * svgWidth;
          const sy = (e.clientY - rect.top) / rect.height * svgHeight;
          const c = Math.floor(sx / CELL_SIZE);
          const r = Math.floor(sy / CELL_SIZE);
          if (r >= 0 && r < rows && c >= 0 && c < cols) onCellClick(r, c);
        }}
      >
        <rect width={svgWidth} height={svgHeight} fill="#050505" />

        {/* Grid lines */}
        {Array.from({ length: rows + 1 }, (_, i) => (
          <line key={`hr${i}`} x1={0} y1={i * CELL_SIZE} x2={svgWidth} y2={i * CELL_SIZE} stroke="#1a1a1a" strokeWidth={1} />
        ))}
        {Array.from({ length: cols + 1 }, (_, i) => (
          <line key={`vc${i}`} x1={i * CELL_SIZE} y1={0} x2={i * CELL_SIZE} y2={svgHeight} stroke="#1a1a1a" strokeWidth={1} />
        ))}

        {/* Cells */}
        {cells.map(({ r, c, ct, color }) => (
          <rect key={`${r},${c}`} x={c * CELL_SIZE} y={r * CELL_SIZE} width={CELL_SIZE} height={CELL_SIZE} fill={color} />
        ))}

        {/* Stations */}
        {Object.entries(stations).map(([sid, st]) => (
          <text key={sid} x={st.col * CELL_SIZE + CELL_SIZE / 2} y={st.row * CELL_SIZE + CELL_SIZE / 2 + 4}
            textAnchor="middle" fontSize={10} fontWeight="bold" fill="white" fontFamily="var(--font-mono)">
            {sid}
          </text>
        ))}

        {/* Paths */}
        {robots.map(robot => {
          const color = ROBOT_COLORS[robot.robot_id] || '#fff';
          const path = robot.current_path || [];
          if (path.length < 2) return null;
          // Combine current pos + path
          const fullPath = [[robot.row, robot.col], ...path];
          const pts = fullPath.map(([r, c]) => `${c * CELL_SIZE + CELL_SIZE / 2},${r * CELL_SIZE + CELL_SIZE / 2}`).join(' ');
          
          return (
            <polyline key={`path-${robot.robot_id}`} points={pts} fill="none" stroke={color} strokeWidth={2} strokeDasharray="4,4" opacity={0.6} />
          );
        })}

        {/* Targets */}
        {robots.map(robot => {
          const color = ROBOT_COLORS[robot.robot_id] || '#fff';
          const path = robot.current_path || [];
          if (path.length > 0) {
            const [tr, tc] = path[path.length - 1];
            return (
              <circle key={`target-${robot.robot_id}`} cx={tc * CELL_SIZE + CELL_SIZE / 2} cy={tr * CELL_SIZE + CELL_SIZE / 2} r={6} fill="none" stroke={color} strokeWidth={2} />
            );
          }
          return null;
        })}

        {/* Robots */}
        {robots.map(robot => {
          const color = ROBOT_COLORS[robot.robot_id] || '#fff';
          const cx = robot.col * CELL_SIZE + CELL_SIZE / 2;
          const cy = robot.row * CELL_SIZE + CELL_SIZE / 2;
          const isFailed = robot.status === 'FAILED';
          return (
            <g key={robot.robot_id}>
              <rect x={cx - 8} y={cy - 8} width={16} height={16} fill={isFailed ? '#dc2626' : color} stroke="white" strokeWidth={1.5} opacity={isFailed ? 0.5 : 1} />
              <text x={cx} y={cy + 3} textAnchor="middle" fontSize={8} fill="white" fontWeight="bold" fontFamily="var(--font-mono)">
                {robot.robot_id.replace('AMR-', '')}
              </text>
            </g>
          );
        })}
      </svg>
    </div>
  );
}
