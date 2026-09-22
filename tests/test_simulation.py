import unittest
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from backend.simulation.warehouse import Warehouse
from backend.simulation.amr_agent import AMRAgent

class TestWarehouse(unittest.TestCase):
    def setUp(self):
        self.warehouse = Warehouse(rows=30, cols=30)
        
    def test_grid_initialization(self):
        self.assertEqual(len(self.warehouse.grid), 30)
        self.assertEqual(len(self.warehouse.grid[0]), 30)
        
    def test_station_creation(self):
        st = self.warehouse.get_station('P1')
        self.assertIsNotNone(st)
        self.assertEqual(st.station_id, 'P1')
        self.assertEqual(st.cell_type.value, 'PICKUP')

class TestAMRAgent(unittest.TestCase):
    def test_agent_initialization(self):
        agent = AMRAgent("AMR-99", 5, 5)
        self.assertEqual(agent.state.robot_id, "AMR-99")
        self.assertEqual(agent.state.row, 5)
        self.assertEqual(agent.state.col, 5)
        self.assertEqual(agent.state.status, "IDLE")
        self.assertEqual(agent.state.battery, 100.0)

if __name__ == '__main__':
    unittest.main()
