"""Regression tests for geometry and physical-workflow safety gates."""
import math
from unittest.mock import Mock

from bel_amr_ros.astar_planner import AStarPlanner
from bel_amr_ros.warehouse_map import WarehouseMap
from bel_amr_ros.pickup_geometry import choose_approach, docking_distance


def test_nearby_pallet_does_not_send_robot_away():
    planner = AStarPlanner(WarehouseMap())
    robot = (13.92, -8.0)
    point, path = choose_approach(planner, robot, (13.02, -8.0))
    assert point == robot
    assert path == [robot]


def test_reachable_approach_from_normal_spawn():
    planner = AStarPlanner(WarehouseMap())
    point, path = choose_approach(planner, (13.9, -8), (4.4161, 14.6952))
    assert path[0] == (13.9, -8)
    assert path[-1] == point
    assert all(planner.has_line_of_sight(a, b) for a, b in zip(path, path[1:]))


def test_docking_geometry_accounts_for_pallet_orientation():
    assert docking_distance(0) > docking_distance(math.pi/2)
    assert math.isclose(docking_distance(0, math.pi/2), docking_distance(math.pi/2))


def test_dynamic_obstacles_expire_without_erasing_static_map():
    warehouse = WarehouseMap()
    cell = warehouse.world_to_grid(10, -8)
    assert not warehouse.is_occupied_grid(*cell)
    warehouse.add_dynamic_obstacle(10, -8)
    count = len(warehouse.dynamic_cells)
    warehouse.add_dynamic_obstacle(10, -8)
    assert len(warehouse.dynamic_cells) == count
    assert warehouse.is_occupied_grid(*cell)
    warehouse.dynamic_cells = {c: 0 for c in warehouse.dynamic_cells}
    assert not warehouse.is_occupied_grid(*cell)
    assert warehouse.is_occupied_grid(*warehouse.world_to_grid(-4.9, -0.69))


def test_unacknowledged_pickup_cannot_start_delivery():
    from bel_amr_ros.robot_controller import AMRController
    controller = Mock()
    controller.robot_state = 'PICKING'
    controller.attachment_state = 'detached'
    controller.attachment_time = 0
    controller.action_started = 10
    controller.last_attachment_command = 0
    AMRController.attachment_step(controller, 12)
    controller.attach_pub.publish.assert_called_once()
    controller.set_route.assert_not_called()
    AMRController.attachment_step(controller, 21)
    controller.fail_task.assert_called_once()


def test_stale_pose_stops_robot():
    from bel_amr_ros.robot_controller import AMRController
    controller = Mock()
    controller.pose_received = True
    controller.pose_time = 0
    AMRController.control_loop(controller)
    controller.stop_robot.assert_called_once()
    controller.move_to_goal.assert_not_called()


def test_same_cell_los_rejects_occupied_and_out_of_bounds():
    planner = AStarPlanner(WarehouseMap())
    assert not planner.has_line_of_sight((-4.9, -0.69), (-4.9, -0.69))
    assert not planner.has_line_of_sight((100,100), (100,100))


def test_same_cell_route_preserves_both_exact_endpoints():
    planner = AStarPlanner(WarehouseMap())
    start, goal = (10.01,-7.99), (10.09,-7.91)
    assert planner.plan(start, goal) == [start, goal]


def test_relative_attachment_check_handles_rotation_and_detects_separation():
    from bel_amr_ros.robot_controller import AMRController
    c = Mock()
    c.x, c.y, c.yaw = 5.0, 6.0, math.pi/2
    c.pallet_pose, c.pallet_yaw = (5.0,7.1), math.pi/2
    c.normalize_angle = AMRController.normalize_angle
    c.pallet_relative = lambda: AMRController.pallet_relative(c)
    c.carry_relative = (1.1,0.0,0.0)
    assert AMRController.relative_geometry_valid(c)
    c.pallet_pose = (5.0,6.5)
    assert not AMRController.relative_geometry_valid(c)


def test_detach_acknowledgment_does_not_complete_task():
    from bel_amr_ros.robot_controller import AMRController
    c = Mock()
    c.robot_state = 'DROPPING'
    c.attachment_state = 'detached'
    c.attachment_time, c.action_started = 12, 10
    c.x, c.y, c.pallet_pose = 8.0,16.0,(9.1,16.0)
    AMRController.attachment_step(c, 13)
    c.finish_task.assert_not_called()
    c.transition.assert_called_with('VERIFYING_RELEASE',
        'Joint detached; prove pallet stays at receiving location')


def test_active_task_does_not_route_to_charger():
    from bel_amr_ros.robot_controller import AMRController
    c = Mock()
    c.robot_state, c.battery_level = 'MOVING_TO_PICKUP', 21.0
    AMRController.update_battery(c)
    c.set_route.assert_not_called()
    assert c.robot_state == 'MOVING_TO_PICKUP'


def test_link_local_pallet_pose_is_ignored():
    from geometry_msgs.msg import PoseStamped
    from bel_amr_ros.robot_controller import AMRController
    c = Mock()
    c.pallet_pose = (4.4161,14.6952)
    msg = PoseStamped()
    msg.header.frame_id = 'pallet_box_mobile'
    AMRController.pallet_callback(c,msg)
    assert c.pallet_pose == (4.4161,14.6952)


def test_diagonal_lidar_inflation_does_not_block_safe_robot_center():
    warehouse = WarehouseMap()
    robot = (10.2874,6.8321)  # Clear of the newly mapped structural column.
    angle = 1.85 - math.radians(40)
    hit = (robot[0]+0.8*math.cos(angle), robot[1]+0.8*math.sin(angle))
    warehouse.add_dynamic_obstacle(*hit)
    assert not warehouse.is_occupied_grid(*warehouse.world_to_grid(*robot))
    assert warehouse.is_occupied_grid(*warehouse.world_to_grid(*hit))


def test_drop_waits_for_world_motion_to_settle():
    from bel_amr_ros.robot_controller import AMRController
    c = Mock()
    c.robot_state, c.attachment_state = 'DROPPING', 'attached'
    c.attachment_time, c.action_started = 9.0, 10.0
    c.last_attachment_command = 0.0
    c.x, c.y, c.pallet_pose = 8.02, 16.0, (9.12, 16.0)
    c.release_settle_pose = (8.0,16.0,9.1,16.0)
    c.release_settle_since = 10.0
    AMRController.attachment_step(c, 12.0)
    c.detach_pub.publish.assert_not_called()
    AMRController.attachment_step(c, 13.9)
    c.detach_pub.publish.assert_not_called()
    AMRController.attachment_step(c, 14.1)
    c.detach_pub.publish.assert_called_once()


def test_warehouse_columns_block_unloaded_and_loaded_routes():
    warehouse = WarehouseMap()
    assert warehouse.is_occupied_grid(*warehouse.world_to_grid(7.425,15.0))
    loaded = WarehouseMap(robot_inflation=1.85)
    assert loaded.is_occupied_grid(*loaded.world_to_grid(8.0,16.0))
    assert not loaded.is_occupied_grid(*loaded.world_to_grid(8.0,18.5))


def test_pickup_rejects_exit_toward_warehouse_column():
    planner = AStarPlanner(WarehouseMap())
    loaded = WarehouseMap(robot_inflation=1.85)
    loaded.pallets.pop('pallet_box_mobile')
    point, path = choose_approach(
        planner, (6.436,14.444), (4.4161,14.6952),
        departure_planner=AStarPlanner(loaded))
    assert point[1] > 14.6952  # North approach has room to reverse with load.


def test_carry_timeout_uses_simulation_time_under_slow_rendering():
    from bel_amr_ros.robot_controller import AMRController
    c = Mock()
    c.robot_state, c.attachment_state = 'VERIFYING_CARRY', 'attached'
    c.attachment_reference = (0.0,0.0,1.1,0.0)
    c.x, c.y, c.pallet_pose = 0.1, 0.0, (1.2,0.0)
    c.pallet_time, c.action_started = 1000.0, 0.0
    c.motion_started_sim, c.pose_sim_time = 0.0, 10.0
    c.relative_geometry_valid.return_value = True
    c.rear_obstacle_distance, c.obstacle_stop_distance = 4.0, 0.8
    AMRController.verify_motion(c, 1000.0)
    c.fail_task.assert_not_called()
    c.cmd_vel_pub.publish.assert_called_once()
    c.pose_sim_time = 91.0
    AMRController.verify_motion(c, 1000.0)
    c.fail_task.assert_called_once_with('Physical verification motion timed out', holding=True)
