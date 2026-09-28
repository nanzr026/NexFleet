"""Motion feedback loss must preserve navigation and honor existing shields."""

from types import SimpleNamespace
from unittest.mock import Mock, patch

from bel_amr_ros.robot_controller import AMRController


def controller():
    """Create a minimal controller fixture with valid odometry and LiDAR."""
    node = SimpleNamespace(
        # Odometry watchdog state
        odom_received=True,
        last_odom_time=10.0,
        odom_timeout=1.0,
        waiting_for_odometry=False,

        # Navigation/task state
        robot_state='MOVING',
        current_task_id='test',
        waypoints=[(2.0, 0.0)],
        goal_x=2.0,
        goal_y=0.0,

        # CBS-style safety state
        safety_yield_active=False,

        # LiDAR safety state
        scan_received=True,
        last_scan_time=10.0,
        scan_timeout=1.0,
        scan_front_valid=True,
        obstacle_detected=False,
        reroute_rotation_ready=False,

        # Mocked controller operations
        stop_robot=Mock(),
        move_to_goal=Mock(),
        publish_navigation_intent=Mock(),
        get_logger=Mock(),
        update_safety_hold=Mock(),
        handle_persistent_obstacle=Mock(),
        advance_task_lifecycle=Mock(),
        log_motion_hold=Mock(),
    )

    node.odometry_is_fresh = (
        lambda: AMRController.odometry_is_fresh(node)
    )

    return node


def test_feedback_loss_stops_and_recovery_preserves_route():
    """Stale odometry must stop motion without destroying the active route."""
    node = controller()

    with patch(
        'bel_amr_ros.robot_controller.time.monotonic',
        return_value=12.0,
    ):
        AMRController.control_loop(node)
        AMRController.control_loop(node)

    assert node.stop_robot.call_count == 2
    node.move_to_goal.assert_not_called()

    assert node.current_task_id == 'test'
    assert node.waypoints == [(2.0, 0.0)]

    node.get_logger().warning.assert_called_once()

    # Restore both odometry and LiDAR freshness.
    node.last_odom_time = 12.0
    node.last_scan_time = 12.0

    with patch(
        'bel_amr_ros.robot_controller.time.monotonic',
        return_value=12.1,
    ):
        AMRController.control_loop(node)

    node.move_to_goal.assert_called_once()
    assert not node.waiting_for_odometry


def test_recovered_feedback_does_not_override_cbs_or_lidar():
    """Fresh odometry must not bypass CBS or LiDAR safety shields."""
    for shield in ('safety_yield_active', 'obstacle_detected'):
        node = controller()
        setattr(node, shield, True)

        with patch(
            'bel_amr_ros.robot_controller.time.monotonic',
            return_value=10.1,
        ):
            AMRController.control_loop(node)

        node.stop_robot.assert_called_once()
        node.move_to_goal.assert_not_called()


def test_never_received_feedback_stops():
    """The robot must remain stopped until odometry has been received."""
    node = controller()

    node.odom_received = False
    node.last_odom_time = None

    AMRController.control_loop(node)

    node.stop_robot.assert_called_once()
    node.move_to_goal.assert_not_called()
