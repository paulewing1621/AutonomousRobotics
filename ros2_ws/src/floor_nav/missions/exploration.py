#!/usr/bin/env python3
import sys
import time
import subprocess
import rclpy
import tf2_ros
from task_manager_client_py.TaskClient import TaskClient, TaskException
from rclpy.utilities import remove_ros_args
from kobuki_ros_interfaces.msg import Sound
from sensor_msgs.msg import BatteryState

rclpy.init(args=sys.argv)
sys.argv = remove_ros_args(args=sys.argv)

tc = TaskClient('/floor_tasks', 0.2)

EXPLORATION_DURATION = 300
RETURN_TIMEOUT       = 180
MAX_RETURN_ATTEMPTS  = 3
RETRY_EXPLORE        = 20

_sound_node = rclpy.create_node('sound_publisher')
_sound_pub  = _sound_node.create_publisher(Sound, '/commands/sound', 1)

def play_sound(value: int):
    msg = Sound()
    msg.value = value
    _sound_pub.publish(msg)
    rclpy.spin_once(_sound_node, timeout_sec=0.05)

_battery_node = rclpy.create_node('battery_monitor')
def _on_battery(msg: BatteryState):
    global _battery_pct
    _battery_pct = msg.percentage

_battery_node.create_subscription(BatteryState, '/sensors/battery_state', _on_battery, 1)

def select_auto_source(topic: str):
    try:
        subprocess.run([
            'ros2', 'service', 'call', '/auto_mux/select',
            'topic_tools_interfaces/srv/MuxSelect',
            f"{{topic: '{topic}'}}"
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2.0)
    except subprocess.TimeoutExpired:
        pass
    tc.get_logger().info("auto_mux → %s" % topic)

def capture_dock_position(timeout=30.0):
    """Lit la position courante dans le repère map via TF."""
    tmp = rclpy.create_node('dock_pos_reader')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, tmp)
    deadline  = time.time() + timeout
    last_exc  = None
    while time.time() < deadline:
        rclpy.spin_once(tmp, timeout_sec=0.2)
        try:
            t = buf.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            tmp.destroy_node()
            return x, y
        except Exception as e:
            last_exc = e
    tmp.destroy_node()
    raise RuntimeError("TF map→base_link indisponible après %.0fs : %s" % (timeout, last_exc))

def stop_exploration():
    subprocess.run(
        ['ros2', 'param', 'set', '/occgrid_planner', '~/exploring', 'false'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    tc.get_logger().info("Exploration stoppée.")

try:
    tc.SetMuxGeneric(topic='/mux/autoCommand', service_name='/cmd_mux/select')

    # INIT
    tc.get_logger().info("--- ATTENTE INITIALISATION 30s ---")
    time.sleep(30)

    # Recule
    tc.get_logger().info("--- RECULER ---")
    select_auto_source('/floor_nav/cmd')
    tc.Constant(linear=-0.2, duration=4)

    # Prend la position
    try:
        dock_x, dock_y = capture_dock_position(timeout=30.0)
    except RuntimeError as e:
        tc.get_logger().error(str(e))
        rclpy.shutdown()
        sys.exit(1)
    tc.get_logger().info("Position dock capturée : (%.3f, %.3f)" % (dock_x, dock_y))

    tc.get_logger().info("--- Tour sur lui-même ---")
    tc.Constant(angular=1.0, duration=6)

    rclpy.spin_once(_battery_node, timeout_sec=2.0)
    tc.get_logger().info("BATTERY : %.1f%%" % _battery_pct)

    # Exploration
    tc.get_logger().info("--- EXPLORATION %ds ---" % EXPLORATION_DURATION)
    low_battery = _battery_pct < 20.0
    if not low_battery:
        select_auto_source('/path_follower/cmd')
        time.sleep(EXPLORATION_DURATION)
    else:
        tc.get_logger().info("Not enough battery")

    play_sound(3)
    tc.get_logger().info("--- FIN EXPLORATION (batterie=%.1f%%, low=%s) ---"
                         % (_battery_pct, low_battery))

    # Reviens a la base
    stop_exploration()
    time.sleep(1.0)

    select_auto_source('/path_follower/cmd')

    for attempt in range(1, MAX_RETURN_ATTEMPTS + 1):
        tc.get_logger().info("--- RETOUR BASE tentative %d/%d (%.3f, %.3f) ---"
                             % (attempt, MAX_RETURN_ATTEMPTS, dock_x, dock_y))
        try:
            tc.PlanTo(goal_x=dock_x, goal_y=dock_y,
                      dist_threshold=0.5,
                      task_timeout=RETURN_TIMEOUT,
                      pub_period=1.0)
            tc.get_logger().info("Retour base réussi.")
            play_sound(1)
            break
        except TaskException as e:
            tc.get_logger().warn("PlanTo tentative %d échouée : %s" % (attempt, str(e)))
            if attempt < MAX_RETURN_ATTEMPTS:
                tc.get_logger().info("Exploration de dégagement %ds..." % RETRY_EXPLORE)
                subprocess.run(
                    ['ros2', 'param', 'set', '/occgrid_planner', '~/exploring', 'true'],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                )
                time.sleep(RETRY_EXPLORE)
                stop_exploration()
                time.sleep(1.0)
            else:
                tc.get_logger().warn("Impossible de rentrer après %d tentatives." % MAX_RETURN_ATTEMPTS)

except TaskException as e:
    tc.get_logger().error("Exception caught : " + str(e))

# Autodock
try:
    tc.get_logger().info("--- AUTO DOCK ---")
    select_auto_source('/floor_nav/cmd')
    tc.AutoDock()
    tc.get_logger().info("Docking completed.")
    play_sound(1)
except TaskException as e:
    tc.get_logger().error("AutoDock failed : " + str(e))
finally:
    stop_exploration()
    tc.SetMuxGeneric(topic='/teleop/twistCommand', service_name='/cmd_mux/select')
    _sound_node.destroy_node()
    tc.get_logger().info("Mux restauré en mode téléop.")

tc.get_logger().info("Mission completed.")