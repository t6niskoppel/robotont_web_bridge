"""Robotont ROS node next to rosbridge: finds robots, keeps motion safe, drives the LED module.

rosbridge (rosbridge_server) is the web bridge and web_video_server serves the camera images; this node
only adds what neither of them does. Topics are relative to the node namespace, e.g. /rt55/web/robots when
launched with namespace:=rt55, so several robots can share a ROS domain without mixing their web pages:

* web/robots      (std_msgs/String, JSON list) robots found in this ROS domain, by namespace.
* web/command     (std_msgs/String, JSON {robot, vx, vy, wz}) velocity request. It is re-published to
                  <robot>/cmd_vel at 20 Hz and expires after 0.5 s, so a closed browser stops the robot.
* web/stop        (std_msgs/String, JSON {robot}) zero velocity now.
* web/led         (std_msgs/String, JSON {robot, side, color}) colour for 'all' or a side.
* web/distances   (std_msgs/String, JSON {robot: {front|back|left|right: reading}}) from <robot>/scan.

A robot is any namespace that has a cmd_vel subscriber (the real driver or the Gazebo bridge).
"""
import json
import math
import re
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from robotont_msgs.msg import ColorRGB, LedModuleMode, LedModuleSegment
from tf2_ros import Buffer, TransformException, TransformListener

from .scan import DIRECTIONS, project_scan
from .camera import camera_topics

MAX_LINEAR = 0.3
MAX_ANGULAR = 1.0
COMMAND_TIMEOUT = 0.5
CMD_VEL = re.compile(r'^(?:/([A-Za-z0-9_]+))?/cmd_vel$')
LED_COUNT = 60
# Gen 3 mainboard: pixel 0 is near the centre of the straight front edge.
LED_GROUPS = {
    'front': ((0, 9), (51, 59)),
    'right': ((10, 22),),
    'back': ((23, 36),),
    'left': ((37, 50),),
}
HEX_COLOR = re.compile(r'^#[0-9a-fA-F]{6}$')


def robot_name(namespace):
    digits = re.search(r'(\d+)$', namespace)
    return f'robotont {digits.group(1)}' if digits else 'robotont'


class Robot:
    """State for one robot namespace."""

    def __init__(self, node, namespace):
        self.node = node
        self.namespace = namespace
        prefix = f'/{namespace}' if namespace else ''
        self.velocity_publisher = node.create_publisher(Twist, prefix + '/cmd_vel', QoSProfile(depth=1))
        self.mode_publisher = node.create_publisher(LedModuleMode, prefix + '/led_mode', 10)
        self.segment_publisher = node.create_publisher(LedModuleSegment, prefix + '/led_segment', 10)
        self.velocity = (0.0, 0.0, 0.0)
        self.deadline = 0.0
        self.stopping_until = 0.0
        self.led_state = []        # (first, last, (r, g, b)) in the order they were set
        self.led_generation = 0
        self.readings = {}
        self.scan_received = -math.inf
        self.scan_subscription = node.create_subscription(
            LaserScan, prefix + '/scan', self.receive_scan, qos_profile_sensor_data)
        self.base_frames = [f'{namespace}/base_footprint', 'base_footprint'] if namespace else ['base_footprint']

    def close(self):
        self.stop()
        self.node.destroy_subscription(self.scan_subscription)
        self.node.destroy_publisher(self.velocity_publisher)
        self.node.destroy_publisher(self.mode_publisher)
        self.node.destroy_publisher(self.segment_publisher)

    # Motion -----------------------------------------------------------------------------------
    def command(self, vx, vy, wz):
        values = (vx, vy, wz)
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
            raise ValueError('Velocity must contain three finite numbers')
        if math.hypot(vx, vy) > MAX_LINEAR + 1e-6 or abs(wz) > MAX_ANGULAR:
            raise ValueError('Maximum speed is 0.3 m/s and 1 rad/s')
        self.velocity = tuple(float(v) for v in values)
        self.deadline = time.monotonic() + COMMAND_TIMEOUT
        self.publish_velocity(self.velocity)

    def stop(self):
        self.velocity = (0.0, 0.0, 0.0)
        self.deadline = 0.0
        self.stopping_until = time.monotonic() + COMMAND_TIMEOUT
        self.publish_velocity(self.velocity)

    def tick(self):
        now = time.monotonic()
        if self.deadline:
            if now < self.deadline:
                self.publish_velocity(self.velocity)
            else:
                self.stop()
        elif now < self.stopping_until:
            self.publish_velocity((0.0, 0.0, 0.0))
        # Otherwise stay silent so Nav2 or another teleop is not overridden.

    def publish_velocity(self, velocity):
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = velocity
        self.velocity_publisher.publish(message)

    # Scan -------------------------------------------------------------------------------------
    def receive_scan(self, message):
        readings = None
        for base in self.base_frames:
            try:
                transform = self.node.buffer.lookup_transform(
                    base, message.header.frame_id, Time.from_msg(message.header.stamp)).transform
            except TransformException:
                continue
            t, q = transform.translation, transform.rotation
            readings = project_scan(message.ranges, message.angle_min, message.angle_increment,
                                    message.range_min, message.range_max, (t.x, t.y), (q.x, q.y, q.z, q.w))
            break
        if readings is None:
            readings = {side: {'available': False, 'reason': 'Sensor transform is unavailable'}
                        for side in DIRECTIONS}
        self.readings = readings
        self.scan_received = time.monotonic()

    def distances(self):
        if time.monotonic() - self.scan_received > 1.0:
            return {side: {'available': False, 'reason': 'No fresh scan data'} for side in DIRECTIONS}
        return self.readings

    # LEDs -------------------------------------------------------------------------------------
    def set_led(self, color, groups):
        """Show `color` on `groups` (pixel ranges).

        The firmware cycles colours (COLORS_SMOOTH) at start-up and again after the stop button or a low
        battery, and the cycle overwrites pixel commands. So every command first sets mode 0 (NONE), which
        also clears the strip, and then replays everything that should be lit, with a pause between the
        writes (the firmware drops a frame that arrives while the previous one is still being sent)."""
        rgb = (int(color[1:3], 16), int(color[3:5], 16), int(color[5:7], 16))
        for span in groups:
            # A new range replaces the older ranges it fully covers, so the state never grows without bound.
            self.led_state = [(a, b, c) for a, b, c in self.led_state if not (span[0] <= a and b <= span[1])]
            self.led_state.append((span[0], span[1], rgb))
        self.led_generation += 1
        generation = self.led_generation
        self.mode_publisher.publish(LedModuleMode(mode=LedModuleMode.NONE))
        for i, (start, end, colour) in enumerate(list(self.led_state)):
            self.node.after(0.12 + 0.05 * i, lambda s=start, e=end, c=colour:
                            self.write_segment(generation, s, e, c))

    def write_segment(self, generation, start, end, colour):
        if generation != self.led_generation or self.node.robots.get(self.namespace) is not self:
            return  # a newer command has started over from mode 0 and replays everything itself
        # A single colour in a segment fills the whole range: one message instead of one per pixel.
        self.segment_publisher.publish(LedModuleSegment(
            idx_start=start, idx_end=end, colors=[ColorRGB(r=colour[0], g=colour[1], b=colour[2])]))


class WebBridge(Node):
    def __init__(self):
        super().__init__('web_bridge')
        self.declare_parameter('namespace', '')
        self.robot_namespace = self.get_parameter('namespace').value.strip('/')
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.robots = {}
        self.found = []
        self.pending_timers = []
        self.robots_publisher = self.create_publisher(String, 'web/robots', 10)
        self.distances_publisher = self.create_publisher(String, 'web/distances', 10)
        self.create_subscription(String, 'web/command', self.on_command, 10)
        self.create_subscription(String, 'web/stop', self.on_stop, 10)
        self.create_subscription(String, 'web/led', self.on_led, 10)
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self.tick, clock=steady)
        self.create_timer(1.0, self.discover, clock=steady)
        self.create_timer(0.2, self.publish_distances, clock=steady)
        self.discover()

    def after(self, seconds, action):
        def run():
            timer.cancel()
            self.pending_timers.remove(timer)
            action()
        timer = self.create_timer(seconds, run, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.pending_timers.append(timer)

    # Robots -----------------------------------------------------------------------------------
    def discover(self):
        names = dict(self.get_topic_names_and_types())
        found = []
        for topic, types in names.items():
            match = CMD_VEL.match(topic)
            if not match or 'geometry_msgs/msg/Twist' not in types or self.count_subscribers(topic) == 0:
                continue
            namespace = match.group(1) or ''
            if namespace == 'web' or (self.robot_namespace and namespace != self.robot_namespace):
                continue
            prefix = f'/{namespace}' if namespace else ''
            # Only topics somebody publishes: a topic can also appear because of a subscriber.
            def has(suffix):
                topic = prefix + suffix
                return topic in names and self.count_publishers(topic) > 0

            topics, transport, generation = camera_topics(has)
            for key, suffix in (('scan', '/scan'), ('odom', '/odom'), ('battery', '/battery_state')):
                if has(suffix):
                    topics[key] = suffix
            found.append({
                'ns': namespace, 'name': robot_name(namespace),
                'generation': generation,
                'topics': {key: prefix + suffix for key, suffix in topics.items()},
                'rgb_transport': transport,
                'led': self.count_subscribers(prefix + '/led_mode') > 0,
            })
        active = {entry['ns'] for entry in found}
        for namespace in list(self.robots):
            if namespace not in active:
                self.robots.pop(namespace).close()
        self.found = sorted(found, key=lambda robot: robot['ns'])
        for entry in self.found:
            self.robot(entry['ns'])
        self.robots_publisher.publish(String(data=json.dumps(self.found)))

    def robot(self, namespace):
        if namespace not in [entry['ns'] for entry in self.found]:
            raise ValueError(f"Unknown robot '{namespace}'")
        if namespace not in self.robots:
            self.robots[namespace] = Robot(self, namespace)
        return self.robots[namespace]

    # Browser requests -------------------------------------------------------------------------
    def dispatch(self, message, action):
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError('Request must be a JSON object')
            action(self.robot(payload.get('robot', '')), payload)
        except (ValueError, TypeError, KeyError) as error:
            self.get_logger().warning(f'Rejected web request: {error}')

    def on_command(self, message):
        def command(robot, payload):
            robot.command(payload.get('vx'), payload.get('vy'), payload.get('wz'))
        self.dispatch(message, command)

    def on_stop(self, message):
        def stop(robot, payload):
            robot.stop()
        self.dispatch(message, stop)

    def on_led(self, message):
        def apply(robot, payload):
            side = payload.get('side')
            color = payload.get('color')
            if not isinstance(color, str) or not HEX_COLOR.match(color):
                raise ValueError('LED color must be a hex RGB color')
            if side == 'all':
                spans = [(0, LED_COUNT - 1)]
            elif side in LED_GROUPS:
                spans = list(LED_GROUPS[side])
            else:
                raise ValueError('Unknown LED group')
            robot.set_led(color, spans)
        self.dispatch(message, apply)

    def tick(self):
        for robot in self.robots.values():
            robot.tick()

    def publish_distances(self):
        data = {ns: robot.distances() for ns, robot in self.robots.items()}
        self.distances_publisher.publish(String(data=json.dumps(data, allow_nan=False, default=str)))


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = WebBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for robot in node.robots.values():
            try:
                robot.stop()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
