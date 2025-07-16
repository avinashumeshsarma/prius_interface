import rclpy
from rclpy.node import Node
from autoware_vehicle_msgs.msg import GearCommand, HazardLightsCommand, TurnIndicatorsCommand
from tier4_vehicle_msgs.msg import ActuationCommandStamped, VehicleEmergencyStamped
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import GearReport, SteeringReport, VelocityReport, TurnIndicatorsReport, HazardLightsReport, ControlModeReport
import zmq
import threading
import capnp
import os
import math

class ZMQCapnpBridgeNode(Node):
    def __init__(self):
        super().__init__('zmq_capnp_bridge_node')

        # Declare and get parameters
        self.declare_parameter('capnp_dir', '')
        self.declare_parameter('publish_rate_hz', 10.0)
        capnp_dir = self.get_parameter('capnp_dir').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        self.zmq_publish_rate=100

        # Load Cap'n Proto schema
        if capnp_dir:
            capnp.remove_import_hook()
            self.log_capnp = capnp.load(os.path.join(capnp_dir, 'log.capnp'))
        else:
            self.get_logger().error('Capnp directory not provided. Exiting.')
            return

        # ZMQ setup
        self.zmq_context = zmq.Context()

        # Subscriber socket for carState
        self.sub_socket = self.zmq_context.socket(zmq.SUB)
        self.sub_socket.connect("tcp://127.0.0.1:9041")
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, '')

        # Publisher socket for carControl
        self.pub_socket = self.zmq_context.socket(zmq.PUB)
        self.pub_socket.bind("tcp://127.0.0.1:63225")

        # ROS-to-ZMQ control signals dictionary
        self.control_signals = {
            'steering_angle_deg': 0.0,
            'accel': 0.0,
            'brake': 0.0,
            'hazard_lights': False,
            'turn_signal': 0,
            'gear': 0,
            'emergency': False,
            'enable': True,
            'latActive': True,
            'longActive': True
        }

        # ZMQ-to-ROS state signals dictionary
        self.state_signals = {
            'steering_angle_deg': 0.0,
            'vEgo': 0.0,
            'gear': 'P',
            'turn_signal': 0,
            'hazard_lights': False,
            'yaw_rate': 0.0,
            'control_mode': 1,
            'steeringPressed': False,
            'gasPressed': False,
            'brakePressed': False
        }

        self.turn_signal_mapping = { #DBC -> Autoware
            1: 2,  # Left
            2: 3,  # Right
            3: 1   # Disabled
        }
        self.gear_mapping = { #DBC -> Autoware
            'park': 22,  # P → Autoware: 22
            'reverse': 20,  # R → 20
            'neutral': 1,   # N → 1
            'drive': 3,   # D → 3
            # B (treated as Drive) → 3
        }

        # ROS publishers
        self.control_mode_pub = self.create_publisher(ControlModeReport, '/vehicle/status/control_mode', 10)
        self.gear_pub = self.create_publisher(GearReport, '/vehicle/status/gear_status', 10)
        self.hazard_pub = self.create_publisher(HazardLightsReport, '/vehicle/status/hazard_lights_status', 10)
        self.steer_pub = self.create_publisher(SteeringReport, '/vehicle/status/steering_status', 10)
        self.turn_pub = self.create_publisher(TurnIndicatorsReport, '/vehicle/status/turn_indicators_status', 10)
        self.vel_pub = self.create_publisher(VelocityReport, '/vehicle/status/velocity_status', 10)

        # ROS subscribers
        self.create_subscription(ActuationCommandStamped, '/control/command/actuation_cmd', self.actuation_cb, 10)
        self.create_subscription(Control, '/control/command/control_cmd', self.control_cb, 10)
        self.create_subscription(VehicleEmergencyStamped, '/control/command/emergency_cmd', self.emergency_cb, 10)
        self.create_subscription(GearCommand, '/control/command/gear_cmd', self.gear_cmd_cb, 10)
        self.create_subscription(HazardLightsCommand, '/control/command/hazard_lights_cmd', self.hazard_cmd_cb, 10)
        self.create_subscription(TurnIndicatorsCommand, '/control/command/turn_indicators_cmd', self.turn_cmd_cb, 10)

                # Start periodic publishing timer
        self.create_timer(1.0 / self.publish_rate, self.publish_ros_messages)

        self.create_timer(1.0 / self.zmq_publish_rate, self.send_car_control)

        self.listener_thread = threading.Thread(target=self.listen_loop, daemon=True)
        self.listener_thread.start()

    def actuation_cb(self, msg):
        self.control_signals['steering_angle_deg'] = msg.actuation.steer_cmd*180/math.pi
        self.control_signals['accel'] = msg.actuation.accel_cmd
        self.control_signals['brake'] = msg.actuation.brake_cmd

    def control_cb(self, msg):
        self.control_signals['enable'] = True#msg.control_enabled
        self.control_signals['latActive'] = True#msg.lateral_active
        self.control_signals['longActive'] = True#msg.longitudinal_active

    def emergency_cb(self, msg):
        self.control_signals['emergency'] = msg.emergency

    def gear_cmd_cb(self, msg):
        self.control_signals['gear'] = msg.command

    def hazard_cmd_cb(self, msg):
        self.control_signals['hazard_lights'] = msg.command

    def turn_cmd_cb(self, msg):
        self.control_signals['turn_signal'] = msg.command

    def listen_loop(self):
        while rclpy.ok():
            try:
                msg = self.sub_socket.recv()
                # event = self.log_capnp.Event.from_bytes(msg)
                with self.log_capnp.Event.from_bytes(msg) as evt:
                    if evt.which() == 'carState':
                        car_state = getattr(evt, 'carState')
                        self.process_car_state(car_state)
            except Exception as e:
                self.get_logger().error(f"ZMQ receive error: {e}")

    def process_car_state(self, car_state):
        self.state_signals['vEgo'] = car_state.vEgo
        self.state_signals['steering_angle_deg'] = car_state.steeringAngleDeg
        self.state_signals['steeringPressed'] = car_state.steeringPressed
        self.state_signals['gasPressed'] = car_state.gasPressed
        self.state_signals['brakePressed'] = car_state.brakePressed
        self.state_signals['yaw_rate'] = car_state.yawRate
        self.state_signals['gear'] = car_state.gearShifter
        if car_state.leftBlinker:
            self.state_signals['turn_signal'] = 2
            if car_state.rightBlinker:
                self.state_signals['hazard_lights'] = True
            else:
                self.state_signals['hazard_lights'] = False
        elif car_state.rightBlinker:
            self.state_signals['turn_signal'] = 3
            self.state_signals['hazard_lights'] = False
        else:
            self.state_signals['turn_signal'] = 1
        # self.state_signals['turn_signal'] = car_state.leftBlinker or car_state.rightBlinker
        # self.state_signals['hazard_lights'] = car_state.leftBlinker and car_state.rightBlinker
        self.state_signals['steering_angle_deg'] = car_state.steeringAngleDeg
        self.state_signals['steeringPressed'] = car_state.steeringPressed
        self.state_signals['gasPressed'] = car_state.gasPressed
        self.state_signals['brakePressed'] = car_state.brakePressed

    def publish_ros_messages(self):
        self.publish_velocity()
        self.publish_steering()
        self.publish_gear()
        self.publish_turn_indicators()
        self.publish_hazard_lights()
        self.publish_control_mode()

    def publish_velocity(self):
        msg = VelocityReport()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.longitudinal_velocity = float(self.state_signals['vEgo'])
        msg.lateral_velocity = 0.0 #Need to change according to panda_can_rcv
        msg.heading_rate = float(self.state_signals['yaw_rate'])*0.017453 #
        msg.header.frame_id = "base_link"
        self.vel_pub.publish(msg)

    def publish_steering(self):
        msg = SteeringReport()
        msg.stamp = self.get_clock().now().to_msg()
        msg.steering_tire_angle = float(self.state_signals['steering_angle_deg'])*0.017453
        self.steer_pub.publish(msg)

    def publish_gear(self):
        msg = GearReport()
        msg.stamp = self.get_clock().now().to_msg()
        # print(self.state_signals['gear'])
        # self.get_logger().info('The error check : "%s"' % self.state_signals['gear'])
        msg.report = int(self.gear_mapping[self.state_signals['gear']])
        self.gear_pub.publish(msg)

    def publish_turn_indicators(self):
        msg = TurnIndicatorsReport()
        msg.stamp = self.get_clock().now().to_msg()
        msg.report = int(self.state_signals['turn_signal'])
        self.turn_pub.publish(msg)

    def publish_hazard_lights(self):
        msg = HazardLightsReport()
        msg.stamp = self.get_clock().now().to_msg()
        msg.report = bool(self.state_signals['hazard_lights'])
        self.hazard_pub.publish(msg)

    def publish_control_mode(self):
        msg = ControlModeReport()
        msg.stamp = self.get_clock().now().to_msg()
        msg.mode = int(self.state_signals['control_mode'])
        self.control_mode_pub.publish(msg)

    def send_car_control(self):
        try:
            event = self.log_capnp.Event.new_message()
            event.init('carControl')
            #event.carControl.actuators.accel = self.control_signals['accel']
            event.carControl.actuators.accel = -self.control_signals['brake'] if self.control_signals['brake'] > 0 else self.control_signals['accel']
            event.carControl.actuators.steeringAngleDeg = self.control_signals['steering_angle_deg']
            event.carControl.enabled = self.control_signals['enable']
            event.carControl.latActive = self.control_signals['latActive']
            event.carControl.longActive = self.control_signals['longActive']

            event.carControl.hudControl.leadDistanceBars = 2

            event.carControl.cruiseControl.cancel = False

            self.pub_socket.send(event.to_bytes())
        except Exception as e:
            self.get_logger().error(f"ZMQ publish error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ZMQCapnpBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
