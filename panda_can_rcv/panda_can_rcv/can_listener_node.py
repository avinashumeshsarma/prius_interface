import rclpy
from rclpy.node import Node
import time
from panda_can_rcv.can_interface import CANInterface
from panda_can_rcv.message_relay import MessageRelay
from panda_msgs.msg import CANMessage
from autoware_vehicle_msgs.msg import GearReport, SteeringReport, VelocityReport, TurnIndicatorsReport, HazardLightsReport, ControlModeReport
from std_msgs.msg import String
import cantools
import os
import numpy as np

class CANListenerNode(Node):
    def __init__(self):
        super().__init__('can_listener_node')

        # Load parameters from YAML
        self.declare_parameter('mode', 'real')
        self.declare_parameter('input_topic', '/test/can_input')
        self.declare_parameter('decode_topic', '/can/decoded')
        self.declare_parameter('filter_topic','/can/filtered')
        self.declare_parameter('allowed_ids', [0])
        self.declare_parameter('dbc_file_path','panda_can_rcv/config/test_dbc.dbc')
        self.declare_parameter('gear_status_topic', '/vehicle/status/gear_status')
        self.declare_parameter('steering_status_topic', '/vehicle/status/steering_status')
        self.declare_parameter('velocity_status_topic', '/vehicle/status/velocity_status')
        self.declare_parameter('turn_indicators_status_topic', '/vehicle/status/turn_indicators_status')
        self.declare_parameter('hazard_lights_status_topic', '/vehicle/status/hazard_lights_status')
        self.declare_parameter('control_mode_status_topic', '/vehicle/status/control_mode')
        self.declare_parameter('driving_mode',4)


        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.decode_topic = self.get_parameter('decode_topic').get_parameter_value().string_value
        self.filter_topic = self.get_parameter('filter_topic').get_parameter_value().string_value
        self.allowed_ids = self.get_parameter('allowed_ids').get_parameter_value().integer_array_value
        self.dbc_file_path = self.get_parameter('dbc_file_path').get_parameter_value().string_value
        self.driving_mode = self.get_parameter('driving_mode').get_parameter_value().integer_value
        self.publish_rate_hz = 6.0  # unified publish rate


        # Load DBC file
        if os.path.exists(self.dbc_file_path):
            self.dbc = cantools.database.load_file(self.dbc_file_path)
            self.get_logger().info(f"Loaded DBC file: {self.dbc_file_path}")
        else:
            self.get_logger().error(f"DBC file not found: {self.dbc_file_path}")
            self.dbc = None

        # self.signal_publishers = {
        #     0x127: {
        #         'GEAR': {
        #             'topic': self.get_parameter('gear_status_topic').get_parameter_value().string_value,
        #             'msg_type': GearReport,
        #             'publisher': None,
        #             'field_mapping': {'gear': int},
        #         },
        #     },
        #     0x25: {
        #         'STEER_ANGLE': {
        #             'topic': self.get_parameter('steering_status_topic').get_parameter_value().string_value,
        #             'msg_type': SteeringReport,
        #             'publisher': None,
        #             'field_mapping': {'steering_angle': float},
        #         },
        #     },
        #     0xB4: {
        #         'SPEED': {
        #             'topic': self.get_parameter('velocity_status_topic').get_parameter_value().string_value,
        #             'msg_type': VelocityReport,
        #             'publisher': None,
        #             'field_mapping': {'velocity': float},
        #         },
        #     },
        #     0x614: {
        #         'TURN_SIGNALS': {
        #             'topic': self.get_parameter('turn_indicators_status_topic').get_parameter_value().string_value,
        #             'msg_type': TurnIndicatorsReport,
        #             'publisher': None,
        #             'field_mapping': {'report': int},
        #         },
        #         'HAZARD_LIGHT': {
        #             'topic': self.get_parameter('hazard_lights_status_topic').get_parameter_value().string_value,
        #             'msg_type': HazardLightsReport,
        #             'publisher': None,
        #             'field_mapping': {'report': int},
        #         },
        #     },
        # }

        # for signals in self.signal_publishers.values():
        #     for signal in signals.values():
        #         signal['publisher'] = self.create_publisher(
        #             signal['msg_type'],
        #             signal['topic'],
        #             10
        #         )

        self.velocity_publisher = self.create_publisher(VelocityReport, self.get_parameter('velocity_status_topic').get_parameter_value().string_value, 10)
        self.gear_publisher = self.create_publisher(GearReport, self.get_parameter('gear_status_topic').get_parameter_value().string_value, 10)
        self.steer_publisher = self.create_publisher(SteeringReport, self.get_parameter('steering_status_topic').get_parameter_value().string_value, 10)
        self.turn_signal_publisher = self.create_publisher(TurnIndicatorsReport, self.get_parameter('turn_indicators_status_topic').get_parameter_value().string_value, 10)
        self.hazard_light_publisher = self.create_publisher(HazardLightsReport, self.get_parameter('hazard_lights_status_topic').get_parameter_value().string_value, 10)
        self.control_mode_publisher = self.create_publisher(ControlModeReport, self.get_parameter('control_mode_status_topic').get_parameter_value().string_value, 10)


        self.filter_publisher = self.create_publisher(CANMessage, self.filter_topic, 10)
        self.decode_publisher = self.create_publisher(String, self.decode_topic, 10)

        self.latest_signals = {
            'SPEED': None,
            'STEER_ANGLE': None,
            'GEAR': None,
            'TURN_SIGNALS': None,
            'HAZARD_LIGHT': None,
            'YAW_RATE': None,
        }

        self.gear_mapping = { #DBC -> Autoware
            0: 22,  # P → Autoware: 22
            1: 20,  # R → 20
            2: 1,   # N → 1
            3: 3,   # D → 3
            4: 3    # B (treated as Drive) → 3
        }

        self.turn_signal_mapping = { #DBC -> Autoware
            1: 2,  # Left
            2: 3,  # Right
            3: 1   # Disabled
        }

        self.publisher_timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_signals)



        if self.mode == 'real':
            self.can_interface = CANInterface()
            self.timer = self.create_timer(0.000002, self.read_from_device)
            self.get_logger().info(f"Running in REAL mode, reading from black panda")

        elif self.mode == 'test':
            self.subscriber = self.create_subscription(
                CANMessage,
                self.input_topic,
                self.test_callback,
                10
            )
            self.get_logger().info(f"Running in TEST mode")

        self.get_logger().info(f"Allowed CAN IDs: {[hex(id) for id in self.allowed_ids]}")

    def read_from_device(self):
        messages = self.can_interface.read_messages()
        for msg_id, data, src in messages:
            msg = CANMessage()
            msg.id = msg_id
            msg.data = list(data)
            msg.timestamp = self.get_clock().now().nanoseconds
            self.process_message(msg)
    
    def test_callback(self, msg):
        self.process_message(msg)

    def process_message(self, msg: CANMessage):
        # Filtering
        if self.allowed_ids and msg.id not in self.allowed_ids:
            return  # Skip filtered messages

        # Further processing steps can go here...
        # For example, decoding data, applying scaling, logging, etc.

        self.filter_publisher.publish(msg)
        self.get_logger().debug(f"Published: ID={hex(msg.id)}, Data={msg.data}, Timestamp={msg.timestamp}")

        if self.dbc:
            try:
                message = self.dbc.get_message_by_frame_id(msg.id)
                decoded_signals = message.decode(bytes(msg.data))

                # for key in self.latest_signals:
                #     if key in decoded_signals:
                #         self.latest_signals[key] = decoded_signals[key]

                for key in self.latest_signals:
                    if key in decoded_signals:
                        value = decoded_signals[key]
                        if isinstance(value, cantools.database.can.signal.NamedSignalValue):
                            value = value.value  # Convert to raw int
                        self.latest_signals[key] = value




                decoded_msg = String()
                decoded_msg.data = f"ID: {hex(msg.id)}, Signals: {decoded_signals}"
                self.decode_publisher.publish(decoded_msg)
                self.get_logger().debug(f"Decoded Published: {decoded_msg.data}")

            except Exception as e:
                self.get_logger().warn(f"Failed to decode message ID {hex(msg.id)}: {e}")



    def publish_signals(self):
        now = self.get_clock().now().to_msg()
       

        if (self.latest_signals['SPEED'] and self.latest_signals['YAW_RATE']) is not None:
            vel_msg = VelocityReport()
            vel_msg.header.stamp = now
            vel_msg.header.frame_id = "base_link"

            if self.latest_signals['SPEED']==0: #Calculation for slip angle
                beta=0
            else:
                beta=np.arctan((1.485*float(self.latest_signals['YAW_RATE'])*0.017453)/float(self.latest_signals['SPEED'])/3.6) # Rear-Wheel base considered as 1.485
        
            if self.latest_signals['GEAR']==1:
                vel_msg.longitudinal_velocity = -1.0*float(self.latest_signals['SPEED'])/3.6#*np.cos(beta)/3.6
            else:
                vel_msg.longitudinal_velocity = float(self.latest_signals['SPEED'])/3.6#*np.cos(beta)/3.6
            vel_msg.lateral_velocity = 0.0#float(self.latest_signals['SPEED'])*np.sin(beta)/3.6
            vel_msg.heading_rate = float(self.latest_signals['YAW_RATE'])*0.017453
            self.velocity_publisher.publish(vel_msg)

        if self.latest_signals['GEAR'] is not None:
            raw_gear = int(self.latest_signals['GEAR'])
            mapped_gear = self.gear_mapping.get(raw_gear, 1) # Default to Neutral

            gear_msg = GearReport()
            gear_msg.stamp = now
            gear_msg.report = mapped_gear
            self.gear_publisher.publish(gear_msg)

        if self.latest_signals['STEER_ANGLE'] is not None:
            steer_msg = SteeringReport()
            steer_msg.stamp = now
            steer_msg.steering_tire_angle = float(self.latest_signals['STEER_ANGLE'])*0.017453
            self.steer_publisher.publish(steer_msg)

        if self.latest_signals['TURN_SIGNALS'] is not None:
            raw_ts = int(self.latest_signals['TURN_SIGNALS'])
            mapped_ts = self.turn_signal_mapping.get(raw_ts, 1)  # Default to Disabled

            ts_msg = TurnIndicatorsReport() 
            ts_msg.stamp = now
            ts_msg.report = mapped_ts
            self.turn_signal_publisher.publish(ts_msg)

        if self.latest_signals['HAZARD_LIGHT'] is not None:
            hz_msg = HazardLightsReport()
            hz_msg.stamp = now
            hz_msg.report = int(self.latest_signals['HAZARD_LIGHT'])+1
            self.hazard_light_publisher.publish(hz_msg)
        
        #   Control Mode Report - Fixing it a constant value of MANUAL Driving
        control_mode_msg = ControlModeReport()
        control_mode_msg.stamp = now
        control_mode_msg.mode = self.driving_mode 
        self.control_mode_publisher.publish(control_mode_msg)

        # if 'SPEED' in decoded_signals:
        #     velocity = float(decoded_signals['SPEED'])
        #     velocity_msg = VelocityReport()
        #     velocity_msg.header.stamp = self.get_clock().now().to_msg()
        #     velocity_msg.header.frame_id = "base_link" 
        #     velocity_msg.lateral_velocity = 0.0
        #     velocity_msg.heading_rate = 0.0
        #     velocity_msg.longitudinal_velocity = velocity
        #     self.signal_publishers[0xB4]['SPEED']['publisher'].publish(velocity_msg)

            # except Exception as e:
            #     self.get_logger().warn(f"Failed to decode message ID {hex(msg.id)}: {e}")


    

def main(args=None):
    rclpy.init(args=args)
    node = CANListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
