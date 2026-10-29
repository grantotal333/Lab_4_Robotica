
#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray, Bool, String


class UR5PickPlace(Node):

    def __init__(self):
        super().__init__('ur5_pick_place_node')

        # Controlador de las seis articulaciones del UR5
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )

        # Controlador del gripper
        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/gripper_controller/commands',
            10
        )

        # Entrada digital simulada
        self.digital_input_sub = self.create_subscription(
            Bool,
            '/pick_place/digital_input',
            self.digital_input_callback,
            10
        )

        # Salida digital simulada
        self.digital_output_pub = self.create_publisher(
            Bool,
            '/pick_place/digital_output',
            10
        )

        # Mensajes informativos
        self.status_pub = self.create_publisher(
            String,
            '/pick_place/status',
            10
        )

        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        # Posiciones articulares en radianes
        # Son objetivos iniciales de prueba en simulacion.
        self.positions = {
            'HOME':     [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
            'PICK':     [0.5, -1.2, 1.3, -1.5, -1.57, 0.5],
            'TRANSFER': [0.0, -1.3, 1.4, -1.5, -1.57, 0.0],
            'PLACE':    [-0.5, -1.2, 1.3, -1.5, -1.57, -0.5]
        }

        # Secuencia completa
        self.steps = [
            ('arm', 'HOME'),
            ('gripper', 'open'),
            ('message', 'Robot en HOME. Gripper abierto.'),
            ('arm', 'PICK'),
            ('gripper', 'close'),
            ('message', 'Operacion PICK finalizada.'),
            ('arm', 'TRANSFER'),
            ('message', 'Esperando entrada digital...'),
            ('wait_input', None),
            ('arm', 'PLACE'),
            ('gripper', 'open'),
            ('output', True),
            ('message', 'Operacion PLACE finalizada.'),
            ('arm', 'HOME'),
            ('message', 'Rutina Pick and Place completada.')
        ]

        self.step_index = 0
        self.busy = False
        self.input_received = False
        self.wait_until = 0.0
        self.finished = False

        # La salida digital inicia apagada
        self.set_digital_output(False)

        self.timer = self.create_timer(0.2, self.run_sequence)

        self.get_logger().info(
            'Nodo Pick and Place iniciado correctamente.'
        )

    def publish_status(self, message):
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
        self.get_logger().info(message)

    def digital_input_callback(self, msg):
        if msg.data:
            self.input_received = True
            self.get_logger().info('Entrada digital TRUE recibida.')

    def set_digital_output(self, value):
        msg = Bool()
        msg.data = value
        self.digital_output_pub.publish(msg)

    def move_arm(self, position_name):
        self.busy = True

        if not self.arm_client.server_is_ready():
            self.publish_status('Esperando controlador del brazo...')
            self.busy = False
            return

        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.positions[position_name]
        point.time_from_start.sec = 5

        trajectory.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.publish_status(
            'Moviendo UR5 hacia ' + position_name
        )

        future = self.arm_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:
            self.publish_status(f'Error al enviar trayectoria: {error}')
            self.finished = True
            return

        if not goal_handle.accepted:
            self.publish_status('Trayectoria rechazada.')
            self.finished = True
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        try:
            response = future.result()
            result = response.result

            if (response.status == 4 and
                    result.error_code == FollowJointTrajectory.Result.SUCCESSFUL):
                self.publish_status('Movimiento completado correctamente.')
                self.step_index += 1
                self.busy = False
            else:
                self.publish_status(
                    f'Error en trayectoria. '
                    f'Status={response.status}, '
                    f'Codigo={result.error_code}'
                )
                self.finished = True

        except Exception as error:
            self.publish_status(f'Error del controlador: {error}')
            self.finished = True

    def move_gripper(self, operation):
        msg = Float64MultiArray()

        if operation == 'open':
            msg.data = [0.04, 0.04]
            self.publish_status('Abriendo gripper.')
        else:
            msg.data = [0.004, 0.004]
            self.publish_status('Cerrando gripper.')

        self.gripper_pub.publish(msg)

        # Espera de asentamiento para el gripper simulado
        self.wait_until = time.monotonic() + 2.0
        self.busy = True

    def run_sequence(self):
        if self.finished:
            return

        if self.busy:
            if self.wait_until > 0.0:
                if time.monotonic() >= self.wait_until:
                    self.wait_until = 0.0
                    self.busy = False
                    self.step_index += 1
            return

        if self.step_index >= len(self.steps):
            self.finished = True
            self.publish_status('Proceso terminado.')
            return

        operation, value = self.steps[self.step_index]

        if operation == 'arm':
            self.move_arm(value)

        elif operation == 'gripper':
            self.move_gripper(value)

        elif operation == 'message':
            self.publish_status(value)
            self.step_index += 1

        elif operation == 'wait_input':
            if self.input_received:
                self.publish_status('Entrada digital confirmada.')
                self.input_received = False
                self.step_index += 1

        elif operation == 'output':
            self.set_digital_output(value)
            self.publish_status(
                f'Salida digital activada: {value}'
            )
            self.step_index += 1


def main(args=None):
    rclpy.init(args=args)

    node = UR5PickPlace()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
