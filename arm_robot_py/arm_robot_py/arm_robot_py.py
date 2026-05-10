#!/usr/bin/env python3
"""
Nodo arm_robot_py adaptado para simulación con MoveIt2.

Lee:
  - /home/.../contador_objetos.txt → número de vasos detectados
  - /home/.../object_point_from_robot.txt → coordenadas del vaso

Mueve el robot WX250 en simulación usando MoveIt2 para coger los vasos
y depositarlos en una posición fija.
"""

import os
import ast
import time
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from pymoveit2 import MoveIt2, MoveIt2Gripper
from arm_robot_py import wx250_robot as robot


# Rutas absolutas a los archivos compartidos
WORKSPACE_DIR = '/media/jtaboadab/L/Proyectos/decmani_sim_ws'
CONTADOR_FILE = os.path.join(
    WORKSPACE_DIR, 'src', 'decmani_sim',
    'detectron2_py', 'detectron2_py', 'contador_objetos.txt'
)
OBJECT_FILE = os.path.join(
    WORKSPACE_DIR, 'src', 'decmani_sim',
    'coord_trans_py', 'coord_trans_py', 'object_point_from_robot.txt'
)


class ArmRobotNode(Node):
    """Nodo que controla el brazo robótico mediante MoveIt2."""

    def __init__(self):
        super().__init__('arm_robot_node')

        callback_group = ReentrantCallbackGroup()

        # Interfaz MoveIt2 para el brazo
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )

        # Interfaz MoveIt2 para el gripper
        self.moveit2_gripper = MoveIt2Gripper(
            node=self,
            gripper_joint_names=robot.gripper_joint_names(),
            open_gripper_joint_positions=robot.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=robot.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=robot.MOVE_GROUP_GRIPPER,
            callback_group=callback_group,
        )

        # Velocidad y aceleración por defecto (escalado)
        self.moveit2.max_velocity = 0.5
        self.moveit2.max_acceleration = 0.5

        # Tiempo máximo para que OMPL encuentre una trayectoria
        self.moveit2.allowed_planning_time = 10.0
        # Intentos de planificación paralelos (OMPL lanza threads)
        self.moveit2.num_planning_attempts = 10

        self.get_logger().info("Nodo arm_robot_node listo")

    def go_to_pose(self, x, y, z, qx=0.0, qy=0.7071, qz=0.0, qw=0.7071):
        """Mueve el efector final a una pose (x,y,z) usando IK + move_to_configuration.
        
        Workaround: pymoveit2.move_to_pose tiene un bug con OMPL que impide
        samplear estados goal. Resolvemos IK manualmente y movemos por joints.
        """
        self.get_logger().info(f"Moviendo a pose: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        
        # Resolver IK manualmente
        joints = self.moveit2.compute_ik(
            position=[x, y, z],
            quat_xyzw=[qx, qy, qz, qw],
        )
        
        if joints is None:
            self.get_logger().warn(f"IK FAILED para pose ({x:.3f}, {y:.3f}, {z:.3f})")
            return
        
        # Extraer solo los 5 joints del brazo
        arm_joints = list(joints.position[:5])
        self.get_logger().info(f"IK OK. Joints: {[round(j, 3) for j in arm_joints]}")
        
        # Mover por configuración (evita el bug de OMPL con pose goals)
        self.moveit2.move_to_configuration(joint_positions=arm_joints)
        self.moveit2.wait_until_executed()

    def go_to_named(self, name):
        """Mueve el robot a una pose nombrada (home, sleep)."""
        self.get_logger().info(f"Moviendo a pose nombrada: {name}")
        
        # Posiciones definidas en el SRDF
        poses = {
            "home":  [0.0, -1.88, 1.5, 0.8, 0.0],
            "sleep": [0.0, -1.85, 1.55, 0.8, 0.0],
        }
        
        if name not in poses:
            self.get_logger().error(f"Pose desconocida: {name}")
            return
        
        joint_positions = poses[name]
        self.moveit2.move_to_configuration(joint_positions)
        self.moveit2.wait_until_executed()

    def gripper_open(self):
        """Abre el gripper."""
        self.get_logger().info("Abriendo gripper")
        self.moveit2_gripper.open()
        self.moveit2_gripper.wait_until_executed()

    def gripper_close(self):
        """Cierra el gripper."""
        self.get_logger().info("Cerrando gripper")
        self.moveit2_gripper.close()
        self.moveit2_gripper.wait_until_executed()

    def leer_contador(self):
        """Lee el número de objetos detectados."""
        try:
            with open(CONTADOR_FILE, 'r') as f:
                return ast.literal_eval(f.read())
        except Exception as e:
            self.get_logger().warn(f"No se pudo leer contador: {e}")
            return 0

    def leer_objeto(self):
        """Lee la posición del objeto detectado."""
        try:
            with open(OBJECT_FILE, 'r') as f:
                lista = ast.literal_eval(f.read())
                return lista[0], lista[1], lista[2]
        except Exception as e:
            self.get_logger().warn(f"No se pudo leer posición: {e}")
            return 0.0, 0.0, 0.0


def ciclo_principal(node):
    """Ciclo principal de la aplicación."""
    rclpy.spin_once(node, timeout_sec=2.0)
    
    while rclpy.ok():
        input("\n>>> Pulsa ENTER para iniciar ciclo de recogida...")

        desp = -0.08

        # Ir a home
        node.gripper_close()
        time.sleep(1)
        node.gripper_open()
        time.sleep(1)
        node.go_to_named("home")
        time.sleep(1)

        while rclpy.ok():
            # Verificar si hay objetos
            contador = node.leer_contador()
            node.get_logger().info(f"Objetos detectados: {contador}")

            if contador == 0:
                node.get_logger().info("No hay más objetos, volviendo a sleep")
                node.go_to_named("home")
                node.go_to_named("sleep")
                break

            input("\n>>> Pulsa ENTER para recoger el vaso...")
            # Leer posición del objeto
            x, y, z = node.leer_objeto()
            node.get_logger().info(f"Vaso en: ({x:.3f}, {y:.3f}, {z:.3f})")

            # Acercarse al vaso (z=0.25)
            node.go_to_pose(x, y, 0.20)

            # Bajar al vaso (z=0.1)
            node.go_to_pose(x, y, 0.15)

            # Cerrar gripper
            node.gripper_close()
            time.sleep(1)

            # Subir
            node.go_to_pose(x, y, 0.25)

            """# Mover a posición de depósito
            node.go_to_pose(0.4, -desp, 0.30)
            node.go_to_pose(0.4, -desp, 0.16)"""

            # Bajar
            node.go_to_pose(x, y, 0.15)

            # Soltar
            node.gripper_open()
            time.sleep(1)

            # Subir
            # node.go_to_pose(0.4, -desp, 0.30)

            node.go_to_named("home")

            # Siguiente posición de depósito
            desp += 0.08


def main():
    rclpy.init()
    node = ArmRobotNode()

    # Ejecutar el spin en otro thread
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        ciclo_principal(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()