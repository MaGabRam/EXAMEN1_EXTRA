#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import numpy as np
import time


class Indice:
    def __init__(self):
        self.q = np.array([-0.22, 0.7, 0.03])
        self.l1, self.l2, self.l3 = 2.0, 1.5, 0.8
        self.step_size = 0.05
        self.tolerance = 0.01
        self.damping_factor = 0.1
        self.target_pos = np.array([4.0, 2.0, 1.5])

    def forward_kinematics(self, q):
        q1, q2, q3 = q
        x = self.l1*np.cos(q1) + self.l2*np.cos(q1 + q2) + self.l3*np.cos(q1 + q2 + q3)
        y = self.l1*np.sin(q1) + self.l2*np.sin(q1 + q2) + self.l3*np.sin(q1 + q2 + q3)
        z = 0
        return np.array([x, y, z])

    def jacobian(self, q):
        q1, q2, q3 = q
        j11 = -self.l1*np.sin(q1) - self.l2*np.sin(q1 + q2) - self.l3*np.sin(q1 + q2 + q3)
        j12 = -self.l2*np.sin(q1 + q2) - self.l3*np.sin(q1 + q2 + q3)
        j13 = -self.l3*np.sin(q1 + q2 + q3)
        j21 =  self.l1*np.cos(q1) + self.l2*np.cos(q1 + q2) + self.l3*np.cos(q1 + q2 + q3)
        j22 =  self.l2*np.cos(q1 + q2) + self.l3*np.cos(q1 + q2 + q3)
        j23 =  self.l3*np.cos(q1 + q2 + q3)
        return np.array([[j11, j12, j13], [j21, j22, j23], [0, 0, 0]])

    def update(self, logger):
        current_pos = self.forward_kinematics(self.q)
        error = self.target_pos - current_pos
        error_norm = np.linalg.norm(error)

        logger.info(f"[ÍNDICE] Posición actual: {current_pos}")
        logger.info(f"[ÍNDICE] Objetivo: {self.target_pos}")
        logger.info(f"[ÍNDICE] Error: {error_norm:.4f}")

        if error_norm > self.tolerance:
            J = self.jacobian(self.q)
            determinant = np.linalg.det(J)
            logger.info(f"[ÍNDICE] Determinante Jacobiano: {determinant:.4f}")

            JtJ = J.T @ J
            damping = self.damping_factor * np.eye(JtJ.shape[0])
            J_dls = np.linalg.solve(JtJ + damping, J.T) @ error
            self.q += J_dls * self.step_size

        return self.q


class Pulgar:
    def __init__(self):
        self.q = np.array([-0.22, 0.7, 0.03])
        self.l1, self.l2, self.l3 = 0.5, 1.3, 0.9
        self.step_size = 0.05
        self.tolerance = 0.01
        self.damping_factor = 0.1
        self.target_pos = np.array([3.0, 1.5, 0.5])

    def forward_kinematics(self, q):
        q1, q2, q3 = q
        r = self.l2*np.cos(q2) + self.l3*np.cos(q2 + q3)
        x = r*np.cos(q1)
        y = r*np.sin(q1)
        z = self.l2*np.sin(q2) + self.l3*np.sin(q2 + q3) + self.l1
        return np.array([x, y, z])

    def jacobian(self, q):
        q1, q2, q3 = q
        j11 = -(self.l2*np.cos(q2) + self.l3*np.cos(q2+q3)) * np.sin(q1)
        j12 = -(self.l2*np.sin(q2) + self.l3*np.sin(q2+q3)) * np.cos(q1)
        j13 = - self.l3*np.sin(q2+q3) * np.cos(q1)
        j21 =  (self.l2*np.cos(q2) + self.l3*np.cos(q2+q3)) * np.cos(q1)
        j22 = -(self.l2*np.sin(q2) + self.l3*np.sin(q2+q3)) * np.sin(q1)
        j23 = - self.l3*np.sin(q2+q3) * np.sin(q1)
        j31 = 0.0
        j32 =  self.l2*np.cos(q2) + self.l3*np.cos(q2+q3)
        j33 =  self.l3*np.cos(q2+q3)
        return np.array([[j11, j12, j13], [j21, j22, j23], [j31, j32, j33]])

    def update(self, logger):
        current_pos = self.forward_kinematics(self.q)
        error = self.target_pos - current_pos
        error_norm = np.linalg.norm(error)

        logger.info(f"[PULGAR] Posición actual: {current_pos}")
        logger.info(f"[PULGAR] Objetivo: {self.target_pos}")
        logger.info(f"[PULGAR] Error: {error_norm:.4f}")

        if error_norm > self.tolerance:
            J = self.jacobian(self.q)
            determinant = np.linalg.det(J)
            logger.info(f"[PULGAR] Determinante Jacobiano: {determinant:.4f}")

            JtJ = J.T @ J
            damping = self.damping_factor * np.eye(JtJ.shape[0])
            J_dls = np.linalg.solve(JtJ + damping, J.T) @ error
            self.q += J_dls * self.step_size

        return self.q


class InverseKinematics(Node):
    def __init__(self):
        super().__init__('dual_inverse_kinematics')

        self.pub_index = self.create_publisher(JointState, '/index/joint_states', 10)
        self.pub_thumb = self.create_publisher(JointState, '/thumb/joint_states', 10)
        self.sub_index = self.create_subscription(Point, '/index/target_position', self.cb_index, 10)
        self.sub_thumb = self.create_subscription(Point, '/thumb/target_position', self.cb_thumb, 10)

        self.solver_i = Indice()
        self.solver_t = Pulgar()

        self.timer = self.create_timer(0.1, self.update_loop)

    def cb_index(self, msg):
        self.solver_i.target_pos = np.array([msg.x, msg.y, msg.z])

    def cb_thumb(self, msg):
        self.solver_t.target_pos = np.array([msg.x, msg.y, msg.z])

    def update_loop(self):
        qi = self.solver_i.update(self.get_logger())
        qt = self.solver_t.update(self.get_logger())

        now = self.get_clock().now().to_msg()

        msg_i = JointState()
        msg_i.header.stamp = now
        msg_i.name = ['q11', 'q12', 'q13']
        msg_i.position = qi.tolist()
        self.pub_index.publish(msg_i)

        msg_t = JointState()
        msg_t.header.stamp = now
        msg_t.name = ['q20', 'q21', 'q22']
        msg_t.position = qt.tolist()
        self.pub_thumb.publish(msg_t)


def main():
    rclpy.init()
    node = InverseKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
