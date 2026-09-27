import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup

class FSMNode(Node):
    def __init__(self):
        super().__init__('fsm_node')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info('fsm_node iniciado (stub)')
        self.timer = self.create_timer(3.0, self.enviar_goal)

    def enviar_goal(self):
        self.timer.cancel()  # solo lo manda una vez, para no saturar el log
        self.get_logger().info('Esperando a move_group...')
        self._action_client.wait_for_server()
        goal_msg = MoveGroup.Goal()  # goal vacío (stub, sin pose real todavía)
        self.get_logger().info('Enviando goal (stub) a move_group')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info('Goal aceptado por move_group')
        else:
            self.get_logger().info('Goal rechazado')

def main(args=None):
    rclpy.init(args=args)
    node = FSMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()