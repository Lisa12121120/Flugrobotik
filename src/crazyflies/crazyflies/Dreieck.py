import rclpy
from rclpy.node import Node
from crazyflie_interfaces.msg import Position
import time

class Dreieck(Node):
    def __init__(self):
        super().__init__("dreieck_flight")

        self.pub = self.create_publisher(Position, "/cf0/cmd_position", 10)

        time.sleep(1.0)
        self.get_logger().info("Starte Dreiecksflug...")

        self.fly_triangle()

    def send_position(self, x, y, z, yaw=0.0):
        msg = Position()
        msg.x = x
        msg.y = y
        msg.z = z
        msg.yaw = yaw
        self.pub.publish(msg)

    def fly_triangle(self):
        height = 0.5
        delay  = 2.0

        points = [
            (0.5, 0.0, height),
            (0.0, 0.5, height),
            (-0.5, 0.0, height),
        ]

        for (x, y, z) in points:
            self.get_logger().info(f"Fliege zu Punkt: {x}, {y}")
            self.send_position(x, y, z)
            time.sleep(delay)

        self.get_logger().info("Zurück zur Mitte")
        self.send_position(0.0, 0.0, height)
        time.sleep(delay)

        self.get_logger().info("Dreieck abgeschlossen.")

def main(args=None):
    rclpy.init(args=args)
    node = Dreieck()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
