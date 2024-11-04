import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class KeyboardTurtlebot3(Node):
    """
    Turtlebot3でキーボード入力によって動作するプログラム
    """
    def __init__(self):
        # Nodeの初期化
        super().__init__('keyboard_turtlebot3')
        # Publisherの作成
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # Twistメッセージの初期化
        self.twist = Twist()

        # keyboard_turtlebot3ノードの開始メッセージ
        self.get_logger().info("Keyboard Turtlebot3 started.")
        # キーボード入力のヘルプメッセージ
        self.get_logger().info("Press the following keys to move the Turtlebot3:")
        self.get_logger().info("w: Forward, x: Backward, a: Left, d: Right, s: Stop, 1: Rotate, q: Quit")

    def get_key(self):
        """キー入力を取得
        Returns:
            str: キー入力
        """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def process_key(self, key):
        """キー入力に応じてTwistメッセージを設定"""
        if key == 'w':
            # 前進
            self.twist.linear.x = 0.1
            self.twist.angular.z = 0.0
        elif key == 'x':
            # 後退
            self.twist.linear.x = -0.1
            self.twist.angular.z = 0.0
        elif key == 'a':
            # 左旋回
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.1
        elif key == 'd':
            # 右旋回
            self.twist.linear.x = 0.0
            self.twist.angular.z = -0.1
        elif key == 's':
            # 停止
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
        elif key == '1':
            # 一回転する
            self.twist.linear.x = 0.0
            self.twist.angular.z = 3.14
        elif key == 'q':
            # 終了
            self.get_logger().info("Shutdown command received")
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            rclpy.shutdown()

        # メッセージをパブリッシュ
        self.publisher_.publish(self.twist)

    def run(self):
        """キー入力をループで処理"""
        try:
            while rclpy.ok():
                key = self.get_key()
                self.process_key(key)
                # キーログ
                self.get_logger().info(f"Key pressed: {key}")
        except KeyboardInterrupt:
            self.get_logger().info("Keyboard Interrupt received, shutting down...")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTurtlebot3()

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
