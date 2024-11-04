"""
Turtlesimを使って，ロボットを30cm前進して時計回りに90度旋回する動作を繰り返すプログラムを作成せよ．
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class MoveTurtlesim(Node):
    """
    Turtlesimを操作するノード
    """
    def __init__(self):
        """
        Publisherの設定
        """
        # ノード名の設定
        super().__init__('move_turtlesim')
        # Publisherの設定 (Topic Name: /turtle1/cmd_vel, Message Type: Twist, Queue Size: 10)
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # タイマーの設定 (1秒ごとにコールバック関数を呼び出し)
        self.timer = self.create_timer(1, self.timer_callback)

        self.sub = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        # カウント変数の初期化
        self.count = 0
        # 速度の初期化
        self.twist = Twist()

    def timer_callback(self):
        """
        メッセージをPublishするコールバック関数
        """
        # 30cm前進して時計回りに90度旋回する動作を繰り返す
        if self.count % 4 == 0:
            # 30cm前進
            self.twist.linear.x = 4.0
            self.twist.angular.z = 0.0
        elif self.count % 4 == 1:
            # 90度旋回
            self.twist.linear.x = 0.0
            self.twist.angular.z = 1.57
        elif self.count % 4 == 2:
            # 30cm前進
            self.twist.linear.x = 4.0
            self.twist.angular.z = 0.0
        elif self.count % 4 == 3:
            # 90度旋回
            self.twist.linear.x = 0.0
            self.twist.angular.z = 1.57
        # メッセージをPublish
        self.pub.publish(self.twist)
        # カウント変数の更新
        self.count += 1

        # ログの出力 (Optional)
        self.get_logger().info('Publishing: {}'.format(self.twist))

    def pose_callback(self, msg):
        self.get_logger().info('(x, y, theta) = ({}, {}, {})'.format(msg.x, msg.y, msg.theta))

def main(args=None):
    """
    ノードの実行を行うmain関数
    """
    # ノードの初期化
    rclpy.init(args=args)
    # インスタンスの作成
    move_turtlesim = MoveTurtlesim()
    # ノードの実行
    rclpy.spin(move_turtlesim)
    # ノードの終了処理
    move_turtlesim.destroy_node()
    # プロセスの終了
    rclpy.shutdown()

# main関数の呼び出し
if __name__ == '__main__':
    main()