"""
以下の条件に従って，Int64型をPublishするプログラムと，それをSubscribeするプログラムを各自作成したros2_tutorial_pkgに作成し，実行せよ．
- subscriber.pyの作成
    - 使用するメッセージ型「std_msgs/msg/Int64」
    - Subscriberの設定
        - Topic Name: /talker/int64
        - Message Type: Int64
        - Callback Fuction: listener_callback
"""

#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

class Int64Subscriber(Node):
    """
    Int64型のメッセージをSubscribeするノード
    """
    def __init__(self):
        """
        Subscriberの設定
        """
        # ノード名の設定
        super().__init__('int64_subscriber')
        # Subscriberの設定 (Topic Name: /talker/int64, Message Type: Int64, Callback Fuction: listener_callback)
        self.subscription = self.create_subscription(Int64, '/talker/int64', self.listener_callback, 10)
        # ログの出力 (Optional)
        self.get_logger().info('Subscribing: /talker/int64')

    def listener_callback(self, msg):
        """
        メッセージをSubscribeするコールバック関数
        """
        # ログの出力 (Optional)
        self.get_logger().info('I heard: {}'.format(msg.data))

def main(args=None):
    """
    ノードの実行を行うmain関数
    """
    # ノードの初期化
    rclpy.init(args=args)
    # インスタンスの作成
    int64_subscriber = Int64Subscriber()
    # ノードの実行
    rclpy.spin(int64_subscriber)
    # ノードの終了処理
    int64_subscriber.destroy_node()
    # ノードの終了
    rclpy.shutdown()

if __name__ == '__main__':
    main()