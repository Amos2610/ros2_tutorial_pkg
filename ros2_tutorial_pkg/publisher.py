"""
以下の条件に従って，Int64型をPublishするプログラムと，それをSubscribeするプログラムを各自作成したros2_tutorial_pkgに作成し，実行せよ．

- publisher.pyの作成
    - 使用するメッセージ型「std_msgs/Int64」をインポート
    - Publisherの設定
        - Topic Name: /talker/int64
        - Message Type: Int64
        - Queue Size: 10
"""

#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

class Int64Publisher(Node):
    """
    Int64型のメッセージをPublishするノード
    """
    def __init__(self):
        """
        Publisherの設定
        """
        # ノード名の設定
        super().__init__('int64_publisher')
        # Publisherの設定 (Topic Name: /talker/int64, Message Type: Int64, Queue Size: 10)
        self.publisher = self.create_publisher(Int64, '/talker/int64', 10)
        # タイマーの設定 (0.5秒ごとにコールバック関数を呼び出し)
        self.timer = self.create_timer(0.5, self.timer_callback)
        # カウント変数の初期化
        self.count = 0

    def timer_callback(self):
        """
        メッセージをPublishするコールバック関数
        """
        # Int64型のメッセージを作成
        msg = Int64()
        # メッセージにデータを代入
        msg.data = self.count
        # メッセージをPublish
        self.publisher.publish(msg)
        # ログの出力 (Optional)
        self.get_logger().info('Publishing: {}'.format(msg.data))
        # カウント変数の更新
        self.count += 1

def main(args=None):
    """
    ノードの実行を行うmain関数
    """
    # ノードの初期化
    rclpy.init(args=args)
    # インスタンスの作成
    int64_publisher = Int64Publisher()
    # ノードの実行
    rclpy.spin(int64_publisher)
    # ノードの終了処理
    int64_publisher.destroy_node()
    # プロセスの終了
    rclpy.shutdown()

# main関数の呼び出し
if __name__ == '__main__':
    main()