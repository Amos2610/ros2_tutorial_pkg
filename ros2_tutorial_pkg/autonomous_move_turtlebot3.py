"""
課題５: Turtlebot3で自律走行させるプログラムを作成せよ．
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import signal
import sys
import math


class TurtleBot3Autonomous(Node):
    def __init__(self):
        """
        TurtleBot3で自律走行するノード
        """
        # ノード名の設定
        super().__init__('turtlebot3_autonomous')
        
        # パブリッシャーとサブスクライバーの設定
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )

        # タイマーの設定
        self.timer = self.create_timer(0.1, self.control_loop)

        # ロボットの停止
        self.stop_robot()

        # 変数の初期化
        self.twist = Twist()
        self.scan_data = [float('inf'), float('inf'), float('inf')]  # 中央、左、右方向の距離データ
        self.check_forward_dist = 0.4  # 前方障害物の閾値
        self.check_side_dist = 0.3  # 側方障害物の閾値
        self.escape_range = math.radians(30)  # 回避動作の回転範囲
        self.tb3_pose = 0.0  # 現在の向き（シミュレート）
        self.prev_tb3_pose = 0.0  # 前回の向き
        self.state = "GET_TB3_DIRECTION"  # 初期ステート

    def laser_callback(self, msg):
        """
        レーザースキャンデータのコールバック関数
        レーザースキャンデータから、指定した角度のデータを取得（正面0度、左30度、右330度）
        """
        # レーザースキャンデータの取得角度
        scan_angles = [0, 30, 330]
        # レーザースキャンデータの取得
        for i, angle in enumerate(scan_angles):
            # 角度からインデックスに変換
            index = int(angle * len(msg.ranges) / 360)
            # インデックスが範囲内の場合
            if 0 <= index < len(msg.ranges):
                # 障害物までの距離を取得
                self.scan_data[i] = msg.ranges[index] if not math.isinf(msg.ranges[index]) else msg.range_max

    def update_command_velocity(self, linear, angular):
        """
        速度指令の更新を行う関数
        """
        # 速度指令の更新
        self.twist.linear.x = linear
        self.twist.angular.z = angular
        # 速度指令のパブリッシュ
        self.publisher.publish(self.twist)

    def control_loop(self):
        """
        制御ループ(ステートマシン)の関数
            ステートマシンの制御ロジック (GET_TB3_DIRECTION -> TB3_DRIVE_FORWARD -> GET_TB3_DIRECTION)
        """
        # 現在の向きを取得するステート
        if self.state == "GET_TB3_DIRECTION": 
            # 前方に障害物がない場合
            if self.scan_data[0] > self.check_forward_dist:
                if self.scan_data[1] < self.check_side_dist: # 側方(30度)に障害物がある場合
                    self.prev_tb3_pose = self.tb3_pose
                    self.state = "TB3_RIGHT_TURN" # 右回転
                elif self.scan_data[2] < self.check_side_dist: # 側方(330度)に障害物がある場合
                    self.prev_tb3_pose = self.tb3_pose
                    self.state = "TB3_LEFT_TURN"  # 左回転
                # 右にも左にも障害物がない場合, 前進
                else: 
                    self.state = "TB3_DRIVE_FORWARD" 

            # 前方に障害物がある場合
            else:
                self.prev_tb3_pose = self.tb3_pose 
                self.state = "TB3_RIGHT_TURN" # 右回転
        
        # 前進するステート
        elif self.state == "TB3_DRIVE_FORWARD": 
            print("TB3_DRIVE_FORWARD")
            self.update_command_velocity(0.2, 0.0)  # 前進
            self.state = "GET_TB3_DIRECTION"

        # 右回転するステート
        elif self.state == "TB3_RIGHT_TURN":
            print("TB3_RIGHT_TURN")
            self.update_command_velocity(0.0, -0.5)  # 右回転
            self.tb3_pose -= math.radians(5)  # シミュレートされた回転
            if abs(self.tb3_pose - self.prev_tb3_pose) >= self.escape_range:
                self.state = "GET_TB3_DIRECTION"
        # 左回転するステート
        elif self.state == "TB3_LEFT_TURN":
            print("TB3_LEFT_TURN")
            self.update_command_velocity(0.0, 0.5)  # 左回転
            self.tb3_pose += math.radians(5)  # シミュレートされた回転
            if abs(self.tb3_pose - self.prev_tb3_pose) >= self.escape_range:
                self.state = "GET_TB3_DIRECTION"

    def stop_robot(self):
        """
        ロボットを停止する関数
        """
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info("Stop Robot")

def signal_handler(sig, frame, cls):
    """
    シグナルハンドラの関数
    """
    # ロボットの停止
    cls.stop_robot()
    # ノードの終了処理
    cls.destroy_node()

    # プロセスの終了
    rclpy.shutdown()
    sys.exit(0)

def main(args=None):
    """
    ノードの実行を行うmain関数
    """
    # ノードの初期化
    rclpy.init(args=args)
    # インスタンスの作成
    turtlebot3_autonomous = TurtleBot3Autonomous()

    # シグナルハンドラの設定
    signal.signal(signal.SIGINT, lambda signum, frame: signal_handler(signum, frame, turtlebot3_autonomous))
    
    # ノードの実行
    rclpy.spin(turtlebot3_autonomous)

if __name__ == '__main__':
    main()
