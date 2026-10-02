#!/usr/bin/env python3
"""
位置控制版的 FollowPath action server，取代 nav2_controller 的 MPPIController.

標準 Nav2 的 nav2_core::Controller plugin 介面規定輸出一定是速度
(geometry_msgs/Twist)，沒辦法透過官方 plugin 機制直接輸出位置setpoint。
這裡改成寫一個獨立 node，自己實作 FollowPath action 的 server 端(跟
controller_server 正常提供的是同一個 action，bt_navigator 感覺不到差異)，
收到 planner_server/smoother_server 規劃好的路徑後，依序把每個路徑點當
PX4 TrajectorySetpoint 的 position 送出去 —— 純位置控制，不經過
機體/世界座標速度旋轉這一整套，PX4 自己的位置控制環負責飛過去。

因為是位置控制，目標點(map frame x,y)跟世界座標(PX4 NED n,e)之間只有一次
固定的軸對應(map.x=n, map.y=-e，跟 px4_odom_tf.py 一致)，完全不需要知道
機體當下朝向 —— px4_nav2_bridge 專案裡一路在查的 yaw 回授不可信問題，
對這個架構來說不會影響移動方向對不對。

同時吸收了原本 cmd_vel_bridge.py 的角色：20Hz heartbeat、OFFBOARD+ARM
服務、沒有 active goal 時的懸停保持、geofence。velocity_smoother、
/cmd_vel 這些中間層對位置控制沒有意義，不再需要。

使用：
  ros2 run nav2_test_pos position_controller --ros-args -p target_alt:=1.0
  ros2 service call /position_controller/start std_srvs/srv/Trigger
  ros2 service call /position_controller/land  std_srvs/srv/Trigger
"""
import math
import time

from nav2_msgs.action import FollowPath
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger

NAN = float('nan')


class PositionController(LifecycleNode):
    def __init__(self):
        super().__init__('controller_server')

        # ---------- 參數（大部分沿用 cmd_vel_bridge.py 的慣例） ----------
        self.declare_parameter('topic_offboard_mode', '/fmu/in/offboard_control_mode')
        self.declare_parameter('topic_setpoint', '/fmu/in/trajectory_setpoint')
        self.declare_parameter('topic_command', '/fmu/in/vehicle_command')
        self.declare_parameter('topic_odometry', '/fmu/out/vehicle_odometry')
        self.declare_parameter('target_alt', 1.0)          # 飛行高度 [m]，向上為正
        self.declare_parameter('waypoint_tolerance', 0.2)  # [m] 路徑中間點，多近算到了
        self.declare_parameter('xy_goal_tolerance', 0.15)  # [m] 最終目標點，多近算抵達
        self.declare_parameter('odom_timeout', 0.3)        # [s] 沒收到 odometry 就懸停
        # progress checker：required_movement_radius 內，movement_time_allowance
        # 秒內沒有移動這麼多，就判定「卡住了」，abort 整個 action
        self.declare_parameter('required_movement_radius', 0.2)
        self.declare_parameter('movement_time_allowance', 20.0)
        # geofence，map 座標 [m]
        self.declare_parameter('fence_x_min', -6.0)
        self.declare_parameter('fence_x_max', 6.0)
        self.declare_parameter('fence_y_min', -6.0)
        self.declare_parameter('fence_y_max', 6.0)

        def p(n):
            return self.get_parameter(n).value

        self.target_alt = p('target_alt')
        self.waypoint_tolerance = p('waypoint_tolerance')
        self.xy_goal_tolerance = p('xy_goal_tolerance')
        self.odom_timeout = p('odom_timeout')
        self.required_movement_radius = p('required_movement_radius')
        self.movement_time_allowance = p('movement_time_allowance')
        self.fence = (p('fence_x_min'), p('fence_x_max'), p('fence_y_min'), p('fence_y_max'))

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---------- Publishers ----------
        self.pub_mode = self.create_publisher(
            OffboardControlMode, p('topic_offboard_mode'), px4_qos)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, p('topic_setpoint'), px4_qos)
        self.pub_cmd = self.create_publisher(VehicleCommand, p('topic_command'), px4_qos)

        # ---------- Subscribers ----------
        self.create_subscription(VehicleOdometry, p('topic_odometry'), self.on_odom, px4_qos)

        # ---------- Services（跟原本 cmd_vel_bridge.py 一樣） ----------
        self.create_service(Trigger, '~/start', self.srv_start)
        self.create_service(Trigger, '~/land', self.srv_land)

        # ---------- 狀態 ----------
        self.pos_ned = None           # [n, e, d]，來自 vehicle_odometry
        self.last_odom_t = None
        self.hold_ne = None            # 沒有 active goal 時，鎖住的 [n, e]
        self.target_xy = None          # 目前 action 正在瞄準的 map 座標 (x, y)，None=沒有在導航
        self.active = False            # lifecycle 是否已經 activate

        cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            FollowPath,
            'follow_path',
            execute_callback=self.execute_follow_path,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=cb_group,
        )

        self.create_timer(0.05, self.px4_loop, callback_group=cb_group)  # 20 Hz

        self.get_logger().info('position_controller ready (lifecycle node)')

    # ================================================================
    # Lifecycle
    # ================================================================
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('on_configure')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('on_activate')
        self.active = True
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('on_deactivate')
        self.active = False
        self.target_xy = None
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    # ================================================================
    # Odometry callback
    # ================================================================
    def on_odom(self, msg: VehicleOdometry):
        self.pos_ned = [float(msg.position[0]), float(msg.position[1]), float(msg.position[2])]
        self.last_odom_t = self.now_s()

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def current_map_xy(self):
        """Convert PX4 NED(n,e) to map 慣例(map.x=n, map.y=-e)，跟 px4_odom_tf.py 一致."""
        n, e, _d = self.pos_ned
        return n, -e

    def map_xy_to_ne(self, x, y):
        """Convert map 慣例(x,y) back to PX4 NED(n,e)."""
        return x, -y

    def apply_fence(self, x, y):
        x_min, x_max, y_min, y_max = self.fence
        x = max(x_min, min(x_max, x))
        y = max(y_min, min(y_max, y))
        return x, y

    # ================================================================
    # PX4 20Hz heartbeat：永遠在送，不管有沒有 active goal
    # ================================================================
    def px4_loop(self):
        odom_ok = (self.last_odom_t is not None
                   and (self.now_s() - self.last_odom_t) < self.odom_timeout)

        sp = TrajectorySetpoint()
        sp.timestamp = self.px4_ts()
        z_ned = -self.target_alt

        if self.target_xy is not None and odom_ok:
            self.hold_ne = None
            tx, ty = self.apply_fence(*self.target_xy)
            n, e = self.map_xy_to_ne(tx, ty)
            sp.position = [n, e, z_ned]
        else:
            # 沒有 active goal，或 odometry 逾時 -> 鎖住當下位置懸停
            if self.hold_ne is None and self.pos_ned is not None:
                self.hold_ne = self.pos_ned[:2]
                if not odom_ok:
                    self.get_logger().warn('odometry 逾時，懸停', throttle_duration_sec=2.0)
            if self.hold_ne is not None:
                sp.position = [self.hold_ne[0], self.hold_ne[1], z_ned]
            else:
                sp.position = [NAN, NAN, z_ned]

        sp.velocity = [NAN, NAN, NAN]
        sp.yaw = NAN
        sp.yawspeed = 0.0

        mode = OffboardControlMode()
        mode.timestamp = sp.timestamp
        mode.position = True
        mode.velocity = False
        self.pub_mode.publish(mode)
        self.pub_sp.publish(sp)

    # ================================================================
    # FollowPath action
    # ================================================================
    def goal_callback(self, goal_request):
        if not self.active:
            return GoalResponse.REJECT
        if not goal_request.path.poses:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute_follow_path(self, goal_handle):
        waypoints = [(ps.pose.position.x, ps.pose.position.y)
                     for ps in goal_handle.request.path.poses]
        goal_x, goal_y = waypoints[-1]
        idx = 0

        feedback = FollowPath.Feedback()
        result = FollowPath.Result()

        last_progress_t = self.now_s()
        last_progress_xy = None

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.target_xy = None
                goal_handle.canceled()
                result.error_code = FollowPath.Result.NONE
                return result

            if self.pos_ned is None:
                time.sleep(0.1)
                continue

            cur_x, cur_y = self.current_map_xy()
            tx, ty = waypoints[idx]
            self.target_xy = (tx, ty)

            dist_to_wp = math.hypot(tx - cur_x, ty - cur_y)
            tol = self.xy_goal_tolerance if idx == len(waypoints) - 1 else self.waypoint_tolerance
            if dist_to_wp < tol:
                if idx < len(waypoints) - 1:
                    idx += 1
                else:
                    self.target_xy = None
                    goal_handle.succeed()
                    result.error_code = FollowPath.Result.NONE
                    return result

            # progress checker：required_movement_radius 內，movement_time_allowance
            # 秒內移動不夠多就判定卡住
            now = self.now_s()
            if last_progress_xy is None:
                last_progress_xy = (cur_x, cur_y)
                last_progress_t = now
            elif math.hypot(
                    cur_x - last_progress_xy[0],
                    cur_y - last_progress_xy[1]) >= self.required_movement_radius:
                last_progress_xy = (cur_x, cur_y)
                last_progress_t = now
            elif now - last_progress_t > self.movement_time_allowance:
                self.target_xy = None
                goal_handle.abort()
                result.error_code = FollowPath.Result.FAILED_TO_MAKE_PROGRESS
                result.error_msg = 'required_movement_radius 內沒有足夠移動'
                return result

            feedback.distance_to_goal = math.hypot(goal_x - cur_x, goal_y - cur_y)
            feedback.speed = 0.0
            goal_handle.publish_feedback(feedback)

            time.sleep(0.1)

        self.target_xy = None
        goal_handle.abort()
        result.error_code = FollowPath.Result.UNKNOWN
        return result

    # ================================================================
    # 指令（跟原本 cmd_vel_bridge.py 一樣）
    # ================================================================
    def px4_ts(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def send_cmd(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.timestamp = self.px4_ts()
        m.command = command
        m.param1 = float(p1)
        m.param2 = float(p2)
        m.target_system = 0  # 廣播，見 cmd_vel_bridge.py 原本的說明
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        self.pub_cmd.publish(m)

    def srv_start(self, req, res):
        if self.last_odom_t is None:
            res.success, res.message = False, '還沒收到 odometry'
            return res
        self.hold_ne = self.pos_ned[:2]
        self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        res.success, res.message = True, 'Offboard + arm 已送出'
        return res

    def srv_land(self, req, res):
        self.target_xy = None
        self.send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        res.success, res.message = True, 'Land 已送出'
        return res


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = PositionController()
    # 預設 MultiThreadedExecutor() 的執行緒數等於 CPU 核心數（RPi4 約4個）。
    # execute_follow_path() 每個 goal 執行期間會整個佔用一個執行緒直到結束
    # (while 迴圈 + time.sleep)，如果前一個 goal 因為 bt_navigator 端逾時放棄、
    # 但 server 端其實沒有真的被取消、還在背景跑，執行緒就不會釋放，連續失敗
    # 幾次後執行緒被佔滿，新 goal 連「確認收到」都要排隊等很久。多給一點執行緒
    # 當緩衝，不要讓這種情況輕易卡死。
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
