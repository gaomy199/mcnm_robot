import subprocess
import threading
import os
import math
from enum import Enum
import signal
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class TaskState(Enum):
    IDLE = 0
    NAV_TO_GRASP = 1
    GRASPING = 2
    NAV_TO_HOME = 3
    DONE = 4
    FAILED = 5

class TaskExecutor(Node):
    def __init__(self):
        super().__init__('task_executor')
        # home和target位置参数默认值
        self.declare_parameter('grasp_x', 0.0)
        self.declare_parameter('grasp_y', 0.0)
        self.declare_parameter('grasp_yaw', 0.0)
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('home_yaw', 0.0)
        self.declare_parameter('grasp_script',
                               '/home/joyandai/lerobot/eval_train3_080000_once.sh')
        self.declare_parameter('grasp_timeout_sec', 180)
        self.declare_parameter('return_home', True)

        # Nav2 client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 任务接口
        self.task_sub = self.create_subscription(
            String, '/start_task', self.task_callback, 10)
        self.status_pub = self.create_publisher(String, '/task_status', 10)
        self.state = TaskState.IDLE
        self.grasp_process = None
        self.get_logger().info('Task executor ready. Send to /start_task to begin.')

    # 工具
    def _make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _publish_status(self, msg):
        self.status_pub.publish(String(data=msg))
        self.get_logger().info(f'[STATUS] {msg}')

    def _get_pose_param(self, prefix):
        x = self.get_parameter(f'{prefix}_x').value
        y = self.get_parameter(f'{prefix}_y').value
        yaw = self.get_parameter(f'{prefix}_yaw').value
        return self._make_pose(x, y, yaw)

    # 入口
    def task_callback(self, msg):
        if self.state not in (TaskState.IDLE, TaskState.DONE, TaskState.FAILED):
            self.get_logger().warn(f'Busy ({self.state.name}), ignore.')
            return
        self.get_logger().info(f'New task: {msg.data}')
        # 阶段1测试，跳过导航直接抓取
        # self._start_grasp()      # 改成这个，直接调用grasp
        self._goto_grasp()     # 真实的导航+抓取 恢复这一行

    # 阶段 1: 导航到抓取点
    def _goto_grasp(self):
        self.state = TaskState.NAV_TO_GRASP
        self._publish_status('nav_to_grasp')
        self._send_nav_goal(self._get_pose_param('grasp'),
                            on_success=self._start_grasp,
                            on_fail=self._task_failed)

    # 阶段 2: 启动 lerobot 抓取
    def _start_grasp(self):
        self.state = TaskState.GRASPING
        self._publish_status('grasping')
        threading.Thread(target=self._run_grasp_subprocess, daemon=True).start()

    def _run_grasp_subprocess(self):
        script = self.get_parameter('grasp_script').value
        timeout = self.get_parameter('grasp_timeout_sec').value

        try:
            self.get_logger().info(f'Run: bash {script}')
            self.grasp_process = subprocess.Popen(
                ['bash', script],
                env=os.environ.copy(),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                preexec_fn=os.setsid,
            )
            try:
                stdout, _ = self.grasp_process.communicate(timeout=timeout)
                rc = self.grasp_process.returncode
                if rc == 0:
                    self.get_logger().info('Grasp completed (rc=0)')
                    if self.get_parameter('return_home').value:
                        self._goto_home()
                    else:
                        self._task_done()
                else:
                    self.get_logger().error(f'Grasp script rc={rc}, output tail:\n'
                                            f'{stdout[-1000:] if stdout else ""}')
                    self._task_failed()
            except subprocess.TimeoutExpired:
                self.get_logger().error(f'Grasp timeout {timeout}s, killing process group')
                # 杀整个 process group
                os.killpg(os.getpgid(self.grasp_process.pid), signal.SIGKILL)
                self.grasp_process.wait()
                self._task_failed()
        except Exception as e:
            self.get_logger().error(f'Grasp exception: {e}')
            self._task_failed()
        finally:
            self.grasp_process = None

    # 阶段 3: 返回原点
    def _goto_home(self):
        self.state = TaskState.NAV_TO_HOME
        self._publish_status('nav_to_home')
        self._send_nav_goal(self._get_pose_param('home'),
                            on_success=self._task_done,
                            on_fail=self._task_failed)

    # 终态
    def _task_done(self):
        self.state = TaskState.DONE
        self._publish_status('done')

    def _task_failed(self):
        self.state = TaskState.FAILED
        self._publish_status('failed')

    # Nav2 通用封装
    def _send_nav_goal(self, pose, on_success, on_fail):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 server unavailable')
            on_fail()
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        future = self.nav_client.send_goal_async(goal)

        def goal_response_cb(fut):
            handle = fut.result()
            if not handle.accepted:
                self.get_logger().error('Nav goal rejected')
                on_fail()
                return
            handle.get_result_async().add_done_callback(
                lambda f: (on_success() if f.result().status == 4 else on_fail()))

        future.add_done_callback(goal_response_cb)


def main():
    rclpy.init()
    node = TaskExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出时清理子进程
        if node.grasp_process and node.grasp_process.poll() is None:
            node.grasp_process.kill()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()