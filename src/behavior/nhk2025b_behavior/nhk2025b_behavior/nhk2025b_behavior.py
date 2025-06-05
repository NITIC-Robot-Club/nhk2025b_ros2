import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from nhk2025b_msgs.msg import State, StateArray, RobotStatus, Command
import math
import re
import sys

def parse_labels_and_positions(md_path):
    with open(md_path, encoding="utf-8") as f:
        content = f.read()
    m = re.search(r'stateDiagram-v2([\s\S]+?)(```|$)', content)
    if not m:
        raise RuntimeError("stateDiagram-v2 not found")
    body = m.group(1)

    label_to_state = {}
    state_to_pose = {}
    state_id_map = {}
    state_counter = 0
    transitions = []

    for line in body.splitlines():
        m = re.match(r"\s*([a-zA-Z0-9_]+)\s*:\s*(.+)", line)
        if m:
            state, label = m.groups()
            if not re.match(r'\w+\(.*\)', label):
                label_to_state[label.strip()] = state
                if state not in state_id_map:
                    state_id_map[state] = state_counter
                    state_counter += 1
        m_pos = re.match(r'\s*([a-zA-Z0-9_]+)\s*:\s*set_position\(([^)]*)\)', line)
        if m_pos:
            state = m_pos.group(1)
            raw_args = m_pos.group(2).split(",")
            args = [float(a.strip()) for a in raw_args]
            state_to_pose[state] = args
        # 遷移抽出:  source --> dest : cond
        m_tr = re.match(r'\s*([a-zA-Z0-9_\[\]\*]+)\s*-->\s*([a-zA-Z0-9_\[\]\*]+)\s*(?::\s*(.+))?', line)
        if m_tr:
            src, dst, cond = m_tr.groups()
            transitions.append((src, dst, cond.strip() if cond else None))

    return label_to_state, state_to_pose, state_id_map, transitions

class BehaviorManagerNode(Node):
    def __init__(self, state_graph_path):
        super().__init__('behavior_manager')
        self.label_to_state, self.state_to_pose, self.state_id_map, self.transitions = parse_labels_and_positions(state_graph_path)
        self.id_to_label = {v: k for k, v in self.label_to_state.items()}
        self.id_to_state = {v: s for s, v in self.state_id_map.items()}

        self.state_array_pub = self.create_publisher(StateArray, '/behavior/state_array', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/behavior/goal_pose', 10)
        self.state_pub = self.create_publisher(State, '/behavior/state', 10)

        self.create_subscription(PoseStamped, '/localization/current_pose', self.current_pose_callback, 10)
        self.create_subscription(RobotStatus, '/robot_status', self.robot_status_callback, 10)
        self.create_subscription(Int32, '/behavior/set_status_num', self.set_status_num_callback, 10)
        self.create_subscription(Command, '/command', self.command_callback, 10)

        self.state_array = self.build_state_array()
        self.current_state = '[*]' # 初期状態
        self.last_result = None

        self.current_pose = None
        self.robot_status = None
        self.goal_pose = None
        self.command = Command()
        self.waiting_for_goal = False

        self.finished = False  # 完了フラグ

        self.state_array_timer = self.create_timer(2.0, self.publish_state_array)
        self.automaton_timer = self.create_timer(0.5, self.automaton_step)  # 0.5秒ごとに状態遷移

        self.get_logger().info('BehaviorManagerNode started')

    def build_state_array(self):
        msg = StateArray()
        msg.name = 'running_flow'
        for label, state in self.label_to_state.items():
            state_msg = State()
            state_msg.name = label
            state_msg.id = self.state_id_map[state]
            msg.state.append(state_msg)
        return msg

    def publish_state_array(self):
        self.state_array_pub.publish(self.state_array)
    
    def command_callback(self, msg):
        self.command = msg

    def publish_state(self, finished=False):
        # 現在の状態をpub
        if finished:
            msg = State()
            msg.name = "終了"
            msg.id = -1
            self.state_pub.publish(msg)
            return

        current_label = None
        for label, state in self.label_to_state.items():
            if state == self.current_state:
                current_label = label
                break
        msg = State()
        msg.name = current_label if current_label else self.current_state
        msg.id = self.state_id_map[self.current_state] if self.current_state in self.state_id_map else -1
        self.state_pub.publish(msg)

    def set_status_num_callback(self, msg):
        # 状態番号で強制遷移
        target_id = msg.data
        found = None
        for s in self.state_array.state:
            if s.id == target_id:
                found = s
                break
        if not found:
            self.get_logger().error(f'ID {target_id} のラベルが見つかりません')
            return
        label = found.name
        state = self.label_to_state[label]
        self.get_logger().info(f'[set_status_num] ラベル {label}({state}) へ強制遷移')
        self.current_state = state
        self.waiting_for_goal = False
        self.finished = False

    def current_pose_callback(self, msg):
        self.current_pose = msg

    def robot_status_callback(self, msg):
        self.robot_status = msg

    def automaton_step(self):
        if self.finished:
            # すでに完了していたら「終了」stateだけをpublish
            self.publish_state(finished=True)
            return

        # まず現在の状態に対応するset_positionがあれば、それを目標として出す
        advanced = False
        if self.current_state in self.state_to_pose:
            x, y, yaw = self.state_to_pose[self.current_state]
            self.set_position(x, y, yaw)
            # ゴールに近ければ「次の状態」へ進む
            if self.current_pose is not None:
                if self.is_pose_close(self.current_pose, x, y, yaw):
                    self.get_logger().info(f'ゴール({self.current_state})に到達したため次へ進みます')
                    advanced = True
        else:
            advanced = True  # set_positionがなければすぐ進む

        next_state = None
        cond_env = {
            "check_ready": self.check_ready
        }

        for src, dst, cond in self.transitions:
            if src == self.current_state:
                if cond is None or cond.lower() == 'else':
                    if advanced:
                        next_state = dst
                        break
                else:
                    try:
                        # 条件式を評価
                        if eval(cond, {}, cond_env):
                            next_state = dst
                            break
                    except Exception as e:
                        self.get_logger().warn(f"条件式eval失敗: {cond} ({e})")
                        continue

        if next_state and next_state not in ('[*]', '[*] '):
            self.get_logger().info(f'自動遷移: {self.current_state} -> {next_state}')
            self.current_state = next_state
            self.waiting_for_goal = False
            self.finished = False
        elif next_state in ('[*]', '[*] '):
            self.get_logger().info('状態機械が終了状態に到達')
            self.finished = True
            self.publish_state(finished=True)
            return

        self.publish_state()

    def set_position(self, x, y, yaw_deg):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        yaw = math.radians(yaw_deg)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.pose_pub.publish(pose)
        self.goal_pose = pose

    def is_pose_close(self, pose: PoseStamped, goal_x, goal_y, goal_yaw):
        # 平面距離
        dx = pose.pose.position.x - goal_x
        dy = pose.pose.position.y - goal_y
        dist = math.hypot(dx, dy)
        # 角度（クォータニオン→ヨー）
        q = pose.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        # 角度差
        dyaw = abs((yaw - math.radians(goal_yaw) + math.pi) % (2*math.pi) - math.pi)
        deg = math.degrees(dyaw)
        return dist < 0.1 and deg < 10.0

    # 必要に応じて実装
    def check_ready(self):
        return self.command.automate_ready

def main(args=None):
    rclpy.init(args=args)
    state_graph_path = sys.argv[1] if len(sys.argv) > 1 else 'state_graph.md'
    node = BehaviorManagerNode(state_graph_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()