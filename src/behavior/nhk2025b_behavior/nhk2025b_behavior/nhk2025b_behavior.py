import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from nhk2025b_msgs.msg import State, StateArray, RobotStatus, Command
import math
import re

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
        m_tr = re.match(r'\s*([a-zA-Z0-9_\[\]\*]+)\s*-->\s*([a-zA-Z0-9_\[\]\*]+)\s*(?::\s*(.+))?', line)
        if m_tr:
            src, dst, cond = m_tr.groups()
            transitions.append((src, dst, cond.strip() if cond else None))

    return label_to_state, state_to_pose, state_id_map, transitions

class nhk2025b_behavior(Node):
    def __init__(self):
        super().__init__('behavior_manager')

        # パラメータ宣言と取得
        self.declare_parameter('state_graph_path', 'state_graph.md')
        state_graph_path = self.get_parameter('state_graph_path').get_parameter_value().string_value

        self.label_to_state, self.state_to_pose, self.state_id_map, self.transitions = parse_labels_and_positions(state_graph_path)
        self.id_to_label = {v: k for k, v in self.label_to_state.items()}
        self.id_to_state = {v: s for s, v in self.state_id_map.items()}

        self.state_array_pub = self.create_publisher(StateArray, '/behavior/avaiable_state_array', 1)
        self.pose_pub = self.create_publisher(PoseStamped, '/behavior/goal_pose', 1)
        self.state_pub = self.create_publisher(State, '/behavior/state_now', 1)

        self.create_subscription(PoseStamped, '/localization/current_pose', self.current_pose_callback, 1)
        self.create_subscription(RobotStatus, '/robot_status', self.robot_status_callback, 1)
        self.create_subscription(Int32, '/behavior/set_status_num', self.set_status_num_callback, 1)
        self.create_subscription(Command, '/command', self.command_callback, 1)

        self.state_array = self.build_state_array()
        self.current_state = '[*]'  # 初期状態
        self.last_result = None

        self.current_pose = None
        self.robot_status = None
        self.goal_pose = None
        self.command = Command()
        self.waiting_for_goal = False

        self.finished = False  # 完了フラグ

        self.state_array_timer = self.create_timer(0.5, self.publish_state_array)
        self.automaton_timer = self.create_timer(0.05, self.automaton_step)

        self.last_state_name = None  # 直前のstate name
        self.last_state_id = None    # 直前のstate id

        self.get_logger().info(f'nhk2025b_behavior started with state_graph_path: {state_graph_path}')

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
        if current_label:
            msg = State()
            msg.name = current_label
            msg.id = self.state_id_map[self.current_state] if self.current_state in self.state_id_map else -1
            self.last_state_name = msg.name
            self.last_state_id = msg.id
        else:
            msg = State()
            msg.name = self.last_state_name if self.last_state_name is not None else self.current_state
            msg.id = self.last_state_id if self.last_state_id is not None else -1
        self.state_pub.publish(msg)

    def set_status_num_callback(self, msg):
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
            self.publish_state(finished=True)
            return

        advanced = False
        if self.current_state in self.state_to_pose:
            x, y, yaw = self.state_to_pose[self.current_state]
            self.set_position(x, y, yaw)
            if self.current_pose is not None and self.is_pose_close(self.current_pose, x, y, yaw):
                # self.get_logger().info(f'ゴール({self.current_state})に到達したため次へ進みます')
                advanced = True
        else:
            advanced = True

        next_state = None
        cond_env = {
            "check_ready": self.check_ready
        }

        if advanced:
            for src, dst, cond in self.transitions:
                if src == self.current_state:
                    if cond is None or cond.lower() == 'else':
                        next_state = dst
                        break
                    else:
                        try:
                            if eval(cond, {}, cond_env):
                                next_state = dst
                                break
                        except Exception as e:
                            self.get_logger().warn(f"条件式eval失敗: {cond} ({e})")
                            continue

        if next_state and next_state not in ('[*]', '[*] '):
            # self.get_logger().info(f'自動遷移: {self.current_state} -> {next_state}')
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
        dx = pose.pose.position.x - goal_x
        dy = pose.pose.position.y - goal_y
        dist = math.hypot(dx, dy)
        q = pose.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        dyaw = abs((yaw - math.radians(goal_yaw) + math.pi) % (2*math.pi) - math.pi)
        deg = math.degrees(dyaw)
        return dist < 0.10 and deg < 7.0

    def check_ready(self):
        return self.command.automate_ready

def main(args=None):
    rclpy.init(args=args)
    node = nhk2025b_behavior()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
