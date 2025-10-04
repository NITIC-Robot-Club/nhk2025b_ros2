import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, Float32
from geometry_msgs.msg import PoseStamped
from nhk2025b_msgs.msg import State, StateArray, RobotStatus, Command, EArm, BoxArm, Conveyor, PylonArm
import math
import re
import time

def parse_labels_and_positions(md_path):
    with open(md_path, encoding="utf-8") as f:
        content = f.read()
    m = re.search(r'stateDiagram-v2([\s\S]+?)(```|$)', content)
    if not m:
        raise RuntimeError("stateDiagram-v2 not found")
    body = m.group(1)

    label_to_state = {}
    state_to_pose = {}
    state_to_actions = {}
    state_id_map = {}
    state_counter = 0
    transitions = []

    for line in body.splitlines():
        if line=="":
            continue
        
        if line[0]=="#":
            continue
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
            continue
        m_action = re.match(r'\s*([a-zA-Z0-9_]+)\s*:\s*(set_\w+\(.*\))', line)
        if m_action:
            state = m_action.group(1)
            action_str = m_action.group(2)
            if state not in state_to_actions:
                state_to_actions[state] = []
            state_to_actions[state].append(action_str)
            continue
        m_tr = re.match(r'\s*([a-zA-Z0-9_\[\]\*]+)\s*-->\s*([a-zA-Z0-9_\[\]\*]+)\s*(?::\s*(.+))?', line)
        if m_tr:
            src, dst, cond = m_tr.groups()
            transitions.append((src, dst, cond.strip() if cond else None))

    return label_to_state, state_to_pose, state_to_actions, state_id_map, transitions

class nhk2025b_behavior(Node):
    def __init__(self):
        super().__init__('behavior_manager')

        # パラメータ宣言と取得
        self.declare_parameter('state_graph_path', 'state_graph.md')
        state_graph_path = self.get_parameter('state_graph_path').get_parameter_value().string_value
        self.md_name = state_graph_path.split('/')[-1]
        if not state_graph_path.endswith('.md'):
            self.get_logger().error(f'状態遷移グラフのファイル名は.mdで終わる必要があります: {state_graph_path}')
            raise RuntimeError(f'状態遷移グラフのファイル名は.mdで終わる必要があります: {state_graph_path}')
        else:
            self.get_logger().info(f'状態遷移グラフのファイル名: {self.md_name}')
            self.md_name = self.md_name.split('.')[0]  # 拡張子を除去
        self.is_red = False

        self.current_pose = None
        self.robot_status = None
        self.goal_pose = PoseStamped()
        self.e_arm_target = EArm()
        self.box_arm_target = BoxArm()
        self.conveyor_target = Conveyor()
        self.pylon_arm_target = PylonArm()
        self.command = Command()

        self.e_arm_result = EArm()
        self.box_arm_result = BoxArm()
        self.pylon_arm_result = PylonArm()

        self.pylon_arm_height_tolerance = 0.01
        self.pylon_arm_expand_tolerance = 0.1
        self.box_arm_height_tolerance = 0.01
        self.box_arm_expand_tolerance = 0.1
        self.e_arm_expand_tolerance = 0.1
        self.e_arm_get_tolerance = 0.01

        self.robot_default_width = 1.0
        self.robot_expanded_width = 1.4
        
        self.function_dict = {
            "set_e_arm": self.set_e_arm,
            "set_box_arm_height": self.set_box_arm_height,
            "set_box_arm_expand": self.set_box_arm_expand,
            "set_box_arm_strong": self.set_box_arm_strong,
            "set_box_arm_weak": self.set_box_arm_weak,
            "set_conveyor_rpm": self.set_conveyor_rpm,
            "set_pylon_arm_height": self.set_pylon_arm_height,
            "set_pylon_arm_expand": self.set_pylon_arm_expand,
            "set_pylon_arm_rpm": self.set_pylon_arm_rpm,
        }
        
        self.cond_env = {
            "check_allow_automate": self.check_allow_automate,
            "check_pylon_arm": self.check_pylon_arm,
            "check_box_arm": self.check_box_arm,
            "check_e_arm": self.check_e_arm,
        }

        self.label_to_state, self.state_to_pose, self.state_to_actions, self.state_id_map, self.transitions = parse_labels_and_positions(state_graph_path)
        self.id_to_label = {v: k for k, v in self.label_to_state.items()}
        self.id_to_state = {v: s for s, v in self.state_id_map.items()}
        self.label_snapshots = self.build_state_snapshots()

        self.state_array_pub = self.create_publisher(StateArray, '/behavior/avaiable_state_array', 1)
        self.pose_pub = self.create_publisher(PoseStamped, '/behavior/goal_pose', 1)
        self.state_pub = self.create_publisher(State, '/behavior/state_now', 1)
        self.e_arm_pub = self.create_publisher(EArm, '/e_arm/cmd', 1)
        self.box_arm_pub = self.create_publisher(BoxArm, '/box_arm/cmd', 1)
        self.conveyor_pub = self.create_publisher(Conveyor, '/conveyor/cmd', 1)
        self.pylon_arm_pub = self.create_publisher(PylonArm, '/pylon_arm/cmd', 1)
        self.robot_width_pub = self.create_publisher(Float32, '/robot_width', 1)

        self.create_subscription(PoseStamped, '/localization/current_pose', self.current_pose_callback, 1)
        self.create_subscription(RobotStatus, '/robot_status', self.robot_status_callback, 1)
        self.create_subscription(Int32, '/behavior/set_status_num', self.set_status_num_callback, 1)
        self.create_subscription(Command, '/command', self.command_callback, 1)
        self.create_subscription(Bool, '/is_red', self.is_red_callback, 1)

        self.create_subscription(EArm, '/e_arm/result', self.e_arm_callback , 1)
        self.create_subscription(BoxArm, '/box_arm/result', self.box_arm_callback , 1)
        self.create_subscription(PylonArm, '/pylon_arm/result', self.pylon_arm_callback, 1)

        self.state_array = self.build_state_array()
        self.current_state = '[*]'  # 初期状態
        self.last_result = None


        self.waiting_for_goal = False

        self.finished = False  # 完了フラグ

        # sleep 1 second
        time.sleep(1)

        self.state_array_timer = self.create_timer(0.5, self.publish_state_array)
        self.automaton_timer = self.create_timer(0.05, self.automaton_step)
        self.publish_target_timer = self.create_timer(0.1, self.publish_target) 

        self.last_state_name = None  # 直前のstate name
        self.last_state_id = None    # 直前のstate id

        self.get_logger().info(f'nhk2025b_behavior started with state_graph_path: {state_graph_path}')

    def build_state_array(self):
        msg = StateArray()
        msg.name = self.md_name
        for label, state in self.label_to_state.items():
            state_msg = State()
            state_msg.name = label
            state_msg.id = self.state_id_map[state]
            msg.state.append(state_msg)
        return msg

    def build_state_snapshots(self):
        snapshots = {}
        for label, state in self.label_to_state.items():
            actions = []
            visited = set()
            def dfs(s):
                if s in visited:
                    return
                visited.add(s)
                if s in self.state_to_actions:
                    for act in self.state_to_actions[s]:
                        actions.append(act)
                for src, dst, cond in self.transitions:
                    if dst == s:  # 逆向きに辿る
                        dfs(src)
            dfs(state)
            snapshots[label] = actions
        return snapshots


    def publish_state_array(self):
        self.state_array_pub.publish(self.state_array)
    
    def command_callback(self, msg):
        self.command = msg

    def is_red_callback(self, msg):
        self.is_red = msg.data
      
    def pylon_arm_callback(self, msg):
        self.pylon_arm_result = msg

    def e_arm_callback(self, msg):
        self.e_arm_result = msg

    def box_arm_callback(self, msg):
        self.box_arm_result = msg
        expanded = False
        for i in range(2):
            if msg.expand[i] > 0.1:
                expanded = True
        if expanded:
            self.robot_width_pub.publish(Float32(data=self.robot_expanded_width))
        else:
            self.robot_width_pub.publish(Float32(data=self.robot_default_width))


    def check_allow_automate(self):
        return self.command.allow_automate 
    
    def check_pylon_arm(self):
        for i in range(2):
            if abs(self.pylon_arm_target.height[i] - self.pylon_arm_result.height[i]) > self.pylon_arm_height_tolerance:
                return False
            if abs(self.pylon_arm_target.expand[i] - self.pylon_arm_result.expand[i]) > self.pylon_arm_expand_tolerance:
                return False
        return True
    
    def check_box_arm(self):
        for i in range(2):
            if abs(self.box_arm_target.height[i] - self.box_arm_result.height[i]) > self.box_arm_height_tolerance:
                return False
            if abs(self.box_arm_target.expand[i] - self.box_arm_result.expand[i]) > self.box_arm_expand_tolerance:
                return False
        return True

    def check_e_arm(self):
        if abs(self.e_arm_target.expand - self.e_arm_result.expand) > self.e_arm_expand_tolerance:
            return False
        if abs(self.e_arm_target.get - self.e_arm_result.get) > self.e_arm_get_tolerance:
            return False
        return True

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

        # ---- 状態復元 ----
        if label in self.label_snapshots:
            for act in self.label_snapshots[label]:
                try:
                    eval(act, {}, self.function_dict)
                except Exception as e:
                    self.get_logger().warn(f"スナップショット復元失敗: {act} ({e})")

        self.goal_pose.pose.position.x = -1.0
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
                self.get_logger().info(f'ゴール({self.current_state})に到達したため次へ進みます')
                advanced = True
        else:
            advanced = True

        if self.current_state in getattr(self, 'state_to_actions', {}):
            for act in self.state_to_actions[self.current_state]:
                try:
                    eval(act, {}, self.function_dict)
                except Exception as e:
                    self.get_logger().warn(f"アクション eval 失敗: {act} ({e})")

        next_state = None

        if advanced:
            for src, dst, cond in self.transitions:
                if src == self.current_state:
                    if cond is None or cond.lower() == 'else':
                        next_state = dst
                        break
                    else:
                        try:
                            if eval(cond, {}, self.cond_env):
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

    def is_pose_close(self, pose: PoseStamped, goal_x, goal_y, goal_yaw):
        if self.is_red:
            goal_y = -goal_y
        dx = pose.pose.position.x - goal_x
        dy = pose.pose.position.y - goal_y
        dist = math.hypot(dx, dy)
        q = pose.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        dyaw = abs((yaw - math.radians(goal_yaw) + math.pi) % (2*math.pi) - math.pi)
        deg = math.degrees(dyaw)
        return dist < 0.10 and deg < 10.0

    def set_position(self, x, y, yaw_deg):
        if self.is_red:
            y = -y
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
        
        def is_same_pose(p1: PoseStamped, p2: PoseStamped):
            if p1 is None or p2 is None:
                return False
            pos_eps = 1e-4
            yaw_eps = 1e-3
            dx = abs(p1.pose.position.x - p2.pose.position.x)
            dy = abs(p1.pose.position.y - p2.pose.position.y)
            dz = abs(p1.pose.position.z - p2.pose.position.z)
            # クォータニオンからyawを計算
            def get_yaw(q):
                return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
            yaw1 = get_yaw(p1.pose.orientation)
            yaw2 = get_yaw(p2.pose.orientation)
            dyaw = abs((yaw1 - yaw2 + math.pi) % (2*math.pi) - math.pi)
            return dx < pos_eps and dy < pos_eps and dz < pos_eps and dyaw < yaw_eps

        if not is_same_pose(self.goal_pose, pose):
            self.pose_pub.publish(pose)
            self.goal_pose = pose

    def publish_target(self):
        self.e_arm_pub.publish(self.e_arm_target)
        self.box_arm_pub.publish(self.box_arm_target)
        self.conveyor_pub.publish(self.conveyor_target)
        self.pylon_arm_pub.publish(self.pylon_arm_target)
    
    def set_e_arm(self, expand = None, get = None):
        if expand is not None:
            self.e_arm_target.expand = expand / 180 * math.pi
        if get is not None:
            self.e_arm_target.get = get / 180 * math.pi

    def set_box_arm_expand(self, left=None, right=None):
        if left is not None:
            self.box_arm_target.expand[0] = left / 180 * math.pi
        if right is not None:
            self.box_arm_target.expand[1] = right / 180 * math.pi
    
    def set_box_arm_height(self, left=None, right=None):
        if left is not None:
            self.box_arm_target.height[0] = left
        if right is not None:
            self.box_arm_target.height[1] = right
        
    def set_box_arm_strong(self, left=None, right=None):
        if left is not None:
            self.box_arm_target.strong[0] = left
        if right is not None:
            self.box_arm_target.strong[1] = right
    
    def set_box_arm_weak(self, left=None, right=None):
        if left is not None:
            self.box_arm_target.weak[0] = left
        if right is not None:
            self.box_arm_target.weak[1] = right

    def set_conveyor_rpm(self, left=None, right=None):
        if left is not None:
            self.conveyor_target.conveyor_rpm[0] = left
        if right is not None:
            self.conveyor_target.conveyor_rpm[1] = right

    def set_pylon_arm_height(self, left=None, right=None):
        if left is not None:
            self.pylon_arm_target.height[0] = left
        if right is not None:
            self.pylon_arm_target.height[1] = right

    def set_pylon_arm_expand(self, left=None, right=None):
        if left is not None:
            self.pylon_arm_target.expand[0] = left / 180 * math.pi
        if right is not None:
            self.pylon_arm_target.expand[1] = right / 180 * math.pi

    def set_pylon_arm_rpm(self, left=None, right=None):
        if left is not None:
            self.pylon_arm_target.collect_rpm[0] = left
        if right is not None:
            self.pylon_arm_target.collect_rpm[1] = right


def main(args=None):
    rclpy.init(args=args)
    node = nhk2025b_behavior()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass

if __name__ == '__main__':
    main()
