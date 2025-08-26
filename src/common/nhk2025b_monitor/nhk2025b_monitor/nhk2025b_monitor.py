import rclpy
from rclpy.node import Node
from nhk2025b_msgs.msg import PcState
import psutil
import time
import socket


class nhk2025b_monitor(Node):
    def __init__(self):
        super().__init__('pc_state_publisher')
        self.publisher_ = self.create_publisher(PcState, '/pc_state', 1)
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.hostname = socket.gethostname()  # PC名を取得

        # ネットワーク速度計算用
        self.last_net = psutil.net_io_counters()
        self.last_time = time.time()

    def get_avg_temperature(self):
        temps = psutil.sensors_temperatures()
        values = []
        for entries in temps.values():
            for t in entries:
                if t.current is not None:
                    values.append(t.current)
        if values:
            return sum(values) / len(values)
        else:
            return 0.0

    def timer_callback(self):
        msg = PcState()

        msg.hostname = self.hostname

        # CPU
        msg.cpu_percent = psutil.cpu_percent(interval=None)

        # RAM
        msg.ram_percent = psutil.virtual_memory().percent

        # 温度 (全センサーの平均)
        msg.temperature = self.get_avg_temperature()

        # ネットワーク
        now = time.time()
        net = psutil.net_io_counters()
        dt = now - self.last_time
        if dt > 0:
            msg.net_up_mbps = (net.bytes_sent - self.last_net.bytes_sent) / dt / 1e6
            msg.net_down_mbps = (net.bytes_recv - self.last_net.bytes_recv) / dt / 1e6
        else:
            msg.net_up_mbps = 0.0
            msg.net_down_mbps = 0.0

        self.last_net = net
        self.last_time = now

        # Publish
        self.publisher_.publish(msg)
        self.get_logger().info(
            f"{msg.hostname} | CPU {msg.cpu_percent:.1f}% | "
            f"RAM {msg.ram_percent:.1f}% | Temp(avg) {msg.temperature:.1f}°C | "
            f"Net ↑{msg.net_up_mbps:.2f}Mbps ↓{msg.net_down_mbps:.2f}Mbps"
        )



def main(args=None):
    rclpy.init(args=args)
    node = nhk2025b_monitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
