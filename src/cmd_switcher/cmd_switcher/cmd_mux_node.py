import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.time import Time

class CmdMuxNode(Node):
    def __init__(self):
        super().__init__('cmd_mux_node')

        # --- НАСТРОЙКИ ПРИОРИТЕТОВ (СЕКУНДЫ) ---
        self.timeout_manual = 0.2       # Пульт (физический RC) — самый важный, реагирует мгновенно
        self.timeout_app_manual = 0.3   # ДОБАВЛЕНО: ручное управление из desktop-приложения (по Wi-Fi/VPN,
                                        # даём чуть больше времени на задержки сети, чем у RC)
        self.timeout_home = 0.5         # Режим "Домой"/уклонение важнее автопилота
        self.timeout_auto = 2.0         # Автопилот может молчать дольше
        # ---------------------------------------

        # Хранилище последних сообщений: (msg, timestamp)
        self.last_manual = None
        self.last_app_manual = None
        self.last_home = None
        self.last_auto = None

        # Подписчики на разные источники
        self.sub_manual = self.create_subscription(Twist, '/cmd_vel/manual', self.cb_manual, 10)
        # ДОБАВЛЕНО: канал ручного управления из desktop-приложения (клавиатура/джойстик в UI),
        # публикуется через rosbridge. Приоритет НИЖЕ физического пульта — если оператор
        # одновременно держит в руках RC-пульт, он всегда может перехватить управление.
        self.sub_app_manual = self.create_subscription(Twist, '/cmd_vel/app_manual', self.cb_app_manual, 10)
        self.sub_home = self.create_subscription(Twist, '/cmd_vel/home', self.cb_home, 10)
        self.sub_auto = self.create_subscription(Twist, '/cmd_vel/auto', self.cb_auto, 10)

        # Паблишер в драйвер робота
        self.pub_final = self.create_publisher(Twist, '/cmd_vel', 10)

        # Таймер проверки (50 Гц)
        self.timer = self.create_timer(0.02, self.publish_logic)

        self.get_logger().info("Cmd Mux Node started. Listening for manual, app_manual, home, auto...")

    def cb_manual(self, msg):
        self.last_manual = (msg, self.get_clock().now())

    def cb_app_manual(self, msg):
        self.last_app_manual = (msg, self.get_clock().now())

    def cb_home(self, msg):
        self.last_home = (msg, self.get_clock().now())

    def cb_auto(self, msg):
        self.last_auto = (msg, self.get_clock().now())

    def publish_logic(self):
        now = self.get_clock().now()
        final_cmd = Twist() # По умолчанию стоп (все нули)

        # 1. ПРОВЕРКА ФИЗИЧЕСКОГО ПУЛЬТА (Высший приоритет)
        if self.last_manual:
            msg, time_received = self.last_manual
            age_sec = (now - time_received).nanoseconds / 1e9
            if age_sec < self.timeout_manual:
                self.pub_final.publish(msg)
                return

        # 2. ПРОВЕРКА РУЧНОГО УПРАВЛЕНИЯ ИЗ ПРИЛОЖЕНИЯ (ДОБАВЛЕНО)
        if self.last_app_manual:
            msg, time_received = self.last_app_manual
            age_sec = (now - time_received).nanoseconds / 1e9
            if age_sec < self.timeout_app_manual:
                self.pub_final.publish(msg)
                return

        # 3. ПРОВЕРКА РЕЖИМА "ДОМОЙ" (Средний приоритет)
        if self.last_home:
            msg, time_received = self.last_home
            age_sec = (now - time_received).nanoseconds / 1e9
            if age_sec < self.timeout_home:
                self.pub_final.publish(msg)
                return

        # 4. ПРОВЕРКА АВТОПИЛОТА (Низший приоритет)
        if self.last_auto:
            msg, time_received = self.last_auto
            age_sec = (now - time_received).nanoseconds / 1e9
            if age_sec < self.timeout_auto:
                self.pub_final.publish(msg)
                return

        # Если никто не прислал свежих данных -> Стоп
        self.pub_final.publish(final_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = CmdMuxNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
