from rclpy.node import Node
from std_msgs.msg import Int32

COIN_VALUES = {
    3: (2.0, "2€"),
    2: (0.2, "20c"),
    1: (0.01, "1c"),
    0: (0.0, "Unrecognized")
}

class CoinSubscriber(Node):
    def __init__(self):
        super().__init__('coin_subscriber')
        self.subscription = self.create_subscription(
            Int32,
            '/coinValuePublisher',
            self.coin_callback,
            10
        )
        self.total_amount = 0.0
        self.coin_counts = {label: 0 for _, label in COIN_VALUES.values()}
        self.ui_update_callback = None

    def coin_callback(self, msg):
        value, label = COIN_VALUES.get(msg.data, (0.0, "Unrecognized"))
        self.coin_counts[label] += 1
        if label != "Unrecognized":
            self.total_amount += value

        if self.ui_update_callback:
            self.ui_update_callback(self.total_amount, self.coin_counts)