import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import tkinter as tk
from tkinter import font
from threading import Thread

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


class CoinCounterApp:
    def __init__(self, node):
        self.node = node
        self.node.ui_update_callback = self.update_ui

        self.root = tk.Tk()
        self.root.title("Real Time Coin Acceptor")
        self.root.geometry("600x600")
        self.root.configure(bg="#f5f7fa")

        # Font control
        self.font_size = 14
        self.base_font = font.Font(family="Arial", size=self.font_size)

        # Main card (resizable)
        self.card = tk.Frame(self.root, bg="white", bd=0, relief="flat")
        self.card.pack(fill="both", expand=True, padx=20, pady=20)
        self.card.configure(highlightbackground="#ddd", highlightthickness=1)

        self.amount_label = tk.Label(
            self.card, text=f"Total Amount: €{self.node.total_amount:.2f}",
            font=("Arial", self.font_size + 4, "bold"), bg="white", fg="#333"
        )
        self.amount_label.pack(pady=(20, 15))

        # Container for centering coin counters
        self.center_frame = tk.Frame(self.card, bg="white")
        self.center_frame.pack(fill="both", expand=True)

        # Coin counters frame
        self.coins_frame = tk.Frame(self.center_frame, bg="white")
        self.coins_frame.pack(expand=True)  # <-- centers vertically

        self.coin_name_labels = {}
        self.coin_value_labels = {}
        for label in ["2€", "20c", "1c"]:
            frame = tk.Frame(self.coins_frame, bg="#f9f9f9", bd=0, relief="flat")
            frame.pack(fill="x", padx=40, pady=6)

            name_lbl = tk.Label(frame, text=label, font=self.base_font,
                                width=6, anchor="w", bg="#f9f9f9", fg="#222")
            name_lbl.pack(side="left", padx=10)
            self.coin_name_labels[label] = name_lbl

            val_lbl = tk.Label(frame, text="0", font=self.base_font,
                               bg="#f9f9f9", fg="#555")
            val_lbl.pack(side="right", padx=10)
            self.coin_value_labels[label] = val_lbl

        # Unrecognized percentage
        self.unrecognized_label = tk.Label(
            self.card, text="Unrecognized: 0.0%",
            font=("Arial", self.font_size, "italic"),
            bg="white", fg="#e67e22"
        )
        self.unrecognized_label.pack(pady=15)

        # Control buttons (reset + text size control)
        controls_frame = tk.Frame(self.card, bg="white")
        controls_frame.pack(pady=15)

        self.reset_button = tk.Button(
            controls_frame, text="Reset", command=self.reset_amount,
            font=("Arial", self.font_size, "bold"),
            bg="#e74c3c", fg="white", relief="flat", bd=0
        )
        self.reset_button.grid(row=0, column=0, padx=10, ipadx=10, ipady=5)

        self.increase_button = tk.Button(
            controls_frame, text="A+", command=self.increase_font,
            font=("Arial", self.font_size, "bold"),
            bg="#4CAF50", fg="white", relief="flat", bd=0
        )
        self.increase_button.grid(row=0, column=1, padx=10, ipadx=10, ipady=5)

        self.decrease_button = tk.Button(
            controls_frame, text="A-", command=self.decrease_font,
            font=("Arial", self.font_size, "bold"),
            bg="#3498db", fg="white", relief="flat", bd=0
        )
        self.decrease_button.grid(row=0, column=2, padx=10, ipadx=10, ipady=5)

        # Run ROS2 in a separate thread
        self.ros_thread = Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.ros_thread.start()

    def update_ui(self, amount, coin_counts):
        self.root.after(0, lambda: self.amount_label.config(text=f"Total Amount: €{amount:.2f}"))

        for label, count in coin_counts.items():
            if label in self.coin_value_labels:
                self.root.after(0, lambda l=label, c=count: self.coin_value_labels[l].config(text=str(c)))

        total_coins = sum(coin_counts.values())
        unrec = coin_counts.get("Unrecognized", 0)
        percent = (unrec / total_coins * 100) if total_coins > 0 else 0.0
        self.root.after(0, lambda: self.unrecognized_label.config(text=f"Unrecognized: {percent:.1f}%"))

    def reset_amount(self):
        self.node.total_amount = 0.0
        self.node.coin_counts = {label: 0 for _, label in COIN_VALUES.values()}
        self.update_ui(0.0, self.node.coin_counts)

    def increase_font(self):
        self.font_size += 2
        self.apply_font_size()

    def decrease_font(self):
        if self.font_size > 8:
            self.font_size -= 2
            self.apply_font_size()

    def apply_font_size(self):
        new_font = font.Font(family="Arial", size=self.font_size)
        self.base_font = new_font

        # Update all labels and buttons
        self.amount_label.config(font=("Arial", self.font_size + 4, "bold"))
        self.unrecognized_label.config(font=("Arial", self.font_size, "italic"))
        self.reset_button.config(font=("Arial", self.font_size, "bold"))
        self.increase_button.config(font=("Arial", self.font_size, "bold"))
        self.decrease_button.config(font=("Arial", self.font_size, "bold"))

        for lbl in self.coin_name_labels.values():
            lbl.config(font=new_font)
        for lbl in self.coin_value_labels.values():
            lbl.config(font=new_font)

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = CoinSubscriber()
    app = CoinCounterApp(node)
    try:
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
