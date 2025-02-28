import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PySide2.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QPushButton, QWidget
from functools import partial

from project3_interfaces.msg import TableCommand 


class TablePublisher(Node):
    def __init__(self):
        super().__init__('table_publisher')
        self.publisher = self.create_publisher(TableCommand , 'to_robot', 10)

    def publish_table_number(self, table_number):
        msg = TableCommand()
        msg.to_robot = table_number
        self.publisher.publish(msg)
        self.get_logger().info(f'Published table number: {table_number}')

class TablePublisherGUI(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setup_ui()

    def setup_ui(self):
        self.setWindowTitle("Table Control Panel")
        self.setGeometry(100, 100, 400, 400)

        central_widget = QWidget()
        layout = QVBoxLayout(central_widget)

        # Add buttons for each table
        for i in range(1, 10):
            button = QPushButton(f"Go to Table {i}")
            button.clicked.connect(partial(self.publish_table_number, i))
            layout.addWidget(button)

        # Add button for the kitchen
        #kitchen_button = QPushButton("Go to Kitchen")
        #kitchen_button.clicked.connect(partial(self.publish_table_number, "kitchen"))
        #layout.addWidget(kitchen_button)

        self.setCentralWidget(central_widget)

    def publish_table_number(self, table_number):
        self.node.publish_table_number(str(table_number))


def main():
    rclpy.init()
    node = TablePublisher()

    app = QApplication(sys.argv)
    gui = TablePublisherGUI(node)

    gui.show()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
