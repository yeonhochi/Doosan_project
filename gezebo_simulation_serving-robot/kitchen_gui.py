from PySide2.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QLabel, QPushButton, QWidget, QTextBrowser, QMessageBox
from PySide2.QtCore import QRect, Signal,Slot
from datetime import datetime
from db_handler import DBHandler  # DBHandler 가져오기
from functools import partial
import sys
from ros_kitchen_nodes import KitchenClient
from functools import partial

class KitchenGUI(QMainWindow):
    new_order_signal = Signal(int, dict)  # 신호 정의: 테이블 ID와 주문 정보

    def __init__(self, kitchen_client,db_handler):
        super().__init__()
        self.setWindowTitle("Kitchen Order GUI")
        self.setGeometry(100, 100, 800, 800)
        
        # KitchenClient 객체 생성
        self.kitchen_client = kitchen_client
        self.kitchen_client.set_order_callback(self.handle_new_order)  # 새로운 주문 콜백 설정

        # DBHandler 인스턴스 생성
        self.db_handler = db_handler
        
        self.order_list = []
        self.sold_out_items = set()
        self.price_list = {
            '족발 중': 36000,
            '족발 대': 42000,
            '진로': 5000,
            '참이슬': 5000
        }

        self.setupUi()
        self.new_order_signal.connect(self.update_table_order)  # 신호 연결

    def setupUi(self):
        self.centralwidget = QWidget(self)
        self.setCentralWidget(self.centralwidget)

        self.textBrowsers = []
        self.resetButtons = []
        self.cookingCompleteButtons = []

        for i in range(1, 10):
            y_offset = 20 + ((i - 1) // 3) * 250  # 간격 증가

            browser = QTextBrowser(self.centralwidget)
            browser.setGeometry(QRect(20 + ((i - 1) % 3) * 260, y_offset, 240, 160))
            browser.setPlainText(f"Table {i}")
            self.textBrowsers.append(browser)

            reset_button = QPushButton(self.centralwidget)
            reset_button.setGeometry(QRect(20 + ((i - 1) % 3) * 260, y_offset + 170, 240, 30))
            reset_button.setText(f"Reset Table {i}")
            reset_button.clicked.connect(partial(self.reset_table, i))
            self.resetButtons.append(reset_button)

            cooking_complete_button = QPushButton(self.centralwidget)
            cooking_complete_button.setGeometry(QRect(20 + ((i - 1) % 3) * 260, y_offset + 210, 240, 30))
            cooking_complete_button.setText(f"Cooking Complete Table {i}")
            cooking_complete_button.clicked.connect(partial(self.on_cooking_complete, i))
            self.cookingCompleteButtons.append(cooking_complete_button)

        # 하단 버튼들 위치 조정 (기존보다 아래로 이동)
        button_y_offset = 770  # 버튼들을 더 아래로 배치

        self.soldOutButton = QPushButton(self.centralwidget)
        self.soldOutButton.setGeometry(QRect(20, button_y_offset, 240, 50))
        self.soldOutButton.setText("Notify Sold-Out")
        self.soldOutButton.clicked.connect(self.kitchen_client.notify_sold_out_items)  # ✅ KitchenClient와 연결

        self.resetStockButton = QPushButton(self.centralwidget)
        self.resetStockButton.setGeometry(QRect(280, button_y_offset, 240, 50))
        self.resetStockButton.setText("Reset Stock")
        self.resetStockButton.clicked.connect(self.reset_stock)

        self.revenueButton = QPushButton(self.centralwidget)
        self.revenueButton.setGeometry(QRect(540, button_y_offset, 240, 50))
        self.revenueButton.setText("Show Revenue (By Table)")
        self.revenueButton.clicked.connect(self.show_revenue_by_table)

        self.productRevenueButton = QPushButton(self.centralwidget)
        self.productRevenueButton.setGeometry(QRect(800, button_y_offset, 240, 50))
        self.productRevenueButton.setText("Show Revenue (By Product)")
        self.productRevenueButton.clicked.connect(self.show_revenue_by_product)

        self.allResetButton = QPushButton(self.centralwidget)
        self.allResetButton.setGeometry(QRect(1060, button_y_offset, 240, 50))
        self.allResetButton.setText("All Reset")
        self.allResetButton.clicked.connect(self.reset_all_tables)

    def reset_stock(self):
        """Reset stock levels for all items."""
        today = datetime.today().strftime('%Y-%m-%d')
        try:
            self.db_handler.reset_stock(today)  # DBHandler에서 재고 초기화
            QMessageBox.information(self, "Stock Reset", "Stock levels have been reset successfully.")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to reset stock: {e}")

    def on_cooking_complete(self, table_id=None):  # table이 아닌 table_id로 정의
        """
        Cooking Complete 버튼 클릭 시 호출되는 함수.
        - Qt 메시지 박스를 띄우고, ROS2를 사용하여 로봇에게 이동 명령을 보냄.
        """
        if table_id is not None:
            # Qt 메시지 박스 알림
            QMessageBox.information(self, "Cooking Complete", f"Cooking completed for Table {table_id}")

            # ROS2 Publish로 로봇 이동 명령 발행
            command = "go_to_table"
            self.kitchen_client.publish_to_robot(table_id, command)

            print(f"[ROS2] Published command '{command}' for Table {table_id}")

        else:
            QMessageBox.warning(self, "Error", "No table ID provided for Cooking Completion!")
            print("[ERROR] No table ID provided for Cooking Completion.")

    def reset_table(self,table_id):
        "특정 테이블의 주문 정보와 GUI 초기화"
        #주문 정보 초기화
        self.order_list = [order for order in self.order_list if order['table_id'] !=table_id]
        #GUI 초기화
        browser = self.textBrowsers[table_id -1]
        browser.clear()
        browser.setPlainText(f"Table {table_id} Reset")
        
    def reset_all_tables(self):
        """모든 테이블의 주문 정보와 gui 초기화"""
        self.order_list.clear()
        for i, browser in enumerate(self.textBrowsers):
            browser.clear()
            browser.setPlainText(f"Table {i+1} Reset")

    
    def show_revenue_by_table(self):
        """오늘의 테이블별 매출 정보를 팝업창에 표시."""
        today = datetime.today().strftime('%Y-%m-%d')  # 오늘 날짜
        try:
            result = self.db_handler.get_revenue_by_table(today)  # DBHandler에서 테이블별 매출 가져오기
            if not result:
                QMessageBox.information(self, "Revenue by Table", "No revenue data found for today.")
                return
            
            # 매출 데이터를 포맷팅
            revenue_message = "\n".join([f"Table {table_id}: {revenue:,}원" for table_id, revenue in result.items()])
            QMessageBox.information(self, "Revenue by Table", revenue_message)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to retrieve revenue by table: {e}")

    def show_revenue_by_product(self):
        """오늘의 제품별 매출 정보를 팝업창에 표시."""
        today = datetime.today().strftime('%Y-%m-%d')  # 오늘 날짜
        try:
            result = self.db_handler.get_revenue_by_product(today)  # DBHandler에서 제품별 매출 가져오기
            if not result:
                QMessageBox.information(self, "Revenue by Product", "No revenue data found for today.")
                return

            # 매출 데이터를 포맷팅
            revenue_message = "\n".join([f"{product}: {revenue:,}원" for product, revenue in result.items()])
            QMessageBox.information(self, "Revenue by Product", revenue_message)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to retrieve revenue by product: {e}")


    def notify_sold_out_items(self):
        """수동으로 품절 상태를 확인하고 알림 전송."""
        try:
            self.kitchen_client.process_manual_soldout()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to notify sold-out items: {e}")


    @Slot(int, dict)
    def update_table_order(self, table_id, items):
        """테이블의 주문 정보를 UI에 업데이트"""
        browser = self.textBrowsers[table_id - 1]
        order_details = "\n".join([f"{item}: {quantity}" for item, quantity in items.items()])
        browser.setPlainText(f"Table {table_id} Order:\n{order_details}")

    def handle_new_order(self, table_id, items):
        """KitchenClient에서 전달된 주문 정보를 처리"""
        print(f"📥 [NEW ORDER] Table {table_id}: {items}")  # 디버깅용 출력
        self.new_order_signal.emit(table_id, items)  # ✅ UI 업데이트 신호 전송


    @Slot(list)
    def update_soldout_items(self, items):
        """품절 항목을 UI에서 비활성화"""
        QMessageBox.information(self, "Sold-Out Notification", f"현재 품절된 품목:\n{', '.join(items)}")
