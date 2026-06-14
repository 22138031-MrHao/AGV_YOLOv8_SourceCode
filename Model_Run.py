import sys
import cv2
import os
import torch
import unicodedata
import serial
import time
from datetime import datetime
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QListWidget, QListWidgetItem, QTextEdit, QPushButton)
from PyQt5.QtGui import QImage, QPixmap, QFont
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from ultralytics import YOLO

# ==========================================
# CẤU HÌNH HỆ THỐNG
# ==========================================
MODEL_PATH = r'E:\Model\runs\detect\model_chuan_xe_tu_hanh\weights\best.pt'
WATCHDOG_DIR = r'E:\Model\test_images' 
SAVED_IMAGES_DIR = r'E:\Model\Luu_Tru_Anh' 
USB_PORT = 'COM10' 
USB_BAUD = 2000000
LOGO_PATH = r'E:\Model\logo_nlu.png' 
# ==========================================

def remove_accents(input_str):
    nfkd_form = unicodedata.normalize('NFKD', input_str)
    return u"".join([c for c in nfkd_form if not unicodedata.combining(c)])

# =====================================================================
# LUỒNG CHẠY NGẦM: TỰ ĐỘNG BÁM CÁP USB (AUTO-RECONNECT)
# =====================================================================
class CameraUSBWorker(QThread):
    image_received = pyqtSignal(str, float) 
    log_message = pyqtSignal(str)

    def __init__(self, port, baudrate, save_dir):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.save_dir = save_dir
        self.running = True
        self.ser = None

    def run(self):
        while self.running:
            try:
                # 1. Kiểm tra và kết nối lại nếu chưa có kết nối
                if self.ser is None or not self.ser.is_open:
                    self.log_message.emit(f"🔄 Đang tìm và mở cổng USB {self.port}...")
                    self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
                    time.sleep(2) # Chờ ESP khởi động
                    self.log_message.emit(f"✅ CÁP USB ĐÃ KẾT NỐI TẠI {self.port}! Đang chờ ảnh...")

                # 2. Lắng nghe dữ liệu
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith("START:"):
                    img_len = int(line.split(":")[1])
                    
                    start_time = time.time()
                    img_bytes = self.ser.read(img_len)
                    self.ser.readline() # Đọc bỏ chữ END
                    
                    transfer_time = (time.time() - start_time) * 1000
                    
                    if len(img_bytes) == img_len:
                        save_path = os.path.join(self.save_dir, "temp_usb_realtime.jpg")
                        with open(save_path, "wb") as f:
                            f.write(img_bytes)
                            
                        self.log_message.emit(f"✅ Nhận ảnh ({transfer_time:.1f} ms)! AI đang phân tích...")
                        self.image_received.emit(save_path, transfer_time)
                    else:
                        self.log_message.emit("❌ Lỗi: Gói tin ảnh qua USB bị đứt đoạn!")

            except serial.SerialException:
                # Bắt lỗi khi rút cáp USB đột ngột
                self.log_message.emit(f"❌ Cáp USB bị rút hoặc lỗi! Tự động thử lại sau 3s...")
                if self.ser and self.ser.is_open:
                    self.ser.close()
                self.ser = None
                time.sleep(3) # Đợi 3 giây rồi vòng lại thử kết nối
            except Exception as e:
                time.sleep(1)

    def send_command(self, cmd):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(cmd.encode('utf-8'))
            except Exception:
                pass

    def force_reconnect(self):
        # Chủ động ngắt kết nối để luồng loop tự động thiết lập lại
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None

    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()

# =====================================================================
# GIAO DIỆN CHÍNH
# =====================================================================
class TomatoExpertGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("HỆ THỐNG GIÁM SÁT NÔNG NGHIỆP TỰ HÀNH")
        self.setGeometry(50, 50, 1600, 950)
        self.setStyleSheet("background-color: #1a1b26; color: #a9b1d6;")

        os.makedirs(WATCHDOG_DIR, exist_ok=True)
        os.makedirs(SAVED_IMAGES_DIR, exist_ok=True)

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print(f"Đang tải AI bằng {self.device}...")
        self.model = YOLO(MODEL_PATH)
        
        self.expert_db = {
            'bệnh cháy lá': {'name': 'Bệnh Cháy Lá', 'remedy': '- Cắt tỉa lá bệnh ngay lập tức.\n- Phun thuốc diệt nấm (Mancozeb).'},
            'lá khỏe mạnh': {'name': 'Lá Khỏe Mạnh', 'remedy': '- Cây phát triển bình thường.\n- Tiếp tục duy trì chế độ chăm sóc hiện tại.'},
            'bệnh mốc sương': {'name': 'Bệnh Mốc Sương', 'remedy': '- NGUY HIỂM: Phun thuốc Ridomil Gold ngay.'},
            'sâu đục lá': {'name': 'Sâu Đục Lá', 'remedy': '- Dùng bẫy dính vàng.\n- Phun thuốc nội hấp Abamectin.'},
            'bệnh nấm mốc lá': {'name': 'Bệnh Nấm Mốc Lá', 'remedy': '- Tăng thông thoáng, giảm độ ẩm nhà màng.'},
            'virus khảm lá': {'name': 'Virus Khảm Lá', 'remedy': '- Nhổ bỏ cây bệnh, tiêu diệt côn trùng chích hút.'},
            'bệnh đốm lá septoria': {'name': 'Bệnh Đốm Lá Septoria', 'remedy': '- Tránh tưới nước lên lá, dùng thuốc gốc đồng.'},
            'nhện đỏ': {'name': 'Nhện Đỏ', 'remedy': '- Phun nước áp lực mạnh, dùng thuốc đặc trị nhện.'},
            'virus vàng xoăn lá': {'name': 'Virus Vàng Xoăn Lá', 'remedy': '- Nhổ bỏ cây, diệt bọ phấn trắng truyền bệnh.'}
        }

        self.init_ui()
        
        self.image_counter = 1
        self.refresh_image_counter()
        
        self.last_frame_time = time.time()

        self.usb_thread = CameraUSBWorker(port=USB_PORT, baudrate=USB_BAUD, save_dir=WATCHDOG_DIR)
        self.usb_thread.image_received.connect(self.process_image)
        self.usb_thread.log_message.connect(self.update_status_bar)
        self.usb_thread.start() 

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        super_layout = QVBoxLayout(central_widget)
        super_layout.setContentsMargins(15, 15, 15, 15)
        super_layout.setSpacing(15)

        # =====================================================================
        # KHU VỰC 1: BẢNG HIỆU (HEADER)
        # =====================================================================
        header_widget = QWidget()
        header_widget.setStyleSheet("""
            QWidget { background-color: #24283b; border-radius: 12px; border: 2px solid #414868; }
        """)
        header_layout = QHBoxLayout(header_widget)
        header_layout.setContentsMargins(20, 15, 20, 15)
        header_layout.addStretch()

        center_group = QHBoxLayout()
        center_group.setSpacing(25) 

        self.logo_label = QLabel()
        if os.path.exists(LOGO_PATH):
            pixmap = QPixmap(LOGO_PATH).scaled(120, 120, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.logo_label.setPixmap(pixmap)
            self.logo_label.setStyleSheet("border: none; background: transparent;")
        else:
            self.logo_label.setText("LOGO NLU\n(Thiếu file)")
            self.logo_label.setAlignment(Qt.AlignCenter)
            self.logo_label.setStyleSheet("background-color: #1f2335; color: #f7768e; border: 2px dashed #f7768e; border-radius: 10px; font-weight: bold;")
            self.logo_label.setFixedSize(120, 120)
        
        center_group.addWidget(self.logo_label)

        title_layout = QVBoxLayout()
        title_layout.setSpacing(5)
        title_layout.setAlignment(Qt.AlignCenter)
        
        line1 = QLabel("TRƯỜNG ĐẠI HỌC NÔNG LÂM TP.HCM")
        line1.setStyleSheet("font-size: 42px; font-weight: 900; color: #4da6ff; border: none; background: transparent;")
        line1.setAlignment(Qt.AlignCenter)
        
        line2 = QLabel("MODEL XỬ LÝ ẢNH GIÁM SÁT BỆNH HẠI CÂY CÀ CHUA")
        line2.setStyleSheet("font-size: 24px; font-weight: bold; color: #ff9e64; border: none; background: transparent;")
        line2.setAlignment(Qt.AlignCenter)
        
        line3 = QLabel("Khoa Cơ Khí - Công Nghệ")
        line3.setStyleSheet("font-size: 20px; font-weight: bold; color: #a9b1d6; border: none; background: transparent;")
        line3.setAlignment(Qt.AlignCenter)

        title_layout.addWidget(line1)
        title_layout.addWidget(line2)
        title_layout.addWidget(line3)
        
        center_group.addLayout(title_layout)
        header_layout.addLayout(center_group)
        header_layout.addStretch()

        dummy_label = QLabel()
        dummy_label.setFixedSize(120, 120)
        dummy_label.setStyleSheet("border: none; background: transparent;")
        header_layout.addWidget(dummy_label)

        super_layout.addWidget(header_widget, stretch=1)

        # =====================================================================
        # KHU VỰC 2: MAIN PANEL (CAMERA + BẢNG ĐIỀU KHIỂN)
        # =====================================================================
        main_layout = QHBoxLayout()
        main_layout.setSpacing(15)

        left_panel = QVBoxLayout()
        self.img_screen = QLabel("ĐANG CHỜ ẢNH CAMERA TỪ XE TỰ HÀNH...")
        self.img_screen.setStyleSheet("background-color: #24283b; border: 2px solid #414868; border-radius: 12px; font-size: 20px;")
        self.img_screen.setAlignment(Qt.AlignCenter)
        left_panel.addWidget(self.img_screen, stretch=9)

        self.status_bar = QLabel("Trạng thái: Đang khởi động hệ thống...")
        self.status_bar.setStyleSheet("color: #bb9af7; font-size: 15px; font-weight: bold; padding-top: 5px;")
        left_panel.addWidget(self.status_bar, stretch=1)
        
        main_layout.addLayout(left_panel, stretch=7)

        right_panel = QVBoxLayout()
        
        header_kl = QLabel("KẾT LUẬN TỔNG QUAN")
        header_kl.setStyleSheet("font-size: 20px; font-weight: bold; color: #f7768e; margin-bottom: 5px;")
        right_panel.addWidget(header_kl)

        self.detail_view = QTextEdit()
        self.detail_view.setReadOnly(True)
        self.detail_view.setStyleSheet("background-color: #1f2335; border: 1px solid #414868; border-radius: 8px; font-size: 16px; padding: 10px; color: #cfc9c2;")
        right_panel.addWidget(self.detail_view, stretch=3)

        # NÚT KẾT NỐI LẠI USB THAY THẾ CHO NÚT CHỤP THỦ CÔNG
        self.btn_reconnect = QPushButton("🔌 KẾT NỐI LẠI CÁP USB")
        self.btn_reconnect.setStyleSheet("""
            QPushButton { background-color: #7aa2f7; color: #1a1b26; font-weight: bold; font-size: 16px; padding: 15px; border-radius: 8px; margin-top: 10px; }
            QPushButton:hover { background-color: #9ece6a; }
        """)
        self.btn_reconnect.setCursor(Qt.PointingHandCursor)
        self.btn_reconnect.clicked.connect(self.reconnect_usb)
        right_panel.addWidget(self.btn_reconnect)

        self.btn_reset = QPushButton("🔄 LÀM MỚI (XÓA SẠCH GIAO DIỆN & FILE LƯU)")
        self.btn_reset.setStyleSheet("""
            QPushButton { background-color: #f7768e; color: #1a1b26; font-weight: bold; font-size: 14px; padding: 10px; border-radius: 8px; margin-top: 5px; }
            QPushButton:hover { background-color: #ff9eaf; }
        """)
        self.btn_reset.setCursor(Qt.PointingHandCursor)
        self.btn_reset.clicked.connect(self.clear_history)
        right_panel.addWidget(self.btn_reset)

        log_label = QLabel("LỊCH SỬ GIÁM SÁT (Đồng bộ test_images)")
        log_label.setStyleSheet("font-size: 16px; font-weight: bold; margin-top: 10px; color: #7dcfff;")
        right_panel.addWidget(log_label)

        self.history_list = QListWidget()
        self.history_list.setStyleSheet("""
            QListWidget { background-color: #24283b; border: 1px solid #414868; border-radius: 8px; font-size: 15px; }
            QListWidget::item { padding: 10px; border-bottom: 1px solid #1f2335; }
            QListWidget::item:selected { background-color: #33467c; }
        """)
        self.history_list.itemClicked.connect(self.load_history_image)
        right_panel.addWidget(self.history_list, stretch=6)

        main_layout.addLayout(right_panel, stretch=3)

        super_layout.addLayout(main_layout, stretch=9)

    def refresh_image_counter(self):
        if os.path.exists(WATCHDOG_DIR):
            existing_numbers = [1]
            for f in os.listdir(WATCHDOG_DIR):
                if f.lower().endswith('.jpg') and '_' in f and not f.startswith('temp_'):
                    prefix = f.split('_')[0]
                    if prefix.isdigit():
                        existing_numbers.append(int(prefix))
            self.image_counter = max(existing_numbers) + 1
        else:
            self.image_counter = 1

    def reconnect_usb(self):
        self.status_bar.setText("Trạng thái: Đang khởi động lại kết nối USB...")
        self.usb_thread.force_reconnect()

    def update_status_bar(self, message):
        self.status_bar.setText(f"Trạng thái: {message}")

    # =====================================================================
    # HÀM XỬ LÝ ẢNH CHÍNH
    # =====================================================================
    def process_image(self, path, transfer_time):
        try:
            original = cv2.imread(path)
            if original is None: return
        except: return

        infer_start = time.time()
        results = self.model.predict(source=path, conf=0.45, imgsz=800, verbose=False)
        infer_end = time.time()
        infer_time = (infer_end - infer_start) * 1000 
        
        img_plotted = results[0].plot(font_size=2, line_width=2).copy() 
        
        fps = 1.0 / (infer_end - self.last_frame_time) if (infer_end - self.last_frame_time) > 0 else 0.0
        self.last_frame_time = infer_end

        # --- CĂN CHỈNH BẢNG THÔNG SỐ (HUD) ---
        overlay = img_plotted.copy()
        cv2.rectangle(overlay, (10, 10), (320, 110), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, img_plotted, 0.4, 0, img_plotted)

        cv2.putText(img_plotted, f"FPS: {fps:.1f} frames/sec", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 180, 0), 2, cv2.LINE_AA)
        cv2.putText(img_plotted, f"USB Transfer: {transfer_time:.1f} ms", (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 150, 220), 2, cv2.LINE_AA)
        cv2.putText(img_plotted, f"AI Inference: {infer_time:.1f} ms", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 0), 2, cv2.LINE_AA)

        boxes = results[0].boxes
        valid_detections = []
        
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            if (x2 - x1) < 80 or (y2 - y1) < 80: continue 
                
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            vn_name = self.model.names[cls_id]
            
            if vn_name.lower() == "bệnh cháy sớm":
                vn_name = "Bệnh Cháy Lá"

            valid_detections.append({'name': vn_name, 'conf': conf})

        final_diagnosis = "Chưa xác định"
        final_conf = 0.0
        is_sick = False
        
        if len(valid_detections) == 0:
            final_diagnosis = "Lá khỏe mạnh"
        else:
            diseases = [d for d in valid_detections if d['name'] != 'Lá khỏe mạnh']
            if len(diseases) > 0:
                is_sick = True
                primary_disease = max(diseases, key=lambda x: x['conf'])
                final_diagnosis = primary_disease['name']
                final_conf = primary_disease['conf']
            else:
                best_healthy = max(valid_detections, key=lambda x: x['conf'])
                final_diagnosis = "Lá khỏe mạnh"
                final_conf = best_healthy['conf']

        remedy_txt = ""
        for key, val in self.expert_db.items():
            if val['name'].lower() == final_diagnosis.lower():
                remedy_txt = val['remedy']
                break
                
        if is_sick:
            info_text = f"🚨 CẢNH BÁO BỆNH: {final_diagnosis.upper()} (Độ tin cậy: {final_conf*100:.1f}%)\n\n📍 HƯỚNG DẪN XỬ LÝ:\n{remedy_txt}"
        else:
            info_text = f"✅ TÌNH TRẠNG TỐT: {final_diagnosis.upper()}\n\n📍 HƯỚNG DẪN:\n{remedy_txt}"
            
        self.detail_view.setText(info_text)

        safe_name = remove_accents(final_diagnosis).lower()
        snap_filename = f"{self.image_counter:02d}_{safe_name}.jpg"
        
        snap_path = os.path.join(WATCHDOG_DIR, snap_filename)
        cv2.imwrite(snap_path, img_plotted)
        
        # Lưu thêm vào thư mục Luu_Tru_Anh
        save_archive_path = os.path.join(SAVED_IMAGES_DIR, snap_filename)
        cv2.imwrite(save_archive_path, img_plotted)
        
        self.image_counter += 1 

        self.display_image(snap_path)

        time_str = datetime.now().strftime('%H:%M:%S')
        status_icon = "🔴" if is_sick else "🟢"
        list_item = QListWidgetItem(f"{time_str} - {status_icon} {final_diagnosis}")
        list_item.setData(Qt.UserRole, snap_path) 
        list_item.setData(Qt.UserRole + 1, info_text) 
        self.history_list.insertItem(0, list_item) 

        self.status_bar.setText(f"Trạng thái: Đã lưu ({snap_filename}) - AI: {infer_time:.1f}ms")

        # ==============================================================
        # TÍN HIỆU HOÀN THÀNH (DONE) GỬI XUỐNG XE ĐỂ XE TIẾP TỤC CHẠY
        # ==============================================================
        self.usb_thread.send_command("DONE\n")
        self.status_bar.setText(f"Trạng thái: AI quét xong! Đã gửi tín hiệu DONE cho xe chạy tiếp!")

        try: os.remove(path)
        except Exception: pass

    def display_image(self, img_path):
        img = cv2.imread(img_path)
        if img is None: return
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qimg = QImage(rgb.data, w, h, ch*w, QImage.Format_RGB888)
        pix = QPixmap.fromImage(qimg).scaled(1200, 800, Qt.KeepAspectRatio)
        self.img_screen.setPixmap(pix)

    def load_history_image(self, item):
        saved_path = item.data(Qt.UserRole)
        saved_info = item.data(Qt.UserRole + 1)
        if os.path.exists(saved_path):
            self.display_image(saved_path)
            self.detail_view.setText(f"[XEM LẠI LỊCH SỬ]\n{saved_info}")
            self.status_bar.setText(f"Trạng thái: Đang xem lại ảnh ({os.path.basename(saved_path)})")

    def clear_history(self):
        self.history_list.clear()
        self.detail_view.clear()
        self.img_screen.clear()
        self.img_screen.setText("ĐANG CHỜ ẢNH CAMERA TỪ XE TỰ HÀNH...")
        self.status_bar.setText("Trạng thái: Lệnh Hard Reset thành công! Toàn bộ file ảnh đã bị xóa sạch.")
        
        if os.path.exists(WATCHDOG_DIR):
            for f in os.listdir(WATCHDOG_DIR):
                try: os.remove(os.path.join(WATCHDOG_DIR, f))
                except Exception: pass
                
        if os.path.exists(SAVED_IMAGES_DIR):
            for f in os.listdir(SAVED_IMAGES_DIR):
                try: os.remove(os.path.join(SAVED_IMAGES_DIR, f))
                except Exception: pass

        self.image_counter = 1
        self.last_frame_time = time.time()

    def closeEvent(self, event):
        self.usb_thread.stop()
        self.usb_thread.wait()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setFont(QFont("Arial", 10))
    ex = TomatoExpertGUI()
    ex.show()
    sys.exit(app.exec_())