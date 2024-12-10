import sys
import socket
import threading
import time
import sqlite3
from datetime import datetime
from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QVBoxLayout,
    QDialog,
    QLineEdit,
    QLabel,
    QPushButton,
    QHBoxLayout,
    QMessageBox,
    QComboBox,
)
from PyQt6.QtCore import QTimer, pyqtSignal
from interfaz import Ui_MainWindow

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

class MplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super(MplCanvas, self).__init__(fig)

class SamplingDialog(QDialog):
    frequency_changed = pyqtSignal(float)

    def __init__(self, current_frequency, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Configurar Frecuencia de Muestreo")
        self.current_frequency = current_frequency

        layout = QVBoxLayout()
        self.label = QLabel(f"Frecuencia de muestreo actual: {self.current_frequency} ms")
        layout.addWidget(self.label)

        self.input_label = QLabel("Nueva frecuencia de muestreo (ms):")
        layout.addWidget(self.input_label)

        self.frequency_input = QLineEdit()
        layout.addWidget(self.frequency_input)

        button_layout = QHBoxLayout()
        self.ok_button = QPushButton("Aceptar")
        self.cancel_button = QPushButton("Cancelar")
        button_layout.addWidget(self.ok_button)
        button_layout.addWidget(self.cancel_button)

        layout.addLayout(button_layout)
        self.setLayout(layout)

        self.ok_button.clicked.connect(self.accept)
        self.cancel_button.clicked.connect(self.reject)

    def accept(self):
        try:
            new_frequency = float(self.frequency_input.text())
            if new_frequency <= 0:
                raise ValueError("La frecuencia debe ser mayor que cero.")
            self.frequency_changed.emit(new_frequency)
            super().accept()
        except ValueError as e:
            QMessageBox.warning(self, "Valor inválido", str(e))

class ModeDialog(QDialog):
    mode_changed = pyqtSignal(int)

    def __init__(self, current_mode, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Configurar Modo de Envío de Datos")
        self.current_mode = current_mode

        layout = QVBoxLayout()
        self.label = QLabel(f"Modo actual: {'Datos en Bruto' if current_mode == 1 else 'Datos Procesados'}")
        layout.addWidget(self.label)

        self.mode_label = QLabel("Seleccionar nuevo modo:")
        layout.addWidget(self.mode_label)

        self.mode_combo = QComboBox()
        self.mode_combo.addItem("Modo 1: Datos en Bruto", 1)
        self.mode_combo.addItem("Modo 2: Datos Procesados", 2)
        layout.addWidget(self.mode_combo)

        button_layout = QHBoxLayout()
        self.ok_button = QPushButton("Aceptar")
        self.cancel_button = QPushButton("Cancelar")
        button_layout.addWidget(self.ok_button)
        button_layout.addWidget(self.cancel_button)

        layout.addLayout(button_layout)
        self.setLayout(layout)

        self.ok_button.clicked.connect(self.accept)
        self.cancel_button.clicked.connect(self.reject)

    def accept(self):
        new_mode = self.mode_combo.currentData()
        self.mode_changed.emit(new_mode)
        super().accept()

class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.server_running = True
        self.data_sending_enabled = False
        self.current_frequency = 1000
        self.current_mode = 1  # Modo inicial: Datos en Bruto

        self.temperature_data = []
        self.humidity_data = []
        self.time_data = []
        self.start_time = None

        self.temp_canvas = MplCanvas(self, width=5, height=4, dpi=100)
        self.hum_canvas = MplCanvas(self, width=5, height=4, dpi=100)

        self.temp_layout = QVBoxLayout(self.temp_label)
        self.temp_layout.addWidget(self.temp_canvas)

        self.hum_layout = QVBoxLayout(self.hum_label)
        self.hum_layout.addWidget(self.hum_canvas)

        threading.Thread(target=self.start_server, daemon=True).start()

        self.inicio_button.clicked.connect(self.toggle_data_sending)
        self.muestreo_button.clicked.connect(self.open_sampling_dialog)
        self.datos_button.clicked.connect(self.open_mode_dialog)

        self.timer = QTimer()
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self.update_plots)
        self.timer.start()

        self.current_conn = None

        self.init_database()

    def init_database(self):
        conn = sqlite3.connect("sensor_data.db")
        cursor = conn.cursor()
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS samples (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT NOT NULL,
                temperature REAL NOT NULL,
                temp_max REAL NOT NULL,
                temp_min REAL NOT NULL,
                temp_mean REAL NOT NULL,
                humidity REAL NOT NULL,
                hum_max REAL NOT NULL,
                hum_min REAL NOT NULL,
                hum_mean REAL NOT NULL
            )
        """)
        conn.commit()
        conn.close()

    def toggle_data_sending(self):
        self.data_sending_enabled = not self.data_sending_enabled
        if self.data_sending_enabled:
            self.inicio_button.setText("Stop Data")
            self.temperature_data = []
            self.humidity_data = []
            self.time_data = []
            self.start_time = time.time()
            
            # Enviar comando "DATA" al cliente
            if self.current_conn:
                try:
                    self.current_conn.sendall("DATA\n".encode())
                    print("Comando enviado: DATA")
                except Exception as e:
                    print(f"Error al enviar comando DATA: {e}")
        else:
            self.inicio_button.setText("Start Data")

            # Enviar comando "STOP" al cliente
            if self.current_conn:
                try:
                    self.current_conn.sendall("STOP\n".encode())
                    print("Comando enviado: STOP")
                except Exception as e:
                    print(f"Error al enviar comando STOP: {e}")

    def open_sampling_dialog(self):
        dialog = SamplingDialog(self.current_frequency, self)
        dialog.frequency_changed.connect(self.set_new_frequency)
        dialog.exec()

    def open_mode_dialog(self):
        dialog = ModeDialog(self.current_mode, self)
        dialog.mode_changed.connect(self.set_new_mode)
        dialog.exec()

    def set_new_frequency(self, new_frequency):
        self.current_frequency = new_frequency
        if self.current_conn:
            try:
                # Enviar el comando en el formato "SET_FREQ <frecuencia>"
                message = f"SET_FREQ {int(new_frequency)}\n"
                self.current_conn.sendall(message.encode())
                print(f"Nueva frecuencia enviada: {message.strip()}")
            except Exception as e:
                print(f"Error al enviar la nueva frecuencia: {e}")

    def set_new_mode(self, new_mode):
        self.current_mode = new_mode
        if self.current_conn:
            try:
                # Formato del mensaje ajustado para enviar MODE1 o MODE2
                message = f"MODE{new_mode}\n"
                self.current_conn.sendall(message.encode())
                print(f"Nuevo modo enviado: {message.strip()}")
            except Exception as e:
                print(f"Error al enviar el nuevo modo: {e}")

    def start_server(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Permitir reutilizar la direccion inmediatamente
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(("0.0.0.0", 12345))
        self.server_socket.listen(1)
        print("Server listening on 0.0.0.0:12345")

        while self.server_running:
            conn, addr = self.server_socket.accept()
            self.current_conn = conn
            print(f"Connection from {addr}")
            threading.Thread(target=self.handle_client, args=(conn,), daemon=True).start()

    def handle_client(self, conn):
        thread_conn = sqlite3.connect("sensor_data.db")
        thread_cursor = thread_conn.cursor()

        while self.server_running:
            try:
                data = conn.recv(64)
                if not data:
                    break
                data_str = data.decode().strip()
                # Procesamiento de mensajes
                if data_str.startswith("DATA"):
                    if self.data_sending_enabled:
                        try:
                            # Ajustar el separador para espacios
                            _, temperature_str, humidity_str = data_str.split()
                            temperature = float(temperature_str)
                            humidity = float(humidity_str)

                            if self.start_time is None:
                                self.start_time = time.time()

                            current_time = time.time() - self.start_time
                            self.temperature_data.append(temperature)
                            self.humidity_data.append(humidity)
                            self.time_data.append(current_time)

                            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                            # Insertar el dato con todos los campos (temp=hum=valores individuales)
                            thread_cursor.execute("""
                                INSERT INTO samples (timestamp, temperature, temp_max, temp_min, temp_mean, humidity, hum_max, hum_min, hum_mean) 
                                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                            """, (timestamp, temperature, temperature, temperature, temperature,
                                  humidity, humidity, humidity, humidity))
                            thread_conn.commit()

                            self.lcd_Temp.display(temperature)
                            self.lcd_Hum.display(humidity)

                        except ValueError as e:
                            print(f"Error al decodificar los datos: {data_str} -> {e}")
                            continue

                elif data_str.startswith("STATS"):
                    # Mensaje STATS: "STATS max_temp min_temp mean_temp max_hum min_hum mean_hum"
                    parts = data_str.split()
                    if len(parts) == 7:
                        try:
                            max_temp = float(parts[1])
                            min_temp = float(parts[2])
                            mean_temp = float(parts[3])
                            max_hum = float(parts[4])
                            min_hum = float(parts[5])
                            mean_hum = float(parts[6])

                            # Actualizar las etiquetas con los valores de temperatura y humedad
                            self.max_esp_label.setText(f"{max_temp:.2f}\n{max_hum:.2f}")
                            self.min_esp_label.setText(f"{min_temp:.2f}\n{min_hum:.2f}")
                            self.mean_esp_label.setText(f"{mean_temp:.2f}\n{mean_hum:.2f}")

                            self.max_raspi_label.setText(f"{max_temp:.2f}\n{max_hum:.2f}")
                            self.min_raspi_label.setText(f"{min_temp:.2f}\n{min_hum:.2f}")
                            self.mean_raspi_label.setText(f"{((max_temp+min_temp)/2):.2f}\n{((max_hum+min_hum)/2):.2f}")

                            # Guardar los datos de STATS en la base de datos
                            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                            thread_cursor.execute("""
                                INSERT INTO samples (timestamp, temperature, temp_max, temp_min, temp_mean, humidity, hum_max, hum_min, hum_mean)
                                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                            """, (timestamp, mean_temp, max_temp, min_temp, mean_temp, mean_hum, max_hum, min_hum, mean_hum))
                            thread_conn.commit()

                        except ValueError as e:
                            print(f"Error al procesar STATS: {data_str} -> {e}")
                    else:
                        print(f"Formato STATS incorrecto: {data_str}")

                else:
                    print(f"Mensaje no reconocido: {data_str}")
            except Exception as e:
                print(f"Error en la comunicacion: {e}")
                break

        thread_conn.close()
        conn.close()

    def update_plots(self):
        if self.data_sending_enabled and self.time_data:
            self.temp_canvas.axes.cla()
            self.temp_canvas.axes.plot(self.time_data, self.temperature_data, 'r-')
            self.temp_canvas.axes.set_title('Temperatura en el tiempo')
            self.temp_canvas.axes.set_xlabel('Tiempo (s)')
            self.temp_canvas.axes.set_ylabel('Temperatura (°C)')
            self.temp_canvas.axes.grid(True)
            self.temp_canvas.draw()

            self.hum_canvas.axes.cla()
            self.hum_canvas.axes.plot(self.time_data, self.humidity_data, 'b-')
            self.hum_canvas.axes.set_title('Humedad en el tiempo')
            self.hum_canvas.axes.set_xlabel('Tiempo (s)')
            self.hum_canvas.axes.set_ylabel('Humedad (%)')
            self.hum_canvas.axes.grid(True)
            self.hum_canvas.draw()

    def closeEvent(self, event):
        self.server_running = False
        if hasattr(self, 'server_socket'):
            self.server_socket.close()
            print("Server socket closed.")
        event.accept()

if __name__ == "__main__":
    app = QApplication([])
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
