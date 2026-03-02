import serial
import threading
import time
import re
import numpy as np

class SerialReceiver:
    def __init__(self, port="COM7", baudrate=115200, dummy=False):
        self.port = port
        self.baudrate = baudrate
        self.latest_altitude = 0.0
        self.latest_accel = (0.0, 0.0, 0.0)
        self.latest_angles = (0.0, 0.0, 0.0) # roll, pitch, yaw
        self._received = False
        self.is_running = True
        self.dummy = dummy

        if self.dummy:
            print("[SerialReceiver] ダミーモードで起動しました。")
            self.thread = threading.Thread(target=self._dummy_loop, daemon=True)
            self.thread.start()
            return

        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            print(f"[SerialReceiver] {port} で受信機と接続しました。")
        except Exception as e:
            print(f"[SerialReceiver] エラー: {port} が開けません。ダミーモードに切り替えます。")
            self.dummy = True
            self.thread = threading.Thread(target=self._dummy_loop, daemon=True)
            self.thread.start()
            return

        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()

    def _receive_loop(self):
        while self.is_running:
            try:
                if self.ser.in_waiting > 0:
                    raw = self.ser.readline()
                    line = raw.decode('utf-8', errors='ignore').strip()
                    if not line: continue

                    # 形式: DATA,高度,Roll,Pitch,Yaw
                    if line.startswith("DATA,"):
                        parts = line.split(',')
                        if len(parts) >= 5:
                            self.latest_altitude = float(parts[1])
                            self.latest_angles = (
                                float(parts[2]),
                                float(parts[3]),
                                float(parts[4])
                            )
                            # 互換性のためのダミー加速度（重力成分のみ）
                            r, p = np.radians(self.latest_angles[0]), np.radians(self.latest_angles[1])
                            self.latest_accel = (np.sin(p), -np.sin(r)*np.cos(p), np.cos(r)*np.cos(p))
                            self._received = True

            except Exception as e:
                print(f"[SerialReceiver] 受信エラー: {e}")
            time.sleep(0.005)

    def _dummy_loop(self):
        """テキトーな動きを生成するダミーデータループ"""
        start_time = time.time()
        while self.is_running:
            t = time.time() - start_time
            # 高度: 5m〜10mをゆっくり上下
            self.latest_altitude = 7.5 + 2.5 * np.sin(t * 0.5)
            # 角度: 旋回とピッチ変動をシミュレート
            roll = 20.0 * np.sin(t * 0.8)
            pitch = 10.0 * np.cos(t * 1.2)
            yaw = (t * 10) % 360
            self.latest_angles = (roll, pitch, yaw)
            
            # 互換性のための加速度
            r, p = np.radians(roll), np.radians(pitch)
            self.latest_accel = (np.sin(p), -np.sin(r)*np.cos(p), np.cos(r)*np.cos(p))
            
            self._received = True
            time.sleep(0.02) # 50Hz

    def get_altitude(self):
        return self.latest_altitude

    def get_accel(self):
        return self.latest_accel

    def get_angles(self):
        return self.latest_angles

    def stop(self):
        self.is_running = False
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
