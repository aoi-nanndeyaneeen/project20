import serial
import threading
import time
import re

class SerialReceiver:
    def __init__(self, port="COM7", baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.latest_altitude = 0.0
        self.latest_accel = (0.0, 0.0, 0.0)
        self._accel_received = False  # デバッグ用フラグ
        self.is_running = True

        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            print(f"[SerialReceiver] {port} でRP2040と接続しました。")
        except Exception as e:
            print(f"[SerialReceiver] エラー: {port} が開けません。詳細: {e}")
            self.is_running = False
            return

        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()

    def _receive_loop(self):
        while self.is_running:
            try:
                if self.ser.in_waiting > 0:
                    raw = self.ser.readline()
                    line = raw.decode('utf-8', errors='ignore').strip()

                    if not line:
                        continue

                    # デバッグ：受信した生ラインを表示（確認後コメントアウト可）
                    # print(f"[RAW] {repr(line)}")

                    # 高度: "Alt  : -8.92 m" など
                    if "Alt" in line:
                        m = re.search(r"[-+]?\d+\.?\d*", line)
                        if m:
                            self.latest_altitude = float(m.group())

                    # 加速度: "Accel: [-45.96, -84.84, 152.80] g" など
                    elif "Accel" in line or "accel" in line:
                        nums = re.findall(r"[-+]?\d+\.?\d*", line)
                        if len(nums) >= 3:
                            self.latest_accel = (
                                float(nums[0]),
                                float(nums[1]),
                                float(nums[2])
                            )
                            if not self._accel_received:
                                print(f"[SerialReceiver] 加速度の受信を確認: {self.latest_accel}")
                                self._accel_received = True

            except Exception as e:
                print(f"[SerialReceiver] 受信エラー: {e}")
            time.sleep(0.005)  # 0.01→0.005に短縮して取りこぼし減少

    def get_altitude(self):
        return self.latest_altitude

    def get_accel(self):
        if not self._accel_received:
            # まだ一度も受信できていない場合に警告（頻繁に出ないよう1回のみ）
            print("[SerialReceiver] 警告: Accelデータ未受信。フォーマットを確認してください。")
            self._accel_received = True  # 警告は1回だけ
        return self.latest_accel

    def stop(self):
        self.is_running = False
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()