import serial
import struct
import threading
import time
import math

class SerialReceiver:
    def __init__(self, port="COM3", baudrate=115200, mock_mode=False, 
                 mock_min_z=0.05, mock_max_z=0.4, mock_speed=1.0):
        """
        mock_mode=True の場合、ダミーデータを生成します。
        mock_min_z: ダミーの最小の高さ(m)
        mock_max_z: ダミーの最大の高さ(m)
        mock_speed: 上下するスピード
        """
        self.mock_mode = mock_mode
        self.current_z = (mock_min_z + mock_max_z) / 2.0
        self.running = False
        
        # モック（ダミー）用の設定パラメータ
        self.mock_min_z = mock_min_z
        self.mock_max_z = mock_max_z
        self.mock_speed = mock_speed
        self.start_time = time.time()
        
        if not self.mock_mode:
            try:
                self.ser = serial.Serial(port, baudrate, timeout=1)
                print(f"[Serial] {port} で通信を開始しました。")
            except Exception as e:
                print(f"[Serial] エラー: {e}")
                print("[Serial] モック(テスト)モードで起動します。")
                self.mock_mode = True

    def start(self):
        self.running = True
        self.start_time = time.time() # ダミー波形用の時間リセット
        if not self.mock_mode:
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()

    def _read_loop(self):
        buffer = ""
        while self.running:
            if self.ser.in_waiting > 0:
                try:
                    char = self.ser.read().decode('ascii')
                    if char == '\n':
                        self._decode_hex_string(buffer.strip())
                        buffer = ""
                    else:
                        buffer += char
                except Exception as e:
                    pass
            time.sleep(0.001)

    def _decode_hex_string(self, hex_str):
        try:
            if len(hex_str) >= 8:
                b = bytes.fromhex(hex_str[:8])
                z_val = struct.unpack('<f', b)[0]
                self.current_z = z_val
        except Exception as e:
            pass

    def get_z(self):
        """現在のZ座標(高さ)を返す"""
        if self.mock_mode:
            # 時間経過を使ってサイン波(波打ち)を計算し、滑らかに上下させる
            elapsed = time.time() - self.start_time
            amplitude = (self.mock_max_z - self.mock_min_z) / 2.0
            center = (self.mock_max_z + self.mock_min_z) / 2.0
            
            # math.sin は -1.0 〜 +1.0 の値を返す
            self.current_z = center + amplitude * math.sin(elapsed * self.mock_speed)
            
        return self.current_z

    def stop(self):
        self.running = False
        if not self.mock_mode:
            self.ser.close()