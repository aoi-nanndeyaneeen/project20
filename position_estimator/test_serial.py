import serial
import time
from datetime import datetime

# ★ "COM7" の部分は、実際のRP2040のCOMポート番号に合わせてください
PORT = "COM7"
BAUDRATE = 115200

# ★ ログファイル名（自動で日時付き）
log_filename = f"serial_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"

try:
    print(f"{PORT} に接続を試みます...")
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)

    print("接続成功！データを受信します（終了は Ctrl+C）")
    print(f"ログ保存先: {log_filename}")
    print("-" * 30)

    # ログファイルを開く（追記モード）
    with open(log_filename, "a", encoding="utf-8") as logfile:

        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()

                # タイムスタンプ作成
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

                # コンソール表示
                print(f"[{timestamp}] 受信: {line}")

                # ファイルへ書き込み
                logfile.write(f"[{timestamp}] {line}\n")
                logfile.flush()  # 即保存（重要）

            time.sleep(0.01)

except KeyboardInterrupt:
    print("\nログ取得を終了しました。")

except Exception as e:
    print(f"\n【エラー発生】{PORT} に接続できませんでした！")
    print(f"詳細: {e}")
    print("→ VS Codeのシリアルモニタが開きっぱなしになっていませんか？")