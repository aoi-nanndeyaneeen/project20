import cv2
import numpy as np
import matplotlib.pyplot as plt
import csv
import datetime
import os

from camera import CameraTracker
from communication import SerialReceiver

# ==========================================
# 設定
# ==========================================
FIELD_POINTS = np.array([
    [0.0, 0.0, 0.0],
    [0.297, 0.0, 0.0],
    [0.297, 0.210, 0.0],
    [0.0, 0.210, 0.0]
], dtype=np.float32)

def get_ray(u, v, K, R, tvec):
    pt_2d = np.array([[[u, v]]], dtype=np.float32)
    undistorted = cv2.undistortPoints(pt_2d, K, None)
    dir_cam = np.array([undistorted[0][0][0], undistorted[0][0][1], 1.0])
    cam_pos = -R.T.dot(tvec).flatten()
    dir_world = R.T.dot(dir_cam)
    dir_world /= np.linalg.norm(dir_world)
    return cam_pos, dir_world.flatten()

def accel_to_angles(accel):
    """生の加速度ベクトルから絶対roll/pitchを計算（スケール不問）
    ヨーは加速度計単体では取得不可（要ジャイロ積分 or 磁気センサ）
    """
    ax, ay, az = accel
    norm = np.sqrt(ax**2 + ay**2 + az**2)
    if norm < 1e-6:
        return 0.0, 0.0
    # 正規化して重力方向だけ取り出す
    ax, ay, az = ax/norm, ay/norm, az/norm
    pitch = np.degrees(np.arctan2(-ax, np.sqrt(ay**2 + az**2)))
    roll  = np.degrees(np.arctan2(ay, az))
    return roll, pitch

def calc_tilt(accel, ref_roll, ref_pitch):
    """現在の絶対角度からキャリブ時の基準角度を引いた相対傾き"""
    roll, pitch = accel_to_angles(accel)
    return roll - ref_roll, pitch - ref_pitch

def draw_horizon(frame, roll_deg, pitch_deg):
    """人工水平儀をフレーム左上に描画"""
    cx, cy, r = 120, 120, 90  # 中心座標・半径

    # 背景円
    cv2.circle(frame, (cx, cy), r, (40, 40, 40), -1)
    cv2.circle(frame, (cx, cy), r, (200, 200, 200), 2)

    # 地平線：pitchで上下オフセット、rollで回転
    pitch_px = int(pitch_deg / 90.0 * r)
    angle_rad = np.radians(roll_deg)
    cos_a, sin_a = np.cos(angle_rad), np.sin(angle_rad)

    # 地平線の両端点（回転＋オフセット適用）
    def horizon_pt(sign):
        px = int(cx + sign * r * cos_a - pitch_px * sin_a)
        py = int(cy + sign * r * sin_a + pitch_px * cos_a)
        return (px, py)

    p1, p2 = horizon_pt(-1), horizon_pt(1)

    # 地平線より下（空色）を塗りつぶし
    mask = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)
    cv2.circle(mask, (cx, cy), r - 1, 255, -1)
    sky_mask = np.zeros_like(mask)
    dx, dy = -(p2[1] - p1[1]), p2[0] - p1[0]  # 法線方向
    pts = np.array([p1, p2,
                    (p2[0] + dx * 5, p2[1] + dy * 5),
                    (p1[0] + dx * 5, p1[1] + dy * 5)], dtype=np.int32)
    cv2.fillPoly(sky_mask, [pts], 255)
    cv2.bitwise_and(sky_mask, mask, sky_mask)

    # 空色（青）と地面色（茶）を描画
    frame[mask == 255] = (100, 60, 30)       # 地面
    frame[sky_mask == 255] = (180, 120, 40)  # 空

    # 地平線ライン
    cv2.line(frame, p1, p2, (255, 255, 255), 2)

    # 機体基準の中心マーカー
    cv2.line(frame, (cx - 30, cy), (cx - 10, cy), (0, 255, 255), 3)
    cv2.line(frame, (cx + 10, cy), (cx + 30, cy), (0, 255, 255), 3)
    cv2.circle(frame, (cx, cy), 4, (0, 255, 255), -1)

    # 外枠を再描画
    cv2.circle(frame, (cx, cy), r, (200, 200, 200), 2)

    # テキスト
    cv2.putText(frame, f"Roll :{roll_deg:+.1f}deg",  (10, 230),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    cv2.putText(frame, f"Pitch:{pitch_deg:+.1f}deg", (10, 260),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

# ==========================================
# メイン処理
# ==========================================
if __name__ == "__main__":
    print("システムを起動中...")

    log_filename = "3d_tracking_log.csv"
    if not os.path.exists(log_filename):
        with open(log_filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Time", "Target_X(m)", "Target_Y(m)", "Target_Z(m)",
                             "Roll(deg)", "Pitch(deg)"])

    cam = CameraTracker(camera_id=0, width=1920, height=1080)
    alt_sensor = SerialReceiver(port="COM7", baudrate=115200)

    K = cam.get_approx_camera_matrix()
    points_2d = []

    # --- キャリブレーション用オフセット ---
    alt_offset  = 0.0
    ref_roll    = 0.0   # キャリブ時の絶対roll
    ref_pitch   = 0.0   # キャリブ時の絶対pitch

    cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)

    def click_calib(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(points_2d) < 4:
            points_2d.append([x, y])

    cv2.setMouseCallback("Camera", click_calib)
    print("【準備】基準点4箇所（左下→右下→右上→左上）をクリックしてください。")

    try:
        while True:
            ret, frame = cam.cap.read()
            if not ret:
                continue
            display = frame.copy()
            for i, p in enumerate(points_2d):
                cv2.circle(display, tuple(p), 5, (0, 0, 255), -1)
                cv2.putText(display, str(i + 1), (p[0] + 10, p[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.imshow("Camera", display)
            if cv2.waitKey(1) == 13 and len(points_2d) == 4:
                break

        success, rvec, tvec = cv2.solvePnP(
            FIELD_POINTS, np.array(points_2d, dtype=np.float32), K, None)
        R, _ = cv2.Rodrigues(rvec)

        plt.ion()
        global fig, ax
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111, projection='3d')

        def ensure_plot():
            """matplotlibウィンドウが閉じられていたら再生成"""
            global fig, ax
            if not plt.fignum_exists(fig.number):
                fig = plt.figure(figsize=(8, 8))
                ax = fig.add_subplot(111, projection='3d')

        print("【開始】トラッキング開始！")
        print("  Space: 現在の高度・傾きでキャリブレーション")
        print("  b    : 背景リセット")
        print("  q    : 終了")

        with open(log_filename, mode='a', newline='') as log_file:
            csv_writer = csv.writer(log_file)

            while True:
                frame, center_uv = cam.read_and_track()
                if frame is None:
                    break

                raw_alt   = alt_sensor.get_altitude()
                raw_accel = alt_sensor.get_accel()
                current_z = raw_alt - alt_offset
                roll, pitch = calc_tilt(raw_accel, ref_roll, ref_pitch)

                # 人工水平儀を描画
                draw_horizon(frame, roll, pitch)

                # キャリブレーション状態を表示
                cv2.putText(frame, f"Alt(raw):{raw_alt:.2f}m  offset:{alt_offset:.2f}m",
                            (10, 310), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (180, 180, 180), 2)
                cv2.putText(frame, "[SPACE] Calibrate  [B] BG Reset  [Q] Quit",
                            (10, frame.shape[0] - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)

                if center_uv is not None:
                    u, v = center_uv
                    O, D = get_ray(u, v, K, R, tvec)

                    if D[2] != 0:
                        t = (current_z - O[2]) / D[2]
                        P = O + t * D

                        current_time = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        csv_writer.writerow([current_time,
                                             round(P[0], 3), round(P[1], 3), round(current_z, 3),
                                             round(roll, 2), round(pitch, 2)])

                        text = f"X:{P[0]:.2f} Y:{P[1]:.2f} Z:{current_z:.2f}m"
                        cv2.putText(frame, text, (50, 100),
                                    cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 255), 4)

                        ax.cla()
                        ensure_plot()
                        
                        # 描画範囲の設定
                        side = 1.0
                        ax.set_xlim(-side, side)
                        ax.set_ylim(-side, side)
                        ax.set_zlim(0, side * 1.5)
                        ax.set_box_aspect((1, 1, 1.5))

                        # フィールドの描画
                        xs = np.append(FIELD_POINTS[:, 0], FIELD_POINTS[0, 0])
                        ys = np.append(FIELD_POINTS[:, 1], FIELD_POINTS[0, 1])
                        zs = np.append(FIELD_POINTS[:, 2], FIELD_POINTS[0, 2])
                        ax.plot(xs, ys, zs, color='gray', alpha=0.5, label="Field")

                        # カメラ位置
                        ax.scatter(*O, color='red', s=50, label="Camera")

                        # 飛行機の描画 (簡易的な十字＋尾翼)
                        def draw_plane(pos, r, p, y, scale=0.1):
                            # ローカル座標系でのパーツ定義
                            # 胴体(X軸方向), 翼(Y軸方向)
                            body = np.array([[-1, 0, 0], [1, 0, 0]]) * scale
                            wing = np.array([[0, -1.2, 0], [0, 1.2, 0]]) * scale
                            tail = np.array([[-0.8, -0.4, 0], [-0.8, 0.4, 0]]) * scale
                            vert = np.array([[-0.8, 0, 0], [-0.8, 0, 0.4]]) * scale

                            # 回転行列の作成 (yaw -> pitch -> roll)
                            def get_R(r_deg, p_deg, y_deg):
                                r, p, y = np.radians(r_deg), np.radians(p_deg), np.radians(y_deg)
                                Rx = np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)]])
                                Ry = np.array([[np.cos(p),0,np.sin(p)],[0,1,0],[-np.sin(p),0,np.cos(p)]])
                                Rz = np.array([[np.cos(y),-np.sin(y),0],[np.sin(y),np.cos(y),0],[0,0,1]])
                                return Rz @ Ry @ Rx

                            R_plane = get_R(r, p, y)
                            
                            for part, color in [(body, 'blue'), (wing, 'blue'), (tail, 'red'), (vert, 'red')]:
                                pts = (R_plane @ part.T).T + pos
                                ax.plot(pts[:,0], pts[:,1], pts[:,2], color=color, linewidth=3)

                        _, _, yaw = alt_sensor.get_angles()
                        draw_plane(P, roll, pitch, yaw)

                        ax.set_xlabel('X(m)'); ax.set_ylabel('Y(m)'); ax.set_zlabel('Z(m)')
                        ax.set_title(f"3D Live View\nAlt: {current_z:.2f}m")
                        
                        try:
                            plt.draw()
                            plt.pause(0.001)
                        except Exception:
                            ensure_plot()

                cv2.imshow("Camera", frame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('b'):
                    cam.prev_gray = None
                    print("背景をリセットしました")
                elif key == ord(' '):
                    alt_offset = raw_alt
                    ref_roll, ref_pitch = accel_to_angles(raw_accel)
                    print(f"[キャリブレーション] 高度オフセット={alt_offset:.2f}m, "
                          f"基準角度: Roll={ref_roll:.1f}°, Pitch={ref_pitch:.1f}°")

    finally:
        cam.release()
        alt_sensor.stop()
        cv2.destroyAllWindows()