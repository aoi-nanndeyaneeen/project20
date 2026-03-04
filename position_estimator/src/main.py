import cv2
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import csv
import datetime
import os
import threading

from camera import CameraTracker
from communication import SerialReceiver
from visualizer import Visualizer3D

# ==========================================
# 設定
# ==========================================
FIELD_POINTS = np.array([
    [0.0,  0.0,  0.0],
    [16.0, 0.0,  0.0],
    [16.0, 16.0, 0.0],
    [0.0,  16.0, 0.0]
], dtype=np.float32)

# 3D表示の固定範囲
FIELD_X_MIN, FIELD_X_MAX = -2.0, 18.0
FIELD_Y_MIN, FIELD_Y_MAX = -2.0, 18.0
FIELD_Z_MIN, FIELD_Z_MAX =  0.0, 15.0

# 外れ値フィルタ：この範囲外の検知は無視する（フィールドより少し広め）
VALID_X_MIN, VALID_X_MAX = -5.0, 21.0
VALID_Y_MIN, VALID_Y_MAX = -5.0, 21.0
VALID_Z_MIN, VALID_Z_MAX = -1.0, 20.0

# スレッド間共有
frame_lock = threading.Lock()
frame_data = {
    "frame": None,
    "updated": False,
}

plot_lock = threading.Lock()
plot_data = {
    "P": None, "O": None,
    "roll": 0.0, "pitch": 0.0, "current_z": 0.0,
    "updated": False,
}

# キー入力をメインスレッド（matplotlib側）で受け取るためのフラグ
key_flags = {
    "calib": False,
    "bg_reset": False,
    "quit": False,
}

# ==========================================
# 計算関数
# ==========================================
def get_ray(u, v, K, R, tvec):
    pt_2d = np.array([[[u, v]]], dtype=np.float32)
    undistorted = cv2.undistortPoints(pt_2d, K, None)
    dir_cam = np.array([undistorted[0][0][0], undistorted[0][0][1], 1.0])
    cam_pos = -R.T.dot(tvec).flatten()
    dir_world = R.T.dot(dir_cam)
    dir_world /= np.linalg.norm(dir_world)
    return cam_pos, dir_world.flatten()

def accel_to_angles(accel):
    ax_, ay, az = accel
    norm = np.sqrt(ax_**2 + ay**2 + az**2)
    if norm < 1e-6:
        return 0.0, 0.0
    ax_, ay, az = ax_/norm, ay/norm, az/norm
    pitch = np.degrees(np.arctan2(-ax_, np.sqrt(ay**2 + az**2)))
    roll  = np.degrees(np.arctan2(ay, az))
    return roll, pitch

def calc_tilt(accel, ref_roll, ref_pitch):
    roll, pitch = accel_to_angles(accel)
    return roll - ref_roll, pitch - ref_pitch

def is_valid_position(P):
    """フィールド外の明らかな外れ値を除外"""
    return (VALID_X_MIN <= P[0] <= VALID_X_MAX and
            VALID_Y_MIN <= P[1] <= VALID_Y_MAX and
            VALID_Z_MIN <= P[2] <= VALID_Z_MAX)

# ==========================================
# OpenCV描画関数
# ==========================================
def draw_horizon(frame, roll_deg, pitch_deg):
    cx, cy, r = 120, 120, 90
    cv2.circle(frame, (cx, cy), r, (40, 40, 40), -1)
    pitch_px  = int(np.clip(pitch_deg / 90.0 * r, -r, r))
    angle_rad = np.radians(roll_deg)
    cos_a, sin_a = np.cos(angle_rad), np.sin(angle_rad)

    def hp(sign):
        return (int(cx + sign*r*cos_a - pitch_px*sin_a),
                int(cy + sign*r*sin_a + pitch_px*cos_a))

    p1, p2 = hp(-1), hp(1)
    mask = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)
    cv2.circle(mask, (cx, cy), r-1, 255, -1)
    sky_mask = np.zeros_like(mask)
    dx, dy = -(p2[1]-p1[1]), p2[0]-p1[0]
    pts = np.array([p1, p2,
                    (p2[0]+dx*5, p2[1]+dy*5),
                    (p1[0]+dx*5, p1[1]+dy*5)], dtype=np.int32)
    cv2.fillPoly(sky_mask, [pts], 255)
    cv2.bitwise_and(sky_mask, mask, sky_mask)
    frame[mask == 255]     = (100, 60, 30)
    frame[sky_mask == 255] = (180, 120, 40)
    cv2.line(frame, p1, p2, (255,255,255), 2)
    cv2.line(frame, (cx-30,cy), (cx-10,cy), (0,255,255), 3)
    cv2.line(frame, (cx+10,cy), (cx+30,cy), (0,255,255), 3)
    cv2.circle(frame, (cx,cy), 4, (0,255,255), -1)
    cv2.circle(frame, (cx,cy), r, (200,200,200), 2)
    cv2.putText(frame, f"Roll :{roll_deg:+.1f}deg",  (10,230), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 2)
    cv2.putText(frame, f"Pitch:{pitch_deg:+.1f}deg", (10,260), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 2)
    cv2.putText(frame, "Yaw : N/A",                  (10,290), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128,128,128), 2)

# ==========================================
# OpenCVスレッド（カメラ・検知・ログ）
# ==========================================
def camera_thread_func(cam, alt_sensor, K, R, tvec, log_path, shared):
    """
    shared辞書でメインスレッドとデータ交換：
      shared["alt_offset"], shared["pos_offset"],
      shared["ref_roll"], shared["ref_pitch"]
    """
    O_fixed = (-R.T.dot(tvec)).flatten()
    last_P_raw = None

    write_header = not os.path.exists(log_path)
    log_fh = open(log_path, mode='a', newline='')
    writer = csv.writer(log_fh)
    if write_header:
        writer.writerow(["Time","Detected",
                         "Target_X(m)","Target_Y(m)","Target_Z(m)",
                         "Roll(deg)","Pitch(deg)",
                         "Alt_raw(m)","Alt_offset(m)"])

    cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)

    def on_click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(shared["points_2d"]) < 4:
            shared["points_2d"].append([x, y])
    cv2.setMouseCallback("Camera", on_click)

    try:
        while not shared.get("quit", False):
            raw_alt   = alt_sensor.get_altitude()
            raw_accel = alt_sensor.get_accel()

            alt_offset = shared["alt_offset"]
            pos_offset = shared["pos_offset"]
            ref_roll   = shared["ref_roll"]
            ref_pitch  = shared["ref_pitch"]

            current_z = raw_alt - alt_offset
            roll, pitch = calc_tilt(raw_accel, ref_roll, ref_pitch)

            frame, center_uv = cam.read_and_track()
            if frame is None:
                break

            draw_horizon(frame, roll, pitch)
            cv2.putText(frame,
                f"Alt(raw):{raw_alt:.2f}m  offset:{alt_offset:.2f}m  rel:{current_z:.2f}m",
                (10,340), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (180,180,180), 2)
            cv2.putText(frame, "[SPACE]Calib [B]BG Reset [Q]Quit",
                (10, frame.shape[0]-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 2)

            current_time = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
            P_vec = None

            if center_uv is not None:
                u, v = center_uv
                O_ray, D = get_ray(u, v, K, R, tvec)
                if abs(D[2]) > 1e-6:
                    t_val = (current_z - O_ray[2]) / D[2]
                    P_raw = O_ray + t_val * D
                    # ★ 外れ値フィルタ（フィールド外の誤検知を除外）
                    if is_valid_position(np.append(P_raw[:2], current_z)):
                        P_vec = P_raw - pos_offset
                        last_P_raw = P_raw.copy()
                        cv2.putText(frame,
                            f"X:{P_vec[0]:.2f} Y:{P_vec[1]:.2f} Z:{current_z:.2f}m",
                            (50,100), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0,255,255), 4)
                    else:
                        cv2.putText(frame, "OUT OF RANGE",
                            (50,100), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,0,255), 3)

            # 毎フレームログ
            if P_vec is not None:
                writer.writerow([current_time, 1,
                    round(P_vec[0],3), round(P_vec[1],3), round(current_z,3),
                    round(roll,2), round(pitch,2),
                    round(raw_alt,3), round(alt_offset,3)])
            else:
                writer.writerow([current_time, 0, "", "", round(current_z,3),
                    round(roll,2), round(pitch,2),
                    round(raw_alt,3), round(alt_offset,3)])
            log_fh.flush()

            # キャリブ要求の処理
            if shared.get("do_calib", False):
                shared["do_calib"] = False
                shared["alt_offset"] = raw_alt
                r0, p0 = accel_to_angles(raw_accel)
                shared["ref_roll"]  = r0
                shared["ref_pitch"] = p0
                if last_P_raw is not None:
                    # ★ 累積ではなく「現在の生座標をそのままオフセットにセット」
                    shared["pos_offset"] = last_P_raw.copy()
                    print(f"[Calib] Z={raw_alt:.2f}m  XY={last_P_raw[:2]}→原点  "
                          f"Roll={r0:.1f}°  Pitch={p0:.1f}°")
                else:
                    print(f"[Calib] Z={raw_alt:.2f}m (XY未検知)  "
                          f"Roll={r0:.1f}°  Pitch={p0:.1f}°")

            if shared.get("do_bg_reset", False):
                shared["do_bg_reset"] = False
                cam.prev_gray = None
                print("背景をリセットしました")

            # plot_dataへ渡す
            with plot_lock:
                plot_data["P"]         = P_vec
                plot_data["O"]         = O_fixed
                plot_data["roll"]      = roll
                plot_data["pitch"]     = pitch
                plot_data["current_z"] = current_z
                plot_data["updated"]   = True

            cv2.imshow("Camera", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                shared["quit"] = True
            elif key == ord('b'):
                shared["do_bg_reset"] = True
            elif key == ord(' '):
                shared["do_calib"] = True

    finally:
        log_fh.close()
        cam.release()
        cv2.destroyAllWindows()

# ==========================================
# Matplotlib描画（メインスレッド）
# ==========================================
def draw_plane_on_ax(ax_3d, pos, roll, pitch, scale=0.5):
    body = np.array([[-1,0,0],[1,0,0]]) * scale
    wing = np.array([[0,-1.2,0],[0,1.2,0]]) * scale
    tail = np.array([[-0.8,-0.4,0],[-0.8,0.4,0]]) * scale
    vert = np.array([[-0.8,0,0],[-0.8,0,0.4]]) * scale
    r_, p_ = np.radians(roll), np.radians(pitch)
    Rx = np.array([[1,0,0],[0,np.cos(r_),-np.sin(r_)],[0,np.sin(r_),np.cos(r_)]])
    Ry = np.array([[np.cos(p_),0,np.sin(p_)],[0,1,0],[-np.sin(p_),0,np.cos(p_)]])
    for part, color in [(body,'blue'),(wing,'blue'),(tail,'red'),(vert,'red')]:
        pts = ((Ry @ Rx) @ part.T).T + pos
        ax_3d.plot(pts[:,0], pts[:,1], pts[:,2], color=color, linewidth=3)

# ==========================================
# エントリポイント
# ==========================================
if __name__ == "__main__":
    print("システムを起動中...")

    cam        = CameraTracker(camera_id=0, width=1920, height=1080)
    alt_sensor = SerialReceiver(port="COM7", baudrate=115200)
    K          = cam.get_approx_camera_matrix()

    shared = {
        "points_2d":  [],
        "alt_offset": 0.0,
        "pos_offset": np.zeros(3),
        "ref_roll":   0.0,
        "ref_pitch":  0.0,
        "do_calib":   False,
        "do_bg_reset":False,
        "quit":       False,
    }

    # --- 基準点クリック（OpenCVウィンドウで行う） ---
    cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)
    def on_click_init(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(shared["points_2d"]) < 4:
            shared["points_2d"].append([x, y])
    cv2.setMouseCallback("Camera", on_click_init)
    print("【準備】基準点4箇所（左下→右下→右上→左上）をクリックしてください。")

    while True:
        ret, frame = cam.cap.read()
        if not ret:
            continue
        disp = frame.copy()
        for i, p in enumerate(shared["points_2d"]):
            cv2.circle(disp, tuple(p), 5, (0,0,255), -1)
            cv2.putText(disp, str(i+1), (p[0]+10,p[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
        cv2.imshow("Camera", disp)
        if cv2.waitKey(1) == 13 and len(shared["points_2d"]) == 4:
            break

    success, rvec, tvec = cv2.solvePnP(
        FIELD_POINTS, np.array(shared["points_2d"], dtype=np.float32), K, None)
    R, _ = cv2.Rodrigues(rvec)
    cv2.destroyAllWindows()

    print("【開始】トラッキング開始！")
    print("  Space: X/Y/Z + 傾き キャリブレーション")
    print("  b    : 背景リセット  /  q: 終了")

    # OpenCVスレッド起動
    cam_thread = threading.Thread(
        target=camera_thread_func,
        args=(cam, alt_sensor, K, R, tvec, "3d_tracking_log.csv", shared),
        daemon=True)
    cam_thread.start()

    # matplotlibはメインスレッドで
    plt.ion()
    fig = plt.figure(figsize=(9, 9))
    ax  = fig.add_subplot(111, projection='3d')
    plt.show(block=False)

    xs = np.append(FIELD_POINTS[:,0], FIELD_POINTS[0,0])
    ys = np.append(FIELD_POINTS[:,1], FIELD_POINTS[0,1])
    zs = np.append(FIELD_POINTS[:,2], FIELD_POINTS[0,2])

    while not shared.get("quit", False):
        with plot_lock:
            updated   = plot_data["updated"]
            P         = plot_data["P"]
            O         = plot_data["O"]
            roll      = plot_data["roll"]
            pitch     = plot_data["pitch"]
            current_z = plot_data["current_z"]
            if updated:
                plot_data["updated"] = False

        if updated and O is not None:
            ax.cla()
            ax.set_xlim(FIELD_X_MIN, FIELD_X_MAX)
            ax.set_ylim(FIELD_Y_MIN, FIELD_Y_MAX)
            ax.set_zlim(FIELD_Z_MIN, FIELD_Z_MAX)
            ax.set_box_aspect((FIELD_X_MAX-FIELD_X_MIN,
                               FIELD_Y_MAX-FIELD_Y_MIN,
                               FIELD_Z_MAX-FIELD_Z_MIN))
            ax.plot(xs, ys, zs, color='gray', linewidth=2, label="Field 16x16m")
            ax.scatter(*O, color='red', s=80, zorder=5, label="Camera")

            if P is not None:
                draw_plane_on_ax(ax, P, roll, pitch, scale=0.5)
                ax.plot([O[0],P[0]], [O[1],P[1]], [O[2],P[2]],
                        color='orange', linestyle='--', linewidth=1, alpha=0.6)
                ax.scatter(*P, color='green', s=80, zorder=5,
                           label=f"({P[0]:.1f},{P[1]:.1f},{current_z:.1f}m)")

            ax.set_xlabel('X(m)'); ax.set_ylabel('Y(m)'); ax.set_zlabel('Z(m)')
            ax.set_title(f"Roll:{roll:+.1f}°  Pitch:{pitch:+.1f}°  Alt:{current_z:.2f}m")
            ax.legend(fontsize=7, loc='upper right')

        try:
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
        except Exception:
            pass
        plt.pause(0.05)

    alt_sensor.stop()
    plt.close('all')