import cv2
import numpy as np
import matplotlib.pyplot as plt
from camera import CameraTracker
from communication import SerialReceiver

# ==========================================
# 設定
# ==========================================
# テスト用の基準点 (A4用紙サイズを想定)
FIELD_POINTS = np.array([
    [0.0, 0.0, 0.0],
    [0.297, 0.0, 0.0],
    [0.297, 0.210, 0.0],
    [0.0, 0.210, 0.0]
], dtype=np.float32)

def get_ray(u, v, K, R, tvec):
    """ピクセル座標(u,v)から3D空間の視線ベクトル（レイ）を計算"""
    pt_2d = np.array([[[u, v]]], dtype=np.float32)
    undistorted = cv2.undistortPoints(pt_2d, K, None)
    dir_cam = np.array([undistorted[0][0][0], undistorted[0][0][1], 1.0])
    cam_pos = -R.T.dot(tvec).flatten()
    dir_world = R.T.dot(dir_cam)
    dir_world /= np.linalg.norm(dir_world)
    return cam_pos, dir_world.flatten()

# ==========================================
# メイン処理
# ==========================================
if __name__ == "__main__":
    print("システムを起動中...")

    # 1. カメラの初期化 (1080p指定)
    cam = CameraTracker(camera_id=0, width=1920, height=1080)
    
    # 2. 通信モジュールの初期化 (0.05m〜0.4mの間を動くダミーデータ)
    comm = SerialReceiver(
        port="COM3", 
        baudrate=115200, 
        mock_mode=True, 
        mock_min_z=0.05, 
        mock_max_z=0.40, 
        mock_speed=1.5
    )
    comm.start()

    K = cam.get_approx_camera_matrix()
    points_2d = []

    # ウィンドウを自由にリサイズできる設定
    cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)
    
    def click_calib(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(points_2d) < 4:
            points_2d.append([x, y])

    cv2.setMouseCallback("Camera", click_calib)
    print("【準備】基準点4箇所（左下→右下→右上→左上）をクリックしてください。")

    while True:
        ret, frame = cam.cap.read()
        if not ret: 
            continue
        
        display = frame.copy()
        for i, p in enumerate(points_2d):
            cv2.circle(display, tuple(p), 5, (0,0,255), -1)
            cv2.putText(display, str(i+1), (p[0]+10, p[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
        
        cv2.imshow("Camera", display)
        
        if cv2.waitKey(1) == 13 and len(points_2d) == 4: # Enterキーで確定
            break

    success, rvec, tvec = cv2.solvePnP(FIELD_POINTS, np.array(points_2d, dtype=np.float32), K, None)
    R, _ = cv2.Rodrigues(rvec)

    plt.ion()
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')

    print("【開始】トラッキングを開始します！ (終了: qキー, 背景リセット: bキー)")

    while True:
        frame, center_uv = cam.read_and_track()
        if frame is None: 
            break
        
        current_z = comm.get_z() 

        if center_uv is not None:
            u, v = center_uv
            O, D = get_ray(u, v, K, R, tvec)
            
            if D[2] != 0:
                t = (current_z - O[2]) / D[2]
                P = O + t * D
                
                text = f"X:{P[0]:.2f}, Y:{P[1]:.2f}, Z:{current_z:.2f}m"
                # 解像度が上がったので、文字サイズ(2.0)と太さ(4)を大きくしました
                cv2.putText(frame, text, (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 255), 4)

                ax.cla()
                # Z軸の高さが低いため、スケールを小さく(約0.5m)固定して見やすくする
                max_range = max(0.5, current_z + 0.2)
                ax.set_box_aspect((1, 1, 1))
                ax.set_xlim(-max_range/2, max_range/2)
                ax.set_ylim(-max_range/2, max_range/2)
                ax.set_zlim(0, max_range)

                xs = np.append(FIELD_POINTS[:,0], FIELD_POINTS[0,0])
                ys = np.append(FIELD_POINTS[:,1], FIELD_POINTS[0,1])
                zs = np.append(FIELD_POINTS[:,2], FIELD_POINTS[0,2])
                ax.plot(xs, ys, zs, color='blue', label="Ground")
                
                ax.scatter(*O, color='red', s=100, label="Camera")
                ax.plot([O[0], P[0]], [O[1], P[1]], [O[2], P[2]], color='orange', linestyle='--')
                ax.scatter(*P, color='green', s=100, label=f"Airplane (Z={current_z:.2f}m)")
                
                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('Z')
                ax.legend()
                
                plt.draw()
                plt.pause(0.001)

        cv2.imshow("Camera", frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('b'):
            cam.prev_gray = None 
            print("背景をリセットしました")

    cam.release()
    comm.stop()
    cv2.destroyAllWindows()