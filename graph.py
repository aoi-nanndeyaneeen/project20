import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.dates as mdates

# 1. データの読み込み（同じディレクトリにあるCSVファイルを指定）
file_path = '3d_tracking_log.csv'
df = pd.read_csv(file_path)

# 2. 'Time'列を日時型（datetime型）に変換
# 形式: 時:分:秒.ミリ秒
df['Time'] = pd.to_datetime(df['Time'], format='%H:%M:%S.%f')

# 3. グラフの描画領域を作成
# 左側に時間ごとの各座標のグラフ、右側に3D軌跡のグラフを配置します
fig = plt.figure(figsize=(15, 8))
fig.suptitle('3D Tracking Data Analysis', fontsize=16)

# --- 左側: 時間ごとのX, Y, Z座標の変化 ---
# サブプロットの配置 (行, 列, インデックス)
ax1 = fig.add_subplot(3, 2, 1)
ax1.plot(df['Time'], df['Target_X(m)'], color='tab:red', label='X(m)')
ax1.set_ylabel('X (m)')
ax1.grid(True, linestyle='--', alpha=0.7)
ax1.legend()

ax2 = fig.add_subplot(3, 2, 3, sharex=ax1)
ax2.plot(df['Time'], df['Target_Y(m)'], color='tab:green', label='Y(m)')
ax2.set_ylabel('Y (m)')
ax2.grid(True, linestyle='--', alpha=0.7)
ax2.legend()

ax3 = fig.add_subplot(3, 2, 5, sharex=ax1)
ax3.plot(df['Time'], df['Target_Z(m)'], color='tab:blue', label='Z(m)')
ax3.set_ylabel('Z (m)')
ax3.set_xlabel('Time (MM:SS)')
ax3.grid(True, linestyle='--', alpha=0.7)
ax3.legend()

# X軸の時刻フォーマットを「分:秒」に整える
ax3.xaxis.set_major_formatter(mdates.DateFormatter('%M:%S'))
plt.setp(ax1.get_xticklabels(), visible=False) # ax1のX軸ラベルを非表示
plt.setp(ax2.get_xticklabels(), visible=False) # ax2のX軸ラベルを非表示

# --- 右側: 3Dでの移動軌跡 ---
ax4 = fig.add_subplot(1, 2, 2, projection='3d')
ax4.plot(df['Target_X(m)'], df['Target_Y(m)'], df['Target_Z(m)'], color='purple', linewidth=2, label='Trajectory')
# 開始点と終了点を強調
ax4.scatter(df['Target_X(m)'].iloc[0], df['Target_Y(m)'].iloc[0], df['Target_Z(m)'].iloc[0], color='blue', s=50, label='Start')
ax4.scatter(df['Target_X(m)'].iloc[-1], df['Target_Y(m)'].iloc[-1], df['Target_Z(m)'].iloc[-1], color='red', s=50, label='End')

ax4.set_xlabel('X (m)')
ax4.set_ylabel('Y (m)')
ax4.set_zlabel('Z (m)')
ax4.set_title('3D Trajectory')
ax4.legend()

# 4. レイアウトを調整して表示
plt.tight_layout()
plt.subplots_adjust(top=0.92) # タイトルとの被りを防ぐ
plt.show()