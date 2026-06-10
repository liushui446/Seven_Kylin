import matplotlib
# 银河麒麟V10 自带官方中文字体（100%可用）
matplotlib.rcParams['font.sans-serif'] = ['Noto Sans CJK SC']
matplotlib.rcParams['axes.unicode_minus'] = False


from matplotlib import patches
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from datetime import datetime
import time
import json
import sys
import numpy as np
import threading
import queue
import os
import socket  # 替换win32pipe/win32file的核心模块
import errno   # 处理Unix Socket异常


# 中文字体设置
# plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei']
# plt.rcParams['axes.unicode_minus'] = False

# 全局变量
is_paused = False
current_frame = 0
total_frames = 0
is_listening = True
result_queue = queue.Queue()
# 替换：Unix Socket对象（替代原Windows管道句柄）
sock_send = None  # 发送命令用：对应/tmp/ClientToServerPipe
sock_listen = None  # 监听结果用：对应/tmp/ServerToClientPipe

# ========================= 可视化相关全局变量 =========================
# 编队数据缓存：{ formation_id: { frame_id: { "nodes": [...], "formation_type": N } } }
formation_data = {}
# 存储初始输入命令数据（编队配置信息）
cmd_cache = {
    "formations": [],
    "cmd_info": {}
}
# 实时动画相关
ani = None
fig = None
viz_lock = threading.Lock()
# 编队颜色映射
FORMATION_COLORS = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728',
                    '#9467bd', '#8c564b', '#e377c2', '#7f7f7f']

# ========================= 中心化可视化配置 =========================
CENTER_FORMATION_ID = "1"       # 锁定为画面中心的编队ID（可修改）
VIEW_RANGE_METERS = 150.0       # 可视化窗口范围：±150米
DEG_TO_M = 111319.9             # 经纬度转米系数

# ========================= 【新增】帧播放控制全局变量 =========================
playback_speed = 1.0            # 播放速度（1.0=正常速度，2.0=2倍速，0.5=0.5倍速）
is_playing = False              # 是否正在播放
playback_thread = None          # 播放线程
current_playback_frame = 0      # 当前播放到的帧号
max_cached_frame = 0            # 已缓存的最大帧号

# 命令文件映射
CMD_FILE_MAP = {
    "1": "cmd_json_1.json",
    "2": "cmd_json_2.json",
    "3": "cmd_json_3.json",
    "4": "cmd_json_4.json",
    "5": "cmd_json_1_for.json",
    "6": "cmd_json_test.json",
    "7": "cmd_json_power_test.json",
    "8": "cmd_json_freq_test.json",
    "9": "cmd_json_2_format.json",
    "10": "cmd_json_addnode.json",
    "11": "cmd_formation_start.json",
    "12": "cmd_formation_cal1.json",
    "13": "cmd_formation_cal2.json",
    "14": "cmd_formation_pause.json",
    "15": "cmd_formation_end.json"
}

# 替换：Unix Socket路径（严格对应C++端的路径）
SOCKET_PATH_CLIENT_TO_SERVER = "/tmp/ClientToServerPipe"  # 发送命令
SOCKET_PATH_SERVER_TO_CLIENT = "/tmp/ServerToClientPipe"  # 监听结果

# 编队类型名称映射
FORMATION_TYPE_NAMES = {
    1: "楔形", 2: "纵队", 3: "横队",
    4: "菱形", 5: "三角形", 6: "圆形"
}

# ========================= 工具函数 =========================
def geo_to_relative_meters(lon: float, lat: float, center_lon: float, center_lat: float) -> tuple:
    """GPS经纬度 → 以中心编队为原点的东北坐标系（米）"""
    lon_scale = DEG_TO_M * np.cos(np.radians(center_lat))
    east = (lon - center_lon) * lon_scale
    north = (lat - center_lat) * DEG_TO_M
    return east, north


def _get_formation_offsets(formation_type, num_nodes, interval=10.0):
    """根据编队类型计算各节点相对于编队中心的理想偏移"""
    offsets = []
    if formation_type == 2:  # 纵队
        for i in range(num_nodes):
            offsets.append((0.0, -i * interval))
    elif formation_type == 3:  # 横队
        half = (num_nodes - 1) / 2.0
        for i in range(num_nodes):
            offsets.append(((i - half) * interval, 0.0))
    elif formation_type == 4:  # 菱形
        offsets.append((0.0, 0.0))
        row = 1
        while len(offsets) < num_nodes:
            for j in range(-row, row + 1):
                if len(offsets) < num_nodes:
                    offsets.append((j * interval, -row * interval))
            row += 1
    elif formation_type == 1 or formation_type == 5:  # 楔形/三角形
        row = 0
        while len(offsets) < num_nodes:
            for j in range(-row, row + 1):
                if len(offsets) < num_nodes:
                    offsets.append((j * interval, -row * interval))
            row += 1
    else:  # 默认：纵队
        for i in range(num_nodes):
            offsets.append((0.0, -i * interval))
    return offsets[:num_nodes]


def visualize_formation_static(formation_id=None):
    """静态轨迹总览 — 弹出独立窗口，绘制所有UAV完整轨迹"""
    global formation_data
    if not formation_data:
        print("没有可用的编队数据！请先发送编队命令并等待监听数据。")
        return

    if formation_id is None:
        formation_id = list(formation_data.keys())[0]
        print(f"未指定编队ID，使用第一个：编队 {formation_id}")

    fid_str = str(formation_id)
    if fid_str not in formation_data:
        print(f"编队 {formation_id} 不存在！可用ID：{list(formation_data.keys())}")
        return

    frames = formation_data[fid_str]
    if not frames:
        print(f"编队 {formation_id} 无数据！")
        return

    sorted_frames = sorted(frames.items(), key=lambda x: x[0])
    frame_ids = [f[0] for f in sorted_frames]

    # 提取所有节点的轨迹
    node_tracks = {}  # {node_id: [(lon, lat), ...]}
    for fid, fdata in sorted_frames:
        for node in fdata.get("nodes", []):
            nid = node.get("node_id", 0)
            if nid not in node_tracks:
                node_tracks[nid] = []
            node_tracks[nid].append((node.get("lon", 0), node.get("lat", 0)))

    # 获取编队配置
    fm_cfg = _get_formation_config(fid_str)

    fig = plt.figure(figsize=(16, 10))
    fig.suptitle(f'编队 {formation_id} — 编队变换仿真结果'
                 f'（类型: {FORMATION_TYPE_NAMES.get(fm_cfg.get("formation_type", "?"), "?")}, '
                 f'{fm_cfg.get("num_uavs", "?")}架UAV）', fontsize=14)

    # ——— 左图：2D轨迹总览 ———
    ax1 = plt.subplot(2, 3, (1, 3))
    color_idx = 0
    for nid in sorted(node_tracks.keys()):
        track = node_tracks[nid]
        lons = [p[0] for p in track]
        lats = [p[1] for p in track]
        color = FORMATION_COLORS[color_idx % len(FORMATION_COLORS)]
        ax1.plot(lons, lats, '-', color=color, linewidth=1.5, alpha=0.8,
                 label=f'UAV {nid}')
        # 起点和终点标记
        if len(track) > 1:
            ax1.scatter(lons[0], lats[0], c=color, marker='o', s=40)
            ax1.scatter(lons[-1], lats[-1], c=color, marker='s', s=40)
        elif len(track) == 1:
            ax1.scatter(lons[0], lats[0], c=color, marker='o', s=40)
        color_idx += 1

    # 标记编队中心初始位置
    if fm_cfg:
        ax1.scatter(fm_cfg.get("pos_lon", 0), fm_cfg.get("pos_lat", 0),
                    c='red', marker='*', s=200, zorder=5, label='编队中心')

    ax1.set_xlabel('经度(°)'); ax1.set_ylabel('纬度(°)')
    ax1.set_title(f'UAV轨迹总览（{len(frame_ids)}帧）')
    ax1.legend(loc='upper right', fontsize=7)
    ax1.grid(True)
    ax1.set_aspect('equal')

    # ——— 右上：最后一帧编队快照 ———
    ax2 = plt.subplot(2, 3, 4)
    last_frame = sorted_frames[-1][1]
    _draw_formation_snapshot(ax2, last_frame, fm_cfg,
                             title=f'最终帧 #{sorted_frames[-1][0]} 编队快照')

    # ——— 右中：编队误差柱状图 ———
    ax3 = plt.subplot(2, 3, 5)
    errors = []
    nids_list = []
    for node in last_frame.get("nodes", []):
        nids_list.append(node.get("node_id", 0))
        errors.append(node.get("formation_error", 0))
    colors = [FORMATION_COLORS[i % len(FORMATION_COLORS)]
              for i in range(len(nids_list))]
    ax3.bar(nids_list, errors, color=colors)
    ax3.set_xlabel('节点ID'); ax3.set_ylabel('编队误差(m)')
    ax3.set_title('最终帧各节点编队保持误差')
    ax3.grid(True, axis='y')

    # ——— 右下：帧数统计 ———
    ax4 = plt.subplot(2, 3, 6)
    ax4.axis('off')
    info_lines = [
        f"编队ID: {formation_id}",
        f"编队类型: {FORMATION_TYPE_NAMES.get(fm_cfg.get('formation_type', '?'), '?')}",
        f"UAV数量: {fm_cfg.get('num_uavs', '?')}",
        f"总帧数: {len(frame_ids)}",
        f"帧范围: #{min(frame_ids)} - #{max(frame_ids)}",
        f"初始航向: {fm_cfg.get('init_heading', '?')}°",
        f"初始速度: {fm_cfg.get('init_speed', '?')} m/s",
        f"节点间距: {fm_cfg.get('interval', '?')} m",
        "",
        f"最终帧编队误差:",
    ]
    for node in last_frame.get("nodes", []):
        info_lines.append(
            f"  UAV {node.get('node_id', 0)}: "
            f"误差={node.get('formation_error', 0):.2f}m, "
            f"航向={node.get('heading', 0):.1f}°")
    info_text = "\n".join(info_lines)
    ax4.text(0.05, 0.95, info_text, transform=ax4.transAxes,
             fontsize=9, verticalalignment='top', fontfamily='monospace')

    plt.tight_layout()
    plt.show()


def _draw_formation_snapshot(ax, frame_data, fm_cfg, title="编队快照"):
    """在指定axes上绘制单个帧的编队快照"""
    nodes = frame_data.get("nodes", [])
    if not nodes:
        return

    lons = [n.get("lon", 0) for n in nodes]
    lats = [n.get("lat", 0) for n in nodes]
    nids = [n.get("node_id", 0) for n in nodes]

    # 绘制理想编队位置（target）
    ft = fm_cfg.get("formation_type",
                    frame_data.get("formation_type", 2))
    interval = fm_cfg.get("interval", 10.0)
    ideal_offsets = _get_formation_offsets(ft, len(nodes), interval)
    # 计算理想位置需基于编队中心
    center_lon = fm_cfg.get("pos_lon", np.mean(lons))
    center_lat = fm_cfg.get("pos_lat", np.mean(lats))
    # 近似转换：1°≈111km, 用interval的经纬度近似
    deg_per_m = 1.0 / 111000.0

    ideal_lons = []
    ideal_lats = []
    for dx, dy in ideal_offsets:
        ideal_lons.append(center_lon + dx * deg_per_m)
        ideal_lats.append(center_lat + dy * deg_per_m)

    # 绘制理想编队（空心标记+虚线连接）
    ax.scatter(ideal_lons, ideal_lats, c='gray', marker='o', s=60,
               facecolors='none', linewidths=1, label='理想位置', zorder=2)
    for i in range(len(ideal_lons)):
        for j in range(i + 1, len(ideal_lons)):
            ax.plot([ideal_lons[i], ideal_lons[j]],
                    [ideal_lats[i], ideal_lats[j]],
                    'gray', linewidth=0.5, linestyle='--', alpha=0.4, zorder=1)

    # 绘制实际位置（实心标记+实线连接）
    for i, (lon, lat, nid) in enumerate(zip(lons, lats, nids)):
        color = FORMATION_COLORS[nid % len(FORMATION_COLORS)]
        ax.scatter(lon, lat, c=color, marker='o', s=80, zorder=4)
        ax.annotate(f"{nid}", (lon, lat), textcoords="offset points",
                    xytext=(4, 4), fontsize=7, zorder=5)
        # 误差连线（从理想到实际）
        if i < len(ideal_lons):
            ax.plot([ideal_lons[i], lon], [ideal_lats[i], lat],
                    'r-', linewidth=0.5, alpha=0.5, zorder=3)

    # 实际节点之间的连接线
    for i in range(len(lons)):
        for j in range(i + 1, len(lons)):
            ax.plot([lons[i], lons[j]], [lats[i], lats[j]],
                    'b-', linewidth=0.3, alpha=0.3, zorder=2)

    ax.set_xlabel('经度(°)'); ax.set_ylabel('纬度(°)')
    ax.set_title(title)
    ax.grid(True)
    ax.set_aspect('equal')


def _get_formation_config(fid_str):
    """从cmd_cache获取特定编队的配置"""
    for fm in cmd_cache.get("formations", []):
        if str(fm.get("formation_id", "")) == fid_str:
            return fm
    return {}


def visualize_formation_config():
    """可视化编队初始配置 — 显示各编队的预设位置和理想编队形状"""
    formations = cmd_cache.get("formations", [])
    if not formations:
        print("没有缓存的编队配置数据！请先发送编队命令（如: 11）。")
        return

    n_fms = len(formations)
    cols = min(3, n_fms)
    rows = (n_fms + cols - 1) // cols

    fig, axes = plt.subplots(rows, cols, figsize=(6 * cols, 6 * rows))
    fig.suptitle('编队初始配置 — 理想编队布局', fontsize=14)

    if n_fms == 1:
        axes = [axes]
    elif rows == 1:
        axes = axes.flatten() if hasattr(axes, 'flatten') else [axes]
    else:
        axes = axes.flatten()

    for idx, fm in enumerate(formations):
        ax = axes[idx]
        fid = fm.get("formation_id", "?")
        ftype = fm.get("formation_type", "?")
        num_uavs = fm.get("num_uavs", 0)
        interval = fm.get("interval", 10.0)
        center_lon = fm.get("pos_lon", 0)
        center_lat = fm.get("pos_lat", 0)
        heading = fm.get("init_heading", 0)

        offsets = _get_formation_offsets(ftype, num_uavs, interval)
        deg_per_m = 1.0 / 111000.0

        lons, lats, labels = [], [], []
        for i, (dx, dy) in enumerate(offsets):
            lon = center_lon + dx * deg_per_m
            lat = center_lat + dy * deg_per_m
            lons.append(lon)
            lats.append(lat)
            labels.append(str(i))

        # 绘制节点
        for i in range(len(lons)):
            color = FORMATION_COLORS[i % len(FORMATION_COLORS)]
            ax.scatter(lons[i], lats[i], c=color, marker='o', s=100, zorder=3)
            ax.annotate(labels[i], (lons[i], lats[i]),
                        textcoords="offset points", xytext=(5, 5), fontsize=8)
        # 节点间连线
        for i in range(len(lons)):
            for j in range(i + 1, len(lons)):
                ax.plot([lons[i], lons[j]], [lats[i], lats[j]],
                        'gray', linewidth=0.5, alpha=0.5, zorder=1)

        # 编队中心
        ax.scatter(center_lon, center_lat, c='red', marker='*', s=200,
                   zorder=5, label='编队中心')

        # 航向箭头
        heading_rad = np.radians(90 - heading)  # 从正北顺时针
        arrow_len = interval * num_uavs * 0.3 * deg_per_m
        ax.arrow(center_lon, center_lat,
                 arrow_len * np.cos(heading_rad),
                 arrow_len * np.sin(heading_rad),
                 head_width=interval * 0.05 * deg_per_m,
                 head_length=interval * 0.1 * deg_per_m,
                 fc='red', ec='red', alpha=0.7, zorder=4)

        ax.set_xlabel('经度(°)'); ax.set_ylabel('纬度(°)')
        ax.set_title(f'编队{fid}: {FORMATION_TYPE_NAMES.get(ftype, "?")} '
                     f'({num_uavs}UAV, 航向{heading}°)')
        ax.grid(True)
        ax.set_aspect('equal')

    # 隐藏多余的子图
    for idx in range(n_fms, len(axes)):
        axes[idx].set_visible(False)

    # 打印配置信息
    print("\n===== 编队初始配置 =====")
    cmd_info = cmd_cache.get("cmd_info", {})
    print(f"命令类型: cmd={cmd_info.get('cmd', '?')}, sim_type={cmd_info.get('sim_type', '?')}")
    for fm in formations:
        print(f"\n编队 {fm.get('formation_id', '?')}:")
        print(f"  类型: {FORMATION_TYPE_NAMES.get(fm.get('formation_type', '?'), '?')}")
        print(f"  UAV数: {fm.get('num_uavs', '?')}")
        print(f"  中心: ({fm.get('pos_lon', 0):.6f}°, {fm.get('pos_lat', 0):.6f}°)")
        print(f"  航向: {fm.get('init_heading', '?')}°  速度: {fm.get('init_speed', '?')} m/s")
        print(f"  间距: {fm.get('interval', '?')}m  碰撞半径: {fm.get('collision_radius', '?')}m")
    print("========================\n")

    plt.tight_layout()
    plt.show()

# ========================= 【核心修改】带逐帧播放的中心化可视化 =========================
def launch_formation_viewer(refresh_ms=200):
    """
    中心化实时监控 + 逐帧播放功能：
    1. 锁定指定编队为画面中心(0,0)
    2. 坐标单位：米（东/北方向）
    3. 支持1、2、3、4、5帧逐帧播放
    4. 支持播放/暂停/调速/跳帧
    """
    global ani, fig, formation_data, current_playback_frame, max_cached_frame

    plt.ion()
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # 固定米单位坐标轴
    ax.set_xlabel('东 (m)')
    ax.set_ylabel('北 (m)')
    ax.set_title(f'锁定：编队{CENTER_FORMATION_ID} | 帧#0 | 已缓存：0帧')
    ax.grid(True)
    ax.set_aspect('equal')
    ax.set_xlim(-VIEW_RANGE_METERS, VIEW_RANGE_METERS)
    ax.set_ylim(-VIEW_RANGE_METERS, VIEW_RANGE_METERS)

    # 轨迹拖尾缓存
    track_history = {}

    def refresh():
        """定时刷新函数：根据当前播放帧号绘制画面"""
        with viz_lock:
            if not formation_data:
                return

            # 清空画布，重置坐标轴
            ax.clear()
            ax.set_xlabel('东 (m)')
            ax.set_ylabel('北 (m)')
            ax.set_xlim(-VIEW_RANGE_METERS, VIEW_RANGE_METERS)
            ax.set_ylim(-VIEW_RANGE_METERS, VIEW_RANGE_METERS)
            ax.grid(True)
            ax.set_aspect('equal')

            # ============== 第一步：获取中心编队坐标（原点）==============
            center_lon, center_lat = 0.0, 0.0
            center_exists = False
            
            # 检查当前播放帧是否存在
            if CENTER_FORMATION_ID in formation_data:
                center_frames = formation_data[CENTER_FORMATION_ID]
                if current_playback_frame in center_frames:
                    center_frame = center_frames[current_playback_frame]
                    center_nodes = center_frame.get("nodes", [])
                    if center_nodes:
                        # 以中心编队的0号节点为坐标原点
                        leader = center_nodes[0]
                        center_lon = leader.get("lon", 0.0)
                        center_lat = leader.get("lat", 0.0)
                        center_exists = True

            if not center_exists:
                ax.set_title(f"等待中心编队{CENTER_FORMATION_ID}数据... | 已缓存：{max_cached_frame}帧")
                return

            # ============== 第二步：绘制所有编队（当前播放帧）==============
            fids = sorted(formation_data.keys(), key=lambda x: int(x) if x.isdigit() else 0)

            for fi, fid_str in enumerate(fids):
                frames = formation_data[fid_str]
                if not frames or current_playback_frame not in frames:
                    continue

                # 获取当前播放帧的数据
                current_frame_data = frames[current_playback_frame]
                nodes = current_frame_data.get("nodes", [])
                fm_color = FORMATION_COLORS[fi % len(FORMATION_COLORS)]

                if fid_str not in track_history:
                    track_history[fid_str] = {}

                east_list, north_list = [], []
                # 遍历节点，转换坐标并绘制
                for node in nodes:
                    nid = node.get("node_id", 0)
                    lon = node.get("lon", 0.0)
                    lat = node.get("lat", 0.0)

                    # 经纬度转相对米坐标
                    east, north = geo_to_relative_meters(lon, lat, center_lon, center_lat)
                    east_list.append(east)
                    north_list.append(north)

                    # 保存轨迹（用于拖尾）
                    if nid not in track_history[fid_str]:
                        track_history[fid_str][nid] = []
                    track_history[fid_str][nid].append((east, north))

                    # 绘制节点
                    ax.scatter(east, north, c=fm_color, s=80, zorder=3, edgecolors='black')
                    ax.annotate(f"{nid}", (east, north), xytext=(4, 4), fontsize=7, zorder=4)

                # 绘制编队内部连线（显示队形）
                for i in range(len(east_list)):
                    for j in range(i + 1, len(east_list)):
                        ax.plot([east_list[i], east_list[j]], [north_list[i], north_list[j]],
                                color=fm_color, linewidth=0.4, alpha=0.35)

                # 标注编队ID（中心编队特殊标记）
                if len(east_list) > 0:
                    cx = np.mean(east_list)
                    cy = np.mean(north_list)
                    label = f"编队{fid_str}(中心)" if fid_str == CENTER_FORMATION_ID else f"编队{fid_str}"
                    ax.annotate(label, (cx, cy), fontsize=9, fontweight='bold', color=fm_color,
                                ha='center', va='center',
                                bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.85))

                # 绘制轨迹拖尾（最近50帧）
                for nid, track in track_history[fid_str].items():
                    if len(track) > 1:
                        trail = track[-50:]
                        es = [p[0] for p in trail]
                        ns = [p[1] for p in trail]
                        ax.plot(es, ns, '-', color=fm_color, linewidth=0.6, alpha=0.3)

            # 更新标题（显示当前帧号和已缓存帧数）
            play_status = "▶ 播放中" if is_playing else "⏸ 已暂停"
            ax.set_title(f'{play_status} | 锁定：编队{CENTER_FORMATION_ID} | 帧#{current_playback_frame} | 已缓存：{max_cached_frame}帧 | 速度：{playback_speed:.1f}x')
            fig.canvas.draw_idle()

    # 启动定时器：定时刷新画面
    timer = fig.canvas.new_timer(interval=refresh_ms)
    timer.add_callback(refresh)
    timer.start()
    plt.show(block=False)
    print(f"[可视化] 中心化监控启动 | 中心：编队{CENTER_FORMATION_ID} | 视窗：±{VIEW_RANGE_METERS}米")
    print("[播放控制] 输入 'play' 开始播放 | 'pause' 暂停 | 'speed x' 调整速度 | 'goto x' 跳转到指定帧")
    return timer

# ========================= 【新增】帧播放控制线程 =========================
def playback_worker():
    """独立的播放线程：控制帧的播放速度"""
    global current_playback_frame, max_cached_frame, is_playing
    
    while is_listening:
        if is_playing and current_playback_frame < max_cached_frame:
            # 播放下一帧
            current_playback_frame += 1
            # 根据播放速度调整等待时间
            time.sleep(0.05 / playback_speed)  # 基础帧率20fps
        else:
            # 暂停或已播放完所有缓存帧
            time.sleep(0.01)

# ========================= 【新增】更新最大缓存帧号 =========================
def update_max_cached_frame():
    """更新已缓存的最大帧号"""
    global max_cached_frame
    with viz_lock:
        for fid_str in formation_data:
            frames = formation_data[fid_str]
            if frames:
                current_max = max(frames.keys())
                if current_max > max_cached_frame:
                    max_cached_frame = current_max

# ========================= 【核心修改】Unix Socket 通信相关函数 =========================
def create_unix_socket(socket_path: str) -> socket.socket or None:
    """创建并连接Unix Domain Socket客户端"""
    try:
        # 创建AF_UNIX类型的Socket
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        # 连接到服务端Socket路径
        sock.connect(socket_path)
        # 设置非阻塞（可选，保持和原逻辑一致）
        # sock.setblocking(True)  # 同步模式，和原Windows管道一致
        print(f"[Socket] 成功连接到 {socket_path}")
        return sock
    except socket.error as e:
        if e.errno == errno.ENOENT:
            print(f"[Socket] 连接失败：{socket_path} 不存在（服务端未启动？）")
        elif e.errno == errno.ECONNREFUSED:
            print(f"[Socket] 连接失败：{socket_path} 被拒绝（服务端未监听？）")
        else:
            print(f"[Socket] 连接异常：{e}")
        return None

def reconnect_send_pipe() -> bool:
    """重连发送命令的Socket（ClientToServerPipe）"""
    global sock_send
    try:
        if sock_send is not None:
            sock_send.close()
    except:
        pass
    sock_send = create_unix_socket(SOCKET_PATH_CLIENT_TO_SERVER)
    return sock_send is not None

def reconnect_listen_pipe() -> bool:
    """重连监听结果的Socket（ServerToClientPipe）"""
    global sock_listen
    try:
        if sock_listen is not None:
            sock_listen.close()
    except:
        pass
    sock_listen = create_unix_socket(SOCKET_PATH_SERVER_TO_CLIENT)
    return sock_listen is not None

def update_formation_cache(cmd: dict):
    """从编队命令中提取并缓存编队配置数据"""
    global cmd_cache
    with viz_lock:
        if "formations" in cmd:
            cmd_cache["formations"] = cmd["formations"]
            print(f"[可视化] 已缓存 {len(cmd['formations'])} 个编队的初始配置")
            for fm in cmd["formations"]:
                fid = fm.get("formation_id", "?")
                ftype = fm.get("formation_type", "?")
                num = fm.get("num_uavs", "?")
                print(f"  编队{fid}: {FORMATION_TYPE_NAMES.get(ftype, '?')} "
                      f"({num}UAV, 航向{fm.get('init_heading', '?')}°)")

        cmd_cache["cmd_info"] = {
            "sim_type": cmd.get("sim_type", "未知"),
            "cmd": cmd.get("cmd", "未知"),
        }

def send_pipe_command(cmd: dict) -> bool:
    """替换为Unix Socket发送：同步发送命令 + 同步读取结果"""
    global sock_send
    if sock_send is None:
        print("[发送] 发送Socket未初始化，尝试重连...")
        if not reconnect_send_pipe():
            return False

    try:
        # 1. 构造命令并转为字节串（保持原分隔符）
        cmd_str = json.dumps(cmd, ensure_ascii=False) + "\n###END###\n"
        cmd_bytes = cmd_str.encode("utf-8")
        
        # 2. 同步发送数据
        bytes_sent = sock_send.send(cmd_bytes)
        print(f"[发送] 发送字节数：{len(cmd_bytes)}，实际发送：{bytes_sent}")

        # 3. 同步读取返回结果（适配大缓冲区）
        buffer_size = 4095 * 10000
        result_bytes = b""
        # 循环读取直到找到分隔符
        while True:
            chunk = sock_send.recv(buffer_size)
            if not chunk:
                print("[发送] 连接断开，读取结果失败")
                return False
            result_bytes += chunk
            # 检查是否包含分隔符
            if b"\n###END###\n" in result_bytes:
                break

        bytes_read = len(result_bytes)
        print(f"[调试] 接收到的原始数据大小：{bytes_read}")
        
        # 校验读取结果
        if bytes_read == 0:
            print("[发送] 读取结果失败：无数据")
            return False

        # 4. 解析结果（保持原容错逻辑）
        result_str = result_bytes.decode("utf-8", errors="ignore")
        
        # 验证分隔符
        delimiter = "\n###END###\n"
        delimiter_pos = result_str.find(delimiter)
        
        if delimiter_pos == -1:
            print(f"[警告] 未找到 JSON 数据分隔符 {repr(delimiter)}")
            # 尝试直接解析整个字符串作为 JSON
            try:
                result_json = json.loads(result_str)
                print(f"[成功] 直接解析成功")
            except json.JSONDecodeError as e:
                print(f"[错误] JSON 解析失败：{e}")
                return False
        else:
            # 提取分隔符之前的 JSON 数据
            json_str = result_str[:delimiter_pos]
            try:
                result_json = json.loads(json_str)
            except json.JSONDecodeError as e:
                print(f"[错误] JSON 解析失败：{e}")
                return False
        
        # 将 JSON 结果输出到文件
        with open("message.json", "w", encoding="utf-8") as f:
            json.dump(result_json, f, ensure_ascii=False, indent=2)
        print("结果已保存到 message.json")

        return True

    except socket.error as e:
        print(f"[发送] Socket异常：{e}")
        # 重连后重试一次
        if reconnect_send_pipe():
            return send_pipe_command(cmd)
        return False
    except Exception as e:
        print(f"[发送] 未知异常：{e}")
        return False

def listen_pipe_continually():
    """替换为Unix Socket监听：解析编队仿真数据并缓存到 formation_data"""
    global is_listening, sock_listen, formation_data, max_cached_frame
    if sock_listen is None:
        print("[监听] 监听Socket未初始化，尝试重连...")
        if not reconnect_listen_pipe():
            return

    cnt_result = 1
    first_data_logged = False
    print(f"[监听] 启动稳定监听（编队可视化模式）")

    while is_listening:
        try:
            buffer_size = 4095 * 10000
            result_bytes = b""
            # 循环读取直到找到分隔符
            while True:
                chunk = sock_listen.recv(buffer_size)
                if not chunk:
                    print("[监听] 连接断开，尝试重连...")
                    if not reconnect_listen_pipe():
                        time.sleep(1)
                        continue
                    break
                result_bytes += chunk
                if b"\n###END###\n" in result_bytes:
                    break

            if not result_bytes:
                continue

            bytes_read = len(result_bytes)
            result_str = result_bytes.decode("utf-8", errors="ignore")
            delimiter = "\n###END###\n"
            delimiter_pos = result_str.find(delimiter)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M")
            filename = f"result_{timestamp}_+ {cnt_result}.json"
            cnt_result += 1

            json_str = result_str[:delimiter_pos] if delimiter_pos != -1 else result_str

            # 解析JSON
            result_json = None
            try:
                result_json = json.loads(json_str)
                with open(filename, "w", encoding="utf-8") as f:
                    json.dump(result_json, f, ensure_ascii=False, indent=2)
            except json.JSONDecodeError as e:
                print(f"[监听] JSON解析失败: {e}")
                print(f"[监听] 原始数据前200字符: {json_str[:200]}")
                continue
            except IOError as e:
                print(f"[监听] 文件写入失败: {e}")

            # ===== 首次收到数据：打印结构信息 =====
            if result_json is not None and not first_data_logged:
                first_data_logged = True
                if isinstance(result_json, dict):
                    keys = list(result_json.keys())
                    print(f"[监听] 首次数据：字典，字段={keys}")
                    if "formations" in result_json:
                        fms = result_json["formations"]
                        if isinstance(fms, dict):
                            for fid, frames in fms.items():
                                if isinstance(frames, list) and len(frames) > 0:
                                    f0 = frames[0]
                                    print(f"[监听]  编队{fid}: {len(frames)}帧, "
                                          f"首帧字段={list(f0.keys())}")
                                    nodes = f0.get("nodes", [])
                                    if nodes:
                                        print(f"[监听]  编队{fid} 首帧 {len(nodes)}个节点, "
                                              f"节点字段={list(nodes[0].keys())}")
                elif isinstance(result_json, list):
                    print(f"[监听] 首次数据：列表，长度={len(result_json)}")
                else:
                    print(f"[监听] 首次数据：类型={type(result_json).__name__}")

            # ===== 缓存编队轨迹数据 =====
            if result_json is not None:
                try:
                    with viz_lock:
                        # 编队格式：{"formations": {"1": [...], "2": [...]}}
                        if isinstance(result_json, dict) and "formations" in result_json:
                            fms = result_json["formations"]
                            if isinstance(fms, dict):
                                for fid_str, frames in fms.items():
                                    if not isinstance(frames, list):
                                        continue
                                    if fid_str not in formation_data:
                                        formation_data[fid_str] = {}
                                    cached_count = 0
                                    for frame in frames:
                                        if not isinstance(frame, dict):
                                            continue
                                        frame_id = frame.get("frame_id", 0)
                                        frame_id = int(frame_id) if isinstance(frame_id, (int, float, str)) else 0
                                        formation_data[fid_str][frame_id] = frame
                                        cached_count += 1
                                    if cached_count > 0:
                                        all_fids = sorted(formation_data[fid_str].keys())
                                        print(f"[监听] ✓ 编队{fid_str}: +{cached_count}帧 "
                                              f"(累计{len(formation_data[fid_str])}帧, "
                                              f"范围#{min(all_fids)}-#{max(all_fids)})")
                                        # 更新最大缓存帧号
                                        if max(all_fids) > max_cached_frame:
                                            max_cached_frame = max(all_fids)

                        # 兼容：列表格式
                        elif isinstance(result_json, list):
                            for item in result_json:
                                if not isinstance(item, dict):
                                    continue
                                # 如果有formations字段
                                if "formations" in item:
                                    fms = item["formations"]
                                    if isinstance(fms, dict):
                                        for fid_str, frames in fms.items():
                                            if fid_str not in formation_data:
                                                formation_data[fid_str] = {}
                                            for frame in frames:
                                                if isinstance(frame, dict):
                                                    fid_val = frame.get("frame_id", 0)
                                                    fid_val = int(fid_val) if isinstance(fid_val, (int, float, str)) else 0
                                                    formation_data[fid_str][fid_val] = frame
                                    continue
                                # 含frame_id的单个字典
                                fid_val = item.get("frame_id")
                                if fid_val is not None:
                                    fid_val = int(fid_val) if isinstance(fid_val, (int, float, str)) else 0
                                    formation_type = item.get("formation_type", 0)
                                    fid_str = str(formation_type) if formation_type else "0"
                                    if fid_str not in formation_data:
                                        formation_data[fid_str] = {}
                                    formation_data[fid_str][fid_val] = item

                        # 兼容：单个字典含frame_id
                        elif isinstance(result_json, dict) and "frame_id" in result_json:
                            fid_val = result_json.get("frame_id", 0)
                            fid_val = int(fid_val) if isinstance(fid_val, (int, float, str)) else 0
                            fid_str = "0"  # 默认编队ID
                            if fid_str not in formation_data:
                                formation_data[fid_str] = {}
                            formation_data[fid_str][fid_val] = result_json

                except Exception as cache_err:
                    print(f"[监听] 缓存异常: {cache_err}")
                    import traceback
                    traceback.print_exc()

        except socket.error as e:
            print(f"[监听] Socket异常: {e}")
            if not reconnect_listen_pipe():
                time.sleep(1)
                continue
        except Exception as outer_err:
            print(f"[监听] 未预期异常: {outer_err}")
            import traceback
            traceback.print_exc()
            time.sleep(0.01)
            continue

    print("[监听] 退出")

# ========================= 初始化和主控制函数（新增）=========================
def init_sockets():
    """初始化两个Unix Socket连接"""
    global sock_send, sock_listen
    sock_send = create_unix_socket(SOCKET_PATH_CLIENT_TO_SERVER)
    sock_listen = create_unix_socket(SOCKET_PATH_SERVER_TO_CLIENT)
    return sock_send is not None and sock_listen is not None

def main():
    """主函数：初始化Socket + 启动监听线程 + 可视化 + 命令交互"""
    global is_listening, is_playing, playback_thread, current_playback_frame, playback_speed

    # 1. 初始化Socket连接
    if not init_sockets():
        print("[主程序] Socket初始化失败，退出")
        return

    # 2. 启动监听线程
    listen_thread = threading.Thread(target=listen_pipe_continually, daemon=True)
    listen_thread.start()

    # 3. 启动播放控制线程
    playback_thread = threading.Thread(target=playback_worker, daemon=True)
    playback_thread.start()

    # 4. 启动可视化界面
    launch_formation_viewer()

    # 5. 命令交互循环
    print("\n===== 命令交互 =====")
    print("输入命令编号(1-15)发送仿真命令 | 输入播放控制指令：")
    print("  play - 开始播放 | pause - 暂停 | speed x - 调整速度 | goto x - 跳帧 | exit - 退出")
    while is_listening:
        try:
            cmd_input = input("> ").strip()
            if not cmd_input:
                continue

            # 退出指令
            if cmd_input.lower() == "exit":
                is_listening = False
                is_playing = False
                print("退出中...")
                break

            # 播放控制：play
            elif cmd_input.lower() == "play":
                is_playing = True
                print(f"已开始播放（当前速度：{playback_speed:.1f}x）")

            # 播放控制：pause
            elif cmd_input.lower() == "pause":
                is_playing = False
                print("已暂停播放")

            # 播放控制：speed x
            elif cmd_input.lower().startswith("speed "):
                try:
                    speed = float(cmd_input.split()[1])
                    if speed > 0:
                        playback_speed = speed
                        print(f"播放速度已设为：{speed:.1f}x")
                    else:
                        print("速度必须大于0")
                except:
                    print("格式错误！示例：speed 1.5")

            # 播放控制：goto x
            elif cmd_input.lower().startswith("goto "):
                try:
                    frame = int(cmd_input.split()[1])
                    with viz_lock:
                        if 0 <= frame <= max_cached_frame:
                            current_playback_frame = frame
                            print(f"已跳转到帧#{frame}")
                        else:
                            print(f"帧号超出范围！有效范围：0-{max_cached_frame}")
                except:
                    print("格式错误！示例：goto 100")

            # 发送预设命令文件
            elif cmd_input in CMD_FILE_MAP:
                cmd_file = CMD_FILE_MAP[cmd_input]
                try:
                    with open(cmd_file, "r", encoding="utf-8") as f:
                        cmd_json = json.load(f)
                    # 缓存编队配置
                    update_formation_cache(cmd_json)
                    # 发送命令
                    if send_pipe_command(cmd_json):
                        print(f"成功发送命令：{cmd_input} → {cmd_file}")
                    else:
                        print(f"发送命令失败：{cmd_input}")
                except FileNotFoundError:
                    print(f"命令文件不存在：{cmd_file}")
                except json.JSONDecodeError:
                    print(f"命令文件格式错误：{cmd_file}")

            else:
                print("无效指令！")

        except KeyboardInterrupt:
            is_listening = False
            is_playing = False
            print("\n用户中断，退出中...")
            break
        except Exception as e:
            print(f"命令处理异常：{e}")

    # 清理资源
    if sock_send:
        sock_send.close()
    if sock_listen:
        sock_listen.close()
    plt.close('all')
    print("主程序已退出")

if __name__ == "__main__":
    main()