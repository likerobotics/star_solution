#!/usr/bin/env python3
import numpy as np
import os
import matplotlib.pyplot as plt
import plotly.graph_objects as go
import open3d as o3d

import os
import numpy as np
import math

from sklearn.cluster import DBSCAN


def largest_cluster_outliers(points_np: np.ndarray,
                             eps: float = 0.1,
                             min_samples: int = 10):
    """
    Делает кластеризацию DBSCAN и считает выбросами все точки,
    кроме самого большого кластера.

    :param points_np: numpy (N,4): [x,y,z,intensity]
    :param eps: радиус окрестности для DBSCAN (в метрах)
    :param min_samples: минимальное число соседей для точки
    :return: (labels, main_cluster, outliers)
             labels: массив меток кластеров для каждой точки
             main_cluster: точки из самого большого кластера
             outliers: все остальные точки
    """
    if points_np.shape[0] == 0:
        return None, np.empty((0, 4)), np.empty((0, 4))

    coords = points_np[:, :3]  # только x,y,z

    # Кластеризация
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(coords)
    labels = clustering.labels_

    # Найдём самый большой кластер (игнорируем -1 = шум DBSCAN)
    unique, counts = np.unique(labels[labels != -1], return_counts=True)
    if len(unique) == 0:
        # если DBSCAN ничего не нашёл → считаем всё выбросами
        return labels, np.empty((0, 4)), points_np

    largest_cluster_label = unique[np.argmax(counts)]

    # Делим точки
    main_cluster = points_np[labels == largest_cluster_label]
    outliers = points_np[labels != largest_cluster_label]

    return labels, main_cluster, outliers

def load_all_dumps(dump_dir="~/lidar_filtered_dumps"):
    """Загружает все .npy файлы из папки и возвращает список массивов."""
    dump_dir = os.path.expanduser(dump_dir)
    files = sorted([os.path.join(dump_dir, f) for f in os.listdir(dump_dir) if f.endswith(".npy")])
    if not files:
        raise FileNotFoundError(f"Нет файлов в {dump_dir}")
    clouds = [np.load(f) for f in files]
    return files, clouds


def load_all_dumps_with_pose(dump_dir="~/lidar_filtered_dumps"):
    """Загружает .npy облака и соответствующие .pose.txt файлы.
    
    Возвращает:
        files: список путей к .npy файлам
        clouds_and_poses: список кортежей (облако Nx4, поза [x, y, yaw])
    """
    dump_dir = os.path.expanduser(dump_dir)
    files = sorted([
        os.path.join(dump_dir, f)
        for f in os.listdir(dump_dir)
        if f.endswith(".npy")
    ])

    if not files:
        raise FileNotFoundError(f"Нет .npy файлов в {dump_dir}")

    clouds_and_poses = []
    for npy_path in files:
        base = npy_path[:-4]  # удаляем ".npy"
        pose_path = base + ".pose.txt"
        if not os.path.exists(pose_path):
            print(f"[WARN] Нет соответствующего pose-файла для {npy_path}, пропущено.")
            continue

        points = np.load(npy_path)
        with open(pose_path, "r") as f:
            line = f.readline().strip()
            pose = np.array([float(x) for x in line.split()], dtype=np.float32)

        clouds_and_poses.append((points, pose))

    return files, clouds_and_poses


def visualize_inline(points_np: np.ndarray, sample: int = 5000, title: str = ""):
    """Визуализация точек в Jupyter (matplotlib 3D scatter)."""
    if points_np.size == 0:
        print("Файл пустой или нет точек")
        return

    # Подвыборка для скорости
    if points_np.shape[0] > sample:
        idx = np.random.choice(points_np.shape[0], sample, replace=False)
        pts = points_np[idx]
    else:
        pts = points_np

    x, y, z, intensity = pts.T

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    sc = ax.scatter(x, y, z, c=intensity, cmap="viridis", s=2)
    fig.colorbar(sc, ax=ax, label="Intensity")

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title(title if title else "Lidar point cloud")

    plt.show()
    
def visualize_cloud(points_np: np.ndarray, title: str = "Lidar point cloud"):
    """Интерактивная визуализация всех точек с одинаковым масштабом по осям (равный шаг сетки)."""
    if points_np.size == 0:
        print("Файл пустой или нет точек")
        return

    x, y, z, intensity = points_np.T

    # Находим диапазоны
    x_min, x_max = np.min(x), np.max(x)
    y_min, y_max = np.min(y), np.max(y)
    z_min, z_max = np.min(z), np.max(z)

    # Вычисляем общий масштаб
    max_range = max(x_max - x_min, y_max - y_min, z_max - z_min)
    x_mid = (x_max + x_min) / 2
    y_mid = (y_max + y_min) / 2
    z_mid = (z_max + z_min) / 2

    # Одинаковый шаг сетки: делаем куб вокруг данных
    x_range = [x_mid - max_range / 2, x_mid + max_range / 2]
    y_range = [y_mid - max_range / 2, y_mid + max_range / 2]
    z_range = [z_mid - max_range / 2, z_mid + max_range / 2]

    fig = go.Figure(
        data=[
            go.Scatter3d(
                x=x,
                y=y,
                z=z,
                mode="markers",
                marker=dict(
                    size=2,
                    color=intensity,
                    colorscale="Viridis",
                    opacity=0.8
                )
            )
        ]
    )

    fig.update_layout(
        scene=dict(
            xaxis=dict(title="X [m]", range=x_range),
            yaxis=dict(title="Y [m]", range=y_range),
            zaxis=dict(title="Z [m]", range=z_range),
            aspectmode="manual",         # ручное управление
            aspectratio=dict(x=1, y=1, z=1)  # равный масштаб по осям
        ),
        title=title,
        margin=dict(l=0, r=0, b=0, t=30)
    )

    fig.show()


def plot_intensity_hist(points_np: np.ndarray, bins: int = 50, title: str = "Intensity distribution"):
    """Строит гистограмму интенсивности точек (matplotlib)."""
    if points_np.size == 0:
        print("Нет точек для гистограммы")
        return

    intensities = points_np[:, 3]

    plt.figure(figsize=(8, 5))
    plt.hist(intensities, bins=bins, color="blue", alpha=0.7)
    plt.xlabel("Intensity")
    plt.ylabel("Count")
    plt.title(title)
    plt.grid(True, alpha=0.3)
    plt.show()



def ransac_plane_outliers(points_np, distance_threshold=0.01, ransac_n=3, num_iterations=1000):
    """
    Находит плоскость методом RANSAC и возвращает выбросы (outliers).
    
    :param points_np: numpy массив (N,4) [x,y,z,intensity]
    :param distance_threshold: максимальное расстояние до плоскости, чтобы считать точку принадлежащей ей
    :param ransac_n: сколько точек использовать для гипотезы плоскости
    :param num_iterations: количество итераций RANSAC
    :return: (plane_model, inliers, outliers)
             plane_model: параметры плоскости (a,b,c,d) для ax+by+cz+d=0
             inliers: numpy (M,4) точки на плоскости
             outliers: numpy (K,4) точки вне плоскости
    """
    if points_np.shape[0] < ransac_n:
        raise ValueError("Недостаточно точек для RANSAC")

    # Open3D PointCloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_np[:, :3])

    # RANSAC плоскости
    plane_model, inliers_idx = pcd.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=ransac_n,
        num_iterations=num_iterations
    )

    inliers = points_np[inliers_idx]
    outliers = np.delete(points_np, inliers_idx, axis=0)

    return plane_model, inliers, outliers


def filter_points(points_np: np.ndarray,
                  intensity_min: float = 0.0,
                  intensity_max: float = 255.0,
                  range_min: float = 0.0,
                  range_max: float = np.inf,
                  z_min: float = -np.inf,
                  z_max: float = np.inf) -> np.ndarray:
    """
    Фильтрует точки по интенсивности, радиусу и Z.

    :param points_np: numpy (N,4): [x,y,z,intensity]
    :param intensity_min: минимальная интенсивность
    :param intensity_max: максимальная интенсивность
    :param range_min: минимальное расстояние от начала координат
    :param range_max: максимальное расстояние
    :param z_min: минимальная высота (ось Z)
    :param z_max: максимальная высота (ось Z)
    :return: отфильтрованный numpy (M,4)
    """
    if points_np.size == 0:
        return points_np

    x, y, z, intensity = points_np.T
    r = np.sqrt(x**2 + y**2 + z**2)

    mask = (
        (intensity >= intensity_min) & (intensity <= intensity_max) &
        (r >= range_min) & (r <= range_max) &
        (z >= z_min) & (z <= z_max)
    )

    return points_np[mask]

def visualize_plane_with_points(plane_model, inliers, outliers,
                                size=2, plane_size=1.5, title="Plane + Points"):
    """
    Визуализация плоскости (RANSAC) и точек с одинаковым масштабом по осям.
    
    :param plane_model: (a,b,c,d) параметры плоскости
    :param inliers: numpy (M,4) точки на плоскости
    :param outliers: numpy (K,4) точки вне плоскости
    :param size: размер точек
    :param plane_size: половина размера квадрата для визуализации плоскости
    :param title: заголовок
    """
    a, b, c, d = plane_model

    # Сетка для визуализации плоскости
    xx, yy = np.meshgrid(np.linspace(-plane_size, plane_size, 10),
                         np.linspace(-plane_size, plane_size, 10))

    if c != 0:
        zz = (-a * xx - b * yy - d) / c
    else:
        zz = np.zeros_like(xx)

    # Точки на плоскости (зелёные)
    x_in, y_in, z_in, _ = inliers.T
    trace_inliers = go.Scatter3d(
        x=x_in, y=y_in, z=z_in,
        mode="markers",
        marker=dict(size=size, color="green", opacity=0.6),
        name="Inliers"
    )

    # Точки вне плоскости (красные)
    x_out, y_out, z_out, _ = outliers.T
    trace_outliers = go.Scatter3d(
        x=x_out, y=y_out, z=z_out,
        mode="markers",
        marker=dict(size=size, color="red", opacity=0.6),
        name="Outliers"
    )

    # Плоскость
    trace_plane = go.Surface(
        x=xx, y=yy, z=zz,
        colorscale=[[0, "blue"], [1, "blue"]],
        opacity=0.3,
        name="Plane"
    )

    # === одинаковый масштаб по X,Y,Z ===
    all_points = np.vstack([inliers[:, :3], outliers[:, :3]])
    x_min, x_max = np.min(all_points[:, 0]), np.max(all_points[:, 0])
    y_min, y_max = np.min(all_points[:, 1]), np.max(all_points[:, 1])
    z_min, z_max = np.min(all_points[:, 2]), np.max(all_points[:, 2])

    max_range = max(x_max - x_min, y_max - y_min, z_max - z_min)
    x_mid = (x_max + x_min) / 2
    y_mid = (y_max + y_min) / 2
    z_mid = (z_max + z_min) / 2

    x_range = [x_mid - max_range / 2, x_mid + max_range / 2]
    y_range = [y_mid - max_range / 2, y_mid + max_range / 2]
    z_range = [z_mid - max_range / 2, z_mid + max_range / 2]

    fig = go.Figure(data=[trace_inliers, trace_outliers, trace_plane])
    fig.update_layout(
        scene=dict(
            xaxis=dict(title="X", range=x_range),
            yaxis=dict(title="Y", range=y_range),
            zaxis=dict(title="Z", range=z_range),
            aspectmode="manual",
            aspectratio=dict(x=1, y=1, z=1)
        ),
        title=title,
        margin=dict(l=0, r=0, b=0, t=30)
    )
    fig.show()

def project_points_to_plane(points_np: np.ndarray, plane_model):
    """
    Проецирует 3D точки на плоскость и возвращает их 2D координаты.

    :param points_np: numpy (N,3) или (N,4) [x,y,z,(intensity)]
    :param plane_model: (a,b,c,d) уравнение плоскости ax+by+cz+d=0
    :return: (proj_points_3d, proj_points_2d, basis)
             proj_points_3d: точки после проекции (N,3)
             proj_points_2d: координаты в базисе плоскости (N,2)
             basis: (u,v,n) ортонормированный базис
    """
    a, b, c, d = plane_model
    n = np.array([a, b, c], dtype=np.float64)
    n = n / np.linalg.norm(n)  # нормируем

    # Если вход (N,4) → берём только xyz
    xyz = points_np[:, :3]

    # --- проекция на плоскость ---
    dist = (xyz @ n + d)  # скалярное расстояние до плоскости
    proj_points = xyz - np.outer(dist, n)

    # --- строим базис (u,v) на плоскости ---
    # берём любую неколлинеарную вектору n ось
    tmp = np.array([1, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1, 0])
    u = np.cross(n, tmp)
    u = u / np.linalg.norm(u)
    v = np.cross(n, u)

    # --- 2D координаты ---
    u_coords = proj_points @ u
    v_coords = proj_points @ v
    proj_points_2d = np.vstack([u_coords, v_coords]).T

    return proj_points, proj_points_2d, (u, v, n)

def pattern_match(points_2d: np.ndarray, d_a: float = 0.01):
    """
    Ищет паттерн на 2D точках по алгоритму и возвращает точки, попавшие в паттерн.

    :param points_2d: numpy (N,2) [u,v] — координаты точек в 2D
    :param d_a: допуск в метрах
    :return: (matched_points, percent)
             matched_points: numpy (M,2) точки, попавшие в паттерн
             percent: процент точек (0..100)
    """

    if points_2d.shape[0] == 0:
        return np.empty((0, 2)), 0.0

    # --- 1. Находим L ---
    x_min, x_max = np.min(points_2d[:, 0]), np.max(points_2d[:, 0])
    y_min, y_max = np.min(points_2d[:, 1]), np.max(points_2d[:, 1])

    Lx = x_max - x_min
    Ly = y_max - y_min
    L = max(Lx, Ly)

    # --- 2. Область LxL ---
    cx = (x_min + x_max) / 2
    cy = (y_min + y_max) / 2
    half_L = L / 2

    x0, x1 = cx - half_L, cx + half_L
    y0, y1 = cy - half_L, cy + half_L

    # --- 3. Сетка 3x3 ---
    xs = np.linspace(x0, x1, 4)
    ys = np.linspace(y0, y1, 4)

    # Клетки 2,4,5,6,8 → индексы (ix,iy)
    target_cells = [(1,0), (0,1), (1,1), (2,1), (1,2)]

    matched = []

    # --- 4. Проверка попадания точек ---
    for px, py in points_2d:
        for (ix, iy) in target_cells:
            if (xs[ix] - d_a <= px <= xs[ix+1] + d_a and
                ys[iy] - d_a <= py <= ys[iy+1] + d_a):
                matched.append([px, py])
                break  # точка учтена, выходим из цикла

    matched_points = np.array(matched)
    percent = 100.0 * len(matched) / points_2d.shape[0]

    return matched_points, percent, L/3

def pose_to_transform(x, y, yaw):
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return np.array([
        [cos_yaw, -sin_yaw, 0.0, x],
        [sin_yaw,  cos_yaw, 0.0, y],
        [0.0,       0.0,    1.0, 0.0],
        [0.0,       0.0,    0.0, 1.0]
    ], dtype=np.float32)

def transform_point_cloud(points_xyz_i, transform_4x4):
    xyz = points_xyz_i[:, :3]
    intensity = points_xyz_i[:, 3:]
    xyz_hom = np.hstack((xyz, np.ones((xyz.shape[0], 1), dtype=xyz.dtype)))
    transformed_xyz = (transform_4x4 @ xyz_hom.T).T[:, :3]
    return np.hstack((transformed_xyz, intensity))

def compute_centroid(points_xyz_i):
    xyz = points_xyz_i[:, :3]
    return np.mean(xyz, axis=0)

def compare_clouds_by_centroid(points_A, pose_A, points_B, pose_B):
    T_A = pose_to_transform(*pose_A)
    T_B = pose_to_transform(*pose_B)

    cloud_A_world = transform_point_cloud(points_A, T_A)
    cloud_B_world = transform_point_cloud(points_B, T_B)

    centroid_A = compute_centroid(cloud_A_world)
    centroid_B = compute_centroid(cloud_B_world)

    dist_xy = np.linalg.norm(centroid_A[:2] - centroid_B[:2])
    dist_xyz = np.linalg.norm(centroid_A - centroid_B)

    return centroid_A, centroid_B, dist_xy, dist_xyz