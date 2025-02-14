import pandas as pd
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
from matplotlib.widgets import Button

# 读取CSV文件
file_path = './all_line.csv'  # 替换为您的文件路径
data = pd.read_csv(file_path)

# 检查数据类型，确保 x 和 y 列为数值类型
data['x'] = pd.to_numeric(data['x'], errors='coerce')
data['y'] = pd.to_numeric(data['y'], errors='coerce')

# 删除无效值（如果存在）
data = data.dropna(subset=['x', 'y'])

# 提取绿色点、红色起点和橙色分支点
green_points = data[['x', 'y']].values  # 转换为 NumPy数组
start_points = [(2.41209, 0.374651), (-15.1729, 78.3113), (-6.75753, -101.319), (-92.9438, -46.3207),
                (-141.743, -67.0911), (-128.672, -11.3069), (-45.8839, 87.4729)]  # 红色点
branch_points = [(14.2777, -2.80559), (1.51205, -75.0624), (-133.693, -35.8988)]  # 橙色点

# 合并绿色点和橙色分支点作为有效路径点
valid_points = np.vstack([green_points, branch_points])

# 使用 KDTree 快速计算最近点
green_tree = KDTree(green_points)
branch_tree = KDTree(branch_points)

# 创建图
G = nx.Graph()

# 添加绿色点到图，并初始化 connected_count 属性
for i, point in enumerate(green_points):
    G.add_node(i, pos=point, connected_count=0)

# 添加橙色点到图
for i, point in enumerate(branch_points):
    G.add_node(len(green_points) + i, pos=point)

# 创建从红色点到绿色点的连接（添加连接限制）
start_nodes = []  # 记录红色点与最近绿色点的对应关系
for i, start in enumerate(start_points):
    dist, idx = green_tree.query(start)  # 找到最近的绿色点
    G.add_node(f"start_{i}", pos=start)  # 添加红色起点为新节点
    
    # 根据红色点连接限制
    if i == 3 or i == 5:  # 第四和第六个红色点连接两个绿色点
        dists, idxs = green_tree.query(start, k=2)  # 找最近的两个绿色点
        for d, green_idx in zip(dists, idxs):
            if G.nodes[green_idx]['connected_count'] < 2:  # 绿色点可用
                G.add_edge(f"start_{i}", green_idx, weight=d)  # 连接到绿色点
                G.nodes[green_idx]['connected_count'] += 1  # 增加连接计数
                start_nodes.append((f"start_{i}", green_idx))  # 保存映射关系
    else:  # 其他红色点只能连接一个绿色点
        if G.nodes[idx]['connected_count'] < 2:  # 绿色点可用
            G.add_edge(f"start_{i}", idx, weight=dist)  # 连接到最近的绿色点
            G.nodes[idx]['connected_count'] += 1  # 增加连接计数
            start_nodes.append((f"start_{i}", idx))  # 保存映射关系

# 橙色点与最近绿色点连接（添加连接限制）
for i, branch_point in enumerate(branch_points):
    dists_green, idxs_green = green_tree.query(branch_point, k=3)  # 找最近的三个绿色点
    for idx in idxs_green:
        if G.nodes[idx]['connected_count'] < 2:  # 绿色点可用
            G.add_edge(len(green_points) + i, idx, weight=np.linalg.norm(branch_point - green_points[idx]))
            G.nodes[idx]['connected_count'] += 1  # 增加连接计数

# 绿色点与最近的两个绿色点或橙色点链接（保持绿色点连接限制）
for i, green_point in enumerate(green_points):
    dists_green, idxs_green = green_tree.query(green_point, k=3)  # 最近的两个绿色点
    dists_branch, idxs_branch = branch_tree.query(green_point, k=1)  # 最近的一个橙色点
    idxs_branch_list = [idxs_branch] if isinstance(idxs_branch, np.int64) else idxs_branch.tolist()
    
    # 绿色点连接最近的两个绿色点
    green_idxs_to_connect = idxs_green[1:3] if len(idxs_green) >= 3 else idxs_green[1:]
    for idx in green_idxs_to_connect:
        if G.nodes[i]['connected_count'] < 2 and G.nodes[idx]['connected_count'] < 2:  # 每个绿色点最多连接两个
            G.add_edge(i, idx, weight=np.linalg.norm(green_point - green_points[idx]))
            G.nodes[i]['connected_count'] += 1
            G.nodes[idx]['connected_count'] += 1
    
    # 比较距离并添加连接到橙色点
    if len(idxs_branch_list) > 0 and dists_branch < dists_green[1] - 0.00001 and G.nodes[i]['connected_count'] < 2:
        branch_idx = len(green_points) + idxs_branch_list[0]
        G.add_edge(i, branch_idx, weight=np.linalg.norm(green_point - branch_points[idxs_branch_list[0]]))
        G.nodes[i]['connected_count'] += 1

# 绘制图和路径
plt.figure(figsize=(10, 10))

# 绘制绿色点
plt.scatter(green_points[:, 0], green_points[:, 1], c='green', s=10, label='Path Points')

# 绘制红色点
for i, red_point in enumerate(start_points):
    plt.scatter(red_point[0], red_point[1], c='red', s=50, label=f'Start Point {i + 1}')
    plt.text(red_point[0], red_point[1], f'Start {i + 1}', fontsize=10, color='blue', ha='right', va='bottom')

# 绘制橙色点
for i, branch_point in enumerate(branch_points):
    plt.scatter(branch_point[0], branch_point[1], c='orange', s=30, label=f'Branch Point {i + 1}')
    plt.text(branch_point[0], branch_point[1], f'Branch {i + 1}', fontsize=10, color='orange', ha='right', va='bottom')

# 绘制路径
for edge in G.edges():
    p1 = G.nodes[edge[0]]['pos']
    p2 = G.nodes[edge[1]]['pos']
    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], c='gray', linewidth=0.5)

plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('Customized Point Connections')
plt.grid(True)
plt.show()


# 初始化变量
selected_points = []  # 用于存储用户选择的起点和终点
current_path = []  # 当前路径

# 绘图函数
def draw_graph():
    plt.clf()  # 清除当前绘图

    # 绘制绿色点
    plt.scatter(green_points[:, 0], green_points[:, 1], c='green', s=10, label='Path Points')

    # 绘制红色点
    for i, red_point in enumerate(start_points):
        plt.scatter(red_point[0], red_point[1], c='red', s=50, label=f'Start Point {i + 1}')
        plt.text(red_point[0], red_point[1], f'Start {i + 1}', fontsize=10, color='blue', ha='right', va='bottom')

    # 绘制橙色点
    for i, branch_point in enumerate(branch_points):
        plt.scatter(branch_point[0], branch_point[1], c='orange', s=30, label=f'Branch Point {i + 1}')
        plt.text(branch_point[0], branch_point[1], f'Branch {i + 1}', fontsize=10, color='orange', ha='right', va='bottom')

    # 绘制当前路径（如果存在）
    if current_path:
        for i in range(len(current_path) - 1):
            node1 = current_path[i]
            node2 = current_path[i + 1]
            p1 = G.nodes[node1]['pos']
            p2 = G.nodes[node2]['pos']
            plt.plot([p1[0], p2[0]], [p1[1], p2[1]], c='blue', linewidth=2)  # 蓝色路径
        for node in current_path:
            pos = G.nodes[node]['pos']
            plt.scatter(pos[0], pos[1], c='blue', s=50)  # 蓝色节点

    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Interactive Path Planning')
    plt.grid(True)
    plt.draw()

# 鼠标点击事件处理
def on_click(event):
    global selected_points
    if event.xdata is None or event.ydata is None:
        return  # 鼠标点击在无效区域时返回
    if len(selected_points) < 2:
        pos = (event.xdata, event.ydata)
        selected_points.append(pos)
        print(f"Point {len(selected_points)} selected: {pos}")
        if len(selected_points) == 2:
            calculate_path()

# 计算路径函数
def calculate_path():
    global selected_points, current_path
    if len(selected_points) != 2:
        return
    start, end = selected_points

    # 查找最近的图节点
    start_dist, start_idx = green_tree.query(start)
    end_dist, end_idx = green_tree.query(end)

    # 计算最短路径
    if nx.has_path(G, start_idx, end_idx):
        current_path = nx.shortest_path(G, source=start_idx, target=end_idx, weight='weight')
        print(f"Shortest path: {current_path}")
    else:
        print("No path exists between the selected points.")

    draw_graph()

# 清除按钮事件处理
def clear(event):
    global selected_points, current_path
    selected_points = []  # 清除选择的点
    current_path = []  # 清除当前路径
    print("Cleared selections and path.")
    draw_graph()

# 初始化图形
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.2)
draw_graph()

# 添加按钮
ax_clear = plt.axes([0.7, 0.05, 0.1, 0.075])
btn_clear = Button(ax_clear, 'Clear')
btn_clear.on_clicked(clear)

# 添加鼠标点击事件
fig.canvas.mpl_connect('button_press_event', on_click)

plt.show()

