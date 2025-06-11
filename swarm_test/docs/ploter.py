import os
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# 获取当前脚本的目录
script_dir = os.path.dirname(os.path.abspath(__file__))

# 绘制距离-时间曲线图
def plot1():
    csv_file = script_dir + "/data0729/data_copy1.csv"

    df = pd.read_csv(csv_file)

    x1 = df.iloc[4]; y1 = df.iloc[5] # N+DCBF:1,5
    x2 = df.iloc[2]; y2 = df.iloc[3] # A+ACBF:3 

    x_r = [i*0.02-0.09 for i in range(y1.size)]

    palettes = sns.color_palette("hls", 8)
    plotsize = (9,5)
    # 创建图形和轴
    sns.set_theme(style="whitegrid", rc={'font.family':'serif', "axes.labelsize":15})
    fig, axs = plt.subplots(figsize = plotsize)
    axs.set_xlabel("Time [s]")
    axs.set_ylabel("Obstacle Distance [m]")
    axs.set_xlim(2.5, 5.5)
    axs.set_ylim(0, 4.0)

    axs.plot(x_r, 0.8*np.ones(len(x_r)), linestyle="--", color="red", linewidth=3, label='Collision Distance')

    # 绘制折线图
    sns.lineplot(x=x_r, y=y1, label='N-DCBF', linewidth=3)
    sns.lineplot(x=x2, y=y2, label='E-ECBF(Ours)', linewidth=3)

    # 设置网格线
    axs.grid(True)

    # 添加图例
    legend = plt.legend(loc='upper right')
    for text in legend.get_texts():
        text.set_fontsize(15)
    legend.set_visible(True)

    # 显示图形
    plt.show()


# 绘制距离直方图
def plot2():
    csv_file = script_dir + "/data0729/data_copy2.csv"
    df = pd.read_csv(csv_file)

    nums = 6.25/0.125
    x_value = [i*0.125 for i in range(int(nums))]
    y1_value = [0 for i in range(int(nums))]
    y2_value = [0 for i in range(int(nums))]

    y1_data = df.iloc[0].tolist() # A+ACBF:0
    y2_data = df.iloc[2].tolist() # N+DCBF:1,2
    for i in range(len(y1_data)):
        index = int(y1_data[i]/0.125)
        y1_value[index] += 1

    for i in range(len(y2_data)):
        index = int(y2_data[i]/0.125)
        y2_value[index] += 1
    
    palettes = sns.color_palette("hls", 8)

    # 创建两个DataFrame
    df1 = pd.DataFrame({
        'category': x_value,
        'value': y1_value
    })

    df2 = pd.DataFrame({
        'category': x_value,
        'value': y2_value
    })

    # 创建一个图形和两个轴对象
    sns.set_theme(style="whitegrid", rc={'font.family':'serif', "axes.labelsize":15})
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 5))  # 2行1列的子图布局
    
    # 在第一个子图上绘制df的柱状图
    sns.barplot(ax=ax1, x='category', y='value', data=df1, color=palettes[0], edgecolor='black', width=1.0, label='E-ECBF(Ours)')
    # 在第二个子图上绘制df2的柱状图
    sns.barplot(ax=ax2, x='category', y='value', data=df2, color=palettes[5], edgecolor='black', width=1.0, label='N-DCBF')

    # 隐藏 ax1, ax2 子图的 x 轴和 y 轴标签
    ax1.set_xlabel("")
    ax1.set_ylabel("")
    ax2.set_xlabel("")
    ax2.set_ylabel("")
    # 设置 x 轴刻度
    ax1.set_xticks([i for i in range(0, len(x_value), 8)])  # 每隔2个单位显示一个刻度
    ax2.set_xticks([i for i in range(0, len(x_value), 8)])  # 每隔2个单位显示一个刻度

    # 设置x，y轴刻度线
    # ax1.grid(True)

    fig.supxlabel('Distance [m]')
    fig.supylabel('Number of Samples')

    # 添加图例
    # 添加图例到 ax1，并设置图例字体大小
    legend1 = ax1.legend(prop={'size': 15})
    # 添加图例到 ax2，并设置图例字体大小
    legend2 = ax2.legend(prop={'size': 15})
    legend1.set_visible(True)
    legend2.set_visible(True)

    # 调整子图间距
    plt.tight_layout()

    # 显示图表
    plt.show()

# 计算时间箱线图
def plot3():
    csv_file = script_dir + "/data0807/data.csv"
    df = pd.read_csv(csv_file)
    y1 = df.iloc[0].tolist() # 前端路径搜索时间
    y2 = df.iloc[1].tolist() # 后端时间-无障碍物
    y3 = df.iloc[2].tolist() # 后端时间-有障碍物

    y1_u = [i/1000.0 for i in y1]
    y2_u = [i/1000.0 for i in y2]
    y3_u = [i/1000.0 for i in y3]

    # 将数组放入一个字典中，并为每组数据指定一个标签
    data_dict = {
        'Path searching': y1_u,
        'MPC w/o obstacle': y2_u,
        'MPC w/ obstacle': y3_u
    }

    df_use = pd.DataFrame(data_dict)

    # 使用NumPy的std函数计算标准差  
    std_path_searching = np.std(data_dict['Path searching'])  
    std_mpc_wo_obstacle = np.std(data_dict['MPC w/o obstacle'])  
    std_mpc_w_obstacle = np.std(data_dict['MPC w/ obstacle'])  
    
    print("Path searching standard deviation:", std_path_searching)  
    print("MPC w/o obstacle standard deviation:", std_mpc_wo_obstacle)  
    print("MPC w obstacle standard deviation:", std_mpc_w_obstacle)

    # 使用NumPy计算均值  
    mean_path_searching = np.mean(data_dict['Path searching'])  
    mean_mpc_wo_obstacle = np.mean(data_dict['MPC w/o obstacle'])  
    mean_mpc_w_obstacle = np.mean(data_dict['MPC w/ obstacle'])  
    
    print("Path searching mean:", mean_path_searching)  
    print("MPC w/o obstacle mean:", mean_mpc_wo_obstacle)  
    print("MPC w obstacle mean:", mean_mpc_w_obstacle)

    sns.set_theme(style="whitegrid", rc={'font.family':'serif', "axes.labelsize":15})
    fig, ax1 = plt.subplots(figsize=(9, 5)) 
    ax1.set_ylabel("Runtime [s]", fontsize=15)
    ax1.tick_params(axis='x', labelsize=15)  # 设置x轴刻度的字体大小  
    ax1.tick_params(axis='y', labelsize=15)

    # 绘制箱线图，指定x为分类标签，y为数据值
    sns.boxplot(data=df_use, width=0.3, color='white', linewidth=2,ax=ax1, 
                medianprops=dict(color='orange', linewidth=2),
                flierprops=dict(marker='x', markeredgecolor='red', markersize=8))

    ax1.set_ylim(0, 0.12)
    # 设置x，y轴刻度线
    ax1.grid(True)

    # 显示图表
    plt.show()

def plot_test():
    csv_file1 = script_dir + "/data0807/data-front.csv"
    df1 = pd.read_csv(csv_file1)
    y1 = df1.iloc[0].tolist() # 前端路径搜索时间

    df2 = pd.read_csv(script_dir + "/data0807/data-end.csv")
    y2_t = df2.iloc[4].tolist() # 后端路径搜索标记
    y2_v = df2.iloc[5].tolist() # 后端路径搜索时间

    y2 = [] # 后端时间-无障碍物
    y3 = [] # 后端时间-有障碍物
    for i in range(len(y2_t)):
        if (y2_t[i] == 1):
            if y2_v[i]>70.0:
                y3.append(y2_v[i])
        else:
            y2.append(y2_v[i])
    # print(len(y1))5
    print(len(y2))
    print(y2)

def plot_one():
    # print("plot_one")
    df = pd.read_excel(script_dir + "/data0729/data_use.xlsx")
    df_head = df.head()
    # print(df_head)
    plotsize = (9, 5)
    # 设置matplotlib的字体为Times New Roman
    matplotlib.rcParams['font.family'] = 'Times New Roman'
    # 解决负号('-')显示为方块的问题
    matplotlib.rcParams['font.sans-serif'] = ['SimHei']  # 指定默认字体，SimHei为黑体
    matplotlib.rcParams['axes.unicode_minus'] = False  # 解决负号('-')显示为方块的问题

    # sns.set_theme(style='whitegrid', rc={'font.family': 'serif', 'font.serif': ['Times New Roman'], "axes.labelsize": 15}) # style='whitegrid'
    sns.set_theme(style='whitegrid')

    # 获取颜色列表
    palettes = sns.color_palette()

    fig, axs = plt.subplots(figsize=plotsize)
    axs.set_xlabel("X Position [m]")
    axs.set_ylabel("Y Position [m]")
    axs.set_xlim(0, 8)
    axs.set_ylim(-2.2)

    # 使用tick_params设置坐标轴刻度标签的字体大小
    axs.tick_params(axis='both', labelsize=15)  # 'both'代表x轴和y轴

    sns.lineplot(data=df, x=df.columns[2], y=df.columns[3], 
                alpha=0.9, color=palettes[0], linewidth=4, label="MMPC")

    sns.lineplot(data=df, x=df.columns[4], y=df.columns[5], 
                alpha=0.9, color=palettes[1], linewidth=4, label="MPC-DCBF")

    sns.lineplot(data=df, x=df.columns[6], y=df.columns[7], 
                alpha=0.9, color=palettes[2], linewidth=4, label="MPC-ECBF (tau_scale=0.00)")

    sns.lineplot(data=df, x=df.columns[8], y=df.columns[9], 
                alpha=0.9, color=palettes[3], linewidth=4, label="MPC-ECBF (tau_scale=0.35)")

    # 标出t=2.3的点
    t_index = 23
    xt = []
    yt = []
    color_t = [(.0, .0, .0)]
    for i in range(5):
        xt.append(df[df.columns[i*2]][t_index])
        yt.append(df[df.columns[i*2+1]][t_index])
        if i>0:
            color_t.append(palettes[i-1])

    plt.scatter(xt, yt, c=color_t, s=30, alpha=1)

    # 创建一个圆形补丁，指定中心和半径
    circle = patches.Circle((df[df.columns[0]][t_index], df[df.columns[1]][t_index]), alpha=0.5 ,radius=1.0, edgecolor='none', facecolor='dimgray')
    axs.add_patch(circle)

    theta = np.linspace(0, 2 * np.pi, 100)  # 生成圆上的点  
    x_center, y_center = df[df.columns[0]][t_index], df[df.columns[1]][t_index]  # 圆的中心坐标  
    radius = 1.0  # 圆的半径  
    x = x_center + radius * np.cos(theta)  
    y = y_center + radius * np.sin(theta)  
    
    # 绘制圆形线条，使用虚线样式  
    axs.plot(x, y, 'k--', alpha=0.5, lw=5)  # 'k--'表示黑色虚线，lw=2表示线宽为2 

    # 关闭网格线
    axs.grid(True)

    # 创建图例
    legend = plt.legend()

    # 设置图例的字体大小
    for text in legend.get_texts():
        text.set_fontsize(15)  # 设置字体大小为12

    # 隐藏图例
    legend.set_visible(True)

    plt.show()


def plot_one1():

    df = pd.read_excel(script_dir + "/data1226/output.xlsx")
    df_head = df.head()
    print(df_head)

    plotsize = (9, 5)
    # 设置matplotlib的字体为Times New Roman
    # matplotlib.rcParams['font.family'] = 'Times New Roman'
    # 解决负号('-')显示为方块的问题
    matplotlib.rcParams['font.sans-serif'] = ['SimHei']  # 指定默认字体，SimHei为黑体
    matplotlib.rcParams['axes.unicode_minus'] = False  # 解决负号('-')显示为方块的问题
    sns.set_theme(style='whitegrid')

    # 获取颜色列表
    palettes = sns.color_palette()

    fig, axs = plt.subplots(figsize=plotsize)
    axs.set_xlabel("X Position [m]")
    axs.set_ylabel("Y Position [m]")
    axs.set_xlim(0, 8)
    axs.set_ylim(-2.2)

    # 使用tick_params设置坐标轴刻度标签的字体大小
    axs.tick_params(axis='both', labelsize=15)  # 'both'代表x轴和y轴

    sns.lineplot(data=df, x=df.columns[2], y=df.columns[3], 
                alpha=0.9, color=palettes[0], linewidth=4, label="MMPC")
    
    sns.lineplot(data=df, x=df.columns[4], y=df.columns[5], 
                alpha=0.9, color=palettes[1], linewidth=4, label="MPC-CBF")
    
    sns.lineplot(data=df, x=df.columns[6], y=df.columns[7], 
                alpha=0.9, color=palettes[2], linewidth=4, label="MPC-DCBF")
    
    sns.lineplot(data=df, x=df.columns[8], y=df.columns[9], 
                alpha=0.9, color=palettes[7], linewidth=4, label="MPC-ECBF(Ke=0)")
    
    sns.lineplot(data=df, x=df.columns[10], y=df.columns[11], 
                alpha=0.9, color=palettes[6], linewidth=4, label="MPC-ECBF(Ke=0.1)")
    
    sns.lineplot(data=df, x=df.columns[12], y=df.columns[13], 
                alpha=0.9, color=palettes[4], linewidth=4, label="MPC-ECBF(Ke=0.2)")

    sns.lineplot(data=df, x=df.columns[14], y=df.columns[15], 
                alpha=0.9, color=palettes[3], linewidth=4, label="MPC-ECBF(Ke=0.35)")

    x_data_8 = df.iloc[:16,16]
    y_data_8 = df.iloc[:16,17]
    sns.lineplot(data=df, x=x_data_8, y=y_data_8, 
                alpha=0.9, color=palettes[5], linewidth=4, label="MPC-ECBF(Ke=0.4)")


    # 标出t=2.3的点
    t_index = 23
    xt = []
    yt = []
    color_t = [(.0, .0, .0)]
    for i in range(8):
        xt.append(df[df.columns[i*2]][t_index])
        yt.append(df[df.columns[i*2+1]][t_index])

    color_t.append(palettes[0])
    color_t.append(palettes[1])
    color_t.append(palettes[2])
    color_t.append(palettes[7])
    color_t.append(palettes[6])
    color_t.append(palettes[4])
    color_t.append(palettes[3])

    plt.scatter(xt, yt, c=color_t, s=30, alpha=1)

    # 创建一个圆形补丁，指定中心和半径
    circle = patches.Circle((df[df.columns[0]][t_index], df[df.columns[1]][t_index]), alpha=0.5 ,radius=1.0, edgecolor='none', facecolor='dimgray')
    axs.add_patch(circle)

    theta = np.linspace(0, 2 * np.pi, 100)  # 生成圆上的点  
    x_center, y_center = df[df.columns[0]][t_index], df[df.columns[1]][t_index]  # 圆的中心坐标  
    radius = 1.0  # 圆的半径  
    x = x_center + radius * np.cos(theta)  
    y = y_center + radius * np.sin(theta)  

    # 绘制圆形线条，使用虚线样式  
    axs.plot(x, y, 'k--', alpha=0.5, lw=5)  # 'k--'表示黑色虚线，lw=2表示线宽为2 


    # 关闭网格线
    axs.grid(True)

    # 图例
    legend = plt.legend(loc='lower right')
    legend.set_visible(True)

    # 设置图例的字体大小
    for text in legend.get_texts():
        text.set_fontsize(15)  # 设置字体大小为12

    plt.show()


if __name__ == '__main__':
    plot_one1()
    # plot_one()
    # plot1()
    # plot2()
    # plot3() # 计算时间箱线图
