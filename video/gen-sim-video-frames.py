import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import json
import math
from PIL import Image


DATA_PATH="./output/sim-frames"


def load_data():
    data_dir = f"./data/"

    if not Path(data_dir).exists():
        raise RuntimeError(f"Invalid run number ({RUN_NUM}) for map 795-2.")

    # Load metadata
    metadata = None
    metadata_path = f"{data_dir}/metadata.json"
    with open(metadata_path, 'r') as f:
        mdatastr = f.read()
        metadata = json.loads(mdatastr)

    # Load floor map
    floor_map = np.load(f"{data_dir}/floor_map.npy")

    # Load and store all run data
    configurations = []
    graph_points = []
    state_space = []

    for hy_i in range(4):
        config_path = f"{data_dir}/hy-{hy_i}-sim-0-configurations.npy"
        graph_path = f"{data_dir}/hy-{hy_i}-sim-0-graph-points.npy"
        ss_path = f"{data_dir}/hy-{hy_i}-sim-0-ss.npy"

        configurations.append(np.load(config_path))
        graph_points.append(np.load(graph_path))
        state_space.append(np.load(ss_path))

    return metadata, floor_map, configurations, graph_points, state_space


def plot_map(fig, ax, floor_map):
    # numcols, numrows = fig.get_size_inches()*fig.dpi # size in pixels
    im = ax.imshow(floor_map)
    ext = im.get_extent()
    numcols = ext[0] + ext[1]
    numrows = ext[2] + ext[3]

    ax.set_xlim([14,206])
    ax.set_ylim([100,45])

    return int(numcols), int(numrows)



def plot_straight_line(fig, ax, alpha=1.0):
    X = [22, 268]
    Y = [78, 78]
    ax.plot(X,Y,color='y',lw=5,alpha=alpha)


def disable_ticks(ax):
    ax.tick_params(
        bottom=False,      # ticks along the bottom edge are off
        top=False,         # ticks along the top edge are off
        left=False,
        right=False,
        labelleft=False,
        labelbottom=False) # labels along the bottom edge are off


def setup_ss_plot(fig, axins):
    AXISFONTSIZE=10
    axins.set_xlim([-math.pi/2, math.pi/2])
    axins.set_ylim([-1.5,1.5])
    axins.grid(True, which='both')
    axins.axhline(y=0, color='k')
    axins.axvline(x=0, color='k')
    axins.set_ylabel(r'$d$', fontsize=AXISFONTSIZE, rotation=0)
    axins.set_xlabel(r'$\theta$', fontsize=AXISFONTSIZE)

    axins.yaxis.set_label_coords(.1,.53)
    axins.xaxis.set_label_coords(.55,.1)



def plot_ss(fig, ax, traj, dlim, traj_len, needs_setup=False, color='k', linestyle='-'):
    cur_color = color
    cur_linestyle = linestyle
    theta_vals = [-point[0] for point in traj]
    d_vals = [point[1] for point in traj]


    # Plot IC
    theta_ic = theta_vals[0]
    d_ic = d_vals[0]

    # Plot traj
    ax.plot(theta_ic, d_ic, marker='o', markersize=6, color=cur_color)
    ax.plot(theta_vals, d_vals, color=cur_color, linestyle=cur_linestyle)

    # Short trajectory
    print(f"Length of traj: {len(traj)}")
    if len(traj) < traj_len:
        ax.plot(theta_vals[-1], d_vals[-1], marker='x', markersize=10, color='r')

    # Plot arrow
    POINTS_TO_PLOT = [50, 100, 200, 500, 1000, 1500]

    for point in POINTS_TO_PLOT:
        point = int(point)
        print(f"Len: {len(theta_vals)}\nPoint: {point}")
        if len(theta_vals)-2 <= point:
            break
        theta_arrow = theta_vals[point]
        d_arrow = d_vals[point]
        dx = theta_vals[point+1] - theta_vals[point]
        dy = d_vals[point+1] - d_vals[point]
        ax.arrow(theta_arrow, d_arrow, dx, dy, length_includes_head=True, shape='full', head_width=0.05, color=cur_color)


COLORS = ['r', 'g', 'b', 'orange']
def plot_traj_to_point(fig, ax, state_space, i):

    for j, traj in enumerate(state_space):
        plot_ss(fig, ax, traj[0:i+1], None, i, color=COLORS[j])



def plot_trajectory_on_map(fig, ax, trajectory, traj_len, color):
    #if not simulation_finished(trajectory):
    if len(trajectory) < traj_len:
        linestyle='--'
        end_marker='x'
    else:
        linestyle = '-'
        end_marker='o'
    xvals = [point[0] for point in trajectory]
    yvals = [point[1] for point in trajectory]
    # marker, markersize previously
    ax.plot(xvals, yvals, linewidth=1, linestyle=linestyle, color=color, alpha=0.5)
    ax.plot(xvals[-1], yvals[-1], marker=end_marker, markersize=5, alpha=0.8, color=color)



def gen_frame(i):
    fig, ax = plt.subplots(figsize=(16, 9), dpi=1920/16)
    disable_ticks(ax)

    metadata, floor_map, configurations, graph_points, state_space = load_data()

    # Generate inset axes
    # Top down map
    axins_topdown = ax.inset_axes([0,2/3,1,1/3])
    disable_ticks(axins_topdown)
    plot_map(fig, axins_topdown, floor_map)
    plot_straight_line(fig,axins_topdown)

    for j, traj in enumerate(graph_points):
        plot_trajectory_on_map(fig, axins_topdown, traj[0:i+1], i, COLORS[j])

    # State space plot
    axins_ss = ax.inset_axes([0,0,1/2,2/3])
    setup_ss_plot(fig, axins_ss)
    disable_ticks(axins_ss)
    plot_traj_to_point(fig, axins_ss, state_space, i)
    # plot_map(fig, axins_ss, floor_map)

    axins_vid0 = ax.inset_axes([1/2,1/3,1/4,1/3])
    axins_vid1 = ax.inset_axes([1/2,0,1/4,1/3])
    axins_vid2 = ax.inset_axes([3/4,1/3,1/4,1/3])
    axins_vid3 = ax.inset_axes([3/4,0,1/4,1/3])
    disable_ticks(axins_vid0)
    disable_ticks(axins_vid1)
    disable_ticks(axins_vid2)
    disable_ticks(axins_vid3)

    num0 = min(i, len(configurations[0])-2)
    im0 = Image.open(f"./frames/hy-0-sim-0-videoframes/{num0:04}.png")

    num1 = min(i, len(configurations[1])-2)
    im1 = Image.open(f"./frames/hy-1-sim-0-videoframes/{num1:04}.png")

    num2 = min(i, len(configurations[2])-2)
    im2 = Image.open(f"./frames/hy-2-sim-0-videoframes/{num2:04}.png")

    num3 = min(i, len(configurations[3])-2)
    im3 = Image.open(f"./frames/hy-3-sim-0-videoframes/{num3:04}.png")

    print(im0.size)

    axins_vid0.imshow(im0)
    axins_vid1.imshow(im1)
    axins_vid2.imshow(im2)
    axins_vid3.imshow(im3)

    # axins_vid0.set_xlim([80,560])
    axins_vid0.text(100, 80, r"$\beta/\gamma=5000$", bbox=dict(boxstyle="round",ec=COLORS[0],fc=COLORS[0]))
    # ax.text(img_x, img_y, r'Image', size=20, bbox=dict(boxstyle="round",ec=(1,1,1),fc=(1,1,1)))
    # axins_vid1.set_xlim([80,560])
    axins_vid1.text(100, 80, r"$\beta/\gamma=500$", bbox=dict(boxstyle="round",ec=COLORS[1],fc=COLORS[1]))
    # axins_vid2.set_xlim([80,560])
    axins_vid2.text(100, 80, r"$\beta/\gamma=50$", bbox=dict(boxstyle="round",ec=COLORS[2],fc=COLORS[2]))
    # axins_vid3.set_xlim([80,560])
    axins_vid3.text(100, 80, r"$\beta/\gamma=5$", bbox=dict(boxstyle="round",ec=COLORS[3],fc=COLORS[3]))
    # axins_vid0.set_ylim([])

    # fig.show()
    # plt.show()
    fig.tight_layout()
    fig.savefig(f"{DATA_PATH}/{i:04}.png")
    # fig.savefig(f"{DATA_PATH}/{i:04}.png",bbox_inches='tight',pad_inches = 0)

if __name__ == '__main__':
    Path(DATA_PATH).mkdir(exist_ok=True, parents=True)
    START=1200
    NUM=800
    for i in range(START,START+NUM):
        gen_frame(i)
