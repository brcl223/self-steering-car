from matplotlib import pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
from itertools import count
from pathlib import Path
import json
import math
import sys
import cv2
#from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
from matplotview import inset_zoom_axes

plt.rcParams['text.usetex'] = True

##########################################################
# Modified from the following tutorial:
# https://colab.research.google.com/github/facebookresearch/habitat-sim/blob/main/examples/tutorials/colabs/ECCV_2020_Navigation.ipynb#scrollTo=mY-75WXw918t
##########################################################
# display a topdown map with matplotlib
#def display_map(topdown_map, key_points=None, desired_traj=None):
#    plt.figure(figsize=(12, 8))
#    ax = plt.subplot(1, 1, 1)
#    ax.axis("off")
#    plt.imshow(topdown_map)
#    # Plot desired trajectory
#    if desired_traj is not None:
#        for point in desired_traj:
#            plt.plot(point[0], point[1], color="yellow", marker="x", markersize=10, alpha=0.8)
#    # plot points on map
#    if key_points is not None:
#        for point in key_points:
#            plt.plot(point[0], point[1], marker="o", markersize=10, alpha=0.8)
#    plt.show(block=False)

TITLEFONTSIZE = 20
AXISFONTSIZE = 14

def plot_ss(fig, ax, ss, dlim, traj_len, needs_setup=False, color='k', linestyle='-'):
    if needs_setup:
        EPS=0.2
        ax.grid(True, which='both')
        ax.axhline(y=0, color='k')
        ax.axvline(x=0, color='k')
        ax.set_title('HM3D State Space',fontsize=TITLEFONTSIZE)
        ax.set_xlim([-math.pi/2, math.pi/2])
        ax.set_ylim([dlim[0]-EPS, dlim[1]+EPS])
        ax.set_ylabel(r'$d$ [m]', fontsize=AXISFONTSIZE)
        ax.set_xlabel(r'$\theta$ [rad]', fontsize=AXISFONTSIZE)
        custom_lines = [Line2D([0], [0], color='b', linestyle='-', lw=2),
                        Line2D([0], [0], color='orange', linestyle='-', lw=2), # linestyle='--'
                        Line2D([0], [0], color='g', linestyle='-', lw=2), # linestyle='-.'
                        Line2D([0], [0], color='r', marker='x', markersize=10, lw=0)]
        ax.legend(custom_lines, [r'$\frac{\beta}{\gamma}=20$', 
            r'$\frac{\beta}{\gamma}=2$', 
            r'$\frac{\beta}{\gamma}=0.2$',
            'Collision'],loc='lower right',fontsize=13)

    for traj in ss:
        cur_color = color
        cur_linestyle = linestyle
        theta_vals = [-point[0] for point in traj]
        d_vals = [point[1] for point in traj]


        # Plot IC
        theta_ic = traj[0][0]
        d_ic = traj[0][1]

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
            if len(theta_vals) <= point:
                break
            theta_arrow = theta_vals[point]
            d_arrow = d_vals[point]
            dx = theta_vals[point+1] - theta_vals[point]
            dy = d_vals[point+1] - d_vals[point]
            ax.arrow(theta_arrow, d_arrow, dx, dy, length_includes_head=True, shape='full', head_width=0.05, color=cur_color)


def plot_map(fig, ax, topdown_map, desired_traj=None):
    ax.axis("off")
    ax.imshow(topdown_map)
    # Plot desired trajectory
    if desired_traj is not None:
        xvals = [point[0] for point in desired_traj]
        yvals = [point[1] for point in desired_traj]
        ax.plot(xvals, yvals, 'y-', linewidth=4)
            #ax.plot(point[0], point[1], 'y--', marker="x", markersize=10, alpha=0.)
    #fig.show()


def perc_successful_trajs(trajectories):
    ctr = 0.
    tot = 0.
    for traj in trajectories:
        if simulation_finished(traj):
            ctr += 1
        tot += 1
    return ctr / tot


def plot_trajectory_on_map(fig, ax, trajectory_set, traj_len, color):
    print("Plotting trajectories...")
    for trajectory in trajectory_set:
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


def plot_trajectory_inset(fig, ax):
    # remove white padding
    #fig.subplots_adjust(left=0, right=1, top=1, bottom=0)
    #fig.axis('off')
    #fig.axis('image')

    # redraw the canvas
    #fig = plt.gcf()
    #fig.canvas.draw()

    # convert canvas to image using numpy
    #img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
    #img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
    #img = np.flip(img.reshape(fig.canvas.get_width_height()[::-1] + (3,)), axis=0)
    #img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))

    # opencv format
    #img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    #cv2.imwrite('test.png', img)

    # inset axes....
    #axins = ax.inset_axes([0.25, 0.5, 0.5, 0.47])
    axins = inset_zoom_axes(ax, [0.18, 0.55, 0.25, 0.75])
    #axins = zoomed_inset_axes(ax, zoom = 2, loc='right')
    #axins.imshow(img, origin="lower", extent=(0,300, -50, 250))
    #axins.imshow(img, origin="lower", extent=(0,200, -25, 175))
    #axins.imshow(img, origin="lower")
    # subregion of the original image
    #x1, x2, y1, y2 = 0, 400, 0, 400
    #x1, x2, y1, y2 = 15, 60, 40, 110
    x1, x2, y1, y2 = 15, 45, 110, 40
    axins.set_xlim(x1, x2)
    axins.set_ylim(y1, y2)
    axins.set_xticklabels([])
    axins.set_yticklabels([])

    #ax.indicate_inset_zoom(axins, edgecolor="black")


def add_traj_plot_metadata(fig, ax):
    # Add title
    ax.set_title("HM3D Exploration Results", y=-0.01, fontsize=TITLEFONTSIZE)

    # Add a legend
    custom_lines = [Line2D([0], [0], color='b', lw=2),
                    Line2D([0], [0], color='orange', lw=2),
                    Line2D([0], [0], color='g', lw=2),
                    Line2D([0], [0], color='k', marker='x', linestyle='--', lw=2),
                    Line2D([0], [0], color='k', marker='o', linestyle='-', lw=2)]
    ax.legend(custom_lines, 
    [r'$\frac{\beta}{\gamma}=20$', r'$\frac{\beta}{\gamma}=2$', r'$\frac{\beta}{\gamma}=0.2$', 'Collision', 'Success'], 
    loc='upper right',fontsize=14)


def simulation_finished(configurations):
    MAGIC_NUM = 300
    return len(configurations) > MAGIC_NUM


def load_data(glb_map, run_num, hyper_param_set):
    data_dir = f"./data/experiments/{glb_map}/run-{run_num}/"

    if not Path(data_dir).exists():
        raise RuntimeError(f"Invalid run number ({run_num}) for map {glb_map}.")

    # Load metadata
    metadata = None
    metadata_path = f"{data_dir}/metadata.json"
    with open(metadata_path, 'r') as f:
        mdatastr = f.read()
        metadata = json.loads(mdatastr)

    # Load floor map
    floor_map = np.load(f"{data_dir}/floor_map.npy")
    total_sims = 0

    # Load and store all run data
    configurations = []
    graph_points = []
    state_space = []

    for i in count():
        config_path = f"{data_dir}/hy-{hyper_param_set}-sim-{i}-configurations.npy"
        graph_path = f"{data_dir}/hy-{hyper_param_set}-sim-{i}-graph-points.npy"
        ss_path = f"{data_dir}/hy-{hyper_param_set}-sim-{i}-ss.npy"

        # We've reached our limit
        if not Path(config_path).exists():
            total_sims = i
            break

        configurations.append(np.load(config_path))
        graph_points.append(np.load(graph_path))
        state_space.append(np.load(ss_path))

    return total_sims, metadata, floor_map, configurations, graph_points, state_space



def print_header(s):
    print("##################################################")
    print(f"{s}")
    print("##################################################\n")



GLB_MAP = '795-2'
def main():
    if len(sys.argv) > 1:
        run_num = int(sys.argv[1])
        assert run_num >= 0, "Run Number must be positive integer!"
    else:
        raise RuntimeError("Invalid number of command line arguments.\nUsage: python3 plot.py <run num>\nExiting...")

    print_header(f"Plotting map: {GLB_MAP}, run number: {run_num}")
    total_sims, metadata, floor_map, configurations, graph_points0, state_space0 = load_data(GLB_MAP, run_num, 0)
    total_sims, metadata, floor_map, configurations, graph_points1, state_space1 = load_data(GLB_MAP, run_num, 1)
    total_sims, metadata, floor_map, configurations, graph_points2, state_space2 = load_data(GLB_MAP, run_num, 2)
    expected_traj_len = metadata['simsteps']

    # Plot state space map
    ssfig, ss_ax = plt.subplots()
    plot_ss(ssfig, ss_ax, state_space0, metadata['drange'], expected_traj_len, needs_setup=True, color='b')
    plot_ss(ssfig, ss_ax, state_space1, metadata['drange'], expected_traj_len, color='orange', linestyle='-') #linestyle='--'
    plot_ss(ssfig, ss_ax, state_space2, metadata['drange'], expected_traj_len, color='g', linestyle='-') #linestyle='-.'

    # Plot topdown map
    traj_points = [metadata['config-start'], metadata['config-end']]
    mapfig, map_ax = plt.subplots()
    plot_map(mapfig, map_ax, floor_map, desired_traj=traj_points)
    plot_trajectory_on_map(mapfig, map_ax, graph_points0, expected_traj_len, 'b')
    plot_trajectory_on_map(mapfig, map_ax, graph_points1, expected_traj_len, 'orange')
    plot_trajectory_on_map(mapfig, map_ax, graph_points2, expected_traj_len, 'g')
    plot_trajectory_inset(mapfig, map_ax)
    #succ_perc = [perc_successful_trajs(trajs) for trajs in [graph_points0, graph_points1, graph_points2]]
    add_traj_plot_metadata(mapfig, map_ax)

    # For debugging
    #plt.close(ssfig)
    #mapfig.show()
    #plt.close(mapfig)
    #ssfig.show()
    #plt.show()

    #mapfig.savefig(f'./data/plots/map-{GLB_MAP}.png')
    ssfig.savefig(f'./data/plots/ss-{GLB_MAP}.png')


if __name__ == '__main__':
    main()