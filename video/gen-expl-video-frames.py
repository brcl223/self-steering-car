import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import json
import math
from PIL import Image


RUN_NUM=11
DEBUG=False
DATA_PATH="./output/expl-frames/"

Path(DATA_PATH).mkdir(exist_ok=True, parents=True)

# TODO:
# - Check aspect ratio of box, should be 1920x1080 => (16:9)

# Task 1: Draw overhead map on frame
# Task 2: Draw robot and line on frame
# Task 3: Draw d-theta on frame
# Task 4: Draw inset graph on frame
# Task 5: Animate car driving
#   Subtask: Get trajectory of car in (s,\theta,d) space
#   Subtask: Make fade functions of time
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

    #print(f"Image Size: {numcols} x {numrows}")

    ax.plot(0,0,marker='x',markersize=14,color='k')
    ax.plot(numcols,0,marker='x',markersize=14,color='r')
    ax.plot(0,numrows,marker='x',markersize=14,color='g')
    ax.plot(numcols,numrows,marker='x',markersize=14,color='b')

    # Disable params
    ax.tick_params(
        bottom=False,      # ticks along the bottom edge are off
        top=False,         # ticks along the top edge are off
        left=False,
        right=False,
        labelleft=False,
        labelbottom=False) # labels along the bottom edge are off

    return int(numcols), int(numrows)



def plot_straight_line(fig, ax, alpha=1.0):
    X = [22, 268]
    Y = [78, 78]
    ax.plot(X,Y,color='y',lw=5,alpha=alpha)


def plot_square(fig, ax, ps, color='k', lw=2, alpha=1):
    assert len(ps) == 4, f"Invalid number of points for a square!\nProvided: {len(ps)}, expected 4."

    ps.append(ps[0])
    X = [x for x,y in ps]
    Y = [y for x,y in ps]

    ax.plot(X,Y,color=color,lw=lw,alpha=alpha)


def half_extent_to_plot_space(o, e, affmap=None):
    out = []
    # X1, Y1
    out.append( np.array([o[0]-e[0], o[1]+e[1]]) )
    # X2, Y2
    out.append( np.array([o[0]+e[0], o[1]+e[1]]) )
    # X3, Y3
    out.append( np.array([o[0]+e[0], o[1]-e[1]]) )
    # X4, Y4
    out.append( np.array([o[0]-e[0], o[1]-e[1]]) )

    if affmap is not None:
        for i in range(len(out)):
            x = np.array([out[i][0], out[i][1], 1])
            mmul = affmap @ x
            out[i] = mmul[0:2]
    return out


def R2d(theta):
    r = np.deg2rad(theta)
    return np.array([
        [np.cos(r), -np.sin(r)],
        [np.sin(r), np.cos(r)]
    ])


def affine(d, theta):
    bottom_vec = np.array([0,0,1]).reshape(1,-1)
    return np.vstack(
        (np.hstack((R2d(theta), d.reshape(-1,1))),
         bottom_vec))


def plot_robot(fig, ax, o=np.array([0,0]), r=0, alpha=1):
    # All shapes: Half extent squares
    # Params:
    wr, hr = 5,5
    wh, hh = 1,3
    wt, ht = 6,1

    # BODY
    # body_o = np.copy(o)
    body_o = np.array([0,0])
    body_e = np.array([wr, -hr])

    # HEAD
    head_o = body_o + np.array([hr + wh, 0])
    head_e = np.array([wh, -hh])

    # TRACKS
    track1_o = body_o + np.array([0, hr+ht])
    track2_o = body_o + np.array([0, -(hr+ht)])
    track_e = np.array([wt, -ht])

    # Draw body
    plot_square(fig, ax, half_extent_to_plot_space(body_o, body_e, affmap=affine(o,r)), alpha=alpha)
    # Draw head
    plot_square(fig, ax, half_extent_to_plot_space(head_o, head_e, affmap=affine(o,r)), alpha=alpha)
    # Plot tracks
    plot_square(fig, ax, half_extent_to_plot_space(track1_o, track_e, affmap=affine(o,r)), alpha=alpha)
    plot_square(fig, ax, half_extent_to_plot_space(track2_o, track_e, affmap=affine(o,r)), alpha=alpha)


def plot_d(fig, ax, o=np.array([0,0]), r=0, plot_text=False, alpha=1):

    # Plot d line
    d_o_y =  78
    d_e_y = o[1]
    d_x = o[0]
    head_width = min(3, abs(d_e_y-d_o_y))
    ax.arrow(d_x, d_o_y, 0, d_e_y-d_o_y, color='r', linestyle='--', width=1, head_width=head_width, zorder=5, length_includes_head=True, alpha=alpha)

    if plot_text:
        ax.text(d_x-10, 0.5*(d_o_y + d_e_y)+3, "d", color='r', fontsize=40, alpha=alpha)

def plot_theta(fig, ax, o=np.array([0,0]), r=0, plot_text=False, alpha=1):
    # Plot theta line
    # Plot straight line through robot
    affmap = affine(o, r)
    linelen = 95
    t_o = affmap @ np.array([-linelen, 0, 1])
    t_e = affmap @ np.array([linelen, 0, 1])
    # Rotate by r degrees and shift to origin on bot

    X = [t_o[0], t_e[0]]
    Y = [t_o[1], t_e[1]]
    ax.plot(X, Y, color='b', linewidth=3, linestyle='--', zorder=5, alpha=alpha)

    if plot_text:
        ax.text(131, 80, r'$\theta$', color='b', fontsize=40, alpha=alpha)


def gen_inset_img_frame(fig, ax, extents=np.array([0.1, 0.1, 0.1, 0.2])):
    axins = ax.inset_axes(extents, zorder=10)

    # Disable params
    axins.tick_params(
        bottom=False,      # ticks along the bottom edge are off
        top=False,         # ticks along the top edge are off
        left=False,
        right=False,
        labelleft=False,
        labelbottom=False) # labels along the bottom edge are off

    return axins



def gen_inset_ss_frame(fig, ax):
    axins = ax.inset_axes([0.2, 0.25, 0.25, 0.5], zorder=10)
    axins.set_alpha(1)

    AXISFONTSIZE=10
    axins.set_xlim([-math.pi/2, math.pi/2])
    axins.set_ylim([-1,1])
    axins.grid(True, which='both')
    axins.axhline(y=0, color='k')
    axins.axvline(x=0, color='k')
    axins.set_ylabel(r'$d$', fontsize=AXISFONTSIZE)
    axins.set_xlabel(r'$\theta$', fontsize=AXISFONTSIZE)

    return axins



def plot_d_theta_line_on_inset(fig, axins):
    # Draw d line
    axins.plot([-math.pi, math.pi], [0.5, 0.5], color='r', lw=4)
    # Draw \theta line
    axins.plot([0.75, 0.75], [-1.05, 1.05], color='b', lw=4)




def shrink_map(fig, ax, o, e):
    # xmin, xmax = 14,154
    # ymin, ymax = 111,38.5
    ps = half_extent_to_plot_space(o, e)
    xmin, xmax = ps[0][0], ps[2][0]
    ymin, ymax = ps[0][1], ps[2][1]
    ax.set_xlim([xmin, xmax])
    ax.set_ylim([ymin,ymax])


def gen_ss_points(ic, T, dt=1/30, alpha=5e-5, beta=1, gamma=1):
    out = np.zeros((len(ic), T))
    out[:,0] = ic
    for t in range(T-1):
        s, theta, d = out[:,t]
        f = -theta - d
        v = beta * math.exp(-alpha * f**2)
        w = gamma * f
        s_next = s + v * math.cos(theta)*dt
        theta_next = theta + w*dt
        d_next = d + v * math.sin(theta)*dt
        out[:,t+1] = [s_next, theta_next, d_next]

    return out



def plot_ss_traj_on_inset(fig, axins, ss_points):
    T = ss_points[1,:]
    D = ss_points[2,:]
    axins.plot(T, D, marker='o', markersize=3, color='b')


def outside_valid_time(t, Tf, Ts=0):
    return t < Ts or t >= Tf


def interpolate(x1: float, x2: float, y1: float, y2: float, x: float):
    """Perform linear interpolation for x between (x1,y1) and (x2,y2) """

    return ((y2 - y1) * x + x2 * y1 - x1 * y2) / (x2 - x1)


def filename(t):
    return f"{DATA_PATH}/{t:03}.png"


# Just draw the map
def phase1(t, T):
    if outside_valid_time(t, T):
        return
    metadata, floor_map, configurations, graph_points, state_space = load_data()
    fig, ax = plt.subplots(figsize=(16, 9), dpi=1920/16)
    plot_map(fig, ax, floor_map)

    fig.tight_layout()

    # Debugging
    if DEBUG:
        fig.show()
        plt.show()
    else:
        fig.savefig(filename(t))


# Draw the map and fade the line in
def phase2(t, Ts, Tf, frames_to_full_fade=10):
    if outside_valid_time(t, Tf, Ts=Ts):
        return

    metadata, floor_map, configurations, graph_points, state_space = load_data()
    fig, ax = plt.subplots(figsize=(16, 9), dpi=1920/16)
    plot_map(fig, ax, floor_map)

    line_alpha = lambda x: min(0.8, (x-Ts)/frames_to_full_fade)
    plot_straight_line(fig, ax, alpha=line_alpha(t))

    fig.tight_layout()

    # Debugging
    if DEBUG:
        fig.show()
        plt.show()
    else:
        fig.savefig(filename(t))



# Zoom into the robot
def phase3(t, Ts, Tf):
    if outside_valid_time(t, Tf, Ts=Ts):
        return

    metadata, floor_map, configurations, graph_points, state_space = load_data()
    fig, ax = plt.subplots(figsize=(16, 9), dpi=1920/16)
    numcols, numrows = plot_map(fig, ax, floor_map)

    plot_straight_line(fig, ax, alpha=0.8)

    # LERP to zoom position
    frame_o = np.array([82,75])
    frame_e = np.array([68,31])

    xmin = frame_o[0] - frame_e[0]
    xmax = frame_o[0] + frame_e[0]
    ymin = frame_o[1] - frame_e[1]
    ymax = frame_o[1] + frame_e[1]

    zoom_finish = Tf - Ts
    cur_i = t - Ts
    xmax_cur = interpolate(0, zoom_finish, numcols, xmax, cur_i)
    xmin_cur = interpolate(0, zoom_finish, 0, xmin, cur_i)
    ymax_cur = interpolate(0, zoom_finish, numrows, ymax, cur_i)
    ymin_cur = interpolate(0, zoom_finish, 0, ymin, cur_i)

    print(f"xmax: {xmax_cur}, xmin: {xmin_cur}\nymax:{ymax_cur}, ymin: {ymin_cur}")

    ax.set_xlim([xmin_cur, xmax_cur])
    ax.set_ylim([ymax_cur, ymin_cur])

    fig.tight_layout()

    # Debugging
    if DEBUG:
        fig.show()
        plt.show()
    else:
        fig.savefig(filename(t))



# Draw robot + d + \theta lines
def phase4(t, Ts, Tf, fade_frames=5, theta_delay_frames=5, d_delay_frames=5):
    if outside_valid_time(t, Tf, Ts=Ts):
        return

    d_delay_frames += theta_delay_frames
    assert ((Tf - Ts) > d_delay_frames), "Not enough frames for d-theta transition"

    metadata, floor_map, configurations, graph_points, state_space = load_data()
    fig, ax = plt.subplots(figsize=(16, 9), dpi=1920/16)
    numcols, numrows = plot_map(fig, ax, floor_map)

    plot_straight_line(fig, ax, alpha=0.8)

    robot_o = np.array([100, 60])
    robot_r = 45

    robot_alpha = lambda x: min(1, (x-Ts)/fade_frames)
    theta_alpha = lambda x: min(1, (x-Ts+theta_delay_frames)/fade_frames)
    d_alpha = lambda x: min(1, (x-Ts+d_delay_frames)/fade_frames)

    plot_robot(fig, ax, o=robot_o, r=robot_r, alpha=robot_alpha(t))

    if t - Ts > theta_delay_frames:
        plot_theta(fig, ax, o=robot_o, r=robot_r, plot_text=True, alpha=theta_alpha(t))
    if t - Ts > d_delay_frames:
        plot_d(fig, ax, o=robot_o, r=robot_r, plot_text=True, alpha=d_alpha(t))

    frame_o = np.array([82,75])
    frame_e = np.array([68,31])
    shrink_map(fig, ax, frame_o, frame_e)

    fig.tight_layout()

    # Debugging
    if DEBUG:
        fig.show()
        plt.show()
    else:
        fig.savefig(filename(t))


# \tilde F
def phase5(t, Ts, Tf):
    if outside_valid_time(t, Tf, Ts=Ts):
        return

    metadata, floor_map, configurations, graph_points, state_space = load_data()
    fig, ax = plt.subplots(figsize=(16, 9), dpi=1920/16)
    numcols, numrows = plot_map(fig, ax, floor_map)

    plot_straight_line(fig, ax, alpha=0.8)

    robot_o = np.array([100, 60])
    robot_r = 45

    plot_robot(fig, ax, o=robot_o, r=robot_r)
    plot_theta(fig, ax, o=robot_o, r=robot_r, plot_text=True)
    plot_d(fig, ax, o=robot_o, r=robot_r, plot_text=True)

    #axins = gen_inset_ss_frame(fig, ax)
    axins = gen_inset_img_frame(fig, ax, extents=np.array([0.1, 0.1, 0.15, 0.3]))
    # Load image
    im = Image.open('./frames/hy-1-sim-0-videoframes/0000.png')
    axins.imshow(im)

    # Plot image block
    img_x, img_y = 40, 93
    arrow_y = img_y - 3
    # ax.text(img_x, img_y, r'Image', size=20, bbox=dict(boxstyle="round",ec=(1,1,1),fc=(1,1,1)))

    ax.arrow(50,arrow_y,20,0,width=1,head_width=3,length_includes_head=True)

    # Plot \tilde F block
    f_x, f_y = 75, img_y
    ax.text(f_x, f_y, r'$\tilde F$', size=40, bbox=dict(boxstyle="round",ec=(1,1,1),fc=(1,1,1)),zorder=50)

    ax.arrow(85,arrow_y,20,0,width=1,head_width=3,length_includes_head=True)

    # Plot [v,w]^\intercal block
    vw_x, vw_y = 110, f_y
    ax.text(vw_x, vw_y, r'$[v,\omega]^\intercal$', size=40, bbox=dict(boxstyle="round",ec=(1,1,1),fc=(1,1,1)),zorder=50)

    frame_o = np.array([82,75])
    frame_e = np.array([68,31])
    shrink_map(fig, ax, frame_o, frame_e)

    fig.tight_layout()

    # Debugging
    if DEBUG:
        fig.show()
        plt.show()
    else:
        fig.savefig(filename(t))


# Plot \theta-d straight lines on plot
def phase6(t, Ts, Tf):
    if outside_valid_time(t, Tf, Ts=Ts):
        return

    robot_o = np.array([100, 60])
    robot_r = 45
    liney = 78

    D_SCALE=30
    S_SCALE=4

    ic = [0, np.deg2rad(robot_r), (robot_o[1]-liney)/D_SCALE]
    timesteps = 1 + (t-Ts)
    ss_points = gen_ss_points(ic, timesteps)

    last = np.copy(ss_points[:,-1])
    ds = last[0:3:2]
    dr = np.rad2deg(last[1])

    ds[0] *= S_SCALE
    ds[1] *= D_SCALE

    inset_o = np.array([50, 78]) + [ds[0], 0]
    inset_e = np.array([10, 10])
    frame_o = np.array([82,75]) + [ds[0], 0]
    frame_e = np.array([68,31])

    robot_o[0] += ds[0]
    robot_o[1] = liney + ds[1]
    robot_r = dr

    metadata, floor_map, configurations, graph_points, state_space = load_data()
    fig, ax = plt.subplots(figsize=(16, 9), dpi=1920/16)
    numcols, numrows = plot_map(fig, ax, floor_map)

    plot_straight_line(fig, ax, alpha=0.8)

    plot_robot(fig, ax, o=robot_o, r=robot_r)
    plot_theta(fig, ax, o=robot_o, r=robot_r, plot_text=False)
    plot_d(fig, ax, o=robot_o, r=robot_r, plot_text=False)

    axins = gen_inset_ss_frame(fig, ax)

    plot_ss_traj_on_inset(fig, axins, ss_points)

    # frame_o = np.array([82,75])
    # frame_e = np.array([68,31])
    shrink_map(fig, ax, frame_o, frame_e)

    fig.tight_layout()

    # Debugging
    if DEBUG:
        fig.show()
        plt.show()
    else:
        fig.savefig(filename(t))



def main_old():
    metadata, floor_map, configurations, graph_points, state_space = load_data()

    fig, ax = plt.subplots()

    robot_o = np.array([100, 60])
    robot_r = 45
    liney = 78

    ic = [0, np.deg2rad(robot_r), (robot_o[1]-liney)/30]
    timesteps = 1000
    ss_points = gen_ss_points(ic, timesteps)

    ds = ss_points[0:3:2,-1]
    dr = ss_points[1,-1]

    ds[0] *= 2

    inset_o = np.array([50, 78]) + [ds[0], 0]
    inset_e = np.array([10, 10])
    frame_o = np.array([82,75]) + [ds[0], 0]
    frame_e = np.array([68,31])

    robot_o[0] += ds[0]
    robot_o[1] = liney + ds[1]
    robot_r = dr

    numcols, numrows = plot_map(fig, ax, floor_map)
    plot_straight_line(fig, ax)
    plot_robot(fig, ax, o=robot_o, r=robot_r)
    plot_d_theta(fig, ax, o=robot_o, r=robot_r)
    # axins = gen_inset_ss_frame(fig, ax, inset_o, inset_e, numcols, numrows)
    axins = gen_inset_ss_frame(fig, ax)
    # plot_d_theta_line_on_inset(fig, axins)
    plot_ss_traj_on_inset(fig, axins, ss_points)
    shrink_map(fig, ax, frame_o, frame_e)


    fig.show()
    plt.show()



# FPS: 30
FPS = 30
if __name__ == '__main__':
    T_P1 = FPS * 2 # total frames
    T_P2 = T_P1 + FPS * 2
    T_P3 = T_P2 + FPS * 4
    T_P4 = T_P3 + FPS * 8
    T_P5 = T_P4 + FPS * 8
    T_P6 = T_P5 + FPS * 6
    T_TOT = T_P6

    NUM=800
    # for i in range(NUM, min(NUM+200,T_TOT)):
    for i in range(T_P3, T_P4):
        print(f"Iter {i+1} of {T_TOT}")
        phase1(i, T_P1)
        phase2(i, T_P1, T_P2)
        phase3(i, T_P2, T_P3)
        phase4(i, T_P3, T_P4, fade_frames=10, theta_delay_frames=FPS*3, d_delay_frames=FPS)
        phase5(i, T_P4, T_P5)
        phase6(i, T_P5, T_P6)
