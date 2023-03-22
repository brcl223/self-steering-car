# ffmpeg -framerate 30 -pattern_type glob -i '/path/to/*.png' -c:v libx264 -pix_fmt yuv420 out.mp4

import habitat_sim
from habitat.utils.visualizations import maps
import numpy as np
import quaternion
import cv2
import random
from PIL import Image
import pickle
from habitat_sim.utils import common as utils
import math
from pathlib import Path
import json

import matplotlib.pyplot as plt


# HM3D Maps
GLB_MAPS = {
    '780': {
        'env': "./data/scene_datasets/train/00780-3iZkJUc7KhX/3iZkJUc7KhX.glb",
        'conf': "./data/scene_datasets/train/hm3d_annotated_basis.scene_dataset_config.json",
    },
    '795': {
        'env': "./data/scene_datasets/train/00795-awcRF7AZnJu/awcRF7AZnJu.glb",
        'conf': "./data/scene_datasets/train/hm3d_annotated_basis.scene_dataset_config.json",
    },
    'minival': {
        'env': "data/scene_datasets/hm3d-minival-habitat-v0.2/00808-y9hTuugGdiq/y9hTuugGdiq.basis.glb",
        'conf': "./data/scene_datasets/hm3d-minival-habitat-v0.2/hm3d_annotated_basis.scene_dataset_config.json",
    }
}


def get_glb_map(glb_map):
    glb_map = glb_map.split('-')[0]
    return GLB_MAPS[glb_map]

# Initial configuration per map, position
IC_p = {
    'minival': np.array([-4.409, -2.74, 1.5]),
    '780': np.array([-1.8068, 0.07731, 2.4038]),
    '795-1': np.array([9.124, -4.523, 0.]),
    '795-2': np.array([-2.4, -4.523, 0.]),
}

# Final desired configuration per map, position
FC_p = {
    '795-1': np.array([-2.4, -4.523, 0.]),
    '795-2': np.array([9.124, -4.523, 0.]),
}

# Initial configuration per map, rotation
IC_r = {
    'minival': quaternion.from_rotation_vector(np.array([0, -math.pi/2, 0])),
    '780': quaternion.from_rotation_vector(np.array([0, math.pi/2, 0])),
    '795-1': quaternion.from_rotation_vector(np.array([0, math.pi/2, 0])),
    '795-2': quaternion.from_rotation_vector(np.array([0, -math.pi/2, 0])),
}

DRANGE = {
    '795-1': (-1.4155, 1.4155),
    '795-2': (-1.4155, 1.4155),
}

# Y-coord isn't needed, but should still be present so preprocessing function works
DESIRED_TRAJ = {
    '795-1': [np.array([9.9735, 0., 0.]), np.array([-2., 0., 0.])],
    '795-2': [np.array([9.9735, 0., 0.]),np.array([-2., 0., 0.])],
}


class Simulation:
    """
    Wrapper class for OpenBot simulation trials
    """
    def __init__(self, glb_map, ctrl_hz=30):
        self.glb_map = glb_map
        self.hallway_env_path = get_glb_map(glb_map)['env']
        self.config_file_path = get_glb_map(glb_map)['conf']
        self.simulation_settings = {
            "width": 640, # agent camera width
            "height": 480, # agent camera height
            "scene": self.hallway_env_path, # sets hallway scene
            "scene_dataset": self.config_file_path, # sets config file
            "default_agent" : 0,
            "sensor_height": 0.3, # camera height above ground
            "color_sensor": True, # agent has rgb cam
            "depth_sensor": True, # agent has depth cam
            "seed": 1,
            "enable_physics": True, # kinematics enabled
        }
        self.cfg = self._create_sim_config(self.simulation_settings) 
        self.sim = habitat_sim.Simulator(self.cfg) # simulation instance created
        self.agent = self.sim.initialize_agent(self.simulation_settings["default_agent"]) # agent instance created
        self.agent_state = habitat_sim.AgentState()
        self.agent_state.position = IC_p[glb_map]
        self.agent_state.rotation = IC_r[glb_map]
        self.agent.set_state(self.agent_state)

        self.sim.config.sim_cfg.allow_sliding = True

        # Timestep for simulation
        self.ctrl_hz = ctrl_hz
        self.dt = 1. / ctrl_hz

        # Velocity Controller
        self.vel_ctrl = habitat_sim.physics.VelocityControl()
        self.vel_ctrl.controlling_lin_vel = True
        self.vel_ctrl.lin_vel_is_local = True
        self.vel_ctrl.controlling_ang_vel = True
        self.vel_ctrl.ang_vel_is_local = True


    def reset_with_ics(self, pos=None, rot=None):
        self.sim.reset()
        if pos is None:
            pos = IC_p[self.glb_map]
        if rot is None:
            rot = IC_r[self.glb_map]
        self.agent_state.position = pos
        self.agent_state.rotation = rot
        self.agent.set_state(self.agent_state)


    def get_agent_configuration(self):
        cur_state = self.agent.get_state()
        return cur_state.position, quaternion.as_rotation_vector(cur_state.rotation)


    def _create_sim_config(self, settings):
        """
        Returns AI Habitat simulation instance.
        """
        sim_cfg = habitat_sim.SimulatorConfiguration()
        sim_cfg.scene_id = settings["scene"]
        agent_cfg = habitat_sim.agent.AgentConfiguration()

        rgb_sensor_spec = habitat_sim.CameraSensorSpec()
        rgb_sensor_spec.uuid = "color_sensor"
        rgb_sensor_spec.sensor_type = habitat_sim.SensorType.COLOR
        rgb_sensor_spec.resolution = [settings["height"], settings["width"]]
        rgb_sensor_spec.position = [0.0, settings["sensor_height"], 0.0]

        depth_sensor_spec = habitat_sim.CameraSensorSpec()
        depth_sensor_spec.uuid = "depth_sensor"
        depth_sensor_spec.sensor_type = habitat_sim.SensorType.DEPTH
        depth_sensor_spec.resolution = [settings["height"], settings["width"]]
        depth_sensor_spec.position = [0.0, settings["sensor_height"], 0.0]

        agent_cfg.sensor_specifications = [rgb_sensor_spec, depth_sensor_spec]
        return habitat_sim.Configuration(sim_cfg, [agent_cfg])


    def get_img(self, depth=True, rgb=False):
        if not depth and not rgb:
            raise RuntimeError("Must select one (or both) of depth or rgb!")

        out = []
        observations = self.sim.get_sensor_observations()

        MAGIC_NUM = 6553.6

        if depth:
            depth_img = observations['depth_sensor']
            #print(f"Stats on depth [Raw]: max: {np.amax(depth_img)}\nmin: {np.amin(depth_img)}")
            depth_output = np.array(np.clip(depth_img, 0., 10.) * MAGIC_NUM).astype(np.uint16)
            #print(f"Stats on depth [Z16]: max: {np.amax(depth_output)}\nmin: {np.amin(depth_output)}")
            depth_img = Image.fromarray((depth_img / 10 * 255).astype(np.uint8), mode="L")
            depth_img = depth_img.convert('RGB')
            depth_img = np.array(depth_img)
            depth_img = cv2.cvtColor(depth_img, cv2.COLOR_BGR2GRAY)
            out.append(depth_output)
            out.append(depth_img)

        if rgb:
            rgb_img = observations['color_sensor']
            rgb_img = Image.fromarray(rgb_img, mode="RGBA")
            rgb_img = np.array(rgb_img)
            out.append(rgb_img)

        return out


    def advance_sim_with_velocity_ctrl(self, v, w):
        # Forward velocity in -z direction
        self.vel_ctrl.linear_velocity = np.array([0, 0, -v])
        # Angular velocity in y direction
        self.vel_ctrl.angular_velocity = np.array([0, w, 0])

        # Physics sim
        agent_state = self.agent.state
        previous_rigid_state = habitat_sim.RigidState(
            utils.quat_to_magnum(agent_state.rotation), agent_state.position
        )

        # Manually integrate rigid state
        target_rigid_state = self.vel_ctrl.integrate_transform(
            self.dt, previous_rigid_state
        )

        # Find end position after velocity integration
        end_pos = self.sim.step_filter(
            previous_rigid_state.translation, target_rigid_state.translation
        )

        # Set computed state
        self.agent_state.position = end_pos
        self.agent_state.rotation = utils.quat_from_magnum(
            target_rigid_state.rotation
        )
        self.agent.set_state(self.agent_state)

        # Check if collision occured
        dist_moved_before_filter = (
            target_rigid_state.translation - previous_rigid_state.translation
        ).dot()
        dist_moved_after_filter = (
            end_pos - previous_rigid_state.translation
        ).dot()

        # Just used for numerical robustness
        EPS = 1e-8
        collided = (dist_moved_after_filter + EPS) < dist_moved_before_filter

        # Advance the simulation
        self.sim.step_physics(self.dt)

        return collided


def vctrl(sensor_based_score_function, depth_img, alpha=1, beta=1, gamma=1):
    F = sensor_based_score_function.decision_function(depth_img.reshape(1,-1))[0]
    #print(f"Value of F: {F}")
    v = beta * math.exp(-alpha * F**2)
    w = -gamma * F
    return v, w


# Load SVM from sklearn 0.24.2
def load_sensor_based_score_function(fname="./models/svc1.pickle"):
    with open(fname, 'rb') as f:
        model = pickle.load(f)
    return model



def print_header(s):
    print("##################################################")
    print(f"{s}")
    print("##################################################\n")


def display_img(depth=None, rgb=None, waitkey=1):
    if depth is None and rgb is None:
        raise RuntimeError("Must display one (or both) of color or depth!")

    if depth is not None and rgb is not None:
        # Make the grey scale image have three channels
        depth_rgba = cv2.cvtColor(depth, cv2.COLOR_GRAY2BGRA)
        img = np.vstack((rgb, depth_rgba))
    elif depth is not None:
        img = depth
    else:
        img = rgb

    WINDOW_NAME="Image Display"
    cv2.imshow(WINDOW_NAME, img)
    cv2.waitKey(waitkey)


def sample_X0(drange=(-1,1)):
    theta = np.random.uniform(low=-math.pi/2, high=math.pi/2)
    d_candidate = np.random.uniform(low=drange[0], high=drange[1])
    d_min = -drange[0]/-(math.pi/2) * theta + drange[0]
    d_max = -drange[1]/(math.pi/2) * theta + drange[1]
    assert d_min < 0 and d_max > 0, "[sample_X0]: d range calculations invalid!"
    d = clamp(d_candidate, d_min, d_max)
    return theta, d


def clamp(num, min_value, max_value):
   return max(min(num, max_value), min_value)


##########################################################
# Taken from the following tutorial:
# https://colab.research.google.com/github/facebookresearch/habitat-sim/blob/main/examples/tutorials/colabs/ECCV_2020_Navigation.ipynb#scrollTo=mY-75WXw918t
##########################################################
# convert 3d points to 2d topdown coordinates
def convert_points_to_topdown(pathfinder, points, meters_per_pixel):
    points_topdown = []
    bounds = pathfinder.get_bounds()
    for point in points:
        # convert 3D x,z to topdown x,y
        #print(f"point[0]={point[0]} (type: {type(point[0])})")
        #print(f"point[2]={point[2]} (type: {type(point[2])})")
        #print(f"bounds[0][0]={bounds[0][0]} (type: {type(bounds[0][0])})")
        #print(f"bounds[0][2]={bounds[0][2]} (type: {type(bounds[0][2])})")
        px = (point[0] - bounds[0][0]) / meters_per_pixel
        py = (point[2] - bounds[0][2]) / meters_per_pixel
        points_topdown.append(np.array([px, py]))
    return points_topdown


# Signed distance between point and line calculates d
def calc_ss_d_var(P1, P2, x):
    return -((P2[0] - P1[0])*(P1[1] - x[1]) - (P1[0] - x[0])*(P2[1] - P1[1])) / math.sqrt((P2[0] - P1[0])**2 + (P2[1] - P1[1])**2)


def create_map_plot(sim, meters_per_pixel, height=None):
    # Map key
    if not sim.sim.pathfinder.is_loaded:
        raise RuntimeError("Pathfinder module not loaded!")

    if height is None:
        height = sim.sim.pathfinder.get_bounds()[0][1]
    #print(f"Bounds: {sim.sim.pathfinder.get_bounds()}")

    #sim_topdown_map = sim.sim.pathfinder.get_topdown_view(meters_per_pixel, height)
    #display_map(sim_topdown_map)
    hablab_topdown_map = maps.get_topdown_map(
        sim.sim.pathfinder, height, meters_per_pixel=meters_per_pixel
    )
    recolor_map = np.array(
        [[255, 255, 255], [128, 128, 128], [0, 0, 0]], dtype=np.uint8
    )
    hablab_topdown_map = recolor_map[hablab_topdown_map]
    return hablab_topdown_map


def get_new_data_path(glb_map):
    data_dir_start_str = f"./data/experiments/{glb_map}"
    Path(data_dir_start_str).mkdir(parents=True, exist_ok=True)
    LARGE_NUM = 10000000
    run_num = None
    for i in range(LARGE_NUM):
        data_dir_candidate = f"{data_dir_start_str}/run-{i}/"
        data_path = Path(data_dir_candidate)
        if data_path.exists():
            continue
        else:
            run_num = i
            #data_path.mkdir(parents=False, exist_ok=False)
            break
    return data_dir_candidate, run_num


def create_data_path(data_path):
    Path(data_path).mkdir(parents=True, exist_ok=True)


def save_metadata(data_dir, metadata):
    fname = f"{data_dir}/metadata.json"
    datastr = json.dumps(metadata)
    with open(fname, 'w') as f:
        f.write(datastr)


def calc_bounded_d(theta, d_cand, dbound, m):
    return m(d_cand, -dbound/(math.pi/2) * theta + dbound)


def ics_are_identical(ic1, ic2, eps=1e-6):
    if abs(ic1[0] - ic2[0]) < eps and abs(ic1[1] - ic2[1]) < eps:
        return True
    return False


def generate_ics2(drange=(-1,1), tmax=math.pi/2, GRID_SIZE=2, eps=0.2):
    # Don't care about where we start, just start
    thetas = np.linspace(start=-tmax+eps, stop=tmax-eps, num=GRID_SIZE)
    ds = np.linspace(start=drange[0]+eps, stop=drange[1]-eps, num=GRID_SIZE)
    out = [(t,d) for t in thetas for d in ds]
    return out


def generate_ics(drange=(-1,1), GRID_SIZE=2):
    ics = []
    eps = 0.1 # to add / sub from range to make sure we stay within our bounds
    thetas = np.linspace(start=-math.pi/2+eps, stop=math.pi/2-eps, num=GRID_SIZE)
    d_candidates = np.linspace(start=drange[0]+eps, stop=drange[1]-eps, num=GRID_SIZE)

    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            t = thetas[i]
            d_min = drange[0]/(math.pi/2) * t + drange[0]
            d_max = -drange[1]/(math.pi/2) * t + drange[1]
            print(f"Min: {d_min}, d: {d_candidates[j]}, max: {d_max}")
            assert d_min <= 0 and d_max >= 0, "[generate_ics]: d range calculations invalid!"
            d = clamp(d_candidates[j], d_min, d_max)
            print(f"theta: {t}, d: {d}")
            ics.append((t, d))

    # Get rid of extraneous ICs
    elems_to_remove = []
    for i in range(len(ics)-1):
        for j in range(i+1,len(ics)):
            if ics_are_identical(ics[i], ics[j]) and i not in elems_to_remove:
                elems_to_remove.append(i)

    print(f"Removing {len(elems_to_remove)} elements from ICs list...")
    elems_to_remove.reverse()
    for i in elems_to_remove:
        del ics[i]

    return ics


def ics_for_video():
    return [(math.pi/3, 0.5)]



def save_video_frame(path, i, np_arr):
    img = cv2.cvtColor(np_arr, cv2.COLOR_GRAY2BGRA)
    fname = f"{path}/{i:04}.png"
    cv2.imwrite(fname, img)



NUM_SIMS = 25
SIM_TIMESTEPS = 2000
CTRL_LOOP_HZ = 100
METERS_PER_PIXEL = 0.05
DISPLAY_SIM = False
RECORD=True

def main():
    glb_map = '795-2'
    data_dir, run_num = get_new_data_path(glb_map)

    #ics = generate_ics(drange=DRANGE[glb_map])
    #ics = generate_ics2(drange=DRANGE[glb_map], tmax=math.pi/3, GRID_SIZE=3)
    ics = ics_for_video()

    print_header("Loading simulation...")
    sim = Simulation(glb_map, ctrl_hz=CTRL_LOOP_HZ)
    print_header("Loading SVM model...")
    sebsf = load_sensor_based_score_function()
    print_header(f"Beginning simulation...\nMAP: {glb_map}\nRun Number: {run_num}")

    # Hyperparams
    hyperparams = [
        {
            'alpha': 0.00005,
            'beta': 0.5,
            'gamma': 0.0001,
        },
        {
            'alpha': 0.00005,
            'beta': 0.5,
            'gamma': 0.001,
        },
        {
            'alpha': 0.00005,
            'beta': 0.5,
            'gamma': 0.01,
        },
        {
            'alpha': 0.00005,
            'beta': 0.5,
            'gamma': 0.1,
        },
    ]

    # Only need [x,z] coords
    P1 = IC_p[glb_map][0:3:2]
    P2 = FC_p[glb_map][0:3:2]
    T = quaternion.as_rotation_vector(IC_r[glb_map])[1]

    start, end = convert_points_to_topdown(sim.sim.pathfinder, DESIRED_TRAJ[glb_map], METERS_PER_PIXEL)

    metadata = {
        'map': glb_map,
        'config-start': start.tolist(),
        'config-end': end.tolist(),
        'hyperparams': hyperparams,
        'ctrl-loop-freq': CTRL_LOOP_HZ,
        'simsteps': SIM_TIMESTEPS,
        'drange': DRANGE[glb_map],
    }

    #############################################
    # Begin simulation
    #############################################
    # Iterate through all hyper-parameters
    for hi, params in enumerate(hyperparams):
        alpha = params['alpha']
        beta = params['beta']
        gamma = params['gamma']

        print_header(f"Beginning Simulation with Hyperparameter Set {hi+1} of {len(hyperparams)}...")

        for n in range(len(ics)):
            if RECORD:
                video_data_path = f"{data_dir}/hy-{hi}-sim-{n}-videoframes/"
                create_data_path(video_data_path)
            print_header(f"Beginning simulation {n+1} of {len(ics)}")
            # Sample uniformly and reset simulator
            #theta, d = sample_X0(drange=DRANGE[glb_map])

            theta, d = ics[n]
            pos = IC_p[glb_map]
            rot = quaternion.as_rotation_vector(IC_r[glb_map])
            pos[2] = d
            rot[1] += theta
            rot = quaternion.from_rotation_vector(rot)
            sim.reset_with_ics(pos=pos, rot=rot)

            configurations = []
            configurations.append(sim.get_agent_configuration())

            # Simulate for SIM_TIMESTEPS
            print("Starting simulation...")
            for t in range(SIM_TIMESTEPS):
                if (t+1) % 100 == 0:
                    print(f"Currently on timestep {t+1} of {SIM_TIMESTEPS}")
                z16_depth, d, r = sim.get_img(depth=True, rgb=True)
                v,w = vctrl(sebsf, z16_depth, alpha=alpha, beta=beta, gamma=gamma)
                collision = sim.advance_sim_with_velocity_ctrl(v,w)
                configurations.append(sim.get_agent_configuration())
                if RECORD:
                    #np.save(f"{video_data_path}/frame-{t}.npy", d)
                    save_video_frame(video_data_path, t, d)
                if DISPLAY_SIM:
                    display_img(depth=d, rgb=r)
                if collision:
                    # If we collide, end the simulation, save the data, and move to the next
                    break
                    #raise RuntimeError("Don't know how to deal with collisions yet")

            print("Beginning post-processing...")
            # Post simulation, process data for graphing over map
            positions = [pos for pos, _rot in configurations]
            graph_data = convert_points_to_topdown(sim.sim.pathfinder, positions, METERS_PER_PIXEL)
            ss_data = [(rot[1] - T, pos[2]) for pos, rot in configurations]

            print("Saving data...")
            # Create data directory before saving
            create_data_path(data_dir)
            # Save our metadata
            save_metadata(data_dir, metadata)
            # Next, get map of hallway and save it
            floor_map = create_map_plot(sim, METERS_PER_PIXEL)
            np.save(f"{data_dir}/floor_map.npy", floor_map)
            # Finally, save the data
            np.save(f"{data_dir}/hy-{hi}-sim-{n}-configurations.npy", configurations)
            np.save(f"{data_dir}/hy-{hi}-sim-{n}-graph-points.npy", graph_data)
            np.save(f"{data_dir}/hy-{hi}-sim-{n}-ss.npy", ss_data)

    # Destroy sim windows if no longer needed
    if DISPLAY_SIM:
        cv2.destroyAllWindows() # close all windows
        cv2.waitKey(1) # Helps ensure we close even if all windows closed


if __name__ == '__main__':
    main()