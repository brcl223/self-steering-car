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

# Initial configuration per map, position
IC_p = {
    'minival': np.array([-4.409, -2.74, 1.5]),
    '780': np.array([-1.8068, 0.07731, 2.4038]),
    #'795': np.array([9.124, -4.523, 0.0421]),
    '795': np.array([9.124, -4.523, 0.]),
}

# Final desired configuration per map, position
FC_p = {
    '795': np.array([-2.4, -4.523, 0.]),
}

# Initial configuration per map, rotation
IC_r = {
    'minival': quaternion.from_rotation_vector(np.array([0, -math.pi/2, 0])),
    '780': quaternion.from_rotation_vector(np.array([0, math.pi/2, 0])),
    '795': quaternion.from_rotation_vector(np.array([0, math.pi/2, 0])),
}

DRANGE = {
    '795': (-1.4155, 1.4155),
}


class Simulation:
    """
    Wrapper class for OpenBot simulation trials
    """
    def __init__(self, glb_map, ctrl_hz=30):
        #self.hallway_env_path = "data/scene_datasets/hm3d-minival-habitat-v0.2/00808-y9hTuugGdiq/y9hTuugGdiq.basis.glb"
        #self.config_file_path = "./data/scene_datasets/hm3d-minival-habitat-v0.2/hm3d_annotated_basis.scene_dataset_config.json"
        #self.hallway_env_path = "./data/scene_datasets/train/00780-3iZkJUc7KhX/3iZkJUc7KhX.glb"
        #self.config_file_path = "./data/scene_datasets/train/hm3d_annotated_basis.scene_dataset_config.json"
        self.glb_map = glb_map
        self.hallway_env_path = GLB_MAPS[glb_map]['env']
        self.config_file_path = GLB_MAPS[glb_map]['conf']
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
        #self.agent_state.position = np.array([2.4347131,-2.6599462,1.5344026]) # initial position of agent
        #self.agent_state.rotation = quaternion.quaternion(-0.642787933349609, 0, -0.766044139862061, 0) # initial orientation of agent
        #self.agent_state.rotation = quaternion.from_rotation_vector([0,0,0])
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


    def set_state(self, pos, rot=[0,0,0]):
        self.agent_state.position = np.array(pos)
        self.agent_state.rotation = quaternion.from_rotation_vector(rot)
        self.agent.set_state(self.agent_state)

        # Return whether our new state produces a collision
        #obs = self.sim.get_sensor_observations()
        obs = self.sim.step("move_forward")
        return obs["collided"]


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
        EPS = 1e-5
        collided = (dist_moved_after_filter + EPS) < dist_moved_before_filter

        # Advance the simulation
        self.sim.step_physics(self.dt)

        return collided


def vctrl(sensor_based_score_function, depth_img, alpha=1, beta=1, gamma=1):
    F = sensor_based_score_function.decision_function(depth_img.reshape(1,-1))[0]
    #print(f"Value of F: {F}")
    v = beta * math.exp(-alpha * F**2)
    #w = -gamma * F
    w = gamma * F
    return v, w


def print_score_function(sebsf, depth_img):
    print(f"Value of score function: {sebsf.decision_function(depth_img.reshape(1,-1))[0]}")


# Load SVM from sklearn 0.24.2
def load_sensor_based_score_function(fname="./models/svc1.pickle"):
    with open(fname, 'rb') as f:
        model = pickle.load(f)
    return model



def print_header(s):
    print("##################################################")
    print(f"{s}")
    print("##################################################\n")


def display_img(depth=None, rgb=None):
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


# Distance between point and line calculates d
def calc_ss_d_var(P1, P2, x):
    return -((P2[0] - P1[0])*(P1[1] - x[1]) - (P1[0] - x[0])*(P2[1] - P1[1])) / math.sqrt((P2[0] - P1[0])**2 + (P2[1] - P1[1])**2)


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
        px = (point[0] - bounds[0][0]) / meters_per_pixel
        py = (point[2] - bounds[0][2]) / meters_per_pixel
        points_topdown.append(np.array([px, py]))
    return points_topdown


##########################################################
# Taken from the following tutorial:
# https://colab.research.google.com/github/facebookresearch/habitat-sim/blob/main/examples/tutorials/colabs/ECCV_2020_Navigation.ipynb#scrollTo=mY-75WXw918t
##########################################################
# display a topdown map with matplotlib
def display_map(topdown_map, key_points=None):
    plt.figure(figsize=(12, 8))
    ax = plt.subplot(1, 1, 1)
    ax.axis("off")
    plt.imshow(topdown_map)
    # plot points on map
    if key_points is not None:
        for point in key_points:
            plt.plot(point[0], point[1], marker="o", markersize=10, alpha=0.8)
    plt.show(block=False)


DEPTH_WINDOW_NAME = "depth"
RGB_WINDOW_NAME = "rgb"
KEY_D = 100
KEY_A = 97
KEY_W = 119
KEY_P = ord('p')
KEY_G = ord('g')
KEY_R = ord('r')
KEY_S = ord('s')
KEY_U = ord('u')
KEY_M = ord('m')
KEY_F = ord('f')
KEY_ESC = 27
def main():
    print_header("Loading simulation...")
    glb_map = '795'
    sim = Simulation(glb_map, ctrl_hz=100)
    print_header("Simulation loaded!")
    print_header("Loading SVM model...")
    sebsf = load_sensor_based_score_function()
    print_header("SVM Model Loaded!")

    # Hyperparams
    alpha = 0.00005
    beta = 1
    gamma = 0.05

    #############################################
    # Quick exploration code
    #############################################
    # Examine initial state
    key = 0
    sim_started = False
    P = None
    P2 = None
    T = None
    ss_data = []
    frame_ctr = 0
    wait = 0
    while key != KEY_ESC:
        z16_depth, d, r = sim.get_img(depth=True, rgb=True)
        display_img(depth=d,rgb=r)
        key = cv2.waitKey(wait)

        if key == KEY_D:
            sim.sim.step("turn_right")
        elif key == KEY_A:
            sim.sim.step("turn_left")
        elif key == KEY_W:
            sim.sim.step("move_forward")
        elif key == KEY_M:
            # Map key
            if not sim.sim.pathfinder.is_loaded:
                raise RuntimeError("Pathfinder module not loaded!")
            meters_per_pixel = 0.05
            #height = -4.
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
            display_map(hablab_topdown_map)
        elif key == KEY_F:
            print_score_function(sebsf, z16_depth)
        elif key == KEY_P:
            # Print agent state
            state = sim.agent.get_state()
            cur_t = quaternion.as_rotation_vector(state.rotation)
            print(f"Agent position: {state.position}")
            print(f"Agent rotation: {cur_t}")
            print(f"State: d={-state.position[2]}, theta={-cur_t[1] + math.pi/2}")
        elif key == KEY_R:
            # Reset simulator
            sim.reset_with_ics()
            state = sim.agent.get_state()
            print(f"Agent position after reset: {state.position}")
            print(f"Agent rotation after reset: {quaternion.as_rotation_vector(state.rotation)}")
        elif key == KEY_U:
            # Sample uniformly and reset simulator
            theta, d = sample_X0(drange=DRANGE[glb_map])
            pos = IC_p[glb_map]
            rot = quaternion.as_rotation_vector(IC_r[glb_map])
            pos[2] = d
            rot[1] += theta
            rot = quaternion.from_rotation_vector(rot)
            sim.reset_with_ics(pos=pos, rot=rot)
        elif key == KEY_G:
            # GO KEY! - Advance one from from sebsf
            v,w = vctrl(sebsf, z16_depth, alpha=alpha, beta=beta, gamma=gamma)
            print(f"v: {v}\nw: {w}")
            collision = sim.advance_sim_with_velocity_ctrl(v, w)
            if collision:
                print("We collided!")
        elif key == KEY_S and not sim_started:
            # Start our simulation
            sim_started = True
            P = IC_p[glb_map]
            P2 = FC_p[glb_map]
            T = quaternion.as_rotation_vector(IC_r[glb_map])[1] # Only rotate in y direction
            wait = 5 # ms delay to waitKey to allow simulation to progress automatically
        elif key != KEY_S and sim_started:
            # If we don't stop the simulator with the s key while it's running...
            # Calculate SS variables and save
            cur_state = sim.agent.get_state()
            pos = cur_state.position
            rot = quaternion.as_rotation_vector(cur_state.rotation)
            theta = rot[1] - T
            # TODO: Why are these not equal on 795-1?
            d = math.cos(T-math.pi/2)*(P[2] - pos[2]) - math.sin(T-math.pi/2)*(P[0] - pos[0])
            d2 = calc_ss_d_var(P[0:3:2], P2[0:3:2], pos[0:3:2])
            print(f"SS Calculation\nTheta: {theta}\nD: {d}\nD2: {d2}")
            ss_data.append([theta, d])
            frame_ctr += 1

            # Progress the simulation
            v,w = vctrl(sebsf, z16_depth, alpha=alpha, beta=beta, gamma=gamma)
            collision = sim.advance_sim_with_velocity_ctrl(v, w)
            if collision:
                print("We collided!")
        elif key == KEY_S and sim_started:
            print(f"Total simulation frames: {frame_ctr}")
            sim_started = False
            wait = 0
            frame_ctr = 0
            # TODO: Save the ss_data somewhere eventually
        else:
            print(f"Got keystroke: {key}")

    cv2.destroyAllWindows() # close all windows
    cv2.waitKey(1) # Helps ensure we close even if all windows closed


if __name__ == '__main__':
    main()