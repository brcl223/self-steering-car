import pyrealsense2.pyrealsense2 as rs
import numpy as np
import serial # For communicating with Arduino
from time import sleep
import math
import matplotlib.pyplot as plt

#####################################################
# Process Params (i.e. Global Vars)
#####################################################

# CHANGE RUN_NUM BEFORE EACH RUN!
RUN_NUM = 14
# Epsilon value for determining if we are within range
# of our turning target (absolute measure)
# TODO: Tune me
TARGET_EPS = 30
# Epsilon value for determining if we are within range
# of our turning target (relative measure)
# TODO: Tune me
TURNING_EPS = 10

#####################################################
# Code Begins Here
#####################################################
def wrap2pi(x):
    while x < -math.pi:
        x += 2 * math.pi
    while x > math.pi:
        x -= 2 * math.pi
    return x


class DTMap:
    def __init__(self):
        self.data = {}
        self.images = {}

    def add_data(self, data, images):
        assert len(images) == len(data)
        data = np.array(data)
        dlen = data.shape[0]
        num_ext, peaks, valleys = detect_num_extremums(data)
        assert num_ext == 4, "Not enough extremums"
        theta_pi_min = abs(valleys[0] - valleys[1])
        theta_pi_max = abs(peaks[0] - peaks[1])
        print(f"Theta pi min: {theta_pi_min}\nTheta pi max: {theta_pi_max}")
        #assert abs(theta_pi_min - theta_pi_max) < 3, "Data resolution is not fine enough"
        # Average it. Probably need to do better than this though
        theta_pi = 0.5 * (theta_pi_min + theta_pi_max)

        theta_vals = [wrap2pi((i - peaks[0]) / theta_pi * math.pi) for i in range(dlen)]
        print("#######################\n\n")
        v1min = valleys[0]
        v2min = valleys[1]
        d = (data[v1min] - data[v2min]) / (data[v1min] + data[v2min])
        print(f"d: {d}")
        print("\n\n#######################\n\n")
        self.data[d] = theta_vals
        self.images[d] = images

    def plot_data(self):
        print(self.data.items())
        fig, ax = plt.subplots()
        for k,v in self.data.items():
            print(f"Plotting d: {k}")
            y = [k] * len(v)
            ax.scatter(v,y)

        # Plot Left / Right line
        ax.plot([-math.pi, math.pi], [1, -1], linestyle='dashed')

        ax.set(xlabel='Theta', ylabel='d', title='DT Map')
        plt.tight_layout()
        fig.savefig(f"./data/dt-map-{RUN_NUM}.png")
        
    def save_images(self):
        import pathlib
        DATA_LOC = f"./data/run-{RUN_NUM}-imgs/"
        pathlib.Path(f"{DATA_LOC}/left").mkdir(parents=True, exist_ok=True)
        pathlib.Path("{DATA_LOC}/right").mkdir(parents=True, exist_ok=True)

        for d in self.data.keys():
            imgs = self.images[d]
            thetas = self.data[d]
            for i, theta in enumerate(thetas):
                # if x < -y turn right
                if (2 / math.pi * theta) < (-d):
                    fname = f"{DATA_LOC}/right/{d}-{i}.txt"
                # else turn left
                else:
                    fname = f"{DATA_LOC}/left/{d}-{i}.txt"
                np.savetxt(fname, imgs[i])

        
def check_peak(data, idx):
    score = 0

    if data[idx] > data[idx - 1]: score += 1
    if data[idx] > data[idx - 2]: score += 1
    if data[idx] > data[idx + 1]: score += 1
    if data[idx] > data[idx + 2]: score += 1

    if score >= 4:
        return True

    return False


def check_valley(data, idx):
    score = 0

    if data[idx] < data[idx - 1]: score += 1
    if data[idx] < data[idx - 2]: score += 1
    if data[idx] < data[idx + 1]: score += 1
    if data[idx] < data[idx + 2]: score += 1

    if score >= 4:
        return True

    return False


def detect_num_extremums(data):
    smooth = smooth_data(data)
    dlen = smooth.shape[0]
    peaks = valls = 0
    vall_idx = []
    peak_idx = []

    if dlen < 5:
        # We don't have enough data to make a decision yet
        return 0, [], []

    # Simple robust "gradient descent" style peak check
    for i in range(2, dlen - 2):
        if check_peak(smooth, i):
            peaks += 1
            peak_idx.append(i)
        elif check_valley(smooth, i):
            valls += 1
            vall_idx.append(i)

    print(f"Num Peaks: {peaks}\nNum Valleys: {valls}")

    assert abs(peaks - valls) <= 1, "Extremums are out of whack!"

    return peaks + valls, peak_idx, vall_idx


def smooth_data(data):
    data = np.array(data)
    dlen = data.shape[0]

    if dlen < 2:
        # This could cause copy issues, but I don't think
        # it is right now
        return data

    smooth_data = np.zeros(data.shape)
    conv_filt = [1/4, 1/2, 1/4]

    smooth_data[0] = np.dot(conv_filt, [data[0], data[0], data[1]])

    for i in range(1, data.shape[0] - 1):
        smooth_data[i] = np.dot(conv_filt, data[i-1:i+2])

    smooth_data[-1] = np.dot(conv_filt, [data[-2], data[-1], data[-1]])

    return smooth_data


# Builds camel graph
def scan_surroundings(arduino, pipeline):
    data_points = []
    # This shouldn't be here
    #init_arduino(arduino)

    nimages = 3

    seg_data_points = []
    lr_data_points = []
    imgs = []

    NUM_MOVEMENTS = 75
    for j in range(NUM_MOVEMENTS):
        print(f"Beginning movement {j+1} of {NUM_MOVEMENTS}...")
        depth_est, image = get_depth_estimate(pipeline, nimages=nimages, return_image=True)
        data_points.append(depth_est)
        imgs.append(image)

        num_extremums, _b1, _b2 = detect_num_extremums(smooth_data(data_points))
        if num_extremums >= 4:
            print("Found two peaks and two vallies!")
            break

        print("Turning arduino")
        #turn_for_next_img(arduino)
        send_cmd(arduino, 'L', 125, 75)

    np.savetxt(f"./data/run-{RUN_NUM}.txt", data_points)
    np.savetxt(f"./data/run-{RUN_NUM}-smooth.txt", smooth_data(data_points))
    
    return data_points, smooth_data(data_points), imgs



def send_cmd(arduino, motion, speed, duration):
    # Send command
    cmd = f"{motion}.{str(speed)}.{str(duration)}\n"
    arduino.write(bytes(cmd, 'utf-8'))
    #sleep((duration / 1000.0) - 0.001)

    # Await Response
    # TODO: Add timeout here
    while True:
        data = arduino.readline().decode('utf-8')
        #print(f"[DEBUG] Got line from arduino: {data}",end="")
        if "Complete" in data:
            break
        if "ERROR" in data:
            raise Exception(f"Command Error: Invalid command ({cmd})")


# For whatever reason, the first message or so takes a lot longer
# to process than the succeeding calls. So here I just make some
# dummy instructions to drive forwards and back at first so we
# don't have this issue when we begin turning
def init_arduino(arduino):
    print("Initializing Arduino...")
    sleep(2)
    #send_cmd(arduino, 'F', 125, 200)
    #sleep(0.5)
    #send_cmd(arduino, 'B', 125, 200)
    send_cmd(arduino, 'S', '000', 500)
    sleep(2)
    print("Arduino initialized!")


# Computes our singular depth value estimate
# trying to find where the nearest objects are
def get_depth_estimate(pipe, nimages=60, nregions=1, return_imgs=False):
    
    IMG_COUNT = nimages # at 30fps this takes ~2 seconds
    MAX_SKIPPED = 10
    skipped_imgs = 0
    image = None

    depth_total = np.zeros(nregions)

    WIDTH = int(640 / nregions)
    HEIGHT = 480
    IMG_PIXELS = WIDTH * HEIGHT

    print("Beginning depth estimate...")

    for i in range(IMG_COUNT):
        #print(f"Image {i+1} of {IMG_COUNT}...")

        frames = pipe.wait_for_frames()
        depth = frames.get_depth_frame()

        if not depth:
            skipped_imgs += 1
            if skipped_imgs > MAX_SKIPPED:
                raise Exception("Too many skipped images!")
            continue

        data = np.asanyarray(depth.get_data())
        if not image:
            image = np.array(data, copy=True)
        # This is where different depth functions
        # can be tried out. Could be weighted sum
        # or bounding box (or more)
        for j in range(nregions):
            start = WIDTH * j
            end = WIDTH * (j+1)
            depth_total[j] += (data[:,start:end].sum() / IMG_PIXELS)

    # Get average from number of images
    depth_total /= (IMG_COUNT - skipped_imgs)
    print(f"Done!\nDepth Total Estimate: {depth_total}")

    # Just return scalar if one region
    if nregions == 1:
        depth_total = depth_total[0]

    if return_images:
        assert image is not None
        return depth_total, image

    return depth_total



def turn_to_min_angle(arduino, pipe, data):
    global TURNING_EPS

    print("Turning to min angle...")

    target_depth = np.amin(data)
    target_depth_idx = data.argmin()

    depth_est = get_depth_estimate(pipe)
    tries = 0

    print("Initial start:")
    print(f"Target Depth: {target_depth}")
    print(f"Current Depth: {depth_est}")
    cur_dir = "R"
    cur_len = 100
    cur_spd = 105
    ####################
    # Course refinement
    ####################
    EPS_THETA = 0.02 # 2%
    while rel_error(depth_est, target_depth) > EPS_THETA:
        send_cmd(arduino, cur_dir, cur_spd, cur_len)
        depth_est = get_depth_estimate(pipe, nimages=3)
        print(f"Target Depth: {target_depth}")
        print(f"Current Depth: {depth_est}")
        print(f"Rel Err: {100 * rel_error(depth_est, target_depth):.2f}% > {100* EPS_THETA}%")
        tries += 1
        if tries > 200:
            raise Exception("Could not get back to min!")

    print_header("Found course refinement!\nBeginning Fine Refinement")
    ####################
    # Fine refinement
    ####################
    EPS_THETA_REFINED = 0.002 # 0.2% - Very close
    min_target = depth_est
    num_attempts = 0
    while num_attempts < 100:
        print(f"Cur_dir: {cur_dir}")
        # Find minimum target
        send_cmd(arduino, cur_dir, cur_spd, cur_len)
        depth_est = get_depth_estimate(pipe, nimages=10)
        print(f"Current Min Depth: {min_target}")
        print(f"Current Depth: {depth_est}")
        print(f"Rel Err: {100 * rel_error(depth_est, min_target):.2f}% > {100* EPS_THETA_REFINED}%")
        # Keep rotating while we find minimums
        if depth_est < min_target:
            min_target = depth_est
        # This would be a miracle
        elif depth_est == min_target or rel_error(depth_est, min_target) < EPS_THETA_REFINED:
            break
        # if depth_est > min_target
        else:
            min_target = depth_est
            # Switch directions
            cur_dir = "R" if cur_dir == "L" else "L"
            # Reduce speed
            if cur_spd > 98:
                cur_spd -= 1
            if cur_len > 50:
                cur_len -= 1

        num_attempts += 1

    #cur_dir = "R" if cur_dir == "L" else "L"
    #send_cmd(arduino, cur_dir, cur_spd, cur_len)
    print_header("Fine refinement finished!")


def get_depth_target(data, perc=0.0):
    assert -0.5 <= perc <= 0.5
    _b1, _b2, valley_idxs = detect_num_extremums(data)
    v1 = data[valley_idxs[0]]
    v2 = data[valley_idxs[1]]
    dtot = v1 + v2
    return ((1- perc) / (2)) * dtot


def get_cur_depth_from_data(data):
    _b1, _b2, valley_idxs = detect_num_extremums(data)
    v1 = data[valley_idxs[0]]
    v2 = data[valley_idxs[1]]
    return min(v1, v2)


def rel_error(d1, d2):
    return abs(d1-d2) / (d1+d2)


def is_at_depth(d, d_target, p_err=0.04):
    p_d = rel_error(d,d_target)
   # print(f"Percent distance: {p_d}")
   # print(f"D1: {d1}, D2: {d2}")
    return p_d < p_err


def in_center_of_hall(data):
    _1, _2, valley_idxs = detect_num_extremums(data)
    v1 = data[valley_idxs[0]]
    v2 = data[valley_idxs[1]]
    if is_at_depth(v1, v2):
        return True
    return False


def reverse_to_depth(arduino, pipe, data, depth=0.0):
    d_target = get_depth_target(data,perc=depth)
    cur_depth = get_depth_estimate(pipe, nimages=3)
    TURN_CTR_MAX = 100
    print("Reversing data:")
    print(f"Target Depth: {d_target}")
    print(f"Current Depth: {cur_depth}")
    turn_ctr = 1
    #cur_dir = 'B'
    cur_dir = 'F' if d_target - cur_depth < 0 else 'B'
    cur_spd = 100
    while not is_at_depth(cur_depth, d_target, p_err=0.01):
        send_cmd(arduino, cur_dir, cur_spd, 75)
        cur_depth = get_depth_estimate(pipe, nimages=3)
        print(f"Target Depth: {d_target}")
        print(f"Current Depth: {cur_depth}")
        print(f"Difference: {abs(d_target - cur_depth)}")
        turn_ctr += 1
        #if turn_ctr > 50:
        if turn_ctr > TURN_CTR_MAX:
            break
        elif cur_dir == 'B' and d_target - cur_depth < 0:
            cur_dir = 'F'
            cur_spd -= 5
        elif cur_dir == 'F' and d_target - cur_depth > 0:
            cur_dir = 'B'
            cur_spd -= 5


def print_header(header):
    print("\n\n###################################")
    print(header)
    print("###################################\n\n")


def main():
    print(f"Beginning run {RUN_NUM}...")

    # Initialize and start camera
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)

    MAX_CALIBRATION_ATTEMPTS = 10
    calibration_attempt = 0

    dtmap = DTMap()

    with serial.Serial('/dev/ttyUSB0', 9600, timeout=10) as arduino:
        init_arduino(arduino)
        dcur = 0.45

        while calibration_attempt < MAX_CALIBRATION_ATTEMPTS:
            print_header("SCANNING SURROUNDINGS")
            data, smooth_data = scan_surroundings(arduino, pipeline)
            dtmap.add_data(data)

            print("\n\n****************************")
            print(f"After Scanning, current depth: {get_cur_depth_from_data(smooth_data)}")
            print(f"Aiming for depth target: {get_depth_target(smooth_data, perc=dcur)}")
            print("*******************************\n\n")

            # CHANGE RUN_NUM BEFORE EACH RUN!
            print_header("SAVING DATA")
            np.savetxt(f"./data/run-{RUN_NUM}-calib-{calibration_attempt}.txt", 
                    data)
            np.savetxt(f"./data/run-{RUN_NUM}-calib-{calibration_attempt}-smooth.txt", smooth_data)

            print_header("CHECKING IF CENTERED")
            if in_center_of_hall(smooth_data):
                print_header("WE ARE CENTERED!")
                # Happy dance
                send_cmd(arduino, 'R', 150, 1000)
                send_cmd(arduino, 'L', 150, 100)
                send_cmd(arduino, 'R', 150, 100)
                break

            print_header("TURNING TO DESIRED ANGLE")
            turn_to_min_angle(arduino, pipeline, smooth_data)

            print_header(f"REVERSING TO TARGET DEPTH (d={dcur})")
            reverse_to_depth(arduino, pipeline, smooth_data, depth=dcur)
            calibration_attempt += 1
            dcur = max(dcur - 0.15, 0)


    dtmap.plot_data()
    dtmap.save_images()
    print_header("RUN COMPLETE")


if __name__ == '__main__':
    main()
