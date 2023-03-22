import pyrealsense2.pyrealsense2 as rs
import numpy as np
import serial # For communicating with Arduino
from time import sleep
#import scipy.signal as sig

#####################################################
# Process Params (i.e. Global Vars)
#####################################################

# CHANGE RUN_NUM BEFORE EACH RUN!
RUN_NUM = 8
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

    nimages = 15

    seg_data_points = []
    lr_data_points = []

    NUM_MOVEMENTS = 75
    for j in range(NUM_MOVEMENTS):
        #sleep(COMMAND_INTERVAL_SECS)
        print(f"Beginning movement {j+1} of {NUM_MOVEMENTS}...")
        data_points.append(get_depth_estimate(pipeline, nimages=nimages))
        #seg_data = get_depth_estimate(pipeline, nregions=3, nimages=nimages)
        #print(f"\n###################\nl: {l}\nf: {f}\nr: {r}\n###############\n")
        #lr_data = get_depth_estimate(pipeline, nregions=2, nimages=nimages)
        #print(f"\n###################\nl2: {l2}\nr2: {r2}\n###############\n")
        #sleep(COMMAND_INTERVAL_SECS)

        #seg_data_points.append(seg_data)
        #lr_data_points.append(lr_data)

        num_extremums, _b1, _b2 = detect_num_extremums(smooth_data(data_points))
        if num_extremums >= 4:
            print("Found two peaks and two vallies!")
            break

        print("Turning arduino")
        #turn_for_next_img(arduino)
        send_cmd(arduino, 'L', 125, 75)

    #print("Saving segmented and LR test data...")
    np.savetxt(f"./data/run-{RUN_NUM}.txt", data_points)
    np.savetxt(f"./data/run-{RUN_NUM}-smooth.txt", smooth_data(data_points))
    #np.savetxt(f"./data/run-{RUN_NUM}-segmented.txt", seg_data_points)
    #np.savetxt(f"./data/run-{RUN_NUM}-lr.txt", lr_data_points)

    return data_points, smooth_data(data_points)



def send_cmd(arduino, motion, speed, duration):
    # Send command
    cmd = f"{motion}.{str(speed)}.{str(duration)}\n"
    arduino.write(bytes(cmd, 'utf-8'))
    #sleep((duration / 1000.0) - 0.001)

    # Await Response
    # TODO: Add timeout here
    while True:
        data = arduino.readline().decode('utf-8')
        print(f"[DEBUG] Got line from arduino: {data}",end="")
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
def get_depth_estimate(pipe, nimages=60, nregions=1):
    
    IMG_COUNT = nimages # at 30fps this takes ~2 seconds
    MAX_SKIPPED = 10
    skipped_imgs = 0

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

    while abs(depth_est - target_depth) > TURNING_EPS:
        send_cmd(arduino, 'L', 100, 75)
        depth_est = get_depth_estimate(pipe, nimages=10)
        print(f"Target Depth: {target_depth}")
        print(f"Current Depth: {depth_est}")
        print(f"Difference: {abs(depth_est - target_depth)}")
        tries += 1
        if tries > 200:
            raise Exception("Could not get back to min!")



def get_depth_target(data):
    _b1, _b2, valley_idxs = detect_num_extremums(data)
    v1 = data[valley_idxs[0]]
    v2 = data[valley_idxs[1]]
    return 0.5 * (v1 + v2)


def is_centered(d1, d2):
    p_err = 0.075 # %
    p_d = abs(d1 - d2) / (d1 + d2)
    print(f"Percent distance: {p_d}")
    print(f"D1: {d1}, D2: {d2}")
    return p_d < p_err


def in_center_of_hall(data):
    _1, _2, valley_idxs = detect_num_extremums(data)
    v1 = data[valley_idxs[0]]
    v2 = data[valley_idxs[1]]
    if is_centered(v1, v2):
        return True
    return False


def reverse_until_centered(arduino, pipe, data):
    d_target = get_depth_target(data)
    cur_depth = get_depth_estimate(pipe, nimages=15)

    print("Reversing data:")
    print(f"Target Depth: {d_target}")
    print(f"Current Depth: {cur_depth}")
    turn_ctr = 1
    while not is_centered(d_target, cur_depth):
        send_cmd(arduino, 'B', 100, 75)
        cur_depth = get_depth_estimate(pipe, nimages=15)
        print(f"Target Depth: {d_target}")
        print(f"Current Depth: {cur_depth}")
        print(f"Difference: {abs(d_target - cur_depth)}")
        turn_ctr += 1
        if turn_ctr > 30:
            break


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
    with serial.Serial('/dev/ttyUSB0', 9600, timeout=10) as arduino:
        init_arduino(arduino)

        while calibration_attempt < MAX_CALIBRATION_ATTEMPTS:
            print_header("SCANNING SURROUNDINGS")
            data, smooth_data = scan_surroundings(arduino, pipeline)

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

            print_header("REVERSING TO TARGET DEPTH")
            reverse_until_centered(arduino, pipeline, smooth_data)
            calibration_attempt += 1


    print_header("RUN COMPLETE")


if __name__ == '__main__':
    main()
