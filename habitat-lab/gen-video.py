import cv2
import numpy as np
from pathlib import Path

RUN_NUM = 10
NUM_PARAMS = 4
img_array = []


def load_and_test_video(param_num):
    video_path = f"./data/experiments/795-2/run-{RUN_NUM}/hy-{param_num}-sim-0-videoframes/video.mp4"
    if not Path(video_path).exists():
        raise RuntimeError(f"Invalid data path: {video_path}.\nCould not create video. Exiting...")
    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print(f"Unable to open {video_path}. Not sure why.")

    cap.release()


def create_video(param_num):
    data_path = f"./data/experiments/795-2/run-{RUN_NUM}/hy-{param_num}-sim-0-videoframes/"
    if not Path(data_path).exists():
        raise RuntimeError(f"Invalid data path: {data_path}.\nCould not create video. Exiting...")
    for i in range(2000):
        filename = f"{data_path}/frame-{i}.npy"
        if not Path(filename).exists():
            print(f"Unable to locate file number {i+1}. Skipping further frames...")
            break
        img = np.load(filename)
        depth_rgba = cv2.cvtColor(img, cv2.COLOR_GRAY2BGRA)
        height, width, layer = depth_rgba.shape
        size = (width,height)
        img_array.append(img)


    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    #fourcc = cv2.VideoWriter_fourcc(*'h264')
    FPS = 30
    video_fname = f"{data_path}/video.mp4"
    #video_fname = f"{data_path}/video.avi"
    #fourcc = cv2.VideoWriter_fourcc(*'X264')        
    #fourcc = cv2.VideoWriter_fourcc(*'XVID')
    #out = cv2.VideoWriter(video_fname, cv2.CAP_FFMPEG, fourcc, FPS, size)
    out = cv2.VideoWriter(video_fname, fourcc, FPS, size)
    
    for i in range(len(img_array)):
        out.write(img_array[i])
    out.release()
    print(f"Video saved!\nVideo location: {video_fname}")


def main():
    print(f"Number of params to create videos for: {NUM_PARAMS}")
    for j in range(NUM_PARAMS):
        create_video(j)
        load_and_test_video(j)


if __name__ == '__main__':
    main()