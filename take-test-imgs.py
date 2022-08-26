import cv2
import pyrealsense2.pyrealsense2 as rs
import numpy as np
import scipy.ndimage as nd


DATA_LOC = f"./data/manual-test-imgs/"


def img_score(img):
    img = np.array(img)
    PIXELS = 640 * 480
    return img.sum() / PIXELS


def main():
    pipeline = rs.pipeline()
    config = rs.config()
    #config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)

    img_ctr = 11

    print("Starting image loop...")
    while input(f"Image {img_ctr}\nContinue? [y/n]: ") != 'n':
        frames = pipeline.wait_for_frames()
        depth = frames.get_depth_frame()
        data = np.asanyarray(depth.get_data())
        data_filtered = nd.filters.gaussian_filter(data, 5, mode='nearest')
        score = int(img_score(data))
        score_filtered = int(img_score(data_filtered))
        color_img = cv2.applyColorMap(cv2.convertScaleAbs(data, alpha=0.03), cv2.COLORMAP_JET)
        color_img_filtered = cv2.applyColorMap(cv2.convertScaleAbs(data_filtered, alpha=0.03), cv2.COLORMAP_JET)
        fname = f"{DATA_LOC}/img-{img_ctr}-score-{score}"

        print(f"Current img score: {score}")
        print(f"Current img score filtered: {score_filtered}")
        print("Saving files...")
        cv2.imwrite(f"{fname}.png", color_img)
        np.savetxt(f"{fname}.txt", data)
        cv2.imwrite(f"{fname}-filtered.png", color_img_filtered)
        np.savetxt(f"{fname}-filtered.txt", data_filtered)
        img_ctr += 1


if __name__ == '__main__':
    main()
