import cv2
import numpy as np


RUN = 8
DATA_LOC = f'./data/run-{RUN}-imgs/'
TOTAL_IMGS = 38 # adjust based on number of images in DATA_LOC


def depth_score(img):
    img = np.array(img)
    PIXELS = 640 * 480
    return img.sum() / PIXELS


def main():
    try:
        for img_idx in range(TOTAL_IMGS):
            fname = f"{DATA_LOC}/{img_idx}.txt"
            img = np.loadtxt(fname)
            score = depth_score(img)
            color_img = cv2.applyColorMap(cv2.convertScaleAbs(img, alpha=0.03), cv2.COLORMAP_JET)
            #img //= 256
            img.astype(np.uint16)
            cv2.imshow('BW', img)
            cv2.imshow('Color', color_img)
            print(f"Image {img_idx} Score: {score}")
            while cv2.waitKey(0) == -1:
                pass
    finally:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
