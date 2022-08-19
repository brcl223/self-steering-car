import cv2
import numpy as np
from PIL import Image
import pyrealsense2.pyrealsense2 as rs
import pickle
import serial
import time

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

clf = None
with open('OpenBotSVM/svc1.pickle', 'rb') as pfile:
	clf = pickle.load(pfile)
def crop_img(img):
    # Cut image evenly off both sides. "Cut the crust off"
	cut_amt = (480-320)//2
	return img[cut_amt:-cut_amt,:]

try:
	with serial.Serial('/dev/ttyUSB0', 9600, timeout=10) as arduino:
		while True:
			frames = pipeline.wait_for_frames()
			depth_frame = frames.get_depth_frame()
			if not depth_frame: continue
        
			depth_image = np.asanyarray(depth_frame.get_data())
			print(depth_image.shape)
			#depth_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
			depth_image = crop_img(depth_image).reshape(1,-1)
			print(depth_image.shape)
			#depth_image = crop_img(depth_image)
			#depth_image = cv2.resize(depth_image, dsize=(150,150))#.reshape(1,-1)
			#cv2.imshow('test',depth_image)
			#cv2.waitKey(0)
			prediction = clf.predict(depth_image)
			print(f"Prediction: {prediction}")
			if prediction[0] == 'right':
				arduino.write(bytes('R', 'utf-8'))
				print("Left")
				#time.sleep(1)
			elif prediction[0] == 'left':
				arduino.write(bytes('L', 'utf-8'))
				print("Right")
				#time.sleep(1)
			elif prediction[0] == 'front':
				arduino.write(bytes('F', 'utf-8'))
				print("Front")
			else:
				assert False, "Invalid classifier"
			
         

finally:
	pipeline.stop()
	#cv2.destroyallwindows()
