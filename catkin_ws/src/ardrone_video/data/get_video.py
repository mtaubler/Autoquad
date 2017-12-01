import numpy as np
import rosbag
import cv2
import sys

def main(argv):
	filename1 = argv[1]
	filename2 = argv[2]

	bag = rosbag.Bag(filename1, "r")
	out_file = filename2
	fourcc_settings = cv2.VideoWriter_fourcc(*'XVID')
	out_vid_dims = (640,360)
	fps = 30
	out = cv2.VideoWriter(out_file,
	                      fourcc=fourcc_settings,
	                      fps=fps,
	                      frameSize=out_vid_dims)
	
	for topic, msg, t in bag.read_messages(topics=['ardrone/front/image_raw']):
		frame = np.fromstring(msg.data, dtype=np.uint8)
		frame = frame.reshape(msg.height, msg.width, 3)
		outvid = cv2.resize(frame, out_vid_dims)
		out.write(outvid)

		cv2.imshow('frame', frame)

	out.release()
	


if __name__ == "__main__":
	main(sys.argv)
