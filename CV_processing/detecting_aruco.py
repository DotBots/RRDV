# import the necessary packages
import imutils
import time
import cv2
import sys
import json
import numpy as np
import pandas as pd
from dist_to_line import minDistance


# Configuration
ANGLE = 20 # degrees
ANGLE = np.cos(np.radians(ANGLE))

ROBOT_SIZE = 100 # mm radius

# Read Json file
# Read API keys
with open("Experiments/corners.json", "r") as read_file:
	corners_file = json.load(read_file)


arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
arucoParams = cv2.aruco.DetectorParameters_create()
# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting video stream...")
# vs = VideoStream(src=0).start()

def click_event(event, x, y, flags, params):
 
    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:
 
        # displaying the coordinates
        # on the Shell
        print(f'x:{x*6}, y:{y*6}')
 
cv2.namedWindow("Frame")
cv2.namedWindow("Warped")
cv2.setMouseCallback("Frame", click_event)
cv2.setMouseCallback("Warped", click_event)

# Define the codec and create VideoWriter object and save the video
# fourcc = cv2.VideoWriter_fourcc(*'XVID')
# fourcc = cv2.VideoWriter_fourcc(*'MP4V')
# out = cv2.VideoWriter('output.avi',fourcc, 30.0, (1920,1080))

# Initialize video input
video_path = 'Experiments/220824 - Third experiment/1 - transmit all/DJI_0469.MP4'
# video_path = 'Experiments/220824 - Third experiment/3 - transmit all/DJI_0475.MP4'
video_name = video_path.split('/')[-1]
print(f"Video to analyze = {video_name}")

cap = cv2.VideoCapture(video_path)
ret, calib_frame = cap.read()
print(calib_frame.shape)
if (calib_frame is None):
	print("Error: Video not Found")
	exit()

## Calculate the Homograpy matrix
v_c = corners_file[video_name]['corners']
v_r = corners_file['experiment']['corners']
pts_src = np.array([v_c['tl'], v_c['tr'], v_c['br'], v_c['bl']])
pts_dst = np.array([v_r['tl'], v_r['tr'], v_r['br'], v_r['bl']])
h, status = cv2.findHomography(pts_src, pts_dst)
# h, status = cv2.getPerspectiveTransform(pts_src, pts_dst)  # https://stackoverflow.com/q/11237948

## Create a dataframe for storing everything
df = pd.DataFrame(columns=['timestamp', 'r1_x', 'r1_y', 'r1_a', 'r2_x', 'r2_y', 'r2_a', 'r3_x', 'r3_y', 'r3_a', 'r4_x', 'r4_y', 'r4_a', 'r5_x', 'r5_y', 'r5_a'])


# loop over the frames from the video stream
try:
	frame_num = 0
	while True:
		# grab the frame from the threaded video stream and resize it
		# to have a maximum width of 1000 pixels
		# frame = vs.read()
		# If the video ended, close it an open it again to make it loop.
		if not cap.isOpened():
			cap.release()
			break
			# cap = cv2.VideoCapture(video_path)

		ret, frame = cap.read()	
		frame_num += 1
		# if frame_num < 690 or frame_num > 1090:
		# 	continue
		# if frame_num < 1500:
		# 	continue

		# print(f'frame = {frame_num}')
		frame_g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

		# frame = imutils.resize(frame, width=1500)
		# detect ArUco markers in the input frame
		(corners, ids, rejected) = cv2.aruco.detectMarkers(frame_g, arucoDict, parameters=arucoParams)



		# verify *at least* one ArUco marker was detected
		if len(corners) > 0:
			
			# create a dict to save the position of the robots in each frame
			robot_pos = {}
			robot_pos['timestamp'] = corners_file[video_name]['timestamp'] + 1/29.97 * 1000 * (frame_num)

			im_out = cv2.warpPerspective(frame, h, (6930, 4260))

			# flatten the ArUco IDs list
			ids = ids.flatten()
			# debug print
			# print(f"ids = {ids}, corners = {corners}")
			# loop over the detected ArUCo corners
			for (markerCorner, markerID) in zip(corners, ids):
				# extract the marker corners (which are always returned
				# in top-left, top-right, bottom-right, and bottom-left
				# order)
				print(f"markerCorner = {markerCorner}")
				print(f"markerCorner.shape = {markerCorner.shape}")
				corners = markerCorner.reshape((4, 2))
				# print(f"markerCorner = {markerCorner}")
				(topLeft, topRight, bottomRight, bottomLeft) = corners
				# convert each of the (x, y)-coordinate pairs to integers
				topRight = (int(topRight[0]), int(topRight[1]))
				bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
				bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
				topLeft = (int(topLeft[0]), int(topLeft[1]))

				# draw the bounding box of the ArUCo detection
				# cv2.line(frame, topLeft, topRight, (0, 255, 0), 6)
				cv2.line(frame, topRight, bottomRight, (0, 255, 0), 6)
				cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 6)
				cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 6)
				# compute and draw the center (x, y)-coordinates of the
				# ArUco marker
				cX = int((topLeft[0] + bottomRight[0]) / 2.0)
				cY = int((topLeft[1] + bottomRight[1]) / 2.0)
				cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
				# draw the ArUco marker ID on the frame
				cv2.putText(frame, str(markerID),
					(topLeft[0], topLeft[1] - 30),
					cv2.FONT_HERSHEY_SIMPLEX,
					3, (0, 255, 0), 6)

				## Convert the points to meters
				meterCorner = cv2.perspectiveTransform(markerCorner, h).reshape((4, 2))
				(tl, tr, br, bl) = meterCorner
				midPoint = [(tl[0] + br[0]) / 2.0,  (tl[1] + br[1]) / 2.0]
				# angle = np.arctan2(tr[0] - br[0], tr[1] - br[1])
				angle = np.arctan2(tr[1] - br[1], tr[0] - br[0])

				# draw a line to mark the direction.
				mpLen = 150
				cv2.line(im_out, [int(midPoint[0]), int(midPoint[1])], [int(midPoint[0] + mpLen*np.cos(angle)), int(midPoint[1] + mpLen*np.sin(angle))], (255, 0, 0), 20)
				cv2.circle(im_out, [int(midPoint[0]), int(midPoint[1])], 30, (0, 0, 255), -1)


				# Save to dictionary
				if markerID in [1,2,3,4,5]:
					robot_pos[f"r{markerID}_x"] = midPoint[0]
					robot_pos[f"r{markerID}_y"] = midPoint[1]
					robot_pos[f"r{markerID}_a"] = angle*180/np.pi
				
				if markerID == 1:
					print(f"midPoint = {midPoint}")
					print(f"angle = {angle*180/np.pi}")


			### Calculate distances and angles.
			dist = {}
			robot_combination = [[1,2], [1,3], [1,4], [1,5], [2,3], [2,4], [2,5], [3,4], [3,5], [4,5]] 
			for a,b in robot_combination:
				if a in ids and b in ids:

					# Calculate distance between robot
					dist[f'd{a}{b}'] = ((robot_pos[f"r{b}_x"] - robot_pos[f"r{a}_x"])**2 + (robot_pos[f"r{b}_y"] - robot_pos[f"r{a}_y"])**2)**(1/2)
					# Print distance
					cv2.putText(im_out, str(round(dist[f'd{a}{b}']/10))+'cm',
					[int((robot_pos[f"r{b}_x"] + robot_pos[f"r{a}_x"]) / 2.0),  int((robot_pos[f"r{b}_y"] + robot_pos[f"r{a}_y"]) / 2.0)],
					cv2.FONT_HERSHEY_SIMPLEX,
					3, (255, 255, 255), 6)
					# Print connecting lines between robots.
					cv2.line(im_out, [int(robot_pos[f"r{a}_x"]), int(robot_pos[f"r{a}_y"])], [int(robot_pos[f"r{b}_x"]), int(robot_pos[f"r{b}_y"])], (255, 255, 255), 5)


			df_vec = {}
			for a, b in robot_combination:
				if a in ids and b in ids:
					df_vec[f'v{a}{b}_x'] = (robot_pos[f'r{b}_x'] - robot_pos[f'r{a}_x']) / dist[f'd{a}{b}']
					df_vec[f'v{a}{b}_y'] = (robot_pos[f'r{b}_y'] - robot_pos[f'r{a}_y']) / dist[f'd{a}{b}']
			# Calculate individual robot unit df_vectors
			for a in range(1,5 +1):
				if a in ids:
					df_vec[f'v{a}_x'] = np.cos(np.radians(robot_pos[f'r{a}_a']))
					df_vec[f'v{a}_y'] = np.sin(np.radians(robot_pos[f'r{a}_a']))

		
			# robot angle
			df_angle = {}
			# are they looking at each other? Calculate the dot product between the robot direction and the connecting vector
			for a, b in robot_combination:
				if a in ids and b in ids:
					xa,ya,xab,yab = (df_vec[f'v{a}_x'], df_vec[f'v{a}_y'], df_vec[f'v{a}{b}_x'], df_vec[f'v{a}{b}_y'])
					df_angle[f'a{a}{b}'] = np.dot([xa, ya], [xab, yab])
					xb,yb,xab,yab = (df_vec[f'v{b}_x'], df_vec[f'v{b}_y'], df_vec[f'v{a}{b}_x'], df_vec[f'v{a}{b}_y'])
					df_angle[f'a{b}{a}'] = np.dot([xb, yb], [xab, yab])

			# robot angle
			df_look = {}
			# are they looking at each other?
			for a, b in robot_combination:
				if a in ids and b in ids:
					# Check if robots are looking at each other.
					other_bots = [id for id in list(ids) if id in [1,2,3,4,5]]  # Use only the robots that got detected by ARUCO
					other_bots.remove(a)
					other_bots.remove(b)

					# Check if  robots are looking at each other
					df_look[f'look{a}{b}'] = ((df_angle[f'a{a}{b}'] > ANGLE) & (df_angle[f'a{b}{a}'] < -ANGLE))
					
					# Check that no other robot is too close
					for bot in other_bots:
						df_look[f'look{a}{b}'] &= (minDistance([robot_pos[f"r{a}_x"], robot_pos[f"r{a}_y"]], [robot_pos[f"r{b}_x"], robot_pos[f"r{b}_y"]], [robot_pos[f"r{bot}_x"], robot_pos[f"r{bot}_y"]]) > ROBOT_SIZE)

					# df_look[f'look{a}{b}'] = ((df_angle[f'a{a}{b}'] > ANGLE) & (df_angle[f'a{b}{a}'] < -ANGLE)) \
					# 						& (True not in [minDistance([robot_pos[f"r{a}_x"], robot_pos[f"r{a}_y"]], [robot_pos[f"r{b}_x"], robot_pos[f"r{b}_y"]], [robot_pos[f"r{bot}_x"], robot_pos[f"r{bot}_y"]]) < ROBOT_SIZE for bot in other_bots])
					
					# Check if there is anything in the middle of the robots.
					# if df_look[f'look{a}{b}']:
					# 	# get the index of the other robots
					# 	other_bots = [id for id in list(ids) if id in [1,2,3,4,5]]  # Use only the robots that got detected by ARUCO
					# 	other_bots.remove(a)
					# 	other_bots.remove(b)
					# 	for bot in other_bots:
					# 		if minDistance([robot_pos[f"r{a}_x"], robot_pos[f"r{a}_y"]], [robot_pos[f"r{b}_x"], robot_pos[f"r{b}_y"]], [robot_pos[f"r{bot}_x"], robot_pos[f"r{bot}_y"]]) < ROBOT_SIZE:
					# 			# If a robot is too close to an active line, cancel the line
					# 			df_look[f'look{a}{b}'] = False

					if df_look[f'look{a}{b}']:
						# Print connecting lines between robots.
						cv2.line(im_out, [int(robot_pos[f"r{a}_x"]), int(robot_pos[f"r{a}_y"])], [int(robot_pos[f"r{b}_x"]), int(robot_pos[f"r{b}_y"])], (0, 255, 0), 15)




		# Save dictionary to dataframe
		df = df.append(robot_pos, ignore_index=True)
		# df = pd.concat([df, robot_pos], ignore_index=True)

		im_out = imutils.resize(im_out, width=int(im_out.shape[1]/6))
		cv2.imshow("Warped", im_out)

		frame = imutils.resize(frame, width=int(calib_frame.shape[1]/4))
		# show the output frame
		cv2.imshow("Frame", frame)
		# out.write(frame)
		key = cv2.waitKey(1) & 0xFF
		# if the `q` key was pressed, break from the loop
		if key == ord("q"):
			break
	# do a bit of cleanup
	cv2.destroyAllWindows()
	# vs.stop()
	cap.release()
	# out.release()

	## Save the dataframe to a csv
	print(df.tail())
	print(f'last frame = {frame_num}')
	df.to_csv(video_path.replace('.MP4', '.csv.bak'))
except Exception as e:
	print(f'ERROR! = {2}')
	print(f'last frame = {frame_num}')
	## Save the dataframe to a csv
	# print(df.tail())
	df.to_csv(video_path.replace('.MP4', '.csv.bak'))

	# do a bit of cleanup
	cv2.destroyAllWindows()
	# vs.stop()
	cap.release()
	# out.release()