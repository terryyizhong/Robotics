from localization import design_waypoints, obstacle_2d
import time
import Robot
import commands
import os

car_start_pos = (30, 100)
obstacle_tag_pos = [car_start_pos[0] + 30, car_start_pos[1]]
safe_corners = obstacle_2d(50, obstacle_tag_pos, 20)
boundary_corners = [(0, 0), (0, 200), (500, 200), (500, 0)]
waypoints = design_waypoints(car_start_pos, safe_corners, boundary_corners)
waypoints.insert(0, car_start_pos)
print waypoints


def find_treasure(car_pos, angle):
	main = "./estimate3d"
	f = os.popen(main)
	time.sleep(1.0)
	f.close()
	os.system(main)
	tag_xz = read_t('outf')
	print car_pos, tag[1], tag[2]

def read_t(filename):
	# read the file output by estimated3d of chilitags
	file = open(filename)
	tag_xz = []
	a = file.read().split("\n")
	for line in a:
		s = line.split(",")
		s[0] = float(s[0][4:])
		s[1] = float(s[1])
		s[2] = float(s[2])
		tag_xz.append(s)
	file.close
	return tag_xz


LEFT_TRIM = -9
RIGHT_TRIM = 0
s90, t90 = 100, 0.8
ss = 150
vs = 50  # at speed ss, vs = ? cm/s

robot = Robot.Robot(left_trim=LEFT_TRIM, right_trim=RIGHT_TRIM)
angle = 0
for i in range(12):
    dx = waypoints[i+1][0] - waypoints[i][0]
    dy = waypoints[i+1][1] - waypoints[i][1]
    if dx:
        if dx > 0:
            direction = 0
            if angle == 90:
                robot.right(s90, t90)
            else:
                robot.left(s90, t90)
        else:
            direction = 180
            if angle == 90:
                robot.left(s90, t90)
            else:
                robot.right(s90, t90)
        angle = direction
        find_treasure(waypoints[i], angle)
        ts = abs(dx)*1.0/vs
        time.sleep(1.0);
		robot.forward(ss, ts)
		time.sleep(1.0);
    else:
        if dy > 0:
            direction = 90
            if angle == 180:
                robot.right(s90, t90)
            else:
                robot.left(s90, t90)
        else:
            direction = -90
            if angle == 180:
                robot.left(s90, t90)
            else:
                robot.right(s90, t90)
        angle = direction
        find_treasure(waypoints[i], angle)
        ts =abs(dy*1.0)/vs
        time.sleep(1.0);
		robot.forward(ss, ts)
		time.sleep(1.0);
    find_treasure(waypoints[i+1], angle)
