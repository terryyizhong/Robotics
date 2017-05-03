# input1: position and tag ids of 4 corners with D
# format:tag_positions = [(id, [id.x, id.y]) for id in ids]
#
# input2(from estimate3d): 4 distances and correspond tag_id to 4 corners
# format: tag_dist = [[id, id.dist], for id in ids]
#
# input3: four_tag and obstacle sizes = [width, height], edge length of obstacle cube = obstacle_edge
#
# input4: boundary corners = [[], [], [], []] (clockwise)


import numpy
from itertools import combinations


def trilateration(p1, r1, p2, r2, p3, r3):
	p1 = numpy.array(p1)
	p2 = numpy.array(p2)
	p3 = numpy.array(p3)

	ex = (p2 - p1)*1.0/(numpy.linalg.norm(p2 - p1))
	i = numpy.dot(ex, p3 - p1)
	ey = (p3 - p1 - i*ex)*1.0/(numpy.linalg.norm(p3 - p1 - i*ex))
	d = numpy.linalg.norm(p2 - p1)
	j = numpy.dot(ey, p3 - p1)

	x = (pow(r1, 2) - pow(r2, 2) + pow(d, 2))/(2.0*d)
	y = ((pow(r1, 2) - pow(r3, 2) + pow(i, 2) + pow(j, 2))/(2.0*j)) - ((i*1.0/j)*x)

	p = p1 + x*ex + y*ey
	return p


def scale(width, height):
	#realword : bot = k : 1
	return (width + height)*1.0 / 20*2


def localization(tag_d, tag_pos):
	# compute the average car posdinate by doing trilateration on combinations of four_tags
	tag_ids = [tag[0] for tag in tag_d]
	position_dic = {tag[0]: tag[1] for tag in tag_pos}
	dist_dic = {tag[0]: tag[1] for tag in tag_d}
	p_add = numpy.zeros(2)
	for combine in combinations(tag_ids, 3):
		p_add += trilateration(position_dic[combine[0]], dist_dic[combine[0]],
								position_dic[combine[1]], dist_dic[combine[1]],
								position_dic[combine[2]], dist_dic[combine[2]])
	p_average = p_add / 4.0
	return p_average


def read(filename):
	# read the file output by estimated3d of chilitags
	file = open(filename)
	tag_d = []
	a = file.read().split("\n")
	for line in a:
		s = line.split(",")
		s[0] = float(s[0][4:])
		s[1] = float(s[1])
		tag_d.append(s)
	file.close
	return tag_d

def obstacle_2d(e, o, w):
	# clockwise order
	corners = [(o[0]-(e/2.0), o[1]-(w/2.0+e/2.0)),
				(o[0]-(e/2.0), o[1]+(w/2.0+e/2.0)),
				(o[0]+(3*e/2.0), o[1]+(w/2.0+e/2.0)),
				(o[0]+(3*e/2.0), o[1]-(w/2.0+e/2.0))]
	return corners

def design_waypoints(start, safe_c, boundery_c):
	w = [None]*12
	w[0] = (start[0], safe_c[0][1])
	w[1] = safe_c[3]
	w[2] = safe_c[2]
	width = safe_c[2][1] - safe_c[3][1]
	d = boundery_c[3][0] - 30 - safe_c[3][0]
	step = d/4.0
	for i in range(3, 11):
		if i % 2:
			w[i] = (w[i-1][0]+step, w[i-1][1])
		elif i % 4 == 0:
			w[i] = (w[i-1][0], w[i-1][1]-width)
		else:
			w[i] = (w[i-1][0], w[i-1][1]+width)
	w[11] = (start[0], safe_c[2][1])

	return w

if __name__ == "__main__":

	# get the input of four_tag ids and postions
    '''	tag_positions = []
    for i in range(4):
        position = input("please input id and position of tag "+str(i+1)+"(clockwise), format: id,x,y : ").split(",")
        id = float(position[0])
        x, y = float(position[1]), float(position[2])
        tag_positions.append((id, [x, y]))
    
    for i in range(4):
        position = input("please input id and position of tag "+str(i+1)+"(clockwise), format: id,x,y : ").split(",")
        id = float(position[0])
        x, y = float(position[1]), float(position[2])
        tag_positions.append((id, [x, y]))
    '''
    tag_positions = [[631, (0,0)], [815,(0,145)],[816,(27,145)], [814,(27,0)]]
    boundary_corners = [(0, 0),(0, 200),(500, 200),(500, 0)]

    # get the input of size of four_tags and obstacle_tag and size of obstacle
    wid = float(input("please input the width of tag: "))
    hei= float(input("please input the height of tag: "))
    obstacle_w = float(input("please input the width of obstacle_tag: "))
    obstacle_h = float(input("please input the height of obstacle_tag: "))
    obstacle_edge = float(input("please input the height of obstacle_edge: "))
    k = scale(wid, hei)
    obstacle_k = scale(obstacle_w, obstacle_h)

    # read dists from each tag to car
    tag_dists = read('/home/ubuntu/Desktop/chilitags1/build/samples/out')
    if len(tag_dists) == 5:
        obstacle_tag_dist = tag_dists[0]
        obstacle_tag_dist[1] *= obstacle_k
        four_tags = tag_dists[1:]
        for tag in four_tags:
            tag[1] *= k

    # compute the car start position and obstacle range
    car_start_pos = localization(four_tags, tag_positions)
    obstacle_tag_pos = [car_start_pos[0]+obstacle_tag_dist[1], car_start_pos[1]]
    safe_corners = obstacle_2d(obstacle_edge, obstacle_tag_pos, obstacle_w)
    waypoints = design_waypoints(car_start_pos, safe_corners, boundary_corners)
    print waypoints
