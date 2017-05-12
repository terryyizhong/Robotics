# input1: position and tag ids of 4 corners with D
# format:tag_positions = [(id, [id.x, id.y]) for id in ids]
#
# input2(from estimate3d): 4 distances and correspond tag_id to 4 corners
# format: tag_dist = [[id, id.dist], for id in ids]
#
# input3: four_tag and obstacle sizes = [width, height], edge length of obstacle cube = obstacle_edge
#
# input4: boundary corners = [[], [], [], []] (clockwise)

import os
import time
import commands
import Robot
import numpy
from itertools import combinations


def trilateration(p1, r1, p2, r2, p3, r3):
    p1 = numpy.array(p1)
    p2 = numpy.array(p2)
    p3 = numpy.array(p3)

    ex = (p2 - p1) * 1.0 / (numpy.linalg.norm(p2 - p1))
    i = numpy.dot(ex, p3 - p1)
    ey = (p3 - p1 - i * ex) * 1.0 / (numpy.linalg.norm(p3 - p1 - i * ex))
    d = numpy.linalg.norm(p2 - p1)
    j = numpy.dot(ey, p3 - p1)

    x = (pow(r1, 2) - pow(r2, 2) + pow(d, 2)) / (2.0 * d)
    y = ((pow(r1, 2) - pow(r3, 2) + pow(i, 2) + pow(j, 2)) / (2.0 * j)) - ((i * 1.0 / j) * x)

    p = p1 + x * ex + y * ey
    return p


def scale(l):
    # real world : bot = k : 1
    return l / 15.0


def localization(tag_d, tag_pos):
    # compute the average car position by doing trilateration on combinations of four_tags
    tag_ids = [tag[0] for tag in tag_d]
    position_dic = {tag[0]: tag[1] for tag in tag_pos}
    dist_dic = {tag[0]: tag[1] for tag in tag_d}
    p_add = numpy.zeros(2)
    for combine in combinations(tag_ids, 3):
        p_add += trilateration(position_dic[combine[0]], dist_dic[combine[0]],
                               position_dic[combine[1]], dist_dic[combine[1]],
                               position_dic[combine[2]], dist_dic[combine[2]])
    p_average = p_add / 4.0
    return p_average[0], p_average[1]


def read(filename):
    # read the file output by estimated3d of chilitags
    f = open(filename)
    tag_d = []
    a = f.read().split("\n")
    for i in range(len(a)-1):
        s = a[i].split(",")
        s[0] = float(s[0][4:])
        s[1] = float(s[1])
        tag_d.append(s)
    f.close
    return tag_d


def obstacle_2d(e, o, w):
    # clockwise order
    corners = [(o[0] - 1.0*e, o[1] - (w / 2.0 +1.0*e)),
               (o[0] - 1.0*e, o[1] + (w / 2.0 +1.0*e)),
               (o[0] + 2.0*e, o[1] + (w / 2.0 +1.0*e)),
               (o[0] + 2.0*e, o[1] - (w / 2.0 +1.0*e))]
    return corners


def design_waypoints(start, safe_c, boundary_c):
    w = [None] * 12
    w[0] = (start[0], safe_c[0][1])
    w[1] = safe_c[3]
    w[2] = safe_c[2]
    width = safe_c[2][1] - safe_c[3][1]
    d = boundary_c[3][0] - 30 - safe_c[3][0]
    step = d / 4.0
    for i in range(3, 11):
        if i % 2:
            w[i] = (w[i - 1][0] + step, w[i - 1][1])
        elif i % 4 == 0:
            w[i] = (w[i - 1][0], w[i - 1][1] - width)
        else:
            w[i] = (w[i - 1][0], w[i - 1][1] + width)
    w[11] = safe_c[2]
    w.insert(0, start)
    return w


def find_treasure(car_pos, angle, tag_set, treasures, t_k=0.5):
    main = "./estimate3d"
    f = os.popen(main)
    time.sleep(1.0)
    f.close()
    os.system(main)
    try:
        tag_xz = read_tr('outf')
    except:
        return 0
    for txz in tag_xz:
        if txz[0] in tag_set:
            continue
        else:
            tag_set.add(txz[0])
            treasures[txz[0]] = treasure_pos(car_pos, angle, t_k*txz[1], t_k*txz[2])


def treasure_pos(car_pos, angle, x, z):
    y = numpy.sqrt(z * z - x * x)
    if angle == 0:
        t_pos = (car_pos[0] + y, car_pos[1] - x)
    elif angle == 90:
        t_pos = (car_pos[0] + x, car_pos[1] + y)
    elif angle == -90:
        t_pos = (car_pos[0] - x, car_pos[1] - y)
    else:
        t_pos = (car_pos[0] - y, car_pos[1] + x)
    return t_pos


def read_tr(filename):
    # read the file output by estimated3d of chilitags
    f = open(filename)
    tag_xz = []
    a = f.read().split("\n")
    for i in range(len(a)-1):
        s = a[i].split(",")
        s[0] = int(s[0][4:])
        s[1] = float(s[1])
        s[2] = float(s[2])
        tag_xz.append(s)
    f.close
    return tag_xz


if __name__ == "__main__":

    # get the input of four_tag ids and positions
    tag_positions = []
    for i in range(4):
        position = raw_input("please input id and position of tag "+str(i+1)+"(clockwise), format: id,x,y : ").split(",")
        id = float(position[0])
        x, y = float(position[1]), float(position[2])
        tag_positions.append([id, (x, y)])
        
    boundary_corners = []
    for i in range(4):
        position = raw_input("please input position of boundary corners "+str(i+1)+"(clockwise), format: x,y : ").split(",")
        x, y = float(position[0]), float(position[1])
        boundary_corners.append((x, y))
    '''
    tag_positions = [[631, (0, 0)], [815, (0, 145)], [816, (27, 145)], [814, (27, 0)]]
    boundary_corners = [(0, 0), (0, 200), (500, 200), (500, 0)]
    '''
    # get the input of size of four_tags and obstacle_tag and size of obstacle
    size = float(input("please input the size of tag: "))
    obstacle_size = float(input("please input the size of obstacle_tag: "))
    obstacle_edge = float(input("please input the length of obstacle_edge: "))
    treasures_size = float(input("please input the size of treasure: "))
    k = scale(size)
    obstacle_k = scale(obstacle_size)
    treasures_k = scale(treasures_size)

    # read dists from each tag to car
    tag_dists = read('out')
    if len(tag_dists) == 5:
        tag_set = {tag[0] for tag in tag_dists}
        obstacle_tag_dist = tag_dists[0]
        obstacle_tag_dist[1] *= obstacle_k
        four_tags = tag_dists[1:]
        for tag in four_tags:
            tag[1] *= k

    # compute the car start position and obstacle range
    car_start_pos = localization(four_tags, tag_positions)
    obstacle_tag_pos = (car_start_pos[0] + obstacle_tag_dist[1], car_start_pos[1])
    safe_corners = obstacle_2d(obstacle_edge, obstacle_tag_pos, size)
    waypoints = design_waypoints(car_start_pos, safe_corners, boundary_corners)

    # start find treasures
    s90 = 100
    t90 = 0.7
    ss = 150
    vs = 24  # at speed ss, vs = ? cm/s
    robot = Robot.Robot(left_trim=-8, right_trim=0)
    angle = 0

    treasures, tag_set = {}, set()
    for i in range(12):
        dx = waypoints[i + 1][0] - waypoints[i][0]
        dy = waypoints[i + 1][1] - waypoints[i][1]
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
            find_treasure(waypoints[i], angle, tag_set, treasures)
            ts = abs(dx) * 1.0 / vs
            time.sleep(1.0)
            robot.forward(ss, ts)
            time.sleep(1.0)
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
            find_treasure(waypoints[i], angle, tag_set, treasures)
            ts = abs(dy * 1.0) / vs
            time.sleep(1.0)
            robot.forward(ss, ts)
            time.sleep(1.0)
        find_treasure(waypoints[i + 1], angle, tag_set, treasures)
    print treasures
    with open('treasure.txt', 'w') as f:
        f.write(str(treasures))
