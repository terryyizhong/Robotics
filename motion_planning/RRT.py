import random
from math import sqrt, cos, sin, atan2


class Node:
    def __init__(self, x=0, y=0, cost=0, prev=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.prev = prev


# read input
def read(filename):
    with open(filename, 'r') as f:
        content = f.readlines()
    r = []
    for line in content:
        nums = line.replace(',', ' ').replace('(', ' ').replace(')', ' ').split()
        nums = convert_to_points(list(map(int, nums)))
        r.append(nums)
    return r[0], r[1:-1], r[-1]


# convert a list of numbers into a list of tuples
def convert_to_points(nums):
    r = []
    for idx, num in enumerate(nums):
        if idx % 2 == 0:
            r.append((num, nums[idx+1]))
    return r


def counter_clockwise(pt1, pt2, pt3):
    return (pt3[1]-pt1[1]) * (pt2[0]-pt1[0]) > (pt2[1]-pt1[1]) * (pt3[0]-pt1[0])


def intersect(line1, line2):
    return (counter_clockwise(line1[0], line2[0], line2[1]) != counter_clockwise(line1[1], line2[0], line2[1])
            and counter_clockwise(line1[0], line1[1], line2[0]) != counter_clockwise(line1[0], line1[1], line2[1]))


def dist(p1, p2):
    return sqrt((p1[0]-p2[0]) * (p1[0]-p2[0])+(p1[1]-p2[1]) * (p1[1]-p2[1]))


def check_intersect(node_a, node_b, obstacles):
    a = (node_a.x, node_a.y)
    b = (node_b.x, node_b.y)
    line1 = (a, b)
    # print line1
    for obstacle in obstacles:
        for idx, point in enumerate(obstacle):
            # print point
            if idx == len(obstacle)-1:
                line2 = (point, obstacle[0])

            else:
                line2 = (point, obstacle[idx+1])
            # print line1, line2
            if intersect(line1, line2):
                return True
        return False


def step_from_to(p1, p2):
    if dist(p1, p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1]-p1[1], p2[0]-p1[0])
        return p1[0]+EPSILON * cos(theta), p1[1]+EPSILON * sin(theta)


def choose_prev(n, new_node, nodes, obstacles):
    for p in nodes:
        if check_intersect(p, new_node, obstacles) and dist([p.x, p.y],
                                                            [new_node.x, new_node.y]) < RADIUS and p.cost+dist(
                [p.x, p.y], [new_node.x, new_node.y]) < n.cost+dist([n.x, n.y], [new_node.x, new_node.y]):
            n = p
    new_node.cost = n.cost+dist([n.x, n.y], [new_node.x, new_node.y])
    new_node.prev = n
    return new_node, n

# constants
EPSILON = 5.0
NODES_NUM = 1000
RADIUS = 15

if __name__ == '__main__':
    boundary, obstacles, points = read("input.txt")
    x_dim = boundary[2][0]
    y_dim = boundary[2][1]
    nodes = [Node(points[0][0], points[0][1])]
    start = nodes[0]
    goal = Node(points[1][0], points[1][1])

    vertex_dic, vertex_list, path_list = {}, [], []
    n_id = 0
    for i in range(NODES_NUM):
        rand = Node(random.random() * x_dim, random.random() * y_dim)
        n = nodes[0]
        idx = 0
        for p in nodes:
            if dist([p.x, p.y], [rand.x, rand.y]) < dist([n.x, n.y], [rand.x, rand.y]):
                n_id = idx
                n = p
            idx += 1
        interpolated_node = step_from_to([n.x, n.y], [rand.x, rand.y])
        new_node = Node(interpolated_node[0], interpolated_node[1])

        if check_intersect(n, rand, obstacles):
            new_node = choose_prev(n, new_node, nodes, obstacles)[0]
            nodes.append(new_node)
            a = str((n_id, len(nodes)-1))
            path_list.append(a)

    idx = 0
    for i in nodes:
        a = str(idx)+":"+str((i.x, i.y))
        vertex_list.append(a)
        vertex_dic[(i.x, i.y)] = idx
        idx = idx+1

    path = []
    n = nodes[0]
    for p in nodes:
        if dist([p.x, p.y], [goal.x, goal.y]) < dist([n.x, n.y], [goal.x, goal.y]):
            n = p
    while n != start:
        a = vertex_dic[(n.x, n.y)]
        path.append(str(a)+':'+str((n.x, n.y)))
        n = n.prev

    with open("RRT_output.txt", 'w') as out:
        out.write(', '.join(vertex_list)+'\n')
        out.write(', '.join(path_list)+'\n')
        out.write(', '.join(path[::-1]))



