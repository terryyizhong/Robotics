from math import sqrt, fabs


class Node:
    def __init__(self, coordinates, neighbors=[], dist=-1, prev=None):
        self.coordinates = coordinates
        self.neighbors = neighbors
        self.dist = dist
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


# counter-clockwise
def counter_clockwise(pt1, pt2, pt3):
    return (pt3[1]-pt1[1]) * (pt2[0]-pt1[0]) > (pt2[1]-pt1[1]) * (pt3[0]-pt1[0])


# Return true if line segments AB and CD intersect
def intersect(line1, line2):
    return (counter_clockwise(line1[0], line2[0], line2[1]) != counter_clockwise(line1[1], line2[0], line2[1])
            and counter_clockwise(line1[0], line1[1], line2[0]) != counter_clockwise(line1[0], line1[1], line2[1]))


# find the distance between two vertices
def dist(p1, p2):
    return sqrt((p1[0]-p2[0]) * (p1[0]-p2[0])+(p1[1]-p2[1]) * (p1[1]-p2[1]))


# calculate determinant to check if a vertex is a reflex
def check_reflex(v1, vertex, v2):
    determinant = (vertex[0]-v1[0]) * (v2[1]-vertex[1])-(v2[0]-vertex[0]) * (vertex[1]-v1[1])
    return determinant > 0


# check if two points are visible to each other
def visible(v1, v2, obstacles, curr_obstacles):
    for obstacle in curr_obstacles:
        if not same_side(v1, v2, obstacle):
            return False
    for obstacle in obstacles:
        if obstacle not in curr_obstacles and blocked(v1, v2, obstacle):
            return False
    return True


# check if an obstacle is on the same side of a path connecting the two vertices
def same_side(v1, v2, obstacle):
    pos = 0
    for point in obstacle:
        det = (v2[0]-v1[0]) * (point[1]-v1[1])-(v2[1]-v1[1]) * (point[0]-v1[0])
        if det * pos < 0:
            return False
        pos += det
    return True


# whether an obstacle blocks between vertices v1 and v2, check intersection between
def blocked(v1, v2, obstacle):
    line1 = (v1, v2)
    for idx, point in enumerate(obstacle):
        if idx == len(obstacle)-1:
            line2 = (point, obstacle[0])
        else:
            line2 = (point, obstacle[idx+1])
        if intersect(line1, line2):
            return True
    return False


# add an edge to the roadmap
def add_to_roadmap(v1, v2, roadmap):
    roadmap[v1] = roadmap.get(v1, [])+[v2]
    roadmap[v2] = roadmap.get(v2, [])+[v1]


# find the Node with minimum dist in current queue
def find_min_dist(queue, node_dict):
    vertex = None
    for v in queue:
        graph_node = node_dict[v]
        if graph_node.dist != -1 and (vertex is None or graph_node.dist < node_dict[vertex].dist):
            vertex = v
    return vertex


if __name__ == '__main__':
    boundary, obstacles, points = read("input.txt")
    reflex_vertices = []
    roadmap = {}

    # find all reflex vertices
    for i, obstacle in enumerate(obstacles):
        for idx, vertex in enumerate(obstacle):
            if idx == 0:
                isReflex = check_reflex(obstacle[-1], vertex, obstacle[idx+1])
            elif idx == len(obstacle)-1:
                isReflex = check_reflex(obstacle[idx-1], vertex, obstacle[0])
            else:
                isReflex = check_reflex(obstacle[idx-1], vertex, obstacle[idx+1])
            if isReflex:
                reflex_vertices.append((vertex, i))

    # build roadmap
    for i, vertex1 in enumerate(reflex_vertices):
        for j in range(i+1, len(reflex_vertices)):
            vertex2 = reflex_vertices[j]
            if vertex1[1] == vertex2[1]:  # two vertices on the same polygon
                obstacle = obstacles[vertex1[1]]
                idx1 = obstacle.index(vertex1[0])
                idx2 = obstacle.index(vertex2[0])
                # two vertices are consecutive or
                if (fabs(idx1-idx2) == 1 or idx1 in (0, len(obstacle)-1) and idx2 in (0, len(obstacle)-1)
                    or visible(vertex1[0], vertex2[0], obstacles, [obstacles[vertex1[1]]])):
                    add_to_roadmap(vertex1[0], vertex2[0], roadmap)
            else:
                curr_obstacles = [obstacles[vertex1[1]], obstacles[vertex2[1]]]
                if visible(vertex1[0], vertex2[0], obstacles, curr_obstacles):
                    add_to_roadmap(vertex1[0], vertex2[0], roadmap)

    # add start and end points to roadmap
    for point in points:
        for vertex in reflex_vertices:
            block = False
            for obstacle in obstacles:
                line1 = (point, vertex[0])
                for idx, pt in enumerate(obstacle):
                    if idx == len(obstacle)-1:
                        line2 = (pt, obstacle[0])
                    else:
                        line2 = (pt, obstacle[idx+1])
                    if vertex[0] not in line2 and intersect(line1, line2):
                        block = True
            if not block:
                add_to_roadmap(vertex[0], point, roadmap)

    # build lists of vertices and edges
    graph, vertex_list, path_list = {}, [], []
    for idx, key in enumerate(roadmap.keys()):
        graph[key] = idx+1
    for key in graph:
        vertex_list.append(str(graph[key])+':'+str(key))
        for neighbor in roadmap[key]:
            if str((graph[neighbor], graph[key])) not in path_list:
                path_list.append(str((graph[key], graph[neighbor])))

    # find shortest path using Dijkstra's algorithm
    start = Node(points[0], roadmap[points[0]], 0)
    end = Node(points[0], roadmap[points[0]])
    node_dict = {points[0]: start, points[1]: end}
    for key in roadmap.keys():
        if key != points[0] and key != points[1]:
            node_dict[key] = Node(key, roadmap[key])
    queue = list(node_dict.keys())

    while queue:
        min_dist_vertex = find_min_dist(queue, node_dict)
        graph_node = node_dict[min_dist_vertex]
        queue.remove(min_dist_vertex)
        for neighbor in graph_node.neighbors:
            if neighbor not in queue:
                continue
            edge_length = dist(min_dist_vertex, neighbor)
            neighbor_node = node_dict[neighbor]
            if neighbor_node.dist == -1 or neighbor_node.dist > graph_node.dist+edge_length:
                neighbor_node.dist = graph_node.dist+edge_length
                neighbor_node.prev = min_dist_vertex

    shortest_path = []
    last = points[1]
    while last:
        shortest_path.insert(0, str(graph[last])+':'+str(last))
        last = node_dict[last].prev

    with open("shortest_path_roadmap_output.txt", 'w') as out:
        out.write(', '.join(vertex_list)+'\n')
        out.write(', '.join(path_list)+'\n')
        out.write(', '.join(shortest_path))
