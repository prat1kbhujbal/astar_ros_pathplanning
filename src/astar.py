import numpy as pb
import matplotlib.pyplot as plt
from queue import PriorityQueue
from obstacle_plot import *
import math


class Astar:
    """Class for AStar
    """

    def __init__(
            self,
            position,
            cost,
            parent,
            paths,
            actions,
            wheel_radius=0.038,
            wheel_distance=0.354):
        self.position = position
        self.cost = cost
        self.parent = parent
        self.actions = actions
        self.paths = paths
        self.wheel_radius = wheel_radius
        self.wheel_distance = wheel_distance


def verify_node(node, clearance):
    """Check for borders with robot and obstacle clearance

    Args:
        node: Current Node
        clearance (int): Total Clearance

    Returns:
        Bool: boolean value
    """
    px = node[0]
    py = node[1]

    if px < clearance:
        return False
    if py < clearance:
        return False
    if px >= (10 - clearance):
        return False
    if py >= (10 - clearance):
        return False
    return True


def solvable(start_node, goal_node, total_clearance):
    """Check if goal/start node inside the obstacle or outside of map"""
    if start_node[0] > 10 or start_node[0] < 0 or start_node[1] > 10 or start_node[
            1] < 0 or goal_node[0] > 10 or goal_node[0] < 0 or goal_node[1] > 10 or goal_node[1] < 0:
        print("Start Node/Goal Node is outside the Map!! Please provide valid nodes.")
        return False
    if not check_obs(
            start_node[0],
            start_node[1],
            total_clearance) or not check_obs(
            goal_node[0],
            goal_node[1],
            total_clearance):
        print("Start Node/Goal Node is inside the obstacle!! Please provide valid nodes.")
        return False
    return True


def actions(node, step_size, rpms):
    """Explore paths
    Returns:
        float array: Explored paths
    """
    '''Explore paths'''
    wv_l, wv_r = rpms
    action = [(move(node, wv_l, wv_r, step_size)),
              (move(node, wv_l, 0, step_size)),
              (move(node, 0, wv_r, step_size)),
              (move(node, wv_l, wv_r + 5, step_size)),
              (move(node, wv_l + 5, wv_r, step_size)),
              (move(node, wv_l + 5, wv_r + 5, step_size)),
              (move(node, 0, wv_r + 5, step_size)),
              (move(node, wv_l + 5, 0, step_size))]
    return action


def move(node, wv_l, wv_r, step_size):
    t = 0
    dt = 0.1
    paths = []
    x, y, th = node.position
    th = pb.deg2rad(th)

    Xn = x
    Yn = y
    thetan = th
    paths.append([Xn, Yn])
    cost = 0.0
    while t < step_size:
        t = t + dt
        dx = 0.5 * node.wheel_radius * (wv_l + wv_r) * math.cos(thetan) * dt
        dy = 0.5 * node.wheel_radius * (wv_l + wv_r) * math.sin(thetan) * dt
        Xn += dx
        Yn += dy
        thetan += (node.wheel_radius / node.wheel_distance) * (wv_l - wv_r) * dt
        cost += math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
        paths.append([Xn, Yn])

    thetan = int(pb.rad2deg(thetan))
    if (thetan >= 360):
        thetan -= 360
    if (thetan <= -360):
        thetan += 360
    return (Xn, Yn, thetan), cost, paths, (wv_l, wv_r)


def heuristic(node_pos, goal_node):
    """Heuristic distance

    Args:
        node_pos : Current node position
        goal_node : Goal node

    Returns:
        float: distance
    """
    w = 1.0  # Assumed weight of heuristic
    px = node_pos[0]
    py = node_pos[1]
    dist = w * pb.hypot(px - goal_node.position[0],
                        py - goal_node.position[1])
    return dist


def planning(node, map, goal_node, step_size, rpms, clearance):
    """Returns explored paths and corresponding costs
    """

    action = actions(node, step_size, rpms)
    explore = []
    for i, path in enumerate(action):
        if verify_node(path[0], clearance):
            if check_obs(path[0][0], path[0][1], clearance):
                explore.append(
                    Astar(
                        path[0],
                        path[1] +
                        node.cost,
                        node,
                        path[2], path[3]))
    return explore


def visited_nodes(node, goal_node, narray):
    """Check if node is visited or not
    """
    node_pos = node.position
    x = node_pos[0]
    y = node_pos[1]
    th = node_pos[2]
    x = int((round(2 * x) / 2) / 0.5)
    y = int((round(2 * y) / 2) / 0.5)
    if ((node.cost + heuristic(node_pos, goal_node)) < narray[x, y, th]):
        return True
    else:
        return False


def goal_reached(node, goal_node, goal_thres):
    """Check if goal reached in goal tolerance

    Args:
        node : Current node
        goal_node : Goal node
        goal_thres : goal tolerance

    Returns:
        boolean value
    """
    node_pos = node.position
    goal_distance = (pb.hypot((node_pos[0] -
                               goal_node.position[0]),
                              (node_pos[1] -
                               goal_node.position[1])))
    if goal_distance < goal_thres:
        return True
    else:
        return False


def astar(start, goal, map_grid, rpm, clearance):
    """ AStar Algorithm
    Args:
        start : Start position
        goal : Goal position
        map_grid : Map grid size
        clearance : Clearance
    """

    if solvable(start, goal, clearance):
        ax = obstacle(start, goal, map_grid, clearance)
        map = pb.zeros((map_grid[0], map_grid[1]), pb.uint8)
        narray = pb.array([[[pb.inf for k in range(360)]
                            for j in range(int(10 / 0.5))]
                           for i in range(int(10 / 0.5))])
        cn = []
        # Goal tolerance
        goal_tolerance = 0.1
        pque = PriorityQueue()
        start_node = Astar(start, 0, None, None, None)
        goal_node = Astar(goal, 0, None, None, None)
        uniquebits = 0
        total_clearance = (start_node.wheel_distance / 2) + clearance
        pque.put((start_node.cost, uniquebits, start_node))
        while not pque.empty():

            current_node = pque.get()
            node = current_node[2]
            cn.append(node)
            if goal_reached(node, goal_node, goal_tolerance):
                print("Path Planned!!")
                goal_node.parent = node.parent
                goal_node.cost = node.cost
                break
            else:
                explortion = planning(
                    node, map, goal_node, 1, rpm, total_clearance)
                for i in explortion:
                    explore_pos = i.position
                    if visited_nodes(i, goal_node, narray):
                        narray[int((round(2 * explore_pos[0]) / 2) / 0.5),
                               int((round(2 * explore_pos[1]) / 2) / 0.5),
                               explore_pos[2]] = i.cost + (
                            heuristic(explore_pos, goal_node))
                        ncost = i.cost + heuristic(explore_pos, goal_node)
                        pque.put((ncost, uniquebits, i))
                        uniquebits += 1
        goal_node = goal_node
        nodes = []
        action_a = []
        path_f = []
        nodes.append(node)
        while goal_node:
            nodes.append(goal_node)
            if goal_node.actions is not None:
                action_a.append(goal_node.actions)
            goal_node = goal_node.parent
        nodes.reverse()
        action_a.reverse()
        for i in nodes:
            # print(i)
            Xn, Yn, ti = i.position
            anodes = i.paths
            if anodes is not None:
                for j in anodes:
                    xn1, yn1 = j
                    path_f.append([xn1, yn1])
                    plt.plot([Xn, xn1], [Yn, yn1], color="r")
                    Xn, Yn = xn1, yn1
                plt.pause(0.01)

        return action_a
