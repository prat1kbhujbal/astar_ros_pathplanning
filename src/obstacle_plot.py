import numpy as pb
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Rectangle


def check_obs(x, y, total_clearance):
    center1_x = 2
    center1_y = 2
    circle1_dia = 2
    circle1_radius = (circle1_dia / 2) + total_clearance

    center2_x = 2
    center2_y = 8
    circle2_dia = 2
    circle2_radius = (circle2_dia / 2) + total_clearance

    if (0.75 - total_clearance) <= x <= (2.25 + total_clearance):
        if (4.25 - total_clearance) <= y <= (5.75 + total_clearance):
            return False
    if (3.75 - total_clearance) <= x <= (6.25 + total_clearance):
        if (4.25 - total_clearance) <= y <= (5.75 + total_clearance):
            return False
    if (7.25 - total_clearance) <= x <= (8.75 + total_clearance):
        if (2 - total_clearance) <= y <= (6 + total_clearance):
            return False

    if ((x - center1_x)**2 + (y - center1_y) **
            2 - ((circle1_radius)**2)) <= 0:
        return False

    if ((x - center2_x)**2 + (y - center2_y) **
            2 - ((circle2_radius)**2)) <= 0:
        return False

    return True


def obstacle(start, goal, grid, clearance):
    '''Map with updated obstacles on map'''

    fig = plt.figure(figsize=(5, 3.5))
    ax = fig.subplots()
    ax.plot(goal[0], goal[1], "oy")
    ax.plot(start[0], start[1], "oy")

    major_ticks_x = pb.arange(0, 10, 1)
    minor_ticks_x = pb.arange(0, 10, 5)
    major_ticks_y = pb.arange(0, 10, 1)
    minor_ticks_y = pb.arange(0, 10, 5)

    ax.set_xticks(major_ticks_x)
    ax.set_xticks(minor_ticks_x, minor=True)
    ax.set_yticks(major_ticks_y)
    ax.set_yticks(minor_ticks_y, minor=True)

    ax.grid(which='minor', alpha=0)
    ax.grid(which='major', alpha=0)

    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)

    ax.set_aspect('equal')
    circle1 = Circle((2, 8), 1, color='k')
    ax.add_patch(circle1)
    circle2 = Circle((2, 2), 1, color='k')
    ax.add_patch(circle2)
    square = Rectangle([0.75, 4.25], 1.5, 1.5, color='k')
    ax.add_patch(square)
    rectangle1 = Rectangle([3.75, 4.25], 2.5, 1.5, color='k')
    ax.add_patch(rectangle1)
    rectangle2 = Rectangle([7.25, 2], 1.5, 2, color='k')
    ax.add_patch(rectangle2)

    return ax
