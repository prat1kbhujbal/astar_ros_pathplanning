import argparse
import numpy as pb
from obstacle_plot import *
from astar2D import *


def main():
    """Main Function"""
    Parser = argparse.ArgumentParser()
    Parser.add_argument(
        '--start', nargs='+', type=float, default=[1, 1, 0],
        help='start node. Default: [1, 1, 0]')
    Parser.add_argument(
        '--goal', nargs='+', type=float, default=[3, 5, 0],
        help='goal node. Default: [3, 5, 0]')
    Parser.add_argument(
        '--rpm', nargs='+', type=float, default=[5, 5],
        help='Left and right rpm. Default: [5, 5, 0]')
    Parser.add_argument(
        '--clearance', default=0.1,
        help='Obstacle clearance. Default: 0.1')

    Args = Parser.parse_args()
    start_node = Args.start
    goal_node = Args.goal
    robot_rpms = Args.rpm
    clearance = float(Args.clearance)

    # Map Grid
    map_grid = [10, 10]
    # Perfrom Astar
    astar(
        start_node,
        goal_node,
        map_grid,
        robot_rpms,
        clearance)


if __name__ == "__main__":
    main()
