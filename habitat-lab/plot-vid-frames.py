import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import json


RUN_NUM=11


# Task 1: Draw overhead map on frame
def load_data():
    data_dir = f"./data/experiments/795-2/run-{RUN_NUM}/"

    if not Path(data_dir).exists():
        raise RuntimeError(f"Invalid run number ({RUN_NUM}) for map 795-2.")

    # Load metadata
    metadata = None
    metadata_path = f"{data_dir}/metadata.json"
    with open(metadata_path, 'r') as f:
        mdatastr = f.read()
        metadata = json.loads(mdatastr)

    # Load floor map
    floor_map = np.load(f"{data_dir}/floor_map.npy")

    # Load and store all run data
    configurations = []
    graph_points = []
    state_space = []

    for hy_i in range(4):
        config_path = f"{data_dir}/hy-{hy_i}-sim-0-configurations.npy"
        graph_path = f"{data_dir}/hy-{hy_i}-sim-0-graph-points.npy"
        ss_path = f"{data_dir}/hy-{hy_i}-sim-0-ss.npy"

        configurations.append(np.load(config_path))
        graph_points.append(np.load(graph_path))
        state_space.append(np.load(ss_path))

    return metadata, floor_map, configurations, graph_points, state_space


def plot_map(fig, ax, floor_map):
    ax.imshow(floor_map)


def main():
    metadata, floor_map, configurations, graph_points, state_space = load_data()

    fig, ax = plt.subplots()

    plot_map(fig, ax, floor_map)

    fig.show()
    plt.show()


if __name__ == '__main__':
    main()