# examples/python/visualization/interactive_visualization.py


import numpy as np

import copy

import open3d as o3d



def demo_crop_geometry(data_folder="data-1050"):
    print("Demo for manual geometry cropping")
    print(
        "1) Press 'Y' twice to align geometry with negative direction of y-axis"

    )
    print("2) Press 'K' to lock screen and to switch to selection mode")
    print("3) Drag for rectangle selection,")
    print("   or use ctrl + left click for polygon selection")
    print("4) Press 'C' to get a selected geometry")
    print("5) Press 'S' to save the selected geometry")
    print("6) Press 'F' to switch to freeview mode")
    pcd = o3d.io.read_point_cloud(f"./{data_folder}/cropped.ply")
    o3d.visualization.draw_geometries_with_editing([pcd])

if __name__ == "__main__":
    demo_crop_geometry(data_folder="data-1050")
