import trimesh
import numpy as np
import tkinter as tk
from tkinter import filedialog, simpledialog, messagebox

def select_stl_file():
    # Open a file dialog to select an STL file
    root = tk.Tk()
    root.withdraw()  # Hide the root window
    stl_file_path = filedialog.askopenfilename(
        title="Select an STL File",
        filetypes=[("STL files", "*.stl")]
    )
    return stl_file_path

def get_user_input():
    root = tk.Tk()
    root.withdraw()  # Hide the root window

    # Prompt for scale factors
    scale_factor_input = simpledialog.askstring(
        "Scale Factors", "Enter scale factors as 'x y z':", parent=root
    )
    if not scale_factor_input:
        messagebox.showerror("Input Error", "Scale factors are required.")
        return None, None, None
    
    # Convert scale factors to a tuple of floats
    try:
        scale_factors = tuple(map(float, scale_factor_input.split()))
        if len(scale_factors) != 3:
            raise ValueError
    except ValueError:
        messagebox.showerror("Input Error", "Please enter three valid numbers for scale factors.")
        return None, None, None

    # Prompt for rpy values
    rpy_input = simpledialog.askstring(
        "Rotation (RPY)", "Enter RPY angles in degrees as 'roll pitch yaw':", parent=root
    )
    if not rpy_input:
        messagebox.showerror("Input Error", "RPY values are required.")
        return None, None, None
    
    # Convert rpy values to a tuple of floats
    try:
        rpy = tuple(map(float, rpy_input.split()))
        if len(rpy) != 3:
            raise ValueError
    except ValueError:
        messagebox.showerror("Input Error", "Please enter three valid numbers for RPY.")
        return None, None, None

    # Prompt for xyz values (not processed)
    xyz_input = simpledialog.askstring(
        "XYZ Position", "Enter XYZ position as 'x y z':", parent=root
    )
    if not xyz_input:
        messagebox.showerror("Input Error", "XYZ values are required.")
        return None, None, None
    
    # Return scale, rpy, and xyz values
    xyz = tuple(map(float, xyz_input.split()))
    return scale_factors, rpy, xyz

def create_rotation_matrix(rpy):
    # Convert rpy (roll, pitch, yaw) to radians
    roll, pitch, yaw = np.radians(rpy)
    # Calculate individual rotation matrices
    Rx = np.array([[1, 0, 0, 0],
                   [0, np.cos(roll), -np.sin(roll), 0],
                   [0, np.sin(roll), np.cos(roll), 0],
                   [0, 0, 0, 1]])
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch), 0],
                   [0, 1, 0, 0],
                   [-np.sin(pitch), 0, np.cos(pitch), 0],
                   [0, 0, 0, 1]])
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0, 0],
                   [np.sin(yaw), np.cos(yaw), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    # Combine rotations in ZYX order
    return Rz @ Ry @ Rx

def create_collision_box():
    # Select STL file
    stl_file_path = select_stl_file()
    if not stl_file_path:
        print("No file selected.")
        return

    # Get scale factors, rpy, and xyz from user
    scale_factors, rpy, xyz = get_user_input()
    if scale_factors is None or rpy is None or xyz is None:
        return

    # Load the STL mesh
    mesh = trimesh.load_mesh(stl_file_path)

    # Apply scaling
    scale_matrix = np.diag(scale_factors + (1,))
    mesh.apply_transform(scale_matrix)

    # Apply rotation based on rpy values
    rotation_matrix = create_rotation_matrix(rpy)
    mesh.apply_transform(rotation_matrix)

    # Find the axis-aligned bounding box size for collision
    bounding_box = mesh.bounding_box_oriented
    extents = bounding_box.extents

    # Display URDF collision tag format
    collision_urdf = f"""
    <collision>
      <geometry>
        <box>
          <size>{extents[0]:.6f} {extents[1]:.6f} {extents[2]:.6f}</size>
        </box>
      </geometry>
      <origin xyz="{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}" 
              rpy="{np.radians(rpy[0]):.6f} {np.radians(rpy[1]):.6f} {np.radians(rpy[2]):.6f}"/>
    </collision>
    """

    print("Collision URDF Tag:\n", collision_urdf)
    return collision_urdf

create_collision_box()