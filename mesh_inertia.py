import trimesh
import tkinter as teke
from tkinter import filedialog

def locateSTL():
    rootObj = teke.Tk()
    rootObj.withdraw()

    file_path = filedialog.askopenfilename(
        title="Select an STL file",
        filetypes=[("STL files", "*.stl")]
    )
    
    return file_path

def calculate_inertia(file_path):
    mesh = trimesh.load(file_path)

    mass = 0.1*mesh.volume
    inertia = mesh.moment_inertia

    return mass, inertia

if __name__ == "__main__":
    stl_file = locateSTL()
    
    if stl_file:
        mass, inertia = calculate_inertia(stl_file)
        
        print("Mass:", mass)
        print("Inertia Tensor:", inertia)
    else:
        print("No file selected.")
