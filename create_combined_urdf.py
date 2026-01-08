#!/usr/bin/env python3
import os
import xml.etree.ElementTree as ET

def create_combined_urdf():
    # Paths
    workspace_dir = os.path.expanduser("~/workspace")
    ur5e_urdf_path = os.path.join(workspace_dir, "ur5e/ur5e_fixed.urdf")
    gripper_urdf_path = os.path.join(workspace_dir, "robotiq_2f_85_gripper_visualization/robotiq_arg2f_85_model_fixed.urdf")
    gripper_mesh_dir = os.path.join(workspace_dir, "robotiq_2f_85_gripper_visualization") # meshes/ is inside this
    
    output_path = os.path.join(workspace_dir, "ur5e_kinematics/ur5e_robotiq.urdf")
    
    # 1. Load UR5e
    tree_ur5e = ET.parse(ur5e_urdf_path)    #  ET is the abbreviation for ElementTree, which is an XML processing module built into Python.
    root_ur5e = tree_ur5e.getroot() # parse() method: reads XML file and parses it into an ElementTree object
    
    # 2. Load Gripper
    tree_gripper = ET.parse(gripper_urdf_path)
    root_gripper = tree_gripper.getroot()
    
    # 3. Merge Content
    # Add Gripper Links and Joints to UR5e
    # Also fix mesh paths in Gripper to be absolute
    
    for child in root_gripper:
        if child.tag in ['link', 'joint', 'transmission', 'material', 'gazebo']:
            # Fix mesh paths in link/visual/geometry/mesh and link/collision/geometry/mesh
            if child.tag == 'link':
                for visual_or_col in child:
                    if visual_or_col.tag in ['visual', 'collision']:
                        for geom in visual_or_col:
                            if geom.tag == 'geometry':
                                for mesh in geom:
                                    if mesh.tag == 'mesh':
                                        filename = mesh.get('filename')
                                        if filename and not filename.startswith('/'):
                                            # It's relative, e.g. "meshes/..."
                                            abs_path = os.path.join(gripper_mesh_dir, filename)
                                            mesh.set('filename', abs_path)
            
            root_ur5e.append(child)
            
    # 4. Add Connection Joint
    # tool0 -> robotiq_arg2f_base_link
    joint = ET.Element("joint", name="ur5e_gripper_joint", type="fixed")    # Create a new XML element named "joint" which is fixed (does not move relative to its parent)
    
    parent = ET.Element("parent", link="tool0")
    child = ET.Element("child", link="robotiq_arg2f_base_link")
    # Rotate -90 deg around Y, and -90 deg around Z to align properly?
    # Usually tool0 Z is out. Gripper Z is out.
    # But often we want gripper X aligned with tool0 X or Y.
    # Let's start with identity and see. If visualization is weird, we adjust.
    # Actually, often robotic grippers need a rotation.
    # Robotiq 2F-85 often aligns Z with Z.
    # However, robotiq base frame might be different. 
    # Let's use identity for now.
    origin = ET.Element("origin", xyz="0 0 0", rpy="0 0 1.57079632679") 
    # Added 90 deg rotation around Z to align fingers likely.
    
    joint.append(parent)
    joint.append(child)
    joint.append(origin)
    
    root_ur5e.append(joint)
    
    # 5. Save
    tree_ur5e.write(output_path, encoding='utf-8', xml_declaration=True)
    print(f"Created combined URDF: {output_path}")

if __name__ == "__main__":
    create_combined_urdf()
