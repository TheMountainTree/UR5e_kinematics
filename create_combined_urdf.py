#!/usr/bin/env python3
import os
import xml.etree.ElementTree as ET

def create_combined_urdf():
    # Paths relative to this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    assets_dir = os.path.join(script_dir, "assets")
    
    ur5e_urdf_path = os.path.join(assets_dir, "ur5e/ur5e.urdf")
    gripper_urdf_path = os.path.join(assets_dir, "robotiq_2f_85/robotiq_2f_85.urdf")
    
    output_path = os.path.join(script_dir, "ur5e_robotiq.urdf")
    
    print(f"Loading UR5e from: {ur5e_urdf_path}")
    tree_ur5e = ET.parse(ur5e_urdf_path)
    root_ur5e = tree_ur5e.getroot()
    
    print(f"Loading Gripper from: {gripper_urdf_path}")
    tree_gripper = ET.parse(gripper_urdf_path)
    root_gripper = tree_gripper.getroot()
    
    # 3. Merge Content
    
    # Update UR5e paths
    # Expecting relative paths "visual/..." or "collision/..." in ur5e.urdf
    # We want them to be "assets/ur5e/visual/..." in the combined URDF (which is in script_dir)
    for child in root_ur5e.iter():
        if child.tag == 'mesh':
            filename = child.get('filename')
            if filename and not filename.startswith('/') and not filename.startswith('package://'):
                 # Assume relative path
                 new_path = os.path.join("assets/ur5e", filename)
                 child.set('filename', new_path)

    # Add Gripper Links and Joints to UR5e
    for child in root_gripper:
        if child.tag in ['link', 'joint', 'transmission', 'material', 'gazebo']:
            # Fix mesh paths in link/visual/geometry/mesh and link/collision/geometry/mesh
            # Expecting "meshes/..." in gripper URDF
            # We want "assets/robotiq_2f_85/meshes/..."
            if child.tag == 'link':
                for visual_or_col in child:
                    if visual_or_col.tag in ['visual', 'collision']:
                        for geom in visual_or_col:
                            if geom.tag == 'geometry':
                                for mesh in geom:
                                    if mesh.tag == 'mesh':
                                        filename = mesh.get('filename')
                                        if filename and not filename.startswith('/') and not filename.startswith('package://'):
                                            new_path = os.path.join("assets/robotiq_2f_85", filename)
                                            mesh.set('filename', new_path)
            
            root_ur5e.append(child)
            
    # 4. Add Connection Joint
    # tool0 -> robotiq_arg2f_base_link
    joint = ET.Element("joint", name="ur5e_gripper_joint", type="fixed")
    
    parent = ET.Element("parent", link="tool0")
    child = ET.Element("child", link="robotiq_arg2f_base_link")
    # Rotate 90 deg around Z
    origin = ET.Element("origin", xyz="0 0 0", rpy="0 0 1.57079632679") 
    
    joint.append(parent)
    joint.append(child)
    joint.append(origin)
    
    root_ur5e.append(joint)
    
    # 5. Save
    tree_ur5e.write(output_path, encoding='utf-8', xml_declaration=True)
    print(f"Created combined URDF: {output_path}")

if __name__ == "__main__":
    create_combined_urdf()
