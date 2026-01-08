#!/usr/bin/env python3
"""Fix URDF mesh paths to use local files."""

import xml.etree.ElementTree as ET
from pathlib import Path
import sys

def fix_urdf_mesh_paths(urdf_path, mesh_dir, output_path=None):
    """Replace package:// paths with local paths in URDF."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    mesh_dir = Path(mesh_dir)

    # Find all mesh elements
    mesh_elements = root.findall('.//mesh')
    if not mesh_elements:
        mesh_elements = root.findall('.//{*}mesh')  # Handle namespaces

    for elem in mesh_elements:
        filename = elem.get('filename')
        if filename and filename.startswith('package://'):
            # Extract relative path after ur5e/
            # package://ur_description/meshes/ur5e/visual/base.dae
            # -> visual/base.dae
            parts = filename.split('/')
            try:
                idx = parts.index('ur5e')
                relative_path = '/'.join(parts[idx+1:])
                new_path = str(mesh_dir / relative_path)

                # Check if file exists
                if not Path(new_path).exists():
                    print(f"Warning: Mesh file not found: {new_path}")
                    # Try alternative: just filename
                    filename_only = parts[-1]
                    alt_path = str(mesh_dir / filename_only)
                    if Path(alt_path).exists():
                        new_path = alt_path
                        print(f"  Using alternative path: {alt_path}")

                elem.set('filename', new_path)
                print(f"Fixed: {filename} -> {new_path}")
            except ValueError:
                print(f"Warning: Could not parse path {filename}")

    # Save modified URDF
    if output_path is None:
        output_path = urdf_path.parent / (urdf_path.stem + '_fixed.urdf')

    tree.write(output_path, encoding='utf-8', xml_declaration=True)
    print(f"\nFixed URDF saved to: {output_path}")
    return output_path

def main():
    urdf_path = Path('/home/themountaintree/workspace/ur5e/ur5e.urdf')
    mesh_dir = Path('/home/themountaintree/workspace/ur5e/ur5e')

    if not urdf_path.exists():
        print(f"URDF not found: {urdf_path}")
        sys.exit(1)

    if not mesh_dir.exists():
        print(f"Mesh directory not found: {mesh_dir}")
        sys.exit(1)

    print(f"Fixing URDF: {urdf_path}")
    print(f"Mesh directory: {mesh_dir}")

    fixed_path = fix_urdf_mesh_paths(urdf_path, mesh_dir)

    # Also check for collision meshes in collision/ directory
    collision_dir = mesh_dir / 'collision'
    if collision_dir.exists():
        print(f"\nCollision directory found: {collision_dir}")
        # Update collision mesh paths
        tree = ET.parse(fixed_path)
        root = tree.getroot()

        collision_meshes = root.findall('.//collision//mesh')
        for elem in collision_meshes:
            filename = elem.get('filename')
            if filename and filename.endswith('.stl'):
                # Find the .stl file
                stl_name = Path(filename).name
                stl_path = collision_dir / stl_name
                if stl_path.exists():
                    elem.set('filename', str(stl_path))
                    print(f"Fixed collision mesh: {stl_name}")

        tree.write(fixed_path, encoding='utf-8', xml_declaration=True)
        print(f"Updated collision meshes in: {fixed_path}")

if __name__ == '__main__':
    main()