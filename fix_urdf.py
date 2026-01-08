#!/usr/bin/env python3
"""Fix URDF mesh paths to use local files."""

import xml.etree.ElementTree as ET
from pathlib import Path
import sys
import os

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
                if 'ur5e' in parts:
                    idx = parts.index('ur5e')
                    relative_path = '/'.join(parts[idx+1:])
                else:
                    # Fallback to last 2 parts if structure matches visual/xxx.dae
                    relative_path = '/'.join(parts[-2:])
                
                # Check if we should make it absolute or relative to URDF
                # Here we assume we want relative to URDF if mesh_dir is relative to URDF
                # But typically this script fixed it to absolute or relative.
                # Let's make it relative to the mesh_dir provided.
                
                # If we want the path in URDF to be "visual/base.dae", we just set it.
                # But this script seems designed to find the file.
                
                new_path = relative_path
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
    script_dir = Path(__file__).parent.resolve()
    urdf_path = script_dir / 'assets/ur5e/ur5e.urdf'
    mesh_dir = script_dir / 'assets/ur5e'

    if not urdf_path.exists():
        print(f"URDF not found: {urdf_path}")
        # Try to find it in case it hasn't been moved yet
        return

    print(f"Fixing URDF: {urdf_path}")
    
    # We update in place or create a new one. 
    # The previous fix_urdf created _fixed.urdf. 
    # Let's just update the file itself or create a fixed one.
    fix_urdf_mesh_paths(urdf_path, mesh_dir, output_path=urdf_path)

if __name__ == '__main__':
    main()
