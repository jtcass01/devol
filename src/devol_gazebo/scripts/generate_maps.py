#!/usr/bin/env python3
"""
Script to extract wall positions from maze SDF files and create map.yaml files
"""

import xml.etree.ElementTree as ET
import yaml
import os
import math

def parse_sdf_walls(sdf_file):
    """Parse SDF file to extract wall positions and sizes"""
    tree = ET.parse(sdf_file)
    root = tree.getroot()
    
    walls = []
    
    # Find all links in the maze model
    for model in root.iter('model'):
        for link in model.iter('link'):
            # Get link name
            link_name = link.get('name', '')
            if 'Wall' not in link_name and 'wall' not in link_name.lower():
                continue
                
            # Get pose
            pose_elem = link.find('pose')
            if pose_elem is not None and pose_elem.text:
                pose = [float(x) for x in pose_elem.text.strip().split()]
                x, y, z, roll, pitch, yaw = pose
            else:
                x, y, z, yaw = 0, 0, 0, 0
            
            # Get box size from collision or visual geometry
            box_elem = None
            for collision in link.iter('collision'):
                box_elem = collision.find('.//box/size')
                if box_elem is not None:
                    break
            
            if box_elem is None:
                for visual in link.iter('visual'):
                    box_elem = visual.find('.//box/size')
                    if box_elem is not None:
                        break
            
            if box_elem is not None and box_elem.text:
                size = [float(x) for x in box_elem.text.strip().split()]
                sx, sy, sz = size
                
                # Account for rotation (if rotated 90 degrees, swap x and y)
                if abs(yaw) > 1.0:  # ~90 degrees in radians
                    sx, sy = sy, sx
                
                walls.append([x, y, sx, sy])
    
    return walls

def calculate_bounds(walls):
    """Calculate bounding box for all walls"""
    min_x = min(w[0] - w[2]/2 for w in walls)
    max_x = max(w[0] + w[2]/2 for w in walls)
    min_y = min(w[1] - w[3]/2 for w in walls)
    max_y = max(w[1] + w[3]/2 for w in walls)
    
    # Add buffer
    buffer = 0.5
    min_x -= buffer
    max_x += buffer
    min_y -= buffer
    max_y += buffer
    
    width = max_x - min_x
    height = max_y - min_y
    
    return min_x, min_y, width, height

def create_map_yaml(maze_name, sdf_file, output_file):
    """Create map.yaml file for a maze"""
    print(f"Processing {maze_name}...")
    
    walls = parse_sdf_walls(sdf_file)
    print(f"  Found {len(walls)} walls")
    
    if not walls:
        print(f"  WARNING: No walls found in {sdf_file}")
        return
    
    origin_x, origin_y, width, height = calculate_bounds(walls)
    
    map_data = {
        'resolution': 0.05,
        'width': round(width, 2),
        'height': round(height, 2),
        'origin_x': round(origin_x, 2),
        'origin_y': round(origin_y, 2),
        'walls': [[round(w[0], 6), round(w[1], 6), round(w[2], 6), round(w[3], 6)] for w in walls]
    }
    
    with open(output_file, 'w') as f:
        f.write(f"# Map definition for {maze_name}\n")
        f.write(f"# Resolution in meters per cell\n")
        f.write(f"resolution: {map_data['resolution']}\n\n")
        f.write(f"# Map dimensions in meters\n")
        f.write(f"width: {map_data['width']}\n")
        f.write(f"height: {map_data['height']}\n\n")
        f.write(f"# Origin (bottom-left corner) in meters\n")
        f.write(f"origin_x: {map_data['origin_x']}\n")
        f.write(f"origin_y: {map_data['origin_y']}\n\n")
        f.write(f"# Wall definitions: [center_x, center_y, size_x, size_y]\n")
        f.write(f"walls:\n")
        for wall in map_data['walls']:
            f.write(f"  - {wall}\n")
    
    print(f"  Created {output_file}")
    print(f"  Map size: {width:.2f}m x {height:.2f}m")

# Process each maze
base_dir = r"c:\EN613\gazebo_controller_ws\src\gazebo_controller\sdf"

mazes = [
    ('Maze_hr', 'Maze_hr'),
    ('Maze_ng', 'Maze_ng'),
    ('Maze_ql_1', 'Maze_ql_1')
]

for maze_folder, maze_name in mazes:
    sdf_file = os.path.join(base_dir, maze_folder, 'maze_world.sdf')
    output_file = os.path.join(base_dir, maze_folder, 'map.yaml')
    
    if os.path.exists(sdf_file):
        create_map_yaml(maze_name, sdf_file, output_file)
    else:
        print(f"WARNING: {sdf_file} not found")

print("\nDone!")
