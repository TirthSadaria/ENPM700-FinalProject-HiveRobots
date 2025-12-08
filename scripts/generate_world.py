#!/usr/bin/env python3

"""
Generate Webots world file with only the requested number of robots.
This script creates a temporary world file with only num_robots robots.
"""

import sys
import os
import tempfile

# Predefined spawn positions (10 positions, spaced from center)
SPAWN_POSITIONS = [
    (0.0, 0.0, 0.0),      # Robot 1: Center
    (1.5, 0.0, 0.0),      # Robot 2: +X
    (-1.5, 0.0, 0.0),     # Robot 3: -X
    (0.0, 1.5, 0.0),      # Robot 4: +Y
    (0.0, -1.5, 0.0),     # Robot 5: -Y
    (1.5, 1.5, 0.0),      # Robot 6: +X +Y
    (-1.5, -1.5, 0.0),    # Robot 7: -X -Y
    (1.5, -1.5, 0.0),     # Robot 8: +X -Y
    (-1.5, 1.5, 0.0),     # Robot 9: -X +Y
    (3.0, 0.0, 0.0),      # Robot 10: +2X
]


def generate_world_file(num_robots: int, output_path: str):
    """Generate a world file with only num_robots robots."""
    
    world_content = f"""#VRML_SIM R2023b utf8

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/floors/protos/RectangleArena.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/robots/robotis/turtlebot/protos/TurtleBot3Burger.proto"

WorldInfo {{
  basicTimeStep 20
}}
Viewpoint {{
  orientation -0.5773502691896258 0.5773502691896258 0.5773502691896258 2.0944
  position -1.0 1.0 1.0
}}
TexturedBackground {{
}}
TexturedBackgroundLight {{
}}
RectangleArena {{
  floorSize 6 4
  floorTileSize 2 2
  wallThickness 0.1
  wallHeight 0.8
  wallAppearance PBRAppearance {{
    baseColorMap ImageTexture {{
      url [
        "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/default/worlds/textures/red_brick_wall.jpg"
      ]
    }}
    roughness 1
    metalness 0
  }}
}}
"""
    
    # Add robots (only up to num_robots)
    for i in range(1, num_robots + 1):
        pos = SPAWN_POSITIONS[(i - 1) % len(SPAWN_POSITIONS)]
        world_content += f"""# Robot {i}
TurtleBot3Burger {{
  translation {pos[0]} {pos[1]} {pos[2]}
  name "TurtleBot3Burger_{i}"
  controller "<extern>"
  lidarSlot [
    LDS-01 {{
      name "LDS-01"
    }}
  ]
}}
"""
    
    # Write to file
    with open(output_path, 'w') as f:
        f.write(world_content)
    
    print(f"Generated world file with {num_robots} robots: {output_path}")


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: generate_world.py <num_robots> <output_path>")
        sys.exit(1)
    
    num_robots = int(sys.argv[1])
    output_path = sys.argv[2]
    
    if num_robots < 1 or num_robots > 10:
        print("Error: num_robots must be between 1 and 10")
        sys.exit(1)
    
    generate_world_file(num_robots, output_path)

