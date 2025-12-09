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
  position 0.0 0.0 8.0
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

# ============================================
# OBSTACLES - Added for SLAM feature detection
# These break the aliasing problem of identical walls
# ============================================

# Central pillar REMOVED - robot spawn point is at center

# Pillar top-left
DEF PILLAR_TL Solid {{
  translation -1.5 1.0 0.25
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.8 0.2 0.2
        roughness 0.8
        metalness 0.2
      }}
      geometry Cylinder {{
        height 0.5
        radius 0.12
      }}
    }}
  ]
  name "pillar_tl"
  boundingObject Cylinder {{
    height 0.5
    radius 0.12
  }}
}}

# Pillar top-right
DEF PILLAR_TR Solid {{
  translation 1.5 1.0 0.25
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.2 0.8 0.2
        roughness 0.8
        metalness 0.2
      }}
      geometry Cylinder {{
        height 0.5
        radius 0.12
      }}
    }}
  ]
  name "pillar_tr"
  boundingObject Cylinder {{
    height 0.5
    radius 0.12
  }}
}}

# Pillar bottom-left
DEF PILLAR_BL Solid {{
  translation -1.5 -1.0 0.25
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.2 0.2 0.8
        roughness 0.8
        metalness 0.2
      }}
      geometry Cylinder {{
        height 0.5
        radius 0.12
      }}
    }}
  ]
  name "pillar_bl"
  boundingObject Cylinder {{
    height 0.5
    radius 0.12
  }}
}}

# Pillar bottom-right
DEF PILLAR_BR Solid {{
  translation 1.5 -1.0 0.25
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.8 0.8 0.2
        roughness 0.8
        metalness 0.2
      }}
      geometry Cylinder {{
        height 0.5
        radius 0.12
      }}
    }}
  ]
  name "pillar_br"
  boundingObject Cylinder {{
    height 0.5
    radius 0.12
  }}
}}

# Box obstacle 1 (near left wall)
DEF BOX1 Solid {{
  translation -2.2 0.5 0.15
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.6 0.4 0.2
        roughness 0.9
        metalness 0.1
      }}
      geometry Box {{
        size 0.4 0.3 0.3
      }}
    }}
  ]
  name "box1"
  boundingObject Box {{
    size 0.4 0.3 0.3
  }}
}}

# Box obstacle 2 (near right wall)
DEF BOX2 Solid {{
  translation 2.2 -0.5 0.15
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.4 0.6 0.2
        roughness 0.9
        metalness 0.1
      }}
      geometry Box {{
        size 0.3 0.4 0.3
      }}
    }}
  ]
  name "box2"
  boundingObject Box {{
    size 0.3 0.4 0.3
  }}
}}

# L-shaped obstacle (top area)
DEF L_WALL1 Solid {{
  translation 0.5 1.5 0.2
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.3 0.3 0.6
        roughness 0.8
        metalness 0.1
      }}
      geometry Box {{
        size 0.8 0.1 0.4
      }}
    }}
  ]
  name "l_wall1"
  boundingObject Box {{
    size 0.8 0.1 0.4
  }}
}}

DEF L_WALL2 Solid {{
  translation 0.85 1.25 0.2
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.3 0.3 0.6
        roughness 0.8
        metalness 0.1
      }}
      geometry Box {{
        size 0.1 0.6 0.4
      }}
    }}
  ]
  name "l_wall2"
  boundingObject Box {{
    size 0.1 0.6 0.4
  }}
}}

# Small pillar array (bottom area) - distinctive pattern
DEF SMALL_PILLAR1 Solid {{
  translation -0.5 -1.2 0.15
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.9 0.5 0.1
        roughness 0.7
        metalness 0.3
      }}
      geometry Cylinder {{
        height 0.3
        radius 0.08
      }}
    }}
  ]
  name "small_pillar1"
  boundingObject Cylinder {{
    height 0.3
    radius 0.08
  }}
}}

DEF SMALL_PILLAR2 Solid {{
  translation -0.2 -1.4 0.15
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.9 0.5 0.1
        roughness 0.7
        metalness 0.3
      }}
      geometry Cylinder {{
        height 0.3
        radius 0.08
      }}
    }}
  ]
  name "small_pillar2"
  boundingObject Cylinder {{
    height 0.3
    radius 0.08
  }}
}}

DEF SMALL_PILLAR3 Solid {{
  translation 0.1 -1.2 0.15
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.9 0.5 0.1
        roughness 0.7
        metalness 0.3
      }}
      geometry Cylinder {{
        height 0.3
        radius 0.08
      }}
    }}
  ]
  name "small_pillar3"
  boundingObject Cylinder {{
    height 0.3
    radius 0.08
  }}
}}
"""
    
    # Add robots (only up to num_robots)
    # Include lidar 180° rotation to correct Webots lidar orientation
    for i in range(1, num_robots + 1):
        pos = SPAWN_POSITIONS[(i - 1) % len(SPAWN_POSITIONS)]
        world_content += f"""# Robot {i}
TurtleBot3Burger {{
  translation {pos[0]} {pos[1]} {pos[2]}
  name "TurtleBot3Burger_{i}"
  controller "<extern>"
  lidarSlot [
    LDS-01 {{
      translation 0 0 0
      rotation 0 0 1 3.14159
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

