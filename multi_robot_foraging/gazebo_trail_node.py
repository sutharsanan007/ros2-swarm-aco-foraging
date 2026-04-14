"""
gazebo_trail_node.py
====================
Spawns tiny colored flat cylinders in Gazebo at each robot's commanded
position as it moves, creating a physical colored trail that's visible
directly in the Gazebo window — no RViz needed for the path display.

Colors (match RViz pheromone trails and robot body colors):
  Robot 1 → GREEN  (0.1, 0.8, 0.2)  — shortest path / winner
  Robot 2 → RED    (0.9, 0.2, 0.1)  — longest path
  Robot 3 → BLUE   (0.2, 0.4, 0.9)  — medium path

Trail dots are spawned:
  - Every 0.22m of travel (sampled from commanded odometry position)
  - In BOTH drive directions (north and south) — shows full path, not just return
  - Iteration 3 trail dots are larger (0.055m radius vs 0.035m) so the
    converged Robot 1 lane stands out visually

Spawning method:
  Uses 'ros2 run ros_gz_sim create' via subprocess — same mechanism as the
  launch file uses to spawn the robots themselves. Proven reliable.
  Each dot is a static SDF model written to a temp file, then spawned.
  stdout/stderr suppressed so the log stays clean.

Total dot count:
  ~8 dots per metre × ~9m per robot total (3 iters × 3m avg) ≈ 72 dots × 3 robots
  ≈ 216 static models. Gazebo handles this easily (no physics on static models).
"""

import os
import math
import tempfile
import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

# Robot trail colors — RGBA matching robot body and RViz trail colors
TRAIL_COLOR = {
    1: (0.1, 0.8, 0.2),   # Green  (Robot 1 — winner)
    2: (0.9, 0.2, 0.1),   # Red    (Robot 2)
    3: (0.2, 0.4, 0.9),   # Blue   (Robot 3)
}

# Dot sizes per iteration — grows each iteration to show reinforcement
DOT_RADIUS = {1: 0.030, 2: 0.040, 3: 0.055}

# Minimum travel between dots (metres)
MIN_SPACING = 0.22

# Gazebo world name (must match foraging_arena.world)
WORLD_NAME = 'foraging_arena'


def make_sdf(name: str, x: float, y: float,
             r: float, g: float, b: float, radius: float) -> str:
    return f"""<?xml version='1.0'?>
<sdf version='1.8'>
  <model name='{name}'>
    <static>true</static>
    <pose>{x:.4f} {y:.4f} 0.008 0 0 0</pose>
    <link name='link'>
      <visual name='vis'>
        <geometry>
          <cylinder>
            <radius>{radius:.4f}</radius>
            <length>0.012</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>{r:.3f} {g:.3f} {b:.3f} 1.0</ambient>
          <diffuse>{r:.3f} {g:.3f} {b:.3f} 0.95</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


class GazeboTrailNode(Node):

    def __init__(self):
        super().__init__('gazebo_trail_node')

        # Subscribe to trail_pos from all three robots
        for rn in (1, 2, 3):
            self.create_subscription(
                Point,
                f'/robot{rn}/trail_pos',
                self.trail_cb,
                10
            )

        # Track last spawned position per (robot_id, iteration) bucket
        self.last_pos: dict = {}

        # Running marker counter for unique model names
        self.counter = 0

        self.get_logger().info(
            'GazeboTrailNode online — '
            'will spawn colored dots in Gazebo as robots move. '
            'Robot1=GREEN, Robot2=RED, Robot3=BLUE. '
            f'Spacing={MIN_SPACING}m, world="{WORLD_NAME}"'
        )

    # ================================================================== #

    def trail_cb(self, msg: Point):
        """Spawn a colored dot whenever the robot has moved MIN_SPACING metres."""
        code = int(round(msg.z))
        rid  = code // 10
        it   = code  % 10

        if rid not in (1, 2, 3) or it not in (1, 2, 3):
            return

        x, y = msg.x, msg.y
        key  = (rid, it)

        # Only spawn if robot has moved enough since the last dot
        if key in self.last_pos:
            lx, ly = self.last_pos[key]
            if math.hypot(x - lx, y - ly) < MIN_SPACING:
                return

        self.last_pos[key] = (x, y)

        r, g, b = TRAIL_COLOR[rid]
        radius  = DOT_RADIUS[it]
        name    = f'trail_r{rid}i{it}_{self.counter:04d}'
        self.counter += 1

        sdf = make_sdf(name, x, y, r, g, b, radius)

        # Write to a temp file — avoids quoting/escaping issues in subprocess
        try:
            with tempfile.NamedTemporaryFile(
                mode='w', suffix='.sdf', delete=False,
                prefix='/tmp/gz_trail_'
            ) as f:
                f.write(sdf)
                fname = f.name

            subprocess.Popen(
                [
                    'ros2', 'run', 'ros_gz_sim', 'create',
                    '-world', WORLD_NAME,
                    '-name',  name,
                    '-file',  fname,
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
        except Exception as e:
            self.get_logger().warn(f'Trail spawn failed for {name}: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = GazeboTrailNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()