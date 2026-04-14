"""
gazebo_trail_node.py
====================
Spawns tiny colored cylinders in Gazebo as robots drive,
creating visible physical trail paths in the Gazebo window.

Queue-based: subscribes to /robotN/trail_pos (Point from unified_agent).
A timer drains ONE spawn every 0.3s to avoid overwhelming Gazebo
entity factory (which can only reliably handle ~3 spawns/second).

Colors match robot bodies and RViz pheromone trails:
  Robot 1 → GREEN  (North arm — winner)
  Robot 2 → RED    (East arm, then North in iter 3)
  Robot 3 → BLUE   (West arm, then North in iter 3)

Dot size grows per iteration:
  Iter 1 → 0.030m  Iter 2 → 0.050m  Iter 3 → 0.075m

In iteration 3 all three converge on the North arm →
you see three overlapping colored dots on x=0, y=0→1.5
showing ACO convergence directly in Gazebo.

SETUP.PY REQUIREMENT — add to entry_points console_scripts:
  'gazebo_trail_node = multi_robot_foraging.gazebo_trail_node:main',
"""

import os
import math
import tempfile
import subprocess
import collections
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

TRAIL_COLOR = {
    1: (0.05, 0.75, 0.15),   # Green
    2: (0.85, 0.15, 0.08),   # Red
    3: (0.15, 0.35, 0.90),   # Blue
}
DOT_RADIUS  = {1: 0.030, 2: 0.050, 3: 0.075}
MIN_SPACING = 0.20          # metres between dots
SPAWN_TICK  = 0.30          # seconds between spawns
WORLD_NAME  = 'foraging_arena'


def make_sdf(name, x, y, r, g, b, radius):
    return (
        f"<?xml version='1.0'?><sdf version='1.8'>"
        f"<model name='{name}'><static>true</static>"
        f"<pose>{x:.4f} {y:.4f} 0.007 0 0 0</pose>"
        f"<link name='l'><visual name='v'>"
        f"<geometry><cylinder>"
        f"<radius>{radius:.4f}</radius><length>0.010</length>"
        f"</cylinder></geometry>"
        f"<material>"
        f"<ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient>"
        f"<diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>"
        f"</material></visual></link></model></sdf>"
    )


def cleanup_file(path, delay=3.0):
    def _rm():
        import time; time.sleep(delay)
        try: os.unlink(path)
        except: pass
    threading.Thread(target=_rm, daemon=True).start()


class GazeboTrailNode(Node):

    def __init__(self):
        super().__init__('gazebo_trail_node')

        for rn in (1, 2, 3):
            self.create_subscription(
                Point, f'/robot{rn}/trail_pos', self.trail_cb, 20
            )

        self.last_pos    = {}          # (rid, it) → (x, y)
        self.spawn_queue = collections.deque()
        self.counter     = 0

        self.create_timer(SPAWN_TICK, self.drain_queue)

        self.get_logger().info(
            f'GazeboTrailNode | spacing={MIN_SPACING}m | '
            f'rate={1/SPAWN_TICK:.1f} spawns/s | '
            f'GREEN=R1  RED=R2  BLUE=R3'
        )

    def trail_cb(self, msg: Point):
        code = int(round(msg.z))
        rid  = code // 10
        it   = code  % 10
        if rid not in (1, 2, 3) or it not in (1, 2, 3):
            return

        x, y = msg.x, msg.y
        key  = (rid, it)

        if key in self.last_pos:
            lx, ly = self.last_pos[key]
            if math.hypot(x - lx, y - ly) < MIN_SPACING:
                return

        self.last_pos[key] = (x, y)
        name = f'td_r{rid}i{it}_{self.counter:05d}'
        self.counter += 1
        self.spawn_queue.append((name, x, y, rid, it))

    def drain_queue(self):
        """Pop and spawn ONE dot per timer tick."""
        if not self.spawn_queue:
            return

        name, x, y, rid, it = self.spawn_queue.popleft()
        r, g, b  = TRAIL_COLOR[rid]
        radius   = DOT_RADIUS[it]
        sdf      = make_sdf(name, x, y, r, g, b, radius)

        try:
            tf = tempfile.NamedTemporaryFile(
                mode='w', suffix='.sdf', delete=False,
                prefix='/tmp/gz_dot_'
            )
            tf.write(sdf); tf.close()
            subprocess.Popen(
                ['ros2', 'run', 'ros_gz_sim', 'create',
                 '-world', WORLD_NAME, '-name', name, '-file', tf.name],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            cleanup_file(tf.name)
        except Exception as e:
            self.get_logger().warn(f'Spawn failed {name}: {e}')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GazeboTrailNode())


if __name__ == '__main__':
    main()