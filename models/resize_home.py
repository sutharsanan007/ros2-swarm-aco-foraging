import os
import re

sdf_path = os.path.expanduser('~/mrs_foraging_ws/src/multi_robot_foraging/models/aruco_home/model.sdf')

with open(sdf_path, 'r') as file:
    content = file.read()

# Replace the tiny 20cm box with a massive 60cm box
content = re.sub(r'<size>.*?</size>', '<size>0.6 0.6 0.02</size>', content)

with open(sdf_path, 'w') as file:
    file.write(content)

print("Home Base upgraded to 60cm Monolith!")