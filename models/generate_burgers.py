import os

models_dir = os.path.expanduser('~/mrs_foraging_ws/src/multi_robot_foraging/models')
robots = ['robot1', 'robot2', 'robot3']

for i, r in enumerate(robots):
    mdir = os.path.join(models_dir, f'{r}_burger')
    os.makedirs(mdir, exist_ok=True)
    
    with open(os.path.join(mdir, 'model.config'), 'w') as f:
        f.write(f'<?xml version="1.0"?><model><name>{r}_burger</name><version>1.0</version><sdf version="1.8">model.sdf</sdf></model>')

    sdf = f'''<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="{r}">
    <link name="base_link">
      <inertial>
        <mass>1.0</mass>
        <pose>0 0 -0.02 0 0 0</pose> 
        <inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia>
      </inertial>
      <collision name="collision"><geometry><cylinder><radius>0.07</radius><length>0.10</length></cylinder></geometry></collision>
      <visual name="visual"><geometry><cylinder><radius>0.07</radius><length>0.10</length></cylinder></geometry><material><ambient>0.2 0.2 0.2 1</ambient></material></visual>
      <sensor name="camera" type="camera">
        <pose>0.05 0 0.12 0 0 0</pose>
        <camera>
          <horizontal_fov>1.089</horizontal_fov>
          <image><width>640</width><height>480</height></image>
          <clip><near>0.05</near><far>10.0</far></clip>
        </camera>
        <always_on>1</always_on><update_rate>10</update_rate><visualize>true</visualize>
        <topic>/{r}/camera/image_raw</topic>
      </sensor>
    </link>
    
    <link name="left_wheel">
      <pose>0 0.08 -0.05 -1.5708 0 0</pose>
      <inertial><mass>0.1</mass><inertia><ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz></inertia></inertial>
      <collision name="col">
        <geometry><cylinder><radius>0.033</radius><length>0.018</length></cylinder></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision>
      <visual name="vis"><geometry><cylinder><radius>0.033</radius><length>0.018</length></cylinder></geometry><material><ambient>0 0 0 1</ambient></material></visual>
    </link>
    
    <link name="right_wheel">
      <pose>0 -0.08 -0.05 -1.5708 0 0</pose>
      <inertial><mass>0.1</mass><inertia><ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz></inertia></inertial>
      <collision name="col">
        <geometry><cylinder><radius>0.033</radius><length>0.018</length></cylinder></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision>
      <visual name="vis"><geometry><cylinder><radius>0.033</radius><length>0.018</length></cylinder></geometry><material><ambient>0 0 0 1</ambient></material></visual>
    </link>
    
    <link name="caster">
      <pose>-0.05 0 -0.068 0 0 0</pose>
      <inertial><mass>0.05</mass><inertia><ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz></inertia></inertial>
      <collision name="col">
        <geometry><sphere><radius>0.015</radius></sphere></geometry>
        <surface><friction><ode><mu>0.0</mu><mu2>0.0</mu2></ode></friction></surface>
      </collision>
      <visual name="vis"><geometry><sphere><radius>0.015</radius></sphere></geometry></visual>
    </link>

    <joint name="left_wheel_joint" type="revolute"><parent>base_link</parent><child>left_wheel</child><axis><xyz>0 0 1</xyz></axis></joint>
    <joint name="right_wheel_joint" type="revolute"><parent>base_link</parent><child>right_wheel</child><axis><xyz>0 0 1</xyz></axis></joint>
    <joint name="caster_joint" type="fixed"><parent>base_link</parent><child>caster</child></joint>

    <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.16</wheel_separation>
      <wheel_radius>0.033</wheel_radius>
      <odom_publish_frequency>30</odom_publish_frequency>
      <topic>/{r}/cmd_vel</topic>
    </plugin>
  </model>
</sdf>'''
    with open(os.path.join(mdir, 'model.sdf'), 'w') as f:
        f.write(sdf)
    print("Generated HD 640x480 Burger Models")