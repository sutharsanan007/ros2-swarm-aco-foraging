import os
import shutil
import xml.etree.ElementTree as ET

models_dir = os.path.expanduser('~/mrs_foraging_ws/src/multi_robot_foraging/models')
tb3_model_dir = '/opt/ros/jazzy/share/turtlebot3_gazebo/models/turtlebot3_waffle'

print("--- Patching Robot Models ---")

for i in range(1, 4):
    r_name = f'robot{i}'
    target_dir = os.path.join(models_dir, f'{r_name}_waffle')
    
    if os.path.exists(target_dir):
        shutil.rmtree(target_dir)
    shutil.copytree(tb3_model_dir, target_dir)

    # 1. Update the config file
    conf_path = os.path.join(target_dir, 'model.config')
    with open(conf_path, 'r') as f: data = f.read()
    with open(conf_path, 'w') as f: f.write(data.replace('<name>turtlebot3_waffle</name>', f'<name>{r_name}_waffle</name>'))

    sdf_path = os.path.join(target_dir, 'model.sdf')
    
    # 2. Update the velocity topic
    with open(sdf_path, 'r') as f: text = f.read()
    text = text.replace('<topic>cmd_vel</topic>', f'<topic>/{r_name}/cmd_vel</topic>')
    with open(sdf_path, 'w') as f: f.write(text)

    # 3. SAFELY parse XML to fix the Gazebo Camera Collision Bug
    tree = ET.parse(sdf_path)
    root = tree.getroot()

    for model in root.iter('model'):
        if model.get('name') == 'turtlebot3_waffle':
            model.set('name', r_name)

    for sensor in root.iter('sensor'):
        if sensor.get('name') == 'camera':
            sensor.set('name', f'{r_name}_camera') # Unique sensor name
            topic = sensor.find('topic')
            if topic is None:
                topic = ET.SubElement(sensor, 'topic')
            topic.text = f'/{r_name}/camera/image_raw' # Hardcoded unique topic
            
        elif sensor.get('name') == 'hls_lfcd_lds':
            sensor.set('name', f'{r_name}_lidar')
            topic = sensor.find('topic')
            if topic is None:
                topic = ET.SubElement(sensor, 'topic')
            topic.text = f'/{r_name}/scan'

    tree.write(sdf_path, encoding='utf-8', xml_declaration=True)
    print(f"[OK] Patched XML cameras for {r_name}")