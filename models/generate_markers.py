import os
import cv2
import cv2.aruco as aruco

models_dir = os.path.expanduser('~/mrs_foraging_ws/src/multi_robot_foraging/models')

try:
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
except AttributeError:
    dictionary = aruco.Dictionary_get(aruco.DICT_6X6_250)

markers = {0: 'aruco_home', 1: 'aruco_res1', 2: 'aruco_res2', 3: 'aruco_res3'}

for m_id, m_name in markers.items():
    mdir = os.path.join(models_dir, m_name)
    tex_dir = os.path.join(mdir, 'materials', 'textures')
    os.makedirs(tex_dir, exist_ok=True)

    # THE BEACON FIX: Make the Home marker 3x larger than the resources
    if m_id == 0:
        res = 800
        pad = 100
        box_size = "0.6 0.6 0.01" # 60cm
    else:
        res = 200
        pad = 40
        box_size = "0.2 0.2 0.01" # 20cm

    try:
        img = aruco.generateImageMarker(dictionary, m_id, res)
    except AttributeError:
        img = aruco.drawMarker(dictionary, m_id, res)
        
    img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    img_padded = cv2.copyMakeBorder(img_bgr, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=[255, 255, 255])
    
    cv2.imwrite(os.path.join(tex_dir, f'marker_{m_id}.png'), img_padded)

    with open(os.path.join(mdir, 'model.config'), 'w') as f:
        f.write(f'<?xml version="1.0"?><model><name>{m_name}</name><version>1.0</version><sdf version="1.8">model.sdf</sdf></model>')

    sdf_content = f'''<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="{m_name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><box><size>{box_size}</size></box></geometry>
        <material><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>model://{m_name}/materials/textures/marker_{m_id}.png</albedo_map></metal></pbr></material>
      </visual>
      <collision name="collision">
        <geometry><box><size>{box_size}</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>'''
    with open(os.path.join(mdir, 'model.sdf'), 'w') as f:
        f.write(sdf_content)
    print(f"Generated 3D marker for {m_name} (Size: {box_size})")