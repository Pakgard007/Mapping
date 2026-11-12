import os
import sys
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# RViz ของงานทำแผนที่ หน้าต่างเดียว: display ทั้งหมดจาก rviz2.rviz ของ LIO-SAM
# (อ่านใหม่ทุกครั้งที่เปิด แก้ที่ไฟล์นั้นไฟล์เดียวพอ) + panel ปุ่มเปิด/ปิด + โมเดลรถกอล์ฟ
# ขนาดจริง (ขนาดแก้ที่ cart/cart.yaml)
# theme:=true (ค่าเริ่มต้น) = หน้าต่างธีมมืด, แผนที่สีเรียบ, สแกนล่าสุดสีสด, เส้นทางเป็นแถบ,
#   กล้องตามหลังรถ, panel กับ Displays เป็นแท็บคู่กัน, ซ่อนแกน TF
# theme:=false = หน้าตาเดิมของ rviz2.rviz (เพิ่มแค่ panel กับโมเดลรถ)
# ปุ่มขั้นที่ 2 เปิด LIO-SAM ด้วย lio_sam.launch.py ของแพ็กเกจนี้ ซึ่งไม่เปิด RViz ซ้ำ
# ขั้นตอนที่เปิดจากปุ่มรันในหน้าต่าง terminal ของตัวเอง ปิด RViz นี้แล้วยังรันต่อ

PANEL = {'Class': 'mapping_rviz_panel/MappingPanel', 'Name': 'Mapping'}
CART_DIR = os.path.join(get_package_share_directory('mapping_rviz_panel'), 'cart')

# Displays are found by topic: rviz2.rviz has two called PointCloud2.
# The map is the backdrop: muted blue by intensity instead of a height rainbow,
# so the scan being matched, the paths and the cart stand out on it. The bounds
# are fixed (an Ouster's signal runs 0 to a few thousand, half of it under 700)
# so the colours do not shift every time the map is republished.
SCENE = {
    '/lio_sam/mapping/map_global': {
        'Color Transformer': 'Intensity', 'Channel Name': 'intensity', 'Use rainbow': False,
        'Min Color': '34; 54; 84', 'Max Color': '170; 214; 240',
        'Autocompute Intensity Bounds': False, 'Min Intensity': 0, 'Max Intensity': 2200,
        'Style': 'Points', 'Size (Pixels)': 2, 'Alpha': 1.0,
    },
    # Only the scan being matched, not every one since the start: the map
    # display above already holds them all, corrected by loop closures.
    # Opaque: points with an alpha below 1 were not drawn at all in testing.
    '/lio_sam/mapping/cloud_registered': {
        'Color Transformer': 'FlatColor', 'Color': '255; 186; 73',
        'Style': 'Points', 'Size (Pixels)': 2, 'Alpha': 1.0, 'Decay Time': 0,
    },
    '/lio_sam/mapping/path': {'Color': '74; 222; 128', 'Line Style': 'Billboards', 'Line Width': 0.18},
    '/lio_sam/mapping/gps_path_aligned': {
        'Color': '167; 139; 250', 'Line Style': 'Billboards', 'Line Width': 0.08},
    '/gps_path': {'Color': '248; 113; 113'},
}
# Height above the road each path is drawn at.
ON_THE_ROAD = {'/lio_sam/mapping/path': 0.10, '/lio_sam/mapping/gps_path_aligned': 0.16}
BACKGROUND = '18; 20; 24'
# Behind the cart and a little to its right, looking down at it. The cart
# drives towards -x of lidar_link, so behind it is the +x side: yaw 0.
VIEW = {
    'Class': 'rviz_default_plugins/ThirdPersonFollower', 'Target Frame': 'lidar_link',
    'Distance': 18, 'Pitch': 0.5, 'Yaw': 0.5, 'Focal Point': {'X': 0, 'Y': 0, 'Z': 0},
}
# Nothing subscribes to /goal_pose, /initialpose or /clicked_point while mapping.
UNUSED_TOOLS = {
    'rviz_default_plugins/SetInitialPose', 'rviz_default_plugins/SetGoal',
    'rviz_default_plugins/PublishPoint',
}


def restyle(manager, lidar_height):
    for display in manager.get('Displays', []):
        topic = display.get('Topic', {}).get('Value')
        display.update(SCENE.get(topic, {}))
        # The paths are where the lidar went, up at roof height: laid on the
        # road instead, they read as the track the cart drove.
        if topic in ON_THE_ROAD:
            display['Offset'] = {'X': 0, 'Y': 0, 'Z': ON_THE_ROAD[topic] - lidar_height}
        # The cart shows where lidar_link is; the axes of every frame hide it.
        if display.get('Class') == 'rviz_default_plugins/TF':
            display['Enabled'] = display['Value'] = False
    manager.setdefault('Global Options', {})['Background Color'] = BACKGROUND
    manager['Tools'] = [t for t in manager.get('Tools', []) if t.get('Class') not in UNUSED_TOOLS]
    manager.setdefault('Views', {}).setdefault('Current', {}).update(VIEW)
    manager['Views']['Current']['Value'] = 'ThirdPersonFollower (rviz_default_plugins)'


def cart_display(dims, out_dir):
    """The cart at its real size, drawn by RViz itself from a URDF file."""
    sys.path.insert(0, CART_DIR)
    import cart_model
    urdf = cart_model.build(dims, os.path.join(out_dir, 'cart'))
    return {
        'Class': 'rviz_default_plugins/RobotModel', 'Name': 'GolfCart', 'Enabled': True,
        'Value': True, 'Description Source': 'File', 'Description File': urdf,
        'Visual Enabled': True, 'Collision Enabled': False, 'Alpha': 1, 'TF Prefix': '',
        'Update Interval': 0,
    }


def with_panel(source, themed):
    """rviz2.rviz + the panel and the cart, written next to the panel's scripts."""
    with open(source, encoding='utf-8') as f:
        config = yaml.safe_load(f)
    out_dir = os.path.join(
        os.environ.get('XDG_RUNTIME_DIR') or tempfile.gettempdir(), 'mapping_panel')
    os.makedirs(out_dir, mode=0o700, exist_ok=True)

    panels = config.setdefault('Panels', [])
    if not any(p.get('Class') == PANEL['Class'] for p in panels):
        panels.insert(0, dict(PANEL))
    # A panel missing from the saved window state lands in the left dock.
    geometry = config.setdefault('Window Geometry', {})
    geometry['Hide Left Dock'] = False
    geometry[PANEL['Name']] = {'collapsed': False}

    manager = config.setdefault('Visualization Manager', {})
    with open(os.path.join(CART_DIR, 'cart.yaml'), encoding='utf-8') as f:
        dims = yaml.safe_load(f)
    if themed:
        restyle(manager, float(dims['lidar']['height']))
    try:
        manager.setdefault('Displays', []).append(cart_display(dims, out_dir))
    except Exception as error:  # a slip in cart.yaml must not keep the panel from opening
        print('mapping_panel: no cart model (%s: %s)' % (type(error).__name__, error))

    out = os.path.join(out_dir, 'mapping.rviz')
    with open(out, 'w', encoding='utf-8') as f:
        yaml.safe_dump(config, f, sort_keys=False, allow_unicode=True, width=100000)
    return out


def launch_setup(context):
    themed = LaunchConfiguration('theme').perform(context).strip().lower() in ('true', '1', 'yes')
    config = with_panel(os.path.join(
        get_package_share_directory('lio_sam'), 'config', 'rviz2.rviz'), themed)
    return [Node(
        package='rviz2',
        executable='rviz2',
        # LIO-SAM's own RViz, when run.launch.py is started by hand, is rviz2.
        name='mapping_panel_rviz',
        arguments=['-d', config],
        # What run.launch.py gives its RViz, so everything behaves as it does
        # there. (With no /clock on the cart it stops RViz's clock, which is
        # why "Decay Time 1000" in rviz2.rviz keeps every scan on screen.)
        parameters=[{'use_sim_time': True}],
        # Read by the panel: dark look for this RViz only.
        additional_env={'MAPPING_RVIZ_THEME': 'dark'} if themed else {},
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'theme', default_value='true',
            description='true = ธีมมืดและฉากที่อ่านง่าย | false = หน้าตาเดิมของ rviz2.rviz',
        ),
        OpaqueFunction(function=launch_setup),
    ])
