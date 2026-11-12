import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import get_launch_description_from_python_launch_file
from launch_ros.actions import Node

# LIO-SAM จาก run.launch.py ของมันเองทุกอย่าง ยกเว้น RViz
# แผนที่ดูใน RViz ของ mapping_panel.launch.py ที่มี panel อยู่ด้วย จึงไม่เปิดอีกหน้าต่าง
# จะแก้ node, TF หรือ argument ให้แก้ที่ run.launch.py ไฟล์เดียวพอ


def generate_launch_description():
    lio_sam = get_launch_description_from_python_launch_file(os.path.join(
        get_package_share_directory('lio_sam'), 'launch', 'run.launch.py'))
    return LaunchDescription([
        entity for entity in lio_sam.entities
        if not (isinstance(entity, Node) and entity.node_executable == 'rviz2')])
