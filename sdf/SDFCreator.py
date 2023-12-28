import os
import sys

script_path = os.path.abspath(__file__)
parent_dir = os.path.dirname(script_path)

if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

import common_models as cm


class SDFCreator:
    def __init__(self):
        self.default_cube = cm.cube_sdf
        self.create_cube = cm.create_cube

if __name__ == '__main__':
    import rospy
    from gazebo_msgs.srv import SpawnModel
    from geometry_msgs.msg import Pose

    sdf = SDFCreator()

    rospy.init_node('spawn_cube')

    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    pose = Pose()
    pose.position.x = 0.2
    pose.position.y = 0
    pose.position.z = 0.2  # 高度，确保它不会与其他对象碰撞

    # 调用服务以放置模型
    spawn_model('my_cube1', sdf.create_cube(0.3, 0.1), '', pose, 'world')