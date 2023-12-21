jibot3_prompt = [{'role': 'system', 'content': 'Input: Query\nOutput: Pure Python Code'},
    {'role': 'user', 'content': 
        '''Rotate the first joint 30 degrees clockwise.'''},
    {'role': 'assistant', 'content': '''self.move_joint([30, 0, 0, 0, 0], 'degree', 'INC')'''},
    {'role': 'user', 'content': 
        '''Move the second joint 0.5 radians anticlockwise.'''},
    {'role': 'assistant', 'content': '''self.move_joint([0, 0.5, 0, 0, 0], 'rad', 'INC')'''},
    {'role': 'user', 'content': 
        '''顺时针旋转第五个关节 0.2 radians。'''},
    {'role': 'assistant', 'content': '''self.move_joint([0, 0, 0, 0, 0.2], 'rad', 'INC')'''},
    {'role': 'user', 'content': 
        '''逆时针旋转第四个关节 30 度。'''},
    {'role': 'assistant', 'content': '''self.move_joint([0, 0, 0, -30, 0], 'degree', 'INC')'''},
    {'role': 'user', 'content':
        '''打开夹爪。'''},
    {'role': 'assistant', 'content': '''self.move_gripper('open')'''},
    {'role': 'user', 'content':
        '''Close the gripper.'''},
    {'role': 'assistant', 'content': '''self.move_gripper('close')'''},
    {'role': 'user', 'content':
        '''伸直机械臂。'''},
    {'role': 'assistant', 'content': '''self.move_to('straight')'''},
    {'role': 'user', 'content':
        '''Move to the home position.'''},
    {'role': 'assistant', 'content': '''self.move_to('home')'''},
    {'role': 'user', 'content':
        '''Move back to the default position.'''},
    {'role': 'assistant', 'content': '''self.move_to('default')'''},
    {'role': 'user', 'content':
        '''显示当前机械臂的关节角度。'''},
    {'role': 'assistant', 'content': '''self.get_joint_values()'''},
    {'role': 'user', 'content':
        '''Display the current cartesian position of the robot.'''},
    {'role': 'assistant', 'content': '''self.get_cartesian_pose()'''},
]