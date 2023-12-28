cube_sdf = """
<sdf version='1.6'>
  <model name='cube'>
    <pose>0 0 0.015 0 0 0</pose> <!-- X Y Z Roll Pitch Yaw -->
    <link name='link'>
      <inertial>
        <mass>0.1</mass>
      </inertial>
      <collision name='collision'>
        <geometry>
          <box>
            <size>0.03 0.03 0.03</size> <!-- 正方体的尺寸 -->
          </box>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <box>
            <size>0.03 0.03 0.03</size> <!-- 正方体的尺寸 -->
          </box>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
"""

def create_cube(size, mass):
    cube_sdf_str = f'''
<sdf version='1.6'>
  <model name='cube'>
    <pose>0 0 {size/2} 0 0 0</pose> <!-- X Y Z Roll Pitch Yaw -->
    <link name='link'>
      <inertial>
        <mass>{mass}</mass>
      </inertial>
      <collision name='collision'>
        <geometry>
          <box>
            <size>{size} {size} {size}</size> <!-- 正方体的尺寸 -->
          </box>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <box>
            <size>{size} {size} {size}</size> <!-- 正方体的尺寸 -->
          </box>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
'''
    return cube_sdf_str