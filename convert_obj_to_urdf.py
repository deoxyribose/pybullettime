import os

def get_urdf_str(mesh_dir, scale_values):
    urdf_str = ("""
<robot name="model.urdf">
  <link name="base_link">
    <contact>
      <friction_anchor/>
      <lateral_friction value="0.8"/>
      <spinning_friction value="0.001"/>
      <rolling_friction value="0.001"/>
      <contact_cfm value="0.1"/>
      <contact_erp value="1.0"/>
    </contact>
    <inertial>
      <origin xyz="0 0 0" />
      <mass value="0.1" />
      <inertia ixx="1e-3" ixy="0" ixz="0" iyy="1e-3" iyz="0" izz="1e-3"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <mesh filename="%s/model.obj" scale="%s %s %s"/>
      </geometry>
      <material name="white">
        <color rgba="1. 1. 1. 1."/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0"/>
      <geometry>
        <mesh filename="%s/model.obj" scale="%s %s %s"/>
      </geometry>
    </collision>
  </link>
</robot>
""") % (mesh_dir, scale_values[0], scale_values[1], scale_values[2], mesh_dir, scale_values[0], scale_values[1], scale_values[2])
    return urdf_str

def obj2urdf(obj_file, urdf_file, scale_values):
    mesh_dir = os.path.dirname(obj_file)
    with open(urdf_file, 'w') as f:
        f.write(get_urdf_str(mesh_dir, scale_values))

if __name__ == "__main__":
    obj2urdf("mesh.obj", "model.urdf", [1, 1, 1])