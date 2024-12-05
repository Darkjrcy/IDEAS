# IDEAS
Ideas to see how Github works before submiting

# Adding a city to Gazebo
## 1. Generate the 3D model of the city
The prefered 3D modeling formats in Gazebo are: Digital Asset Exchange (dae), or Standard Triangle Language (stl). The first step to add the city into the Gazebo world is to generate the city 3D model, to do it you can use blender and follow the steps in the next video: https://www.youtube.com/watch?v=ZsLMt3Ka8UA&t=700s
## 2. Spawn the 3D model in Gazebo
There are two ways to spawn the mdoel into Gazebo, using an URDF file or generating a model directory to spawn it directly from gazebo (recommended).
### 2.1 Generate the model directly in Gazebo (Recommended)
First, create a folder called models into the same package you have your worlds, and input it into the CMakeLists of your package.

### 2.2 Use a URDF file to spawn the Gazebo model into a World
First, add the city model in stl or dae format to your meshes folder. Then, create a joint between a dummy link and your city inside your URDF file. 
```xml
  <!-- LINKS OF THE URDF-->
    <!-- Base dummy link-->
    <link name="base_link"/>
    <!-- Body CIty Link-->
    <link name="city_link">
        <!-- Generate the mesh so the city is visible using the .dae format-->
        <visual>
            <!-- Geometry is the property that finds the 3D modeling of the city,where city_description 
            is the package name where your meshes folder is located and installed-->
            <geometry>
                <mesh filename="file://$(find city_description)/meshes/city.dae" scale="1 1 1"/> <!-- In case of .dae file-->
                <!-- Alternative: <mesh filename="package://city_description/meshes/city.stl" scale="1 1 1"/>-->
            </geometry>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        </visual>
        <collision>
            <!-- Generate the collision so the buildings interact to other models int he simulation 
            with your dae file-->
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
            <geometry>
                <mesh filename="file://$(find city_description)/meshes/city.dae" scale="0.5 0.5 0.5"/>
            </geometry>
        </collision>
        <!-- You need to add intertia to your city so Gazebo knows is a real object; however, if you want
         to have realistic values is preferable to generate the models directly into Gazebo-->
        <inertial>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
            <mass value="100.0"/>
            <inertia ixx="1.0" ixy="-1.0" ixz="-1.0" iyy="1.0" iyz="-1.0" izz="1.0"/>
        </inertial>
    </link>


    <!--JOINTS OF THE URDF-->
    <!-- City joint to the Base Link that helps to see where the city is located with respect to the origin-->
    <joint name="airplane_joint" type="fixed">
        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        <parent link="base_link"/>
        <child link="city_link"/>
    </joint>
```
Now, when you spawn your URDF file the city woudl be spawned in the origin. If you have questions in how to launch an URDF file into Gazebo see: https://www.youtube.com/watch?v=w6Kvq7ac-J8
