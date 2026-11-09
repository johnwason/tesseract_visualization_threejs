// Port of tesseract_viewer_python/examples/shapes_viewer.py
// Open the URL it logs in a browser, usually http://127.0.0.1:8000
// Displays a cylinder, a box and a sphere from an inline URDF.

#include <iostream>
#include <memory>
#include <string>

#include <tesseract/common/resource_locator.h>
#include <tesseract/environment/environment.h>
#include <tesseract/visualization/threejs/threejs_visualization.h>

static const std::string URDF = R"(<robot name="multipleshapes" tesseract:make_convex="true">
  
  <link name="world"/>
  <link name="cylinder_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="clyinder_joint" type="revolute">
    <parent link="world"/>
    <child link="cylinder_link"/>
    <axis xyz="0 1 0"/>
    <limit effort="0" lower="-2.0944" upper="2.0944" velocity="6.2832"/>
  </joint>

  <link name="box_link">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
      <material name="green">
        <color rgba="0 0.8 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="box_joint" type="revolute">
    <parent link="world"/>
    <child link="box_link"/>
    <origin xyz="1 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="0" lower="-2.0944" upper="2.0944" velocity="6.2832"/>
  </joint>

  <link name="sphere_link">
    <visual>
      <geometry>
        <sphere radius="0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
  </link>

  <joint name="sphere_joint" type="revolute">
    <parent link="world"/>
    <child link="sphere_link"/>
    <origin xyz="2 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="0" lower="-2.0944" upper="2.0944" velocity="6.2832"/>
  </joint>

</robot>)";

int main()
{
  tesseract::environment::Environment env;
  if (!env.init(URDF, std::make_shared<tesseract::common::GeneralResourceLocator>()))
  {
    std::cerr << "Failed to load the environment\n";
    return 1;
  }

  tesseract::visualization::threejs::ThreejsVisualization viewer;
  viewer.plotEnvironment(env);

  viewer.waitForInput("press enter");
  return 0;
}
