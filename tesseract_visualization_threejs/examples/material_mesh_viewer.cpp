// Port of tesseract_viewer_python/examples/tesseract_material_mesh_viewer.py
// Open the URL it logs in a browser, usually http://127.0.0.1:8000
// Displays textured glTF and Collada meshes. TESSERACT_RESOURCE_PATH must resolve package://tesseract.

#include <iostream>
#include <memory>
#include <string>

#include <tesseract/common/resource_locator.h>
#include <tesseract/environment/environment.h>
#include <tesseract/visualization/threejs/threejs_visualization.h>

static const std::string URDF = R"(<robot name="mesh_viewer" tesseract:make_convex="true">
  
  <link name="world"/>
  <link name="mesh_gltf2_link">
    <visual>
      <geometry>
        <mesh filename="package://tesseract/support/meshes/tesseract_material_mesh.glb"/>
      </geometry>
    </visual>
  </link>

  <joint name="mesh_gltf2_joint" type="revolute">
    <parent link="world"/>
    <child link="mesh_gltf2_link"/>
    <axis xyz="0 1 0"/>
    <origin xyz="0 -1 0.25"/>
    <limit effort="0" lower="-2.0944" upper="2.0944" velocity="6.2832"/>
  </joint>

  <link name="mesh_dae_link">
    <visual>
      <geometry>
        <mesh filename="package://tesseract/support/meshes/tesseract_material_mesh.dae"/>
      </geometry>
    </visual>
  </link>

  <joint name="mesh_dae_joint" type="revolute">
    <parent link="world"/>
    <child link="mesh_dae_link"/>
    <axis xyz="0 1 0"/>
    <origin xyz="0 1 0.25"/>
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
