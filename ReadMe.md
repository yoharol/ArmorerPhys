# GL Factory

My own head-only c++ interactive rendering toolset as part of my research framework. The core concept is the data-oriented design for flexibility and adaptability. There are only struct and functions. 

For example, here is how to create a scene with lighting and camera:

```c++
  armgl::Scene scene = armgl::create_scene(
      armgl::Light{
          {242, 76, 61},       // light color
          {9, 5, 128},         // ambient color
          {0.0f, 0.35f, 5.0f}  // light position
      },
      armgl::create_camera(
          {2.0f, 0.0f, -2.0f},                  // camera position
          {0.0f, 0.35f, 0.0f},                  // camera target
          {0.0f, 1.0f, 0.0f},                   // camera up axis
          float(SCR_WIDTH) / float(SCR_HEIGHT)  // camera aspect
          ));
```

And then add a mesh to render pipeline:

```c++
  armgl::DiffuseMaterial material{
      {255, 255, 255},  // diffuse color
      {255, 255, 255},  // specular color
      0.5f              // specular strength
  };
  armgl::DiffuseMesh mesh = armgl::create_diffuse_mesh(material);
  // ... Get V and F
  armgl::set_mesh_data(mesh, V, F);
  // add the render function of mesh to render pipeline
  armgl::add_render_func(scene, armgl::get_render_func(mesh));
```

In each step the scene is built by iterating through all render funcs:

```c++
  for (RenderFunc func : scene.render_funcs) {
    func(scene);
  }
```
