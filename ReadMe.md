# ArmorerGL

My own C++ research framework of modern OpenGL, integrating Eigen3, GLFW and libigl.

I aim at building a research framework that is self-contained, flexible and easy-to-use.

The core concept is the data-oriented design for flexibility and adaptability. There are only struct and functions. Based on function based pipeline, any new objects and functionalities can be easily added into the workflow.

For example, here is how to add gui component:

```c++
  agl::Gui gui = agl::create_gui(window, "gui");

  agl::Vec3f diffuse_color(0.0f, 211.f / 255.f, 239.f / 255.f);
  agl::add_gui_func(gui, [&diffuse_color]() {
    ImGui::Text("Diffuse Color:");
    ImGui::ColorEdit3("#c1", diffuse_color.data());
  });
```

Defining callback functions in GLFW is always annoying. With function based pipeline, life is much easier. Callback functions can be added as many as wanted.

```c++
  // Create a singleton input handler 
  agl::InputHandler& handler = agl::create_input_handler(window);

  // Add mouse move input callback function
  agl::add_mouse_move_func(handler, [&](agl::InputHandler& input_handler) {
    std::cout << "mouse moving to " << handler.xpos << handler.ypos;
  });

  // Add mouse button input callback function 
  agl::add_mouse_input_func(handler, [](agl::InputHandler& input_handler,
                                          int button, int action) {
    std::cout << "mouse button event: " << button << " " << action << std::endl;
  });

  // Add keyboard input callback function
  agl::add_key_input_func(
      handler, [](agl::InputHandler& input_handler, int key, int action) {
        std::cout << "key event: " << key << " " << action << std::endl;
      });
```

This is how to add points and lines data into render pipeline:

```c++
  agl::Matx2f v_p; // vertex position
  v_p.resize(2, 2);
  v_p << 1.0f, 1.0f,  //
      1.0f, 0.5f;     //
  agl::Matx3f v_color; // per-vertex color
  v_color.resize(2, 3);
  v_color << 0.0f, 0.0f, 0.0f,  //
      1.0f, 0.2f, 0.0f;         //

  agl::Points points = agl::create_points(); // create points object
  agl::set_points_data(points, v_p, v_color);
  // add points to render pipeline
  agl::add_render_func(scene, agl::get_render_func(points)); 
  
  agl::Lines lines = agl::create_lines(); // create lines object
  agl::set_lines_data(lines, v_p, v_color);
  // add lines to render pipeline
  agl::add_render_func(scene, agl::get_render_func(lines));
```

Rendering new objects can be easily added as render functions.

In each frame the scene is built by iterating through all render functions, as well as input functions and gui functions.

```c++
  for (RenderFunc func : scene.render_funcs) {
    func(scene);
  }
```

Input and gui functions work similarly. 

## Selected results:

### 04_pipeline

Show how to use the pipeline of render, mouse/keyboard input and gui functions pipeline.

Orbit control is defined for default camera: A/D/Q/E to rotate, W/S to zoom in/out.

<img src="./images/04_pipeline.png" alt="drawing" width="500"/>

### 05_2d_game

A mass spring simulation with adjustable physics parameters.

<img src="./images/05_2d_game.png" alt="drawing" width="500"/>

### Mathplot2d 

Draw axis, grids, points and lines in 2d space.

To be added: Triangles, parameter curve, splines

<img src="./images/09_matplot2d.png" alt="drawing" width="500"/>
