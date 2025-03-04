## Laplacian Deformation Tool

My submission for the programming assignment in COMP 438: Geometric Modelling and Processing.
This is an implementation of laplacian deformation for meshes in libigl.

### Compiling

To compile the project, you can generate a Visual Studio solution with:

```
mkdir build
cd build
cmake ..
```

Then build and run the project with Visual Studio.

### Using the tool

Upon starting the application, all the custom key shortcuts are printed in the console. Other than that, the graphical user interface is self-explanatory. To clarify the selection mechanism, there are three states: *neutral*, *anchor*, and *control*. When in *neutral* state, selection is disabled so that moving the camera is easier. In the *anchor* state, clicking and dragging will select anchor points (user constraints that are fixed), while in *control* state, clicking and dragging will select control points (user constraints that will move). Lastly, holding *shift* will allow multi-selection (adding to current selection) while holding *ctrl* will allow removing specific points from current selection.
