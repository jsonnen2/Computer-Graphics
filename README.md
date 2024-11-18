
## Overview

A Computer Graphics scene begins which a collection of objects. My raytracer has support for
triangle meshes written in the .obj file format. I wrote my own meshes for a cylinder, sphere, 
and torus. Objects in my scene have coefficients which control their reflective and 
refractive properties. Objects are illuminated by light sources, of which I have three models:
point, directional, and rectangular. Light's interaction with my objects is governed by the
shading model. I implement Diffuse, Blinn-Phong, and Oren-Nayer shading models. I implement
the supersampling technique of Anti-Aliasing to remove jagged edges from my image. This is
a costly technique, so I wrote a edge detection algorithm to only supersample at place where
I believed it may benefit the quality of the image. A final optimization is the use of 
Bounding Volume Heirarchies. The core idea involves wrapping a complex triangle mesh with a 
simple object (square, sphere, etc.). Then each raycast will check if the ray intersects with 
this simple object before checking every triangle in the mesh. This concludes the overview
of my Computer Graphics project. The reader may examine my scenes in the `./results` folder. 

## Topics 

- Triangle meshes
- Reflection and refraction
- Lighting-- point, directional, rectangular
- Shading models-- diffuse, Blinn-Phong, and Oren-Nayer
- Aliasing and edge detection
- Bounding Volume Heirarchy
