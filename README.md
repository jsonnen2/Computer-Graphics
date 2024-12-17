
## Raytracing

This Julia program shoots Rays into a constructed scene to render an image. 
- meshes: .obj file format, basic polygons
- shading: diffuse, blinn-phong, lambertian
- lights: point, directional, area, spotlight

The technique of anti-aliasing (AA) removes jagged edges from an image by sampling multiple rays per pixel, then averaging the colors together. Anti-aliasing creates a smooth transition between objects in a scene. However, AA is incredibly expensive computationally. I sought to apply AA to a subset of pixels which I believed to be "jagged". This technique utilizes an algorithm to detect edges between objects in a scene. This edge detection algorithm works for shadows, reflections, and refractions. You may view images at ```Rays/results/Edge_Detection```

A second optimization to the Raytracer is the Bounding Volume Hierarchy (BVH). The BVH is an algorithm which splits the scene into a hierarchy of rectangular bounding boxes. Now a ray only needs to check for intersection with shapes within its bounding box. This optimization is especially important in scenes with large meshes with thousands of triangles. 


## Gaussian & Nearest Neighbor Splatting

I used splats of color to recreate an image. I began by sampling color from a source image at random positions. Those random positions make the centroids of my splat. The splat is drawn as an ellipse (Gaussian normal) with scaling parameters in the x and y direction. I use Stochastic Gradient Descent to optimize my scaling and transparency parameters with respect to a loss function. My loss function is simply the difference between the ground truth image and the forward pass of my model. As a future expansion, I could create a multispectoral loss equation which optimizes towards the correct wavelength of light. My neural network is nothing fancy, I trained for 7,000 epochs before I observed diminishing returns. I used sensible learning rates and performed no hyperparameter tuning. 

My best Gaussian Splatting results still contain many areas with no color (no Gaussian was drawn close enough to that pixel). To fix this issue, I decided to interpolate pixels based on their nearest centroid, instead of drawing a Gaussian. This is actually a well studied problem titled "Voronoi Tessellation". I was able to find a github repo by Arthur Santaza which impelements Fortune's algorithm, an O(n log n) approach to interpolate pixels from their nearest centroid. Interfacing this repo with my code, I observed massive performance increases. However, I did not utilize Stochastic Gradient Descent to optimize with respect to a loss function because I do not know how to take the derivative of "find the closest centroid". Additionally, performance was significantly slower for Nearest Neighbor interpolation than Gaussian splats, which was dissapointing. 
