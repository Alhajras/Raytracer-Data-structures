# Rendering-Master-Project

This is a ray tracer implemented as part of the master project at the University of Freiburg. The report of the work done can be found <a href="/Report/report.pdf">here</a>. The ray tracer was tested on a machine with Ubuntu 20.04.2 LTS. In order to run the code, run this command in the root directory:

```
g++ -o output main.cpp
```
and then run the output file as
```
./output
```
This will start rendering a scene a sanford bunny.

Next are some of the renderd results:

| showcase 1 | showcase 2 | showcase 3 | showcase 4 |
| --------  | ------------------- | --------------------- |---------------------|
| ![nature scene](/project/raytracer/images/texture_2.PNG)| ![textured spheres](/project/raytracer/images/texture_1.PNG)      | ![benchmark scene](/project/raytracer/images/texture_3.PNG) | ![showcase scene](/project/raytracer/images/spec_reflection.PNG) |

| showcase 5 | showcase 6|
| --------  | ------------------- |
| ![showcase scene](/project/raytracer/images/refraction__0_5.PNG) | ![showcase scene 3](/project/raytracer/images/multi_shadow_multi_spec.PNG) |

| showcase 7 | showcase 8 |
| --------  | ------------------- |
| ![analysis scene 1](/project/raytracer/images/bunny%20(2).PNG)| ![analysis scene 2](project/raytracer/images/igea.PNG)  |



In the <a href="/project/raytracer/settings.h">settings</a> file one can change the ray tracer configuration, most important configurations are:

```
width: Width of the scene, default value is 640
height: Height of the scene, default value is 480
backgroundColor: Standard bg color. default value is white
aa_samples: Anti aliasing samples, default value is 1
dataStructure: options are (BVH, KDTREE, LBVH, NONE), default value is Bunny
sceneModel: Model from the models in the models directory, default value is the bunny
```

The external libraries are saved in the folder `libraries` and some parts taken from [Ray Tracing in One Weekend](https://raytracing.github.io/books/RayTracingInOneWeekend.html) in the folder `RTIOW`. `aabb.h` is also placed in `Scratchapixel` as I'm using the same implementation explained there. There are some other places which are quite similar to how they were explained in [Ray Tracing in One Weekend](https://raytracing.github.io/books/RayTracingInOneWeekend.html) but it's rather difficult to separate everything as I also did a lot of changes.

## Thanks to
1. [Ray Tracing in One Weekend](https://raytracing.github.io/books/RayTracingInOneWeekend.html)
1. [Scratchapixel](https://www.scratchapixel.com/) for properly explaining all the concepts and the [AABB](https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection) implementation.
1. [Inigo Quilez](https://www.iquilezles.org/www/articles/intersectors/intersectors.htm) for the ray-triangle intersection method.
1. [Chris Gyurgyik](https://github.com/cgyurgyik/fast-voxel-traversal-algorithm/blob/master/overview/FastVoxelTraversalOverview.md) for the nice explanation of Fast Voxel Traversal Algorithm.
1. The grass texture in `textures` was taken from https://3djungle.net/textures/grass/1417/.
1. The other solar textures were taken from https://www.solarsystemscope.com/textures/.
1. The `cube` model was taken from https://people.sc.fsu.edu/~jburkardt/data/obj/obj.html.
1. The blender suzanne model was taken from https://github.com/OpenGLInsights/OpenGLInsightsCode/blob/master/Chapter%2026%20Indexing%20Multiple%20Vertex%20Arrays/article/suzanne.obj
1. The Stanford dragon, Serapis Bust, Srub Pine Tree, Utah Teapot and Happy Buddha were taken from https://casual-effects.com/data/
1. The Stanford XYZ dragon, Stanford bunny, Stanford Lucy and Caltech Spot were taken from https://github.com/alecjacobson/common-3d-test-models
1. The grass model was taken from https://free3d.com/3d-model/high-quality-grass-78178.html
1. PNM image library: https://github.com/ToruNiina/pnm/blob/master/pnm.hpp
1. TinyObjLoader: https://github.com/tinyobjloader/tinyobjloader
