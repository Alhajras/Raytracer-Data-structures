#ifndef SETTINGS_H
#define SETTINGS_H

enum SceneModel { IGEA, ARMADILLO, BUNNY, BUNNIES, TEST, GRASS,  BUDDHA, CITY };

// Settings of the raytracer
struct Settings
{
	uint32_t width = 640; // Width of the scene
	uint32_t height = 480; // Height of the scene
	float fov = 90;
	Vec3f backgroundColor = Vec3f(1, 1, 1); // Standard bg color is white
	float bias = 0.0001; // Error allowed
	uint32_t aa_samples = 1; // Anti aliasing samples
	AccType dataStructure = BVH; // 0 bvh, 1 kd tree
	SceneModel sceneModel = BUNNY;
};

#endif