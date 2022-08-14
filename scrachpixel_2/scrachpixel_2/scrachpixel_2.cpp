// Author Alhajras Algdairy, 14.8.2022, Uni-Freiburg
// A simple Ray tracer that render primitves by using different data structures
// The following resources are being used in the code and edited accordingly: 
//		www.scratchapixel.com
//      www.realtimerendering.com
//      cg.informatik.uni-freiburg.de/teaching.htm
//      stackoverflow.com
//      www.tutorialspoint.com/binary-search-tree-iterator-in-cplusplus
//      https://blogs.nvidia.com/blog/2021/05/27/omniverse-marbles-rtx-playable-sample
//      https://github.com/alecjacobson/common-3d-test-models

#include "bvh.h"
#include <algorithm>
#include <bitset>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <exception>
#include <fstream>
#include <functional>
#include <iomanip>
#include <ios>
#include <iosfwd>
#include <istream>
#include <iterator>
#include <limits>
#include <limits>
#include <list>
#include <locale>
#include <map>
#include <memory>
#include <memory>
#include <new>
#include <numeric>
#include <omp.h>
#include <ostream>
#include <queue>
#include <set>
#include <sstream>
#include <sstream>
#include <stack>
#include <stdexcept>
#include <streambuf>
#include <typeinfo>
#include <utility>
#include <utility>
#include <valarray>

template <> const Matrix44f Matrix44f::kIdentity = Matrix44f();
enum SceneModel { IGEA, ARMADILLO, BUNNY, BUNNIES, TEST, GRASS,  BUDDHA, CITY };
using namespace std;
float RAY_EPSILON = 0.000000001;
unsigned int rayId = 0;
#define M_PI 3.141592653589793
constexpr float EPS = 1e-6;
unsigned int  rootNodeIndex = 0;
char NUMBER_OF_CLONES = 1; // This is used to generatre clones of the model for testing.

// Statstics related
unsigned int bv_intersections_counter = 0;
float tree_raverse_time = 0;


inline
float deg2rad(const float& deg)
{
	return deg * M_PI / 180;
}

inline
float clamp(const float& lo, const float& hi, const float& v)
{
	return std::max(lo, std::min(hi, v));
}

std::vector<SceneObject> sceneFixed;

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


void fresnel(const Vec3f& I, const Vec3f& N, const float& ior, float& kr)
{
	float cosi = clamp(-1, 1, I.dotProduct(N));
	float etai = 1, etat = ior;
	if (cosi > 0) { std::swap(etai, etat); }
	// Compute sini using Snell's law
	float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
	// Total internal reflection
	if (sint >= 1) {
		kr = 1;
	}
	else {
		float cost = sqrtf(std::max(0.f, 1 - sint * sint));
		cosi = fabsf(cosi);
		float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
		float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
		kr = (Rs * Rs + Rp * Rp) / 2;
	}
}

class Triangle
{
public:
	Vec3f v0;                           /// position of the sphere
	Vec3f v1;                           /// position of the sphere
	Vec3f v2;                           /// position of the sphere
	Vec3f surfaceColor, emissionColor;      /// surface color and emission (light)
	float transparency, reflection;         /// surface transparency and reflectivity
	MaterialType materialType;
	Triangle(
		const Vec3f& v_0,
		const Vec3f& v_1,
		const Vec3f& v_2,
		const Vec3f& sc,
		const float& refl = 0,
		const float& transp = 0,
		const Vec3f& ec = 0) :
		v0(v_0), v1(v_1), v2(v_2), surfaceColor(sc), emissionColor(ec),
		transparency(transp), reflection(refl), materialType(DIFFUSE_AND_GLOSSY)
	{ /* empty */
	}
	//[comment]
	// Compute a ray-sphere intersection using the geometric solution
	//[/comment]
	bool rayTriangleIntersect(
		const Vec3f& orig, const Vec3f& dir,
		float& t) const
	{
		float u;
		float v;
#ifdef MOLLER_TRUMBORE
		Vec3f v0v1 = v1 - v0;
		Vec3f v0v2 = v2 - v0;
		Vec3f pvec = dir.crossProduct(v0v2);
		float det = v0v1.dotProduct(pvec);
#ifdef CULLING
		// if the determinant is negative the triangle is backfacing
		// if the determinant is close to 0, the ray misses the triangle
		if (det < EPS) return false;
#else
		// ray and triangle are parallel if det is close to 0
		if (fabs(det) < EPS) return false;
#endif
		float invDet = 1 / det;

		Vec3f tvec = orig - v_0;
		u = tvec.dotProduct(pvec) * invDet;
		if (u < 0 || u > 1) return false;

		Vec3f qvec = tvec.crossProduct(v0v1);
		v = dir.dotProduct(qvec) * invDet;
		if (v < 0 || u + v > 1) return false;

		t = v0v2.dotProduct(qvec) * invDet;

		return true;
#else
		// compute plane's normal
		Vec3f v0v1 = v1 - v0;
		Vec3f v0v2 = v2 - v0;
		// no need to normalize
		Vec3f N = v0v1.crossProduct(v0v2); // N
		float denom = N.dotProduct(N);

		// Step 1: finding P

		// check if ray and plane are parallel ?
		float NdotRayDirection = N.dotProduct(dir);
		if (fabs(NdotRayDirection) < EPS) // almost 0
			return false; // they are parallel so they don't intersect ! 

		// compute d parameter using equation 2
		float d = N.dotProduct(v0);

		// compute t (equation 3)
		t = (N.dotProduct(orig) + d) / NdotRayDirection;
		// check if the triangle is in behind the ray
		if (t < 0) return false; // the triangle is behind

		// compute the intersection point using equation 1
		Vec3f P = orig + t * dir;

		// Step 2: inside-outside test
		Vec3f C; // vector perpendicular to triangle's plane

		// edge 0
		Vec3f edge0 = v1 - v0;
		Vec3f vp0 = P - v0;
		C = edge0.crossProduct(vp0);
		if (N.dotProduct(C) < 0) return false; // P is on the right side

		// edge 1
		Vec3f edge1 = v2 - v1;
		Vec3f vp1 = P - v1;
		C = edge1.crossProduct(vp1);
		if ((u = N.dotProduct(C)) < 0)  return false; // P is on the right side

		// edge 2
		Vec3f edge2 = v0 - v2;
		Vec3f vp2 = P - v2;
		C = edge2.crossProduct(vp2);
		if ((v = N.dotProduct(C)) < 0) return false; // P is on the right side;

		u /= denom;
		v /= denom;

		return true; // this ray hits the triangle
#endif
	}
};

Vec3f reflect(const Vec3f& I, const Vec3f& N)
{
	return I - 2 * I.dotProduct(N) * N;
}

Vec3f refract(const Vec3f& I, const Vec3f& N, const float& ior)
{
	float cosi = clamp(-1, 1, I.dotProduct(N));
	float etai = 1, etat = ior;
	Vec3f n = N;
	if (cosi < 0) { cosi = -cosi; }
	else { std::swap(etai, etat); n = -N; }
	float eta = etai / etat;
	float k = 1 - eta * eta * (1 - cosi * cosi);
	return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
}

bool trace_more(Settings settings,
	const Vec3f& orig, const Vec3f& dir,
	std::vector<Sphere>& spheres,
	float& tNear, uint32_t& index, Vec2f& uv, Sphere* hitObject,
	std::vector<SceneObject>& scene,
	std::shared_ptr<Node>& tree, std::vector<int> boundingBoxes)
{

	bool hit = false;
	return hit;
}

inline float modulo(const float& f)
{
	return f - std::floor(f);
}

bool intersectSphere(Vec3f position, Vec3f direction, SceneObject s, float& iTime, Vec3f& normal, Vec3f& intersection)
{

	float a = direction.dotProduct(direction);
	float b = 2 * direction.dotProduct(position - s.center);
	float c = s.center.dotProduct(s.center) + position.dotProduct(position) + (-2 * s.center.dotProduct(position)) - pow(s.radius, 2);

	float discriminant = b * b - 4 * a * c;

	if (discriminant > 0.0 + RAY_EPSILON)
	{

		float t = (-b - sqrt(discriminant)) / (2 * a);

		float t2 = (-b + sqrt(discriminant)) / (2 * a);


		if (t2 > RAY_EPSILON)
		{
			//we know we have some intersection

			if (t > RAY_EPSILON)
			{
				iTime = t;
			}
			else
			{
				iTime = t2;
			}

			intersection = position + t * direction;
			normal = ((intersection - s.center) / s.radius).normalize();
			return true;
		}
	}
	return false;
}

Vec3f castRay(
	const Vec3f& rayorig,
	const Vec3f& raydir,
	std::vector<Sphere>& spheres,
	std::vector<Sphere>& lights,
	std::vector<SceneObject>& scene,
	std::shared_ptr<Node>& tree,
	const int& depth, const Settings& settings)
{

	float minT = std::numeric_limits<float>::max();
	//SceneObject intersectObj;
	Vec3f minTnormal;
	Vec3f minTintersection;
	bool intersect = false;
	std::vector<int> boundingBoxes;


	std::vector<int> sphereids;

	if (depth > 2) {
		return Vec3f(0.6, 0.8, 1);
	}

	Vec2f uv;
	uint32_t index = 0;

	Vec3f  hitColor = Vec3f(0.6, 0.8, 1);

	Vec3f color(0, 0, 0);
	float tnear = INFINITY;
	Sphere* sphere = NULL;
	float t0, t1;
	int type = 0;
	t0 = INFINITY;


	std::vector<int> ints;

	switch (settings.dataStructure) {
	case LBVH:
	case BVH:
	{
		const clock_t begin_time = clock();
		boxIntersect(rayorig, raydir, tree, boundingBoxes);

		tree_raverse_time += float(clock() - begin_time) / CLOCKS_PER_SEC;

		if (boundingBoxes.size() == 0)
		{
			return Vec3f(0.6, 0.8, 1);
		}
		for (int box : boundingBoxes)
		{

			if (sceneFixed[box].isSphere)
			{
				t0 = INFINITY, t1 = INFINITY;
				if (sceneFixed[box].sphere.raySphereIntersect(rayorig, raydir, t0, t1)) {
					if (t0 < 0) t0 = t1;
					if (t0 < tnear) {
						tnear = t0;
						sphere = &sceneFixed[box].sphere;
						hitColor = sphere->surfaceColor;
					}
				}
			}
		}

		break;
	}
	case KDTREE:
	{
		const clock_t begin_time = clock();
		if (!kdtreeIntersect(rayorig, raydir))
		{
			return Vec3f(0.6, 0.8, 1);
		}
		return Vec3f(0, 0.0, 0);
		tree_raverse_time += float(clock() - begin_time) / CLOCKS_PER_SEC;
		break;
	}
	default: {
		// This is without using any data structures

		for (unsigned i = 0; i < scene.size(); ++i) {
			t0 = INFINITY, t1 = INFINITY;
			if (sceneFixed[i].sphere.raySphereIntersect(rayorig, raydir, t0, t1)) {
				if (t0 < 0) t0 = t1;
				if (t0 < tnear) {
					tnear = t0;
					sphere = &sceneFixed[i].sphere;
					hitColor = sphere->surfaceColor;
				}
			}
		}
	}
	}

	t0 = 500000;


	// if there's no intersection return black or background color
	if (!type && !sphere) return hitColor;
	Vec3f surfaceColor = 0; // color of the ray/surfaceof the object intersected by the ray
	Vec3f hitPoint = rayorig + raydir * tnear; // point of intersection
	Vec3f nhit = NULL;
	nhit = hitPoint - sphere->center;
	nhit.normalize(); // normalize normal direction
	// If the normal and the view direction are not opposite to each other
	// reverse the normal direction. That also means we are inside the sphere so set
	// the inside bool to true. Finally reverse the sign of IdotN which we want
	// positive.
	float bias = 1e-4; // add some bias to the point from which we will be tracing
	bool inside = false;
	if (raydir.dotProduct(nhit) > 0) nhit = -nhit, inside = true;


	Vec3f N = nhit; // normal
	Vec2f st; // st coordinates
	//hitObject->getSurfaceProperties(hitPoint, dir, index, uv, N, st);
	Vec3f tmp = hitPoint;

	// it's a diffuse object, no need to raytrace any further

	if (!type) {
		switch (sphere->materialType) {
		case REFLECTION_AND_REFRACTION:
		{
			Vec3f reflectionDirection = reflect(raydir, N).normalize();
			Vec3f refractionDirection = refract(raydir, N, 3).normalize();
			Vec3f reflectionRayOrig = (reflectionDirection.dotProduct(N) < 0) ?
				hitPoint - N * bias :
				hitPoint + N * bias;
			Vec3f refractionRayOrig = (refractionDirection.dotProduct(N) < 0) ?
				hitPoint - N * bias :
				hitPoint + N * bias;
			Vec3f reflectionColor = castRay(reflectionRayOrig, reflectionDirection, spheres, lights, scene, tree, depth + 1, settings);
			Vec3f refractionColor = castRay(refractionRayOrig, refractionDirection, spheres, lights, scene, tree, depth + 1, settings);
			float kr;
			fresnel(raydir, N, 2, kr);
			hitColor = refractionColor * (1 - kr);
			break;
		}
		case REFLECTION:
		{

			Vec3f reflectionDirection = reflect(raydir, N).normalize();
			Vec3f reflectionRayOrig = (reflectionDirection.dotProduct(N) < 0) ?
				hitPoint - N * bias :
				hitPoint + N * bias;
			Vec3f reflectionColor = castRay(reflectionRayOrig, reflectionDirection, spheres, lights, scene, tree, depth + 1, settings);
			float kr;
			fresnel(raydir, N, 2, kr);
			hitColor = (1 - kr);
			break;
		}            default:
		{
			hitColor = Vec3f(0, 0, 0);

			for (unsigned i = 0; i < lights.size(); ++i) {

				// this is a light
				Vec3f lightAmt = 0, specularColor = 0;
				Vec3f shadowPointOrig = (raydir.dotProduct(N) < 0) ?
					hitPoint + N * bias :
					hitPoint - N * bias;
				// [comment]
				// Loop over all lights in the scene and sum their contribution up
				// We also apply the lambert cosine law here though we haven't explained yet what this means.
				// [/comment]
				//for (uint32_t i = 0; i < lights.size(); ++i) {
				Vec3f lightDir = lights[i].center - hitPoint;
				// square of the distance between hitPoint and the light
				float lightDistance2 = lightDir.dotProduct(lightDir);
				lightDir = lightDir.normalize();
				float LdotN = std::max(0.f, lightDir.dotProduct(N));
				Sphere* shadowHitObject = nullptr;
				float tNearShadow = INFINITY;
				// is the point in shadow, and is the nearest occluding object closer to the object than the light itself?
				bool inShadow = trace_more(settings, shadowPointOrig, lightDir, spheres, tNearShadow, index, uv, shadowHitObject, scene, tree, boundingBoxes) &&
					tNearShadow * tNearShadow < lightDistance2;
				lightAmt += (1 - inShadow) * lights[i].emissionColor * LdotN;
				Vec3f reflectionDirection = reflect(-lightDir, N);
				specularColor += powf(std::max(0.f, -reflectionDirection.dotProduct(raydir)), 25) * lights[i].emissionColor;
				//}
				Vec2f st(0.2);
				float angle = deg2rad(45);

				float s = ((1 + atan2(N.z, N.x) / M_PI) * 0.5) * cos(angle) - (acosf(N.y) / M_PI) * sin(angle);
				float t = (acosf(N.y) / M_PI) * cos(angle) + ((1 + atan2(N.z, N.x) / M_PI) * 0.5) * sin(angle);
				float scaleS = 20, scaleT = 20;
				float pattern = (cos((acosf(N.y) / M_PI) * 2 * M_PI * scaleT) * sin(((1 + atan2(N.z, N.x) / M_PI) * 0.5) * 2 * M_PI * scaleS) + 1) * 0.5; // isect.hitObject->albedo
				//float pattern = (modulo(s * scaleS) < 0.5) ^ (modulo(t * scaleT) < 0.5);
				//float pattern = (modulo(s * scaleS) < 0.5);
				//hitColor += vis * pattern * lightIntensity * std::max(0.f, hitNormal.dotProduct(-lightDir));
				// Remove the pattern to git rid of the texture
				//hitColor += pattern * lightAmt * (lights[i].getDiffuseColor(st) * 0.8) / (2) + specularColor * 0.5;
				hitColor += lightAmt * (lights[i].getDiffuseColor(st) * 0.8) / (2) + specularColor * 0.5;
				hitColor += sphere->surfaceColor;

			}

		}
		}
	}
	return (hitColor);

	//return Vec3f(1, 1, 1);
}

// This will generate a random number to be used for the anti alisaing to generate radndom scatterd rays.
inline double random_double()
{
	static std::uniform_real_distribution<double > distribution(0.0, 1.0);
	static std::mt19937 generator;
	return distribution(generator);
}

inline float random_float()
{
	return static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
}

//  This will write pixels into a file
void write_into_file(const Settings& settings, int frame, Vec3f* image) {
	// Save result to a PPM image (keep these flags if you compile under Windows)
	std::ofstream ofs("./" + std::to_string(frame) + "out.ppm", std::ios::out | std::ios::binary);
	ofs << "P6\n" << settings.width << " " << settings.height << "\n255\n";
	for (unsigned i = 0; i < settings.width * settings.height; ++i) {
		ofs << (unsigned char)(std::min(float(1), image[i].x / settings.aa_samples) * 255) <<
			(unsigned char)(std::min(float(1), image[i].y / settings.aa_samples) * 255) <<
			(unsigned char)(std::min(float(1), image[i].z / settings.aa_samples) * 255);
	}
	ofs.close();

	delete[] image;
}

void printProgress(double percentage) {
	int val = (int)(percentage * 100);
	if (val == 12) {
		int X = 5;
	}
	int lpad = (int)(percentage * 60);
	int rpad = 60 - lpad;
	printf("\r%3d%% [%.*s%*s]", val, lpad, "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||", rpad, "");
	fflush(stdout);
}

void render(const Settings& settings, std::vector<Sphere>& spheres, std::vector<Sphere>& lights, std::vector<Triangle> triangles, int frame, std::vector<SceneObject>& scene, std::shared_ptr<Node>& tree)
{
	Vec3f* image = new Vec3f[settings.width * settings.height], * pixel = image;
	float invWidth = 1 / float(settings.width), invHeight = 1 / float(settings.height);
	float fov = 30, aspectratio = settings.width / float(settings.height);
	float angle = tan(M_PI * 0.5 * fov / 180.);
	// Trace rays
	double loadingProgress = 0;
	for (unsigned y = 0; y < settings.height; ++y) {
		for (unsigned x = 0; x < settings.width; ++x, ++pixel) {
			Vec3f sampled_pixel(0, 0, 0);

			for (unsigned sample = 0; sample < settings.aa_samples; ++sample) {
				float xx = (2 * ((x + random_double()) * invWidth) - 1) * angle * aspectratio;
				float yy = (1 - 2 * ((y + random_double()) * invHeight)) * angle;
				Vec3f raydir(xx, yy, -1);
				raydir.normalize();
				sampled_pixel += castRay(Vec3f(0), raydir, spheres, lights, scene, tree, 1, settings);
			}
			*pixel = sampled_pixel;
			loadingProgress++;
			printProgress(loadingProgress / (settings.width * settings.height));
		}
	}
	write_into_file(settings, frame, image);
}

inline double random_double_2() {
	// Returns a random real in [0,1).
	return rand() / (RAND_MAX + 1.0);
}

inline double random_double_2(double min, double max) {
	// Returns a random real in [min,max).
	return min + (max - min) * random_double();
}

Vec3f RotationPoints(Vec3f Coordenate, Vec3f AngleCoordenate)
{
	Vec3f RotatedPoint;

	//First we rotate the Z:
	RotatedPoint.x = Coordenate.x * cos(AngleCoordenate.z) - Coordenate.y * sin(AngleCoordenate.z);
	RotatedPoint.y = Coordenate.x * sin(AngleCoordenate.z) + Coordenate.y * cos(AngleCoordenate.z);
	RotatedPoint.z = Coordenate.z;

	//Second we rotate the Y:
	RotatedPoint.x = RotatedPoint.x * cos(AngleCoordenate.y) + RotatedPoint.z * sin(AngleCoordenate.y);
	RotatedPoint.y = RotatedPoint.y;
	RotatedPoint.z = RotatedPoint.y * sin(AngleCoordenate.y) + RotatedPoint.z * cos(AngleCoordenate.y);

	//Third we rotate the X:
	RotatedPoint.x = RotatedPoint.x;
	RotatedPoint.y = RotatedPoint.y * cos(AngleCoordenate.x) - RotatedPoint.z * sin(AngleCoordenate.x);
	RotatedPoint.z = RotatedPoint.y * sin(AngleCoordenate.x) + RotatedPoint.z * cos(AngleCoordenate.x);
	return RotatedPoint;
}

std::vector<SceneObject> createScene_new(Settings settings) {
	std::vector<SceneObject> scene;
	unsigned int id = 0;

	std::vector<Vec3f> vertices;
	//Loads OBJ file from path
	std::ifstream file;
	std::string fileName = "C:/Users/alhaj/source/repos/scrachpixel_2/scrachpixel_2/models/";

	switch (settings.sceneModel) {
	case IGEA:
	{
		fileName.append("igea.obj");
		break;
	}
	case ARMADILLO:
	{
		fileName.append("armadillo.obj");
		break;
	}
	case BUNNIES:
	{
		fileName.append("bunnies.obj");
		break;
	}
	case CITY:
	{
		fileName.append("city.obj");
		break;
	}
	case GRASS:
	{
		fileName.append("grass.obj");
		break;
	}
	case BUNNY:
	{
		fileName.append("bunny.obj");
		break;
	}
	case BUDDHA:
	{
		fileName.append("buddha.obj");
		break;
	}
	default: {
		// By default the bunny will be renderd
		fileName.append("test.obj");
		break;
	}
	}
	for (int clone = 0; clone < NUMBER_OF_CLONES; clone++)
	{
		float shift = clone * 20;

		file.open(fileName);
		std::cout << "Loading file:  " << fileName << " ... " << std::endl;
		if (!file.good())
		{
			std::cout << "Can't open file " << fileName << std::endl;
			return scene;
		}

		std::string line;
		while (1)
		{
			std::string text;

			file >> text;
			if (text == "v")
			{
				Vec3f vertex;

				file >> vertex.x;
				file >> vertex.y;
				file >> vertex.z;


				SceneObject s;
				s.objId = id;
				s.radius = 0.01 * 5;
				s.center = vertex * 100 + shift;
				s.center.y += -10;
				s.center.z += -50;
				s.position = s.center;
				s.shininess = 64;
				s.isSphere = true;
				Vec3f minPoint = s.center - s.radius;
				Vec3f maxPoint = s.center + s.radius;
				s.boxBoundries = BoxBoundries(minPoint, maxPoint);
				s.sphere = Sphere(id++, s.center, s.radius, Vec3f(0.8, 0.7, 0), 0, 0.0, REFLECTION_AND_REFRACTION);
				double random = random_double_2();
				scene.push_back(s);
				sceneFixed.push_back(s);
			}
			else
			{
				break;
			}
		}
		file.close();

	}
	SceneObject s;
	Sphere ground = Sphere(id, Vec3f(0.93591022, -105.47120094, -43.2363205), 100, Vec3f(0, 0, 0), 0, 0.0);
	s.objId = id;
	s.radius = ground.radius; //  hoody = *10 // Bunny =0.01 * 5
	s.center = ground.center; // igea = *50, bunny = *20, hoody = / 50 
	s.position = s.center;
	s.shininess = 64;
	s.isSphere = true;
	Vec3f minPoint = s.center - s.radius;
	Vec3f maxPoint = s.center + s.radius;
	s.boxBoundries = BoxBoundries(minPoint, maxPoint);
	s.sphere = ground;
	double random = random_double_2();

	scene.push_back(s);
	sceneFixed.push_back(s);
	std::cout << "Number of spheres: " << id << std::endl;
	return scene;

}

int main(int argc, char** argv)
{
	const clock_t begin_total_time = clock();
	const clock_t begin_time = clock();
	Settings settings;

	// Print settings
	std::cout << "Start rendering .... \n";
	std::cout << "Settings are.... \n";
	std::cout << "Height: " << settings.height << endl;
	std::cout << "Width: " << settings.width << endl;
	std::cout << "DataStructure: " << settings.dataStructure << endl;
	std::cout << "Anti-aliasing samples: " << settings.aa_samples << endl;
	std::cout << "\n====================================================================\n";



	/////////////////////////////////
	//*****Builds bvh tree*****
	std::vector<std::shared_ptr<Node>> nodes;
	std::shared_ptr<Node> root = std::make_shared<Node>();
	root->maxX = -1 * std::numeric_limits<float>::max();
	root->minX = std::numeric_limits<float>::max();
	root->maxY = -1 * std::numeric_limits<float>::max();
	root->minY = std::numeric_limits<float>::max();;
	root->maxZ = -1 * std::numeric_limits<float>::max();
	root->minZ = std::numeric_limits<float>::max();;

	std::vector<SceneObject> scene = createScene_new(settings);


	std::cout << "Wraping BV for each object .... \n";

	std::cout << "Done .... \n Time: ";

	std::cout << float(clock() - begin_time) / CLOCKS_PER_SEC << "s";

	std::cout << "\n====================================================================\n";




	for (int frame = 15; frame > 14; frame--) {
		int shift = 20;
		int sh_y = 4;
		std::vector<Sphere> spheres;
		std::vector<Sphere> lights;

		std::vector<Triangle> triangles;


		//Sphere light = Sphere(0,Vec3f(0, 10, -10), 1, Vec3f(1, 1, 1), 0, 0.0, Vec3f(1));
		Sphere light2 = Sphere(0, Vec3f(0, 3, 30), 10, Vec3f(1, 1, 1), 0, 0.0, Vec3f(1));

		Sphere gray = Sphere(0, Vec3f(-5, 0, -20), 2, Vec3f(0.1, 0.4, 0.6), 1, 0.0);
		Sphere gray_1 = Sphere(1, Vec3f(-5.5, 0, -23), 0.5, Vec3f(0, 0, 0), 1, 0.0);

		gray.materialType = REFLECTION_AND_REFRACTION;

		spheres.push_back(gray); //gray left
		spheres.push_back(gray_1); //gray left
		spheres.push_back(Sphere(2, Vec3f(0.0, -100, -20), 98, Vec3f(0.20, 0.20, 0.20), 0, 0.0)); // ground
		spheres.push_back(Sphere(3, Vec3f(5, 0, -20), 2, Vec3f(0.1, 0.77, 0.97), 1, 0.0)); //yellow right

		//lights.push_back(light);
		lights.push_back(light2);


		switch (settings.dataStructure) {
		case BVH:
		{
			std::cout << "<<<<<<< This is BVH >>>>>>" << endl;
			std::cout << "construct BVH Tree .... \n";
			const clock_t begin_time = clock();
			int totalNodes = 0;
			std::vector<std::shared_ptr<SceneObject>> orderedPrims;
			orderedPrims.reserve(scene.size());
			root = constructBVHNew(scene, 0, scene.size(),
				&totalNodes);
			std::cout << "Done .... Time: ";

			std::cout << float(clock() - begin_time) / CLOCKS_PER_SEC << "s\n";

			std::cout << "Total number of nodes: " << totalNodes << "\n";

			render(settings, spheres, lights, triangles, frame, scene, root);
			break;
		}
		case KDTREE:
		{
			std::cout << "<<<<<<< This is KDTREE >>>>>>";
			std::cout << "construct KD-Tree .... ";
			const clock_t begin_time = clock();
			constructKDTreeNew(
				scene,
				80, 1, 0.5f,
				1, -1
			);
			std::cout << "Done .... Time: ";
			std::cout << float(clock() - begin_time) / CLOCKS_PER_SEC << "s\n";
			std::cout << "Number of nodes: " << totalKdNodes << "\n";
			render(settings, spheres, lights, triangles, frame, scene, root);
			break;
		}
		case LBVH:
		{
			std::cout << "<<<<<<< This is LBVH >>>>>>";
			std::cout << "construct LBVH Tree .... ";
			const clock_t begin_time = clock();
			root = constructLBVHTree( scene, root, nodes);
			std::cout << "Done .... Time: ";
			std::cout << float(clock() - begin_time) / CLOCKS_PER_SEC << "s\n";
			render(settings, spheres, lights, triangles, frame, scene, root);
			break;
		}
		default: {
			std::cout << "<<<<<<< Warning: No data structure is used, this can take long time! >>>>>>";
			render(settings, spheres, lights, triangles, frame, scene, root);
			break;
		}
		}
	}
	std::cout << "\n Number of the tree nodes: " << nodes.size() << " bv node" << endl;
	std::cout << "\n Tree traverse time spent: " << tree_raverse_time << "s" << endl;
	std::cout << "\n Number of Sphere intersection tests: " << spheres_intersections_counter << " test" << endl;

	std::cout << "\n Total time spent: ";
	std::cout << float(clock() - begin_total_time) / CLOCKS_PER_SEC << "s" << endl;
	std::cout << "\n--------- Rendering Completed ---------\n";
	return 0;
}