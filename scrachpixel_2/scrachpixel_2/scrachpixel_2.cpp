// Author Alhajras Algdairy, 1.7.2021, Uni-Freiburg
// A simple Raytracer that render triangles and spheres
// The following resources are being used in the code and edited accordingly: 
//		www.scratchapixel.com
//      www.realtimerendering.com
//      cg.informatik.uni-freiburg.de/teaching.htm
//      stackoverflow.com
//      www.tutorialspoint.com/binary-search-tree-iterator-in-cplusplus

#include "bvh.h"
#include <cstdio>
#include <utility>
#include <limits>
#include <cstdlib>
#include <memory>
#include <cmath>
#include <sstream>
#include <chrono>
//#include "uniform_grid.h"
//#include "geometry.h"
#include <algorithm>
#include <bitset>
#include <complex>
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
#include <list>
#include <locale>
#include <map>
#include <memory>
#include <new>
#include <numeric>
#include <ostream>
#include <queue>
#include <set>
#include <sstream>
#include <stack>
#include <stdexcept>
#include <streambuf>
#include <typeinfo>
#include <utility>
#include <valarray>

const float INF = std::numeric_limits<float>::max();
template <> const Matrix44f Matrix44f::kIdentity = Matrix44f();
using namespace std;
float RAY_EPSILON = 0.000000001;

#define M_PI 3.141592653589793
constexpr float EPS = 1e-6;
int rootNodeIndex = 0;

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


// Settings of the raytracer
struct Settings
{
	uint32_t width = 640; // Width of the scene
	uint32_t height = 480; // Height of the scene
	float fov = 90;
	Vec3f backgroundColor = Vec3f(1, 1, 1); // Standard bg color is white
	float bias = 0.0001; // Error allowed
	uint32_t maxDepth = 0; // Max number of ray trating into the scene
	uint32_t aa_samples = 5; // Anti aliasing samples
	AccType dataStructure = KDTREE; // 0 bvh, 1 kd tree
	int kdtreeDepth = 3;
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
	// As a consequence of the conservation of energy, transmittance is given by:
	// kt = 1 - kr;
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

// From scratchapixel
Vec3f evalBezierCurve(const Vec3f* P, const float& t)
{
	float b0 = (1 - t) * (1 - t) * (1 - t);
	float b1 = 3 * t * (1 - t) * (1 - t);
	float b2 = 3 * t * t * (1 - t);
	float b3 = t * t * t;

	return P[0] * b0 + P[1] * b1 + P[2] * b2 + P[3] * b3;
}

Vec3f evalBezierPatch(const Vec3f* controlPoints, const float& u, const float& v)
{
	Vec3f uCurve[4];
	for (int i = 0; i < 4; ++i)
		uCurve[i] = evalBezierCurve(controlPoints + 4 * i, u);

	return evalBezierCurve(uCurve, v);
}

Vec3f derivBezier(const Vec3f* P, const float& t)
{
	return -3 * (1 - t) * (1 - t) * P[0] +
		(3 * (1 - t) * (1 - t) - 6 * t * (1 - t)) * P[1] +
		(6 * t * (1 - t) - 3 * t * t) * P[2] +
		3 * t * t * P[3];
}

bool trace_more(
	const Vec3f& orig, const Vec3f& dir,
	std::vector<Sphere>& spheres,
	float& tNear, uint32_t& index, Vec2f& uv, Sphere* hitObject, 
	std::vector<SceneObject>& scene,
	std::vector<Node>& tree, std::vector<int> boundingBoxes)
{

	bool hit = false;
	for (int box : boundingBoxes)
	{
		for (int i = 0; i < tree[box].numObjs; i++)
		{
			if (scene[tree[box].objs[i]].isSphere)
			{
				float tNearK = INF;
				uint32_t indexK;
				Vec2f uvK;
				float t1;
				if (scene[tree[box].objs[i]].sphere.raySphereIntersect(orig, dir, tNearK, t1) && tNearK < tNear) {
					hitObject = &scene[tree[box].objs[i]].sphere;
					hit = true;
					tNear = tNearK;
					index = 0;
					uv = uvK;
				}
			}
		}
	}

	return hit;
	//return (*hitObject != nullptr);
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
	std::vector<Node>& tree,
	const int& depth,
	const std::unique_ptr<Grid>& accel, const Settings& settings)
{
	float minT = std::numeric_limits<float>::max();
	//SceneObject intersectObj;
	Vec3f minTnormal;
	Vec3f minTintersection;
	bool intersect = false;
	std::vector<int> boundingBoxes;


	std::vector<int> sphereids;

	if (depth > 5) {
		return Vec3f(0.6, 0.8, 1);
	}

	Vec2f uv;
	uint32_t index = 0;

	Vec3f  hitColor = Vec3f(0.6, 0.8, 1);

	Vec3f color(0, 0, 0);
	//if (raydir.length() != 1) std::cerr << "Error " << raydir << std::endl;
	float tnear = INFINITY;
	Sphere* sphere = NULL;
	float t0, t1;
	int type = 0;
	t0 = INFINITY;


	std::vector<int> ints;

	switch (settings.dataStructure) {
	case BVH:
	{
		bvhTraverse(rayorig, raydir, tree, rootNodeIndex, boundingBoxes);
		if (boundingBoxes.size() == 0)
		{
			return Vec3f(0.6, 0.8, 1);
		}

		for (int box : boundingBoxes)
		{
			for (int i = 0; i < tree[box].numObjs; i++)
			{
				if (scene[tree[box].objs[i]].isSphere)
				{
					t0 = INFINITY, t1 = INFINITY;
					if (scene[tree[box].objs[i]].sphere.raySphereIntersect(rayorig, raydir, t0, t1)) {
						if (t0 < 0) t0 = t1;
						if (t0 < tnear) {
							tnear = t0;
							sphere = &scene[tree[box].objs[i]].sphere;
							hitColor = sphere->surfaceColor;
						}
					}
				}
			}
		}

		break;
	}
	case KDTREE:
	{
		bvhTraverse(rayorig, raydir, tree, rootNodeIndex, boundingBoxes);
		if (boundingBoxes.size() == 0)
		{
			return Vec3f(0.6, 0.8, 1);
		}

		for (int box : boundingBoxes)
		{
			for (int i = 0; i < tree[box].numObjs; i++)
			{
				if (scene[tree[box].objs[i]].isSphere)
				{
					t0 = INFINITY, t1 = INFINITY;
					if (scene[tree[box].objs[i]].sphere.raySphereIntersect(rayorig, raydir, t0, t1)) {
						if (t0 < 0) t0 = t1;
						if (t0 < tnear) {
							tnear = t0;
							sphere = &scene[tree[box].objs[i]].sphere;
							hitColor = sphere->surfaceColor;
						}
					}
				}
			}
		}		break;
	}
	case UNIFORM_GRID:
	{
		int rayId = 0;
		Sphere hitsphere = Sphere(-1, Vec3f(0), 0, Vec3f(0, 0, 0), 0, 0.0);
		accel->intersect(rayorig, raydir, rayId++, tnear, hitsphere);
		sphere = &hitsphere;
		break;
	}
	default: {
		// This is without using any data structures
		for (unsigned i = 0; i < spheres.size(); ++i) {
			t0 = INFINITY, t1 = INFINITY;
			if (spheres[i].raySphereIntersect(rayorig, raydir, t0, t1)) {
				if (t0 < 0) t0 = t1;
				if (t0 < tnear) {
					tnear = t0;
					sphere = &spheres[i];
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
			Vec3f reflectionColor = castRay(reflectionRayOrig, reflectionDirection, spheres, lights, scene, tree, depth + 1, accel, settings);
			Vec3f refractionColor = castRay(refractionRayOrig, refractionDirection, spheres, lights, scene, tree, depth + 1, accel, settings);
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
			Vec3f reflectionColor = castRay(reflectionRayOrig, reflectionDirection, spheres, lights, scene, tree, depth + 1, accel, settings);
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
				float tNearShadow = INF;
				// is the point in shadow, and is the nearest occluding object closer to the object than the light itself?
				bool inShadow = trace_more(shadowPointOrig, lightDir, spheres, tNearShadow, index, uv, shadowHitObject,scene, tree, boundingBoxes) &&
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
				hitColor += pattern * lightAmt * (lights[i].getDiffuseColor(st) * 0.8) / (2) + specularColor * 0.5;
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
void render(const Settings& settings, std::vector<Sphere>& spheres, std::vector<Sphere>& lights, std::vector<Triangle> triangles, int frame, std::vector<SceneObject>& scene, std::vector<Node>& tree, const std::unique_ptr<Grid>& accel)
{
	Vec3f* image = new Vec3f[settings.width * settings.height], * pixel = image;
	float invWidth = 1 / float(settings.width), invHeight = 1 / float(settings.height);
	float fov = 30, aspectratio = settings.width / float(settings.height);
	float angle = tan(M_PI * 0.5 * fov / 180.);
	// Trace rays
	for (unsigned y = 0; y < settings.height; ++y) {
		for (unsigned x = 0; x < settings.width; ++x, ++pixel) {
			Vec3f sampled_pixel(0, 0, 0);

			for (unsigned sample = 0; sample < settings.aa_samples; ++sample) {
				float xx = (2 * ((x + random_double()) * invWidth) - 1) * angle * aspectratio;
				float yy = (1 - 2 * ((y + random_double()) * invHeight)) * angle;
				Vec3f raydir(xx, yy, -1);
				raydir.normalize();
				sampled_pixel += castRay(Vec3f(0), raydir, spheres, lights, scene, tree, 5, accel, settings);
			}
			*pixel = sampled_pixel;
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

std::vector<SceneObject> createScene() {
	std::vector<SceneObject> scene;
	int id = 0;
	for (int i = -5; i < 3; i++) {
		SceneObject s;
		s.objId = id;
		s.radius = 1;
		s.center = Vec3f(i + 0.9 * random_double_2(), i + 0.9 * random_double_2(), i + -20 + 0.9 * random_double_2());
		s.position = s.center;
		s.shininess = 64;
		s.isSphere = true;
		s.sphere = Sphere(id++, s.center, s.radius, Vec3f(random_double_2(), random_double_2(), random_double_2()), 0, 0.0);
		scene.push_back(s);

		}
	return scene;

}

int main(int argc, char** argv)
{
	Settings settings;

	/////////////////////////////////
	//*****Builds bvh tree*****
	std::vector<Node> nodes;
	Node root;

	root.maxX = std::numeric_limits<float>::min();
	root.minX = std::numeric_limits<float>::max();
	root.maxY = std::numeric_limits<float>::min();
	root.minY = std::numeric_limits<float>::max();;
	root.maxZ = std::numeric_limits<float>::min();
	root.minZ = std::numeric_limits<float>::max();;

	std::vector<SceneObject> scene = createScene();

	// Lets bound each object with BBOX
	// This is same for KD-tree and BVH
	for (SceneObject obj : scene)
	{
		// we calculate the minimum edges of the box
		if (obj.position[0] - obj.radius < root.minX)
			root.minX = obj.position[0] - obj.radius;
		if (obj.position[1] - obj.radius < root.minY)
			root.minY = obj.position[1] - obj.radius;
		if (obj.position[2] - obj.radius < root.minZ)
			root.minZ = obj.position[2] - obj.radius;

		// we calculate the maximum edges of the box
		if (obj.position[0] + obj.radius > root.maxX)
			root.maxX = obj.position[0] + obj.radius;
		if (obj.position[1] + obj.radius > root.maxY)
			root.maxY = obj.position[1] + obj.radius;
		if (obj.position[2] + obj.radius > root.maxZ)
			root.maxZ = obj.position[2] + obj.radius;
	}

	// we try to find the midpoint and longest axis of the root box
	if (root.maxX - root.minX > root.maxY - root.minY)
	{
		if (root.maxX - root.minX > root.maxZ - root.minZ)
		{
			root.longestAxis = 0;
			root.midpoint = (root.maxX + root.minX) / 2;
		}
	}
	if (root.maxY - root.minY > root.maxX - root.minX)
	{
		if (root.maxY - root.minY > root.maxZ - root.minZ)
		{
			root.longestAxis = 1;
			root.midpoint = (root.maxY + root.minY) / 2;
		}
	}
	if (root.maxZ - root.minZ > root.maxX - root.minX)
	{
		if (root.maxZ - root.minZ > root.maxY - root.minY)
		{
			root.longestAxis = 2;
			root.midpoint = (root.maxZ + root.minZ) / 2;
		}
	}







	for (int frame = 15; frame > 14; frame--) {
		int shift = 20;
		int sh_y = 4;
		std::vector<Sphere> spheres;
		std::vector<Sphere> lights;

		std::vector<Triangle> triangles;


		Sphere light = Sphere(0,Vec3f(0, 10, -10), 1, Vec3f(1, 1, 1), 0, 0.0, Vec3f(1));
		Sphere light2 = Sphere(0,Vec3f(10, 10, -20), 0.2, Vec3f(1, 1, 1), 0, 0.0, Vec3f(1));

		Sphere gray = Sphere(0, Vec3f(-5, 0, -20), 2, Vec3f(0.1, 0.4, 0.6), 1, 0.0);
		Sphere gray_1 = Sphere(1,Vec3f(-5.5, 0, -23), 0.5, Vec3f(0, 0, 0), 1, 0.0);

		gray.materialType = REFLECTION_AND_REFRACTION;

		spheres.push_back(gray); //gray left
		spheres.push_back(gray_1); //gray left
		spheres.push_back(Sphere(2, Vec3f(0.0, -100, -20), 98, Vec3f(0.20, 0.20, 0.20), 0, 0.0)); // ground
		spheres.push_back(Sphere(3, Vec3f(5, 0, -20), 2, Vec3f(0.1, 0.77, 0.97), 1, 0.0)); //yellow right

		lights.push_back(light);
		lights.push_back(light2);


		switch (settings.dataStructure) {
		case BVH:
		{
			std::cout << "<<<<<<< This is BVH >>>>>>";
			rootNodeIndex = constructBVHTree(scene, root, nodes);
			render(settings, spheres, lights, triangles, frame, scene, nodes, NULL);
			break;
		}
		case KDTREE:
		{
			std::cout << "<<<<<<< This is KDTREE >>>>>>";
			root.longestAxis = 0;
			rootNodeIndex = constructKDTree(scene, root, nodes, settings.kdtreeDepth);
			render(settings, spheres, lights, triangles, frame, scene, nodes, NULL);
			break;
		}
		case UNIFORM_GRID:
		{
			std::cout << "<<<<<<< This is UNIFORM_GRID >>>>>>";
			std::unique_ptr<Grid> accel(new Grid(scene));
			render(settings, spheres, lights, triangles, frame, scene, nodes, accel);
			break;
		}
		default: {

		}
		}
	}
	return 0;
}