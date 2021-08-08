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
#include <cstdint>
#include <limits>
#include <random>
#include <cstdlib>
#include <memory>
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <sstream>
#include <chrono>
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
#include <iostream>
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
#include <string>
#include <typeinfo>
#include <utility>
#include <valarray>

const float INF = std::numeric_limits<float>::max();
template <> const Matrix44f Matrix44f::kIdentity = Matrix44f();
using namespace std;

#define M_PI 3.141592653589793
constexpr float EPS = 1e-6;

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

enum MaterialType { DIFFUSE_AND_GLOSSY, REFLECTION_AND_REFRACTION, REFLECTION };

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

inline
Vec3f mix(const Vec3f& a, const Vec3f& b, const float& mixValue)
{
	return a * (1 - mixValue) + b * mixValue;
}

class Sphere
{
public:
	Vec3f center;                           /// position of the sphere
	float radius, radius2;                  /// sphere radius and radius^2
	Vec3f surfaceColor, emissionColor;      /// surface color and emission (light)
	float transparency, reflection;         /// surface transparency and reflectivity
	MaterialType materialType;
	Sphere* left_child = NULL;
	Sphere* right_child = NULL;
	bool is_bv = false;
	Sphere(
		const Vec3f& c,
		const float& r,
		const Vec3f& sc, const float& refl = 0,
		const float& transp = 0,
		const Vec3f& ec = 0) :
		center(c), radius(r), radius2(r* r), surfaceColor(sc), emissionColor(ec),
		transparency(transp), reflection(refl), materialType(DIFFUSE_AND_GLOSSY)
	{ /* empty */
	}

	void setChildes(Sphere* left, Sphere* right) {
		left_child = left;
		right_child = right;
		is_bv = true;
	}

	//[comment]
	// Compute a ray-sphere intersection using the geometric solution
	//[/comment]
	bool raySphereIntersect(const Vec3f& rayorig, const Vec3f& raydir, float& t0, float& t1) const
	{
		Vec3f l = center - rayorig;
		float tca = l.dotProduct(raydir);
		if (tca < 0) return false;
		float d2 = l.dotProduct(l) - tca * tca;
		if (d2 > radius2) return false;
		float thc = sqrt(radius2 - d2);
		t0 = tca - thc;
		t1 = tca + thc;

		return true;
	}

	Vec3f getDiffuseColor(const Vec2f& st) const
	{
		float scale = 5;
		float pattern = (fmodf(st.x * scale, 1) > 0.5) ^ (fmodf(st.y * scale, 1) > 0.5);
		return mix(Vec3f(0.815, 0.235, 0.031), Vec3f(0.937, 0.937, 0.231), pattern);
	}
};


// This is for BVH
class TreeNode {
public:
	Sphere* val;
	TreeNode* left, * right;
	TreeNode(Sphere* data) {
		val = data;
		left = right = NULL;
	}
};
void insert(TreeNode** root, Sphere* val) {
	queue<TreeNode*> q;
	q.push(*root);
	while (q.size()) {
		TreeNode* temp = q.front();
		q.pop();
		if (!temp->left) {
			if (val != NULL)
				temp->left = new TreeNode(val);
			else
				temp->left = new TreeNode(0);
			return;
		}
		else {
			q.push(temp->left);
		}
		if (!temp->right) {
			if (val != NULL)
				temp->right = new TreeNode(val);
			else
				temp->right = new TreeNode(0);
			return;
		}
		else {
			q.push(temp->right);
		}
	}
}
TreeNode* make_tree(vector<Sphere*> v) {
	TreeNode* root = new TreeNode(v[0]);
	for (int i = 1; i < v.size(); i++) {
		insert(&root, v[i]);
	}
	return root;
}
class BSTIterator {
public:
	stack <TreeNode*> st;
	void fillStack(TreeNode* node) {
		while (node && node->val != 0) {
			st.push(node);
			node = node->left;
		}
	}
	BSTIterator(TreeNode* root) {
		fillStack(root);
	}
	/** @return the next smallest number */
	Sphere* next() {
		TreeNode* curr = st.top();
		st.pop();
		if (curr->right && curr->right->val != 0) {
			fillStack(curr->right);
		}
		return curr->val;
	}
	/** @return whether we have a next smallest number */
	bool hasNext() {
		return !st.empty();
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

/* Hair demo taken from scratchapixel
void createCurveGeometry()
{
	uint32_t ndivs = 8;
	uint32_t ncurves = 1 + (curveNumPts - 4) / 3;
	Vec3f pts[4];
	std::unique_ptr<Vec3f[]> P(new Vec3f[(ndivs + 1) * ndivs * curves + 1]);
	std::unique_ptr<Vec3f[]> N(new Vec3f[(ndivs + 1) * ndivs * ncurves + 1]);
	std::unique_ptr<Vec2f[]> st(new Vec2f[(ndivs + 1) * ndivs * ncurves + 1]);
	for (uint32_t i = 0; i < ncurves; ++i) {
		for (uint32_t j = 0; j < ndivs; ++j) {
			pts[0] = curveData[i * 3];
			pts[1] = curveData[i * 3 + 1];
			pts[2] = curveData[i * 3 + 2];
			pts[3] = curveData[i * 3 + 3];
			float s = j / (float)ndivs;
			Vec3f pt = evalBezierCurve(pts, s);
			Vec3f tangent = derivBezier(pts, s).normalize();

			uint8_t maxAxis;
			if (std::abs(tangent.x) > std::abs(tangent.y))
				if (std::abs(tangent.x) > std::abs(tangent.z))
					maxAxis = 0;
				else
					maxAxis = 2;
			else if (std::abs(tangent.y) > std::abs(tangent.z))
				maxAxis = 1;
			else
				maxAxis = 2;

			Vec3f up, forward, right;

			switch (maxAxis) {
			case 0:
			case 1:
				up = tangent;
				forward = Vec3f(0, 0, 1);
				right = up.crossProduct(forward);
				forward = right.crossProduct(up);
				break;
			case 2:
				up = tangent;
				right = Vec3f(0, 0, 1);
				forward = right.crossProduct(up);
				right = up.crossProduct(forward);
				break;
			default:
				break;
			};

			float sNormalized = (i * ndivs + j) / float(ndivs * ncurves);
			float rad = 0.1 * (1 - sNormalized);
			for (uint32_t k = 0; k <= ndivs; ++k) {
				float t = k / (float)ndivs;
				float theta = t * 2 * M_PI;
				Vec3f pc(cos(theta) * rad, 0, sin(theta) * rad);
				float x = pc.x * right.x + pc.y * up.x + pc.z * forward.x;
				float y = pc.x * right.y + pc.y * up.y + pc.z * forward.y;
				float z = pc.x * right.z + pc.y * up.z + pc.z * forward.z;
				P[i * (ndivs + 1) * ndivs + j * (ndivs + 1) + k] = Vec3f(pt.x + x, pt.y + y, pt.z + z);
				N[i * (ndivs + 1) * ndivs + j * (ndivs + 1) + k] = Vec3f(x, y, z).normalize();
				st[i * (ndivs + 1) * ndivs + j * (ndivs + 1) + k] = Vec2f(sNormalized, t);
			}
		}
	}
	P[(ndivs + 1) * ndivs * ncurves] = curveData[curveNumPts - 1];
	N[(ndivs + 1) * ndivs * ncurves] = (curveData[curveNumPts - 2] - curveData[curveNumPts - 1]).normalize();
	st[(ndivs + 1) * ndivs * ncurves] = Vec2f(1, 0.5);
	uint32_t numFaces = ndivs * ndivs * ncurves;
	std::unique_ptr<uint32_t[]> verts(new uint32_t[numFaces]);
	for (uint32_t i = 0; i < numFaces; ++i)
		verts[i] = (i < (numFaces - ndivs)) ? 4 : 3;
	std::unique_ptr<uint32_t[]> vertIndices(new uint32_t[ndivs * ndivs * ncurves * 4 + ndivs * 3]);
	uint32_t nf = 0, ix = 0;
	for (uint32_t k = 0; k < ncurves; ++k) {
		for (uint32_t j = 0; j < ndivs; ++j) {
			if (k == (ncurves - 1) && j == (ndivs - 1)) { break; }
			for (uint32_t i = 0; i < ndivs; ++i) {
				vertIndices[ix] = nf;
				vertIndices[ix + 1] = nf + (ndivs + 1);
				vertIndices[ix + 2] = nf + (ndivs + 1) + 1;
				vertIndices[ix + 3] = nf + 1;
				ix += 4;
				++nf;
			}
			nf++;
		}
	}

	for (uint32_t i = 0; i < ndivs; ++i) {
		vertIndices[ix] = nf;
		vertIndices[ix + 1] = (ndivs + 1) * ndivs * ncurves;
		vertIndices[ix + 2] = nf + 1;
		ix += 3;
		nf++;
	}
}

*/
bool trace_more(
	const Vec3f& orig, const Vec3f& dir,
	std::vector<Sphere>& spheres,
	float& tNear, uint32_t& index, Vec2f& uv, Sphere* hitObject)
{

	bool hit = false;
	for (uint32_t k = 0; k < spheres.size(); ++k) {
		float tNearK = INF;
		uint32_t indexK;
		Vec2f uvK;
		float t1;
		if (spheres[k].raySphereIntersect(orig, dir, tNearK, t1) && tNearK < tNear) {
			hitObject = &spheres[k];
			hit = true;
			tNear = tNearK;
			index = 0;
			uv = uvK;
		}
	}

	for (uint32_t k = 0; k < spheres.size(); ++k) {
		float tNearK = INF;
		uint32_t indexK;
		Vec2f uvK;
		float t1;
		if (spheres[k].raySphereIntersect(orig, dir, tNearK, t1) && tNearK < tNear) {
			hitObject = &spheres[k];
			hit = true;
			tNear = tNearK;
			index = 0;
			uv = uvK;
		}
	}

	return hit;
	//return (*hitObject != nullptr);
}

inline float modulo(const float& f)
{
	return f - std::floor(f);
}

Vec3f castRay(
	const Vec3f& rayorig,
	const Vec3f& raydir,
	std::vector<Sphere>& spheres,
	std::vector<Sphere>& lights,
	const int& depth, TreeNode* node)
{

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


	//find intersection of this ray with the sphere in the scene

	BSTIterator ob(node);
	Sphere* sphere_ptr = NULL;
	//	This is BVH
	//while (ob.hasNext())
	//{
	//	t0 = INFINITY, t1 = INFINITY;

	//	sphere_ptr = ob.next();
	//	if (!sphere_ptr->raySphereIntersect(rayorig, raydir, t0, t1) && sphere_ptr->is_bv) {
	//		
	//	}
	//	else if(!sphere_ptr->is_bv){
	//		if (t0 < 0) t0 = t1;
	//		if (t0 < tnear) {
	//			tnear = t0;
	//			sphere = sphere_ptr;
	//			hitColor = sphere->surfaceColor;
	//		}
	//	}

	//}
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
			Vec3f reflectionColor = castRay(reflectionRayOrig, reflectionDirection, spheres, lights, depth + 1, node);
			Vec3f refractionColor = castRay(refractionRayOrig, refractionDirection, spheres, lights, depth + 1, node);
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
			Vec3f reflectionColor = castRay(reflectionRayOrig, reflectionDirection, spheres, lights, depth + 1, node);
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
				bool inShadow = trace_more(shadowPointOrig, lightDir, spheres, tNearShadow, index, uv, shadowHitObject) &&
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
void render(const Settings& settings, std::vector<Sphere>& spheres, std::vector<Sphere>& lights, std::vector<Triangle> triangles, int frame, TreeNode* node)
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
				sampled_pixel += castRay(Vec3f(0), raydir, spheres, lights, 5, node);
			}
			*pixel = sampled_pixel;
		}
	}
	write_into_file(settings, frame, image);
}


std::vector<SceneObject> createScene() {
		std::vector<SceneObject> scene;

		SceneObject s1;
		s1.objId = 0;
		s1.radius = 1.0;
		s1.ambient = Vec3f(0, 0, 1);
		s1.specular = Vec3f(0.9, 0.4, 0);
		s1.diffuse = Vec3f(0.8, 0.3, 0.1);
		s1.center = Vec3f(-4, 3, 0);
		s1.position = s1.center;
		s1.shininess = 64;
		s1.reflective = Vec3f(.5, .5, .5);
		s1.sphere = true;
		scene.push_back(s1);

		SceneObject s2;
		s2.objId = 1;
		s2.radius = 1.5;
		s2.ambient = Vec3f(1.0, 1, 1);
		s2.specular = Vec3f(.5, .5, .5);
		s2.diffuse = Vec3f(1, 1, 1);
		s2.center = Vec3f(-2, 0, 0);
		s2.position = s2.center;
		s2.reflective = Vec3f(.5, .5, .5);
		s2.shininess = 64;
		s2.sphere = true;
		scene.push_back(s2);

		SceneObject s3;
		s3.objId = 2;
		s3.radius = 1;
		s3.ambient = Vec3f(1.0, 1.0, 0.0);
		s3.specular = Vec3f(.5, .5, .5);
		s3.diffuse = Vec3f(1, 1, 1);
		s3.center = Vec3f(1, 0, 3);
		s3.position = s3.center;
		s3.reflective = Vec3f(.5, .5, .5);
		s3.shininess = 64;
		s3.sphere = true;
		scene.push_back(s3);

		SceneObject s4;
		s4.objId = 3;
		s4.radius = 1;
		s4.ambient = Vec3f(1.0, 1.0, 0.0);
		s4.specular = Vec3f(.5, .5, .5);
		s4.diffuse = Vec3f(1, 1, 1);
		s4.center = Vec3f(2, 0, 3);
		s4.position = s4.center;
		s4.reflective = Vec3f(.5, .5, .5);
		s4.shininess = 64;
		s4.sphere = true;
		scene.push_back(s4);

		SceneObject s5;
		s5.objId = 4;
		s5.radius = .75;
		s5.ambient = Vec3f(1.0, 1.0, 0.0);
		s5.specular = Vec3f(.5, .5, .5);
		s5.diffuse = Vec3f(1, 1, 1);
		s5.center = Vec3f(1.5, 1, 3);
		s5.position = s5.center;
		s5.reflective = Vec3f(.5, .5, .5);
		s5.shininess = 64;
		s5.sphere = true;
		scene.push_back(s5);

		SceneObject s6;
		s6.objId = 5;
		s6.radius = .75;
		s6.ambient = Vec3f(1.0, 1.0, 0.0);
		s6.specular = Vec3f(.5, .5, .5);
		s6.diffuse = Vec3f(1, 1, 1);
		s6.center = Vec3f(1.5, -1, 3);
		s6.position = s6.center;
		s6.reflective = Vec3f(.5, .5, .5);
		s6.shininess = 64;
		s6.sphere = true;
		scene.push_back(s6);

		SceneObject s7;
		s7.objId = 6;
		s7.radius = 1.0;
		s7.ambient = Vec3f(1.0, 1, 1);
		s7.specular = Vec3f(.5, .5, .5);
		s7.diffuse = Vec3f(1, 1, 1);
		s7.center = Vec3f(0, 2, 0);
		s7.position = s7.center;
		s7.reflective = Vec3f(.5, .5, .5);
		s7.shininess = 64;
		s7.sphere = true;
		scene.push_back(s7);

		SceneObject s8;
		s8.objId = 7;
		s8.radius = 1;
		s8.ambient = Vec3f(0, 1, 0);
		s8.specular = Vec3f(.2, .2, .2);
		s8.diffuse = Vec3f(1, 1, 1);
		s8.center = Vec3f(3, 3, 4);
		s8.position = s8.center;
		s8.reflective = Vec3f(.5, .5, .5);
		s8.shininess = 64;
		s8.sphere = true;
		scene.push_back(s8);

		SceneObject s9;
		s9.objId = 8;
		s9.radius = .75;
		s9.ambient = Vec3f(.9, .9, .9);
		s9.specular = Vec3f(.2, .2, .2);
		s9.diffuse = Vec3f(1, 1, 1);
		s9.center = Vec3f(3, 2, 5.5);
		s9.position = s9.center;
		s9.reflective = Vec3f(.5, .5, .5);
		s9.shininess = 64;
		s9.sphere = true;
		scene.push_back(s9);

		SceneObject s10;
		s10.objId = 9;
		s10.radius = 1;
		s10.ambient = Vec3f(0, 0, 1);
		s10.specular = Vec3f(1, 1, 1);
		s10.diffuse = Vec3f(1, 1, 1);
		s10.center = Vec3f(-3, -.5, 5);
		s10.position = s10.center;
		s10.reflective = Vec3f(.5, .5, .5);
		s10.shininess = 64;
		s10.sphere = true;
		scene.push_back(s10);

		SceneObject s11;
		s11.objId = 10;
		s11.radius = 1;
		s11.ambient = Vec3f(1, 0, 1);
		s11.specular = Vec3f(1, 1, 1);
		s11.diffuse = Vec3f(1, 1, 1);
		s11.center = Vec3f(-4.5, 2, 4);
		s11.position = s11.center;
		s11.reflective = Vec3f(.5, .5, .5);
		s11.shininess = 64;
		s11.sphere = true;
		scene.push_back(s11);

		SceneObject s12;
		s12.objId = 11;
		s12.radius = 1;
		s12.ambient = Vec3f(0, 1, 1);
		s12.specular = Vec3f(.5, .5, .5);
		s12.diffuse = Vec3f(1, 1, 1);
		s12.center = Vec3f(2.5, 3.5, -3.5);
		s12.position = s12.center;
		s12.reflective = Vec3f(.3, .3, .3);
		s12.shininess = 64;
		s12.sphere = true;
		scene.push_back(s12);
		return scene;
	
}

int main(int argc, char** argv)
{
	
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

	std::cout << root.minX<< "\n";
	std::cout << root.minY << "\n";
	std::cout << root.minZ << "\n";
	/////////////////////////////////
	Settings settings;

	for (int frame = 15; frame > 14; frame--) {
		int shift = 20;
		int sh_y = 4;
		std::vector<Sphere> spheres;
		std::vector<Sphere> lights;

		std::vector<Triangle> triangles;


		Sphere light = Sphere(Vec3f(0, 10, -10), 1, Vec3f(1, 1, 1), 0, 0.0, Vec3f(1));
		Sphere light2 = Sphere(Vec3f(10, 10, -20), 0.2, Vec3f(1, 1, 1), 0, 0.0, Vec3f(1));

		Sphere gray = Sphere(Vec3f(-5, 0, -20), 2, Vec3f(0.1, 0.4, 0.6), 1, 0.0);
		Sphere gray_1 = Sphere(Vec3f(-5.5, 0, -23), 0.5, Vec3f(0, 0, 0), 1, 0.0);

		gray.materialType = REFLECTION_AND_REFRACTION;

		spheres.push_back(gray); //gray left
		spheres.push_back(gray_1); //gray left
		spheres.push_back(Sphere(Vec3f(0.0, -100, -20), 98, Vec3f(0.20, 0.20, 0.20), 0, 0.0)); // ground
		spheres.push_back(Sphere(Vec3f(5, 0, -20), 2, Vec3f(0.1, 0.77, 0.97), 1, 0.0)); //yellow right


		// particles

		//BVSphere root = BVSphere(Vec3f(0, 0, -20), 6, Sphere(Vec3f(-3.2, 0, -20), 2.5, Vec3f(0.2, 0.42, 0.33), 1, 0.0), Sphere(Vec3f(-3.2, 0, -20), 2.5, Vec3f(0.2, 0.42, 0.33), 1, 0.0));



				// bv left l 3
		Sphere p_1 = Sphere(Vec3f(-5, 0, -20), 0.2, Vec3f(0.1, 0.77, 0.97), 1, 0.0); //yellow right
		Sphere p_2 = Sphere(Vec3f(-4.5, 0, -20), 0.2, Vec3f(0.1, 0.77, 0.97), 1, 0.0); //yellow right
		Sphere bv_l_3 = Sphere(Vec3f(-4.75, 0, -20), 0.6, Vec3f(0.2, 0.77, 0.33), 1, 0.0); //yellow right
		bv_l_3.setChildes(&p_1, &p_2);
		// bv right l 3
		Sphere p_3 = Sphere(Vec3f(-2, 0, -20), 0.2, Vec3f(0.1, 0.77, 0.97), 1, 0.0); //yellow right
		Sphere p_4 = Sphere(Vec3f(-1.5, 0, -20), 0.2, Vec3f(0.1, 0.77, 0.97), 1, 0.0); //yellow right
		Sphere bv_r_3 = Sphere(Vec3f(-1.75, 0, -20), 0.6, Vec3f(0.2, 0.77, 0.33), 1, 0.0); //yellow right
		bv_r_3.setChildes(&p_3, &p_4);

		// bv left l 2
		Sphere bv_l_2 = Sphere(Vec3f(-3.2, 0, -20), 2.5, Vec3f(0.2, 0.42, 0.33), 1, 0.0); //yellow right
		bv_l_2.setChildes(&bv_l_3, &bv_r_3);


		// bv left l 3
		Sphere p_5 = Sphere(Vec3f(1.5, 0, -20), 0.2, Vec3f(0.1, 0.77, 0.97), 1, 0.0); //yellow right
		Sphere p_6 = Sphere(Vec3f(2, 0, -20), 0.2, Vec3f(0.1, 0.77, 0.97), 1, 0.0); //yellow right
		Sphere bv_l_3_2 = Sphere(Vec3f(1.75, 0, -20), 0.6, Vec3f(0.2, 0.77, 0.33), 1, 0.0);
		bv_l_3_2.setChildes(&p_5, &p_6);

		// bv right l 3
		Sphere p_7 = Sphere(Vec3f(4.5, 0, -20), 0.2, Vec3f(0.1, 0.77, 0.97), 1, 0.0); //yellow right
		Sphere p_8 = Sphere(Vec3f(5, 0, -20), 0.2, Vec3f(0.1, 0.77, 0.97), 1, 0.0); //yellow right
		Sphere bv_r_3_2 = Sphere(Vec3f(4.75, 0, -20), 0.6, Vec3f(0.2, 0.77, 0.33), 1, 0.0);
		bv_r_3_2.setChildes(&p_7, &p_8);

		// bv right l 2
		Sphere bv_r_2 = Sphere(Vec3f(3.2, 0, -20), 2.5, Vec3f(0.2, 0.42, 0.33), 1, 0.0); //yellow right
		bv_r_2.setChildes(&bv_l_3_2, &bv_r_3_2);


		//bv root l1
		Sphere root = Sphere(Vec3f(0, 0, -20), 6, Vec3f(0.0, 0, 0), 1, 0.0); //yellow right
		root.setChildes(&bv_l_2, &bv_r_2);

		lights.push_back(light);
		lights.push_back(light2);

		//vector<int> v = { 1, 2,3, 4,5,6,7, 8,9,10,11,12,13,14,15 };

		vector<Sphere*> v = { &root, &bv_l_2,&bv_r_2, &bv_l_3,&bv_r_3,&bv_l_3_2,&bv_r_3_2, &p_1,&p_2,&p_3,&p_4,&p_5,&p_6,&p_7,&p_8 };
		TreeNode* tt = make_tree(v);
		//spheres.push_back(p_1);
		//spheres.push_back(p_2);
		//spheres.push_back(p_3);
		//spheres.push_back(p_4);
		//spheres.push_back(p_5);
		//spheres.push_back(p_6);
		//spheres.push_back(p_7);
		//spheres.push_back(p_8);

		//// This is a cube
		//triangles.push_back(Triangle(Vec3f(-1, -1 - sh_y, 1 - shift), Vec3f(1, -1 - sh_y, 1 - shift), Vec3f(1, 1 - sh_y, 1 - shift), Vec3f(0.1, 0.3, 0.46), 1, 0.0));
		//triangles.push_back(Triangle(Vec3f(-1, -1 - sh_y, 1 - shift), Vec3f(1, 1 - sh_y, 1 - shift), Vec3f(-1, 1 - sh_y, 1 - shift), Vec3f(0.90, 0.1, 0.46), 1, 0.0));
		//triangles.push_back(Triangle(Vec3f(1, -1 - sh_y, 1 - shift), Vec3f(1, -1 - sh_y, -1 - shift), Vec3f(1, 1 - sh_y, -1 - shift), Vec3f(0.90, 0.3, 0.1), 1, 0.0));
		//triangles.push_back(Triangle(Vec3f(1, -1 - sh_y, 1 - shift), Vec3f(1, 1 - sh_y, -1 - shift), Vec3f(1, 1 - sh_y, 1 - shift), Vec3f(0.1, 0.3, 0.46), 1, 0.0));
		//triangles.push_back(Triangle(Vec3f(1, -1 - sh_y, -1 - shift), Vec3f(-1, -1 - sh_y, -1 - shift), Vec3f(-1, 1 - sh_y, -1 - shift), Vec3f(0.90, 1, 0.46), 1, 0.0));
		//triangles.push_back(Triangle(Vec3f(1, -1 - sh_y, -1 - shift), Vec3f(-1, 1 - sh_y, -1 - shift), Vec3f(1, 1 - sh_y, -1 - shift), Vec3f(0.90, 0.3, 0.1), 1, 0.0));
		//triangles.push_back(Triangle(Vec3f(-1, -1 - sh_y, -1 - shift), Vec3f(-1, -1 - sh_y, 1 - shift), Vec3f(-1, 1 - sh_y, 1 - shift), Vec3f(0.1, 0.3, 0.46), 1, 0.0));
		//triangles.push_back(Triangle(Vec3f(-1, -1 - sh_y, -1 - shift), Vec3f(-1, 1 - sh_y, 1 - shift), Vec3f(-1, 1 - sh_y, -1 - shift), Vec3f(0.90, 0.1, 0.46), 1, 0.0));
		//triangles.push_back(Triangle(Vec3f(-1, 1 - sh_y, 1 - shift), Vec3f(1, 1 - sh_y, 1 - shift), Vec3f(1, 1 - sh_y, -1 - shift), Vec3f(0.90, 0.3, 0.1), 1, 0.0));
		//triangles.push_back(Triangle(Vec3f(-1, 1 - sh_y, 1 - shift), Vec3f(1, 1 - sh_y, -1 - shift), Vec3f(-1, 1 - sh_y, -1 - shift), Vec3f(0.1, 0.3, 0.46), 1, 0.0));
		//triangles.push_back(Triangle(Vec3f(1, -1 - sh_y, 1 - shift), Vec3f(-1, -1 - sh_y, -1 - shift), Vec3f(1, -1 - sh_y, -1 - shift), Vec3f(0.90, 0.1, 0.46), 1, 0.0));
		//triangles.push_back(Triangle(Vec3f(1, -1 - sh_y, 1 - shift), Vec3f(-1, -1 - sh_y, 1 - shift), Vec3f(-1, -1 - sh_y, -1 - shift), Vec3f(0.90, 0.3, 0.1), 1, 0.0));


		//// This is a tetrahidron
		//triangles.push_back(Triangle(Vec3f(-1.0 - sh_y, 1.0, -1.0 + 2 - shift), Vec3f(1.0 - sh_y, -1.0, -1.0 + 2 - shift), Vec3f(-1.0 - sh_y, -1.0, 1.0 + 2 - shift), Vec3f(0.2, 0.3, 0.1), 1, 0.0));
		//triangles.push_back(Triangle(Vec3f(1.0 - sh_y, 1.0, 1.0 + 2 - shift), Vec3f(-1.0 - sh_y, -1.0, 1.0 + 2 - shift), Vec3f(1.0 - sh_y, -1.0, -1.0 + 2 - shift), Vec3f(0.90, 0.3, 0.1), 1, 0.0));
		//triangles.push_back(Triangle(Vec3f(1.0 - sh_y, 1.0, 1.0 + 2 - shift), Vec3f(-1.0 - sh_y, 1.0, -1.0 + 2 - shift), Vec3f(-1.0 - sh_y, -1.0, 1.0 + 2 - shift), Vec3f(0.90, 0.3, 0.6), 1, 0.0));
		//triangles.push_back(Triangle(Vec3f(1.0 - sh_y, 1.0, 1.0 + 2 - shift), Vec3f(1.0 - sh_y, -1.0, -1.0 + 2 - shift), Vec3f(-1.0 - sh_y, 1.0, -1.0 + 2 - shift), Vec3f(0.90, 0.7, 0.1), 1, 0.0));

		render(settings, spheres, lights, triangles, frame, tt);
	}
	return 0;
}