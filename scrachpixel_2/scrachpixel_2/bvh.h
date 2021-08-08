#ifndef BVH_H
#define BVH_H
//==============================================================================================
// To the extent possible under law, the author(s) have dedicated all copyright and related and
// neighboring rights to this software to the public domain worldwide. This software is
// distributed without any warranty.
//
// You should have received a copy (see file COPYING.txt) of the CC0 Public Domain Dedication
// along with this software. If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.
//==============================================================================================

#include <iostream>
#include "geometry.h"
using namespace std;

// This is the node BVH uses
class Node
{
public:
	int left; // left child id
	int right; //right child id
	bool isleaf = false;
	int objs[3]; // Each node saves three objects
	int numObjs;
	//some bounding box variables 
	// here I can create BBOX node is general
	double minX;
	double maxX;
	double minY;
	double maxY;
	double minZ;
	double maxZ;

	double midpoint;
	double longestAxis;
};

// This can be sphere, Triangle or anything
class SceneObject
{
public:
	int objId; // this is the ID of the object
	Vec3f position;
	Vec3f ambient;
	Vec3f diffuse;
	Vec3f specular;
	Vec3f reflective;
	float radius;
	Vec3f center;
	float shininess;
	bool sphere;
};

inline void degrees_to_radians(double degrees) {
	cout << "Hello World!";
}

// Common Headers

//#include "ray.h"
//#include "vec3.h"


#endif
