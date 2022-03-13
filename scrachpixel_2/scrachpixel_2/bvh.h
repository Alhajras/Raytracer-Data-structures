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

#include<list>
#include <iostream>
#include "geometry.h"
#include <vector>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <map>

using namespace std;
enum AccType { BVH, KDTREE, UNIFORM_GRID, LBVH, NONE };
enum MaterialType { DIFFUSE_AND_GLOSSY, REFLECTION_AND_REFRACTION, REFLECTION };
using Vec3f = Vec3<float>;
using Vec3b = Vec3<bool>;
using Vec3i = Vec3<int32_t>;
using Vec3ui = Vec3<uint32_t>;
using Matrix44f = Matrix44<float>;
// igea [20:50]
// bunny [10:15]
const int MaxLeaves = 2;
static int bytePrefix[] = {
		8, 7, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4,
		3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
		2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
		2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
inline
Vec3f mix(const Vec3f& a, const Vec3f& b, const float& mixValue)
{
	return a * (1 - mixValue) + b * mixValue;
}

Vec3f midLeft;
float maxLeftX = -1 * std::numeric_limits<float>::max();
float minLeftX = std::numeric_limits<float>::max();
float maxLeftY = -1 * std::numeric_limits<float>::max();
float minLeftY = std::numeric_limits<float>::max();
float maxLeftZ = -1 * std::numeric_limits<float>::max();
float minLeftZ = std::numeric_limits<float>::max();
Vec3f midRight;
float maxRightX = -1 * std::numeric_limits<float>::max();
float minRightX = std::numeric_limits<float>::max();
float maxRightY = -1 * std::numeric_limits<float>::max();
float minRightY = std::numeric_limits<float>::max();
float maxRightZ = -1 * std::numeric_limits<float>::max();
float minRightZ = std::numeric_limits<float>::max();


class Sphere
{
public:
	int id;
	Vec3f center;                           /// position of the sphere
	float radius, radius2;                  /// sphere radius and radius^2
	Vec3f surfaceColor, emissionColor;      /// surface color and emission (light)
	float transparency, reflection;         /// surface transparency and reflectivity
	MaterialType materialType;
	bool is_bv = false;
	Sphere(
		const int& id,
		const Vec3f& c,
		const float& r,
		const Vec3f& sc, const float& refl = 0,
		const float& transp = 0,
		const Vec3f& ec = 0) :
		id(id), center(c), radius(r), radius2(r* r), surfaceColor(sc), emissionColor(ec),
		transparency(transp), reflection(refl), materialType(DIFFUSE_AND_GLOSSY)
	{ /* empty */
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


// This is the node BVH uses
class Node
{
public:
	unsigned int left = 0; // left child id
	unsigned int right = 0; //right child id
	bool isleaf = false;
	std::vector<int> objs; // Each node saves three objects
	unsigned int objsMorID[3]; // Each node saves three objects
	unsigned int numObjs;
	//some bounding box variables 
	// here I can create BBOX node is general
	float minX;
	float maxX;
	float minY;
	float maxY;
	float minZ;
	float maxZ;

	float midpoint;
	float longestAxis;

	// this is for KDtree
	std::vector<unsigned int> kdLeafChildren;
};


class MoreInfo
{
public:
	int code; // left child id

};

template<typename T> inline T clamp(const T& v, const T& lo, const T& hi)
{
	return std::max(lo, std::min(v, hi));
}

// This can be sphere, Triangle or anything
class SceneObject
{
public:
	int objId; // this is the ID of the object
	Vec3f position;
	float radius;
	Vec3f center;
	float shininess;
	bool isSphere;
	Sphere sphere = Sphere(0, Vec3f(-5, 0, -20), 98, Vec3f(0.20, 0.20, 0.20), 0, 0.0);

	Sphere getSphere() {
		return sphere;
	}
};



int constructBVHTree(std::vector<std::shared_ptr<SceneObject>>& objects, std::shared_ptr<Node> currentNode, std::vector<std::shared_ptr<Node>>& nodes)
{   // If this is a leaf node
	if (objects.size() <= MaxLeaves) // this measn we only have one node and two children
	{
		for (int i = 0; i < (int)objects.size(); i++)
		{
			currentNode->objs.push_back(objects[i]->objId); //we assign the ids of the root node, left and right nodes
		}
		currentNode->numObjs = (int)objects.size(); // how many objects node has
		currentNode->isleaf = true; // leaf node has two objects as children
		nodes.push_back(currentNode);
		return (int)nodes.size() - 1;
	}

	// If it is not a leaf node
	std::shared_ptr<Node> newLeftNode = std::make_shared<Node>();
	newLeftNode->left = NULL;
	newLeftNode->right = NULL;
	std::shared_ptr<Node> newRightNode = std::make_shared<Node>();
	newRightNode->left = NULL;
	newRightNode->right = NULL;
	std::vector<std::shared_ptr<SceneObject>> leftObjects;
	Vec3f midLeft;
	float maxLeftX = -1 * std::numeric_limits<float>::max();
	float minLeftX = std::numeric_limits<float>::max();
	float maxLeftY = -1 * std::numeric_limits<float>::max();
	float minLeftY = std::numeric_limits<float>::max();
	float maxLeftZ = -1 * std::numeric_limits<float>::max();
	float minLeftZ = std::numeric_limits<float>::max();
	Vec3f midRight;
	float maxRightX = -1 * std::numeric_limits<float>::max();
	float minRightX = std::numeric_limits<float>::max();
	float maxRightY = -1 * std::numeric_limits<float>::max();
	float minRightY = std::numeric_limits<float>::max();
	float maxRightZ = -1 * std::numeric_limits<float>::max();
	float minRightZ = std::numeric_limits<float>::max();
	std::vector<std::shared_ptr<SceneObject>> rightObjects;
	for (int i = 0; i < objects.size(); i++)
	{
		// here I am splitting the objects which are their center is in the left of the 
		// Middle point of the node to the left, we can change the currentNode.midpoint,
		// By using splitting average or other ways look at my presentations
		if (objects[i]->position[currentNode->longestAxis] < currentNode->midpoint)
		{
			// we create the new left bbox parameters
			if (objects[i]->position[0] - objects[i]->radius < minLeftX)
				minLeftX = objects[i]->position[0] - objects[i]->radius;
			if (objects[i]->position[1] - objects[i]->radius < minLeftY)
				minLeftY = objects[i]->position[1] - objects[i]->radius;
			if (objects[i]->position[2] - objects[i]->radius < minLeftZ)
				minLeftZ = objects[i]->position[2] - objects[i]->radius;

			if (objects[i]->position[0] + objects[i]->radius > maxLeftX)
				maxLeftX = objects[i]->position[0] + objects[i]->radius;
			if (objects[i]->position[1] + objects[i]->radius > maxLeftY)
				maxLeftY = objects[i]->position[1] + objects[i]->radius;
			if (objects[i]->position[2] + objects[i]->radius > maxLeftZ)
				maxLeftZ = objects[i]->position[2] + objects[i]->radius;

			midLeft += objects[i]->position; // here we are using the average for middle point 
			leftObjects.push_back(objects[i]);
		}
		else
		{
			// we create the new right bbox parameters
			if (objects[i]->position[0] - objects[i]->radius < minRightX)
				minRightX = objects[i]->position[0] - objects[i]->radius;
			if (objects[i]->position[1] - objects[i]->radius < minRightY)
				minRightY = objects[i]->position[1] - objects[i]->radius;
			if (objects[i]->position[2] - objects[i]->radius < minRightZ)
				minRightZ = objects[i]->position[2] - objects[i]->radius;

			if (objects[i]->position[0] + objects[i]->radius > maxRightX)
				maxRightX = objects[i]->position[0] + objects[i]->radius;
			if (objects[i]->position[1] + objects[i]->radius > maxRightY)
				maxRightY = objects[i]->position[1] + objects[i]->radius;
			if (objects[i]->position[2] + objects[i]->radius > maxRightZ)
				maxRightZ = objects[i]->position[2] + objects[i]->radius;

			midRight += objects[i]->position; // here we are using the average for middle point 
			rightObjects.push_back(objects[i]);
		}
	}

	midLeft = Vec3f(midLeft[0] / leftObjects.size(), midLeft[1] / leftObjects.size(), midLeft[2] / leftObjects.size());
	midRight = Vec3f(midRight[0] / rightObjects.size(), midRight[1] / rightObjects.size(), midRight[2] / rightObjects.size());

	// we try to find the midpoint and longest axis of the left box
	if (maxLeftX - minLeftX > maxLeftY - minLeftY)
	{
		if (maxLeftX - minLeftX > maxLeftZ - minLeftZ)
		{
			newLeftNode->longestAxis = 0;
			newLeftNode->midpoint = midLeft[0];
		}
	}
	if (maxLeftY - minLeftY > maxLeftX - minLeftX)
	{
		if (maxLeftY - minLeftY > maxLeftZ - minLeftZ)
		{
			newLeftNode->longestAxis = 1;
			newLeftNode->midpoint = midLeft[1];
		}
	}
	if (maxLeftZ - minLeftZ > maxLeftX - minLeftX)
	{
		if (maxLeftZ - minLeftZ > maxLeftY - minLeftY)
		{
			newLeftNode->longestAxis = 2;
			newLeftNode->midpoint = midLeft[2];
		}
	}

	// we try to find the midpoint and longest axis of the right box
	if (maxRightX - minRightX > maxRightY - minRightY)
	{
		if (maxRightX - minRightX > maxRightZ - minRightZ)
		{
			newRightNode->longestAxis = 0;
			newRightNode->midpoint = midRight[0];
		}
	}
	if (maxRightY - minRightY > maxRightX - minRightX)
	{
		if (maxRightY - minRightY > maxRightZ - minRightZ)
		{
			newRightNode->longestAxis = 1;
			newRightNode->midpoint = midRight[1];
		}
	}
	if (maxRightZ - minRightZ > maxRightX - minRightX)
	{
		if (maxRightZ - minRightZ > maxRightY - minRightY)
		{
			newRightNode->longestAxis = 2;
			newRightNode->midpoint = midRight[2];
		}
	}

	newLeftNode->minX = minLeftX;
	newLeftNode->maxX = maxLeftX;
	newLeftNode->minY = minLeftY;
	newLeftNode->maxY = maxLeftY;
	newLeftNode->minZ = minLeftZ;
	newLeftNode->maxZ = maxLeftZ;

	newRightNode->minX = minRightX;
	newRightNode->maxX = maxRightX;
	newRightNode->minY = minRightY;
	newRightNode->maxY = maxRightY;
	newRightNode->minZ = minRightZ;
	newRightNode->maxZ = maxRightZ;

	currentNode->left = constructBVHTree(leftObjects, newLeftNode, nodes);
	currentNode->right = constructBVHTree(rightObjects, newRightNode, nodes);

	nodes.push_back(currentNode);
	return (int)nodes.size() - 1;
}



////////////////// LBVH /////////////////////////

// C++ implementation of Radix Sort

void display(int* array, int size) {
	for (int i = 0; i < size; i++)
		cout << array[i] << " ";
	cout << endl;
}
void radixSort(std::vector<unsigned int> mortonEncode, int n, int max) {
	int i, j, m, p = 1, index, temp, count = 0;
	list<int> pocket[10];      //radix of decimal number is 10
	for (i = 0; i < max; i++) {
		m = pow(10, i + 1);
		p = pow(10, i);
		for (j = 0; j < n; j++) {
			temp = mortonEncode[j] % m;
			index = temp / p;      //find index for pocket array
			pocket[index].push_back(mortonEncode[j]);
		}
		count = 0;
		for (j = 0; j < 10; j++) {
			//delete from linked lists and store to array
			while (!pocket[j].empty()) {
				mortonEncode[count] = *(pocket[j].begin());
				pocket[j].erase(pocket[j].begin());
				count++;
			}
		}
	}
}

// https://developer.nvidia.com/blog/thinking-parallel-part-iii-tree-construction-gpu/
// Expands a 10-bit integer into 30 bits
// by inserting 2 zeros after each bit.
unsigned int expandBits(unsigned int v)
{
	v = (v * 0x00010001u) & 0xFF0000FFu;
	v = (v * 0x00000101u) & 0x0F00F00Fu;
	v = (v * 0x00000011u) & 0xC30C30C3u;
	v = (v * 0x00000005u) & 0x49249249u;
	return v;
}

// Calculates a 30-bit Morton code for the
// given 3D point located within the unit cube [0,1].
unsigned int morton3D(float x, float y, float z)
{
	x = min(max(x * 1024.0f, 0.0f), 1023.0f);
	y = min(max(y * 1024.0f, 0.0f), 1023.0f);
	z = min(max(z * 1024.0f, 0.0f), 1023.0f);
	unsigned int xx = expandBits((unsigned int)x);
	unsigned int yy = expandBits((unsigned int)y);
	unsigned int zz = expandBits((unsigned int)z);
	return xx * 4 + yy * 2 + zz;
}


int findSplit(std::vector<unsigned int> sortedMortonCodes,
	int           first,
	int           last)
{
	// Identical Morton codes => split the range in the middle.

	int firstCode = sortedMortonCodes[first];
	int lastCode = sortedMortonCodes[last];

	if (firstCode == lastCode)
		return (first + last) >> 1;

	// Calculate the number of highest bits that are the same
	// for all objects, using the count-leading-zeros intrinsic.

	//int x = firstCode ^ lastCode;

	int c = firstCode ^ lastCode;
	int len = -1;
	while ((++len < 8) && ((c & 0x80) == 0))
		c = c << 1;

	int commonPrefix = len;

	// Use binary search to find where the next bit differs.
	// Specifically, we are looking for the highest object that
	// shares more than commonPrefix bits with the first one.

	int split = first; // initial guess
	int step = last - first;

	do
	{
		step = (step + 1) >> 1; // exponential decrease
		int newSplit = split + step; // proposed new position

		if (newSplit < last)
		{
			unsigned int splitCode = sortedMortonCodes[newSplit];

			c = firstCode ^ splitCode;
			len = -1;
			while ((++len < 8) && ((c & 0x80) == 0))
				c = c << 1;

			int splitPrefix = len;
			if (splitPrefix > commonPrefix)
				split = newSplit; // accept proposal
		}
	} while (step > 1);

	return split;
}


int generateHierarchy(std::vector<SceneObject> objects,
	std::vector<unsigned int> sortedMortonCodes,
	std::shared_ptr<Node> currentNode, std::vector<std::shared_ptr<Node>>& nodes,
	uint64_t           first,
	uint64_t           last)
{

	if (first == last) // this measn we only have one node and two children
	{
		for (int i = 0; i < (int)sortedMortonCodes.size(); i++)
		{
			currentNode->objsMorID[i] = sortedMortonCodes[i]; //we assign the ids of the root node, left and right nodes
		}
		currentNode->numObjs = (int)sortedMortonCodes.size(); // how many objects node has
		currentNode->isleaf = true; // leaf node has two objects as children
		nodes.push_back(currentNode);
		return (int)nodes.size() - 1;
	}
	//if (first == last)
	//{
	//	currentNode.isleaf = true; // leaf node has two objects as children
	//	nodes.push_back(currentNode);
	//	return (int)nodes.size() - 1;

	//}

	int split = findSplit(sortedMortonCodes, first, last);

	std::shared_ptr<Node> newLeftNode = std::make_shared<Node>();
	newLeftNode->left = NULL;
	newLeftNode->right = NULL;
	std::shared_ptr<Node> newRightNode = std::make_shared<Node>();
	newRightNode->left = NULL;
	newRightNode->right = NULL;


	/*	for (int i = 0; i < (int)objects.size(); i++)
		{
			if (objects[i].objId == first){

			}
			if (objects[i].objId == last) {

			}
		}*/
	newLeftNode->minX = currentNode->minX;
	newLeftNode->maxX = currentNode->maxX / 2;
	newLeftNode->minY = currentNode->minY;
	newLeftNode->maxY = currentNode->maxY / 2;
	newLeftNode->minZ = currentNode->minZ;
	newLeftNode->maxZ = currentNode->maxZ / 3;

	newRightNode->minX = currentNode->maxX / 2;
	newRightNode->maxX = currentNode->maxX;
	newRightNode->minY = currentNode->maxY / 2;
	newRightNode->maxY = currentNode->maxY;
	newRightNode->maxY = currentNode->maxY;
	newRightNode->minZ = currentNode->maxZ / 2;
	newRightNode->maxZ = currentNode->maxZ;
#	/*leftObjects.push_back(objects[i]);
	leftObjects.push_back(objects[i]);*/


	int l = generateHierarchy(objects, sortedMortonCodes, newLeftNode, nodes,
		first, split);
	int r = generateHierarchy(objects, sortedMortonCodes, newRightNode, nodes,
		split + 1, last);


	currentNode->left = l;
	currentNode->right = r;

	nodes.push_back(currentNode);
	return (int)nodes.size() - 1;
}

int constructLBVHTree(std::map<unsigned int, SceneObject >& hashMap, std::vector<SceneObject>& objects, std::shared_ptr<Node> currentNode, std::vector<std::shared_ptr<Node>>& nodes)
{
	std::vector<unsigned int> sortedMortonCodes;
	// we map the centrod of spheres
	for (uint64_t i = 0; i < objects.size(); ++i) {
		Vec3f centroid = (objects[i].center + 30) / 1000;
		unsigned int moCode = morton3D(centroid.x, centroid.y, centroid.z);
		objects[i].objId = moCode;
		sortedMortonCodes.push_back(moCode);
		hashMap[moCode] = objects[i];
		std::cout << sortedMortonCodes[i] << "\n";
	}
	radixSort(sortedMortonCodes, sortedMortonCodes.size(), 10);

	return generateHierarchy(objects, sortedMortonCodes, currentNode, nodes, 0, sortedMortonCodes.size() - 1);
}

int constructKDTree(std::vector<SceneObject>& objects, std::shared_ptr<Node> currentNode, std::vector<std::shared_ptr<Node>>& nodes, int depth)
{
	//bool shouldSplit;
	//int count = objects.size();

	//// If this is a leaf node
	//if(count <= 3 || (depth >= 20 && count <= 2))
	//{
	//	for (int i = 0; i < (int)objects.size(); i++)
	//	{
	//		currentNode.objs[i] = objects[i].objId; //we assign the ids of the root node, left and right nodes
	//	}
	//	currentNode.numObjs = (int)objects.size(); // how many objects node has
	//	currentNode.isleaf = true; // leaf node has two objects as children
	//	nodes.push_back(currentNode);
	//	return (int)nodes.size() - 1;
	//}


	   // If this is a leaf node
	if (objects.size() <= 15) // this measn we only have one node and two children
	{
		for (int i = 0; i < (int)objects.size(); i++)
		{
			currentNode->objs[i] = objects[i].objId; //we assign the ids of the root node, left and right nodes
		}
		currentNode->numObjs = (int)objects.size(); // how many objects node has
		currentNode->isleaf = true; // leaf node has two objects as children
		nodes.push_back(currentNode);
		return (int)nodes.size() - 1;
	}


	// If it is not a leaf node
	std::shared_ptr<Node> newLeftNode = std::make_shared<Node>();
	newLeftNode->left = NULL;
	newLeftNode->right = NULL;
	std::shared_ptr<Node> newRightNode = std::make_shared<Node>();
	newRightNode->left = NULL;
	newRightNode->right = NULL;
	std::vector<int> leftPointers;
	std::vector<SceneObject> leftObjects;
	Vec3f midLeft;
	float maxLeftX = -1 * std::numeric_limits<float>::max();
	float minLeftX = std::numeric_limits<float>::max();
	float maxLeftY = -1 * std::numeric_limits<float>::max();
	float minLeftY = std::numeric_limits<float>::max();
	float maxLeftZ = -1 * std::numeric_limits<float>::max();
	float minLeftZ = std::numeric_limits<float>::max();
	Vec3f midRight;
	float maxRightX = -1 * std::numeric_limits<float>::max();
	float minRightX = std::numeric_limits<float>::max();
	float maxRightY = -1 * std::numeric_limits<float>::max();
	float minRightY = std::numeric_limits<float>::max();
	float maxRightZ = -1 * std::numeric_limits<float>::max();
	float minRightZ = std::numeric_limits<float>::max();
	std::vector<int> rightPointers;
	std::vector<SceneObject> rightObjects;
	for (int i = 0; i < objects.size(); i++)
	{
		// here I am splitting the objects which are their center is in the left of the 
		// Middle point of the node to the left, we can change the currentNode.midpoint,
		// By using splitting average or other ways look at my presentations
		if (objects[i].position[currentNode->longestAxis] < currentNode->midpoint)
		{
			// we create the new left bbox parameters
			if (objects[i].position[0] - objects[i].radius < minLeftX)
				minLeftX = objects[i].position[0] - objects[i].radius;
			if (objects[i].position[1] - objects[i].radius < minLeftY)
				minLeftY = objects[i].position[1] - objects[i].radius;
			if (objects[i].position[2] - objects[i].radius < minLeftZ)
				minLeftZ = objects[i].position[2] - objects[i].radius;

			if (objects[i].position[0] + objects[i].radius > maxLeftX)
				maxLeftX = objects[i].position[0] + objects[i].radius;
			if (objects[i].position[1] + objects[i].radius > maxLeftY)
				maxLeftY = objects[i].position[1] + objects[i].radius;
			if (objects[i].position[2] + objects[i].radius > maxLeftZ)
				maxLeftZ = objects[i].position[2] + objects[i].radius;

			midLeft += objects[i].position; // here we are using the average for middle point 
			leftObjects.push_back(objects[i]);
		}
		else
		{
			// we create the new right bbox parameters
			if (objects[i].position[0] - objects[i].radius < minRightX)
				minRightX = objects[i].position[0] - objects[i].radius;
			if (objects[i].position[1] - objects[i].radius < minRightY)
				minRightY = objects[i].position[1] - objects[i].radius;
			if (objects[i].position[2] - objects[i].radius < minRightZ)
				minRightZ = objects[i].position[2] - objects[i].radius;

			if (objects[i].position[0] + objects[i].radius > maxRightX)
				maxRightX = objects[i].position[0] + objects[i].radius;
			if (objects[i].position[1] + objects[i].radius > maxRightY)
				maxRightY = objects[i].position[1] + objects[i].radius;
			if (objects[i].position[2] + objects[i].radius > maxRightZ)
				maxRightZ = objects[i].position[2] + objects[i].radius;

			midRight += objects[i].position; // here we are using the average for middle point 
			rightObjects.push_back(objects[i]);
		}
	}

	midLeft = Vec3f(midLeft[0] / leftObjects.size(), midLeft[1] / leftObjects.size(), midLeft[2] / leftObjects.size());
	midRight = Vec3f(midRight[0] / rightObjects.size(), midRight[1] / rightObjects.size(), midRight[2] / rightObjects.size());

	int axisindex = currentNode->longestAxis;
	switch (axisindex) {
	case 0:
		newLeftNode->longestAxis = 1;
		newLeftNode->midpoint = midLeft[1];
		newRightNode->longestAxis = 1;
		newRightNode->midpoint = midRight[1];

		break;
	case 1:
		newLeftNode->longestAxis = 2;
		newLeftNode->midpoint = midLeft[2];
		newRightNode->longestAxis = 2;
		newRightNode->midpoint = midRight[2];

		break;
	default:
		newLeftNode->longestAxis = 0;
		newLeftNode->midpoint = midLeft[0];
		newRightNode->longestAxis = 0;
		newRightNode->midpoint = midRight[0];

	}


	newLeftNode->minX = minLeftX;
	newLeftNode->maxX = maxLeftX;
	newLeftNode->minY = minLeftY;
	newLeftNode->maxY = maxLeftY;
	newLeftNode->minZ = minLeftZ;
	newLeftNode->maxZ = maxLeftZ;

	newRightNode->minX = minRightX;
	newRightNode->maxX = maxRightX;
	newRightNode->minY = minRightY;
	newRightNode->maxY = maxRightY;
	newRightNode->minZ = minRightZ;
	newRightNode->maxZ = maxRightZ;

	int l = constructKDTree(leftObjects, newLeftNode, nodes, 3);
	int r = constructKDTree(rightObjects, newRightNode, nodes, 3);

	currentNode->left = l;
	currentNode->right = r;

	nodes.push_back(currentNode);
	return (int)nodes.size() - 1;
}


bool boundingBoxIntersection(Vec3f position, Vec3f direction, std::shared_ptr<Node> box)
{	
	float tmin = (box->minX - position[0]) / direction[0];
	float tmax = (box->maxX - position[0]) / direction[0];

	if (tmin > tmax)
		std::swap(tmin, tmax);

	float tymin = (box->minY - position[1]) / direction[1];
	float tymax = (box->maxY - position[1]) / direction[1];

	if (tymin > tymax)
		std::swap(tymin, tymax);

	if ((tmin > tymax) || (tymin > tmax))
		return false;

	if (tymin > tmin)
		tmin = tymin;

	if (tymax < tmax)
		tmax = tymax;

	float tzmin = (box->minZ - position[2]) / direction[2];
	float tzmax = (box->maxZ - position[2]) / direction[2];

	if (tzmin > tzmax)
		std::swap(tzmin, tzmax);

	if ((tmin > tzmax) || (tzmin > tmax))
		return false;

	if (tzmin > tmin)
		tmin = tzmin;

	if (tzmax < tmax)
		tmax = tzmax;

	return true;
}



void bvhTraverse(Vec3f position, Vec3f direction, std::vector<std::shared_ptr<Node>>& tree, int currentNode, std::vector<int>& boxes)
{
	int retValue = -1;
	if (tree[currentNode]->isleaf)
	{
		if (boundingBoxIntersection(position, direction, tree[currentNode]))
		{
			retValue = currentNode;
			boxes.push_back(retValue);
		}
	}
	else
	{
		if (boundingBoxIntersection(position, direction, tree[tree[currentNode]->left]))
			bvhTraverse(position, direction, tree, tree[currentNode]->left, boxes);
		if (boundingBoxIntersection(position, direction, tree[tree[currentNode]->right]))
			bvhTraverse(position, direction, tree, tree[currentNode]->right, boxes);
	}
}




//////////////////////////////////////// Uniform Grid //////////////////////////////////

const float kInfinity = std::numeric_limits<float>::max();

uint32_t numPrimaryRays(0);
uint32_t numRayTriangleTests(0);
uint32_t numRayTriangleIntersections(0);
uint32_t  numRayBBoxTests(0);
uint32_t numRayBoundingVolumeTests(0);


template<typename T = float>
class BBox
{
public:
	BBox() {}
	BBox(Vec3<T> min_, Vec3<T> max_)
	{
		bounds[0] = min_;
		bounds[1] = max_;
	}
	BBox& extendBy(const Vec3<T>& p)
	{
		if (p.x < bounds[0].x) bounds[0].x = p.x;
		if (p.y < bounds[0].y) bounds[0].y = p.y;
		if (p.z < bounds[0].z) bounds[0].z = p.z;
		if (p.x > bounds[1].x) bounds[1].x = p.x;
		if (p.y > bounds[1].y) bounds[1].y = p.y;
		if (p.z > bounds[1].z) bounds[1].z = p.z;

		return *this;
	}
	/*inline */ Vec3<T> centroid() const { return (bounds[0] + bounds[1]) * 0.5; }
	Vec3<T>& operator [] (bool i) { return bounds[i]; }
	const Vec3<T> operator [] (bool i) const { return bounds[i]; }
	bool intersect(const Vec3<T>&, const Vec3<T>&, const Vec3b&, float&) const;
	Vec3<T> bounds[2] = { kInfinity, -kInfinity };
};

template<typename T>
bool BBox<T>::intersect(const Vec3<T>& orig, const Vec3<T>& invDir, const Vec3b& sign, float& tHit) const
{
	numRayBBoxTests++;
	float tmin, tmax, tymin, tymax, tzmin, tzmax;

	tmin = (bounds[sign[0]].x - orig.x) * invDir.x;
	tmax = (bounds[1 - sign[0]].x - orig.x) * invDir.x;
	tymin = (bounds[sign[1]].y - orig.y) * invDir.y;
	tymax = (bounds[1 - sign[1]].y - orig.y) * invDir.y;

	if ((tmin > tymax) || (tymin > tmax))
		return false;

	if (tymin > tmin)
		tmin = tymin;
	if (tymax < tmax)
		tmax = tymax;

	tzmin = (bounds[sign[2]].z - orig.z) * invDir.z;
	tzmax = (bounds[1 - sign[2]].z - orig.z) * invDir.z;

	if ((tmin > tzmax) || (tzmin > tmax))
		return false;

	if (tzmin > tmin)
		tmin = tzmin;
	if (tzmax < tmax)
		tmax = tzmax;

	tHit = tmin;

	return true;
}


struct CellHG
{
	CellHG() {}

	void insert(SceneObject s)
	{
		spheres.push_back(s);
	}

	void intersect(const Vec3f&, const Vec3f&, const uint32_t&, float&, Sphere&) const;

	std::vector<SceneObject> spheres;
};

class Grid
{
public:
	Grid(std::vector<SceneObject> spheres);
	~Grid()
	{
		//for (uint32_t i = 0; i < resolution[0] * resolution[1] * resolution[2]; ++i)
		//	if (cells[i] != NULL) delete cells[i];
		//delete[] cells;
	}
	void intersect(const Vec3f&, const Vec3f&, const uint32_t&, float&, Sphere& sphere) const;
	//Cell** cells;
	std::vector<CellHG> cells;
	BBox<> bbox;
	Vec3<uint32_t> resolution;
	Vec3f cellDimension;
};

std::vector<CellHG> cells_S;



Grid::Grid(std::vector<SceneObject> spheres) {
	uint32_t totalNumTriangles = spheres.size();

	Node root;

	for (SceneObject obj : spheres)
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

	//min
	bbox.bounds[0].x = root.minX;
	bbox.bounds[0].y = root.minY;
	bbox.bounds[0].z = root.minZ;

	//max
	bbox.bounds[1].x = root.maxX;
	bbox.bounds[1].y = root.maxY;
	bbox.bounds[1].z = root.maxZ;


	// Create the grid
	Vec3f size = bbox[1] - bbox[0];
	float cubeRoot = std::powf(totalNumTriangles / (size.x * size.y * size.z), 1. / 3.f);
	for (uint8_t i = 0; i < 3; ++i) {
		resolution[i] = std::floor(size[i] * cubeRoot);
		if (resolution[i] < 1) resolution[i] = 1;
		if (resolution[i] > 128) resolution[i] = 128;
	}
	//resolution[0] = 1;
	//resolution[1] = 1;
	//resolution[2] = 1;
	// I have fixed this try to change it
	cellDimension = size / resolution;

	// [comment]
	// Allocate memory - note that we don't create the cells yet at this
	// point but just an array of pointers to cell. We will create the cells
	// dynamically later when we are sure to insert something in them
	// [/comment]
	uint32_t numCells = resolution.x * resolution.y * resolution.z;

	//cells = new Grid::Cell * [numCells];
	for (uint32_t index = 0; index < numCells; ++index) {
		cells_S.push_back(CellHG());
	}
	//memset(cells, 0x0, sizeof(Grid) * numCells);

	for (const auto& sphere : spheres) {

		Vec3f min = 10000000000;
		Vec3f max = -10000000000;
		min.x = sphere.position.x - sphere.radius;
		min.y = sphere.position.y - sphere.radius;
		min.z = sphere.position.z - sphere.radius;

		max.x = sphere.position.x + sphere.radius;
		max.y = sphere.position.y + sphere.radius;
		max.z = sphere.position.z + sphere.radius;

		// Convert to cell coordinates
		min = (min - bbox[0]) / cellDimension;
		max = (max - bbox[0]) / cellDimension;


		uint32_t zmin = clamp<uint32_t>(std::floor(min[2]), 0, resolution[2] - 1);
		uint32_t zmax = clamp<uint32_t>(std::floor(max[2]), 0, resolution[2] - 1);
		uint32_t ymin = clamp<uint32_t>(std::floor(min[1]), 0, resolution[1] - 1);
		uint32_t ymax = clamp<uint32_t>(std::floor(max[1]), 0, resolution[1] - 1);
		uint32_t xmin = clamp<uint32_t>(std::floor(min[0]), 0, resolution[0] - 1);
		uint32_t xmax = clamp<uint32_t>(std::floor(max[0]), 0, resolution[0] - 1);
		// Loop over the cells the triangle overlaps and insert
		for (uint32_t z = zmin; z <= zmax; ++z) {
			for (uint32_t y = ymin; y <= ymax; ++y) {
				for (uint32_t x = xmin; x <= xmax; ++x) {
					uint32_t index = z * resolution[0] * resolution[1] + y * resolution[0] + x;
					std::cout << index << "\n";
					cells_S[index].insert(sphere);
				}
			}
		}
	}
	std::cout << "";
}


void CellHG::intersect(
	const Vec3f& orig, const Vec3f& dir, const uint32_t& rayId,
	float& tHit, Sphere& sphere) const
{
	float uhit, vhit;
	float t0, t1;
	Vec3f  hitColor = Vec3f(0.6, 0.8, 1);

	for (uint32_t i = 0; i < spheres.size(); ++i) {
		// [comment]
		// Be sure that rayId is never 0 - because all mailbox values
		// in the array are initialized with 0 too
		// [/comment]

		// TODO Add the mailbox here
		//if (rayId != triangles[i].mesh->mailbox[triangles[i].tri]) {
			//triangles[i].mesh->mailbox[triangles[i].tri] = rayId;
		float t, u, v;
		float t0 = INFINITY, t1 = INFINITY;
		if (spheres[i].sphere.raySphereIntersect(orig, dir, t0, t1)) {
			if (t0 < 0) t0 = t1;
			if (t0 < tHit) {
				tHit = t0;
				sphere = spheres[i].sphere;
				return;
				//sphere = &spheres[i].sphere;
				//hitColor = sphere->surfaceColor;
			}
		}
		//}
	}

	return;
}


void Grid::intersect(const Vec3f& orig, const Vec3f& dir, const uint32_t& rayId, float& tHit, Sphere& sphere) const
{
	std::cout << rayId << "\n";
	//SceneObject sceneObject;
	Sphere* hitsphere = NULL;

	const Vec3f invDir = 1 / dir;
	const Vec3b sign(dir.x < 0, dir.y < 0, dir.z < 0);
	float tHitBox;
	if (!bbox.intersect(orig, invDir, sign, tHitBox)) return;

	// initialization step
	Vec3i exit, step, cell;
	Vec3f deltaT, nextCrossingT;
	for (uint8_t i = 0; i < 3; ++i) {
		// convert ray starting point to cell coordinates
		float rayOrigCell = ((orig[i] + dir[i] * tHitBox) - bbox[0][i]);
		cell[i] = clamp<uint32_t>(std::floor(rayOrigCell / cellDimension[i]), 0, resolution[i] - 1);
		if (dir[i] < 0) {
			deltaT[i] = -cellDimension[i] * invDir[i];
			nextCrossingT[i] = tHitBox + (cell[i] * cellDimension[i] - rayOrigCell) * invDir[i];
			exit[i] = -1;
			step[i] = -1;
		}
		else {
			deltaT[i] = cellDimension[i] * invDir[i];
			nextCrossingT[i] = tHitBox + ((cell[i] + 1) * cellDimension[i] - rayOrigCell) * invDir[i];
			exit[i] = resolution[i];
			step[i] = 1;
		}
	}

	// Walk through each cell of the grid and test for an intersection if
	// current cell contains geometry

	for (int i = 0; i < cells_S.size(); i++) {
		cells_S[i].intersect(orig, dir, rayId, tHit, sphere);
		if (!&sphere) return;

	}
	//while (1) {
	//	uint32_t o = cell[2] * resolution[0] * resolution[1] + cell[1] * resolution[0] + cell[0];
	//std:cout << o << "\n";
	//	//if (cells_S[o]. NULL) {
	//	b = cells_S[o].intersect(orig, dir, rayId, tHit, sceneObject);
	//	if (b) return true;
	//	//if (intersectedMesh != nullptr) { ray.color = cells[o]->color; }
	////}
	//	uint8_t k =
	//		((nextCrossingT[0] < nextCrossingT[1]) << 2) +
	//		((nextCrossingT[0] < nextCrossingT[2]) << 1) +
	//		((nextCrossingT[1] < nextCrossingT[2]));
	//	static const uint8_t map[8] = { 2, 1, 2, 1, 2, 2, 0, 0 };
	//	uint8_t axis = map[k];

	//	if (tHit < nextCrossingT[axis]) break;
	//	cell[axis] += step[axis];
	//	if (cell[axis] == exit[axis]) break;
	//	nextCrossingT[axis] += deltaT[axis];
	//}

	return;
}

// Common Headers

//#include "ray.h"
//#include "vec3.h"


#endif