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
#include <algorithm>

using namespace std;
enum AccType { BVH, KDTREE, UNIFORM_GRID, LBVH, NONE };
enum SplitMethod { MIDDLE };

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

class BoxBoundries
{
public:
	Vec3f min;
	Vec3f max;

	BoxBoundries()
	{ /* empty */
		min = std::numeric_limits<float>::max();
		max = -1 * std::numeric_limits<float>::max();
	}
	BoxBoundries(
		const Vec3f& min,
		const Vec3f& max) :
		min(min), max(max)
	{ /* empty */
	}

};


// This is the node BVH uses
class Node
{
public:
	std::shared_ptr<Node> leftchild; // left child id
	std::shared_ptr<Node> rightchild; //right child id

	unsigned int left = 0; // left child id
	unsigned int right = 0; //right child id
	unsigned int firstPrimOffset;
	bool isleaf = false;
	unsigned int nPrimitives = 0;
	//int objs; // Each node saves three objects
	std::vector<unsigned int> objsMorID; // Each node saves three objects
	std::vector<unsigned int> objs; // Each node saves three objects
	//some bounding box variables 
	// here I can create BBOX node is general
	float minX;
	float maxX;
	float minY;
	float maxY;
	float minZ;
	float maxZ;
	BoxBoundries boxBoundries;
	float midpoint;
	char longestAxis;
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
	BoxBoundries boxBoundries;
	Sphere sphere = Sphere(0, Vec3f(-5, 0, -20), 98, Vec3f(0.20, 0.20, 0.20), 0, 0.0);

	Sphere getSphere() {
		return sphere;
	}
};

int GeMaximumAxis(BoxBoundries box)  {
	Vec3f axises = Vec3f(box.max - box.min);

	if (axises.x > axises.y && axises.x > axises.z)
		return 0;
	else if (axises.y > axises.z)
		return 1;
	else
		return 2;
}

BoxBoundries JoinBounds(BoxBoundries box1, BoxBoundries box2) {
	float minX, maxX;
	float minY, maxY;
	float minZ, maxZ;
	minX = std::min(box1.min.x, box2.min.x);
	minY = std::min(box1.min.y, box2.min.y);
	minZ = std::min(box1.min.z, box2.min.z);

	maxX = std::max(box1.max.x, box2.max.x);
	maxY = std::max(box1.max.y, box2.max.y);
	maxZ = std::max(box1.max.z, box2.max.z);

	return BoxBoundries(Vec3f(minX, minY, minZ), Vec3f(maxX, maxY, maxZ));
}

BoxBoundries JoinBoxPopintBounds(BoxBoundries box1, Vec3f p2) {
	float minX, maxX;
	float minY, maxY;
	float minZ, maxZ;
	minX = std::min(box1.min.x, p2.x);
	minY = std::min(box1.min.y, p2.y);
	minZ = std::min(box1.min.z, p2.z);
								
	maxX = std::max(box1.max.x, p2.x);
	maxY = std::max(box1.max.y, p2.y);
	maxZ = std::max(box1.max.z, p2.z);

	return BoxBoundries(Vec3f(minX, minY, minZ), Vec3f(maxX, maxY, maxZ));
}

struct LinearBVHNode {
	BoxBoundries boxBoundries;
	union {
		int primitivesOffset;   // leaf
		int secondChildOffset;  // interior
	};
	uint16_t nPrimitives;  // 0 -> interior node
	uint8_t axis;          // interior node: xyz
	uint8_t pad[1];        // ensure 32 byte total size
};
int flattenBVHTree(std::shared_ptr<Node> node, int* offset) {
	//LinearBVHNode* linearNode = &nodes[*offset];
	//linearNode->boxBoundries = node->boxBoundries;
	int myOffset = (*offset)++;
	//if (node->nPrimitives > 0) {
	//	linearNode->primitivesOffset = node->firstPrimOffset;
	//	linearNode->nPrimitives = node->nPrimitives;
	//}
	//else {
	//	// Create interior flattened BVH node
	//	linearNode->axis = node->longestAxis;
	//	linearNode->nPrimitives = 0;
	//	flattenBVHTree(node->leftchild, offset);
	//	linearNode->secondChildOffset =
	//		flattenBVHTree(node->rightchild, offset);
	//}
	return myOffset;
}

std::shared_ptr<Node> constructBVHNew(
	std::vector<SceneObject>& allSceneObjects,
	int startIndex,
	int endIndex,
	int* totalNodes)
{   
	//We create the node and add the total nodes counter
	std::shared_ptr<Node> node = std::make_shared<Node>();
	(*totalNodes)++;
	
	//We compute the biggest bounding box that bound the whole scene
	BoxBoundries bounds;
	for (int i = startIndex; i < endIndex; ++i)
		bounds = JoinBounds(bounds, allSceneObjects[i].boxBoundries);
	node->boxBoundries = bounds;

	int objectsNumber = endIndex - startIndex;

	//For now we support only one leaf
	if (objectsNumber == 1) {
		// Create leaf
		node->isleaf = true;
		node->nPrimitives = 1;
		node->objs.push_back(allSceneObjects[startIndex].objId);
		return node;
	}

	/*Splitting part*/

	BoxBoundries centroidBounds = bounds;
	for (int i = startIndex; i < endIndex; ++i)
		centroidBounds = JoinBoxPopintBounds(centroidBounds, allSceneObjects[i].center);
	
	int dim = GeMaximumAxis(centroidBounds);

	//We start splitting the objects into two equal halfs
	int midSplitIndex = (startIndex + endIndex) / 2;

	/*This condition happens if more than one object have the save dimentions and position
	We can either save them all in a list but for now we can just pick the first object and create a leaf*/
	if (centroidBounds.max[dim] == centroidBounds.min[dim]) {
		node->isleaf = true;
		node->nPrimitives = 1;
		node->boxBoundries = bounds;
		for ( int obj = 0; obj< objectsNumber; obj++)
		node->objs.push_back(allSceneObjects[obj].objId);
		return node;
	}

	//We create a node that contains left and right chilren
	// This is the middle point of the new bounding box with respect to the longest axis
	//All object on the left will go to the left child and the right will go to the rightchild
	float pmid = (centroidBounds.min[dim] + centroidBounds.max[dim]) / 2;

	/*Iterator to the first element of the second group, this will splet objects to left and right and return the pointer 
	to the right group object*/
	SceneObject* midPtr = std::partition(
		&allSceneObjects[startIndex], &allSceneObjects[endIndex - 1] + 1,
		[dim, pmid](const SceneObject& pi) {
			return pi.center[dim] < pmid;
		});

	//We subtract one pointer from the midpointer
	midSplitIndex = midPtr - &allSceneObjects[0];

	if (midSplitIndex != startIndex && midSplitIndex != endIndex) { 
		midSplitIndex = (startIndex + endIndex) / 2;
		std::nth_element(&allSceneObjects[startIndex], &allSceneObjects[midSplitIndex],
			&allSceneObjects[endIndex - 1] + 1,
			[dim](const SceneObject& a,
				const SceneObject& b) {
					return a.center[dim] < b.center[dim];
			});
		}

	node->leftchild = constructBVHNew(allSceneObjects, startIndex, midSplitIndex, totalNodes);
	node->rightchild = constructBVHNew(allSceneObjects, midSplitIndex, endIndex, totalNodes);

	// This is not a leaf node so it does not conatin any object
	node->longestAxis = dim;
	node->nPrimitives = 0;

	return node;
}
std::shared_ptr<Node> constructBVHTreed(
	std::vector<SceneObject>& primitiveInfo,
	int start,
	int end,
	int*totalNodes,
	std::vector<std::shared_ptr<SceneObject>>& orderedPrims)
{
	std::shared_ptr<Node> node = std::make_shared<Node>();
	(*totalNodes)++;
	// Compute bounds of all primitives in BVH node
	BoxBoundries bounds;
	for (int i = start; i < end; ++i)
		bounds = JoinBounds(bounds, primitiveInfo[i].boxBoundries);
	int nPrimitives = end - start;
	if (nPrimitives == 1) {
		// Create leaf _BVHBuildNode_
		int firstPrimOffset = orderedPrims.size();
		for (int i = start; i < end; ++i) {
			int primNum = primitiveInfo[i].objId;
			orderedPrims.push_back(std::make_shared<SceneObject>(primitiveInfo[i]));
		}
		node->isleaf = true; // leaf node has two objects as children
		node->nPrimitives = nPrimitives;
		node->boxBoundries = bounds;
		node->firstPrimOffset = firstPrimOffset;
		//node->objs = primitiveInfo[start].objId;
		return node;
	}

	else {
		// Compute bound of primitive centroids, choose split dimension _dim_
		//BoxBoundries centroidBounds;
		//for (int i = start; i < end; ++i)
		//	centroidBounds = JoinBoxPopintBounds(centroidBounds, primitiveInfo[i].center);
		int dim = GeMaximumAxis(bounds);

		// Partition primitives into two sets and build children
		int mid = (start + end) / 2;
		//if (centroidBounds.max[dim] == centroidBounds.min[dim]) {
		//	// Create leaf _BVHBuildNode_
		//	int firstPrimOffset = orderedPrims.size();
		//	for (int i = start; i < end; ++i) {
		//		int primNum = primitiveInfo[i].objId;
		//		orderedPrims.push_back(std::make_shared<SceneObject>(primitiveInfo[primNum]));
		//	}
		//	node->isleaf = true; // leaf node has two objects as children
		//	node->nPrimitives = nPrimitives;
		//	node->boxBoundries = bounds;
		//	node->firstPrimOffset = firstPrimOffset;
		//	return node;
		//}
		//else {
			// Partition primitives based on _splitMethod_
			SplitMethod splitMethod = MIDDLE;
			switch (splitMethod) {
			case MIDDLE: {
				// Partition primitives through node's midpoint
				float pmid = (bounds.min[dim] + bounds.max[dim]) / 2;
				SceneObject* midPtr = std::partition(
					&primitiveInfo[start], &primitiveInfo[end - 1] + 1,
					[dim, pmid](const SceneObject& pi) {
						return pi.center[dim] < pmid;
					});
				mid = midPtr - &primitiveInfo[0];
				// For lots of prims with large overlapping bounding boxes, this
				// may fail to partition; in that case don't break and fall
				// through
				// to EqualCounts.
				if (mid != start && mid != end) { break; }
			}
			//}
			node->leftchild = constructBVHTreed(primitiveInfo, start, mid, totalNodes, orderedPrims);
			node->rightchild = constructBVHTreed(primitiveInfo, mid, end, totalNodes, orderedPrims);

			node->longestAxis = dim;
			node->boxBoundries = JoinBounds(node->leftchild->boxBoundries, node->rightchild->boxBoundries);
			node-> nPrimitives = 0;
		}
	}
	return node;
}

int constructBVHTree(std::vector<std::shared_ptr<SceneObject>>& sceneObjects, std::shared_ptr<Node> currentNode, std::vector<std::shared_ptr<Node>>& nodes)
{   // If this is a leaf node
	if (sceneObjects.size() <= MaxLeaves) // this measn we only have one node and two children
	{
		for (int i = 0; i < (int)sceneObjects.size(); i++)
		{
			//currentNode->objs.push_back(sceneObjects[i]->objId); //we assign the ids of the root node, left and right nodes
		}
		currentNode->isleaf = true; // leaf node has two sceneObjects as children
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

	std::shared_ptr<SceneObject> sceneObject;
	Vec3f position;
	float radius;

	//This is my bottle neck
	for (int i = 0; i < sceneObjects.size(); i++)
	{
		 sceneObject = sceneObjects[i];
		 position = sceneObject->position;
		 radius = sceneObject->radius;

		// here I am splitting the objects which are their center is in the left of the 
		// Middle point of the node to the left, we can change the currentNode.midpoint,
		// By using splitting average or other ways look at my presentations
		if (position[currentNode->longestAxis] < currentNode->midpoint)
		{	
			// we create the new left bbox parameters
			if (position[0] - radius < minLeftX)
				minLeftX = position[0] - radius;
			if (position[1] - radius < minLeftY)
				minLeftY = position[1] - radius;
			if (position[2] - radius < minLeftZ)
				minLeftZ = position[2] - radius;

			if (position[0] + radius > maxLeftX)
				maxLeftX = position[0] + radius;
			if (position[1] + radius > maxLeftY)
				maxLeftY = position[1] + radius;
			if (position[2] + radius > maxLeftZ)
				maxLeftZ = position[2] + radius;

			midLeft += position; // here we are using the average for middle point 
			leftObjects.push_back(sceneObject);
		}
		else
		{
			// we create the new right bbox parameters
			if (position[0] - radius < minRightX)
				minRightX = position[0] - radius;
			if (position[1] - radius < minRightY)
				minRightY = position[1] - radius;
			if (position[2] - radius < minRightZ)
				minRightZ = position[2] - radius;

			if (position[0] + radius > maxRightX)
				maxRightX = position[0] + radius;
			if (position[1] + radius > maxRightY)
				maxRightY = position[1] + radius;
			if (position[2] + radius > maxRightZ)
				maxRightZ = position[2] + radius;

			midRight += position; // here we are using the average for middle point 
			rightObjects.push_back(sceneObject);
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
			// if it is not working check this commit
			currentNode->objsMorID[i] = sortedMortonCodes[i]; //we assign the ids of the root node, left and right nodes
		}
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
			//currentNode->objs[i] = objects[i].objId; //we assign the ids of the root node, left and right nodes
		}
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
	BoxBoundries bounds = box->boxBoundries;

	float tmin = (bounds.min.x - position.x) / direction.x;
	float tmax = (bounds.max.x - position.x) / direction.x;

	if (tmin > tmax) swap(tmin, tmax);

	float tymin = (bounds.min.y - position.y) / direction.y;
	float tymax = (bounds.max.y - position.y) / direction.y;

	if (tymin > tymax) swap(tymin, tymax);

	if ((tmin > tymax) || (tymin > tmax))
		return false;

	if (tymin > tmin)
		tmin = tymin;

	if (tymax < tmax)
		tmax = tymax;

	float tzmin = (bounds.min.z - position.z) / direction.z;
	float tzmax = (bounds.max.z - position.z) / direction.z;

	if (tzmin > tzmax) swap(tzmin, tzmax);

	if ((tmin > tzmax) || (tzmin > tmax))
		return false;

	if (tzmin > tmin)
		tmin = tzmin;

	if (tzmax < tmax)
		tmax = tzmax;

	return true;
}

void boxIntersect(Vec3f position, Vec3f direction,
	std::shared_ptr<Node>& root,
	std::vector<int>& boxes) {

	if (!boundingBoxIntersection(position, direction, root)) {
		return;
	}

	if (root->isleaf)
	{
			//retValue = currentNode;
		for (int obj = 0; obj < root->objs.size(); obj++)
			boxes.push_back(root->objs[obj]);
			return;
	}
	else
	{
		if (boundingBoxIntersection(position, direction, root->leftchild))
			boxIntersect(position, direction, root->leftchild, boxes);
		if (boundingBoxIntersection(position, direction, root->rightchild))
			boxIntersect(position, direction, root->rightchild, boxes);
	}
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