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
#include <vector>

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

int constructTree(std::vector<SceneObject>& objects, Node& currentNode, std::vector<Node>& nodes)
{   // If this is a leaf node
    if(objects.size() <= 3) // this measn we only have one node and two children
    {
        for(int i = 0; i<(int)objects.size();i++)
        {
            currentNode.objs[i] = objects[i].objId; //we assign the ids of the root node, left and right nodes
        }
        currentNode.numObjs = (int)objects.size(); // how many objects node has
        currentNode.isleaf = true; // leaf node has two objects as children
        nodes.push_back(currentNode);
        return (int)nodes.size()-1;
    }
    
	// If it is not a leaf node
    Node newLeftNode;
    newLeftNode.left = NULL;
    newLeftNode.right = NULL;
    Node newRightNode;
    newRightNode.left = NULL;
    newRightNode.right = NULL;
    std::vector<int> leftPointers;
    std::vector<SceneObject> leftObjects;
    Vec3f midLeft;
    float maxLeftX = std::numeric_limits<float>::min();
    float minLeftX = std::numeric_limits<float>::max();
    float maxLeftY = std::numeric_limits<float>::min();
    float minLeftY = std::numeric_limits<float>::max();
    float maxLeftZ = std::numeric_limits<float>::min();
    float minLeftZ = std::numeric_limits<float>::max();
	Vec3f midRight;
    float maxRightX = std::numeric_limits<float>::min();
    float minRightX = std::numeric_limits<float>::max();
    float maxRightY = std::numeric_limits<float>::min();
    float minRightY = std::numeric_limits<float>::max();
    float maxRightZ = std::numeric_limits<float>::min();
    float minRightZ = std::numeric_limits<float>::max();
    std::vector<int> rightPointers;
    std::vector<SceneObject> rightObjects;
    for(int i = 0; i < objects.size(); i++)
    {   
		// here I am splitting the objects which are their center is in the left of the 
		// Middle point of the node to the left, we can change the currentNode.midpoint,
		// By using splitting average or other ways look at my presentations
        if(objects[i].position[currentNode.longestAxis] < currentNode.midpoint)
        {
			// we create the new left bbox parameters
            if(objects[i].position[0]-objects[i].radius < minLeftX)
                minLeftX = objects[i].position[0]-objects[i].radius;
            if(objects[i].position[1]-objects[i].radius < minLeftY)
                minLeftY = objects[i].position[1]-objects[i].radius;
            if(objects[i].position[2]-objects[i].radius < minLeftZ)
                minLeftZ = objects[i].position[2]-objects[i].radius;
            
            if(objects[i].position[0]+objects[i].radius > maxLeftX)
                maxLeftX = objects[i].position[0]+objects[i].radius;
            if(objects[i].position[1]+objects[i].radius > maxLeftY)
                maxLeftY = objects[i].position[1]+objects[i].radius;
            if(objects[i].position[2]+objects[i].radius > maxLeftZ)
                maxLeftZ = objects[i].position[2]+objects[i].radius;
            
            midLeft += objects[i].position; // here we are using the average for middle point 
            leftObjects.push_back(objects[i]);
        }
        else
        {
			// we create the new right bbox parameters
            if(objects[i].position[0]-objects[i].radius < minRightX)
                minRightX = objects[i].position[0]-objects[i].radius;
            if(objects[i].position[1]-objects[i].radius < minRightY)
                minRightY = objects[i].position[1]-objects[i].radius;
            if(objects[i].position[2]-objects[i].radius < minRightZ)
                minRightZ = objects[i].position[2]-objects[i].radius;
            
            if(objects[i].position[0]+objects[i].radius > maxRightX)
                maxRightX = objects[i].position[0]+objects[i].radius;
            if(objects[i].position[1]+objects[i].radius > maxRightY)
                maxRightY = objects[i].position[1]+objects[i].radius;
            if(objects[i].position[2]+objects[i].radius > maxRightZ)
                maxRightZ = objects[i].position[2]+objects[i].radius;

            midRight += objects[i].position; // here we are using the average for middle point 
            rightObjects.push_back(objects[i]);
        }
    }
    
    midLeft = Vec3f(midLeft[0]/leftObjects.size(),midLeft[1]/leftObjects.size(),midLeft[2]/leftObjects.size());
    midRight = Vec3f(midRight[0]/rightObjects.size(),midRight[1]/rightObjects.size(),midRight[2]/rightObjects.size());
    
	// we try to find the midpoint and longest axis of the left box
    if(maxLeftX-minLeftX > maxLeftY - minLeftY)
    {
        if(maxLeftX-minLeftX > maxLeftZ - minLeftZ)
        {
            newLeftNode.longestAxis = 0;
            newLeftNode.midpoint = midLeft[0];
        }
    }
    if(maxLeftY-minLeftY > maxLeftX - minLeftX)
    {
        if(maxLeftY-minLeftY > maxLeftZ - minLeftZ)
        {
            newLeftNode.longestAxis = 1;
            newLeftNode.midpoint = midLeft[1];
        }
    }
    if(maxLeftZ - minLeftZ > maxLeftX - minLeftX)
    {
        if(maxLeftZ - minLeftZ > maxLeftY-minLeftY)
        {
            newLeftNode.longestAxis = 2;
            newLeftNode.midpoint = midLeft[2];
        }
    }

	// we try to find the midpoint and longest axis of the right box
    if(maxRightX-minRightX > maxRightY - minRightY)
    {
        if(maxRightX-minRightX > maxRightZ - minRightZ)
        {
            newRightNode.longestAxis = 0;
            newRightNode.midpoint = midRight[0];
        }
    }
    if(maxRightY-minRightY > maxRightX - minRightX)
    {
        if(maxRightY-minRightY > maxRightZ - minRightZ)
        {
            newRightNode.longestAxis = 1;
            newRightNode.midpoint = midRight[1];
        }
    }
    if(maxRightZ-minRightZ > maxRightX - minRightX)
    {
        if(maxRightZ-minRightZ > maxRightY - minRightY)
        {
            newRightNode.longestAxis = 2;
            newRightNode.midpoint = midRight[2];
        }
    }

    newLeftNode.minX = minLeftX;
    newLeftNode.maxX = maxLeftX;
    newLeftNode.minY = minLeftY;
    newLeftNode.maxY = maxLeftY;
    newLeftNode.minZ = minLeftZ;
    newLeftNode.maxZ = maxLeftZ;
    
    newRightNode.minX = minRightX;
    newRightNode.maxX = maxRightX;
    newRightNode.minY = minRightY;
    newRightNode.maxY = maxRightY;
    newRightNode.minZ = minRightZ;
    newRightNode.maxZ = maxRightZ;
    
    int l = constructTree(leftObjects, newLeftNode,nodes);
    int r = constructTree(rightObjects, newRightNode,nodes);
    
    currentNode.left = l;
    currentNode.right = r;
    
    nodes.push_back(currentNode);
    return (int)nodes.size()-1;
}


bool boundingBoxIntersection(Vec3f position, Vec3f direction, Node& box)
{
	float tmin = (box.minX - position[0]) / direction[0];
	float tmax = (box.maxX - position[0]) / direction[0];

	if (tmin > tmax)
		std::swap(tmin, tmax);

	float tymin = (box.minY - position[1]) / direction[1];
	float tymax = (box.maxY - position[1]) / direction[1];

	if (tymin > tymax)
		std::swap(tymin, tymax);

	if ((tmin > tymax) || (tymin > tmax))
		return false;

	if (tymin > tmin)
		tmin = tymin;

	if (tymax < tmax)
		tmax = tymax;

	float tzmin = (box.minZ - position[2]) / direction[2];
	float tzmax = (box.maxZ - position[2]) / direction[2];

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



void bvhTraverse(Vec3f position, Vec3f direction, std::vector<Node>& tree, int currentNode, std::vector<int>& boxes)
{
	int retValue = -1;
	if (tree[currentNode].isleaf)
	{
		if (boundingBoxIntersection(position, direction, tree[currentNode]))
		{
			retValue = currentNode;
			boxes.push_back(retValue);
		}
	}
	else
	{
		if (boundingBoxIntersection(position, direction, tree[tree[currentNode].left]))
			bvhTraverse(position, direction, tree, tree[currentNode].left, boxes);
		if (boundingBoxIntersection(position, direction, tree[tree[currentNode].right]))
			bvhTraverse(position, direction, tree, tree[currentNode].right, boxes);
	}
}


// Common Headers

//#include "ray.h"
//#include "vec3.h"


#endif
