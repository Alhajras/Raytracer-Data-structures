# Raytracer-Data-structures

This commit works for all data but not for big scenes: 
2b0f013647b32f5b4ec168ff56159a657383c837

Probkems:

- [x] BVH throws stack over flow, BVH bunny works only if Maxleaves =15 and objects = 30000. 
- [x] The minleaves should become 1.
- [x] Test for 1m+.
- [x] I have removed the vector where I was saving all objects into the node.
- [x] I have removed the int refrence and I am using a pointer to the children.
- [x] BUG found if two objects have the same clone (Position) it seems that I do not return anything.
- [x] I am saving the index of the premitive under the leaf.
- [x] The bug was I was having one vectore called scene and I wsa sorting it and playing with it but in renderin I am using it as source of the truth for IDS, now I am using two vectores, scene and sceneFixed.
- [x] Add multithreading to LBVH used openMp #pragma omp and also threadiung the construct if only the objects are more than 100. 
