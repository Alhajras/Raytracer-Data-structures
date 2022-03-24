# Raytracer-Data-structures

This commit works for all data but not for big scenes: 
2b0f013647b32f5b4ec168ff56159a657383c837

Probkems:

- [ ] BVH throws stack over flow, BVH bunny works only if Maxleaves =15 and objects = 30000. 
- [ ] The minleaves should become 1.
- [ ] Test for 1m+.
- [ ] I have removed the vector where I was saving all objects into the node.
- [ ] I have removed the int refrence and I am using a pointer to the children.
- [ ] BUG found if two objects have the same clone (Position) it seems that I do not return anything.
- [ ] I am saving the index of the premitive under the leaf.
- [ ] The bug was I was having one vectore called scene and I wsa sorting it and playing with it but in renderin I am using it as source of the truth for IDS, now I am using two vectores, scene and sceneFixed.
