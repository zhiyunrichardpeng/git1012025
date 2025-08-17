#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

// Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
// {
//     // TODO Traverse the BVH to find intersection

//     Intersection isect;


//     // def inside the triagle():
//     //     three product multiplication. got d1, d2, d3.
//     //     if all d1, d2, d3 >=0 or all d1, d2, d3 <=0:
//     //         return True
//     //     else:
//     //         return False

//     // def obj intersect with the ray():
//     //     t = (p-o) multiply n / (d multiply n)
//     //     if t >=0 and t < inf:
//     //         // intersected with the plane
//     //         if o+td is inside the triagle:
//     //             return o+td


//     // if IntersectP(const Ray& ray, const Vector3f& invDir,
//     //     const std::array<int, 3>& dirIsNeg) == False:
//     //     return

//     // else:

//     // Check if ray intersects with node's bounding box
//     if (!node->bounds.IntersectP(ray, ray.direction_inv, {ray.direction.x < 0, 
//         ray.direction.y < 0, 
//         ray.direction.z < 0}))
//     {
//     return isect; // No intersection
//     }
    
//         // if node is leaf node:
//         //     for obj in all objs:
//         //         if obj intersect with the ray:
//         //             if intersection_closest < the intersect point:
//         //                 intersection_closest = the intersect point
//         //     return intersection_closest

//     // If leaf node, check all objects
//     if (node->left == nullptr && node->right == nullptr) {
//         return node->object->getIntersection(ray);
//     }

//     // hit1 = Intersect(ray, node.child1);
//     // hit2 = Intersect(ray, node.child2);
//     // Recursively check children
//     Intersection hit1 = getIntersection(node->left, ray);
//     Intersection hit2 = getIntersection(node->right, ray);    

//     return hit1.distance < hit2.distance ? hit1 : hit2;    

// }

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    Intersection isect;
    
    // std::cout << "Checking node: " << node << std::endl;
    
    // Check if ray intersects with node's bounding box
    if (!node->bounds.IntersectP(ray, ray.direction_inv, {ray.direction.x < 0, 
        ray.direction.y < 0, 
        ray.direction.z < 0}))
    {
        // std::cout << "No intersection with bounding box" << std::endl;
        return isect; // No intersection
    }
    
    // If leaf node, check all objects
    if (node->left == nullptr && node->right == nullptr) {
        // std::cout << "Leaf node, checking object" << std::endl;
        auto obj_intersect = node->object->getIntersection(ray);
        // std::cout << "Object intersection found? " << obj_intersect.happened << std::endl;
        return obj_intersect;
    }

    // according to the code below, this code base define the leaf should only contain one object. so my "objects " reference failed.
    // if (objects.size() == 1) {
    //     // Create leaf _BVHBuildNode_
    //     node->bounds = objects[0]->getBounds();
    //     node->object = objects[0];
    //     node->left = nullptr;
    //     node->right = nullptr;
    //     return node;
    // }
    // else if (objects.size() == 2) {
    //     node->left = recursiveBuild(std::vector{objects[0]});
    //     node->right = recursiveBuild(std::vector{objects[1]});

    // if (node->left == nullptr && node->right == nullptr) {
    //     Intersection closest;
    //     for (auto& object : node->objects) {  // Assume node stores a list
    //         auto hit = object->getIntersection(ray);
    //         if (hit.distance < closest.distance) {
    //             closest = hit;
    //         }
    //     }
    //     return closest;
    // }

    // Recursively check children
    // std::cout << "Checking left child" << std::endl;
    Intersection hit1 = getIntersection(node->left, ray);
    // std::cout << "Checking right child" << std::endl;
    Intersection hit2 = getIntersection(node->right, ray);    

    // Return closest intersection
    return hit1.distance < hit2.distance ? hit1 : hit2;    
}