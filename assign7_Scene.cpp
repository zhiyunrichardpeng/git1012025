//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
// Vector3f Scene::castRay(const Ray &ray, int depth) const
// {
//     // TO DO Implement Path Tracing Algorithm here

//     // Intersection with scene
//     Intersection inter = intersect(ray);

//     // If ray hits nothing, return background color
//     if (!inter.happened) {
//         return Vector3f(0.0f);
//     }

//     // If ray hits light source, return emission directly
//     if (inter.m->hasEmission()) {
//         return inter.m->getEmission();
//     }

//     // if (depth > scene.maxDepth) {  // scene.maxDepth is unknown. don't know how.

//     //     return Vector3f(0.0,0.0,0.0);
//     // }    

//     // Direct illumination
//     Vector3f L_dir(0.0f);
//     {
//         Intersection light_inter;
//         float pdf_light;
//         sampleLight(light_inter, pdf_light); // This fills light_inter and pdf_light
        
//         Vector3f x = light_inter.coords;
//         Vector3f ws = (x - inter.coords).normalized();
//         Vector3f NN = light_inter.normal;
        
//         // Shoot shadow ray
//         Vector3f origin = inter.coords;
//         Vector3f dir = (x - inter.coords).normalized();
//         Ray shadow_ray(origin + inter.normal * EPSILON, dir);
//         Intersection shadow_inter = intersect(shadow_ray);
        
//         // Check if the shadow ray reaches the light
//         if ((shadow_inter.coords - x).norm() < EPSILON) {
//             Vector3f emit = light_inter.emit;
//             float distance_squared = (x - inter.coords).norm() * (x - inter.coords).norm();
//             L_dir = emit * inter.m->eval(ray.direction, dir, inter.normal) 
//                   * dotProduct(inter.normal, dir) 
//                   * dotProduct(NN, -dir) 
//                   / distance_squared 
//                   / pdf_light;
//         }
//     }

//     // Indirect illumination
//     Vector3f L_indir(0.0f);
//     {
//         // Russian Roulette
//         if (get_random_float() < RussianRoulette) {
//             Vector3f wi = inter.m->sample(ray.direction, inter.normal).normalized();
//             Ray indirect_ray(inter.coords + inter.normal * EPSILON, wi);
//             Intersection indirect_inter = intersect(indirect_ray);
            
//             if (indirect_inter.happened && !indirect_inter.m->hasEmission()) {
//                 float pdf = inter.m->pdf(ray.direction, wi, inter.normal);
//                 if (pdf > EPSILON) {
//                     L_indir = castRay(indirect_ray, depth + 1) 
//                             * inter.m->eval(ray.direction, wi, inter.normal) 
//                             * dotProduct(inter.normal, wi) 
//                             / pdf 
//                             / RussianRoulette;
//                 }
//             }
//         }
//     }


//     // my implement:

//     // float pdf_light = 1/A;  // A should be given somewhere, hiden info.

//     // def shade(p, w0):
//     //     inter = sampleLight(inter, pdf_light);

//     //     x, ws, NN, emit = inter;
//     //     // how to shoot a ray from p to x?
//     //     p + t*Ray.dir  // where is p?, t? 

//     //     // for object in objects:
//     //     if all intersect(p + t*Ray.dir) == False:
//     //         L_dir = emit * eval(w0, ws, N) * dot(ws, N) * dot(ws, NN) / (abs(x - p)**2)/ pdf_light;

//     //     L_indir = 0.0;
//     //     Russian_Roulette = 0.8; // nobody gives me this, so I assume one.
//     //     random_num = rand(0,1);
//     //     if random_num <= Russian_Roulette:
//     //         // we shoot a ray here.
//     //         // nobody tells me the r() function. I assume it is given.
//     //         inter = r(p,wi);
//     //         if inter.hit ~= any emitting objects:
//     //             L_indir = shade(q,wi)*eval(w0, wi, N) * dot(wi, N)/pdf(wo, wi, N)/ Russian_Roulette;



//         // wi = sample(w0, N);


//     return L_dir + L_indir;

//     // Vector3f hitColor = scene.backgroundColor; // it is unknown. don't know how. scene.backgroundColor

//     // return hitColor;

// }



Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // Debug: Print ray info
    // std::cout << "Depth: " << depth << " | Ray origin: " << ray.origin << " | direction: " << ray.direction << std::endl;

    // Intersection with scene
    Intersection inter = intersect(ray);

    // If ray hits nothing, return background color
    if (!inter.happened) {
        // std::cout << "Missed scene\n";
        return Vector3f(0.0f);
    }

    // Debug: Print intersection info
    // std::cout << "Hit at: " << inter.coords << " | Normal: " << inter.normal 
    //           << " | Emission: " << (inter.m->hasEmission() ? "YES" : "NO") << std::endl;

    // If ray hits light source, return emission directly
    if (inter.m->hasEmission()) {
        Vector3f emission = inter.m->getEmission();
        // std::cout << "Hit light! Emission: " << emission << std::endl;
        return emission;
    }

    // Termination condition
    if (depth > 5) {  // Reasonable recursion depth
        return Vector3f(0.0f);
    }

    // Direct illumination
    Vector3f L_dir(0.0f);
    {
        Intersection light_inter;
        float pdf_light = 0.0f;
        sampleLight(light_inter, pdf_light);
        
        // Debug light sampling
        // std::cout << "Sampled light at: " << light_inter.coords 
        //           << " | pdf: " << pdf_light 
        //           << " | emit: " << light_inter.emit << std::endl;

        // if (pdf_light > 0.0f) {
        //     Vector3f x = light_inter.coords;
        //     Vector3f ws = (x - inter.coords).normalized();
        //     Vector3f NN = light_inter.normal;
            
        //     // Shoot shadow ray with small offset
        //     Vector3f origin = inter.coords + inter.normal * 0.0001f;
        //     Ray shadow_ray(origin, ws);
        //     Intersection shadow_inter = intersect(shadow_ray);
            
        if (pdf_light > 0.0f) {
            Vector3f x = light_inter.coords;
            Vector3f ws = (x - inter.coords).normalized();
            Vector3f NN = light_inter.normal;
            
            // Debug light sampling
            // std::cout << "\n--- Direct Light Calculation ---\n";
            // std::cout << "Light position: " << x << "\n";
            // std::cout << "Surface position: " << inter.coords << "\n";
            // std::cout << "Light normal: " << NN << "\n";
            // std::cout << "Surface normal: " << inter.normal << "\n";
            
            // Shoot shadow ray with small offset
            Vector3f origin = inter.coords + inter.normal * 0.0001f;
            Ray shadow_ray(origin, ws);
            Intersection shadow_inter = intersect(shadow_ray);
            
            // Detailed shadow ray debug
            // std::cout << "\nShadow Ray Debug:\n";
            // std::cout << "Ray origin: " << shadow_ray.origin << "\n";
            // std::cout << "Ray direction: " << shadow_ray.direction << "\n";
            // std::cout << "Intersection happened: " << shadow_inter.happened << "\n";
            
            // if (shadow_inter.happened) {
                // std::cout << "Hit object at: " << shadow_inter.coords << "\n";
                // std::cout << "Distance to light: " << (shadow_inter.coords - x).norm() << "\n";
                // std::cout << "Hit object emission: " << shadow_inter.m->hasEmission() << "\n";
                
                // if (shadow_inter.m->hasEmission()) {
                //     std::cout << "Emission value: " << shadow_inter.m->getEmission() << "\n";
                // }

                // std::cout << "--- Intersection Details ---\n";
                // std::cout << "Position: " << shadow_inter.coords << "\n";
                // std::cout << "Expected light pos: " << x << "\n";
                // std::cout << "Distance: " << (shadow_inter.coords - x).norm() << "\n";

                // if (!shadow_inter.m) {
                //     std::cout << "CRITICAL ERROR: No material assigned!\n";
                // } else {
                //     std::cout << "Material emission: " << shadow_inter.m->m_emission << "\n";
                //     std::cout << "hasEmission(): " << shadow_inter.m->hasEmission() << "\n";
                    
                //     // Check if this is actually the light we sampled
                //     std::cout << "Sampled light emit: " << light_inter.emit << "\n";
                //     std::cout << "Intersected emit: " << shadow_inter.m->getEmission() << "\n";
                // }                

            // }
            
            // Check why the condition fails
            // if (!shadow_inter.happened) {
            //     std::cout << "ERROR: Shadow ray didn't hit anything\n";
            // } else if (!shadow_inter.m->hasEmission()) {
            //     std::cout << "ERROR: Shadow ray hit non-emissive object\n";
            // } else if ((shadow_inter.coords - x).norm() >= 0.001f) {
            //     std::cout << "ERROR: Shadow ray hit wrong position on light (distance too large)\n";
            //     std::cout << "Actual distance: " << (shadow_inter.coords - x).norm() << "\n";
            // }

            // // Debug shadow ray
            // std::cout << "Shadow ray to light: " << (shadow_inter.happened ? "HIT" : "MISS")
            //           << " | Dist: " << (shadow_inter.coords - x).norm() << std::endl;

            // Check if the shadow ray reaches the light (with tolerance)
            if (shadow_inter.happened && shadow_inter.m->hasEmission() && 
                (shadow_inter.coords - x).norm() < 0.1f) {  //0.001f
                
                // float distance_squared = (x - inter.coords).squaredNorm();
                // Calculate squared distance manually
                Vector3f diff = x - inter.coords;
                float distance_squared = diff.x * diff.x + diff.y * diff.y + diff.z * diff.z;
                                
                float cos_theta = std::max(0.0f, dotProduct(inter.normal, ws));
                float cos_theta_x = std::max(0.0f, dotProduct(NN, -ws));
                
                Vector3f brdf = inter.m->eval(-ray.direction, ws, inter.normal);
                
                L_dir = light_inter.emit * brdf * cos_theta * cos_theta_x 
                      / distance_squared / pdf_light;
                
                // Debug direct light
                // std::cout << "Direct light contrib: " << L_dir << std::endl;
            }
        }
    }

    // Indirect illumination
    Vector3f L_indir(0.0f);
    if (get_random_float() < RussianRoulette) {
        Vector3f wi = inter.m->sample(-ray.direction, inter.normal).normalized();
        Ray indirect_ray(inter.coords + inter.normal * 0.0001f, wi);
        
        Intersection indirect_inter = intersect(indirect_ray);
        
        if (indirect_inter.happened && !indirect_inter.m->hasEmission()) {
            float pdf = inter.m->pdf(-ray.direction, wi, inter.normal);
            if (pdf > 0.0f) {
                float cos_theta = std::max(0.0f, dotProduct(inter.normal, wi));
                Vector3f brdf = inter.m->eval(-ray.direction, wi, inter.normal);
                
                L_indir = castRay(indirect_ray, depth + 1) * brdf * cos_theta
                        / pdf / RussianRoulette;
                
                // Debug indirect light
                // std::cout << "Indirect light contrib: " << L_indir << std::endl;
            }
        }
    }

    Vector3f result = L_dir + L_indir;
    // std::cout << "Final color: " << result << std::endl;
    return result;
}