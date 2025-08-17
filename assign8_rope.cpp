#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

        // Create masses at evenly spaced positions between start and end
        for (int i = 0; i < num_nodes; i++) {
            // Linear interpolation between start and end
            float t = static_cast<float>(i) / (num_nodes - 1); 
            // static_cast<float>(i): Converts the integer i to a float to ensure floating-point division (avoiding integer truncation).
            Vector2D position = (1 - t) * start + t * end;
            // Example: If num_nodes = 5:

            // Node 0: t = 0.0 → start
            
            // Node 1: t = 0.25 → 0.75*start + 0.25*end
            
            // Node 2: t = 0.5 → 0.5*start + 0.5*end
            
            // Node 3: t = 0.75 → 0.25*start + 0.75*end
            
            // Node 4: t = 1.0 → end            
            // Create new mass with given position, mass, and pinned status
            masses.push_back(new Mass(position, node_mass, false));
        }

//        Comment-in this part when you implement the constructor
// Set pinned nodes
       for (auto &i : pinned_nodes) {
           masses[i]->pinned = true;
       }
       // use interpolation, weighted.
    //    node0 = start
    //    node1 = ( (1/(num_nodes-1)) * start + ((num_nodes-1 - 1)/(num_nodes-1)) * end )/2
    //    ...
    //    node_"num_node-1" = end;

    //    masses[0] -> node0;
    //    ...

    //    // build a spring between two nodes.
    // //    F_node0 = 0.5*k^2*delta_x;
    // for each node pair:
    //     node_index_front = start;
    //     node_index_end = end;
    //     node_coefficient = k;

        // Create springs between consecutive masses
        for (int i = 0; i < num_nodes - 1; i++) {
            springs.push_back(new Spring(masses[i], masses[i+1], k));
        }


    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            //for start end of the spring:, (a)
            // f_bfroma = k * (b-a)/(||b-a||) * (||b-a|| - l) ; // b point's received force, towards a point.
            // f_a = the opposite wise way.

            // Calculate current spring vector and length
            // assume 1 is a, 2 is b, here ab is from a point toward b.
            Vector2D ab = s->m2->position - s->m1->position;
            float curr_length = ab.norm();
            Vector2D direction = ab / curr_length; // Normalized direction            
            
            // Hooke's law: F = -k * (current_length - rest_length) * direction
            Vector2D f_spring = s->k * (curr_length - s->rest_length) * direction;

            // Apply equal and opposite forces to both masses
            s->m1->forces += f_spring;
            s->m2->forces -= f_spring;            
       
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                // F = f_bfroma + gravity;
                // a = F/m;
                // v_tplus1 = vt + a * delta_t;
                // x_tplus1 = xt + v_t * delta_t; // for explicit method
                // // x_tplus1 = vt + v_tplus1 * delta_t; // for semi-implicit
                
                // Add gravity force
                m->forces += m->mass * gravity;  // assume gravity is g=9.8 only, is a accumulation, need to time the mass to become gravity force.

                // Compute acceleration (a = F/m)
                // Vector2D acceleration = m->forces / m->mass;                

                // Explicit Euler
                // begin
                // Vector2D acceleration = m->forces / m->mass;
                // m->position += m->velocity * delta_t;      // (A) Update position FIRST using CURRENT velocity
                // m->velocity += acceleration * delta_t;     // (B) Then update velocity
                // end

                // begin
                // Semi-implicit Euler
                Vector2D acceleration = m->forces / m->mass;
                m->velocity += acceleration * delta_t;     // (A) Update velocity FIRST
                m->position += m->velocity * delta_t;      // (B) Update position using NEW velocity (v(t+Δt))

                // end


                // m->velocity += acceleration * delta_t;
                // // m->position += m->velocity * delta_t;

                // // Semi-implicit Euler integration
                // Vector2D x_mid = m->position + delta_t/2*m->velocity;
                // Vector2D v_mid = (x_mid - m->position)/(delta_t/2);
                // m->position = m->position + delta_t * v_mid

                // TODO (Part 2): Add global damping
                // f_b_damp = - k * (b-1) / (||b-a||) * (delta_b - delta_a) * (b-a)/(||b-a||)
                // a = f_b_damp/m;
                // v_tplus1 = v_tplus1 + a * delta_t;
                // x_tplus1 = x_tplus1 + v_t * delta_t; // for explicit method
                float damping_factor = 0.001f; // Adjust as needed
                m->velocity *= (1 - damping_factor);                
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        const float damping_factor = 0.00005f; // As specified in the assignment        
        
        // Calculate all forces first
        for (auto &m : masses) {
            m->forces = m->mass * gravity; // Start with gravity
        }
                
        // for (auto &s : springs)
        // {
        //     // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
        //     m->position = m->position + (m->position - m->position(t-1)) + acceleration*delta_t*delta_t
        // }


        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            // m->position = m->position + (m->position - m->position(t-1)) + acceleration*delta_t*delta_t

            Vector2D ab = s->m2->position - s->m1->position;
            float curr_length = ab.norm();
            // note this is math-smart simplified eq. the full equation is:   correction = 1 * ab / curr_length * (curr_length - rest_length)
            Vector2D correction = ab * (1 - s->rest_length / curr_length);
            
            // to split the line 148:
            // correction = (ab / curr_length) * (curr_length - rest_length)
            // = ab * (1 - rest_length/curr_length)            

            // Apply half correction to each mass
            if (!s->m1->pinned)
                s->m1->position += 0.5f * correction;
            if (!s->m2->pinned)
                s->m2->position -= 0.5f * correction;
                                        
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                // move mass m.
                // Vector2D ab = s->m2->position - s->m1->position;
                // float curr_length = ab.norm();
                // Vector2D direction = ab / curr_length; // Normalized direction            
                                            
                // // modification vector = propotional * (curr_length - s->rest_length)
                // m1->position = m1->position + 0.5 * modification vector;
                // m2->position = m2->position + 0.5 * modification vector;
                
                // Vector2D velocity = ( m->position - m->last_position ) / delta_t ; 
                Vector2D acceleration = m->forces / m->mass;
                // Verlet integration with damping
                m->position += (1 - damping_factor) * (m->position - m->last_position) + acceleration * delta_t * delta_t;
                m->last_position = temp_position;                                
                // TODO (Part 4): Add global Verlet damping

                // float damping_factor = 0.001f; // Adjust as needed
                // m->position = m->position + (1-damping_factor) * (m->position - m->position(t-1)) + acceleration*delta_t*delta_t
            }
        }


    }
}
