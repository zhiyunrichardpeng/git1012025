using UnityEngine;
using System.Collections;

public class Rigid_Bunny : MonoBehaviour
{
    bool launched = false;
    float dt = 0.015f;
    Vector3 v = new Vector3(0, 0, 0);   // velocity
    Vector3 w = new Vector3(0, 0, 0);   // angular velocity

    float mass;                                 // mass
    Matrix4x4 I_ref;                            // reference inertia
    //Matrix4x4 I;                            // inertia

    float linear_decay = 0.999f;                // for velocity decay
    float angular_decay = 0.98f;
    float restitution = 0.5f;                   // for collision



    Vector3 gravity = new Vector3(0, -9.8f, 0); // gravity vector
    //Vector3 x; // position
    //Quaternion q; //quaternion

    //Vector3 tau = new Vector3(0, 0, 0);                // 

    // Use this for initialization
    void Start()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;

        float m = 1;
        mass = 0;
        for (int i = 0; i < vertices.Length; i++)
        {
            mass += m;
            float diag = m * vertices[i].sqrMagnitude;
            I_ref[0, 0] += diag;
            I_ref[1, 1] += diag;
            I_ref[2, 2] += diag;
            I_ref[0, 0] -= m * vertices[i][0] * vertices[i][0];
            I_ref[0, 1] -= m * vertices[i][0] * vertices[i][1];
            I_ref[0, 2] -= m * vertices[i][0] * vertices[i][2];
            I_ref[1, 0] -= m * vertices[i][1] * vertices[i][0];
            I_ref[1, 1] -= m * vertices[i][1] * vertices[i][1];
            I_ref[1, 2] -= m * vertices[i][1] * vertices[i][2];
            I_ref[2, 0] -= m * vertices[i][2] * vertices[i][0];
            I_ref[2, 1] -= m * vertices[i][2] * vertices[i][1];
            I_ref[2, 2] -= m * vertices[i][2] * vertices[i][2];
        }
        I_ref[3, 3] = 1;

        // Initialize position and rotation
        //x = transform.position;
        //q = transform.rotation;
    }

    Matrix4x4 Get_Cross_Matrix(Vector3 a)
    {
        //Get the cross product matrix of vector a
        Matrix4x4 A = Matrix4x4.zero;
        A[0, 0] = 0;
        A[0, 1] = -a[2];
        A[0, 2] = a[1];
        A[1, 0] = a[2];
        A[1, 1] = 0;
        A[1, 2] = -a[0];
        A[2, 0] = -a[1];
        A[2, 1] = a[0];
        A[2, 2] = 0;
        A[3, 3] = 1;
        return A;
    }

    // my add-up, textbook version.
    Quaternion QuaternionMultiply(Quaternion a, Quaternion b)
    {
        float scalar = a.w * b.w - Vector3.Dot(new Vector3(a.x, a.y, a.z),
                                              new Vector3(b.x, b.y, b.z));

        Vector3 vector = a.w * new Vector3(b.x, b.y, b.z) +
                        b.w * new Vector3(a.x, a.y, a.z) +
                        Vector3.Cross(new Vector3(a.x, a.y, a.z),
                                     new Vector3(b.x, b.y, b.z));

        return new Quaternion(vector.x, vector.y, vector.z, scalar);
    }


    // Manual matrix subtraction function
    Matrix4x4 MatrixSubtract(Matrix4x4 A, Matrix4x4 B)
    {
        Matrix4x4 result = Matrix4x4.zero;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                result[i, j] = A[i, j] - B[i, j];
            }
        }
        return result;
    }

    // Manual matrix multiplication function (for clarity)
    Matrix4x4 MatrixMultiply(Matrix4x4 A, Matrix4x4 B)
    {
        return A * B; // Unity does have multiplication operator for Matrix4x4
    }

    // In this function, update v and w by the impulse due to the collision with
    //a plane <P, N>
    //void Collision_Impulse(Vector3 P, Vector3 N)
    //{
    //    // here, assume the codebase has taken care to feed here each time only one vertex of the bunny mesh.
    //    new vector phi = (x - P) * N;
    //    if phi < 0:
    //        pass;

    //    if phi >= 0:
    //        {
    //        x_new = x - phi * N;

    //        x_sum = x_sum + x_new;

    //        vi = v + w cross product(R* r);
    //        if vi * N < 0:
    //        {
    //            pass;
    //        }

    //        if vi * N >= 0:
    //        {
    //            new Vector3 vn = (vi * N) * N;
    //            new Vector3 vt = vi - vn;

    //            new Vector3 vn_new = miu_N * vn;

    //            new a = max(1 - miu_t * (1 + miu_N) * || vn ||/|| vt || , 0);
    //            new Vector3 vt_new = a * vt;

    //            new v_new = vn_new + vt_new;

    //            new v_sum = v_sum + v_new;
    //        }
    //    }

    //    v_old = vi;
    //    x_old = x;
    //    x = mean(x_new);
    //    v = mean(v_new);

    //    // compute the impulse j

    //    // is r given? or r is the euclidean distance of x to origin?
    //    r = euclidean distance of x to origin || x ||.
    //    K = 1 / M * matrix_1 - (R * r) * I_inv * (R * r);
    //    j = K_inv * (v_new - v_old);

    //    v = v + 1 / M * j;
    //    w = w + I_ive * (R * r cross product j);



    //}


    // In this function, update v and w by the impulse due to the collision with
    // a plane <P, N>
    void Collision_Impulse(Vector3 P, Vector3 N)
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        // hiden info. 
        Vector3[] vertices = mesh.vertices;
        // hiden info. 

        Vector3 sum_x_new = Vector3.zero;
        Vector3 sum_v_new = Vector3.zero;
        int collision_count = 0;

        Matrix4x4 R = Matrix4x4.TRS(Vector3.zero, transform.rotation, Vector3.one);
        //TRS is to convert the quatanion to rotation matrix.
        /*
        //        If your bunny is rotated 45 degrees around Y - axis:
        //transform.rotation = 45° Y - rotation

        //R becomes a matrix that rotates any vector by 45° around Y-axis
         */


        Vector3 center_of_mass = transform.position;

        // 1.c. Collision detection - iterate through all vertices
        for (int i = 0; i < vertices.Length; i++)
        {
            // Transform vertex to world space
            Vector3 vertex_world = center_of_mass + R.MultiplyPoint3x4(vertices[i]);

            // Calculate signed distance to plane
            float phi = Vector3.Dot(vertex_world - P, N);
            // line 170 to 199 all prepare for this line 198. to get the x (vertex_world).

            // If vertex is colliding with the plane (phi < 0 means penetration)
            if (phi < 0)
            {
                collision_count++;
                // later added by DS.

                // Project vertex back to plane surface
                Vector3 x_new = vertex_world - phi * N;
                sum_x_new += x_new;

                // Calculate velocity at this vertex
                Vector3 r = vertex_world - center_of_mass;
                // the way to get the ri, use vertex world minus center of mass.

                Vector3 vi = v + Vector3.Cross(w, r);

                // Decompose velocity into normal and tangential components
                float vn_magnitude = Vector3.Dot(vi, N);
                Vector3 vn = vn_magnitude * N;
                Vector3 vt = vi - vn;

                // Only process if moving toward the plane
                if (vn_magnitude < 0)
                    // previously wrote it the wrong way, opposite way. corrected by DS.
                {
                    // Apply restitution to normal component
                    Vector3 vn_new = -restitution * vn;
                    // miu_N is the restitution, given in line 7. about collision and get back. the value is 0.5 as default. so the speed will lose half, and change direction.


                    // Apply friction to tangential component (simplified)
                    // the complex version is "new a = max(1 - miu_t * (1 + miu_N) * || vn ||/|| vt || , 0);". then new Vector3 vt_new = a * vt;
                    // here my a is the (1.0f - 0.1f) = 0.9 here.
                    Vector3 vt_new = vt * (1.0f - 0.1f); // Simple friction

                    Vector3 v_new = vn_new + vt_new;
                    sum_v_new += v_new;
                }
                else
                {
                    sum_v_new += vi;
                    // missed at the begining. so if the velosity is towards opposite direction of the plane, it should also add to the v sum, because later we need to averate them.
                    // ok, got it. this line is: there is collision, but the vertex is going outwards. "vertex_world" is inside the plane in slide 12, but the vertex is going outward of the plane regarding direction.
                }
            }
        }

        // If no collisions, return early
        if (collision_count == 0)
            return;

        // 1.c. Compute average collision position and velocity
        Vector3 avg_x_new = sum_x_new / collision_count;
        Vector3 avg_v_new = sum_v_new / collision_count;

        // 1.d. Collision response - calculate impulse
        Vector3 r_avg = avg_x_new - center_of_mass;
        // is the r of all the collision vertices. make an average.
        Vector3 v_old = v + Vector3.Cross(w, r_avg);
        // use old v, and the old w, but the new r_avg of the collided vertices.

        // Compute impulse j
        Matrix4x4 I_ref_inv = I_ref.inverse;
        Matrix4x4 R_I_ref_inv_RT = R * I_ref_inv * R.transpose;

        //Matrix4x4 K = Matrix4x4.identity * (1.0f / mass) -
        //             Get_Cross_Matrix(r_avg) * R_I_ref_inv_RT * Get_Cross_Matrix(r_avg);

        // Create (1/mass) * identity matrix properly
        Matrix4x4 massInverseMatrix = Matrix4x4.identity;
        for (int i = 0; i < 4; i++)
        {
            massInverseMatrix[i, i] = 1.0f / mass;
        }

        // Get cross product matrices
        Matrix4x4 r_cross_matrix = Get_Cross_Matrix(r_avg);
        Matrix4x4 r_cross_matrix_T = Get_Cross_Matrix(-r_avg); // Cross matrix is anti-symmetric
        // these are the cross multiplication magic. 

        //Matrix4x4 K = massInverseMatrix - r_cross_matrix * R_I_ref_inv_RT * r_cross_matrix_T;

        // Compute: term = r_cross_matrix * R_I_ref_inv_RT * r_cross_matrix
        Matrix4x4 term1 = MatrixMultiply(r_cross_matrix, R_I_ref_inv_RT);
        Matrix4x4 term2 = MatrixMultiply(term1, r_cross_matrix);

        // K = massInverseMatrix - term2
        Matrix4x4 K = MatrixSubtract(massInverseMatrix, term2);

        Matrix4x4 K_inv = K.inverse;

        // Convert vectors to 4D for matrix multiplication
        Vector4 delta_v_4d = new Vector4(avg_v_new.x - v_old.x, avg_v_new.y - v_old.y, avg_v_new.z - v_old.z, 0);
        Vector4 j_4d = K_inv * delta_v_4d;
        Vector3 j = new Vector3(j_4d.x, j_4d.y, j_4d.z);

        // Update velocities
        v += j / mass;
        w = w + R_I_ref_inv_RT.MultiplyVector(Vector3.Cross(r_avg, j));
        // this equation from L4 slide 28, w's update. and is further from slide 5, where the dt = 1.
        // as for why there is no sigma tao from Rri cross fi, because we used average r from ri in collided vertices.

        /*
          I = R * I_ref * Rᵀ (inertia tensor in world space)

        So I⁻¹ = (R * I_ref * Rᵀ)⁻¹ = R * I_ref⁻¹ * Rᵀ
         */

        /* 
         Q: in "w = w + R_I_ref_inv_RT.MultiplyVector(Vector3.Cross(r_avg, j));", I thought it is I_inverse * tau, where tau is sigma R*r_i cross f_i, why there is no R in the code?
         A: Vector3 r_avg = collision_point_world - com_world;  // ALREADY IN WORLD SPACE! so there 
         */
    }

    // Update is called once per frame
    void Update()
{
//Game Control
if (Input.GetKey("r"))
{
    transform.position = new Vector3(0, 0.6f, 0);
    restitution = 0.5f;
    launched = false;
}
if (Input.GetKey("l"))
{
    v = new Vector3(5, 2, 0);
    launched = true;
}

// Part I: Update velocities
if (launched == false) return;

if (launched)
{
    //g = 9.8;
    // gravity force
    Vector3 f = mass * gravity;
    Vector3 a = f / mass;  // mass ^ (-1) * f;
    v = v + dt * a;


    // assume this function is already iterating each vertices, and now we are only dealing with one single vertex.

    // since there is only gravity, and on torque, so I commented out line 138 to 148.

    //tau = (R * r) cross product f;
    //tau = new Vector3(0, 0, 0); // Example torque   or  (0.1f, 0, 0)

    //I = R * I_ref * R.transpose;

    // update w
    //w = w + dt * I.inverse * tau;

    //Vector3 alpha = I.inverse.MultiplyVector(tau);

    //w = w + dt * alpha;


    // Apply decay
    v = linear_decay * v;
    w = angular_decay * w;


    //else:
    //    //Disable the linear motion and the position update, if
    //    //launched is false.
    //    pass;
    //    // already donein line 88.


    // Part II: Collision Impulse

    // these are two plans: the wall and floor.
    Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
    Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

    // Part III: Update position & orientation
    //Update linear status


    Vector3 x = transform.position;
    //x = x + dt * v + (dt ^ 2) / 2 * a;
    x = x + dt * v;
    //Update angular status
    Quaternion q = transform.rotation;

    //Vector4 temporary_vector = [0, dt / 2 * w];
    //q = q + temporary_vector cross product q;

    // Update orientation using quaternion derivative
    // dq/dt = 0.5 * ω * q (where ω is the quaternion [w.x, w.y, w.z, 0])
    Quaternion omegaQuat = new Quaternion(dt / 2 * w.x, dt / 2 * w.y, dt / 2 * w.z, 0);
    Quaternion delta_q = QuaternionMultiply(omegaQuat, q);

    q = new Quaternion(
        q.x + delta_q.x,
        q.y + delta_q.y,
        q.z + delta_q.z,
        q.w + delta_q.w
    );
    q.Normalize();

    // Part IV: Assign to the object
    transform.position = x;
    transform.rotation = q;
}
}
}
