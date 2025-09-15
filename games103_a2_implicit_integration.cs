using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class implicit_model : MonoBehaviour
{
	float 		t 		= 0.0333f;
    //0.0333 seconds per frame. in the loop, it update every frame, and in one frame,
    // we calculate the x using one t, so QED.

    //30 frames per second(approximately)

//    For slower motion: increase t

//For faster motion: decrease t

//For higher accuracy: decrease t and increase iteration count
    float 		mass	= 1;
	float		damping	= 0.99f;
    //float 		rho		= 0.995f;
    float rho = 0.5f;
    float 		spring_k = 8000;
	int[] 		E;
	float[] 	L;
	Vector3[] 	V;

    float omega = 1;
    float r = 2.7f;
    //float rou = 0.5f;

    // Start is called before the first frame update
    void Start()
    {
		Mesh mesh = GetComponent<MeshFilter> ().mesh;

		//Resize the mesh.
		int n=21;
		Vector3[] X  	= new Vector3[n*n];
		Vector2[] UV 	= new Vector2[n*n];
		int[] triangles	= new int[(n-1)*(n-1)*6];
		for(int j=0; j<n; j++)
		for(int i=0; i<n; i++)
		{
			X[j*n+i] =new Vector3(5-10.0f*i/(n-1), 0, 5-10.0f*j/(n-1));
			UV[j*n+i]=new Vector3(i/(n-1.0f), j/(n-1.0f));
		}
		int t=0;
		for(int j=0; j<n-1; j++)
		for(int i=0; i<n-1; i++)	
		{
			triangles[t*6+0]=j*n+i;
			triangles[t*6+1]=j*n+i+1;
			triangles[t*6+2]=(j+1)*n+i+1;
			triangles[t*6+3]=j*n+i;
			triangles[t*6+4]=(j+1)*n+i+1;
			triangles[t*6+5]=(j+1)*n+i;
			t++;
		}
		mesh.vertices=X;
		mesh.triangles=triangles;
		mesh.uv = UV;
		mesh.RecalculateNormals ();


		//Construct the original E
		int[] _E = new int[triangles.Length*2];
		for (int i=0; i<triangles.Length; i+=3) 
		{
			_E[i*2+0]=triangles[i+0];
			_E[i*2+1]=triangles[i+1];
			_E[i*2+2]=triangles[i+1];
			_E[i*2+3]=triangles[i+2];
			_E[i*2+4]=triangles[i+2];
			_E[i*2+5]=triangles[i+0];
		}
		//Reorder the original edge list
		for (int i=0; i<_E.Length; i+=2)
			if(_E[i] > _E[i + 1]) 
				Swap(ref _E[i], ref _E[i+1]);
		//Sort the original edge list using quicksort
		Quick_Sort (ref _E, 0, _E.Length/2-1);

		int e_number = 0;
		for (int i=0; i<_E.Length; i+=2)
			if (i == 0 || _E [i + 0] != _E [i - 2] || _E [i + 1] != _E [i - 1]) 
					e_number++;

		E = new int[e_number * 2];
		for (int i=0, e=0; i<_E.Length; i+=2)
			if (i == 0 || _E [i + 0] != _E [i - 2] || _E [i + 1] != _E [i - 1]) 
			{
				E[e*2+0]=_E [i + 0];
				E[e*2+1]=_E [i + 1];
				e++;
			}

		L = new float[E.Length/2];
		for (int e=0; e<E.Length/2; e++) 
		{
			int v0 = E[e*2+0];
			int v1 = E[e*2+1];
			L[e]=(X[v0]-X[v1]).magnitude;
		}

		V = new Vector3[X.Length];
		for (int i=0; i<V.Length; i++)
			V[i] = new Vector3 (0, 0, 0);
    }

    void Quick_Sort(ref int[] a, int l, int r)
	{
		int j;
		if(l<r)
		{
			j=Quick_Sort_Partition(ref a, l, r);
			Quick_Sort (ref a, l, j-1);
			Quick_Sort (ref a, j+1, r);
		}
	}

	int  Quick_Sort_Partition(ref int[] a, int l, int r)
	{
		int pivot_0, pivot_1, i, j;
		pivot_0 = a [l * 2 + 0];
		pivot_1 = a [l * 2 + 1];
		i = l;
		j = r + 1;
		while (true) 
		{
			do ++i; while( i<=r && (a[i*2]<pivot_0 || a[i*2]==pivot_0 && a[i*2+1]<=pivot_1));
			do --j; while(  a[j*2]>pivot_0 || a[j*2]==pivot_0 && a[j*2+1]> pivot_1);
			if(i>=j)	break;
			Swap(ref a[i*2], ref a[j*2]);
			Swap(ref a[i*2+1], ref a[j*2+1]);
		}
		Swap (ref a [l * 2 + 0], ref a [j * 2 + 0]);
		Swap (ref a [l * 2 + 1], ref a [j * 2 + 1]);
		return j;
	}

	void Swap(ref int a, ref int b)
	{
		int temp = a;
		a = b;
		b = temp;
	}

	void Collision_Handling()
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X = mesh.vertices;

        // Find the sphere GameObject
        GameObject sphere = GameObject.Find("Sphere");

        // Get sphere properties
        Vector3 c = sphere.transform.position;

        //Handle colllision.
        //for i in range(length(X)):
        //    //c = sphere.transform.position;
        //    phi = abs(X - c) - r;

        //    if phi <= 0:
        //        V[i] = V[i] + 1 / t * (c + r * (X[i] - c) / abs(X[i] - c) - X[i]);
        //        X[i] = c + r * (X[i] - c) / abs(X[i] - c);

        // Handle collision for each vertex
        for (int i = 0; i < X.Length; i++)
        {
            // Skip fixed vertices (0 and 20) if needed
            if (i == 0 || i == 20) continue;

            // Calculate distance from vertex to sphere center
            Vector3 vertexToCenter = X[i] - c;
            float distance = vertexToCenter.magnitude;
            float phi = distance - r; // Signed distance: negative means penetration

            // Check if vertex is inside sphere (collision)
            if (phi < 0)
            {
                // Normalize direction from center to vertex
                Vector3 normal;
                if (distance > 1e-6f) // Avoid division by zero
                {
                    normal = vertexToCenter / distance;
                }
                else
                {
                    // If vertex is exactly at center, use arbitrary direction
                    normal = Vector3.up;
                }

                // Calculate target position on sphere surface
                Vector3 targetPosition = c + r * normal;

                // Apply impulse to velocity (Eq. 4 from assignment)
                V[i] += (1.0f / t) * (targetPosition - X[i]);

                // Update vertex position to sphere surface
                X[i] = targetPosition;
            }
        }



        mesh.vertices = X;
	}

	void Get_Gradient(Vector3[] X, Vector3[] X_hat, float t, Vector3[] G)
	{
        //Momentum and Gravity.

        // "X 		= mesh.vertices;"
        // because this sentence, we now X contain all the vertices.

        // Initialize gradient to zero
        for (int i = 0; i < G.Length; i++)
        {
            G[i] = Vector3.zero;
        }

        //For i in range(length(X)):
        //    g[i] = 1 / (t ^ 2) * m[i] * (X[i] - X_hat[i]);

        // Momentum term: (1/Δt²) * M * (X - X_hat)
        for (int i = 0; i < X.Length; i++)
        {
            G[i] += (1.0f / (t * t)) * mass * (X[i] - X_hat[i]);
        }

        //Spring Force.
        // slide5p18:

        //For e in range(length(E)):
        //    for i and j in e:
        //        g[i] = g[i] + k * (1 - L_e / abs(X[i] - X[j]));
        //        g[j] = g[j] - k * (1 - L_e / abs(X[i] - X[j]));

        // Spring forces (negative gradient of spring energy)
        for (int e = 0; e < E.Length / 2; e++)
        {
            int i = E[2 * e];     // First vertex of edge e
            // the smart way to loop the edge's two vertex indices by using 2*e and 2e+1.
            int j = E[2 * e + 1]; // Second vertex of edge e
            float L_e = L[e];     // Rest length of edge e

            Vector3 delta = X[i] - X[j];
            float current_length = delta.magnitude;

            if (current_length > 1e-6f) // Avoid division by zero
            {
                Vector3 spring_force = spring_k * (1.0f - L_e / current_length) * delta;

                G[i] += spring_force;
                G[j] -= spring_force;
            }
        }

        //For i in range(length(X)):
        //        g[i] = g[i] + [0, 9.8 * m[i], 0];

        // Gravity force (negative since it's added to gradient)
        for (int i = 0; i < X.Length; i++)
        {
            //G[i] += new Vector3(0, -9.8f * mass, 0); // Negative gravity
//            Force from gravity: F_gravity = m * g = m * (0, -9.8, 0)

//Gradient of potential energy: ∇U_gravity = -F_gravity = m * (0, 9.8, 0)
            G[i] += new Vector3(0, 9.8f * mass, 0); // POSITIVE gravity
        }

        // Fix vertices 0 and 20 (as specified in assignment)
        G[0] = Vector3.zero;
        G[20] = Vector3.zero;


    }

    // Update is called once per frame
    void Update () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X 		= mesh.vertices;
		Vector3[] last_X 	= new Vector3[X.Length];
		Vector3[] X_hat 	= new Vector3[X.Length];
		Vector3[] G 		= new Vector3[X.Length];

        // Store current positions BEFORE any updates
        Vector3[] X_current = (Vector3[])X.Clone();

        // Arrays for Chebyshev acceleration
        Vector3[] last_deltaX = new Vector3[X.Length];
        Vector3[] deltaX = new Vector3[X.Length];

        // Initialize last_deltaX to zero
        for (int i = 0; i < last_deltaX.Length; i++)
        {
            last_deltaX[i] = Vector3.zero;
        }

        //Initial Setup.

        //Vector3[] last_V = V;

        //V = V * damping;
        //X_hat = X + dt * V;

        //X = X_hat;

        // 1.a. Initial Setup
        // Apply damping and calculate X_hat
        for (int i = 0; i < V.Length; i++)
        {
            V[i] *= damping;
            //X_hat[i] = X[i] + t * V[i];
            X_hat[i] = X_current[i] + t * V[i];
            //X[i] = X_hat[i]; // Initial guess
        }

        // Set initial guess X = X_hat
        for (int i = 0; i < X.Length; i++)
        {
            X[i] = X_hat[i];
        }


        //      for (int k=0; k<32; k++)
        //{
        //	Get_Gradient(X, X_hat, t, G);

        //          //Update X by gradient.
        //          For i in range(length(X)):
        //              X[i] = X[i] - (1 / (t ^ 2) * m[i] + 4 * k) ^ (-1) * g[i];

        //          For i in range(length(X)):
        //              V[i] = V[i] - (X[i] - X_hat[i]) / t;
        //      }

        // 1.c. Finishing - Solve optimization problem

        //last_deltaX = new Vector3[X.Length] * 0;

        for (int k = 0; k < 32; k++)
        {
            if (k == 0)
            {
                omega = 1.0f;
            }
            else if (k == 1)
            {
                omega = 2.0f / (2.0f - rho * rho);
            }
            else
            {
                omega = 4.0f / (4.0f - rho * rho * omega);
            }

            Get_Gradient(X, X_hat, t, G);

            // Update X using diagonal Hessian approximation
            for (int i = 0; i < X.Length; i++)
            {
                // Diagonal term: (1/Δt²)*mass + 4*spring_k
                float diagonal_term = (1.0f / (t * t)) * mass + 4.0f * spring_k;

                // Avoid division by zero and ensure stability
                if (Mathf.Abs(diagonal_term) > 1e-6f)
                {
                    //X[i] -= (1.0f / diagonal_term) * G[i];
                    // Calculate basic gradient step
                    Vector3 basic_delta = -(1.0f / diagonal_term) * G[i];

                    // Safety clamp to prevent large updates
                    if (basic_delta.magnitude > 1.0f)
                    {
                        basic_delta = basic_delta.normalized * 1.0f;
                    }

                    // Apply Chebyshev acceleration
                    deltaX[i] = omega * basic_delta + (1.0f - omega) * last_deltaX[i];



                    //deltaX = -(1.0f / diagonal_term) * G[i];
                    //deltaX = omega * deltaX + (1 - omega) * last_deltaX;
                    X[i] = X[i] + deltaX[i];
                    
                    // Store for next iteration
                    
                    last_deltaX[i] = deltaX[i];
                }
            }

            // Keep fixed vertices in place
            // forgot to do this.
            //X[0] = mesh.vertices[0];  // Vertex 0 fixed
            //X[20] = mesh.vertices[20]; // Vertex 20 fixed
            X[0] = X_current[0];  // Keep fixed vertices in place
            X[20] = X_current[20];
        }

        // Update velocities
        // should seperate from the aobve position calculation.
        for (int i = 0; i < V.Length; i++)
        {
            //V[i] += (1.0f / t) * (X[i] - X_hat[i]);
            V[i] = (X[i] - X_current[i]) / t;
        }

        //Finishing.

        mesh.vertices = X;

		Collision_Handling ();
		mesh.RecalculateNormals ();
	}
}
