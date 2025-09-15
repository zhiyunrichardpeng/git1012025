using UnityEngine;
using System.Collections;

public class PBD_model: MonoBehaviour {

	float 		t= 0.0333f;
	float		damping= 0.99f;
	int[] 		E;
	float[] 	L;
	Vector3[] 	V;
    float r = 2.7f;


    // Use this for initialization
    void Start () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;

		//Resize the mesh.
		int n=21;
		Vector3[] X  	= new Vector3[n*n];
		Vector2[] UV 	= new Vector2[n*n];
		int[] T	= new int[(n-1)*(n-1)*6];
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
			T[t*6+0]=j*n+i;
			T[t*6+1]=j*n+i+1;
			T[t*6+2]=(j+1)*n+i+1;
			T[t*6+3]=j*n+i;
			T[t*6+4]=(j+1)*n+i+1;
			T[t*6+5]=(j+1)*n+i;
			t++;
		}
		mesh.vertices	= X;
		mesh.triangles	= T;
		mesh.uv 		= UV;
		mesh.RecalculateNormals ();

		//Construct the original edge list
		int[] _E = new int[T.Length*2];
		for (int i=0; i<T.Length; i+=3) 
		{
			_E[i*2+0]=T[i+0];
			_E[i*2+1]=T[i+1];
			_E[i*2+2]=T[i+1];
			_E[i*2+3]=T[i+2];
			_E[i*2+4]=T[i+2];
			_E[i*2+5]=T[i+0];
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
			int i = E[e*2+0];
			int j = E[e*2+1];
			L[e]=(X[i]-X[j]).magnitude;
		}

		V = new Vector3[X.Length];
		for (int i=0; i<X.Length; i++)
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

	void Strain_Limiting()
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
        //Vector3[] vertices = mesh.vertices;
        Vector3[] X = mesh.vertices; // Use X for positions (more conventional)

        int vertexCount = X.Length;

        //for i in range(length(vertices)):
        //    X_new[i] = 0;
        //    n[i] = 0;

        // Initialize arrays for sum and count
        Vector3[] sum_X = new Vector3[vertexCount];
        int[] sum_n = new int[vertexCount];

        // Initialize to zero
        for (int i = 0; i < vertexCount; i++)
        {
            sum_X[i] = Vector3.zero;
            sum_n[i] = 0;
        }

        for (int e = 0; e < E.Length / 2; e++)
        {
            int i = E[2 * e];     // First vertex of edge e
                                  // the smart way to loop the edge's two vertex indices by using 2*e and 2e+1.
            int j = E[2 * e + 1]; // Second vertex of edge e
            float L_e = L[e];     // Rest length of edge e

            Vector3 delta = X[i] - X[j];
            float current_length = delta.magnitude;

            //omega_i = X[i] - 1 / 2 * (X[i] + X[j] + L_e * delta / current_length);
            //omega_j = X[i] + (-1) * (-1) * 1 / 2 * (X[i] + X[j] + L_e * delta / current_length);
            //// only change sign for the second part of the above equation, you see.
            //X_new[i] = X_new[i] + omega_i;
            //X_new[j] = X_new[i] + omega_j;

            //n[i] = n[i] + 1;
            //n[j] = n[j] + 1;

            // Avoid division by zero
            if (current_length > 1e-6f)
            {
                Vector3 direction = delta / current_length;

                // Calculate the target midpoint with correct rest length
                Vector3 midpoint = 0.5f * (X[i] + X[j]);
                Vector3 correction = L_e * direction;

                // Update sums according to equation (5)
                sum_X[i] += 0.5f * (X[i] + X[j] + correction); // +L_e * direction
                sum_X[j] += 0.5f * (X[i] + X[j] - correction); // -L_e * direction

                sum_n[i] += 1;
                sum_n[j] += 1;
            }
        }

        //    for i in range(length(vertices)):

        //            X[i] = (X_new[i] + alpha * X[i]) / (n[i] + alpha);

        //                //Apply PBD here.
        //                //...

        //    }
        //for i in range(length(vertices)):
        //    V[i] = V[i] + 1/t*( (0.2*X[i] + X_new[i])/(0.2 + n[i])  - X[i] );
        //    X[i] = ( 0.2*X[i] + X_new[i] )/(0.2 + n[i]);
        // here alpha = 2
        //                r = b - A * deltaX;
        //if abs(r) < epsilon:
        //    break

        //for i in range(length(vertices)):
        //deltaX[i] = deltaX[i] + alpha * D_inv * r;


        // Update vertices according to equation (6)
        for (int i = 0; i < vertexCount; i++)
        {
            // Skip fixed vertices (0 and 20)
            if (i == 0 || i == 20) continue;

            if (sum_n[i] > 0)
            {
                // Calculate new position
                Vector3 new_pos = (0.2f * X[i] + sum_X[i]) / (0.2f + sum_n[i]);

                // Update velocity first (using old position)
                V[i] += (1.0f / t) * (new_pos - X[i]);

                // Then update position
                X[i] = new_pos;
            }
        }


        //mesh.vertices = vertices;
        mesh.vertices = X;
    }

    //void Collision_Handling()
    //{
    //	Mesh mesh = GetComponent<MeshFilter> ().mesh;
    //	Vector3[] X = mesh.vertices;

    //	//For every vertex, detect collision and apply impulse if needed.
    //	//...
    //	mesh.vertices = X;
    //}

    void Collision_Handling()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
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

    // Update is called once per frame
    void Update () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X = mesh.vertices;

		for(int i=0; i<X.Length; i++)
		{
			if(i==0 || i==20)	continue;

            //Initial Setup
            //...
            V[i] *= damping;

            //V[i] = V[i] + [0, 9.8f, 0] * t;
            V[i] += new Vector3(0, -9.8f, 0) * t; // Note: NEGATIVE Y for gravity
            X[i] = X[i] + t * V[i];

        }
		mesh.vertices = X;

        //deltaX = [0, 0, 0];

        for (int l=0; l<32; l++)
			Strain_Limiting ();

		Collision_Handling ();

		mesh.RecalculateNormals ();

	}


}

