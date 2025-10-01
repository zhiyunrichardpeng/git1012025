using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;

public class FVM : MonoBehaviour
{
    float dt = 0.003f;
    float mass 			= 1;
    float stiffness_0 = 20000.0f;
    float stiffness_1 = 5000.0f;
    float damp = 0.999f;
    //float damp = 0.98f; // or even 0.95 for very stiff materials

    //float stiffness_0 = 2000.0f;  // ↓ 10x
    //float stiffness_1 = 500.0f;   // ↓ 10x
    //float dt = 0.001f;            // ↓ smaller time step

    int[] 		Tet;
	int tet_number;			//The number of tetrahedra

	Vector3[] 	Force;
	Vector3[] 	V;
	Vector3[] 	X;
	int number;				//The number of vertices

	Matrix4x4[] inv_Dm;

	//For Laplacian smoothing.
	Vector3[]   V_sum;
	int[]		V_num;

	SVD svd = new SVD();


    // Store initial positions in Start()
    Vector3[] X_initial;

    // Start is called before the first frame update
    void Start()
    {
    	// FILO IO: Read the house model from files.
    	// The model is from Jonathan Schewchuk's Stellar lib.
    	{
    		string fileContent = File.ReadAllText("Assets/house2.ele");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		
    		tet_number=int.Parse(Strings[0]);
        	Tet = new int[tet_number*4];

    		for(int tet=0; tet<tet_number; tet++)
    		{
				Tet[tet*4+0]=int.Parse(Strings[tet*5+4])-1;
				Tet[tet*4+1]=int.Parse(Strings[tet*5+5])-1;
				Tet[tet*4+2]=int.Parse(Strings[tet*5+6])-1;
				Tet[tet*4+3]=int.Parse(Strings[tet*5+7])-1;
			}
    	}
    	{
			string fileContent = File.ReadAllText("Assets/house2.node");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		number = int.Parse(Strings[0]);
    		X = new Vector3[number];
       		for(int i=0; i<number; i++)
       		{
       			X[i].x=float.Parse(Strings[i*5+5])*0.4f;
       			X[i].y=float.Parse(Strings[i*5+6])*0.4f;
       			X[i].z=float.Parse(Strings[i*5+7])*0.4f;
       		}
    		//Centralize the model.
	    	Vector3 center=Vector3.zero;
	    	for(int i=0; i<number; i++)		center+=X[i];
	    	center=center/number;
	    	for(int i=0; i<number; i++)
	    	{
	    		X[i]-=center;
	    		float temp=X[i].y;
	    		X[i].y=X[i].z;
	    		X[i].z=temp;
	    	}

            Debug.Log($"Loaded {number} vertices and {tet_number} tetrahedra.");
        }
        /*tet_number=1;
        Tet = new int[tet_number*4];
        Tet[0]=0;
        Tet[1]=1;
        Tet[2]=2;
        Tet[3]=3;

        number=4;
        X = new Vector3[number];
        V = new Vector3[number];
        Force = new Vector3[number];
        X[0]= new Vector3(0, 0, 0);
        X[1]= new Vector3(1, 0, 0);
        X[2]= new Vector3(0, 1, 0);
        X[3]= new Vector3(0, 0, 1);*/


        //Create triangle mesh.
       	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];

        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }

        int[] triangles = new int[tet_number*12];
        for(int t=0; t<tet_number*4; t++)
        {
        	triangles[t*3+0]=t*3+0;
        	triangles[t*3+1]=t*3+1;
        	triangles[t*3+2]=t*3+2;
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.triangles = triangles;
		mesh.RecalculateNormals ();


		V 	  = new Vector3[number];
        Force = new Vector3[number];
        V_sum = new Vector3[number];
        V_num = new int[number];

        // Store initial positions for reference configuration
        X_initial = new Vector3[number];
        for (int i = 0; i < number; i++)
        {
            X_initial[i] = X[i];
        }

        inv_Dm = new Matrix4x4[tet_number]; // This line was missing!

        //TODO: Need to allocate and assign 

        for (int tet = 0; tet < tet_number; tet++)
            //ret = Build_Edge_Matrix(tet);
            //inv_Dm = ret.inv;
        {
            Matrix4x4 Dm = Build_Edge_Matrix(tet, X_initial); // Pass current positions (reference config)
            inv_Dm[tet] = Dm.inverse;
        }
        // why only store the inv_Dm in one single variable?

    }

    Matrix4x4 Build_Edge_Matrix(int tet, Vector3[] positions)
    {
    	Matrix4x4 ret=Matrix4x4.zero;
        //TODO: Need to build edge matrix here.
        //for all the vertices in this tet 0 to 4 in i:
        //    for all the vertices in this tet i, 4 in j:
        //        ret[i][j] = length[i][j];
        // do I store the length of the edge there or store the index?
        // how to retrieve the value "length"?
        //for all the vertices in this tet 0 to 3 in i:
        //    ret = [tet[1] - tet[0], tet[2] - tet[0], tet[3] - tet[0]]; //[X10, X20];
                                                                       // how to get X10 X20?

        // Get the four vertices of this tetrahedron
        int i0 = Tet[tet * 4 + 0];
        int i1 = Tet[tet * 4 + 1];
        int i2 = Tet[tet * 4 + 2];
        int i3 = Tet[tet * 4 + 3];

        Vector3 v0 = positions[i0];
        Vector3 v1 = positions[i1];
        Vector3 v2 = positions[i2];
        Vector3 v3 = positions[i3];

        // Edge vectors from v0 to other vertices
        Vector3 e1 = v1 - v0; // X10
        Vector3 e2 = v2 - v0; // X20  
        Vector3 e3 = v3 - v0; // X30

        // Build the 3x3 edge matrix (we'll use 4x4 with last row/col for homogeneous coords)
        ret[0, 0] = e1.x; ret[0, 1] = e2.x; ret[0, 2] = e3.x;
        ret[1, 0] = e1.y; ret[1, 1] = e2.y; ret[1, 2] = e3.y;
        ret[2, 0] = e1.z; ret[2, 1] = e2.z; ret[2, 2] = e3.z;
        ret[3, 3] = 1; // Homogeneous coordinate

        

        return ret;
    }

    // Calculate tetrahedron volume using scalar triple product
    float Calculate_Tetrahedron_Volume(int i0, int i1, int i2, int i3, Vector3[] positions)
    {
        Vector3 v0 = positions[i0];
        Vector3 v1 = positions[i1];
        Vector3 v2 = positions[i2];
        Vector3 v3 = positions[i3];

        Vector3 e1 = v1 - v0;
        Vector3 e2 = v2 - v0;
        Vector3 e3 = v3 - v0;

        // Volume = |(e1 × e2) · e3| / 6
        float volume = Mathf.Abs(Vector3.Dot(Vector3.Cross(e1, e2), e3)) / 6.0f;
        return volume;
    }

    void Smooth_V()
    {
        for (int i = 0; i < number; i++)
        {
            V_sum[i] = new Vector3(0, 0, 0);
            V_num[i] = 0;
        }

        for (int tet = 0; tet < tet_number; tet++)
        {
            Vector3 sum = V[Tet[tet * 4 + 0]] + V[Tet[tet * 4 + 1]] + V[Tet[tet * 4 + 2]] + V[Tet[tet * 4 + 3]];
            V_sum[Tet[tet * 4 + 0]] += sum;
            V_sum[Tet[tet * 4 + 1]] += sum;
            V_sum[Tet[tet * 4 + 2]] += sum;
            V_sum[Tet[tet * 4 + 3]] += sum;
            V_num[Tet[tet * 4 + 0]] += 4;
            V_num[Tet[tet * 4 + 1]] += 4;
            V_num[Tet[tet * 4 + 2]] += 4;
            V_num[Tet[tet * 4 + 3]] += 4;
        }

        for (int i = 0; i < number; i++)
        {
            V[i] = 0.9f * V[i] + 0.1f * V_sum[i] / V_num[i];
        }
    }

    void _Update()
    {
    	// Jump up.
		if(Input.GetKeyDown(KeyCode.Space))
    	{
    		for(int i=0; i<number; i++)
    			V[i].y+=0.2f;
    	}

    	for(int i=0 ;i<number; i++)
    	{
            //TODO: Add gravity to Force.
            //V[i].y = V[i].y + (-9.8f) * dt;

            Force[i] = Vector3.zero;
            // Add gravity to force (not directly to velocity!)
            Force[i].y += mass * (-9.8f); // F = m*g
        }

    	for(int tet=0; tet<tet_number; tet++)
    	{
            //TODO: Deformation Gradient
            //ret = Build_Edge_Matrix(tet);
            //F = [tet[1] - tet[0], tet[2] - tet[0], tet[3] - tet[0]] * ret;

            // Dm: material/reference coords, Ds: spatial/current coords
            // Get vertex indices for this tetrahedron
            int i0 = Tet[tet * 4 + 0];
            int i1 = Tet[tet * 4 + 1];
            int i2 = Tet[tet * 4 + 2];
            int i3 = Tet[tet * 4 + 3];

            // Get current positions
            Vector3 v0 = X[i0];
            Vector3 v1 = X[i1];
            Vector3 v2 = X[i2];
            Vector3 v3 = X[i3];

            // Build current edge matrix
            Matrix4x4 Ds = Build_Edge_Matrix(tet, X);


            // Deformation Gradient: F = Ds * inv_Dm
            Matrix4x4 F = Ds * inv_Dm[tet];

            //TODO: Green Strain
            //G = 1/2 * (F * F.T - I);
            // Green Strain: G = ½(F^T * F - I)
            Matrix4x4 FT = F.transpose;
            Matrix4x4 FT_F = FT * F;
            //Matrix4x4 G = 0.5f * (FT_F - Matrix4x4.identity);

            // FIXED: Matrix subtraction - must do element-wise
            Matrix4x4 G = Matrix4x4.zero;
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    G[i, j] = 0.5f * (FT_F[i, j] - (i == j ? 1.0f : 0.0f));
                }
            }

            //TODO: Second PK Stress
            // how to get s1: /miu, and s0: lambda?
            //traceG = G[0][0] + G[1][1] + G[2][2];
            //S = 2 * s1 * G + s0 * traceG * I;

            // Second PK Stress for StVK model
            float traceG = G[0, 0] + G[1, 1] + G[2, 2];
            //Matrix4x4 S = 2.0f * stiffness_1 * G + stiffness_0 * traceG * Matrix4x4.identity;

            // FIXED: Create scaled identity matrix
            Matrix4x4 S = Matrix4x4.zero;
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    //if (i == j && i < 3)
                    //{
                    //    S[i, j] = 2.0f * stiffness_1 * G[i, j] + stiffness_0 * traceG;
                    //}
                    //else
                    //{
                    //    S[i, j] = 2.0f * stiffness_1 * G[i, j];
                    //}
                    S[i, j] = 2.0f * stiffness_1 * G[i, j];

                }
                // Add volumetric term on diagonal
                S[i, i] += stiffness_0 * traceG;
            }

            ////TODO: Elastic Force
            //A_ref = X01 cross X02 *X13;
            //// something like this, forgot the equation.

            //solver_method = "FVM";
            // Choose solver method
            string solver_method = "FVM"; // or "FEM"  "FVM"  "FVM_BY_SVD"

            string solver_method_SVD = "FVM_BY_SVD"; // or "FEM"  "FVM"  "FVM_ONLY" "FVM_BY_SVD"

            Vector3 f1 = Vector3.zero;
            Vector3 f2 = Vector3.zero;
            Vector3 f3 = Vector3.zero;
            Vector3 f0 = Vector3.zero;

            //if solver_method = "FEM":
            if (solver_method == "FEM") // FIXED: Use == for comparison, not =
                                        //    force_vector = -A_ref * F * S * ret;
                                        //f1 = force_vector[0];
                                        //f2 = force_vector[1];
                                        //f3 = force_vector[2];
                                        //f0 = -f1 - f2 - f3;
            {
                // FEM approach
                // Calculate reference area/volume properly
                float A_ref = Calculate_Tetrahedron_Volume(i0, i1, i2, i3, X_initial);

                // FIXED: Matrix operations must be done properly
                Matrix4x4 force_matrix_temp = F * S * inv_Dm[tet].transpose;

                Matrix4x4 force_matrix = Matrix4x4.zero;
                Matrix4x4 inv_Dm_transpose = Matrix4x4.zero;
                
                inv_Dm_transpose = inv_Dm[tet].transpose;


                for (int i = 0; i < 4; i++)
                {
                    for (int j = 0; j < 4; j++)
                    {
                        force_matrix[i, j] = -A_ref * force_matrix_temp[i,j]; 
                    }
                }

                f1 = new Vector3(force_matrix[0, 0], force_matrix[1, 0], force_matrix[2, 0]);
                f2 = new Vector3(force_matrix[0, 1], force_matrix[1, 1], force_matrix[2, 1]);
                f3 = new Vector3(force_matrix[0, 2], force_matrix[1, 2], force_matrix[2, 2]);
                f0 = -f1 - f2 - f3;
                // FIXED: Accumulate forces for FEM too!

                Vector3 f_total = f0 + f1 + f2 + f3;
                if (f_total.magnitude > 1000f) // arbitrary threshold
                {
                    Debug.Log($"Huge force detected in tet {tet}: {f_total.magnitude}");
                    // Optionally skip or scale down
                    float scale = 1000f / f_total.magnitude;
                    f0 *= scale; f1 *= scale; f2 *= scale; f3 *= scale;
                }

                Force[i0] += f0;
                Force[i1] += f1;
                Force[i2] += f2;
                Force[i3] += f3;

            }
            //if solver_method = "FVM":
            else if (solver_method == "FVM") // FIXED: Use == and proper C# syntax
            {
                //USING FVM:
                //first PK stress:
                //P = F * S
                // First PK Stress: P = F * S
                //            if (solver_method_SVD == "FVM_BY_SVD")
                //            {
                //                [U, LAMBDA, VT] = SVD(F); // L7 P35
                //lambda0 = LAMBDA[0,0];
                //    lambda1 = LAMBDA[1, 1];
                //    lambda2 = LAMBDA[2, 2];
                //                IC = lambda0^2+ lambda1^2+lambda2^2;
                //    IIC = lambda0^2*lambda1^2 + lambda0^2*lambda2^2 + lambda1^2*lambda2^2;
                //                W = stiffness0/2 * (IC - 3)^2 + s1/4*(IIC - 2*IC + 3);
                //    float beta = 1 / 4 * s1 * (((lambda1 ^ 2 * 2 * lambda0) + lambda2 ^ 2 * 2 * lambda0) + (-2 * 2 * lamda0));

                //float dWdlamda0 = stiffness_0 * (IC - 3) * 2 * lambda0 + beta;

                //float beta = 1 / 4 * s1 * (((lambda0 ^ 2 * 2 * lambda1) + lambda2 ^ 2 * 2 * lambda1) + (-2 * 2 * lamda1));
                //float dWdlamda1 = stiffness_0 * (IC - 3) * 2 * lambda1 + beta;

                //float beta = 1 / 4 * s1 * (((lambda0 ^ 2 * 2 * lambda2) + lambda1 ^ 2 * 2 * lambda2) + (-2 * 2 * lamda2));
                //float dWdlamda2 = stiffness_0 * (IC - 3) * 2 * lambda2 + beta;



                //diag[0,0] = dWdlamda0;
                //    diag[1, 1] = dWdlamda1;
                //    diag[2, 2] = dWdlamda2;
                //                P = U * diag * VT


                //            }
                Matrix4x4 P = Matrix4x4.zero;
                if (solver_method_SVD == "FVM_BY_SVD")
                {

                    // Debug: Log F matrix before SVD
                    Debug.Log($"Tet {tet}: F matrix before SVD:\n" +
                             $"[{F[0, 0]:F6}, {F[0, 1]:F6}, {F[0, 2]:F6}]\n" +
                             $"[{F[1, 0]:F6}, {F[1, 1]:F6}, {F[1, 2]:F6}]\n" +
                             $"[{F[2, 0]:F6}, {F[2, 1]:F6}, {F[2, 2]:F6}]");

                    // Check for extreme deformation
                    float detF = F.determinant;
                    if (Mathf.Abs(detF) < 1e-6f || Mathf.Abs(detF) > 1000f)
                    {
                        Debug.LogWarning($"Tet {tet}: Extreme deformation det(F) = {detF:F6}, skipping SVD");
                        continue; // Skip this tetrahedron
                    }

                    // Perform SVD: F = U * Λ * V^T  (Lecture 7 Page 35)
                    //Matrix4x4[] svdResult = SVD(F); // Returns [U, LAMBDA, VT]
                    //Matrix4x4 U = svdResult[0];
                    //Matrix4x4 LAMBDA = svdResult[1];
                    //Matrix4x4 VT = svdResult[2];
                    Matrix4x4 U = Matrix4x4.zero;
                    
                    Matrix4x4 V_temp = Matrix4x4.zero;
                    Matrix4x4 LAMBDA = Matrix4x4.zero; ;
                    svd.svd(F, ref U, ref LAMBDA, ref V_temp);

                    // Extract principal stretches (diagonal of Λ)
                    //float lambda0 = LAMBDA[0, 0];
                    //float lambda1 = LAMBDA[1, 1];
                    //float lambda2 = LAMBDA[2, 2];

                    // Extract principal stretches with safety checks
                    float lambda0 = Mathf.Clamp(LAMBDA[0, 0], 0.1f, 10f); // Clamp extreme values
                    float lambda1 = Mathf.Clamp(LAMBDA[1, 1], 0.1f, 10f);
                    float lambda2 = Mathf.Clamp(LAMBDA[2, 2], 0.1f, 10f);

                    // Calculate invariants
                    float IC = lambda0 * lambda0 + lambda1 * lambda1 + lambda2 * lambda2;  // I_C = tr(C) = tr(F^T F)
                    float IIC = lambda0 * lambda0 * lambda1 * lambda1 +
                                lambda0 * lambda0 * lambda2 * lambda2 +
                                lambda1 * lambda1 * lambda2 * lambda2; // II_C

                    // Strain energy density (Mooney-Rivlin type)
                    //float W = stiffness_0 / 2.0f * Mathf.Pow(IC - 3.0f, 2) +
                    //          stiffness_1 / 4.0f * (IIC - 2.0f * IC + 3.0f);

                    // Strain energy density with safety
                    float strain_energy_vol = stiffness_0 / 2.0f * Mathf.Pow(IC - 3.0f, 2);
                    float strain_energy_dev = stiffness_1 / 4.0f * (IIC - 2.0f * IC + 3.0f);
                    float W = strain_energy_vol + strain_energy_dev;

                    // Debug strain energy
                    Debug.Log($"Tet {tet}: W_vol = {strain_energy_vol:F6}, W_dev = {strain_energy_dev:F6}, W_total = {W:F6}");

                    // Calculate derivatives dW/dλ (principal stresses)
                    //float beta0 = 1.0f / 4.0f * stiffness_1 *
                    //              ((lambda1 * lambda1 * 2.0f * lambda0) +
                    //               (lambda2 * lambda2 * 2.0f * lambda0) +
                    //               (-2.0f * 2.0f * lambda0));

                    //float dWdlambda0 = stiffness_0 * (IC - 3.0f) * 2.0f * lambda0 + beta0;

                    //float beta1 = 1.0f / 4.0f * stiffness_1 *
                    //              ((lambda0 * lambda0 * 2.0f * lambda1) +
                    //               (lambda2 * lambda2 * 2.0f * lambda1) +
                    //               (-2.0f * 2.0f * lambda1));

                    //float dWdlambda1 = stiffness_0 * (IC - 3.0f) * 2.0f * lambda1 + beta1;

                    //float beta2 = 1.0f / 4.0f * stiffness_1 *
                    //              ((lambda0 * lambda0 * 2.0f * lambda2) +
                    //               (lambda1 * lambda1 * 2.0f * lambda2) +
                    //               (-2.0f * 2.0f * lambda2));

                    //float dWdlambda2 = stiffness_0 * (IC - 3.0f) * 2.0f * lambda2 + beta2;

                    // Calculate derivatives dW/dλ with improved formulas
                    // Fixed beta calculations - these were likely causing the explosion
                    float beta0 = stiffness_1 * (lambda0 * (lambda1 * lambda1 + lambda2 * lambda2 - 2f));
                    float beta1 = stiffness_1 * (lambda1 * (lambda0 * lambda0 + lambda2 * lambda2 - 2f));
                    float beta2 = stiffness_1 * (lambda2 * (lambda0 * lambda0 + lambda1 * lambda1 - 2f));

                    float dWdlambda0 = stiffness_0 * (IC - 3.0f) * 2.0f * lambda0 + beta0;
                    float dWdlambda1 = stiffness_0 * (IC - 3.0f) * 2.0f * lambda1 + beta1;
                    float dWdlambda2 = stiffness_0 * (IC - 3.0f) * 2.0f * lambda2 + beta2;

                    // Clamp stress derivatives to prevent explosion
                    float maxStress = 10000f; // Reasonable upper bound
                    dWdlambda0 = Mathf.Clamp(dWdlambda0, -maxStress, maxStress);
                    dWdlambda1 = Mathf.Clamp(dWdlambda1, -maxStress, maxStress);
                    dWdlambda2 = Mathf.Clamp(dWdlambda2, -maxStress, maxStress);

                    Debug.Log($"Tet {tet}: dW/dλ = [{dWdlambda0:F6}, {dWdlambda1:F6}, {dWdlambda2:F6}]");


                    // Create diagonal matrix of principal stresses
                    Matrix4x4 diag = Matrix4x4.zero;
                    diag[0, 0] = dWdlambda0;
                    diag[1, 1] = dWdlambda1;
                    diag[2, 2] = dWdlambda2;
                    diag[3, 3] = 1.0f; // Homogeneous coordinate

                    // First Piola-Kirchhoff stress: P = U * diag * V^T
                    Matrix4x4 V_transpose = V_temp.transpose;
                    P = U * diag * V_transpose;

                    // Debug final P matrix
                    Debug.Log($"Tet {tet}: P matrix:\n" +
                             $"[{P[0, 0]:F6}, {P[0, 1]:F6}, {P[0, 2]:F6}]\n" +
                             $"[{P[1, 0]:F6}, {P[1, 1]:F6}, {P[1, 2]:F6}]\n" +
                             $"[{P[2, 0]:F6}, {P[2, 1]:F6}, {P[2, 2]:F6}]");

                    // First Piola-Kirchhoff stress: P = U * diag * V^T
                    //P = U * diag * V_temp.transpose;

                }
                else
                {
                    P = F * S;
                }

                // Calculate reference volume (volume in rest configuration)
                Matrix4x4 Dm_ref = Build_Edge_Matrix(tet, X_initial);
                float det_Dm = Dm_ref.determinant;

                if (Mathf.Abs(det_Dm) < 1e-8f)
                {
                    continue; // Skip degenerate tet
                }

                float volume_ref = Mathf.Abs(det_Dm) / 6.0f;

                //force_vector = -1 / (6 * det(Dm_inv) * P * Dm.T;
                // Elastic force using FVM approach
                Matrix4x4 H_temp = P * inv_Dm[tet].transpose;

                // FIXED: Matrix scaling - must do element-wise
                Matrix4x4 H = Matrix4x4.zero;
                //Matrix4x4 inv_Dm_T = inv_Dm[tet].transpose;
                for (int i = 0; i < 4; i++)
                {
                    for (int j = 0; j < 4; j++)
                    {
                        H[i, j] = - volume_ref * H_temp[i, j];
                    }
                }

            //f1 = force_vector[0];
            //    f2 = force_vector[1];
            //    f3 = force_vector[2];
            //    f0 = -f1 - f2 - f3;
            // Distribute forces to vertices
            f1 = new Vector3(H[0, 0], H[1, 0], H[2, 0]);
            f2 = new Vector3(H[0, 1], H[1, 1], H[2, 1]);
            f3 = new Vector3(H[0, 2], H[1, 2], H[2, 2]);
            f0 = -f1 - f2 - f3;

            Vector3 f_total = f0 + f1 + f2 + f3;
            if (f_total.magnitude > 1000f) // arbitrary threshold
            {
                Debug.Log($"Huge force detected in tet {tet}: {f_total.magnitude}");
                // Optionally skip or scale down
                float scale = 1000f / f_total.magnitude;
                f0 *= scale; f1 *= scale; f2 *= scale; f3 *= scale;
            }

                // Accumulate forces
                Force[i0] += f0;
                Force[i1] += f1;
                Force[i2] += f2;
                Force[i3] += f3;

            }

        }

        Smooth_V();

        for (int i=0; i<number; i++)
    	{
            //TODO: Update X and V here.
            //V[i] = V[i] + f[i]/mass[i] * dt;
            V[i] += (Force[i] / mass) * dt;
            // Apply damping
            V[i] *= damp;
            //// Laplatian smooth:
            //if i >1:
            //    V[i] = (V[i - 1] + V[i] + V[i + 1]) / 3;

            X[i] = X[i] + V[i] * dt;


            //TODO: (Particle) collision with floor.
            if (X[i].y < -3.0f)
                // floor is -3, is a hidden info. cost me 2 hours.
            {
                X[i].y = -3.0f;

                //if (V[i].y < 0)
                //{
                V[i].y = (-3.0f - X[i].y)/dt * damp;
                // Optional: add friction
                V[i].x *= 0.9f;
                V[i].z *= 0.9f;
                //}
            }

        }
        
        // Laplacian smoothing (optional - can help stability)
        //for (int i = 0; i < number; i++)
        //{
        //    V_sum[i] = Vector3.zero;
        //    V_num[i] = 0;
        //}

    }

    // Update is called once per frame
    void Update()
    {
    	for(int l=0; l<10; l++)
    		 _Update();

        // Optional: log every 10 frames to avoid huge files
        if (Time.frameCount % 10 == 0)
            LogDebugInfo();

        // Dump the vertex array for rendering.
        Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.RecalculateNormals ();
    }

    void LogDebugInfo()
    {
        string logPath = @"C:\keyLabLocal_D\games103_assignment_V2\a3_soft_body\a3_debug_print\debug_log.csv";
        Directory.CreateDirectory(Path.GetDirectoryName(logPath));

        using (StreamWriter writer = new StreamWriter(logPath, true))
        {
            // Write header once
            if (new FileInfo(logPath).Length == 0)
            {
                writer.WriteLine("frame,tet,F00,F01,F02,F10,F11,F12,F20,F21,F22,detF,volume_ref,|f0|,|f1|,|f2|,|f3|");
            }

            // Log first few tets
            for (int tet = 0; tet < Mathf.Min(5, tet_number); tet++)
            {
                int i0 = Tet[tet * 4 + 0];
                int i1 = Tet[tet * 4 + 1];
                int i2 = Tet[tet * 4 + 2];
                int i3 = Tet[tet * 4 + 3];

                Matrix4x4 Ds = Build_Edge_Matrix(tet, X);
                Matrix4x4 F = Ds * inv_Dm[tet];
                float detF = F.determinant;

                Matrix4x4 Dm_ref = Build_Edge_Matrix(tet, X_initial);
                float volume_ref = Mathf.Abs(Dm_ref.determinant) / 6.0f;

                // Recompute forces quickly for logging
                Matrix4x4 FT = F.transpose;
                Matrix4x4 C = FT * F;
                Matrix4x4 G = Matrix4x4.zero;
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        G[i, j] = 0.5f * (C[i, j] - (i == j ? 1.0f : 0.0f));

                float traceG = G[0, 0] + G[1, 1] + G[2, 2];
                Matrix4x4 S = Matrix4x4.zero;
                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 3; j++)
                        S[i, j] = 2.0f * stiffness_1 * G[i, j];
                    S[i, i] += stiffness_0 * traceG;
                }

                Matrix4x4 P = F * S;
                //Matrix4x4 H_temp =  P * inv_Dm[tet].transpose;
                //Matrix4x4 H = Matrix4x4.zero;

                //for (int i = 0; i < 4; i++)
                //{
                //    for (int j = 0; j < 4; j++)
                //    {
                //        H[i, j] = - volume_ref * H_temp[i, j];
                //    }
                //}

                //Vector3 f0 = -new Vector3(H[0, 0], H[1, 0], H[2, 0]) - new Vector3(H[0, 1], H[1, 1], H[2, 1]) - new Vector3(H[0, 2], H[1, 2], H[2, 2]);
                //Vector3 f1 = new Vector3(H[0, 0], H[1, 0], H[2, 0]);
                //Vector3 f2 = new Vector3(H[0, 1], H[1, 1], H[2, 1]);
                //Vector3 f3 = new Vector3(H[0, 2], H[1, 2], H[2, 2]);

                //USING FVM:
                //first PK stress:
                //P = F * S
                // First PK Stress: P = F * S
                //Matrix4x4 P = F * S;
                // Calculate reference volume (volume in rest configuration)
                //Matrix4x4 Dm_ref = Build_Edge_Matrix(tet, X_initial);
                //float det_Dm = Dm_ref.determinant;

                //if (Mathf.Abs(det_Dm) < 1e-8f)
                //{
                //    continue; // Skip degenerate tet
                //}

                //float volume_ref = Mathf.Abs(det_Dm) / 6.0f;

                //force_vector = -1 / (6 * det(Dm_inv) * P * Dm.T;
                // Elastic force using FVM approach
                Matrix4x4 H_temp = P * inv_Dm[tet].transpose;

                // FIXED: Matrix scaling - must do element-wise
                Matrix4x4 H = Matrix4x4.zero;
                //Matrix4x4 inv_Dm_T = inv_Dm[tet].transpose;
                for (int i = 0; i < 4; i++)
                {
                    for (int j = 0; j < 4; j++)
                    {
                        H[i, j] = -volume_ref * H_temp[i, j];
                    }
                }

                //f1 = force_vector[0];
                //    f2 = force_vector[1];
                //    f3 = force_vector[2];
                //    f0 = -f1 - f2 - f3;
                // Distribute forces to vertices
                Vector3 f1 = new Vector3(H[0, 0], H[1, 0], H[2, 0]);
                Vector3 f2 = new Vector3(H[0, 1], H[1, 1], H[2, 1]);
                Vector3 f3 = new Vector3(H[0, 2], H[1, 2], H[2, 2]);
                Vector3 f0 = -f1 - f2 - f3;

                Vector3 f_total = f0 + f1 + f2 + f3;
                if (f_total.magnitude > 1000f) // arbitrary threshold
                {
                    Debug.Log($"Huge force detected in tet {tet}: {f_total.magnitude}");
                    // Optionally skip or scale down
                    float scale = 1000f / f_total.magnitude;
                    f0 *= scale; f1 *= scale; f2 *= scale; f3 *= scale;
                }

                writer.WriteLine($"{Time.frameCount},{tet}," +
                    $"{F[0, 0]:F4},{F[0, 1]:F4},{F[0, 2]:F4}," +
                    $"{F[1, 0]:F4},{F[1, 1]:F4},{F[1, 2]:F4}," +
                    $"{F[2, 0]:F4},{F[2, 1]:F4},{F[2, 2]:F4}," +
                    $"{detF:F6},{volume_ref:F6}," +
                    $"{f0.magnitude:F4},{f1.magnitude:F4},{f2.magnitude:F4},{f3.magnitude:F4}");
            }
        }
    }
}
