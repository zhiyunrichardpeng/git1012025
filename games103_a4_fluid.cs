using UnityEngine;
using System.Collections;

public class wave_motion : MonoBehaviour
{
    int size = 100;
    float rate = 0.005f;
    float gamma = 0.004f;
    float damping = 0.996f;
    float[,] old_h;
    float[,] low_h;
    float[,] vh;
    float[,] b;

    bool[,] cg_mask;
    float[,] cg_p;
    float[,] cg_r;
    float[,] cg_Ap;
    bool tag = true;

    Vector3 cube_v = Vector3.zero;
    Vector3 cube_w = Vector3.zero;

    // Reference to the blocks in the scene
    public GameObject[] blocks;

    // Use this for initialization
    void Start()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        mesh.Clear();

        Vector3[] X = new Vector3[size * size];

        for (int i = 0; i < size; i++)
            for (int j = 0; j < size; j++)
            {
                X[i * size + j].x = i * 0.1f - size * 0.05f;
                X[i * size + j].y = 0;
                X[i * size + j].z = j * 0.1f - size * 0.05f;
            }

        int[] T = new int[(size - 1) * (size - 1) * 6];
        int index = 0;
        for (int i = 0; i < size - 1; i++)
            for (int j = 0; j < size - 1; j++)
            {
                T[index * 6 + 0] = (i + 0) * size + (j + 0);
                T[index * 6 + 1] = (i + 0) * size + (j + 1);
                T[index * 6 + 2] = (i + 1) * size + (j + 1);
                T[index * 6 + 3] = (i + 0) * size + (j + 0);
                T[index * 6 + 4] = (i + 1) * size + (j + 1);
                T[index * 6 + 5] = (i + 1) * size + (j + 0);
                index++;
            }
        mesh.vertices = X;
        mesh.triangles = T;
        mesh.RecalculateNormals();

        low_h = new float[size, size];
        old_h = new float[size, size];
        vh = new float[size, size];
        b = new float[size, size];

        cg_mask = new bool[size, size];
        cg_p = new float[size, size];
        cg_r = new float[size, size];
        cg_Ap = new float[size, size];

        for (int i = 0; i < size; i++)
            for (int j = 0; j < size; j++)
            {
                low_h[i, j] = 99999;
                old_h[i, j] = 0;
                vh[i, j] = 0;
            }
    }

    void A_Times(bool[,] mask, float[,] x, float[,] Ax, int li, int ui, int lj, int uj)
    {
        for (int i = li; i <= ui; i++)
            for (int j = lj; j <= uj; j++)
                if (i >= 0 && j >= 0 && i < size && j < size && mask[i, j])
                {
                    Ax[i, j] = 0;
                    if (i != 0) Ax[i, j] -= x[i - 1, j] - x[i, j];
                    if (i != size - 1) Ax[i, j] -= x[i + 1, j] - x[i, j];
                    if (j != 0) Ax[i, j] -= x[i, j - 1] - x[i, j];
                    if (j != size - 1) Ax[i, j] -= x[i, j + 1] - x[i, j];
                }
    }

    float Dot(bool[,] mask, float[,] x, float[,] y, int li, int ui, int lj, int uj)
    {
        float ret = 0;
        for (int i = li; i <= ui; i++)
            for (int j = lj; j <= uj; j++)
                if (i >= 0 && j >= 0 && i < size && j < size && mask[i, j])
                {
                    ret += x[i, j] * y[i, j];
                }
        return ret;
    }

    void Conjugate_Gradient(bool[,] mask, float[,] b, float[,] x, int li, int ui, int lj, int uj)
    {
        //Solve the Laplacian problem by CG.
        A_Times(mask, x, cg_r, li, ui, lj, uj);

        for (int i = li; i <= ui; i++)
            for (int j = lj; j <= uj; j++)
                if (i >= 0 && j >= 0 && i < size && j < size && mask[i, j])
                {
                    cg_p[i, j] = cg_r[i, j] = b[i, j] - cg_r[i, j];
                }

        float rk_norm = Dot(mask, cg_r, cg_r, li, ui, lj, uj);

        for (int k = 0; k < 128; k++)
        {
            if (rk_norm < 1e-10f) break;
            A_Times(mask, cg_p, cg_Ap, li, ui, lj, uj);
            float alpha = rk_norm / Dot(mask, cg_p, cg_Ap, li, ui, lj, uj);

            for (int i = li; i <= ui; i++)
                for (int j = lj; j <= uj; j++)
                    if (i >= 0 && j >= 0 && i < size && j < size && mask[i, j])
                    {
                        x[i, j] += alpha * cg_p[i, j];
                        cg_r[i, j] -= alpha * cg_Ap[i, j];
                    }

            float _rk_norm = Dot(mask, cg_r, cg_r, li, ui, lj, uj);
            float beta = _rk_norm / rk_norm;
            rk_norm = _rk_norm;

            for (int i = li; i <= ui; i++)
                for (int j = lj; j <= uj; j++)
                    if (i >= 0 && j >= 0 && i < size && j < size && mask[i, j])
                    {
                        cg_p[i, j] = cg_r[i, j] + beta * cg_p[i, j];
                    }
        }

    }

    void Shallow_Wave(float[,] old_h, float[,] h, float[,] new_h)
    {
        //Step 1:
        //TODO: Compute new_h based on the shallow wave model.

        //for i in range(0, length):
        //    for j in range(0, width):
        //        new_h = h[i, j] + (h[i, j] − old_h) ∗ damping
        //            if i - 1 >= 0:
        //    new_h = new_h + rate * (h[i−1, j] - h[i][j]);
        //if i + 1 < length:
        //    new_h = new_h + rate * (h[i+1, j] - h[i][j]);
        //if j - 1 >= 0:
        //    new_h = new_h + rate * (h[i, j-1] - h[i][j]);
        //if j + 1 < width:
        //    new_h = new_h + rate * (h[i, j+1] - h[i][j]);

        // Step 1: Basic wave propagation
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                //// Basic wave equation with Neumann boundary conditions
                //float laplacian = 0f;

                //// Neumann boundary: use central difference with boundary handling
                //if (i > 0) laplacian += h[i - 1, j] - h[i, j];
                //if (i < size - 1) laplacian += h[i + 1, j] - h[i, j];
                //if (j > 0) laplacian += h[i, j - 1] - h[i, j];
                //if (j < size - 1) laplacian += h[i, j + 1] - h[i, j];

                //new_h[i, j] = h[i, j] + (h[i, j] - old_h[i, j]) * damping + laplacian * rate;

                new_h[i, j] = h[i, j] + (h[i, j] - old_h[i, j]) * damping;
                // ie: my: new_h = h[i, j] + (h[i, j] − old_h) ∗ damping

                if (i != 0) new_h[i, j] += (h[i - 1, j] - h[i, j]) * rate;
                //            if i - 1 >= 0:
                //    new_h = new_h + rate * (h[i−1, j] - h[i][j]);

                if (i != size - 1) new_h[i, j] += (h[i + 1, j] - h[i, j]) * rate;
                if (j != 0) new_h[i, j] += (h[i, j - 1] - h[i, j]) * rate;
                if (j != size - 1) new_h[i, j] += (h[i, j + 1] - h[i, j]) * rate;

                old_h[i, j] = h[i, j];
                // before h is assigned to new_h, store h in the old_h. here is what I missed.
            }
        }


        //Step 2: Block->Water coupling
        //TODO: for block 1, calculate low_h.
        //for i in range(0, length):
        //            for j in range(0, width):
        //                if the block1 is in contact with the cell[i][j]:
        //low_h =h[i][j] - e[i][j];          
        //low_h , get is using bounding box and reycasting function. ?? why, we are doing water simulation. not rendering.

        //foreach (GameObject block in blocks)
        {
            //// Reset arrays for this block
            //for (int i = 0; i < size; i++)
            //    for (int j = 0; j < size; j++)
            //    {
            //        low_h[i, j] = 99999; // Reset to large value
            //        cg_mask[i, j] = false;
            //        b[i, j] = 0;
            //        vh[i, j] = 0;
            //    }

            // Calculate contact area and desired heights
            //Vector3 blockPos = block.transform.position;
            //Vector3 blockScale = block.transform.localScale;
            GameObject block = GameObject.Find("Cube");
            Vector3 blockPos = block.transform.position;
            Mesh blockMesh = block.GetComponent<MeshFilter>().mesh;

            // Calculate grid bounds for this block (similar to reference)
            int li = (int)((blockPos.x + 5.0f) * 10) - 3;
            int ui = (int)((blockPos.x + 5.0f) * 10) + 3;
            int lj = (int)((blockPos.z + 5.0f) * 10) - 3;
            int uj = (int)((blockPos.z + 5.0f) * 10) + 3;

            Bounds bounds = blockMesh.bounds;

            // Convert world coordinates to grid indices
            // The grid goes from -5 to +5 in world space (size=100, spacing=0.1)
            //float gridMinX = -size * 0.05f; // -5.0f
            //float gridMinZ = -size * 0.05f; // -5.0f

            //// Simple bounding box check (you can enhance this with raycasting)
            //int min_i = Mathf.Max(0, Mathf.FloorToInt((blockPos.x - blockScale.x / 2 + size * 0.05f) / 0.1f));
            //int max_i = Mathf.Min(size - 1, Mathf.CeilToInt((blockPos.x + blockScale.x / 2 + size * 0.05f) / 0.1f));
            //int min_j = Mathf.Max(0, Mathf.FloorToInt((blockPos.z - blockScale.z / 2 + size * 0.05f) / 0.1f));
            //int max_j = Mathf.Min(size - 1, Mathf.CeilToInt((blockPos.z + blockScale.z / 2 + size * 0.05f) / 0.1f));

            // Reset low_h for the affected region
            for (int i = li - 3; i <= ui + 3; i++)
                for (int j = lj - 3; j <= uj + 3; j++)
                    if (i >= 0 && j >= 0 && i < size && j < size)
                    {
                        low_h[i, j] = 99999; // Reset
                    }

            // Use raycasting to find exact contact points (like reference)
            for (int i = li - 3; i <= ui + 3; i++)
                for (int j = lj - 3; j <= uj + 3; j++)
                    if (i >= 0 && j >= 0 && i < size && j < size)
                    {
                        // Create ray from below to above the water surface
                        Vector3 worldPoint = new Vector3(i * 0.1f - size * 0.05f, -11, j * 0.1f - size * 0.05f);
                        Vector3 worldPointAbove = new Vector3(i * 0.1f - size * 0.05f, -10, j * 0.1f - size * 0.05f);

                        // Convert to block's local space
                        Vector3 localPoint = block.transform.InverseTransformPoint(worldPoint);
                        Vector3 localPointAbove = block.transform.InverseTransformPoint(worldPointAbove);

                        // Cast ray to find intersection with block
                        Ray ray = new Ray(localPoint, localPointAbove - localPoint);
                        float dist = 99999;
                        bounds.IntersectRay(ray, out dist);

                        // Calculate the actual height where block intersects this grid cell
                        low_h[i, j] = -11 + dist; // This gives the block's bottom at this point
                    }
            // Calculate block bounds in grid coordinates
            //int min_i = Mathf.Max(0, Mathf.FloorToInt((blockPos.x - blockScale.x * 0.5f - gridMinX) / 0.1f));
            //int max_i = Mathf.Min(size - 1, Mathf.CeilToInt((blockPos.x + blockScale.x * 0.5f - gridMinX) / 0.1f));
            //int min_j = Mathf.Max(0, Mathf.FloorToInt((blockPos.z - blockScale.z * 0.5f - gridMinZ) / 0.1f));
            //int max_j = Mathf.Min(size - 1, Mathf.CeilToInt((blockPos.z + blockScale.z * 0.5f - gridMinZ) / 0.1f));

            ////0.1f - Grid Cell Size. each grid cell is 0.1 units wide in simulation space. 
            //// 0.05f - Half Domain Scale Factor
            ////0.05f scale: Makes the math convenient when size = 20 gives exactly[-1, 1] domain

            //bool hasContact = false;

            //// Set mask and desired heights only for cells where block is above water
            //for (int i = min_i; i <= max_i; i++)
            //{
            //    for (int j = min_j; j <= max_j; j++)
            //    {
            //        // Check if block bottom is below or at water surface level
            //        float blockBottom = blockPos.y - blockScale.y * 0.5f;
            //        float waterSurface = h[i, j]; // Current water height at this cell

            //        // Only create coupling if block is interacting with water
            //        if (blockBottom <= waterSurface + 0.1f) // Small tolerance
            //        {
            //            // The desired water height is the block's bottom level
            //            // But we want to push water down to at least the block's level
            //            low_h[i, j] = Mathf.Min(low_h[i, j], blockBottom);
            //            cg_mask[i, j] = true;
            //            hasContact = true;

            //            // DEBUG: Show which cells are affected
            //            // Debug.Log($"Block affecting cell ({i},{j}): water={waterSurface:F3}, blockBottom={blockBottom:F3}");
            //        }
            //    }
            //}
            // Set up conjugate gradient problem
            for (int i = 0; i < size; i++)
                for (int j = 0; j < size; j++)
                {
                    // Only apply coupling where block is below water surface
                    if (low_h[i, j] < h[i, j]) // Block is interacting with water
                    {
                        cg_mask[i, j] = true;
                        b[i, j] = (new_h[i, j] - low_h[i, j]) / rate;
                        // classroom euation: b_(i,j) = (1/α)(h_(i,j)^new - h_(i,j) + e_(i,j))
                        // low_h[i, j] ↔ combination of h_(i,j) and e_(i,j) (block height + coupling term)
                        //i.e. low_h[i, j] = -11 + dist; // This gives the block's bottom at this point
                        // some strange operation here from fluid simulation, magic math change.

                    }
                    else
                    {
                        cg_mask[i, j] = false;
                        b[i, j] = 0;
                        vh[i, j] = 0;
                    }
                }

            // Solve for virtual heights
            Conjugate_Gradient(cg_mask, b, vh, li - 1, ui + 1, lj - 1, uj + 1);
        }

        {
            //// Reset arrays for this block
            //for (int i = 0; i < size; i++)
            //    for (int j = 0; j < size; j++)
            //    {
            //        low_h[i, j] = 99999; // Reset to large value
            //        cg_mask[i, j] = false;
            //        b[i, j] = 0;
            //        vh[i, j] = 0;
            //    }

            // Calculate contact area and desired heights
            //Vector3 blockPos = block.transform.position;
            //Vector3 blockScale = block.transform.localScale;
            GameObject block = GameObject.Find("Block");
            Vector3 blockPos = block.transform.position;
            Mesh blockMesh = block.GetComponent<MeshFilter>().mesh;

            // Calculate grid bounds for this block (similar to reference)
            int li = (int)((blockPos.x + 5.0f) * 10) - 3;
            int ui = (int)((blockPos.x + 5.0f) * 10) + 3;
            int lj = (int)((blockPos.z + 5.0f) * 10) - 3;
            int uj = (int)((blockPos.z + 5.0f) * 10) + 3;

            Bounds bounds = blockMesh.bounds;

            // Convert world coordinates to grid indices
            // The grid goes from -5 to +5 in world space (size=100, spacing=0.1)
            //float gridMinX = -size * 0.05f; // -5.0f
            //float gridMinZ = -size * 0.05f; // -5.0f

            //// Simple bounding box check (you can enhance this with raycasting)
            //int min_i = Mathf.Max(0, Mathf.FloorToInt((blockPos.x - blockScale.x / 2 + size * 0.05f) / 0.1f));
            //int max_i = Mathf.Min(size - 1, Mathf.CeilToInt((blockPos.x + blockScale.x / 2 + size * 0.05f) / 0.1f));
            //int min_j = Mathf.Max(0, Mathf.FloorToInt((blockPos.z - blockScale.z / 2 + size * 0.05f) / 0.1f));
            //int max_j = Mathf.Min(size - 1, Mathf.CeilToInt((blockPos.z + blockScale.z / 2 + size * 0.05f) / 0.1f));

            // Reset low_h for the affected region
            for (int i = li - 3; i <= ui + 3; i++)
                for (int j = lj - 3; j <= uj + 3; j++)
                    if (i >= 0 && j >= 0 && i < size && j < size)
                    {
                        low_h[i, j] = 99999; // Reset
                    }

            // Use raycasting to find exact contact points (like reference)
            for (int i = li - 3; i <= ui + 3; i++)
                for (int j = lj - 3; j <= uj + 3; j++)
                    if (i >= 0 && j >= 0 && i < size && j < size)
                    {
                        // Create ray from below to above the water surface
                        Vector3 worldPoint = new Vector3(i * 0.1f - size * 0.05f, -11, j * 0.1f - size * 0.05f);
                        Vector3 worldPointAbove = new Vector3(i * 0.1f - size * 0.05f, -10, j * 0.1f - size * 0.05f);

                        // Convert to block's local space
                        Vector3 localPoint = block.transform.InverseTransformPoint(worldPoint);
                        Vector3 localPointAbove = block.transform.InverseTransformPoint(worldPointAbove);

                        // Cast ray to find intersection with block
                        Ray ray = new Ray(localPoint, localPointAbove - localPoint);
                        float dist = 99999;
                        bounds.IntersectRay(ray, out dist);

                        // Calculate the actual height where block intersects this grid cell
                        low_h[i, j] = -11 + dist; // This gives the block's bottom at this point
                    }
            // Calculate block bounds in grid coordinates
            //int min_i = Mathf.Max(0, Mathf.FloorToInt((blockPos.x - blockScale.x * 0.5f - gridMinX) / 0.1f));
            //int max_i = Mathf.Min(size - 1, Mathf.CeilToInt((blockPos.x + blockScale.x * 0.5f - gridMinX) / 0.1f));
            //int min_j = Mathf.Max(0, Mathf.FloorToInt((blockPos.z - blockScale.z * 0.5f - gridMinZ) / 0.1f));
            //int max_j = Mathf.Min(size - 1, Mathf.CeilToInt((blockPos.z + blockScale.z * 0.5f - gridMinZ) / 0.1f));

            ////0.1f - Grid Cell Size. each grid cell is 0.1 units wide in simulation space. 
            //// 0.05f - Half Domain Scale Factor
            ////0.05f scale: Makes the math convenient when size = 20 gives exactly[-1, 1] domain

            //bool hasContact = false;

            //// Set mask and desired heights only for cells where block is above water
            //for (int i = min_i; i <= max_i; i++)
            //{
            //    for (int j = min_j; j <= max_j; j++)
            //    {
            //        // Check if block bottom is below or at water surface level
            //        float blockBottom = blockPos.y - blockScale.y * 0.5f;
            //        float waterSurface = h[i, j]; // Current water height at this cell

            //        // Only create coupling if block is interacting with water
            //        if (blockBottom <= waterSurface + 0.1f) // Small tolerance
            //        {
            //            // The desired water height is the block's bottom level
            //            // But we want to push water down to at least the block's level
            //            low_h[i, j] = Mathf.Min(low_h[i, j], blockBottom);
            //            cg_mask[i, j] = true;
            //            hasContact = true;

            //            // DEBUG: Show which cells are affected
            //            // Debug.Log($"Block affecting cell ({i},{j}): water={waterSurface:F3}, blockBottom={blockBottom:F3}");
            //        }
            //    }
            //}
            // Set up conjugate gradient problem
            for (int i = 0; i < size; i++)
                for (int j = 0; j < size; j++)
                {
                    // Only apply coupling where block is below water surface
                    if (low_h[i, j] < h[i, j]) // Block is interacting with water
                    {
                        cg_mask[i, j] = true;
                        b[i, j] = (new_h[i, j] - low_h[i, j]) / rate;
                    }
                    else
                    {
                        cg_mask[i, j] = false;
                        b[i, j] = 0;
                        vh[i, j] = 0;
                    }
                }

            // Solve for virtual heights
            Conjugate_Gradient(cg_mask, b, vh, li - 1, ui + 1, lj - 1, uj + 1);
        }
        //// Only proceed if block is actually in contact with water
        //if (!hasContact)
        //{
        //    // Debug.Log("Block not in contact with water surface");
        //    continue;
        //}


        //// Set up b for conjugate gradient
        //for (int i = 0; i < size; i++)
        //{
        //    for (int j = 0; j < size; j++)
        //    {
        //        if (cg_mask[i, j])
        //        {
        //            b[i, j] = (new_h[i, j] - low_h[i, j]) / rate;
        //            // Clamp to reasonable values to prevent explosion
        //            b[i, j] = Mathf.Clamp(b[i, j], -100f, 100f);
        //        }
        //    }
        //}

        //    //TODO: then set up b and cg_mask for conjugate gradient.
        //    b[i][j] = 1/rate* (new_h - low_h);
        //        cg_mask = [1,0,0,0;-1,2,-1,0;0,-1,2,-1;0,0,0,1];
        //        // A is the masked laplacian matrix. not mention in the class.

        //        //TODO: Solve the Poisson equation to obtain vh (virtual height).
        //        vh1 = cg(cg_mask, b);

        //                //TODO: for block 2, calculate low_h.
        //        for i in range(0, length) :
        //            for j in range(0, width) :
        //                if the block1 is in contact with the cell[i][j]:
        //low_h =h[i][j] - e[i][j];                


        //        //TODO: then set up b and cg_mask for conjugate gradient.
        //        b[i][j] = 1/rate* (new_h - low_h);
        //        // cg_mask/* = [1,0,0,0;-1,2,-1,0;0,-1,2,-1;0,0,0,1];*/
        //        cg_mask[i][j] = True;
        //        else:
        //        v[i][j] = 0
        //        cg_mask[i][j] = False;
        // A is the masked laplacian matrix. not mention in the class.

        // Solve Poisson equation
        //Conjugate_Gradient(cg_mask, b, vh, 0, size - 1, 0, size - 1);


        //TODO: Solve the Poisson equation to obtain vh (virtual height).
        //vh2 = cg(cg_mask, b);

        //TODO: then set up b and cg_mask for conjugate gradient.
        //TODO: Solve the Poisson equation to obtain vh (virtual height).

        //TODO: Diminish vh.

        // Apply virtual heights to create ripples
        for (int i = 0; i < size; i++)
                for (int j = 0; j < size; j++)
                {
                    if (cg_mask[i, j])
                        vh[i, j] *= gamma;
                }

            //vh1 = vh1* gama;

            //vh2 = vh2* gama;

            //        //TODO: Update new_h by vh.


        // Update new_h with virtual height effects
        for (int i = 0; i < size; i++)
            for (int j = 0; j < size; j++)
            {
                if (i != 0) new_h[i, j] += (vh[i - 1, j] - vh[i, j]) * rate;
                if (i != size - 1) new_h[i, j] += (vh[i + 1, j] - vh[i, j]) * rate;
                if (j != 0) new_h[i, j] += (vh[i, j - 1] - vh[i, j]) * rate;
                if (j != size - 1) new_h[i, j] += (vh[i, j + 1] - vh[i, j]) * rate;
            }
        // L10 slide31 PART 3, v is the here vh. 



        //        //new_h1 = vh1;
        //        //    new_h2 = vh2;
        //                for i in range(0, length) :
        //                for j in range(0, width) :
        //                        if i - 1 >= 0:
        //                new_h = new_h + rate* (v[i−1, j] - v[i][j]);
        //            if i + 1 < length:
        //                new_h = new_h + rate* (v[i + 1, j] -v[i][j]);
        //            if j - 1 >= 0:
        //                new_h = new_h + rate* (v[i, j - 1] - v[i][j]);
        //            if j + 1 < width:
        //                new_h = new_h + rate* (v[i, j + 1] - v[i][j]);
        // here the rate = alpha in slide

        // Diminish vh and update new_h
        //for (int i = 0; i < size; i++)
        //{
        //    for (int j = 0; j < size; j++)
        //    {
        //        vh[i, j] *= gamma;

        //        // Apply vh laplacian to new_h
        //        float vh_laplacian = 0f;
        //        if (i > 0) vh_laplacian += vh[i - 1, j] - vh[i, j];
        //        if (i < size - 1) vh_laplacian += vh[i + 1, j] - vh[i, j];
        //        if (j > 0) vh_laplacian += vh[i, j - 1] - vh[i, j];
        //        if (j < size - 1) vh_laplacian += vh[i, j + 1] - vh[i, j];

        //        new_h[i, j] += vh_laplacian * rate;
        //    }
        //}

    

        //Step 3
        //TODO: old_h <- h; h <- new_h;
        //        for i in range(0, length):
        //    for j in range(0, width):
        //old_h = h[i][j];
        //        h[i][j] = new_h;
        // Step 3: Update heights for next iteration
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                //old_h[i, j] = h[i, j];
                h[i, j] = new_h[i, j];
            }
        }
        //    //Step 4: Water->Block coupling.
        //    //More TODO here.

        //Update cube
        {

            GameObject Cube = GameObject.Find("Cube");

            Vector3 cube_p = Cube.transform.position;
            Mesh cube_mesh = Cube.GetComponent<MeshFilter>().mesh;

            int li = (int)((cube_p.x + 5.0f) * 10) - 3;
            int ui = (int)((cube_p.x + 5.0f) * 10) + 3;
            int lj = (int)((cube_p.z + 5.0f) * 10) - 3;
            int uj = (int)((cube_p.z + 5.0f) * 10) + 3;
            Bounds bounds = cube_mesh.bounds;

//            5.0f - Grid Offset:
//            csharp
//            (cube_p.x + 5.0f)
//Purpose: Converts from world coordinates to grid indices

//The fluid grid likely spans from - 5 to + 5 in world units

//Adding 5 shifts the coordinate range from[-5, 5] to[0, 10]

//Example: If cube_p.x = -3, then - 3 + 5 = 2(now in 0 - 10 range)

//10 - Grid Resolution:
//            csharp
//            * 10
//Purpose: Converts world units to grid cell indices

//Since each grid cell is 0.1f wide(as seen in the point creation), there are 10 cells per world unit

//Scaling: 1.0 world unit × 10 = 10 grid cells

//Example: Position 2.0 in shifted coordinates becomes grid index 20

//3 - Search Radius:
//            csharp
//            - 3` and `+3
//Purpose: Defines how many grid cells around the cube to check for fluid interactions

//Creates a 7×7 cell region(from - 3 to + 3 = 7 cells) centered on the cube

//This is an optimization - only check cells near the cube instead of the entire grid

            float t = 0.004f;
            float mass = 10.0f;
            Vector3 force = new Vector3(0, -mass * 9.8f, 0);
            Vector3 torque = new Vector3(0, 0, 0);

            for (int i = li - 3; i <= ui + 3; i++)
                for (int j = lj - 3; j <= uj + 3; j++)
                    if (i >= 0 && j >= 0 && i < size && j < size)
                    {
                        Vector3 p = new Vector3(i * 0.1f - size * 0.05f, -11, j * 0.1f - size * 0.05f);
                        Vector3 q = new Vector3(i * 0.1f - size * 0.05f, -10, j * 0.1f - size * 0.05f);

                        // p Start point of the ray. q end point.
                        //-11: Ray starts below the fluid simulation area(y = -11)
                        //- 10: Ray ends at the fluid surface level(y = -10) . The ray direction is (0, 1, 0) - straight up.
                        //i * 0.1f - size * 0.05f these are the scaling and grid size tricks. 
                        //Debug.Log("ok");
                        //Debug.Log(p);

                        p = Cube.transform.InverseTransformPoint(p);
                        q = Cube.transform.InverseTransformPoint(q);
                        //Transforms world coordinates to the cube's local space

                        //Debug.Log(p);

                        Ray ray = new Ray(p, q - p);
                        float dist = 99999;
                        bounds.IntersectRay(ray, out dist);

//                    bounds: The cube's axis-aligned bounding box (AABB) in local space

//IntersectRay(): Calculates the distance from ray origin to where it first hits the bounding box

//out dist: Output parameter that receives the intersection distance

//If no intersection, dist remains 99999

                        /*if(i==50 && j==50)	
                        {
                            Debug.Log(p);
                            Debug.Log(q);
                            Debug.Log(-11+dist);
                        }*/
                        //Debug.Log(cube_p.y-0.5f);


                        if (vh[i, j] != 0)
                        {
                            Vector3 r = p + dist * (q - p) - cube_p;
                            Vector3 f = new Vector3(0, vh[i, j], 0) * 4.0f;
                            //     4.0f (Force Scaling Factor)    empirical scaling factor                   
                            // F_buoyancy = ρ * g * V_displaced
                            //Since vh[i, j] represents fluid height(proportional to displaced volume), the 4.0f factor scales this to appropriate force magnitude.
                            force += f;

                            torque += Vector3.Cross(r, f);
                        }
                    }

            // for stability, add damping.
            cube_v *= 0.99f;
            cube_w *= 0.99f;

            cube_v += force * t / mass;  // v = v + at. a = f/mass. f = gravity + buoyancy.
            cube_p += cube_v * t;
            Cube.transform.position = cube_p;

            cube_w += torque * t / (100.0f * mass);  // omiga update equation, same with LAB1.

            //  q update, the same with the equation in LAB1. 
            Quaternion cube_q = Cube.transform.rotation;
            Quaternion wq = new Quaternion(cube_w.x, cube_w.y, cube_w.z, 0);
            Quaternion temp_q = wq * cube_q;
            cube_q.x += 0.5f * t * temp_q.x; //dt / 2 * w
            cube_q.y += 0.5f * t * temp_q.y;
            cube_q.z += 0.5f * t * temp_q.z;
            cube_q.w += 0.5f * t * temp_q.w;
            Cube.transform.rotation = cube_q;

            ////Update angular status
            //Quaternion q = transform.rotation;

            ////Vector4 temporary_vector = [0, dt / 2 * w];
            ////q = q + temporary_vector cross product q;

            //// Update orientation using quaternion derivative
            //// dq/dt = 0.5 * ω * q (where ω is the quaternion [w.x, w.y, w.z, 0])
            //Quaternion omegaQuat = new Quaternion(dt / 2 * w.x, dt / 2 * w.y, dt / 2 * w.z, 0);
            //Quaternion delta_q = QuaternionMultiply(omegaQuat, q);

            //q = new Quaternion(
            //    q.x + delta_q.x,
            //    q.y + delta_q.y,
            //    q.z + delta_q.z,
            //    q.w + delta_q.w
            //);
            //q.Normalize();
        }
        //    //g = 9.8;
        //    // gravity force
        //    Vector3 f = mass * gravity + v * mass; // how to transfer v to force?
        //Vector3 a = f / mass;  // mass ^ (-1) * f;
        //v = v + dt* a;

        //// Apply decay
        //v = linear_decay* v;
        //w = angular_decay* w;

        //Vector3 x = transform.position;
        ////x = x + dt * v + (dt ^ 2) / 2 * a;
        //x = x + dt* v;


        //// Part IV: Assign to the object
        //transform.position = x;
        //transform.rotation = q;
        //    for i in range(0, length) :
        //        for j in range(0, width) :
        //            if the block1 is in contact with the cell[i][j]:
        //            torque =  v* mass ??;

    }


    // Update is called once per frame
    void Update()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] X = mesh.vertices;
        float[,] new_h = new float[size, size];
        float[,] h = new float[size, size];

        //TODO: Load X.y into h.
        //h = X.y;
        // 1.a: Load heights from mesh
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                h[i, j] = X[i * size + j].y;
            }
        }

        if (Input.GetKeyDown("r"))
        {
            //TODO: Add random water.
            //i = random(1, len(water plane x axis)-1);
            //j = random(1, len(water plane y axis)-1);
            //r = random(0, h*0.1);
            //h[i][j] += r;

            //h[i-1][j] -= 0.25*r;
            //h[i + 1][j] -= 0.25 * r;
            //h[i ][j+1] -= 0.25 * r;
            //h[i][j-1] -= 0.25 * r;

            int i = Random.Range(1, size - 1);
            int j = Random.Range(1, size - 1);
            float r = Random.Range(0.1f, 0.3f);

            h[i, j] += r;

            // Maintain volume by subtracting from neighbors
            h[i - 1, j] -= 0.25f * r;
            h[i + 1, j] -= 0.25f * r;
            h[i, j - 1] -= 0.25f * r;
            h[i, j + 1] -= 0.25f * r;
        }

        for (int l = 0; l < 8; l++)
        {
            Shallow_Wave(old_h, h, new_h);
        }

        //TODO: Store h back into X.y and recalculate normal.
        //X.y = h;

        // Store heights back to mesh
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                X[i * size + j].y = h[i, j];
            }
        }

        mesh.vertices = X;
        mesh.RecalculateNormals();

    }
}
