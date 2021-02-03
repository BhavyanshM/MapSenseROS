
#define FILTER_DISPARITY_THRESHOLD 0
#define MERGE_ANGULAR_THRESHOLD 1
#define MERGE_DISTANCE_THRESHOLD 2
#define PATCH_HEIGHT 3
#define PATCH_WIDTH 4
#define SUB_H 5
#define SUB_W 6

float4 back_project(int2 pos, float Z){
    float px = (pos.x - 341.84)/459.97 * Z;
    float py = (pos.y - 249.17)/459.80 * Z;
    float4 X = (float4)(px, py, Z, 0);
    return X;
}

 float3 estimate_normal(read_only image2d_t in, int x, int y, global float* params){
    float residual = 0;
    float Z = 0;
    int m = 2;
    int count = 0;
    float4 normal = (float4)(0,0,0,0);
    if(y >= 0 && y < (int)params[SUB_H] && x >= 0 && x < (int)params[SUB_W]){

        for(int i = m; i<(int)params[PATCH_HEIGHT]-m; i++){
            for(int j = m; j<(int)params[PATCH_WIDTH]-m; j++){
                count++;
                int gx = x*(int)params[PATCH_HEIGHT] + i;
                int gy = y*(int)params[PATCH_WIDTH] + j;
                int2 pos = (int2)(gx,gy);

                pos = pos + (int2)(-m,-m);
                Z = ((float)read_imageui(in, pos).x)/(float)1000;
                float4 va = back_project(pos,Z);

                pos = pos + (int2)(m,-m);
                Z = ((float)read_imageui(in, pos).x)/(float)1000;
                float4 vb = back_project(pos,Z);

                pos = pos + (int2)(m,m);
                Z = ((float)read_imageui(in, pos).x)/(float)1000;
                float4 vc = back_project(pos,Z);

                normal += cross((vc-vb),(vb-va));

            }
        }
    }
    return (1/(float)(count)) * normal.xyz;
}

 float3 estimate_centroid(read_only image2d_t in, int x, int y, global float* params){
     float Z = 0;
     int count = 0;
     float3 centroid = (float3)(0,0,0);
    if(y >= 0 && y < (int)params[SUB_H] && x >= 0 && x < (int)params[SUB_W]){
        for(int i = 0; i<(int)params[PATCH_HEIGHT]; i++){
            for(int j = 0; j<(int)params[PATCH_WIDTH]; j++){
                count++;
                int gx = x*(int)params[PATCH_HEIGHT] + i;
                int gy = y*(int)params[PATCH_WIDTH] + j;
                int2 pos = (int2)(gx,gy);
                Z = ((float)read_imageui(in, pos).x)/(float)1000;
                if(Z > 0.1f){
                    float4 P = back_project(pos,Z);
                    centroid += P.xyz;
                }

            }
        }
    }
    return (1/(float)(count)) * centroid;
 }

bool isConnected(float3 ag, float3 an, float3 bg, float3 bn, global float* params){
    float3 vec = ag - bg;
    float dist = length(vec);
    float sim = fabs(dot(normalize(an), normalize(bn)));
    float perpDist = fabs(dot(ag-bg, normalize(bn))) + fabs(dot(bg-ag, normalize(ag)));
    if (perpDist < params[MERGE_DISTANCE_THRESHOLD] && sim > params[MERGE_ANGULAR_THRESHOLD]){
        return true;
    }else {
        return false;
    }
}

int filter_depth(read_only image2d_t in, int x, int y, write_only image2d_t out0, global float* params){
    uint Z = 0;
    int count = 0;
    int xs[8], ys[8], values[8];
    int total_unique = 0;

    /* Develop depth profile for 8x8 patch. */
    for(int i = 0; i<(int)params[PATCH_HEIGHT]; i++){
        for(int j = 0; j<(int)params[PATCH_WIDTH]; j++){
            count++;
            int gx = x*(int)params[PATCH_HEIGHT] + i;
            int gy = y*(int)params[PATCH_WIDTH] + j;

            int2 pos = (int2)(gx,gy);
            Z = read_imageui(in, pos).x;
            if (Z != 0){
                if (total_unique == 0){
                    xs[total_unique] = gx;
                    ys[total_unique] = gy;
                    values[total_unique] = Z;
                    total_unique++;
                }else{
                    int unique = 0;
                    for(int k = 0; k<total_unique; k++){

                        if ((Z - values[k])*(Z - values[k]) > params[FILTER_DISPARITY_THRESHOLD]){
                            // if (x==0 && y==0)printf("Compare(%d,%d,%d)(%d,%d)\n", Z, abs(Z - values[k]), values[k], unique, total_unique);
                            unique++;

                        }
                    }
                    if (unique == total_unique && total_unique < 8){
                        // if (x==0 && y==0)printf("Insert(%d,%d)(%d,%d)\n", gx, gy, unique, total_unique);
                        xs[total_unique] = gx;
                        ys[total_unique] = gy;
                        values[total_unique] = Z;
                        total_unique++;
                    }
                }
            }
        }
    }

    /* Use the depth profile to fill in the gaps. */
    for(int i = 0; i<(int)params[PATCH_HEIGHT]; i++){
        for(int j = 0; j<(int)params[PATCH_WIDTH]; j++){
            count++;
            int gx = x*(int)params[PATCH_HEIGHT] + i;
            int gy = y*(int)params[PATCH_WIDTH] + j;

            int2 pos = (int2)(gx,gy);
            Z = read_imageui(in, pos).x;

            if (Z == 0){
                float minDist = 10000000;
                float curDist = 0;
                int minIndex = 0;
                for(int m = 0; m<total_unique; m++){
                    curDist = length((float2)(gx,gy) - (float2)(xs[m], ys[m]));
                    // if (x==0 && y==0) printf("Filling(%d,%d)-(%d,%d):%.2lf\n", gx,gy,xs[m],ys[m],curDist);
                    if (curDist < minDist){
                        minDist = curDist;
                        minIndex = m;
                    }
                }
                Z = values[minIndex];
                // if (x==0 && y==0) printf("%d\t",Z);
            }

            // if (x == 10 && y == 10) printf("Depth:%hu\n",Z);
            write_imageui(out0, pos, (uint4)(Z,0,0,0));
        }
        // if (x==0 && y==0) printf("\n");
    }

}

/* ++++++++++++++++++++++++++++++++++++++++++ OPEN_CL KERNELS BEGIN HERE ++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* OpenCL Kernels Begin Here. All utilities above this point. */

/* Filter Kernel: Filters all pixels within a patch on depth map. Removes outliers, flying points, dead pixels and measurement
 * noise. */

 void kernel filterKernel(read_only image2d_t in, read_only image2d_t in_color, write_only image2d_t out0, global float* params){
    int y = get_global_id(0);
    int x = get_global_id(1);

    if(x==0 && y==0) printf("FilterKernel\n");
    if(y >= 0 && y < (int)params[SUB_H] && x >= 0 && x < (int)params[SUB_W]){

        filter_depth(in, x, y, out0, params);

    }
}

/* Pack Kernel: Generates the patches by packing the centroid, surface normal and metadata related to
 * patches on a sub-sampled grid of the depth map. The following intrinsic parameters are used to convert to Point Cloud.
 * K: [459.97265625, 0.0, 341.83984375, 0.0, 459.8046875, 249.173828125, 0.0, 0.0, 1.0]
 * */
void kernel packKernel(  read_only image2d_t in,
	                        write_only image2d_t out0, write_only image2d_t out1, write_only image2d_t out2, /* float3 maps for normal*/
                            write_only image2d_t out3, write_only image2d_t out4, write_only image2d_t out5, /* float3 maps for centroids */
                            global float* params
                            // write_only image2d_t debug

 )
{
	int y = get_global_id(0);
    int x = get_global_id(1);

    if(x==0 && y==0) printf("PackKernel:(%d,%d)\n", (int)params[PATCH_HEIGHT], (int)params[PATCH_WIDTH]);
    if(y >= 0 && y < (int)params[SUB_H] && x >= 0 && x < (int)params[SUB_W]){
        float3 normal = estimate_normal(in, x, y, params);
        float3 centroid = estimate_centroid(in, x, y, params);

        // if(x==24 && y==50) printf("Normal:(%.4lf, %.4lf, %.4lf)\n", normal.x, normal.y, normal.z);

        write_imagef(out0, (int2)(x,y), (float4)(normal.x,0,0,0));
        write_imagef(out1, (int2)(x,y), (float4)(normal.y,0,0,0));
        write_imagef(out2, (int2)(x,y), (float4)(normal.z,0,0,0));
        write_imagef(out3, (int2)(x,y), (float4)(centroid.x,0,0,0));
        write_imagef(out4, (int2)(x,y), (float4)(centroid.y,0,0,0));
        write_imagef(out5, (int2)(x,y), (float4)(centroid.z,0,0,0));
    }
}


/* Merge Kernel: Creates the graph-based structure by adding connections between the neighboring
 * patches based on similarity.
 */
void kernel mergeKernel( write_only image2d_t out0, write_only image2d_t out1, write_only image2d_t out2, /* float3 maps for normal*/
                          write_only image2d_t out3, write_only image2d_t out4, write_only image2d_t out5, /* float3 maps for centroids */
                          write_only image2d_t out6, /* uint8 map for patch metadata*/
                          global float* params /* All parameters */
                          // write_only image2d_t debug
){
     int y = get_global_id(0);
     int x = get_global_id(1);

     if(x==0 && y==0) printf("MergeKernel:(%d,%d)\n", (int)params[SUB_H], (int)params[SUB_W]);
     if(y > 0 && y < (int)params[SUB_H]-1 && x > 0 && x < (int)params[SUB_W]-1){

        float n1_a = read_imagef(out0, (int2)(x,y)).x;
        float n2_a = read_imagef(out1, (int2)(x,y)).x;
        float n3_a = read_imagef(out2, (int2)(x,y)).x;
        float g1_a = read_imagef(out3, (int2)(x,y)).x;
        float g2_a = read_imagef(out4, (int2)(x,y)).x;
        float g3_a = read_imagef(out5, (int2)(x,y)).x;

        float3 g_a = (float3)(g1_a,g2_a,g3_a);
        float3 n_a = (float3)(n1_a,n2_a,n3_a);

        uint patch = (uint)(0);

        int count = 0;
        for(int i = -1; i<2; i++){
            for(int j = -1; j<2; j++){
                if (!(j==0 && i==0)){

                     float n1_b = read_imagef(out0, (int2)(x+i,y+j)).x;
                     float n2_b = read_imagef(out1, (int2)(x+i,y+j)).x;
                     float n3_b = read_imagef(out2, (int2)(x+i,y+j)).x;
                     float g1_b = read_imagef(out3, (int2)(x+i,y+j)).x;
                     float g2_b = read_imagef(out4, (int2)(x+i,y+j)).x;
                     float g3_b = read_imagef(out5, (int2)(x+i,y+j)).x;

                     float3 g_b = (float3)(g1_b,g2_b,g3_b);
                     float3 n_b = (float3)(n1_b,n2_b,n3_b);

                     if(isConnected(g_a, n_a, g_b, n_b, params)){
                         // printf("Connected: (%d,%d)\n",x+i, y+j);
                         patch = (1 << count) | patch;
                     }
                     count++;
                }
            }
        }
        write_imageui(out6, (int2)(x,y), (uint4)(patch, 0, 0, 0));

    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++