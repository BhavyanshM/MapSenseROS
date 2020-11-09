
 void kernel depthKernel(
	read_only image2d_t in,
	write_only image2d_t out,
	const int h,
	const int w
)

{
	int2 xpos = (int2)(get_global_id(0), get_global_id(1));

	__const sampler_t sampler = CLK_NORMALIZED_COORDS_FALSE | CLK_FILTER_NEAREST | CLK_ADDRESS_CLAMP_TO_EDGE;

	uint4 pix;

	int2 pos = (int2)(xpos.x,xpos.y);
	uint4 cpx = read_imageui(in, sampler, pos);

	int m = 2;

	if (pos.x > m &&
		pos.y > m &&
		pos.x < w-m &&
		pos.y < h-m &&
		cpx.z < 255
	)
	{
		pos = pos + (int2)(-m,-m);
		uint4 pixa = read_imageui(in, sampler, pos);
		uint a = pixa.z*256 + pixa.y;
		float4 va = (float4)(a*pos.x, a*pos.y, a, 0);

		pos = pos + (int2)(m,-m);
		uint4 pixb = read_imageui(in, sampler, pos);
		uint b = pixb.z*256 + pixb.y;
		float4 vb = (float4)(b*pos.x, b*pos.y, b, 0);

		pos = pos + (int2)(m,m);
		uint4 pixc = read_imageui(in, sampler, pos);
		uint c = pixc.z*256 + pixc.y;
		float4 vc = (float4)(c*pos.x, c*pos.y, c, 0);

		float4 xp = cross((vc-vb),(vb-va));
		float dp = dot(xp, vb)/200000;
		float4 normXp = normalize(xp)*255;
		normXp *= dp;

		//pix = (uint4)(xp.x, xp.y, xp.z, 0);
		pix = (uint4)(normXp.x, normXp.y, normXp.z, 0);

	}else{
		pix = (uint4)(0,0,0,0);
	}

	write_imageui(out, xpos, pix);

}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



float poly(float8 P, float x, float y){
	return (P.s0*pow(x,3)+P.s1*pow(x,2)+P.s2*x+P.s3*pow(y,3)+P.s4*pow(y,2)+P.s5*y+P.s6*1);

}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


float residual(float8 P, read_only image2d_t in, __const sampler_t sam,	int a, int b, bool odd){

	float total = 0;
	float X = 0;
	float Y = 0;
	float Z = 0;

	float xy_min = -5;
	float xy_max = 5;
	float n = 15;
	float8 params = (float8)(0.0, -10.0, 0.0, 1.0, -1.0, 0.0, 1.0, 0.0);

	int2 pos = (int2)(a,b);

	for (int i = (int)odd; i<16; i+=2){
		for (int j = (int)!odd; j<16; j+=2){
			X = xy_min + i*(xy_max-xy_min)/n;
			Y = xy_min + j*(xy_max-xy_min)/n;
			uint4 pix = read_imageui(in, sam, pos*16 + (int2)(i,j));
			//Z = poly(params,X,Y);

			Z = (float)(pix.y*256 + pix.x);
			Z = (Z)/((float)10000);

			total += pow((poly(P,X,Y) - Z),2);
		}
	}
	return total;
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


float8 calc_grad(float8 P, float gdel, read_only image2d_t in, __const sampler_t sam,int a,int b,bool odd){
	float grad[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	for (int i = 0; i<7; i++){
		float dp[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
		dp[i] = gdel;
		float8 dps = vload8(0,dp);
		grad[i] = (residual(P+dps, in, sam, a, b,odd)-residual(P-dps, in, sam, a, b,odd))/(2*gdel);
	}

	return vload8(0,grad);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

float8 update_params(float8 P, float8 grad, float alpha){
	return P - alpha*grad;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

float3 plane_grad(read_only image2d_t in, float3 p, int x, int y){
    float3 grad = (float3)(0,0,0);
    float Z = 0;
    if(y >= 0 && y < 60 && x >= 0 && x < 80){

        for(int i = 0; i<SIZE_X; i++){
            for(int j = 0; j<SIZE_Y; j++){
                int gx = x*SIZE_X + i;
                int gy = y*SIZE_Y + j;
                Z = ((float)read_imageui(in, (int2)(gx,gy)).x)/(float)1000;

                float px = (gx - 341.84)/459.97 * Z;
                float py = (gy - 249.17)/459.80 * Z;

                float3 X = (float3)(px, py, 1);
                grad += (1/(float)(SIZE_X*SIZE_Y)) * 2*(dot(p,X) - Z) * X;
                // printf("(%.2lf, %.2lf, %.2lf)\n", grad.x, grad.y, grad.z);

            }
        }
    }
    return grad;
}

 float plane_residual(read_only image2d_t in, float3 p, int x, int y){
    float residual = 0;
    float Z = 0;
    if(y >= 0 && y < 60 && x >= 0 && x < 80){

        for(int i = 0; i<SIZE_X; i++){
            for(int j = 0; j<SIZE_Y; j++){
                int gx = x*SIZE_X + i;
                int gy = y*SIZE_Y + j;
                Z = ((float)read_imageui(in, (int2)(gx,gy)).x)/(float)1000;

                float px = (gx - 341.84)/459.97 * Z;
                float py = (gy - 249.17)/459.80 * Z;

                // if(x==0&&y==0) printf("(%.2lf,%.2lf,%.2lf)\n", px, py, Z);

                float3 X = (float3)(px, py, 1);
                residual += pow((dot(p,X) - Z), 2);
                // printf("(%.2lf, %.2lf, %.2lf)\n", grad.x, grad.y, grad.z);
            }
        }
    }
    return residual;
 }

 float4 back_project(int2 pos, float Z){
     float px = (pos.x - 341.84)/459.97 * Z;
     float py = (pos.y - 249.17)/459.80 * Z;
     float4 X = (float4)(px, py, Z, 0);
     return X;
}

// void sgd(){
//
 // float3 p = (float3)(0,0,0);
 // float lr = 0.04;
 // float residual = 0;
 // for(int i = 0; i<16; i++){
 //     p = p - lr * plane_grad(in, p, x, y);
 //     residual = plane_residual(in, p, x, y);
 //     if(x==20 && y==23) printf("Params(%.4lf, %.4lf, %.4lf): [%.4lf]\n",p.x, p.y, p.z, residual);
 // }
//     // int count = 0;
//     // float r = 100000;
//     // float alpha = 7.5;
//     // float gdel = 0.0001;
//     // float8 P = (float8)(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
//     // float8 grad = (float8)(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
//     //
//     // float8 prod = (float8)(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
//     // float8 epsilon = ((float8)(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0))*0.000001f;
//     //
//     // bool odd = 1;
//     //
//     // grad = calc_grad(P, gdel, in, sampler, pos.x, pos.y, odd);
//     // prod += grad*grad;
//     // grad /= sqrt(prod + epsilon);
//     // P = update_params(P, grad, alpha);
//     // float orig = residual(P, in, sampler, pos.x, pos.y, odd);
//     //
//     //
//     // int n_iterations = 16;
//     // while(count < n_iterations){
//     // 	r = residual(P, in, sampler, pos.x, pos.y, odd);
//     // 	count++;odd = !odd;
//     // 	grad = calc_grad(P, gdel, in, sampler, pos.x, pos.y, odd);
//     // 	prod += grad*grad;
//     // 	grad /= sqrt(prod + epsilon);
//     // 	P = update_params(P, grad, alpha);
//     //
//     // 	//printf("(%d,%d)\n",pos.x,pos.y);
//     // 	//if(pos.x == 47 && pos.y == 50){
//     // 		//printf("(%d,%d,%d,%.2f)\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n",pos.x,pos.y,count,r,P.s0,P.s1,P.s2,P.s3,P.s4,P.s5,P.s6);
//     // 		//printf("(%d,%d,%d,%.2f)\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n-----%d\n",pos.x,pos.y,count,r/orig*100,grad.s0,grad.s1,grad.s2,grad.s3,grad.s4,grad.s5,grad.s6,(int)odd);
//     //
//     // 	//}
//     // }
//
//     // write_imagef(out1, pos, (float4)(P.s0,P.s1,P.s2,P.s3));
//     // write_imagef(out2, pos, (float4)(P.s4,P.s5,P.s6,P.s7));
//
//     //write_imagef(out1, pos, (float4)(1,2,3,4));
//     //write_imagef(out2, pos, (float4)(5,6,7,8));
//
// }

 float3 estimate_normal(read_only image2d_t in, int x, int y){
    float residual = 0;
    float Z = 0;
    int m = 2;
    int count = 0;
    float4 normal = (float4)(0,0,0,0);
    if(y >= 0 && y < 60 && x >= 0 && x < 80){

        for(int i = m; i<SIZE_X-m; i++){
            for(int j = m; j<SIZE_Y-m; j++){
                count++;
                int gx = x*SIZE_X + i;
                int gy = y*SIZE_Y + j;
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

 float3 estimate_centroid(read_only image2d_t in, int x, int y){
     float Z = 0;
     int count = 0;
     float3 centroid = (float3)(0,0,0);
    if(y >= 0 && y < 60 && x >= 0 && x < 80){
        for(int i = 0; i<SIZE_X; i++){
            for(int j = 0; j<SIZE_Y; j++){
                count++;
                int gx = x*SIZE_X + i;
                int gy = y*SIZE_Y + j;
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

void kernel filterKernel(read_only image2d_t in, write_only image2d_t out0){
    int y = get_global_id(0);
    int x = get_global_id(1);

    if(y >= 0 && y < 60 && x >= 0 && x < 80){
        uint Z = 0;
        int count = 0;
        for(int i = 0; i<SIZE_X; i++){
            for(int j = 0; j<SIZE_Y; j++){
                count++;
                int gx = x*SIZE_X + i;
                int gy = y*SIZE_Y + j;
                int2 pos = (int2)(gx,gy);
                Z = read_imageui(in, pos).x;
                // if (x == 10 && y == 10) printf("Depth:%hu\n",Z);
                write_imageui(out0, pos, (uint4)(Z,0,0,0));
            }
        }

    }
}

/*K: [459.97265625, 0.0, 341.83984375, 0.0, 459.8046875, 249.173828125, 0.0, 0.0, 1.0]*/
void kernel packKernel(  read_only image2d_t in,
	                        write_only image2d_t out0, write_only image2d_t out1, write_only image2d_t out2, /* float3 maps for normal*/
                            write_only image2d_t out3, write_only image2d_t out4, write_only image2d_t out5 /* float3 maps for centroids */
                            // write_only image2d_t debug
 )
{
	int y = get_global_id(0);
    int x = get_global_id(1);

    if(y >= 0 && y < 60 && x >= 0 && x < 80){
        float3 normal = estimate_normal(in, x, y);
        float3 centroid = estimate_centroid(in, x, y);

        if(x==24 && y==50) printf("Normal:(%.4lf, %.4lf, %.4lf)\n", normal.x, normal.y, normal.z);

        write_imagef(out0, (int2)(x,y), (float4)(normal.x,0,0,0));
        write_imagef(out1, (int2)(x,y), (float4)(normal.y,0,0,0));
        write_imagef(out2, (int2)(x,y), (float4)(normal.z,0,0,0));
        write_imagef(out3, (int2)(x,y), (float4)(centroid.x,0,0,0));
        write_imagef(out4, (int2)(x,y), (float4)(centroid.y,0,0,0));
        write_imagef(out5, (int2)(x,y), (float4)(centroid.z,0,0,0));
    }
}

bool isConnected(float3 ag, float3 an, float3 bg, float3 bn){
    float3 vec = ag - bg;
    float dist = length(vec);
    if (dist < 0.3){
        return true;
    }else {
        return false;
    }
}

void kernel mergeKernel( write_only image2d_t out0, write_only image2d_t out1, write_only image2d_t out2, /* float3 maps for normal*/
                          write_only image2d_t out3, write_only image2d_t out4, write_only image2d_t out5, /* float3 maps for centroids */
                          write_only image2d_t out6 /* uint8 map for patch metadata*/
                          // write_only image2d_t debug
){
     int y = get_global_id(0);
     int x = get_global_id(1);

     if(y > 0 && y < 60-1 && x > 0 && x < 80-1){

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
                     count++;
                     float n1_b = read_imagef(out0, (int2)(x+i,y+j)).x;
                     float n2_b = read_imagef(out1, (int2)(x+i,y+j)).x;
                     float n3_b = read_imagef(out2, (int2)(x+i,y+j)).x;
                     float g1_b = read_imagef(out3, (int2)(x+i,y+j)).x;
                     float g2_b = read_imagef(out4, (int2)(x+i,y+j)).x;
                     float g3_b = read_imagef(out5, (int2)(x+i,y+j)).x;

                     float3 g_b = (float3)(g1_b,g2_b,g3_b);
                     float3 n_b = (float3)(n1_b,n2_b,n3_b);

                     if(isConnected(g_a, n_a, g_b, n_b)){
                         // printf("Connected: (%d,%d)\n",x+i, y+j);
                         patch = (1 << count) | patch;
                     }

                }
            }
        }

        write_imageui(out6, (int2)(x,y), (uint4)(patch, 0, 0, 0));
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++