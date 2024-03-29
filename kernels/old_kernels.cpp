//
// Created by quantum on 2/1/21.
//

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
