__kernel void depthKernel(
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

float residual(float8 P, read_only image2d_t in, __const sampler_t sam,	int a, int b){
	float total = 0;
	float X = 0;
	float Y = 0;
	float Z = 0;

	float xy_min = -5;
	float xy_max = 5;
	float n = 15;

	float8 params = (float8)(0.0, -10.0, 0.0, 1.0, -1.0, 0.0, 1.0, 0.0);
	int2 pos = (int2)(a,b);
	for (int i = 0; i<16; i++){
		for (int j = 0; j<16; j++){
			X = xy_min + i*(xy_max-xy_min)/n;
			Y = xy_min + j*(xy_max-xy_min)/n;
			uint4 pix = read_imageui(in, sam, pos*16 + (int2)(i,j));
			Z = poly(params,X,Y);
			//Z = (float)(pix.z*216 + pix.y);
			total += pow((poly(P,X,Y) - Z),2);
		}
	}
	return total;
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


float8 calc_grad(float8 P, float gdel, read_only image2d_t in, __const sampler_t sam,int a,int b){
	float grad[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	for (int i = 0; i<7; i++){
		float dp[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
		dp[i] = gdel;
		float8 dps = vload8(0,dp); 
		grad[i] = (residual(P+dps, in, sam, a, b)-residual(P-dps, in, sam, a, b))/(2*gdel);
	}

	return vload8(0,grad);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

float8 update_params(float8 P, float8 grad, float alpha){
	return P - alpha*grad;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


__kernel void segmentKernel(
	read_only image2d_t in, 
	write_only image2d_t out,
	const int h,
	const int w
)

{
	int2 pos = (int2)(get_global_id(0), get_global_id(1));

	__const sampler_t sampler = CLK_NORMALIZED_COORDS_FALSE | CLK_FILTER_NEAREST | CLK_ADDRESS_CLAMP_TO_EDGE;

	int count = 0;
	float r = 100000;
	float alpha = 0.000001;
	float gdel = 0.001;
	float8 P = (float8)(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	float8 grad = (float8)(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	
	float decay = 0.9;

	while(count < 50){
		r = residual(P, in, sampler, pos.x, pos.y);
		count++;
		//if(r > 1000){
			if(pos.x == 24 && pos.y == 24 && count == 19){printf("(%d,%d,%d,%.2f)\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n",pos.x,pos.y,count,r,P.s0,P.s1,P.s2,P.s3,P.s4,P.s5,P.s6);}
			
			grad = calc_grad(P, gdel, in, sampler, pos.x, pos.y);
			P = update_params(P, grad, alpha);
			alpha *= decay;
		//}else{
		//	break;
		//}
	}
	uint4 vc = (uint4)(100, 100, 100, 100);
		
	write_imageui(out, pos, vc);
	

}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++