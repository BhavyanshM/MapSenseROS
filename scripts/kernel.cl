__kernel void depthKernel(__read_only image3d_t in, __write_only image3d_t out)
{
	const int x = get_global_id(0);
	const int y = get_global_id(1);

	sampler_t sampler = CLK_NORMALIZED_COORDS_FALSE | CLK_FILTER_NEAREST | CLK_ADDRESS_CLAMP_TO_EDGE;

	const float pxr = read_imagef(in, sampler, (int4)(x, y, 0,0)).s0;
	const float pxg = read_imagef(in, sampler, (int4)(x, y, 1,0)).s0;
	const float pxb = read_imagef(in, sampler, (int4)(x, y, 2,0)).s0;

	const float finalpx = (x+y)%255;

	write_imagef(out, (int4)(x,y,0,0), (float4)(finalpx, finalpx, finalpx, finalpx));




}