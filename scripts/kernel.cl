__kernel void depthKernel(
	read_only image2d_t in, 
	write_only image2d_t out
)

{
	int2 pos = (int2)(get_global_id(0), get_global_id(1));

	__const sampler_t sampler = CLK_NORMALIZED_COORDS_FALSE | CLK_FILTER_NEAREST | CLK_ADDRESS_CLAMP_TO_EDGE;

	uint4 pix = read_imageui(in, sampler, pos);
	pix.x = 255 - pix.z;
	pix.y = 255 - pix.z;

	write_imageui(out, pos, pix);

}