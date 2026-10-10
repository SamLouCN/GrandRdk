// Test-only exhaustive suppression oracle. Use the same OpenCL expressions
// as the former algorithm: NumPy dot can differ by an ULP at the cosine cutoff.
__kernel void rg_test_peak_suppression(__global const int *peaks,
    __global const float2 *angles,__global uint *masks,int radius) {
    int i=get_global_id(0);if(i>=2880)return;
    __global uint *row=masks+i*90;
    for(int k=0;k<90;++k)row[k]=0;
    float2 normal=angles[peaks[i*4]];
    float offset=peaks[i*4+1]-radius-dot(normal,(float2)(319.5f,179.5f));
    for(int j=0;j<2880;++j) {
        float2 other=angles[peaks[j*4]];float cosine=dot(normal,other);
        float distance=peaks[j*4+1]-radius-dot(other,(float2)(319.5f,179.5f));
        if(peaks[j*4+2]>0&&fabs(cosine)>.9993908f&&fabs(distance*(cosine<0?-1:1)-offset)<4)
            row[j/32]|=1u<<(j%32);
    }
}
