// One dispatch for max-channel brightness, normalized separable Gaussian,
// noise-thresholded unsharp mask and BGR reconstruction. No global temporary.
__kernel void sharpen_only(__global const uchar *src,__global const uchar *valid,
    __global uchar *out,__global const float *weights,int w,int h,int has_valid,float amount,
    __local float *values,__local float *active,
    __local float *horizontal,__local float *horizontal_active,__local float *taps) {
    int lid=get_local_id(0),lanes=get_local_size(0),group=get_group_id(0);
    int radius=5,extent=26,tiles_x=(w+15)/16;
    int ox=(group%tiles_x)*16,oy=(group/tiles_x)*16;
    for(int j=lid;j<extent*extent;j+=lanes) {
        int x=reflected(ox+j%extent-radius,w),y=reflected(oy+j/extent-radius,h);
        int p=y*w+x,v=max((int)src[p*3],max((int)src[p*3+1],(int)src[p*3+2]));
        int keep=v>0&&(!has_valid||valid[p]);
        values[j]=keep?(float)v:0;active[j]=(float)keep;
    }
    for(int j=lid;j<=radius;j+=lanes)taps[j]=weights[radius+j];
    barrier(CLK_LOCAL_MEM_FENCE);
    for(int j=lid;j<extent*16;j+=lanes) {
        int center=(j/16)*extent+j%16+radius;
        float numerator=values[center]*taps[0],denominator=active[center]*taps[0];
        for(int k=1;k<=radius;++k) {
            numerator+=(values[center-k]+values[center+k])*taps[k];
            denominator+=(active[center-k]+active[center+k])*taps[k];
        }
        horizontal[j]=numerator;horizontal_active[j]=denominator;
    }
    barrier(CLK_LOCAL_MEM_FENCE);
    for(int j=lid;j<256;j+=lanes) {
        int x=ox+j%16,y=oy+j/16;if(x>=w||y>=h)continue;
        int p=y*w+x,center=j+radius*16;
        int v=max((int)src[p*3],max((int)src[p*3+1],(int)src[p*3+2]));
        if(!v||(has_valid&&!valid[p])) {
            for(int c=0;c<3;++c)out[p*3+c]=src[p*3+c];continue;
        }
        float numerator=horizontal[center]*taps[0],denominator=horizontal_active[center]*taps[0];
        for(int k=1;k<=radius;++k) {
            numerator+=(horizontal[center-k*16]+horizontal[center+k*16])*taps[k];
            denominator+=(horizontal_active[center-k*16]+horizontal_active[center+k*16])*taps[k];
        }
        float blur=denominator>0?numerator/denominator:(float)v;
        float detail=(float)v-blur;if(fabs(detail)<3)detail=0;
        float target=(float)convert_uchar_sat_rte((float)v+amount*detail);
        float scale=target/(float)v;
        for(int c=0;c<3;++c)out[p*3+c]=convert_uchar_sat_rte((float)src[p*3+c]*scale);
    }
}
