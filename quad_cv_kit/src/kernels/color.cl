// Share the BGR load between exact integer HSV and local OpenCV LAB a.
__kernel void bgr_color(__global const uchar *src,__global uchar *hsv,
    __global float *chroma,__global const uchar *lab_a,
    __global const int *sdiv,__global const int *hdiv,int count) {
    int i=get_global_id(0);if(i>=count)return;
    int b=src[3*i],g=src[3*i+1],r=src[3*i+2];
    int v=max(r,max(g,b)),low=min(r,min(g,b)),diff=v-low;
    int s=(diff*sdiv[v]+2048)>>12;
    int hue=r==v?g-b:(g==v?b-r+2*diff:r-g+4*diff);
    hue=(hue*hdiv[diff]+2048)>>12;if(hue<0)hue+=180;
    hsv[3*i]=(uchar)hue;hsv[3*i+1]=(uchar)s;hsv[3*i+2]=(uchar)v;
    chroma[i]=(float)lab_a[(b<<16)|(g<<8)|r];
}
