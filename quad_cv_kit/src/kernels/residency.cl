// Byte-preserving device transforms. No host transfer of pixel intermediates.
__kernel void resident_copy(__global const uchar *src,__global uchar *out,int n) {
    int i=get_global_id(0);if(i<n)out[i]=src[i];
}
__kernel void resident_roi(__global const uchar *src,__global uchar *out,
    int bytes,int sw,int dw,int dh,int sx,int sy,int dx,int dy,int width,int height) {
    int pixel=get_global_id(0);if(pixel>=dw*dh)return;
    int x=pixel%dw,y=pixel/dw;
    int inside=x>=dx&&x<dx+width&&y>=dy&&y<dy+height;
    int source=((y-dy+sy)*sw+x-dx+sx)*bytes;
    for(int c=0;c<bytes;++c)out[pixel*bytes+c]=inside?src[source+c]:0;
}
__kernel void resident_restrict(__global const uchar *src,__global const uchar *valid,
    __global uchar *out,int w,int h,int x0,int y0,int x1,int y1,int has_valid) {
    int i=get_global_id(0);if(i>=w*h)return;int x=i%w,y=i/w;
    out[i]=(x>=x0&&x<x1&&y>=y0&&y<y1&&(!has_valid||valid[i]))?src[i]:0;
}
__kernel void resident_and(__global const uchar *a,__global const uchar *b,
    __global uchar *out,int n) {
    int i=get_global_id(0);if(i<n)out[i]=a[i]&b[i];
}
__kernel void resident_canvas(__global const uchar *src,__global uchar *out,
    int sw,int rw,int rh,int ox,int oy,int scale) {
    int pixel=get_global_id(0);if(pixel>=640*360)return;
    int x=pixel%640-ox,y=pixel/640-oy;
    for(int c=0;c<3;++c) {
        int i=pixel*3+c;out[i]=0;
        if(x<0||x>=rw||y<0||y>=rh)continue;
        if(scale==1){out[i]=src[(y*sw+x)*3+c];continue;}
        int index=(y*2*sw+x*2)*3+c;
        int sum=src[index]+src[index+3]+src[index+sw*3]+src[index+sw*3+3];
        out[i]=(uchar)((sum+2)>>2);
    }
}
