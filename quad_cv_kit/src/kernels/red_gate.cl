// OpenCL 1.2; no FP64, CUDA, or OpenCV's implicit CPU fallback.
#pragma OPENCL FP_CONTRACT OFF

inline int reflected(int p, int size) {
    if (size <= 1) return 0;
    while (p < 0 || p >= size) p = p < 0 ? -p : 2*size-p-2;
    return p;
}
inline int occupied(__global const uchar *mask, int w, int h, float2 p) {
    int x = convert_int_rte(p.x), y = convert_int_rte(p.y);
    return x >= 0 && x < w && y >= 0 && y < h && mask[y*w+x] != 0;
}
__kernel void gaussian_axis(__global const float *src, __global float *dst,
    __global const float *weights, int w, int h, int radius, int horizontal) {
    int i = get_global_id(0); if (i >= w*h) return;
    int x = i%w, y = i/w; float value = 0;
    for (int k=-radius; k<=radius; ++k) {
        int px = horizontal ? reflected(x+k,w) : x;
        int py = horizontal ? y : reflected(y+k,h);
        value += src[py*w+px]*weights[k+radius];
    }
    dst[i] = value;
}

// 64 lanes cooperatively stage 256 output pixels plus the halo. Horizontal
// strips and vertical 16x16 tiles keep global loads contiguous in both passes.
// Symmetric pairs halve multiplications; no change to Gaussian support/sigma.
__kernel void gaussian_tiled(__global const float *src, __global float *dst,
    __global const float *weights, int w,int h,int radius,int horizontal,
    __local float *tile,__local float *taps) {
    int lid=get_local_id(0),lanes=get_local_size(0),group=get_group_id(0);
    int tiles_x=horizontal?(w+255)/256:(w+15)/16;
    int ox=(group%tiles_x)*(horizontal?256:16);
    int oy=(group/tiles_x)*(horizontal?1:16);
    int count=horizontal?256+2*radius:16*(16+2*radius);
    for(int j=lid;j<count;j+=lanes) {
        int x=horizontal?reflected(ox+j-radius,w):reflected(ox+j%16,w);
        int y=horizontal?oy:reflected(oy+j/16-radius,h);
        tile[j]=src[y*w+x];
    }
    for(int j=lid;j<=radius;j+=lanes)taps[j]=weights[radius+j];
    barrier(CLK_LOCAL_MEM_FENCE);
    for(int j=lid;j<256;j+=lanes) {
        int x=ox+(horizontal?j:j%16),y=oy+(horizontal?0:j/16);
        if(x>=w||y>=h)continue;
        int center=horizontal?j+radius:j+radius*16,step=horizontal?1:16;
        float value=tile[center]*taps[0];
        for(int k=1;k<=radius;++k)value+=(tile[center-k*step]+tile[center+k*step])*taps[k];
        dst[y*w+x]=value;
    }
}

// Final colour Gaussian pass also computes evidence, eliminating a full-image
// blur write/read and the separate local_contrast dispatch.
__kernel void gaussian_local(__global const float *src,__global const float *chroma,
    __global float *out,__global const float *weights,int w,int h,int radius,int first,
    __local float *tile,__local float *taps) {
    int lid=get_local_id(0),lanes=get_local_size(0),group=get_group_id(0);
    int tiles_x=(w+15)/16,ox=(group%tiles_x)*16,oy=(group/tiles_x)*16;
    for(int j=lid;j<16*(16+2*radius);j+=lanes) {
        int x=reflected(ox+j%16,w),y=reflected(oy+j/16-radius,h);
        tile[j]=src[y*w+x];
    }
    for(int j=lid;j<=radius;j+=lanes)taps[j]=weights[radius+j];
    barrier(CLK_LOCAL_MEM_FENCE);
    for(int j=lid;j<256;j+=lanes) {
        int x=ox+j%16,y=oy+j/16;if(x>=w||y>=h)continue;
        int center=j+radius*16;
        float value=tile[center]*taps[0];
        for(int k=1;k<=radius;++k)value+=(tile[center-k*16]+tile[center+k*16])*taps[k];
        int pixel=y*w+x;float delta=chroma[pixel]-value;
        out[pixel]=first?delta:fmax(out[pixel],delta);
    }
}

// Fuse the two small-radius passes in local memory. Only final pixels touch
// global output: no full-image temporary write/read and one launch instead of
// two. Use the same reflection, symmetric pairs and float operation order.
__kernel void gaussian_small_fused(__global const float *src,__global float *dst,
    __global const float *weights,int w,int h,int radius,
    __local float *tile,__local float *horizontal,__local float *taps) {
    int lid=get_local_id(0),lanes=get_local_size(0),group=get_group_id(0);
    int tiles_x=(w+15)/16,ox=(group%tiles_x)*16,oy=(group/tiles_x)*16;
    int extent=16+2*radius;
    for(int j=lid;j<extent*extent;j+=lanes) {
        int x=reflected(ox+j%extent-radius,w),y=reflected(oy+j/extent-radius,h);
        tile[j]=src[y*w+x];
    }
    for(int j=lid;j<=radius;j+=lanes)taps[j]=weights[radius+j];
    barrier(CLK_LOCAL_MEM_FENCE);
    for(int j=lid;j<extent*16;j+=lanes) {
        int center=(j/16)*extent+j%16+radius;
        float value=tile[center]*taps[0];
        for(int k=1;k<=radius;++k)value+=(tile[center-k]+tile[center+k])*taps[k];
        horizontal[j]=value;
    }
    barrier(CLK_LOCAL_MEM_FENCE);
    for(int j=lid;j<256;j+=lanes) {
        int x=ox+j%16,y=oy+j/16;if(x>=w||y>=h)continue;
        int center=j+radius*16;
        float value=horizontal[center]*taps[0];
        for(int k=1;k<=radius;++k)value+=(horizontal[center-k*16]+horizontal[center+k*16])*taps[k];
        dst[y*w+x]=value;
    }
}

// Large Gaussian scales operate on an area-averaged pyramid. The small scale
// remains exact; this is an explicitly reported approximation, never FP16.
__kernel void area_down4(__global const float *src,__global float *dst,int w,int h,int dw,int dh) {
    int i=get_global_id(0);if(i>=dw*dh)return;
    int x=i%dw,y=i/dw,count=0;float sum=0;
    for(int dy=0;dy<4;++dy)for(int dx=0;dx<4;++dx) {
        int xx=x*4+dx,yy=y*4+dy;if(xx<w&&yy<h){sum+=src[yy*w+xx];++count;}
    }
    dst[i]=sum/count;
}
static inline float linear_up4_value(__global const float *src,int i,int w,int dw,int dh) {
    float x=((i%w)+.5f)/4-.5f,y=((i/w)+.5f)/4-.5f;
    int ix=(int)floor(x),iy=(int)floor(y);float fx=x-ix,fy=y-iy,sum=0;
    for(int dy=0;dy<2;++dy)for(int dx=0;dx<2;++dx) {
        int xx=max(0,min(dw-1,ix+dx)),yy=max(0,min(dh-1,iy+dy));
        sum+=src[yy*dw+xx]*(dx?fx:1-fx)*(dy?fy:1-fy);
    }
    return sum;
}
__kernel void linear_up4(__global const float *src,__global float *dst,int w,int h,int dw,int dh) {
    int i=get_global_id(0);if(i>=w*h)return;
    dst[i]=linear_up4_value(src,i,w,dw,dh);
}
__kernel void linear_up4_local(__global const float *src,__global const float *chroma,
    __global float *out,int w,int h,int dw,int dh,int first) {
    int i=get_global_id(0);if(i>=w*h)return;
    float delta=chroma[i]-linear_up4_value(src,i,w,dw,dh);
    out[i]=first?delta:fmax(out[i],delta);
}

// OpenCV 8-bit BGR->HSV uses reciprocal integer tables with 12-bit shifts.
__kernel void bgr_hsv(__global const uchar *src,__global uchar *hsv,
    __global const int *sdiv,__global const int *hdiv,int count) {
    int i=get_global_id(0);if(i>=count)return;
    int b=src[3*i],g=src[3*i+1],r=src[3*i+2];
    int v=max(r,max(g,b)),low=min(r,min(g,b)),diff=v-low;
    int s=(diff*sdiv[v]+2048)>>12;
    int hue=r==v?g-b:(g==v?b-r+2*diff:r-g+4*diff);
    hue=(hue*hdiv[diff]+2048)>>12;if(hue<0)hue+=180;
    hsv[3*i]=(uchar)hue;hsv[3*i+1]=(uchar)s;hsv[3*i+2]=(uchar)v;
}
__kernel void median_parameters(__global const uint *hist,__global float *params,float gain) {
    uint count=0;for(int i=0;i<256;++i)count+=hist[i];
    uint before=0;int low=0,high=0,got_low=0,got_high=0;
    for(int i=0;i<256;++i) {
        before+=hist[i];
        if(!got_low&&before>(count-1)/2){low=i;got_low=1;}
        if(!got_high&&before>count/2){high=i;got_high=1;}
    }
    float pivot=(low+high)*.5f;
    params[0]=pivot;params[1]=gain;
    params[2]=(clamp((low-pivot)*gain+pivot,0.0f,255.0f)+clamp((high-pivot)*gain+pivot,0.0f,255.0f))*.5f;
}
__kernel void contrast_device(__global const uchar *hsv,__global const uchar *valid,
    __global const float *params,__global float *value,__global float *blur_input,int count) {
    int i=get_global_id(0);if(i>=count)return;
    float v=clamp(((float)hsv[3*i+2]-params[0])*params[1]+params[0],0.0f,255.0f);
    value[i]=v;blur_input[i]=(valid[i]&&hsv[3*i+2])?v:params[2];
}
__kernel void sharpen_bgr(__global const uchar *source,__global const uchar *hsv,
    __global const uchar *valid,__global const float *value,__global const float *blur,
    __global uchar *out,int count,float amount,float saturation) {
    int i=get_global_id(0);if(i>=count)return;
    if(!valid[i]||!hsv[3*i+2]) {
        for(int c=0;c<3;++c)out[3*i+c]=source[3*i+c];return;
    }
    float delta=value[i]-blur[i];if(fabs(delta)<3)delta=0;
    float v=(float)convert_uchar_sat_rte(clamp(value[i]+amount*delta,0.0f,255.0f));
    float s=(float)convert_uchar_sat_rte(hsv[3*i+1]*saturation)/255;
    float hue=(float)hsv[3*i]/30;int sector=(int)floor(hue);float f=hue-sector;
    float p=v*(1-s),q=v*(1-s*f),t=v*(1-s*(1-f));float r,g,b;
    switch(sector) {
        case 0:r=v;g=t;b=p;break;case 1:r=q;g=v;b=p;break;
        case 2:r=p;g=v;b=t;break;case 3:r=p;g=q;b=v;break;
        case 4:r=t;g=p;b=v;break;default:r=v;g=p;b=q;break;
    }
    out[3*i]=convert_uchar_sat_rte(b);out[3*i+1]=convert_uchar_sat_rte(g);out[3*i+2]=convert_uchar_sat_rte(r);
}

inline float2 trim_point(__global const float *p,int j,int count) {
    float2 a=(float2)(p[0],p[1]),b=(float2)(p[2],p[3]);
    return j==count-1?b:a+(b-a)*((float)j/(count-1));
}
__kernel void trim_probe(__global const uchar *mask,__global const float *params,
    __global uchar *hits,int w,int h,int stride,int nlines) {
    int i=get_global_id(0),s=i/stride,j=i%stride;if(s>=nlines)return;
    __global const float *p=params+s*12;int count=(int)p[11];
    if(j>=count){hits[i]=0;return;}
    float2 point=trim_point(p,j,count),normal=(float2)(-p[5],p[4]);
    int radius=max(4,min(8,(int)(p[10]*.65f))),hit=0;
    for(int k=-radius;k<=radius;++k)hit|=occupied(mask,w,h,point+k*normal);
    hits[i]=(uchar)hit;
}
__kernel void trim_morph(__global const uchar *src,__global uchar *dst,
    __global const float *params,int stride,int nlines,int dilate,int radius) {
    int i=get_global_id(0),s=i/stride,j=i%stride;if(s>=nlines)return;
    int count=(int)params[s*12+11],hit=dilate?0:1;
    if(j>=count){dst[i]=0;return;}
    for(int k=-radius;k<=radius;++k) {
        int x=j+k,v=x>=0&&x<count?src[s*stride+x]:(dilate?0:1);
        hit=dilate?max(hit,v):min(hit,v);
    }
    dst[i]=(uchar)hit;
}
__kernel void trim_extract(__global const uchar *mask,__global const float *score,
    __global const float *params,__global const uchar *hits,__global const uchar *joined,
    __global float *results,__global int *counts,int w,int h,int stride,int nlines,int maxruns) {
    int s=get_global_id(0);if(s>=nlines)return;
    __global const float *p=params+s*12;int count=(int)p[11],runs=0,start=-1;
    float2 d=(float2)(p[4],p[5]),normal=(float2)(p[6],p[7]);
    for(int end=0;end<=count;++end) {
        if(end<count&&joined[s*stride+end]){if(start<0)start=end;continue;}
        if(start<0)continue;
        int last=end-1,span=end-start,total=0,strong_count=0;
        float low=dot(trim_point(p,start,count),d),high=dot(trim_point(p,last,count),d);
        if(span<24||high-low<24||fmin(high,p[9])-fmax(low,p[8])<fmin(15.0f,.4f*(p[9]-p[8]))) {start=-1;continue;}
        for(int j=start;j<end;++j) {
            total+=hits[s*stride+j]!=0;float2 point=trim_point(p,j,count);
            int x=max(0,min(w-1,convert_int_rte(point.x))),y=max(0,min(h-1,convert_int_rte(point.y)));
            strong_count+=score[y*w+x]>.30f;
        }
        if((float)total/span<.5f){start=-1;continue;}
        float strong_low=low,strong_high=high;
        if(strong_count>10) {
            float q0=.02f*(strong_count-1),q1=.98f*(strong_count-1);
            int ranks[4]={(int)floor(q0),(int)ceil(q0),(int)floor(q1),(int)ceil(q1)},rank=0;
            float values[4]={0,0,0,0};
            for(int j=start;j<end;++j) {
                float2 point=trim_point(p,j,count);
                int x=max(0,min(w-1,convert_int_rte(point.x))),y=max(0,min(h-1,convert_int_rte(point.y)));
                if(score[y*w+x]>.30f) {
                    for(int k=0;k<4;++k)if(rank==ranks[k])values[k]=dot(point,d);
                    ++rank;
                }
            }
            strong_low=values[0]+(values[1]-values[0])*(q0-floor(q0));
            strong_high=values[2]+(values[3]-values[2])*(q1-floor(q1));
        }
        int widths[34],nwidth=0;for(int k=0;k<34;++k)widths[k]=0;
        float offset=dot((float2)(p[0],p[1]),normal);
        float2 a=normal*offset+d*low,b=normal*offset+d*high;
        float2 delta=b-a;float len=length(delta);int ns=max(10,(int)len);
        float2 section=(float2)(-delta.y,delta.x)/fmax(len,1.0f);
        int radius=min(16,max(4,(int)p[10]));
        for(int j=0;j<ns;++j) {
            float2 point=j==ns-1?b:a+delta*((float)j/(ns-1));int width=0;
            for(int k=-radius;k<=radius;++k)width+=occupied(mask,w,h,point+k*section);
            if(width>0){++widths[width];++nwidth;}
        }
        float width=p[10];
        if(nwidth) {
            int sum=0,first=0,second=0;
            for(int k=1;k<34;++k) {
                sum+=widths[k];if(!first&&sum>(nwidth-1)/2)first=k;if(!second&&sum>nwidth/2)second=k;
            }
            width=(first+second)*.5f;
        }
        if(runs<maxruns) {
            int out=(s*maxruns+runs)*6;
            results[out]=low;results[out+1]=high;results[out+2]=fmax(2.0f,width);
            results[out+3]=(float)total/span;results[out+4]=strong_low;results[out+5]=strong_high;
        }
        ++runs;start=-1;
    }
    counts[s]=runs;
}

// Compute expensive cross sections over ALL samples, rather than serially
// inside a handful of line work items. Fast mode samples along rods at ~2px.
__kernel void trim_sections(__global const uchar *mask,__global const float *score,
    __global const float *params,__global uchar *widths,__global uchar *strong,
    int w,int h,int stride,int nlines) {
    int i=get_global_id(0),s=i/stride,j=i%stride;if(s>=nlines)return;
    __global const float *p=params+s*12;int count=(int)p[11];
    if(j>=count){widths[i]=0;strong[i]=0;return;}
    float2 point=trim_point(p,j,count),normal=(float2)(p[6],p[7]);
    int radius=min(16,max(4,(int)p[10])),width=0;
    for(int k=-radius;k<=radius;++k)width+=occupied(mask,w,h,point+k*normal);
    widths[i]=(uchar)width;
    int x=max(0,min(w-1,convert_int_rte(point.x))),y=max(0,min(h-1,convert_int_rte(point.y)));
    strong[i]=(uchar)(score[y*w+x]>.30f);
}
__kernel void trim_extract_fast(__global const float *params,
    __global const uchar *hits,__global const uchar *joined,
    __global const uchar *widths,__global const uchar *strong,
    __global float *results,__global int *counts,int stride,int nlines,int maxruns) {
    int s=get_global_id(0);if(s>=nlines)return;
    __global const float *p=params+s*12;int count=(int)p[11],base=s*stride,runs=0,start=-1;
    float2 d=(float2)(p[4],p[5]);
    for(int end=0;end<=count;++end) {
        if(end<count&&joined[base+end]){if(start<0)start=end;continue;}
        if(start<0)continue;
        int last=end-1,span=end-start,total=0,strong_count=0,nwidth=0,hist[34];
        float low=dot(trim_point(p,start,count),d),high=dot(trim_point(p,last,count),d);
        if(high-low<24||fmin(high,p[9])-fmax(low,p[8])<fmin(15.0f,.4f*(p[9]-p[8]))) {start=-1;continue;}
        for(int k=0;k<34;++k)hist[k]=0;
        for(int j=start;j<end;++j) {
            total+=hits[base+j]!=0;strong_count+=strong[base+j]!=0;
            int width=widths[base+j];if(width){++hist[width];++nwidth;}
        }
        if((float)total/span<.5f){start=-1;continue;}
        float strong_low=low,strong_high=high;
        if(strong_count>5) {
            float q0=.02f*(strong_count-1),q1=.98f*(strong_count-1);
            int ranks[4]={(int)floor(q0),(int)ceil(q0),(int)floor(q1),(int)ceil(q1)},rank=0;
            float values[4]={0,0,0,0};
            for(int j=start;j<end;++j)if(strong[base+j]) {
                for(int k=0;k<4;++k)if(rank==ranks[k])values[k]=dot(trim_point(p,j,count),d);
                ++rank;
            }
            strong_low=values[0]+(values[1]-values[0])*(q0-floor(q0));
            strong_high=values[2]+(values[3]-values[2])*(q1-floor(q1));
        }
        float width=p[10];int sum=0,first=0,second=0;
        for(int k=1;k<34&&nwidth;++k) {
            sum+=hist[k];if(!first&&sum>(nwidth-1)/2)first=k;if(!second&&sum>nwidth/2)second=k;
        }
        if(nwidth)width=(first+second)*.5f;
        if(runs<maxruns) {
            int out=(s*maxruns+runs)*6;
            results[out]=low;results[out+1]=high;results[out+2]=fmax(2.0f,width);
            results[out+3]=(float)total/span;results[out+4]=strong_low;results[out+5]=strong_high;
        }
        ++runs;start=-1;
    }
    counts[s]=runs;
}

__kernel void side_support(__global const uchar *mask,__global const float *params,
    __global float *support,int w,int h,int nsides) {
    int s=get_global_id(0);if(s>=nsides)return;__global const float *p=params+s*12;
    float2 a=(float2)(p[0],p[1]),b=(float2)(p[2],p[3]),d=(float2)(p[4],p[5]);
    float len=length(b-a);support[s]=-1;if(len<22)return;
    float t0=dot(a,d),t1=dot(b,d),low=fmin(t0,t1),high=fmax(t0,t1);
    if(fmax(fmax(p[7]-low,high-p[8]),0.0f)>fmax(22.0f,2.2f*p[6]))return;
    if(fmax(fmax(low-p[9],p[10]-high),0.0f)>fmax(65.0f,.45f*len))return;
    float margin=fmin(.1f,fmax(3.0f,p[6])/len);float2 delta=b-a;
    a+=margin*delta;b-=margin*delta;delta=b-a;len=length(delta);
    int count=max(10,(int)len),radius=max(3,min(8,(int)(p[6]*.55f)));
    float2 normal=(float2)(-delta.y,delta.x)/fmax(len,1.0f);
    int bins[8]={0,0,0,0,0,0,0,0},sizes[8],total=0;
    for(int k=0;k<8;++k)sizes[k]=count/8+(k<count%8);
    int bin=0,end=sizes[0];
    for(int j=0;j<count;++j) {
        while(j>=end&&bin<7)end+=sizes[++bin];
        float2 point=j==count-1?b:a+delta*((float)j/(count-1));int hit=0;
        for(int k=-radius;k<=radius;++k)hit|=occupied(mask,w,h,point+k*normal);
        bins[bin]+=hit;total+=hit;
    }
    if((float)total/count<.62f)return;
    for(int k=0;k<8;++k)if((k<2||k>=6)&&(float)bins[k]/sizes[k]<.35f)return;
    support[s]=(float)total/count;
}
__kernel void local_contrast(__global const float *chroma, __global const float *blur,
    __global float *evidence, int count, int first) {
    int i=get_global_id(0); if(i>=count)return;
    float v=chroma[i]-blur[i]; evidence[i]=first?v:fmax(evidence[i],v);
}
__kernel void tube_threshold(__global const uchar *hsv, __global const float *chroma,
    __global const float *evidence, __global uchar *mask, __global float *score, int count) {
    int i=get_global_id(0); if(i>=count)return;
    int hue=hsv[3*i], sat=hsv[3*i+1], value=hsv[3*i+2];
    float c=chroma[i], l=evidence[i];
    int colored=(((hue<14 || hue>118) && sat>4) || (sat<60 && c>126 && l>3));
    mask[i]=(colored && c>125 && value>18 && (l>1.3f || c>136))?255:0;
    score[i]=clamp((c-125)/15,0.0f,1.0f)*clamp(l/4,0.0f,1.0f)*colored;
}
__kernel void morph3(__global const uchar *src, __global uchar *dst, int w, int h, int dilate) {
    int i=get_global_id(0); if(i>=w*h)return;
    int x=i%w,y=i/w, value=dilate?0:255;
    for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx) {
        int xx=x+dx,yy=y+dy;
        int p=(xx>=0&&xx<w&&yy>=0&&yy<h)?src[yy*w+xx]:(dilate?0:255);
        value=dilate?max(value,p):min(value,p);
    }
    dst[i]=(uchar)value;
}
__kernel void combine_masks(__global const uchar *clean, __global const uchar *near,
    __global const uchar *extra, __global uchar *dst, int count) {
    int i=get_global_id(0);if(i<count)dst[i]=clean[i] | (near[i]&extra[i]);
}
__kernel void log_signal(__global const uchar *bgr, __global float *dst, int count) {
    int i=get_global_id(0); if(i<count)dst[i]=log(((float)bgr[3*i+2]+10)/((float)bgr[3*i+1]+10));
}
__kernel void strict_red(__global const uchar *hsv, __global const float *signal,
    __global const float *blur, __global uchar *mask, int count) {
    int i=get_global_id(0);if(i>=count)return;
    int hue=hsv[3*i],sat=hsv[3*i+1],v=hsv[3*i+2];float s=signal[i];
    int near_red=(hue<30||hue>115)&&sat>20&&v>20&&v<250&&s>-.24f&&s-blur[i]>.055f;
    int red=(hue<25||hue>145)&&sat>50&&v>20&&s>.1f;
    mask[i]=(near_red||red)?255:0;
}
__kernel void histogram_value(__global const uchar *hsv, __global const uchar *valid,
    __global uint *histogram, int count, __local uint *bins) {
    int lid=get_local_id(0),group=get_local_size(0),base=get_group_id(0)*512;
    for(int j=lid;j<256;j+=group)bins[j]=0;
    barrier(CLK_LOCAL_MEM_FENCE);
    for(int i=base+lid;i<min(base+512,count);i+=group) {
        uchar v=hsv[3*i+2];if(valid[i]&&v>0)atomic_inc(bins+v);
    }
    barrier(CLK_LOCAL_MEM_FENCE);
    for(int j=lid;j<256;j+=group)if(bins[j])atomic_add(histogram+j,bins[j]);
}
__kernel void contrast_value(__global const uchar *hsv, __global const uchar *valid,
    __global const float *initial, __global float *value, __global float *blur_input,
    int count, float pivot, float gain, float fill, int use_initial) {
    int i=get_global_id(0);if(i>=count)return;
    float start=use_initial?initial[i]:(float)hsv[3*i+2];
    float v=clamp((start-pivot)*gain+pivot,0.0f,255.0f);
    value[i]=v;blur_input[i]=(valid[i]&&hsv[3*i+2]>0)?v:fill;
}
__kernel void sharpen_value(__global const uchar *hsv, __global const float *value,
    __global const float *blur, __global uchar *output, int count, float amount, float saturation) {
    int i=get_global_id(0);if(i>=count)return;
    float delta=value[i]-blur[i];if(fabs(delta)<3)delta=0;
    output[3*i]=hsv[3*i];output[3*i+1]=convert_uchar_sat_rte(hsv[3*i+1]*saturation);
    output[3*i+2]=convert_uchar_sat_rte(clamp(value[i]+amount*delta,0.0f,255.0f));
}
__kernel void remap_bgr(__global const uchar *src, __global const float *mapx,
    __global const float *mapy, __global uchar *dst, int w, int h) {
    int i=get_global_id(0);if(i>=w*h)return;
    // Match OpenCV INTER_LINEAR's 5-bit interpolation table, constant border.
    float mx=mapx[i],my=mapy[i];
    if(!isfinite(mx)||!isfinite(my)||fabs(mx)>1e7f||fabs(my)>1e7f) {
        dst[3*i]=dst[3*i+1]=dst[3*i+2]=0;return;
    }
    int ix=convert_int_rte(mx*32),iy=convert_int_rte(my*32);
    int x=ix>>5,y=iy>>5,fx=ix&31,fy=iy&31;
    for(int c=0;c<3;++c) {
        int sum=0;
        for(int dy=0;dy<2;++dy)for(int dx=0;dx<2;++dx) {
            int xx=x+dx,yy=y+dy;
            if(xx>=0&&xx<w&&yy>=0&&yy<h)
                sum+=src[3*(yy*w+xx)+c]*(dx?fx:32-fx)*(dy?fy:32-fy);
        }
        dst[3*i+c]=(uchar)((sum+512)>>10);
    }
}

// Deterministic polar Hough + supported runs. Unlike CPU HoughLinesP, it does
// not consume/update a random pixel set. All peaks/runs are retained, uncapped.
__kernel void compact_foreground(__global const uchar *mask, __global int2 *points,
    __global uint *count, int w, int h) {
    int i=get_global_id(0);if(i<w*h&&mask[i]) {
        uint j=atomic_inc(count);points[j]=(int2)(i%w,i/w);
    }
}
__kernel void compact_foreground_fast(__global const uchar *mask,__global int2 *points,
    __global uint *count,int w,int h) {
    int i=get_global_id(0);if(i>=w*h)return;
    int x=i%w,y=i/w;
    if(mask[i]&&((x+y)&1)==0) {
        uint j=atomic_inc(count);points[j]=(int2)(x,y);
    }
}
static inline void hough_vote_impl(__global const int2 *points, int count,
    __global const float2 *angles, __global int *votes, int nrhos, int radius,
    __local int *histogram) {
    int a=get_group_id(0),lid=get_local_id(0),group=get_local_size(0);
    for(int r=lid;r<nrhos;r+=group)histogram[r]=0;
    barrier(CLK_LOCAL_MEM_FENCE);
    float2 normal=angles[a];
    for(int i=lid;i<count;i+=group) {
        float2 p=convert_float2(points[i]);int r=convert_int_rte(dot(p,normal))+radius;
        if(r>=0&&r<nrhos)atomic_inc(histogram+r);
    }
    barrier(CLK_LOCAL_MEM_FENCE);
    for(int r=lid;r<nrhos;r+=group)votes[a*nrhos+r]=histogram[r];
}
__kernel void hough_vote(__global const int2 *points,int count,
    __global const float2 *angles,__global int *votes,int nrhos,int radius,__local int *histogram) {
    hough_vote_impl(points,count,angles,votes,nrhos,radius,histogram);
}
__kernel void hough_vote_compacted(__global const int2 *points,__global const uint *metadata,
    __global const float2 *angles,__global int *votes,int nrhos,int radius,__local int *histogram) {
    // In-order queue already completed compaction; no CPU count read is needed.
    hough_vote_impl(points,(int)metadata[0],angles,votes,nrhos,radius,histogram);
}
static inline void hough_peaks_impl(__global const int *votes, __global int4 *peaks,
    __global uint *count, int nrhos, int nangles, int threshold) {
    int i=get_global_id(0);if(i>=nrhos*nangles)return;
    int value=votes[i];if(value<threshold)return;
    int r=i%nrhos,a=i/nrhos;
    for(int da=-1;da<=1;++da)for(int dr=-1;dr<=1;++dr) {
        int aa=a+da,rr=r+dr;if(aa<0||aa>=nangles||rr<0||rr>=nrhos)continue;
        int j=aa*nrhos+rr;
        if(votes[j]>value||(votes[j]==value&&j<i))return;
    }
    uint j=atomic_inc(count);peaks[j]=(int4)(a,r,value,i);
}
__kernel void hough_peaks(__global const int *votes,__global int4 *peaks,
    __global uint *count,int nrhos,int nangles,int threshold) {
    hough_peaks_impl(votes,peaks,count,nrhos,nangles,threshold);
}
__kernel void hough_peaks_compacted(__global const int *votes,__global int4 *peaks,
    __global uint *metadata,int nrhos,int nangles,int threshold) {
    hough_peaks_impl(votes,peaks,metadata+1,nrhos,nangles,threshold);
}
__kernel void hough_runs(__global const uchar *mask, __global const int4 *peaks,
    __global const float2 *angles, __global float4 *segments, __global int *run_counts,
    int w,int h,int npeaks,int radius,int maxruns,int min_length,int max_gap) {
    int i=get_global_id(0);if(i>=npeaks)return;
    float2 n=angles[peaks[i].x],d=(float2)(-n.y,n.x);float rho=peaks[i].y-radius;
    float t0=fmin(fmin(0.0f,(w-1)*d.x),fmin((h-1)*d.y,(w-1)*d.x+(h-1)*d.y));
    float t1=fmax(fmax(0.0f,(w-1)*d.x),fmax((h-1)*d.y,(w-1)*d.x+(h-1)*d.y));
    int first=-1,last=-1,runs=0;
    for(int s=0;s<=convert_int_rtp(t1-t0)+max_gap+1;++s) {
        float2 p=n*rho+d*(t0+s);
        int hit=occupied(mask,w,h,p)||occupied(mask,w,h,p+n)||occupied(mask,w,h,p-n);
        if(hit) {if(first<0)first=s;last=s;}
        if(first>=0&&s-last>max_gap) {
            if(last-first>=min_length) {
                if(runs<maxruns) {
                    float2 a=n*rho+d*(t0+first),b=n*rho+d*(t0+last);
                    segments[i*maxruns+runs]=(float4)(a,b);
                }
                ++runs;
            }
            first=-1;last=-1;
        }
    }
    run_counts[i]=runs;
}

// One cooperative work group per bounded peak. Clip the line to the image,
// sample every 2px in parallel, then extract runs from cheap local hit flags.
// There is no global greedy-consumption dependency between candidates.
__kernel void hough_runs_fast(__global const uchar *mask,__global const int4 *peaks,
    __global const float2 *angles,__global float4 *segments,__global int *run_counts,
    int w,int h,int npeaks,int radius,int maxruns,int min_length,int max_gap,
    int stride,__local int *hits) {
    int i=get_group_id(0),lid=get_local_id(0),lanes=get_local_size(0);if(i>=npeaks)return;
    float2 n=angles[peaks[i].x],d=(float2)(-n.y,n.x);float rho=peaks[i].y-radius;
    float2 origin=n*rho;float low=-1.e9f,high=1.e9f;
    if(fabs(d.x)>1.e-6f) {
        float a=-origin.x/d.x,b=(w-1-origin.x)/d.x;
        low=fmax(low,fmin(a,b));high=fmin(high,fmax(a,b));
    } else if(origin.x<0||origin.x>w-1)high=low-1;
    if(fabs(d.y)>1.e-6f) {
        float a=-origin.y/d.y,b=(h-1-origin.y)/d.y;
        low=fmax(low,fmin(a,b));high=fmin(high,fmax(a,b));
    } else if(origin.y<0||origin.y>h-1)high=low-1;
    int count=high>=low?min(stride,(int)ceil((high-low)/2)+1):0;
    for(int j=lid;j<count;j+=lanes) {
        float2 point=origin+d*fmin(high,low+2*j);
        hits[j]=occupied(mask,w,h,point)||occupied(mask,w,h,point+n)||occupied(mask,w,h,point-n);
    }
    barrier(CLK_LOCAL_MEM_FENCE);
    if(lid==0) {
        int first=-1,last=-1,runs=0;
        for(int j=0;j<count+(max_gap+1)/2+2;++j) {
            if(j<count&&hits[j]){if(first<0)first=j;last=j;}
            if(first>=0&&(j-last)*2>max_gap) {
                float a=low+2*first,b=fmin(high,low+2*last);
                if(b-a>=min_length) {
                    if(runs<maxruns)segments[i*maxruns+runs]=(float4)(origin+d*a,origin+d*b);
                    ++runs;
                }
                first=-1;last=-1;
            }
        }
        run_counts[i]=runs;
    }
}

// Greedy support consumption in descending peak-vote order, as in the CPU
// probabilistic transform. Without this step a thick rod supplies hundreds of
// overlapping oblique seeds that can bias line merging. One work item keeps
// ordering deterministic; the expensive voting/run extraction remain parallel.
// No candidate-count cap: reject only runs lacking unclaimed pixel support.
__kernel void hough_select(__global uchar *unclaimed,
    __global const float4 *segments,__global const int *run_counts,
    __global float4 *selected,__global uint *selected_count,
    int w,int h,int npeaks,int maxruns,int threshold) {
    uint count=0;
    for(int peak=0;peak<npeaks;++peak)for(int run=0;run<run_counts[peak];++run) {
        float4 ends=segments[peak*maxruns+run];float2 delta=ends.zw-ends.xy;
        float span=length(delta);int samples=max(2,(int)span+1),hits=0;
        float2 d=delta/fmax(1.0f,span),n=(float2)(-d.y,d.x);
        for(int j=0;j<samples;++j) {
            float2 p=ends.xy+delta*((float)j/(samples-1));
            hits+=occupied(unclaimed,w,h,p)||occupied(unclaimed,w,h,p+n)||occupied(unclaimed,w,h,p-n);
        }
        if(hits<threshold||(float)hits/samples<.48f)continue;
        selected[count++]=ends;
        for(int j=0;j<samples;++j) {
            float2 p=ends.xy+delta*((float)j/(samples-1));
            for(int k=-1;k<=1;++k) {
                float2 q=p+k*n;int x=convert_int_rte(q.x),y=convert_int_rte(q.y);
                if(x>=0&&x<w&&y>=0&&y<h)unclaimed[y*w+x]=0;
            }
        }
    }
    selected_count[0]=count;
}

inline int occupied_words(__global const uint *mask,int w,int h,float2 p) {
    int x=convert_int_rte(p.x),y=convert_int_rte(p.y);
    return x>=0&&x<w&&y>=0&&y<h&&mask[y*w+x]!=0;
}

// Preserve greedy peak order and exact support-consumption rules. All lanes
// cooperate on each candidate's samples/reduction and clear pixels atomically.
// A single cooperative work group replaces the old single work item.
__kernel void hough_select_parallel(__global uint *unclaimed,
    __global const float4 *segments,__global const int *run_counts,
    __global float4 *selected,__global uint *selected_count,
    int w,int h,int npeaks,int maxruns,int threshold,__local int *sums) {
    int lid=get_local_id(0),lanes=get_local_size(0);uint count=0;
    for(int peak=0;peak<npeaks;++peak)for(int run=0;run<run_counts[peak];++run) {
        float4 ends=segments[peak*maxruns+run];float2 delta=ends.zw-ends.xy;
        float span=length(delta);int samples=max(2,(int)span+1),hits=0;
        float2 d=delta/fmax(1.0f,span),n=(float2)(-d.y,d.x);
        for(int j=lid;j<samples;j+=lanes) {
            float2 p=ends.xy+delta*((float)j/(samples-1));
            hits+=occupied_words(unclaimed,w,h,p)||occupied_words(unclaimed,w,h,p+n)||occupied_words(unclaimed,w,h,p-n);
        }
        sums[lid]=hits;barrier(CLK_LOCAL_MEM_FENCE);
        for(int step=lanes/2;step>0;step/=2) {
            if(lid<step)sums[lid]+=sums[lid+step];
            barrier(CLK_LOCAL_MEM_FENCE);
        }
        int accept=sums[0]>=threshold&&(float)sums[0]/samples>=.48f;
        // Make every lane finish reading the reduced count before the next
        // iteration reuses sums (including rejected candidates).
        barrier(CLK_LOCAL_MEM_FENCE);
        if(accept) {
            if(lid==0)selected[count]=ends;
            ++count;
            for(int j=lid;j<samples;j+=lanes) {
                float2 p=ends.xy+delta*((float)j/(samples-1));
                for(int k=-1;k<=1;++k) {
                    float2 q=p+k*n;int x=convert_int_rte(q.x),y=convert_int_rte(q.y);
                    if(x>=0&&x<w&&y>=0&&y<h)atomic_and(unclaimed+y*w+x,0u);
                }
            }
        }
        barrier(CLK_GLOBAL_MEM_FENCE);
    }
    if(lid==0)selected_count[0]=count;
}

// Each sample finds the nearest occupied 21px cross-section run. Sampling
// every other GOOD section and 90th-percentile checks match the CPU rules.
__kernel void fit_sections(__global const uchar *mask, __global const float4 *segments,
    __global float2 *points,__global float *widths,__global uchar *good,
    int w,int h,int nsegments,int stride) {
    int i=get_global_id(0),s=i/stride,j=i%stride;if(s>=nsegments)return;
    float4 ends=segments[s];float2 a=ends.xy,b=ends.zw,delta=b-a;
    float seglen=length(delta);int n=max(10,(int)seglen);
    if(j>=n) {good[i]=0;return;}
    int vertical=fabs(delta.x)<fabs(delta.y)*.65f;
    if((vertical&&delta.y<0)||(!vertical&&delta.x<0)) {float2 tmp=a;a=b;b=tmp;delta=-delta;}
    float2 normal=(float2)(-delta.y,delta.x)/fmax(seglen,1.0f);
    float2 p=a+(b-a)*((float)j/(n-1));if(j==n-1)p=b;
    int stripe[21],nearest=-1,distance=100;
    for(int k=0;k<21;++k) {
        stripe[k]=occupied(mask,w,h,p+(k-10)*normal);
        if(stripe[k]&&abs(k-10)<distance){nearest=k;distance=abs(k-10);}
    }
    if(nearest<0){good[i]=0;return;}
    int low=nearest,high=nearest;
    while(low>0&&stripe[low-1])--low;
    while(high<20&&stripe[high+1])++high;
    points[i]=p+((low+high)*.5f-10)*normal;
    widths[i]=(float)(high-low+1);good[i]=1;
}
__kernel void fit_moments(__global const float4 *segments,__global const float2 *points,
    __global const float *widths,__global const uchar *good,__global float *result,
    __global float *residuals,__global float *selected_widths,__global int *counts,
    int nsegments,int stride,int sortsize,float min_support) {
    int s=get_global_id(0);if(s>=nsegments)return;
    int base=s*stride,out=s*9;float4 ends=segments[s];float2 delta=ends.zw-ends.xy;
    float len=length(delta);int n=max(10,(int)len),rank=0,m=0;
    int vertical=fabs(delta.x)<fabs(delta.y)*.65f;
    result[out+8]=0;counts[s]=0;float2 sum=(float2)(0);
    for(int j=0;j<n;++j)if(good[base+j]){if((rank&1)==0){sum+=points[base+j];++m;}++rank;}
    if(len<32||m<10||(float)rank/n<min_support)return;
    if(!(vertical||fabs(delta.y)<fabs(delta.x)*.7f))return;
    float2 mean=sum/m;float xx=0,xy=0,yy=0;rank=0;
    for(int j=0;j<n;++j)if(good[base+j]) {
        if((rank&1)==0){float2 p=points[base+j]-mean;xx+=p.x*p.x;xy+=p.x*p.y;yy+=p.y*p.y;}++rank;
    }
    float angle=.5f*atan2(2*xy,xx-yy);float2 d=(float2)(cos(angle),sin(angle));
    if((vertical&&d.y<0)||(!vertical&&d.x<0))d=-d;
    float2 normal=(float2)(-d.y,d.x);float b=dot(mean,normal),lo=INFINITY,hi=-INFINITY;
    rank=0;m=0;
    for(int j=0;j<n;++j)if(good[base+j]) {
        if((rank&1)==0) {
            float2 p=points[base+j];float t=dot(p,d);lo=fmin(lo,t);hi=fmax(hi,t);
            residuals[s*sortsize+m]=fabs(dot(p,normal)-b);
            selected_widths[s*sortsize+m]=widths[base+j];++m;
        }++rank;
    }
    result[out]=d.x;result[out+1]=d.y;result[out+2]=b;result[out+3]=lo;result[out+4]=hi;
    result[out+6]=(float)rank/n;result[out+7]=(float)vertical;counts[s]=m;
}
__kernel void fit_statistics(__global float *result,__global const float *residuals,
    __global const float *widths,__global const int *counts,int sortsize,
    __local float *rs,__local float *ws) {
    int s=get_group_id(0),lid=get_local_id(0),group=get_local_size(0),m=counts[s];
    for(int j=lid;j<sortsize;j+=group) {
        rs[j]=j<m?residuals[s*sortsize+j]:INFINITY;
        ws[j]=j<m?widths[s*sortsize+j]:INFINITY;
    }
    barrier(CLK_LOCAL_MEM_FENCE);
    for(int k=2;k<=sortsize;k*=2)for(int step=k/2;step>0;step/=2) {
        for(int j=lid;j<sortsize;j+=group) {
            int other=j^step;if(j>=other)continue;int ascending=(j&k)==0;
            float a=rs[j],b=rs[other];if((a>b)==ascending){rs[j]=b;rs[other]=a;}
            a=ws[j];b=ws[other];if((a>b)==ascending){ws[j]=b;ws[other]=a;}
        }
        barrier(CLK_LOCAL_MEM_FENCE);
    }
    if(lid==0&&m>=10) {
        float position=.9f*(m-1);int low=(int)floor(position),high=(int)ceil(position);
        float p90=rs[low]+(rs[high]-rs[low])*(position-low);
        float width=(ws[(m-1)/2]+ws[m/2])*.5f;int out=s*9;
        result[out+5]=width;
        result[out+8]=(p90<=5 && (result[out+4]-result[out+3])/fmax(1.0f,width)>=5)?1:0;
    }
}
