// Resident CV core. All per-frame pixel/line/model/motion decisions stay here.
// The bounded candidate generator is color Hough, NOT OpenCV LSD.
#define RG_W 640
#define RG_H 360
#define RG_N (RG_W*RG_H)
#define RG_LINES 16
#define RG_FEATURES 64
#define RG_STATE 128

// Binary masks only: 32 adjacent pixels per word, no subgroup extensions.
__kernel void morph_pack(__global const uchar *src,__global uint *packed,int w,int h) {
    int i=get_global_id(0),words=(w+31)/32;if(i>=words*h)return;
    int x=(i%words)*32,y=i/words;uint bits=0;
    #pragma unroll
    for(int b=0;b<32;++b)if(x+b<w&&src[y*w+x+b])bits|=1u<<b;
    packed[i]=bits;
}
static inline uint morph_word(__local const uint *tile,int x,int y,int ox,int oy,
    int words,int w,int h,int rows,uint neutral) {
    int gx=ox+x,gy=oy+y;
    if(x<0||x>=10||y<0||y>=rows||gx<0||gx>=words||gy<0||gy>=h)return neutral;
    uint bits=tile[y*10+x];
    if(gx==words-1&&(w&31)){uint valid=(1u<<(w&31))-1u;bits=(bits&valid)|(neutral&~valid);}
    return bits;
}
static inline uint morph_horizontal(__local const uint *tile,int x,int y,int ox,int oy,
    int words,int w,int h,int rows,int dilate) {
    uint neutral=dilate?0u:~0u;
    uint a=morph_word(tile,x-1,y,ox,oy,words,w,h,rows,neutral);
    uint b=morph_word(tile,x,y,ox,oy,words,w,h,rows,neutral);
    uint c=morph_word(tile,x+1,y,ox,oy,words,w,h,rows,neutral);
    uint left=(b<<1)|(a>>31),right=(b>>1)|(c<<31);
    return dilate?(left|b|right):(left&b&right);
}
// One word of horizontal halo suffices for <=4 passes: errors at the outer
// word edge cannot travel 32 bits to the output. Vertical halo is exact.
// Every pass reapplies image-border neutrality, including unused tail bits.
#define MORPH_PACKED(NAME,STEPS) \
__kernel void NAME(__global const uint *src,__global uchar *dst,int w,int h,int operations, \
    __local uint *a,__local uint *b) { \
    int lid=get_local_id(0),words=(w+31)/32,cols=(words+7)/8; \
    int ox=(get_group_id(0)%cols)*8-1,oy=(get_group_id(0)/cols)*8-STEPS,rows=8+2*STEPS; \
    for(int j=lid;j<10*rows;j+=64){int x=ox+j%10,y=oy+j/10; \
        a[j]=x>=0&&x<words&&y>=0&&y<h?src[y*words+x]:0u;} \
    barrier(CLK_LOCAL_MEM_FENCE); \
    for(int pass=0;pass<STEPS;++pass){ \
        __local uint *in=(pass&1)?b:a,*out=(pass&1)?a:b;int dilate=(operations>>pass)&1; \
        int margin=pass+1,active=rows-2*margin; \
        for(int j=lid;j<10*active;j+=64){int x=j%10,y=j/10+margin,gx=ox+x,gy=oy+y; \
            uint p=morph_horizontal(in,x,y-1,ox,oy,words,w,h,rows,dilate); \
            uint q=morph_horizontal(in,x,y,ox,oy,words,w,h,rows,dilate); \
            uint r=morph_horizontal(in,x,y+1,ox,oy,words,w,h,rows,dilate); \
            out[y*10+x]=gx>=0&&gx<words&&gy>=0&&gy<h?(dilate?(p|q|r):(p&q&r)):0u;} \
        barrier(CLK_LOCAL_MEM_FENCE); \
    } \
    __local uint *result=(STEPS&1)?b:a;int xword=ox+1+lid%8,y=oy+STEPS+lid/8; \
    if(xword<words&&y<h){uint bits=result[(STEPS+lid/8)*10+1+lid%8];int x=xword*32; \
        for(int k=0;k<32;++k)if(x+k<w)dst[y*w+x+k]=(uchar)(((bits>>k)&1u)*255u);} \
}
MORPH_PACKED(morph_packed1,1)
MORPH_PACKED(morph_packed2,2)
MORPH_PACKED(morph_packed3,3)
MORPH_PACKED(morph_packed4,4)

// Exact fusion of up to four 3x3 min/max passes. Each pass uses its own
// neutral image border (0 for dilation, 255 for erosion), including intermediate
// images: extending only the initial image would change mixed-pass corners.
__kernel void morph3_fused(__global const uchar *src,__global uchar *dst,
    int w,int h,int steps,int operations,__local uchar *a,__local uchar *b) {
    int lid=get_local_id(0),lanes=get_local_size(0),cols=(w+15)/16;
    int ox=get_group_id(0)%cols*16,oy=get_group_id(0)/cols*16,extent=16+2*steps;
    for(int j=lid;j<extent*extent;j+=lanes) {
        int x=ox+j%extent-steps,y=oy+j/extent-steps;
        a[j]=x>=0&&x<w&&y>=0&&y<h?src[y*w+x]:(operations&1?0:255);
    }
    barrier(CLK_LOCAL_MEM_FENCE);
    for(int pass=0;pass<steps;++pass) {
        __local uchar *in=pass%2?b:a,*out=pass%2?a:b;
        int side=extent-2*(pass+1),dilate=(operations>>pass)&1;
        for(int j=lid;j<side*side;j+=lanes) {
            int tx=j%side+pass+1,ty=j/side+pass+1,x=ox+tx-steps,y=oy+ty-steps;
            int value=dilate?0:255;
            if(x>=0&&x<w&&y>=0&&y<h) {
                for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx) {
                    int sample=in[(ty+dy)*extent+tx+dx];value=dilate?max(value,sample):min(value,sample);
                }
            } else value=(operations>>(pass+1))&1?0:255;
            out[ty*extent+tx]=(uchar)value;
        }
        barrier(CLK_LOCAL_MEM_FENCE);
    }
    __local uchar *result=steps%2?b:a;
    for(int j=lid;j<256;j+=lanes) {
        int x=ox+j%16,y=oy+j/16;
        if(x<w&&y<h)dst[y*w+x]=result[(j/16+steps)*extent+j%16+steps];
    }
}

static inline float rg_area(__global const float *b) {
    return fmax(0.0f,b[2]-b[0])*fmax(0.0f,b[3]-b[1]);
}
static inline float rg_iou(__global const float *a,__global const float *b) {
    float i=fmax(0.0f,fmin(a[2],b[2])-fmax(a[0],b[0]))*
            fmax(0.0f,fmin(a[3],b[3])-fmax(a[1],b[1]));
    float u=rg_area(a)+rg_area(b)-i;return u>0?i/u:0;
}
static inline int rg_boundary(__global const float *b,__global const uchar *valid,int w,int h,int has_valid) {
    int bits=(b[0]<=4?1:0)|(b[2]>=w-4?2:0)|(b[1]<=4?4:0)|(b[3]>=h-4?8:0);
    if(has_valid)for(int side=0;side<4;++side) {
        int bad=0;
        for(int j=0;j<17;++j) {
            float t=(float)j/16;
            float x=side<2?(side?b[2]:b[0]):b[0]+t*(b[2]-b[0]);
            float y=side<2?b[1]+t*(b[3]-b[1]):(side==2?b[1]:b[3]);
            int ix=max(0,min(w-1,convert_int_rte(x))),iy=max(0,min(h-1,convert_int_rte(y)));
            bad+=!valid[iy*w+ix];
        }
        if(bad>=4)bits|=1<<side;
    }
    return bits;
}
// State 96: target box, 100: target flag, 101: last YOLO, 102: boundary bits.
// Control 0: allow search, 1: execute search; 8: canvas ROI, 12: anchor.
__kernel void rg_target(__global const float *boxes,__global const uchar *valid,
    __global float *state,__global float *control,int n,int w,int h,int has_valid,
    int index,int interval,int hold,float padding,float ratio,int ox,int oy) {
    int proposal=-1,match=-1,reason=0;float best_area=-1,best_score=-1,best_match=-1;
    int old=state[100]!=0,within=old&&index-(int)state[101]<=hold;
    for(int i=0;i<n;++i) {
        __global const float *b=boxes+i*6;float area=rg_area(b);
        if(!isfinite(b[0])||!isfinite(b[1])||!isfinite(b[2])||!isfinite(b[3])||area<=0)continue;
        if(area>best_area||(area==best_area&&b[4]>best_score)){proposal=i;best_area=area;best_score=b[4];}
        if(old) {
            int clipped=rg_boundary(b,valid,w,h,has_valid);float iou=rg_iou(b,state+96);
            float ar=fmin(area,rg_area(state+96))/fmax(area,rg_area(state+96));
            if(iou>=.15f&&((clipped&&state[102])||ar>=.4f)) {
                float rank=iou+(state[102]&&clipped?2:0);
                if(rank>best_match){best_match=rank;match=i;}
            }
        }
    }
    if(old&&state[102]&&within) {
        if(match>=0&&rg_boundary(boxes+match*6,valid,w,h,has_valid)){proposal=match;reason=1;}
        else if(state[0]&&match<0){proposal=-1;reason=2;}
    }
    int switched=0;
    if(proposal>=0) {
        __global const float *b=boxes+proposal*6;int bits=rg_boundary(b,valid,w,h,has_valid);
        float ar=old?fmin(rg_area(b),rg_area(state+96))/fmax(rg_area(b),rg_area(state+96)):1;
        switched=old&&(rg_iou(b,state+96)<.15f||(!(bits&&state[102])&&ar<.4f));
        if(switched)state[0]=0;
        for(int k=0;k<4;++k)state[96+k]=clamp(b[k],0.0f,(float)(k%2?h:w));
        state[100]=1;state[101]=(float)index;state[102]=(float)bits;state[104]=b[4];
    } else if(!within){state[100]=0;state[0]=0;}
    for(int k=0;k<32;++k)control[k]=0;
    control[0]=proposal>=0;control[1]=proposal>=0&&(!state[0]||index%interval==0||index-(int)state[1]>hold);
    control[2]=(float)switched;control[3]=(float)reason;control[4]=(float)index;
    control[5]=(float)proposal;control[16]=state[100];control[17]=(float)hold;
    float x0=0,y0=0,x1=w,y1=h;
    if(state[100]) {
        __global const float *b=state+96;
        float px=fmax(4.0f,(b[2]-b[0])*padding),py=fmax(4.0f,(b[3]-b[1])*padding);
        if(state[102]){px=fmax(px,w*.25f);py=fmax(py,h*.25f);}
        x0=fmax(0.0f,b[0]-px);y0=fmax(0.0f,b[1]-py);x1=fmin((float)w,b[2]+px);y1=fmin((float)h,b[3]+py);
        for(int k=0;k<4;++k)control[12+k]=b[k]*ratio+(k%2?oy:ox);
    }
    control[8]=floor(x0*ratio+ox);control[9]=floor(y0*ratio+oy);
    control[10]=ceil(x1*ratio+ox);control[11]=ceil(y1*ratio+oy);
    control[20]=x0;control[21]=y0;control[22]=x1;control[23]=y1;
}

// Area resampling for arbitrary input size, plus validity and enhanced gray.
__kernel void rg_canvas(__global const uchar *reference,__global const uchar *enhanced,
    __global const uchar *valid,__global uchar *bgr,__global uchar *gray,__global uchar *out_valid,
    int sw,int sh,int rw,int rh,int ox,int oy,int has_valid) {
    int i=get_global_id(0);if(i>=RG_N)return;int x=i%RG_W-ox,y=i/RG_W-oy;
    for(int c=0;c<3;++c)bgr[i*3+c]=0;gray[i]=0;out_valid[i]=0;
    if(x<0||y<0||x>=rw||y>=rh)return;
    float ax=(float)x*sw/rw,bx=(float)(x+1)*sw/rw,ay=(float)y*sh/rh,by=(float)(y+1)*sh/rh;
    float sum[3]={0,0,0},g=0,total=(bx-ax)*(by-ay);
    for(int sy=(int)floor(ay);sy<(int)ceil(by)&&sy<sh;++sy)for(int sx=(int)floor(ax);sx<(int)ceil(bx)&&sx<sw;++sx) {
        float weight=(fmin(bx,(float)sx+1)-fmax(ax,(float)sx))*(fmin(by,(float)sy+1)-fmax(ay,(float)sy));
        int p=(sy*sw+sx)*3;
        for(int c=0;c<3;++c)sum[c]+=reference[p+c]*weight;
        g+=(.114f*enhanced[p]+.587f*enhanced[p+1]+.299f*enhanced[p+2])*weight;
    }
    for(int c=0;c<3;++c)bgr[i*3+c]=convert_uchar_sat_rte(sum[c]/total);
    gray[i]=convert_uchar_sat_rte(g/total);
    int vx=min(sw-1,(int)((float)x*sw/rw)),vy=min(sh-1,(int)((float)y*sh/rh));
    out_valid[i]=has_valid?valid[vy*sw+vx]:255;
}
__kernel void rg_masks(__global const uchar *color,__global const uchar *hsv,
    __global const float *signal,__global const float *blur,__global const uchar *valid,
    __global const float *control,__global uchar *mask,__global uchar *strict,__global uchar *white) {
    int i=get_global_id(0);if(i>=RG_N)return;int x=i%RG_W,y=i/RG_W;
    int inside=valid[i]&&x>=control[8]&&x<control[10]&&y>=control[9]&&y<control[11]&&control[16];
    mask[i]=inside?color[i]:0;
    int h=hsv[i*3],s=hsv[i*3+1],v=hsv[i*3+2];
    int near_red=(h<30||h>115)&&s>20&&v>20&&v<250&&signal[i]>-.24f&&signal[i]-blur[i]>.055f;
    int red=(h<25||h>145)&&s>50&&v>20&&signal[i]>.1f;
    strict[i]=inside&&(near_red||red)?255:0;
    white[i]=inside&&(((h>18&&h<102&&s<110&&v>70)||(s<18&&v>130)))?255:0;
}
// Lock-free monotone union-find. Four previously visited neighbors give 8-connectivity.
static inline int rg_root(__global int *parent,int p) {
    int next=atomic_add(parent+p,0);while(next!=p){int after=atomic_add(parent+next,0);
        atomic_min(parent+p,after);p=next;next=after;}return p;
}
__kernel void rg_cc_init(__global const uchar *mask,__global int *parent,__global int *stats,int n,
    __global const float *control) {
    int i=get_global_id(0);if(i>=n||!control[1])return;parent[i]=mask[i]?i:-1;
    stats[i*5]=0;stats[i*5+1]=RG_W;stats[i*5+2]=RG_H;stats[i*5+3]=-1;stats[i*5+4]=-1;
}
__kernel void rg_cc_link(__global const uchar *mask,__global int *parent,int w,int h,__global const float *control) {
    int i=get_global_id(0);if(i>=w*h||!control[1]||!mask[i])return;int x=i%w,y=i/w;
    int neighbors[4]={x?i-1:-1,y&&x?i-w-1:-1,y?i-w:-1,y&&x<w-1?i-w+1:-1};
    for(int k=0;k<4;++k) {
        int j=neighbors[k];if(j<0||!mask[j])continue;
        for(;;) {
            int a=rg_root(parent,i),b=rg_root(parent,j);if(a==b)break;
            int high=max(a,b),low=min(a,b),old=atomic_min(parent+high,low);
            if(old==high)break;
        }
    }
}
__kernel void rg_cc_stats(__global const uchar *mask,__global int *parent,__global int *stats,int w,int h,
    __local int *roots,__global const float *control) {
    if(!control[1])return;
    int base=get_group_id(0)*256,lid=get_local_id(0),lanes=get_local_size(0);
    for(int j=lid;j<256;j+=lanes){int i=base+j;roots[j]=i<w*h&&mask[i]?rg_root(parent,i):-1;
        if(roots[j]>=0)atomic_min(parent+i,roots[j]);}
    barrier(CLK_LOCAL_MEM_FENCE);
    // One global update per component per tile avoids a white background
    // causing hundreds of thousands of contended atomics on a single root.
    for(int j=lid;j<256;j+=lanes) {
        int root=roots[j];if(root<0)continue;int duplicate=0;
        for(int k=0;k<j;++k)if(roots[k]==root){duplicate=1;break;}if(duplicate)continue;
        int count=0,x0=w,y0=h,x1=-1,y1=-1;
        for(int k=j;k<256;++k)if(roots[k]==root){int i=base+k;++count;
            x0=min(x0,i%w);y0=min(y0,i/w);x1=max(x1,i%w);y1=max(y1,i/w);}
        atomic_add(stats+root*5,count);atomic_min(stats+root*5+1,x0);atomic_min(stats+root*5+2,y0);
        atomic_max(stats+root*5+3,x1);atomic_max(stats+root*5+4,y1);
    }
}
__kernel void rg_cc_filter(__global const uchar *mask,__global const int *parent,
    __global const int *stats,__global uchar *out,int n,int minimum,int extent,int both,__global const float *control) {
    int i=get_global_id(0);if(i>=n||!control[1])return;out[i]=0;if(!mask[i])return;
    __global const int *s=stats+parent[i]*5;int w=s[3]-s[1]+1,h=s[4]-s[2]+1;
    out[i]=s[0]>=minimum&&(both?min(w,h):max(w,h))>=extent?255:0;
}

static inline int rg_local_root(__local int *parent,int p) {
    int next=atomic_add(parent+p,0);
    while(next!=p){int after=atomic_add(parent+next,0);atomic_min(parent+p,after);p=next;next=after;}
    return p;
}
static inline void rg_local_union(__local int *parent,int i,int j) {
    for(;;) {
        int a=rg_local_root(parent,i),b=rg_local_root(parent,j);if(a==b)return;
        // Link only an unchanged root. atomic_min can overwrite a concurrently
        // installed edge and disconnect its former parent; CAS never erases it.
        int high=max(a,b),low=min(a,b);if(atomic_cmpxchg(parent+high,high,low)==high)return;
    }
}
// Resolve all intra-tile edges with local atomics. Labels remain the smallest
// global foreground index, exactly as in the former all-global union-find.
__kernel void rg_cc_tile(__global const uchar *mask,__global int *parent,
    __global int *stats,int w,int h,__global const float *control,__local int *roots) {
    if(!control[1])return;
    int lid=get_local_id(0),lanes=get_local_size(0),cols=(w+15)/16;
    int ox=get_group_id(0)%cols*16,oy=get_group_id(0)/cols*16;
    for(int j=lid;j<256;j+=lanes) {
        int x=ox+j%16,y=oy+j/16,i=y*w+x,valid=x<w&&y<h;
        roots[j]=valid&&mask[i]?j:-1;
        if(valid){stats[i*5]=0;stats[i*5+1]=w;stats[i*5+2]=h;stats[i*5+3]=-1;stats[i*5+4]=-1;}
    }
    barrier(CLK_LOCAL_MEM_FENCE);
    for(int j=lid;j<256;j+=lanes) {
        if(atomic_add(roots+j,0)<0)continue;int x=j%16,y=j/16;
        int ns[4]={x?j-1:-1,y&&x?j-17:-1,y?j-16:-1,y&&x<15?j-15:-1};
        for(int k=0;k<4;++k)if(ns[k]>=0&&atomic_add(roots+ns[k],0)>=0)rg_local_union(roots,j,ns[k]);
    }
    barrier(CLK_LOCAL_MEM_FENCE);
    for(int j=lid;j<256;j+=lanes) {
        int x=ox+j%16,y=oy+j/16;if(x>=w||y>=h)continue;
        int root=atomic_add(roots+j,0)<0?-1:rg_local_root(roots,j);
        parent[y*w+x]=root<0?-1:(oy+root/16)*w+ox+root%16;
    }
}
// Only edges crossing a 16x16 tile boundary need global union-find atomics.
// Both upper diagonals are included, also at four-tile corner intersections.
__kernel void rg_cc_boundary(__global const uchar *mask,__global int *parent,
    int w,int h,__global const float *control) {
    int i=get_global_id(0);if(i>=w*h||!control[1]||!mask[i])return;int x=i%w,y=i/w;
    int left=x%16==0,top=y%16==0,right=x%16==15;
    int ns[4]={x&&left?i-1:-1,y&&x&&(left||top)?i-w-1:-1,
               y&&top?i-w:-1,y&&x<w-1&&(right||top)?i-w+1:-1};
    for(int k=0;k<4;++k) {
        int j=ns[k];if(j<0||!mask[j])continue;
        for(;;){int a=rg_root(parent,i),b=rg_root(parent,j);if(a==b)break;
            int high=max(a,b),low=min(a,b);if(atomic_cmpxchg(parent+high,high,low)==high)break;}
    }
}
// An exact local hash reduction, at most 256 roots in 512 buckets. No dropped
// components and no O(256^2) duplicate scans; probing cannot exhaust the table.
__kernel void rg_cc_stats_hash(__global const uchar *mask,__global int *parent,
    __global int *stats,int w,int h,__global const float *control,__local int *table) {
    if(!control[1])return;
    int base=get_group_id(0)*256,lid=get_local_id(0),lanes=get_local_size(0);
    for(int j=lid;j<512;j+=lanes){table[j*6]=-1;table[j*6+1]=0;table[j*6+2]=w;
        table[j*6+3]=h;table[j*6+4]=-1;table[j*6+5]=-1;}
    barrier(CLK_LOCAL_MEM_FENCE);
    for(int j=lid;j<256;j+=lanes) {
        int i=base+j;if(i>=w*h||!mask[i])continue;
        int root=rg_root(parent,i);atomic_min(parent+i,root);
        uint hash=(uint)root;hash^=hash>>16;hash*=0x7feb352du;hash^=hash>>15;
        int slot=(int)(hash&511u);
        for(;;){int old=atomic_cmpxchg(table+slot*6,-1,root);if(old<0||old==root)break;slot=(slot+1)&511;}
        __local int *s=table+slot*6;atomic_add(s+1,1);atomic_min(s+2,i%w);atomic_min(s+3,i/w);
        atomic_max(s+4,i%w);atomic_max(s+5,i/w);
    }
    barrier(CLK_LOCAL_MEM_FENCE);
    for(int j=lid;j<512;j+=lanes) {
        __local const int *s=table+j*6;if(!s[1])continue;
        __global int *out=stats+s[0]*5;
        atomic_add(out,s[1]);atomic_min(out+1,s[2]);atomic_min(out+2,s[3]);atomic_max(out+3,s[4]);atomic_max(out+4,s[5]);
    }
}

// Exact row runs. Capacity ceil(w/2) covers the alternating-pixel worst case;
// row-major run IDs have the same order as their first pixel indices.
__kernel void rg_cc_runs(__global const uchar *mask,__global int2 *runs,
    __global int *counts,__global int *links,__global int *labels,__global int *stats,
    int w,int h,__global const float *control,__local int *prefix) {
    int y=get_group_id(0),lid=get_local_id(0),capacity=(w+1)/2;
    if(y>=h)return;if(!control[1]){if(lid==0)counts[y]=0;return;}
    int chunk=(w+63)/64,begin=min(w,lid*chunk),end=min(w,begin+chunk),count=0;
    for(int x=begin;x<end;++x)count+=mask[y*w+x]&&(x==0||!mask[y*w+x-1]);
    prefix[lid]=count;barrier(CLK_LOCAL_MEM_FENCE);
    for(int step=1;step<64;step*=2){int value=lid>=step?prefix[lid-step]:0;
        barrier(CLK_LOCAL_MEM_FENCE);prefix[lid]+=value;barrier(CLK_LOCAL_MEM_FENCE);}
    int offset=(lid?prefix[lid-1]:0)-1,total=prefix[63];if(lid==0)counts[y]=total;
    // Start/end events use matching ordinal IDs even across lane spans. Each
    // coordinate has a single writer; no lane serially scans a long run.
    __global int *coordinates=(__global int *)runs;
    for(int x=begin;x<end;++x)if(mask[y*w+x]) {
        if(x==0||!mask[y*w+x-1]) {
            int id=y*capacity+(++offset);coordinates[id*2]=x;links[id]=id;
            int pixel=y*w+x;stats[pixel*5]=0;stats[pixel*5+1]=w;stats[pixel*5+2]=h;
            stats[pixel*5+3]=-1;stats[pixel*5+4]=-1;
        }
        if(x==w-1||!mask[y*w+x+1])coordinates[(y*capacity+offset)*2+1]=x;
    }
    barrier(CLK_GLOBAL_MEM_FENCE);
    // Each lane maps a contiguous small span, binary-searching only its first
    // run, then advancing through sorted intervals. No per-pixel atomics.
    int lo=0,hi=total;
    while(lo<hi){int mid=(lo+hi)/2;if(runs[y*capacity+mid].y<begin)lo=mid+1;else hi=mid;}
    int at=lo;
    for(int x=begin;x<end;++x){while(at<total&&runs[y*capacity+at].y<x)++at;
        labels[y*w+x]=mask[y*w+x]?y*capacity+at:-1;}
}
__kernel void rg_cc_run_link(__global const int2 *runs,__global const int *counts,
    __global int *links,int capacity,int h,__global const float *control) {
    int i=get_global_id(0),y=i/capacity,k=i%capacity;
    if(y>=h||!control[1]||y==0||k>=counts[y])return;
    int2 run=runs[i];int base=(y-1)*capacity,lo=0,hi=counts[y-1];
    while(lo<hi){int mid=(lo+hi)/2;if(runs[base+mid].y<run.x-1)lo=mid+1;else hi=mid;}
    for(int j=lo;j<counts[y-1]&&runs[base+j].x<=run.y+1;++j) {
        for(;;){int a=rg_root(links,i),b=rg_root(links,base+j);if(a==b)break;
            int high=max(a,b),low=min(a,b);
            if(atomic_cmpxchg(links+high,high,low)==high)break;}
    }
}
__kernel void rg_cc_run_stats(__global const int2 *runs,__global const int *counts,
    __global int *links,__global int *stats,int capacity,int w,int h,__global const float *control) {
    int i=get_global_id(0),y=i/capacity,k=i%capacity;
    if(y>=h||!control[1]||k>=counts[y])return;
    int root=rg_root(links,i);atomic_min(links+i,root);
    int pixel=(root/capacity)*w+runs[root].x;int2 run=runs[i];__global int *s=stats+pixel*5;
    atomic_add(s,run.y-run.x+1);atomic_min(s+1,run.x);atomic_min(s+2,y);
    atomic_max(s+3,run.y);atomic_max(s+4,y);
}
__kernel void rg_cc_run_filter(__global const int2 *runs,__global const int *links,
    __global int *labels,__global const int *stats,__global uchar *out,int w,int h,int capacity,
    int minimum,int extent,int both,__global const float *control) {
    int i=get_global_id(0);if(i>=w*h||!control[1])return;int run=labels[i];out[i]=0;if(run<0)return;
    int root=links[run],pixel=(root/capacity)*w+runs[root].x;labels[i]=pixel;
    __global const int *s=stats+pixel*5;int cw=s[3]-s[1]+1,ch=s[4]-s[2]+1;
    out[i]=s[0]>=minimum&&(both?min(cw,ch):max(cw,ch))>=extent?255:0;
}

static inline int rg_peak_before(int v,int r,int other,int rho) {
    return r>=0&&(v>other||(v==other&&(rho<0||r<rho)));
}
// Evaluate local maxima once per rho. A lane keeps its exact Top8; a value
// below that list cannot enter the angle's Top8. Six parallel merge levels
// replace eight full rho scans and their repeated 3x3 neighborhood checks.
__kernel void rg_peak_top_tiled(__global const int *votes,__global int *peaks,__global const float *control,
    __global const float2 *angles,int nrhos,int threshold,__local int *values,__local int *indices) {
    int a=get_group_id(0),lid=get_local_id(0),lanes=get_local_size(0),count=lanes;
    float2 normal=angles[a];int eligible=fabs(normal.y)<fabs(normal.x)*.65f||fabs(normal.x)<fabs(normal.y)*.7f;
    int best[8]={0,0,0,0,0,0,0,0},at[8]={-1,-1,-1,-1,-1,-1,-1,-1};
    if(control[1]&&eligible)for(int r=lid;r<nrhos;r+=lanes) {
        int i=a*nrhos+r,v=votes[i];if(v<threshold)continue;int ok=1;
        for(int da=-1;da<=1;++da)for(int dr=-1;dr<=1;++dr) {
            int aa=a+da,rr=r+dr;if(aa<0||aa>=360||rr<0||rr>=nrhos)continue;
            int j=aa*nrhos+rr;if(votes[j]>v||(votes[j]==v&&j<i))ok=0;
        }
        if(ok)for(int k=0;k<8;++k)if(rg_peak_before(v,r,best[k],at[k])) {
            for(int j=7;j>k;--j){best[j]=best[j-1];at[j]=at[j-1];}best[k]=v;at[k]=r;break;
        }
    }
    for(int k=0;k<8;++k){values[lid*8+k]=best[k];indices[lid*8+k]=at[k];}
    barrier(CLK_LOCAL_MEM_FENCE);
    int current=0,bank=lanes*8;
    while(count>1) {
        int other=bank-current;
        for(int j=lid;j<count/2*8;j+=lanes) {
            int pair=j/8,k=j%8,left=current+pair*16,right=left+8,lo=0,hi=k,x=0,y=0;
            while(lo<=hi) {
                x=(lo+hi)/2;y=k-x;
                if(x>0&&y<8&&rg_peak_before(values[right+y],indices[right+y],values[left+x-1],indices[left+x-1]))hi=x-1;
                else if(y>0&&x<8&&!rg_peak_before(values[right+y-1],indices[right+y-1],values[left+x],indices[left+x]))lo=x+1;
                else break;
            }
            int first=y==8||(x<8&&!rg_peak_before(values[right+y],indices[right+y],values[left+x],indices[left+x]));
            int source=first?left+x:right+y;values[other+j]=values[source];indices[other+j]=indices[source];
        }
        barrier(CLK_LOCAL_MEM_FENCE);count/=2;current=other;
    }
    for(int k=lid;k<8;k+=lanes){int p=(a*8+k)*4;
        peaks[p]=a;peaks[p+1]=indices[current+k];peaks[p+2]=values[current+k];peaks[p+3]=a*nrhos+indices[current+k];}
}

// Parallel top-eight rho peaks in each angle, no peak array download/sort.
__kernel void rg_peak_top(__global const int *votes,__global int *peaks,__global const float *control,
    __global const float2 *angles,int nrhos,int threshold,__local int *values,__local int *indices) {
    int a=get_group_id(0),lid=get_local_id(0),lanes=get_local_size(0);
    float2 normal=angles[a];int eligible=fabs(normal.y)<fabs(normal.x)*.65f||fabs(normal.x)<fabs(normal.y)*.7f;
    for(int rank=0;rank<8;++rank) {
        int best=0,at=-1;
        if(control[1]&&eligible)for(int r=lid;r<nrhos;r+=lanes) {
            int i=a*nrhos+r,v=votes[i];if(v<threshold)continue;int ok=1;
            for(int da=-1;da<=1;++da)for(int dr=-1;dr<=1;++dr) {
                int aa=a+da,rr=r+dr;if(aa<0||aa>=360||rr<0||rr>=nrhos)continue;
                int j=aa*nrhos+rr;if(votes[j]>v||(votes[j]==v&&j<i))ok=0;
            }
            for(int k=0;k<rank;++k)if(peaks[(a*8+k)*4+1]==r)ok=0;
            if(ok&&(v>best||(v==best&&(at<0||r<at)))){best=v;at=r;}
        }
        values[lid]=best;indices[lid]=at;barrier(CLK_LOCAL_MEM_FENCE);
        for(int k=lanes/2;k;k/=2) {
            if(lid<k&&(values[lid+k]>values[lid]||(values[lid+k]==values[lid]&&indices[lid+k]>=0&&
                (indices[lid]<0||indices[lid+k]<indices[lid])))){values[lid]=values[lid+k];indices[lid]=indices[lid+k];}
            barrier(CLK_LOCAL_MEM_FENCE);
        }
        if(lid==0){int p=(a*8+rank)*4;peaks[p]=a;peaks[p+1]=indices[0];peaks[p+2]=values[0];peaks[p+3]=a*nrhos+indices[0];}
        barrier(CLK_GLOBAL_MEM_FENCE);
    }
}
// At half-degree spacing only the +/-4 angle buckets can meet the
// original abs(cosine)>.9993908 threshold. Prepare sparse suppression masks
// independently for all candidates, including normal reversal at 0/pi.
// 72 circular consecutive slots touch at most three 32-bit bitset words.
#define RG_PEAKS 2880
#define RG_PEAK_NEIGHBORS 72
__kernel void rg_peak_prepare(__global const int *peaks,__global const float2 *angles,
    __global int *order,__global uint4 *neighbors,__global const float *control,int radius) {
    int i=get_global_id(0);if(i>=RG_PEAKS)return;order[i]=-1;
    if(!control[1]||peaks[i*4+2]<=0)return;
    int a=peaks[i*4];float2 normal=angles[a];
    float offset=peaks[i*4+1]-radius-dot(normal,(float2)(319.5f,179.5f));
    int start=((a-4+360)%360)*8,base=start/32;
    uint masks[3]={0,0,0};
    for(int k=0;k<RG_PEAK_NEIGHBORS;++k) {
        int aa=(a+k/8-4+360)%360,j=aa*8+k%8;
        float2 other=angles[aa];float cosine=dot(normal,other);
        float distance=peaks[j*4+1]-radius-dot(other,(float2)(319.5f,179.5f));
        if(peaks[j*4+2]>0&&fabs(cosine)>.9993908f&&
            fabs(distance*(cosine<0?-1:1)-offset)<4)
            masks[(start%32+k)/32]|=1u<<(j%32);
    }
    neighbors[i]=(uint4)(base,masks[0],masks[1],masks[2]);
}
// One work group per valid peak computes its deterministic rank in parallel.
// Invalid slots were initialized by rg_peak_prepare, so a -1 terminates output.
__kernel void rg_peak_order(__global const int *peaks,__global int *order,
    __global const float *control,__local int *ranks) {
    int i=get_group_id(0),lid=get_local_id(0),lanes=get_local_size(0);
    if(i>=RG_PEAKS||!control[1]||peaks[i*4+2]<=0)return;
    int vote=peaks[i*4+2],key=peaks[i*4+3],rank=0;
    for(int j=lid;j<RG_PEAKS;j+=lanes) {
        int v=peaks[j*4+2],k=peaks[j*4+3];
        rank+=v>0&&(v>vote||(v==vote&&(k<key||(k==key&&j<i))));
    }
    ranks[lid]=rank;barrier(CLK_LOCAL_MEM_FENCE);
    for(int k=lanes/2;k;k/=2){if(lid<k)ranks[lid]+=ranks[lid+k];barrier(CLK_LOCAL_MEM_FENCE);}
    if(lid==0)order[ranks[0]]=i;
}
// Full stable peak ordering: invalid slots sort last; no TopK truncation before
// greedy suppression. Integer vote/key/slot tuple matches rg_peak_order.
#define RG_PEAK_INVALID 2147483647
static inline int rg_peak_id_before(__global const int *peaks,int a,int b) {
    if(a==RG_PEAK_INVALID)return 0;if(b==RG_PEAK_INVALID)return 1;
    int av=peaks[a*4+2],bv=peaks[b*4+2],ak=peaks[a*4+3],bk=peaks[b*4+3];
    return av>bv||(av==bv&&(ak<bk||(ak==bk&&a<b)));
}
__kernel void rg_peak_sort_tiles(__global const int *peaks,__global int *ids,
    __global const float *control,__local int *votes,__local int *keys,__local int *indices) {
    int tile=get_group_id(0),lid=get_local_id(0);
    for(int j=lid;j<256;j+=64){int i=tile*256+j,valid=i<RG_PEAKS&&control[1]&&peaks[i*4+2]>0;
        votes[j]=valid?peaks[i*4+2]:0;keys[j]=valid?peaks[i*4+3]:RG_PEAK_INVALID;
        indices[j]=valid?i:RG_PEAK_INVALID;}
    barrier(CLK_LOCAL_MEM_FENCE);
    for(int width=2;width<=256;width*=2)for(int step=width/2;step;step/=2) {
        for(int j=lid;j<256;j+=64){int other=j^step;if(j>=other)continue;
            int av=votes[j],bv=votes[other],ak=keys[j],bk=keys[other],ai=indices[j],bi=indices[other];
            int before=av>bv||(av==bv&&(ak<bk||(ak==bk&&ai<bi)));
            int after=bv>av||(bv==av&&(bk<ak||(bk==ak&&bi<ai)));
            if((j&width)?before:after){votes[j]=bv;votes[other]=av;keys[j]=bk;keys[other]=ak;
                indices[j]=bi;indices[other]=ai;}}
        barrier(CLK_LOCAL_MEM_FENCE);
    }
    for(int j=lid;j<256;j+=64)ids[tile*256+j]=indices[j];
}
__kernel void rg_peak_sort_merge(__global const int *peaks,__global const int *ids,
    __global int *out,int lists,int span) {
    int i=get_global_id(0),pair=i/(2*span),k=i%(2*span),left=pair*2*span,right=left+span;
    if(pair>=(lists+1)/2)return;
    if(pair*2+1>=lists){out[i]=k<span?ids[left+k]:RG_PEAK_INVALID;return;}
    int lo=max(0,k-span),hi=min(k,span),a=0,b=0;
    while(lo<=hi){a=(lo+hi)/2;b=k-a;
        if(a>0&&b<span&&rg_peak_id_before(peaks,ids[right+b],ids[left+a-1]))hi=a-1;
        else if(b>0&&a<span&&rg_peak_id_before(peaks,ids[left+a],ids[right+b-1]))lo=a+1;
        else break;}
    int take_left=b==span||(a<span&&!rg_peak_id_before(peaks,ids[right+b],ids[left+a]));
    out[i]=ids[take_left?left+a:right+b];
}
__kernel void rg_peak_sort_order(__global const int *ids,__global int *order) {
    int i=get_global_id(0);if(i>=RG_PEAKS)return;order[i]=ids[i]==RG_PEAK_INVALID?-1:ids[i];
}
// Retain exact greedy acceptance (a suppressed peak must not suppress others).
// One pass over the sorted array, exactly three local bitset updates per
// accepted peak, no reduction/barrier per selection.
__kernel void rg_peak_select(__global const int *peaks,__global const float2 *angles,
    __global const int *order,__global const uint4 *neighbors,__global int *selected,
    __global const float *control,int limit,__local uint *suppressed) {
    for(int i=0;i<90;++i)suppressed[i]=0;
    int count=0,counts[2]={0,0};
    if(control[1])for(int rank=0;rank<RG_PEAKS&&count<limit;++rank) {
        int i=order[rank];if(i<0)break;
        if(suppressed[i/32]&(1u<<(i%32)))continue;
        float2 normal=angles[peaks[i*4]];int horizontal=fabs(normal.x)<fabs(normal.y)*.7f;
        if(counts[horizontal]>=limit/2)continue;
        for(int c=0;c<4;++c)selected[count*4+c]=peaks[i*4+c];
        ++count;++counts[horizontal];
        uint4 m=neighbors[i];int base=(int)m.x;
        suppressed[base]|=m.y;
        suppressed[(base+1)%90]|=m.z;
        suppressed[(base+2)%90]|=m.w;
    }
    for(int i=count;i<limit;++i)for(int c=0;c<4;++c)selected[i*4+c]=0;
}
__kernel void rg_hough_vote(__global const int2 *points,__global const uint *metadata,
    __global const float2 *angles,__global int *votes,__global const float *control,
    int nrhos,int radius,__local int *histogram) {
    if(!control[1])return;hough_vote_impl(points,(int)metadata[0],angles,votes,nrhos,radius,histogram);
}
__kernel void rg_runs(__global const uchar *mask,__global const int *peaks,
    __global const float2 *angles,__global float *segments,__global int *counts,
    __global const float *control,int limit,int radius,int maxruns,int stride,__local int *hits) {
    int s=get_group_id(0),lid=get_local_id(0);if(s>=limit)return;
    if(!control[1]||peaks[s*4+2]<=0){if(lid==0)counts[s]=0;return;}
    // Reuse the cooperative run extractor with the same geometry/spacing.
    hough_runs_fast_impl(mask,(__global const int4*)peaks,angles,(__global float4*)segments,counts,
                    RG_W,RG_H,limit,radius,maxruns,35,10,stride,hits);
}

// One cooperative fit per measured run. 128 equidistant cross sections;
// covariance reduction and exact sampled width/residual sorting share local memory.
__kernel void rg_fit(__global const uchar *mask,__global const float *segments,
    __global const int *counts,__global float *lines,__global const float *control,
    int maxruns,int n,__local float *scratch) {
    int s=get_group_id(0),lid=get_local_id(0),lanes=get_local_size(0);if(s>=n)return;
    __global float *out=lines+s*12;if(lid==0)out[11]=0;
    if(!control[1]||s%maxruns>=counts[s/maxruns])return;
    __global const float *p=segments+s*4;float2 a=(float2)(p[0],p[1]),b=(float2)(p[2],p[3]),delta=b-a;
    int vertical=fabs(delta.x)<fabs(delta.y)*.65f;
    if(!(vertical||fabs(delta.y)<fabs(delta.x)*.7f)||length(delta)<35)return;
    if((vertical&&delta.y<0)||(!vertical&&delta.x<0)){float2 tmp=a;a=b;b=tmp;delta=-delta;}
    float2 normal=(float2)(-delta.y,delta.x)/length(delta);
    __local float *xs=scratch,*ys=scratch+128,*ws=scratch+256,*good=scratch+384,*sum=scratch+512;
    for(int j=lid;j<128;j+=lanes) {
        float2 point=a+delta*((float)j/127);int stripe[21],near=-1,distance=100;
        for(int k=0;k<21;++k){stripe[k]=occupied(mask,RG_W,RG_H,point+(k-10)*normal);if(stripe[k]&&abs(k-10)<distance){near=k;distance=abs(k-10);}}
        good[j]=near>=0;ws[j]=INFINITY;xs[j]=ys[j]=0;
        if(near>=0){int lo=near,hi=near;while(lo>0&&stripe[lo-1])--lo;while(hi<20&&stripe[hi+1])++hi;
            point+=((lo+hi)*.5f-10)*normal;xs[j]=point.x;ys[j]=point.y;ws[j]=hi-lo+1;}
    }
    barrier(CLK_LOCAL_MEM_FENCE);
    for(int c=0;c<6;++c) {
        float v=0;for(int j=lid;j<128;j+=lanes)if(good[j]) {
            float x=xs[j],y=ys[j];v+=c==0?1:c==1?x:c==2?y:c==3?x*x:c==4?x*y:y*y;
        }
        sum[lid*6+c]=v;
    }
    barrier(CLK_LOCAL_MEM_FENCE);
    for(int k=lanes/2;k;k/=2){if(lid<k)for(int c=0;c<6;++c)sum[lid*6+c]+=sum[(lid+k)*6+c];barrier(CLK_LOCAL_MEM_FENCE);}
    float m=sum[0];if(m<62)return;
    float2 mean=(float2)(sum[1],sum[2])/m;
    float xx=sum[3]-m*mean.x*mean.x,xy=sum[4]-m*mean.x*mean.y,yy=sum[5]-m*mean.y*mean.y;
    float angle=.5f*atan2(2*xy,xx-yy);float2 d=(float2)(cos(angle),sin(angle));
    if((vertical&&d.y<0)||(!vertical&&d.x<0))d=-d;
    normal=(float2)(-d.y,d.x);float offset=dot(mean,normal);
    for(int j=lid;j<128;j+=lanes){ys[j]=good[j]!=0?fabs(xs[j]*normal.x+ys[j]*normal.y-offset):INFINITY;}
    barrier(CLK_LOCAL_MEM_FENCE);
    for(int k=2;k<=128;k*=2)for(int step=k/2;step;step/=2) {
        for(int j=lid;j<128;j+=lanes){int other=j^step;if(j>=other)continue;
            float av=ws[j],bv=ws[other];if((av>bv)==((j&k)==0)){ws[j]=bv;ws[other]=av;}
            av=ys[j];bv=ys[other];if((av>bv)==((j&k)==0)){ys[j]=bv;ys[other]=av;}}
        barrier(CLK_LOCAL_MEM_FENCE);
    }
    if(lid==0) {
        int count=(int)m;float width=(ws[(count-1)/2]+ws[count/2])*.5f;
        float lo=fmin(dot(a,d),dot(b,d)),hi=fmax(dot(a,d),dot(b,d));
        if(ys[(int)(.9f*(count-1))]>5||(hi-lo)/fmax(width,1.0f)<5)return;
        out[0]=d.x;out[1]=d.y;out[2]=offset;out[3]=lo;out[4]=hi;
        out[5]=width;out[6]=m/128;out[7]=vertical;out[8]=lo;out[9]=hi;out[10]=0;out[11]=1;
    }
}
static inline float rg_strength(__global const float *p){return (p[4]-p[3])*sqrt(fmax(p[5],0.0f))*p[6];}
// Scores from rg_fit/rg_line_validate are finite. Preserve the former float32
// expression and original slot tie break; never recompute sqrt in comparisons.
static inline int rg_rank_before(float a,int ai,float b,int bi) {
    return a>b||(a==b&&ai<bi);
}
#define RG_RANK_TILE 256
#define RG_RANK_INVALID 2147483647
// Local sorting also compacts valid rows. A row below its tile's TopK can
// never belong to global TopK, so only K score/index pairs leave each tile.
__kernel void rg_rank_tiles(__global const float *lines,__global float *scores,
    __global int *ids,int n,int capacity,__local float *values,__local int *indices,
    __global const float *control) {
    int tile=get_group_id(0),lid=get_local_id(0),lanes=get_local_size(0);
    if(!control[1]) {
        for(int j=lid;j<capacity;j+=lanes){scores[tile*capacity+j]=-INFINITY;ids[tile*capacity+j]=RG_RANK_INVALID;}
        return;
    }
    for(int j=lid;j<RG_RANK_TILE;j+=lanes) {
        int i=tile*RG_RANK_TILE+j,valid=i<n&&lines[i*12+11]!=0;
        values[j]=valid?rg_strength(lines+i*12):-INFINITY;
        indices[j]=valid?i:RG_RANK_INVALID;
    }
    barrier(CLK_LOCAL_MEM_FENCE);
    for(int width=2;width<=RG_RANK_TILE;width*=2)for(int step=width/2;step;step/=2) {
        for(int j=lid;j<RG_RANK_TILE;j+=lanes) {
            int other=j^step;if(j>=other)continue;
            float a=values[j],b=values[other];int ai=indices[j],bi=indices[other];
            int swap=(j&width)?rg_rank_before(a,ai,b,bi):rg_rank_before(b,bi,a,ai);
            if(swap){values[j]=b;values[other]=a;indices[j]=bi;indices[other]=ai;}
        }
        barrier(CLK_LOCAL_MEM_FENCE);
    }
    for(int j=lid;j<capacity;j+=lanes){scores[tile*capacity+j]=values[j];ids[tile*capacity+j]=indices[j];}
}
// Merge-path: each output rank independently finds its partition in two
// sorted TopK lists (O(log K)), rather than scanning all input line slots.
__kernel void rg_rank_merge(__global const float *scores,__global const int *ids,
    __global float *out_scores,__global int *out_ids,int lists,int capacity) {
    int i=get_global_id(0),pair=i/capacity,k=i%capacity;
    if(pair>=(lists+1)/2)return;int left=pair*2*capacity,right=left+capacity;
    if(pair*2+1>=lists){out_scores[i]=scores[left+k];out_ids[i]=ids[left+k];return;}
    int lo=0,hi=k,a=0,b=0;
    while(lo<=hi) {
        a=(lo+hi)/2;b=k-a;
        if(a>0&&b<capacity&&rg_rank_before(scores[right+b],ids[right+b],scores[left+a-1],ids[left+a-1]))hi=a-1;
        else if(b>0&&a<capacity&&!rg_rank_before(scores[right+b-1],ids[right+b-1],scores[left+a],ids[left+a]))lo=a+1;
        else break;
    }
    int take_left=b==capacity||(a<capacity&&!rg_rank_before(scores[right+b],ids[right+b],scores[left+a],ids[left+a]));
    int at=take_left?left+a:right+b;out_scores[i]=scores[at];out_ids[i]=ids[at];
}
__kernel void rg_rank_gather(__global const float *lines,__global const int *ids,
    __global float *ranked,int capacity) {
    int i=get_global_id(0);if(i>=capacity)return;int source=ids[i];
    for(int k=0;k<12;++k)ranked[i*12+k]=source==RG_RANK_INVALID?0:lines[source*12+k];
}
// Small final per-orientation Top8: one group caches each score once. Invalid
// output slots are cleared before launch by the caller, as in the old path.
__kernel void rg_rank(__global const float *lines,__global float *ranked,int n,int capacity,int per_orientation,
    __local float *scores,__global const float *control) {
    if(!control[1])return;
    int lid=get_local_id(0),lanes=get_local_size(0);
    for(int i=lid;i<n;i+=lanes)scores[i]=lines[i*12+11]!=0?rg_strength(lines+i*12):-INFINITY;
    barrier(CLK_LOCAL_MEM_FENCE);
    for(int i=lid;i<n;i+=lanes) {
        __global const float *p=lines+i*12;if(!p[11])continue;int rank=0;float score=scores[i];
        for(int j=0;j<n;++j){__global const float *q=lines+j*12;if(!q[11]||(per_orientation&&p[7]!=q[7]))continue;
            rank+=rg_rank_before(scores[j],j,score,i);}
        int limit=per_orientation?capacity/2:capacity;if(rank>=limit)continue;
        if(per_orientation&&!p[7])rank+=limit;
        for(int k=0;k<12;++k)ranked[rank*12+k]=p[k];
    }
}
__kernel void rg_clear(__global float *buffer,int n){int i=get_global_id(0);if(i<n)buffer[i]=0;}
__kernel void rg_merge(__global const float *seeds,__global float *merged,__global const float *control,
    int capacity,__local int *matches) {
    int lid=get_local_id(0),lanes=get_local_size(0),count=0;
    if(!control[1])return;
    for(int i=0;i<capacity;++i) {
        __global const float *p=seeds+i*12;if(!p[11])continue;
        float2 d=(float2)(p[0],p[1]),normal=(float2)(-p[1],p[0]),mid=normal*p[2]+d*((p[3]+p[4])*.5f);
        int found=count;
        for(int j=lid;j<count;j+=lanes) {
            __global const float *q=merged+j*12;float2 od=(float2)(q[0],q[1]),on=(float2)(-q[1],q[0]);
            float2 a=normal*p[2]+d*p[3],b=normal*p[2]+d*p[4];
            float gap=fmax(0.0f,fmax(q[3]-fmax(dot(a,od),dot(b,od)),fmin(dot(a,od),dot(b,od))-q[4]));
            if(p[7]==q[7]&&dot(d,od)>.988f&&fabs(dot(mid,on)-q[2])<fmax(2.5f,.45f*fmin(p[5],q[5]))&&
                gap<fmin(34.0f,fmax(12.0f,2.5f*q[5])))found=min(found,j);
        }
        matches[lid]=found;barrier(CLK_LOCAL_MEM_FENCE);
        for(int k=lanes/2;k;k/=2){if(lid<k)matches[lid]=min(matches[lid],matches[lid+k]);barrier(CLK_LOCAL_MEM_FENCE);}
        found=matches[0];
        if(lid==0) {
            __global float *q=merged+found*12;
            if(found==count)for(int k=0;k<12;++k)q[k]=p[k];
            else {float2 od=(float2)(q[0],q[1]);float a=dot(normal*p[2]+d*p[3],od),b=dot(normal*p[2]+d*p[4],od);
                q[3]=fmin(q[3],fmin(a,b));q[4]=fmax(q[4],fmax(a,b));q[5]=(q[5]+p[5])*.5f;q[8]=q[3];q[9]=q[4];}
        }
        if(found==count)++count;barrier(CLK_GLOBAL_MEM_FENCE);
    }
}
// Validate red-green contrast and trim partial endpoints using strict evidence.
__kernel void rg_line_validate(__global const float *merged,__global const uchar *mask,
    __global const uchar *strict,__global const float *signal,__global float *full,
    __global float *partial,int n,__local float *values,__local int *stats) {
    int s=get_group_id(0),lid=get_local_id(0),lanes=get_local_size(0);if(s>=n)return;
    __global const float *p=merged+s*12;if(!p[11])return;
    float2 d=(float2)(p[0],p[1]),normal=(float2)(-p[1],p[0]),origin=normal*p[2];
    int valid=0,positive=0,first=128,last=-1,hits=0;
    float offsets[5]={-p[5]*1.1f-3,-p[5]*.2f,0,p[5]*.2f,p[5]*1.1f+3};
    for(int j=lid;j<128;j+=lanes) {
        float t=p[3]+(p[4]-p[3])*j/127;float2 point=origin+d*t;float z[5];int inside=1;
        for(int k=0;k<5;++k){float2 q=point+offsets[k]*normal;int x=convert_int_rte(q.x),y=convert_int_rte(q.y);
            inside&=x>=0&&x<RG_W&&y>=0&&y<RG_H;z[k]=signal[max(0,min(RG_H-1,y))*RG_W+max(0,min(RG_W-1,x))];}
        float delta=(z[1]+z[2]+z[3])/3-(z[0]+z[4])*.5f;values[j]=inside?delta:INFINITY;
        valid+=inside;positive+=inside&&delta>.035f;
        int hit=0,radius=max(3,min(8,(int)(p[5]*.55f)));
        for(int k=-radius;k<=radius;++k)hit|=occupied(strict,RG_W,RG_H,point+k*normal);
        if(hit){first=min(first,j);last=max(last,j);++hits;}
    }
    stats[lid*5]=valid;stats[lid*5+1]=positive;stats[lid*5+2]=first;stats[lid*5+3]=last;stats[lid*5+4]=hits;
    barrier(CLK_LOCAL_MEM_FENCE);
    for(int k=lanes/2;k;k/=2){if(lid<k){for(int c=0;c<2;++c)stats[lid*5+c]+=stats[(lid+k)*5+c];
        stats[lid*5+2]=min(stats[lid*5+2],stats[(lid+k)*5+2]);stats[lid*5+3]=max(stats[lid*5+3],stats[(lid+k)*5+3]);stats[lid*5+4]+=stats[(lid+k)*5+4];}barrier(CLK_LOCAL_MEM_FENCE);}
    for(int k=2;k<=128;k*=2)for(int step=k/2;step;step/=2){for(int j=lid;j<128;j+=lanes){int other=j^step;if(j>=other)continue;
        float a=values[j],b=values[other];if((a>b)==((j&k)==0)){values[j]=b;values[other]=a;}}barrier(CLK_LOCAL_MEM_FENCE);}
    if(lid==0) {
        int count=stats[0];if(count<20||values[count/2]<=.035f||(float)stats[1]/count<=.48f)return;
        for(int k=0;k<12;++k)full[s*12+k]=p[k];full[s*12+10]=values[count/2];
        if(stats[3]>=stats[2]&&stats[4]>=40) {
            float lo=p[3]+(p[4]-p[3])*stats[2]/127,hi=p[3]+(p[4]-p[3])*stats[3]/127;
            if(hi-lo>=35&&(hi-lo)/fmax(p[5],1.0f)>=5){for(int k=0;k<12;++k)partial[s*12+k]=p[k];
                partial[s*12+3]=lo;partial[s*12+4]=hi;partial[s*12+8]=lo;partial[s*12+9]=hi;}
        }
    }
}

// Fixed pairs enumerate the eight strongest lines of each orientation.
__kernel void rg_pairs(__global int *pairs,int n) {
    int s=get_global_id(0);if(s>=n)return;int vp=s/28,hp=s%28,a=0,b=0,c=0,d=0,k=0;
    for(int i=0;i<8;++i)for(int j=i+1;j<8;++j){if(k==vp){a=i;b=j;}if(k==hp){c=i+8;d=j+8;}++k;}
    pairs[s*4]=a;pairs[s*4+1]=b;pairs[s*4+2]=c;pairs[s*4+3]=d;
}
static inline float2 rg_endpoint(__global const float *p,int high) {
    return (float2)(-p[1],p[0])*p[2]+(float2)(p[0],p[1])*p[high?4:3];
}
static inline int rg_anchor(float x0,float y0,float x1,float y1,__global const float *control) {
    if(!control[16])return 0;
    float ix=fmax(0.0f,fmin(x1,control[14])-fmax(x0,control[12]));
    float iy=fmax(0.0f,fmin(y1,control[15])-fmax(y0,control[13]));
    float area=fmax(1.0f,(x1-x0)*(y1-y0));
    float bx=fmax(1.0f,control[14]-control[12]),by=fmax(1.0f,control[15]-control[13]);
    float overlap=ix*iy/fmin(area,bx*by);
    float cx=(x0+x1)*.5f,cy=(y0+y1)*.5f;
    return overlap>=.2f&&cx>=control[12]-.2f*bx&&cx<=control[14]+.2f*bx&&cy>=control[13]-.2f*by&&cy<=control[15]+.2f*by;
}
// Each complete model gets a compact output record, including measured rods.
// Candidates are selected on device; only the winning record is downloaded.
__kernel void rg_complete(__global const int *status,__global const float *corners,
    __global const float *support,__global const int *order,__global const float *lines,
    __global const float *control,__global float *models,int n) {
    int s=get_global_id(0);if(s>=n)return;__global float *out=models+s*RG_STATE;out[0]=0;
    if(!control[1]||status[s]!=4)return;float sum=0,area=0,x0=RG_W,y0=RG_H,x1=0,y1=0;
    float widths[4];
    for(int k=0;k<4;++k) {
        float v=support[s*4+k];if(v<0)return;sum+=v;
        float2 a=(float2)(corners[s*8+k*2],corners[s*8+k*2+1]);
        float2 b=(float2)(corners[s*8+((k+1)%4)*2],corners[s*8+((k+1)%4)*2+1]);
        area+=a.x*b.y-a.y*b.x;x0=fmin(x0,a.x);x1=fmax(x1,a.x);y0=fmin(y0,a.y);y1=fmax(y1,a.y);
        __global const float *p=lines+order[s*4+k]*12;widths[k]=p[5];
        for(int c=0;c<12;++c)out[16+k*12+c]=p[c];
        out[64+k*4]=a.x;out[65+k*4]=a.y;out[66+k*4]=b.x;out[67+k*4]=b.y;out[80+k]=v;
        out[8+k*2]=a.x;out[9+k*2]=a.y;
    }
    if(sum*.25f<.73f||!rg_anchor(x0,y0,x1,y1,control))return;
    for(int i=0;i<4;++i)for(int j=i+1;j<4;++j)if(widths[j]<widths[i]){float t=widths[i];widths[i]=widths[j];widths[j]=t;}
    float width=(widths[1]+widths[2])*.5f;
    out[0]=1;out[2]=4;out[3]=1;out[4]=sqrt(fabs(area)*.5f)*pow(width,.7f)*sum*.25f;
    out[5]=width;out[7]=4;out[84]=x0;out[85]=y0;out[86]=x1;out[87]=y1;
}
// Parallel per-line joint construction; white elbows only bridge short gaps.
__kernel void rg_joints(__global const float *lines,__global const uchar *white,__global float *joints) {
    int s=get_global_id(0);if(s>=64)return;int i=s/8,j=s%8+8;
    __global const float *a=lines+i*12,*b=lines+j*12;__global float *out=joints+s*3;out[2]=0;
    if(!a[11]||!b[11])return;int ok=1;float2 q=search_intersection(a,b,&ok);
    if(!ok||fabs(a[0]*b[0]+a[1]*b[1])>.707f||fmax(a[5],b[5])>3*fmin(a[5],b[5]))return;
    float da=fmin(length(rg_endpoint(a,0)-q),length(rg_endpoint(a,1)-q));
    float db=fmin(length(rg_endpoint(b,0)-q),length(rg_endpoint(b,1)-q));
    float gap=fmax(18.0f,fmin(26.0f,1.8f*fmax(a[5],b[5])));
    if(da>gap||db>gap) {
        if(da>55||db>55)return;int x=convert_int_rte(q.x),y=convert_int_rte(q.y),count=0;
        for(int dy=-7;dy<=7;++dy)for(int dx=-7;dx<=7;++dx) {
            int xx=x+dx,yy=y+dy;if(xx>=0&&xx<RG_W&&yy>=0&&yy<RG_H)count+=white[yy*RG_W+xx]!=0;
        }
        if(count<25)return;
    }
    out[0]=q.x;out[1]=q.y;out[2]=1;
}
// A work item owns each connected group. At most 16 rods, no Python graph.
__kernel void rg_partial(__global const float *lines,__global const float *joints,
    __global const float *color,__global const float *control,__global float *models,int base) {
    int s=get_global_id(0);if(s>=16)return;__global float *out=models+(base+s)*RG_STATE;out[0]=0;
    if(!control[1]||!lines[s*12+11])return;
    int members[16],visited[16],count=0;for(int i=0;i<16;++i)visited[i]=0;visited[s]=1;
    for(int pass=0;pass<16;++pass)for(int k=0;k<64;++k)if(joints[k*3+2]) {
        int a=k/8,b=k%8+8;if(visited[a]||visited[b])visited[a]=visited[b]=1;
    }
    int root=16;for(int i=0;i<16;++i)if(visited[i])root=min(root,i);if(s!=root)return;
    // Anchor on the strongest rod, then greedily add connected neighbors.
    int anchor=-1;float strength=-1;
    for(int i=0;i<16;++i)if(visited[i]&&lines[i*12+11]) {
        __global const float *p=lines+i*12;float score=(p[4]-p[3])*pow(p[5],1.5f);
        if(score>strength){strength=score;anchor=i;}
    }
    if(anchor<0)return;members[count++]=anchor;int orientations[2]={0,0};++orientations[anchor>=8];
    while(count<4) {
        int best=-1;float score=-1;
        for(int i=0;i<16;++i)if(visited[i]&&lines[i*12+11]&&orientations[i>=8]<2) {
            int present=0,neighbor=0;
            for(int j=0;j<count;++j){present|=members[j]==i;
                if((i<8)!=(members[j]<8)){int k=i<8?i*8+members[j]-8:members[j]*8+i-8;neighbor|=joints[k*3+2]!=0;}}
            __global const float *p=lines+i*12;float value=(p[4]-p[3])*pow(p[5],1.5f);
            if(!present&&neighbor&&value>score){best=i;score=value;}
        }
        if(best<0)break;members[count++]=best;++orientations[best>=8];
    }
    float widths[4],total=0,redness=-INFINITY,x0=RG_W,y0=RG_H,x1=0,y1=0;
    for(int k=0;k<count;++k){__global const float *p=lines+members[k]*12;
        widths[k]=p[5];total+=p[4]-p[3];redness=fmax(redness,color[members[k]*3]);}
    for(int i=0;i<count;++i)for(int j=i+1;j<count;++j)if(widths[j]<widths[i]){float t=widths[i];widths[i]=widths[j];widths[j]=t;}
    float width=(widths[(count-1)/2]+widths[count/2])*.5f;
    if(width<3||total<70||redness<.15f)return;
    if(count==1&&(color[anchor*3+2]<.4f||color[anchor*3+1]<.15f))return;
    for(int i=0;i<count;++i)for(int j=i+1;j<count;++j)if((members[i]<8)==(members[j]<8)) {
        __global const float *a=lines+members[i]*12,*b=lines+members[j]*12;
        if(fabs(a[0]*b[0]+a[1]*b[1])<.866f||fmax(a[5],b[5])/fmin(a[5],b[5])>2.3f)return;
        float2 a0=rg_endpoint(a,0),a1=rg_endpoint(a,1),b0=rg_endpoint(b,0),b1=rg_endpoint(b,1);
        int visible=a0.x>10&&a0.x<630&&a0.y>10&&a0.y<350&&a1.x>10&&a1.x<630&&a1.y>10&&a1.y<350&&
                    b0.x>10&&b0.x<630&&b0.y>10&&b0.y<350&&b1.x>10&&b1.x<630&&b1.y>10&&b1.y<350;
        float ar=(a[4]-a[3])/a[5],br=(b[4]-b[3])/b[5];if(visible&&fmax(ar,br)/fmin(ar,br)>2.2f)return;
    }
    int corners=0;
    for(int k=0;k<count;++k) {
        int id=members[k];__global const float *p=lines+id*12;float2 a=rg_endpoint(p,0),b=rg_endpoint(p,1);
        for(int j=0;j<count;++j)if((id<8)!=(members[j]<8)) {
            int link=id<8?id*8+members[j]-8:members[j]*8+id-8;
            if(!joints[link*3+2])continue;float2 q=(float2)(joints[link*3],joints[link*3+1]);
            if(length(a-q)<length(b-q))a=q;else b=q;
            int duplicate=0;for(int c=0;c<corners;++c)duplicate|=length(q-(float2)(out[8+c*2],out[9+c*2]))<=15;
            if(!duplicate&&corners<4){out[8+corners*2]=q.x;out[9+corners*2]=q.y;++corners;}
        }
        for(int c=0;c<12;++c)out[16+k*12+c]=p[c];
        out[64+k*4]=a.x;out[65+k*4]=a.y;out[66+k*4]=b.x;out[67+k*4]=b.y;out[80+k]=p[6];
        x0=fmin(x0,fmin(a.x,b.x));x1=fmax(x1,fmax(a.x,b.x));y0=fmin(y0,fmin(a.y,b.y));y1=fmax(y1,fmax(a.y,b.y));
    }
    if(!rg_anchor(x0,y0,x1,y1,control))return;
    out[84]=x0;out[85]=y0;out[86]=x1;out[87]=y1;
    // A four-rod chain is still a partial observation. A closed four-side
    // observation must pass rg_complete; suppress the duplicate chain then.
    if(count==4)for(int i=0;i<base;++i)if(models[i*RG_STATE]&&rg_iou(models+i*RG_STATE+84,out+84)>.5f)return;
    out[0]=1;out[2]=count;out[3]=0;out[4]=strength*(1+.04f*(count-1));out[5]=width;out[7]=corners;
    out[84]=x0;out[85]=y0;out[86]=x1;out[87]=y1;
}

__kernel void rg_rotate_bgr(__global const uchar *src,__global uchar *out,int n) {
    int i=get_global_id(0);if(i>=n)return;
    for(int c=0;c<3;++c)out[i*3+c]=src[(n-1-i)*3+c];
}

// Exact 2x2 opening with OpenCV's default (1,1) anchor, fused on device.
__kernel void rg_white_open2(__global const uchar *src,__global uchar *out,__global const float *control) {
    int i=get_global_id(0);if(i>=RG_N||!control[1])return;int x=i%RG_W,y=i/RG_W,opened=0;
    for(int by=-1;by<=0;++by)for(int bx=-1;bx<=0;++bx) {
        int xx=x+bx,yy=y+by;if(xx<0||yy<0)continue;int eroded=1;
        for(int dy=-1;dy<=0;++dy)for(int dx=-1;dx<=0;++dx) {
            int sx=xx+dx,sy=yy+dy;if(sx>=0&&sy>=0)eroded&=src[sy*RG_W+sx]!=0;
        }
        opened|=eroded;
    }
    out[i]=opened?255:0;
}

__kernel void rg_gray_down(__global const uchar *src,__global uchar *out,int w,int h) {
    int i=get_global_id(0),dw=(w+1)/2,dh=(h+1)/2;if(i>=dw*dh)return;int x=i%dw*2,y=i/dw*2;
    int x1=min(w-1,x+1),y1=min(h-1,y+1);out[i]=(src[y*w+x]+src[y*w+x1]+src[y1*w+x]+src[y1*w+x1]+2)/4;
}
static inline float rg_gray(__global const uchar *src,int w,int h,float2 p) {
    p.x=clamp(p.x,0.0f,(float)w-1);p.y=clamp(p.y,0.0f,(float)h-1);
    int x=(int)floor(p.x),y=(int)floor(p.y),x1=min(w-1,x+1),y1=min(h-1,y+1);
    float fx=p.x-x,fy=p.y-y;return (1-fy)*((1-fx)*src[y*w+x]+fx*src[y*w+x1])+fy*((1-fx)*src[y1*w+x]+fx*src[y1*w+x1]);
}
__kernel void rg_corners(__global const uchar *gray,__global const float *state,__global const float *control,
    __global float *features,__local float *scores,__local int *indices) {
    int tile=get_group_id(0),lid=get_local_id(0),lanes=get_local_size(0),tx=tile%40*16,ty=tile/40*16;
    float best=0;int at=-1;
    if(state[0]&&control[4]-state[1]<=control[17])for(int j=lid;j<256;j+=lanes) {
        int x=tx+j%16,y=ty+j/16;if(x<7||x>=RG_W-7||y<7||y>=RG_H-7)continue;
        int near=0;float2 p=(float2)(x,y);
        for(int k=0;k<(int)state[2];++k){float2 a=(float2)(state[64+k*4],state[65+k*4]),b=(float2)(state[66+k*4],state[67+k*4]);
            float2 d=b-a;float t=clamp(dot(p-a,d)/fmax(dot(d,d),1.0f),0.0f,1.0f);near|=length(p-a-t*d)<16;}
        if(!near)continue;float xx=0,xy=0,yy=0;
        for(int dy=-2;dy<=2;++dy)for(int dx=-2;dx<=2;++dx){int i=(y+dy)*RG_W+x+dx;
            float gx=gray[i+1]-gray[i-1],gy=gray[i+RG_W]-gray[i-RG_W];xx+=gx*gx;xy+=gx*gy;yy+=gy*gy;}
        float score=.5f*(xx+yy-sqrt((xx-yy)*(xx-yy)+4*xy*xy));
        if(score>best){best=score;at=y*RG_W+x;}
    }
    scores[lid]=best;indices[lid]=at;barrier(CLK_LOCAL_MEM_FENCE);
    for(int k=lanes/2;k;k/=2){if(lid<k&&(scores[lid+k]>scores[lid]||(scores[lid+k]==scores[lid]&&indices[lid+k]>=0&&
        (indices[lid]<0||indices[lid+k]<indices[lid])))){scores[lid]=scores[lid+k];indices[lid]=indices[lid+k];}barrier(CLK_LOCAL_MEM_FENCE);}
    if(lid==0){features[tile*4]=indices[0]>=0?indices[0]%RG_W:0;features[tile*4+1]=indices[0]>=0?indices[0]/RG_W:0;
        features[tile*4+2]=scores[0];features[tile*4+3]=scores[0]>100?1:0;}
}
__kernel void rg_feature_rank(__global const float *source,__global float *out,int n) {
    int i=get_global_id(0);if(i>=n||!source[i*4+3])return;int rank=0;float score=source[i*4+2];
    for(int j=0;j<n;++j)rank+=source[j*4+3]&&(source[j*4+2]>score||(source[j*4+2]==score&&j<i));
    if(rank<RG_FEATURES)for(int c=0;c<4;++c)out[rank*4+c]=source[i*4+c];
}
// Cooperative pyramidal LK: template gradients, 13x13 patch, 10 iterations,
// four scales. Each feature has its own work group and reduction scratch.
__kernel void rg_lk(__global const uchar *old0,__global const uchar *old1,__global const uchar *old2,__global const uchar *old3,
    __global const uchar *new0,__global const uchar *new1,__global const uchar *new2,__global const uchar *new3,
    __global const float *features,__global float *moved,__local float *scratch) {
    int s=get_group_id(0),lid=get_local_id(0),lanes=get_local_size(0);if(s>=RG_FEATURES)return;
    if(lid==0)moved[s*4+3]=0;if(!features[s*4+3])return;
    float2 original=(float2)(features[s*4],features[s*4+1]),q=original/8;int okay=1;float error=0;
    for(int level=3;level>=0;--level) {
        __global const uchar *old=level==3?old3:level==2?old2:level==1?old1:old0;
        __global const uchar *next=level==3?new3:level==2?new2:level==1?new1:new0;
        int w=RG_W>>level,h=RG_H>>level;float2 p=original/(1<<level);if(level<3)q*=2;
        for(int iteration=0;iteration<10;++iteration) {
            for(int c=0;c<6;++c)scratch[lid*6+c]=0;
            for(int j=lid;j<169;j+=lanes) {
                float2 delta=(float2)(j%13-6,j/13-6),a=p+delta,b=q+delta;
                float gx=(rg_gray(old,w,h,a+(float2)(1,0))-rg_gray(old,w,h,a-(float2)(1,0)))*.5f;
                float gy=(rg_gray(old,w,h,a+(float2)(0,1))-rg_gray(old,w,h,a-(float2)(0,1)))*.5f;
                float diff=rg_gray(next,w,h,b)-rg_gray(old,w,h,a);
                scratch[lid*6]+=gx*gx;scratch[lid*6+1]+=gx*gy;scratch[lid*6+2]+=gy*gy;
                scratch[lid*6+3]+=gx*diff;scratch[lid*6+4]+=gy*diff;scratch[lid*6+5]+=fabs(diff);
            }
            barrier(CLK_LOCAL_MEM_FENCE);
            for(int k=lanes/2;k;k/=2){if(lid<k)for(int c=0;c<6;++c)scratch[lid*6+c]+=scratch[(lid+k)*6+c];barrier(CLK_LOCAL_MEM_FENCE);}
            float xx=scratch[0],xy=scratch[1],yy=scratch[2],dx=scratch[3],dy=scratch[4],residual=scratch[5];
            // Every lane snapshots the reduction before a converged lane can
            // enter the next level and overwrite shared scratch.
            barrier(CLK_LOCAL_MEM_FENCE);
            float det=xx*yy-xy*xy;
            if(det<1){okay=0;break;}
            float2 step=(float2)((yy*dx-xy*dy)/det,(xx*dy-xy*dx)/det);
            float magnitude=length(step);if(magnitude>4)step*=4/magnitude;q-=step;error=residual/169;
            if(q.x<0||q.x>w-1||q.y<0||q.y>h-1){okay=0;break;}
            barrier(CLK_LOCAL_MEM_FENCE);
            if(length(step)<.01f)break;
        }
        if(!okay)break;
    }
    if(lid==0){moved[s*4]=q.x;moved[s*4+1]=q.y;moved[s*4+2]=error;moved[s*4+3]=okay&&error<35;}
}
// Independent similarity hypotheses and parallel inlier scoring.
__kernel void rg_ransac(__global const float *points,__global const float *moved,
    __global float *hypotheses,__local float *scratch) {
    int s=get_group_id(0),lid=get_local_id(0),lanes=get_local_size(0);
    int i=s%RG_FEATURES,j=(s/RG_FEATURES*17+i*13+7)%RG_FEATURES;
    float2 a=(float2)(points[i*4],points[i*4+1]),b=(float2)(points[j*4],points[j*4+1]);
    float2 u=(float2)(moved[i*4],moved[i*4+1]),v=(float2)(moved[j*4],moved[j*4+1]);
    float2 delta=b-a,change=v-u;float denominator=dot(delta,delta);
    if(lid==0)hypotheses[s*8+4]=0;
    if(!moved[i*4+3]||!moved[j*4+3]||denominator<64)return;
    float aa=dot(delta,change)/denominator,bb=(delta.x*change.y-delta.y*change.x)/denominator;
    float scale=sqrt(aa*aa+bb*bb);if(scale<=.9f||scale>=1.1f)return;
    float2 shift=u-(float2)(aa*a.x-bb*a.y,bb*a.x+aa*a.y);float count=0,error=0;
    for(int k=lid;k<RG_FEATURES;k+=lanes)if(moved[k*4+3]) {
        float2 p=(float2)(points[k*4],points[k*4+1]),q=(float2)(moved[k*4],moved[k*4+1]);
        float2 diff=(float2)(aa*p.x-bb*p.y,bb*p.x+aa*p.y)+shift-q;float e=dot(diff,diff);
        if(e<6.25f){++count;error+=e;}
    }
    scratch[lid*2]=count;scratch[lid*2+1]=error;barrier(CLK_LOCAL_MEM_FENCE);
    for(int k=lanes/2;k;k/=2){if(lid<k){scratch[lid*2]+=scratch[(lid+k)*2];scratch[lid*2+1]+=scratch[(lid+k)*2+1];}barrier(CLK_LOCAL_MEM_FENCE);}
    if(lid==0){__global float *out=hypotheses+s*8;out[0]=aa;out[1]=bb;out[2]=shift.x;out[3]=shift.y;out[4]=scratch[0];out[5]=scratch[1];}
}
__kernel void rg_motion(__global const float *hypotheses,__global const float *points,__global const float *moved,
    __global const float *state,__global float *prediction,__global float *side_params,__global const float *control,
    int n,__local float *scores,__local int *indices) {
    int lid=get_local_id(0),lanes=get_local_size(0),best=-1;float score=-1;
    for(int i=lid;i<n;i+=lanes){float rank=hypotheses[i*8+4]*1000-hypotheses[i*8+5];if(rank>score){score=rank;best=i;}}
    scores[lid]=score;indices[lid]=best;barrier(CLK_LOCAL_MEM_FENCE);
    for(int k=lanes/2;k;k/=2){if(lid<k&&scores[lid+k]>scores[lid]){scores[lid]=scores[lid+k];indices[lid]=indices[lid+k];}barrier(CLK_LOCAL_MEM_FENCE);}
    best=indices[0];if(lid==0)prediction[0]=0;
    if(!state[0]||control[4]-state[1]>control[17]||best<0||hypotheses[best*8+4]<8)return;
    float aa=hypotheses[best*8],bb=hypotheses[best*8+1],tx=hypotheses[best*8+2],ty=hypotheses[best*8+3];
    // Least-squares refinement on the winning inlier set (small fixed set).
    if(lid==0) {
        float sx=0,sy=0,ux=0,uy=0,m=0;
        for(int i=0;i<RG_FEATURES;++i)if(moved[i*4+3]) {
            float x=points[i*4],y=points[i*4+1],u=moved[i*4],v=moved[i*4+1];
            float ex=aa*x-bb*y+tx-u,ey=bb*x+aa*y+ty-v;
            if(ex*ex+ey*ey<6.25f){sx+=x;sy+=y;ux+=u;uy+=v;++m;}
        }
        sx/=m;sy/=m;ux/=m;uy/=m;float num_a=0,num_b=0,den=0;
        for(int i=0;i<RG_FEATURES;++i)if(moved[i*4+3]) {
            float x=points[i*4],y=points[i*4+1],u=moved[i*4],v=moved[i*4+1];
            float ex=aa*x-bb*y+tx-u,ey=bb*x+aa*y+ty-v;
            if(ex*ex+ey*ey<6.25f){x-=sx;y-=sy;u-=ux;v-=uy;num_a+=x*u+y*v;num_b+=x*v-y*u;den+=x*x+y*y;}
        }
        if(den>1){aa=num_a/den;bb=num_b/den;tx=ux-aa*sx+bb*sy;ty=uy-bb*sx-aa*sy;}
        float scale=sqrt(aa*aa+bb*bb);if(scale<=.9f||scale>=1.1f)return;
        for(int i=0;i<96;++i)prediction[i]=state[i];prediction[6]=1;
        prediction[88]=aa;prediction[89]=bb;prediction[90]=tx;prediction[91]=ty;prediction[92]=hypotheses[best*8+4];
        for(int k=0;k<(int)state[7];++k){float x=state[8+k*2],y=state[9+k*2];prediction[8+k*2]=aa*x-bb*y+tx;prediction[9+k*2]=bb*x+aa*y+ty;}
        for(int k=0;k<(int)state[2];++k) {
            __global const float *old=state+16+k*12;__global float *out=prediction+16+k*12;
            float2 d=(float2)(aa*old[0]-bb*old[1],bb*old[0]+aa*old[1])/scale,normal=(float2)(-d.y,d.x);
            float2 a=(float2)(state[64+k*4],state[65+k*4]),b=(float2)(state[66+k*4],state[67+k*4]);
            a=(float2)(aa*a.x-bb*a.y+tx,bb*a.x+aa*a.y+ty);b=(float2)(aa*b.x-bb*b.y+tx,bb*b.x+aa*b.y+ty);
            if(a.x<0||a.x>RG_W-1||a.y<0||a.y>RG_H-1||b.x<0||b.x>RG_W-1||b.y<0||b.y>RG_H-1){prediction[0]=0;return;}
            prediction[64+k*4]=a.x;prediction[65+k*4]=a.y;prediction[66+k*4]=b.x;prediction[67+k*4]=b.y;
            out[0]=d.x;out[1]=d.y;out[2]=dot(a,normal);out[3]=fmin(dot(a,d),dot(b,d));out[4]=fmax(dot(a,d),dot(b,d));
            out[5]=old[5]*scale;out[8]=out[3];out[9]=out[4];
            __global float *p=side_params+k*12;p[0]=a.x;p[1]=a.y;p[2]=b.x;p[3]=b.y;p[4]=d.x;p[5]=d.y;
            p[6]=out[5];p[7]=out[3];p[8]=out[4];p[9]=out[8];p[10]=out[9];p[11]=old[5];
        }
        prediction[5]=state[5]*scale;
    }
}
__kernel void rg_track_validate(__global float *prediction,__global const float *occupancy,
    __global const float *support,__global float *control) {
    if(prediction[0])for(int k=0;k<(int)prediction[2];++k) {
        if(occupancy[k]<.58f||(prediction[3]&&support[k]<0)){prediction[0]=0;break;}
        prediction[80+k]=prediction[3]!=0?support[k]:occupancy[k];
    }
    if(!prediction[0]&&control[0])control[1]=1;
}
static inline int rg_overlap(__global const float *a,__global const float *b) {
    if(!b[0])return 0;
    for(int i=0;i<(int)a[2];++i)for(int j=0;j<(int)b[2];++j) {
        float2 x=(float2)(a[64+i*4]+a[66+i*4],a[65+i*4]+a[67+i*4])*.5f;
        float2 y=(float2)(b[64+j*4]+b[66+j*4],b[65+j*4]+b[67+j*4])*.5f;
        if(length(x-y)<25)return 1;
    }
    return 0;
}
// Preserve nearest selection: 12% width tie, continuity, then observed length.
__kernel void rg_select(__global const float *models,__global const float *prediction,
    __global float *state,__global float *control,int n,__local float *scores,__local int *indices) {
    int lid=get_local_id(0),lanes=get_local_size(0),best=-1;float width=0;
    for(int i=lid;i<n;i+=lanes) {
        __global const float *p=models+i*RG_STATE;if(!control[1]||!p[0])continue;
        if(control[3]==1&&prediction[0]&&p[5]<prediction[5]*.7f)continue;
        width=fmax(width,p[5]);
    }
    scores[lid]=width;barrier(CLK_LOCAL_MEM_FENCE);
    for(int k=lanes/2;k;k/=2){if(lid<k)scores[lid]=fmax(scores[lid],scores[lid+k]);barrier(CLK_LOCAL_MEM_FENCE);}
    float minimum=scores[0]*.88f;int matching=0;
    barrier(CLK_LOCAL_MEM_FENCE);
    for(int i=lid;i<n;i+=lanes){__global const float *p=models+i*RG_STATE;
        matching|=control[1]&&p[0]&&p[5]>=minimum&&rg_overlap(p,prediction);}
    indices[lid]=matching;barrier(CLK_LOCAL_MEM_FENCE);
    for(int k=lanes/2;k;k/=2){if(lid<k)indices[lid]|=indices[lid+k];barrier(CLK_LOCAL_MEM_FENCE);}
    matching=indices[0];barrier(CLK_LOCAL_MEM_FENCE);float score=-1;
    for(int i=lid;i<n;i+=lanes) {
        __global const float *p=models+i*RG_STATE;if(!control[1]||!p[0]||p[5]<minimum)continue;
        if(control[3]==1&&prediction[0]&&p[5]<prediction[5]*.7f)continue;
        if(matching&&!rg_overlap(p,prediction))continue;
        float len=0;for(int j=0;j<(int)p[2];++j)len+=length((float2)(p[66+j*4]-p[64+j*4],p[67+j*4]-p[65+j*4]));
        float rank=len*1000+p[5];if(rank>score||(rank==score&&(best<0||i<best))){score=rank;best=i;}
    }
    scores[lid]=score;indices[lid]=best;barrier(CLK_LOCAL_MEM_FENCE);
    for(int k=lanes/2;k;k/=2){if(lid<k&&(scores[lid+k]>scores[lid]||
        (scores[lid+k]==scores[lid]&&indices[lid+k]>=0&&(indices[lid]<0||indices[lid+k]<indices[lid])))){
        scores[lid]=scores[lid+k];indices[lid]=indices[lid+k];}barrier(CLK_LOCAL_MEM_FENCE);}
    best=indices[0];
    if(lid==0) {
        __global const float *selected=best>=0?models+best*RG_STATE:prediction;
        for(int i=0;i<96;++i)state[i]=selected[i];
        if(best>=0){state[1]=control[4];state[6]=0;}
        control[6]=state[0];control[7]=control[1];control[18]=prediction[0];control[19]=best;
        int count=0;for(int i=0;i<n;++i)count+=control[1]&&models[i*RG_STATE]!=0;control[24]=(float)count;
    }
}

// Planar homography pose from the final four measured sides. Degenerate,
// back-facing or high-reprojection-error poses are rejected on device.
// This intentionally differs from OpenCV IPPE's two-solution ambiguity test.
__kernel void rg_pose(__global const float *state,__global const float *camera,
    __global float *pose,float ratio,int ox,int oy,int has_camera) {
    for(int i=0;i<40;++i)pose[i]=0;
    if(!has_camera||!state[0]||!state[3]||state[102])return;
    float fx=camera[0],fy=camera[4],cx=camera[2],cy=camera[5];if(fx<=0||fy<=0)return;
    float X[4]={-.35f,.35f,.35f,-.35f},Y[4]={-.25f,-.25f,.25f,.25f};
    float matrix[72];for(int i=0;i<72;++i)matrix[i]=0;
    float px[4],py[4];
    for(int i=0;i<4;++i) {
        px[i]=(state[8+i*2]-ox)/ratio;py[i]=(state[9+i*2]-oy)/ratio;
        float u=(px[i]-cx)/fx,v=(py[i]-cy)/fy;int a=i*18,b=a+9;
        matrix[a]=X[i];matrix[a+1]=Y[i];matrix[a+2]=1;matrix[a+6]=-u*X[i];matrix[a+7]=-u*Y[i];matrix[a+8]=u;
        matrix[b+3]=X[i];matrix[b+4]=Y[i];matrix[b+5]=1;matrix[b+6]=-v*X[i];matrix[b+7]=-v*Y[i];matrix[b+8]=v;
    }
    for(int k=0;k<8;++k) {
        int pivot=k;for(int j=k+1;j<8;++j)if(fabs(matrix[j*9+k])>fabs(matrix[pivot*9+k]))pivot=j;
        if(fabs(matrix[pivot*9+k])<1.e-8f)return;
        if(pivot!=k)for(int j=k;j<9;++j){float t=matrix[k*9+j];matrix[k*9+j]=matrix[pivot*9+j];matrix[pivot*9+j]=t;}
        float divisor=matrix[k*9+k];for(int j=k;j<9;++j)matrix[k*9+j]/=divisor;
        for(int i=0;i<8;++i)if(i!=k){float factor=matrix[i*9+k];for(int j=k;j<9;++j)matrix[i*9+j]-=factor*matrix[k*9+j];}
    }
    float h[8];for(int i=0;i<8;++i)h[i]=matrix[i*9+8];
    float r1[3]={h[0],h[3],h[6]},r2[3]={h[1],h[4],h[7]},r3[3],t[3]={h[2],h[5],1};
    float len1=0,len2=0;for(int k=0;k<3;++k){len1+=r1[k]*r1[k];len2+=r2[k]*r2[k];}
    len1=sqrt(len1);len2=sqrt(len2);float scale=2/(len1+len2);if(!isfinite(scale)||scale<=0)return;
    for(int k=0;k<3;++k){r1[k]/=len1;t[k]*=scale;}
    float dot12=0;for(int k=0;k<3;++k)dot12+=r1[k]*r2[k];len2=0;
    for(int k=0;k<3;++k){r2[k]-=dot12*r1[k];len2+=r2[k]*r2[k];}len2=sqrt(len2);if(len2<1.e-6f)return;
    for(int k=0;k<3;++k)r2[k]/=len2;
    r3[0]=r1[1]*r2[2]-r1[2]*r2[1];r3[1]=r1[2]*r2[0]-r1[0]*r2[2];r3[2]=r1[0]*r2[1]-r1[1]*r2[0];
    if(r3[2]<=.1f||t[2]<=0)return;float distance=0;for(int k=0;k<3;++k)distance+=t[k]*r3[k];if(distance<=0)return;
    float error=0;
    for(int i=0;i<4;++i){float x=r1[0]*X[i]+r2[0]*Y[i]+t[0],y=r1[1]*X[i]+r2[1]*Y[i]+t[1],z=r1[2]*X[i]+r2[2]*Y[i]+t[2];
        if(z<=0)return;float dx=fx*x/z+cx-px[i],dy=fy*y/z+cy-py[i];error+=dx*dx+dy*dy;}
    error=sqrt(error/4);float threshold=fmax(2.0f,length((float2)(px[1]-px[0],py[1]-py[0]))*.015f);
    if(!isfinite(error)||error>threshold)return;
    pose[0]=1;pose[1]=error;pose[2]=distance;
    for(int k=0;k<3;++k){pose[3+k*3]=r1[k];pose[4+k*3]=r2[k];pose[5+k*3]=r3[k];pose[12+k]=t[k];
        pose[15+k]=r3[k];pose[18+k]=t[k]-distance*r3[k];pose[21+k]=(k==0?fx:k==1?fy:(fx+fy)*.5f)*pose[18+k]/t[2];}
    pose[24]=atan2(r2[2],r3[2])*57.2957795f;pose[25]=atan2(-r1[2],sqrt(r1[0]*r1[0]+r1[1]*r1[1]))*57.2957795f;
    pose[26]=atan2(r1[1],r1[0])*57.2957795f;pose[27]=fx*t[0]/t[2]+cx;pose[28]=fy*t[1]/t[2]+cy;
    pose[29]=pose[27]-cx;pose[30]=pose[28]-cy;
    for(int i=0;i<4;++i){pose[32+i*2]=px[i];pose[33+i*2]=py[i];}
}
