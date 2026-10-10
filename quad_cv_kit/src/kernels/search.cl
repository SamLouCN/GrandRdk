// Search-specific OpenCL 1.2 pipeline. Included after red_gate.cl.
// No relaxed math: preserve thresholds, greedy order and nearest-even probes.

__kernel void track_occupancy(__global const uchar *mask,__global const float *params,
    __global float *result,int w,int h,int n,__local int *hits) {
    int s=get_group_id(0),lid=get_local_id(0),lanes=get_local_size(0);if(s>=n)return;
    __global const float *p=params+s*12;
    float2 a=(float2)(p[0],p[1]),b=(float2)(p[2],p[3]),delta=b-a;
    float len=length(delta);int count=max(10,(int)len),radius=max(4,(int)(p[11]*.6f));
    float2 normal=(float2)(-delta.y,delta.x)/fmax(len,1.0f);
    int total=0;
    for(int j=lid;j<count;j+=lanes) {
        float2 point=j==count-1?b:a+delta*((float)j/(count-1));int hit=0;
        for(int k=-radius;k<=radius;++k)hit|=occupied(mask,w,h,point+k*normal);
        total+=hit;
    }
    hits[lid]=total;barrier(CLK_LOCAL_MEM_FENCE);
    for(int step=lanes/2;step;step/=2) {
        if(lid<step)hits[lid]+=hits[lid+step];barrier(CLK_LOCAL_MEM_FENCE);
    }
    if(lid==0)result[s]=(float)hits[0]/count;
}

__kernel void search_merge(__global const float *lines,__global float *params,
    __global int *stats,int n,int step,__local int *matches) {
    int lid=get_local_id(0),group=get_local_size(0),count=0,comparisons=0;
    for(int i=0;i<n;++i) {
        __global const float *p=lines+i*12;
        float2 d=(float2)(p[0],p[1]),normal=(float2)(-p[1],p[0]);
        float2 a=normal*p[2]+d*p[3],b=normal*p[2]+d*p[4],mid=(a+b)*.5f;
        int found=count;
        for(int j=lid;j<count;j+=group) {
            __global const float *old=params+j*12;
            float2 od=(float2)(old[4],old[5]),on=(float2)(old[6],old[7]);
            int vertical=fabs(od.x)<fabs(od.y)*.65f;
            if(vertical!=(int)p[7]||dot(od,d)<.988f)continue;
            float distance=fabs(dot(mid,on)-dot((float2)(old[0],old[1]),on));
            float pa=dot(a,od),pb=dot(b,od);
            float gap=fmax(fmax(old[8]-fmax(pa,pb),fmin(pa,pb)-old[9]),0.0f);
            if(distance<fmax(2.5f,.45f*fmin(old[10],p[5]))&&
               gap<fmin(34.0f,fmax(12.0f,2.5f*old[10])))found=min(found,j);
        }
        matches[lid]=found;barrier(CLK_LOCAL_MEM_FENCE);
        for(int k=group/2;k;k/=2) {
            if(lid<k)matches[lid]=min(matches[lid],matches[lid+k]);
            barrier(CLK_LOCAL_MEM_FENCE);
        }
        found=matches[0];
        if(lid==0) {
            comparisons+=found<count?found+1:count;
            __global float *out=params+found*12;
            if(found==count) {
                out[0]=a.x;out[1]=a.y;out[2]=b.x;out[3]=b.y;
                out[4]=d.x;out[5]=d.y;out[6]=normal.x;out[7]=normal.y;
                out[8]=p[3];out[9]=p[4];out[10]=p[5];out[11]=0;
            } else {
                float2 od=(float2)(out[4],out[5]),on=(float2)(out[6],out[7]);
                float offset=dot((float2)(out[0],out[1]),on)/dot(on,on);
                float pa=dot(a,od),pb=dot(b,od);
                out[8]=fmin(out[8],fmin(pa,pb));out[9]=fmax(out[9],fmax(pa,pb));
                out[10]=(out[10]+p[5])*.5f;
                float2 first=on*offset+od*out[8],last=on*offset+od*out[9];
                out[0]=first.x;out[1]=first.y;out[2]=last.x;out[3]=last.y;
            }
        }
        if(found==count)++count;
        barrier(CLK_GLOBAL_MEM_FENCE|CLK_LOCAL_MEM_FENCE);
    }
    for(int j=lid;j<n;j+=group) {
        __global float *p=params+j*12;
        if(j>=count){for(int k=0;k<12;++k)p[k]=0;continue;}
        p[11]=max(10,(int)(length((float2)(p[2]-p[0],p[3]-p[1]))/step));
    }
    if(lid==0){stats[0]=count;stats[1]=comparisons;}
}

__kernel void search_probe(__global const uchar *mask,__global const float *params,
    __global uchar *hits,int w,int h,int stride,int n) {
    int i=get_global_id(0),s=i/stride,j=i%stride;if(s>=n)return;
    __global const float *p=params+s*12;int count=(int)p[11];
    hits[i]=0;if(j>=count)return;
    float2 delta=(float2)(p[2]-p[0],p[3]-p[1]);
    float2 normal=(float2)(-delta.y,delta.x)/fmax(length(delta),1.0f);
    float2 point=trim_point(p,j,count);
    int radius=max(2,(int)(p[10]*.4f)),hit=0;
    for(int k=-radius;k<=radius;++k)hit|=occupied(mask,w,h,point+k*normal);
    hits[i]=(uchar)hit;
}

__kernel void search_runs(__global const float *params,__global const uchar *hits,
    __global const uchar *joined,__global float *runs,__global float *info,
    __global int *total,int stride,int n,int minimum,int maxruns,int step) {
    int output=0;
    for(int s=0;s<n;++s) {
        __global const float *p=params+s*12;int count=(int)p[11],start=-1;
        float2 d=(float2)(p[4],p[5]),normal=(float2)(p[6],p[7]);
        float offset=dot((float2)(p[0],p[1]),normal)/dot(normal,normal);
        for(int end=0;end<=count;++end) {
            if(end<count&&joined[s*stride+end]){if(start<0)start=end;continue;}
            if(start<0)continue;
            int span=end-start;
            float low=dot(trim_point(p,start,count),d),high=dot(trim_point(p,end-1,count),d);
            if(span>=minimum&&(high-low)/fmax(p[10],1.0f)>=5) {
                int occupied_count=0;
                for(int k=start;k<end;++k)occupied_count+=hits[s*stride+k]!=0;
                __global float *out=runs+output*12;
                float2 a=normal*offset+d*low,b=normal*offset+d*high;
                out[0]=a.x;out[1]=a.y;out[2]=b.x;out[3]=b.y;
                for(int k=4;k<8;++k)out[k]=p[k];
                out[8]=low;out[9]=high;out[10]=p[10];out[11]=max(20,(int)((high-low)/step));
                info[output*3]=low;info[output*3+1]=high;
                info[output*3+2]=(float)occupied_count/span;++output;
            }
            start=-1;
        }
    }
    total[0]=output;
}

__kernel void search_contrast(__global const float *reference,__global const float *enhanced,
    __global const float *params,__global float *results,int w,int h,int nruns,int sortsize,
    __local float *values,__local int *counts) {
    int task=get_group_id(0),s=task/2,channel=task%2;
    int lid=get_local_id(0),group=get_local_size(0);
    if(s>=nruns)return;
    __global const float *signal=channel?enhanced:reference;
    __global const float *p=params+s*12;int count=(int)p[11],valid_count=0,positive=0;
    float2 normal=(float2)(p[6],p[7]);
    float offsets[5]={-p[10]*1.1f-3,-p[10]*.2f,0,p[10]*.2f,p[10]*1.1f+3};
    for(int j=lid;j<sortsize;j+=group) {
        float delta=INFINITY;
        if(j<count) {
            float2 point=trim_point(p,j,count);float z[5];int valid=1;
            for(int k=0;k<5;++k) {
                float2 probe=point+offsets[k]*normal;
                int x=convert_int_rte(probe.x),y=convert_int_rte(probe.y);
                valid&=x>=0&&x<w&&y>=0&&y<h;
                z[k]=signal[max(0,min(h-1,y))*w+max(0,min(w-1,x))];
            }
            if(valid) {
                delta=(z[1]+z[2]+z[3])/3-(z[0]+z[4])*.5f;
                ++valid_count;positive+=delta>.035f;
            }
        }
        values[j]=delta;
    }
    counts[lid*2]=valid_count;counts[lid*2+1]=positive;
    barrier(CLK_LOCAL_MEM_FENCE);
    for(int k=group/2;k;k/=2) {
        if(lid<k){counts[lid*2]+=counts[(lid+k)*2];counts[lid*2+1]+=counts[(lid+k)*2+1];}
        barrier(CLK_LOCAL_MEM_FENCE);
    }
    for(int k=2;k<=sortsize;k*=2)for(int step=k/2;step>0;step/=2) {
        for(int j=lid;j<sortsize;j+=group) {
            int other=j^step;if(j>=other)continue;
            float a=values[j],b=values[other];
            if((a>b)==((j&k)==0)){values[j]=b;values[other]=a;}
        }
        barrier(CLK_LOCAL_MEM_FENCE);
    }
    if(lid==0) {
        int m=counts[0];results[task*2]=m>=20?(values[(m-1)/2]+values[m/2])*.5f:0;
        results[task*2+1]=m>=20?(float)counts[1]/m:0;
    }
}

inline float2 search_intersection(__global const float *a,__global const float *b,int *ok) {
    float ax=-a[1],ay=a[0],bx=-b[1],by=b[0],det=ax*by-ay*bx;
    if(fabs(det)<.3f){*ok=0;return (float2)(0,0);}
    return (float2)((a[2]*by-ay*b[2])/det,(ax*b[2]-a[2]*bx)/det);
}
inline float2 search_midpoint(__global const float *p) {
    return (float2)(-p[1],p[0])*p[2]+(float2)(p[0],p[1])*(p[3]+p[4])*.5f;
}

__kernel void search_quads(__global const float *lines,__global const int *pairs,
    __global const uchar *valid,__global int *status,__global float *corners,
    __global float *sides,__global int *order,int n,int w,int h,int has_valid,
    int x0,int y0,int x1,int y1) {
    int s=get_global_id(0);if(s>=n)return;
    for(int k=0;k<4*12;++k)sides[s*48+k]=0;
    status[s]=0;
    int l=pairs[s*4],r=pairs[s*4+1],t=pairs[s*4+2],b=pairs[s*4+3];
    if(search_midpoint(lines+l*12).x>search_midpoint(lines+r*12).x){int tmp=l;l=r;r=tmp;}
    if(search_midpoint(lines+t*12).y>search_midpoint(lines+b*12).y){int tmp=t;t=b;b=tmp;}
    __global const float *lp=lines+l*12,*rp=lines+r*12,*tp=lines+t*12,*bp=lines+b*12;
    if(lp[0]*rp[0]+lp[1]*rp[1]<.90f)return;
    status[s]=1;
    if(tp[0]*bp[0]+tp[1]*bp[1]<.90f)return;
    int ids[4]={t,r,b,l};float minwidth=INFINITY,maxwidth=0;
    for(int k=0;k<4;++k){float width=lines[ids[k]*12+5];minwidth=fmin(minwidth,width);maxwidth=fmax(maxwidth,width);}
    if(maxwidth/minwidth>2.4f)return;
    status[s]=2;int ok=1;float2 q[4];
    for(int k=0;k<4;++k)q[k]=search_intersection(lines+ids[(k+3)%4]*12,lines+ids[k]*12,&ok);
    if(!ok)return;
    float area=0,minlen=INFINITY,maxlen=0,sign=0;
    for(int k=0;k<4;++k) {
        float2 a=q[k],b=q[(k+1)%4],c=q[(k+2)%4],ab=b-a,bc=c-b;
        if(a.x<x0||a.y<y0||a.x>x1-1||a.y>y1-1)return;
        float cross=ab.x*bc.y-ab.y*bc.x;
        if(cross==0||(k&&cross*sign<=0))return;sign=cross;
        area+=a.x*b.y-a.y*b.x;
        float len=length(ab);minlen=fmin(minlen,len);maxlen=fmax(maxlen,len);
    }
    if(fabs(area)*.5f<900||minlen<25||maxlen/minlen>5)return;
    status[s]=3;
    if(has_valid)for(int k=0;k<4;++k)for(int j=0;j<33;++j) {
        float2 probe=j==32?q[(k+1)%4]:q[k]+(q[(k+1)%4]-q[k])*((float)j/32);
        int x=convert_int_rte(probe.x),y=convert_int_rte(probe.y);
        if(x<0||x>=w||y<0||y>=h||!valid[y*w+x])return;
    }
    status[s]=4;
    for(int k=0;k<4;++k) {
        __global const float *p=lines+ids[k]*12;__global float *out=sides+(s*4+k)*12;
        float2 a=q[k],b=q[(k+1)%4];out[0]=a.x;out[1]=a.y;out[2]=b.x;out[3]=b.y;
        out[4]=p[0];out[5]=p[1];out[6]=p[5];out[7]=p[3];out[8]=p[4];out[9]=p[8];out[10]=p[9];out[11]=0;
        corners[s*8+k*2]=a.x;corners[s*8+k*2+1]=a.y;order[s*4+k]=ids[k];
    }
}

__kernel void search_side_support(__global const uchar *mask,__global const float *params,
    __global float *support,int w,int h,int n,__local int *bins) {
    int s=get_group_id(0),lid=get_local_id(0),group=get_local_size(0);if(s>=n)return;
    __global const float *p=params+s*12;
    float2 a=(float2)(p[0],p[1]),b=(float2)(p[2],p[3]),d=(float2)(p[4],p[5]);
    float len=length(b-a);if(lid==0)support[s]=-1;if(len<22)return;
    float t0=dot(a,d),t1=dot(b,d),low=fmin(t0,t1),high=fmax(t0,t1);
    if(fmax(fmax(p[7]-low,high-p[8]),0.0f)>fmax(22.0f,2.2f*p[6]))return;
    if(fmax(fmax(low-p[9],p[10]-high),0.0f)>fmax(65.0f,.45f*len))return;
    float margin=fmin(.1f,fmax(3.0f,p[6])/len);float2 delta=b-a;
    a+=margin*delta;b-=margin*delta;delta=b-a;len=length(delta);
    int count=max(10,(int)len),radius=max(3,min(8,(int)(p[6]*.55f)));
    float2 normal=(float2)(-delta.y,delta.x)/fmax(len,1.0f);
    for(int k=0;k<8;++k)bins[lid*8+k]=0;
    int base=count/8,extra=count%8,boundary=(base+1)*extra;
    for(int j=lid;j<count;j+=group) {
        int bin=j<boundary?j/(base+1):extra+(j-boundary)/base;
        float2 point=j==count-1?b:a+delta*((float)j/(count-1));int hit=0;
        for(int k=-radius;k<=radius;++k)hit|=occupied(mask,w,h,point+k*normal);
        bins[lid*8+bin]+=hit;
    }
    barrier(CLK_LOCAL_MEM_FENCE);
    for(int step=group/2;step;step/=2) {
        if(lid<step)for(int k=0;k<8;++k)bins[lid*8+k]+=bins[(lid+step)*8+k];
        barrier(CLK_LOCAL_MEM_FENCE);
    }
    if(lid==0) {
        int total=0;
        for(int k=0;k<8;++k) {
            total+=bins[k];
            if((k<2||k>=6)&&(float)bins[k]/(base+(k<extra))<.35f)return;
        }
        if((float)total/count>=.62f)support[s]=(float)total/count;
    }
}

__kernel void search_accept(__global const int *status,__global const float *corners,
    __global const float *support,__global const int *order,__global float *results,
    __global int *counts,int n) {
    int total=0,checked=0,color_checked=0;
    for(int s=0;s<n;++s) {
        checked+=status[s]>0;if(status[s]!=4)continue;++color_checked;
        float sum=0;int ok=1;
        for(int k=0;k<4;++k){float v=support[s*4+k];ok&=v>=0;sum+=v;}
        if(!ok||sum*.25f<.73f)continue;
        for(int k=0;k<8;++k)results[total*16+k]=corners[s*8+k];
        for(int k=0;k<4;++k){results[total*16+8+k]=support[s*4+k];results[total*16+12+k]=(float)order[s*4+k];}
        ++total;
    }
    counts[0]=total;counts[1]=checked;counts[2]=color_checked;counts[3]=color_checked*4;
}

__kernel void search_partial_joints(__global const float *lines,__global float *pairs,int nv,int nh) {
    int s=get_global_id(0),i=s/nh,j=s%nh+nv;if(s>=nv*nh)return;
    __global const float *v=lines+i*12,*h=lines+j*12;pairs[s*3+2]=0;
    int ok=1;float2 point=search_intersection(v,h,&ok);
    if(!ok||fabs(v[0]*h[0]+v[1]*h[1])>.707f)return;
    float2 vd=(float2)(v[0],v[1]),hd=(float2)(h[0],h[1]);
    float2 vo=(float2)(-v[1],v[0])*v[2],ho=(float2)(-h[1],h[0])*h[2];
    float dv=fmin(length(vo+vd*v[3]-point),length(vo+vd*v[4]-point));
    float dh=fmin(length(ho+hd*h[3]-point),length(ho+hd*h[4]-point));
    float gap=fmax(18.0f,fmin(26.0f,1.8f*fmax(v[5],h[5])));
    if(fmax(v[5],h[5])>3*fmin(v[5],h[5]))return;
    int white=dv>gap||dh>gap;if(white&&(dv>55||dh>55))return;
    pairs[s*3]=point.x;pairs[s*3+1]=point.y;pairs[s*3+2]=white?2:1;
}

__kernel void search_partial_color(__global const uchar *frame,
    __global const int *sdiv,__global const int *hdiv,
    __global const float *lines,__global float *results,int w,int h,int n,
    __local float *values,__local int *counts) {
    int s=get_group_id(0),lid=get_local_id(0),group=get_local_size(0);if(s>=n)return;
    __global const float *p=lines+s*12;
    float2 d=(float2)(p[0],p[1]),origin=(float2)(-p[1],p[0])*p[2];
    float2 a=origin+d*p[3],b=origin+d*p[4];
    int c60=0,c80=0,red80=0;
    for(int pass=0;pass<2;++pass) {
        int count=pass?80:60;
        for(int j=lid;j<128;j+=group) {
            float v=INFINITY;
            if(j<count) {
                float2 point=j==count-1?b:a+(b-a)*((float)j/(count-1));
                int x=convert_int_rte(point.x),y=convert_int_rte(point.y);
                if(x>=0&&x<w&&y>=0&&y<h) {
                    int pixel=y*w+x;
                    v=log(((float)frame[pixel*3+2]+10)/((float)frame[pixel*3+1]+10));
                    if(pass) {
                        // Only 80 queried pixels need HSV, not the whole frame.
                        int b=frame[pixel*3],g=frame[pixel*3+1],r=frame[pixel*3+2];
                        int high=max(r,max(g,b)),low=min(r,min(g,b)),diff=high-low;
                        int saturation=(diff*sdiv[high]+2048)>>12;
                        int hue=r==high?g-b:(g==high?b-r+2*diff:r-g+4*diff);
                        hue=(hue*hdiv[diff]+2048)>>12;if(hue<0)hue+=180;
                        ++c80;red80+=((hue<30||hue>143)&&saturation>35);
                    }
                    else ++c60;
                }
            }
            values[j]=v;
        }
        counts[lid*3]=pass?c80:c60;counts[lid*3+1]=red80;
        barrier(CLK_LOCAL_MEM_FENCE);
        for(int k=group/2;k;k/=2) {
            if(lid<k){counts[lid*3]+=counts[(lid+k)*3];counts[lid*3+1]+=counts[(lid+k)*3+1];}
            barrier(CLK_LOCAL_MEM_FENCE);
        }
        for(int k=2;k<=128;k*=2)for(int step=k/2;step>0;step/=2) {
            for(int j=lid;j<128;j+=group) {
                int other=j^step;if(j>=other)continue;float a=values[j],b=values[other];
                if((a>b)==((j&k)==0)){values[j]=b;values[other]=a;}
            }
            barrier(CLK_LOCAL_MEM_FENCE);
        }
        if(lid==0) {
            int m=counts[0];results[s*3+pass]=m?(values[(m-1)/2]+values[m/2])*.5f:-INFINITY;
            if(pass)results[s*3+2]=m?(float)counts[1]/m:0;
        }
        barrier(CLK_LOCAL_MEM_FENCE);
    }
}
