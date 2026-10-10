// Ordered double-precision merge, identical thresholds and first-match policy.
#include <math.h>

int quad_merge_trimmed(double *rows, int n, int *indices) {
    int count=0;
    for(int i=0;i<n;++i) {
        double *p=rows+i*12;
        double ax=p[2]*p[4]+p[0]*p[5], ay=p[3]*p[4]+p[1]*p[5];
        double bx=p[2]*p[4]+p[0]*p[6], by=p[3]*p[4]+p[1]*p[6];
        double mx=(ax+bx)*.5, my=(ay+by)*.5;
        int duplicate=-1;
        for(int j=0;j<count;++j) {
            double *old=rows+indices[j]*12;
            if(old[9]!=p[9] || old[0]*p[0]+old[1]*p[1]<.995)continue;
            double distance=fabs(mx*old[2]+my*old[3]-old[4]);
            double pa=ax*old[0]+ay*old[1], pb=bx*old[0]+by*old[1];
            double gap=fmax(fmax(fmin(pa,pb)-old[6],old[5]-fmax(pa,pb)),0.);
            if(distance<fmax(2.,.45*fmin(old[7],p[7])) && gap<20.) {
                duplicate=indices[j];
                old[5]=fmin(old[5],fmin(pa,pb));old[6]=fmax(old[6],fmax(pa,pb));
                old[10]=fmin(old[10],p[10]);old[11]=fmax(old[11],p[11]);
                break;
            }
        }
        if(duplicate<0)indices[count++]=i;
    }
    return count;
}
