// Test-only host execution of the ACTUAL OpenCL source compiled with
// -D__kernel=. This checks arithmetic and barriers without claiming GPU tests.
#include <algorithm>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

extern "C" float cosf(float), sinf(float), logf(float), fabsf(float), ceilf(float), floorf(float),
    sqrtf(float), nearbyintf(float), atan2f(float,float), fmaxf(float,float), fminf(float,float);
using uint=unsigned int;
using uchar=unsigned char;
typedef float float2 __attribute__((ext_vector_type(2)));
typedef int int2 __attribute__((ext_vector_type(2)));

class Barrier {
    std::mutex mutex;
    std::condition_variable condition;
    unsigned waiting=0,generation=0,total;
public:
    explicit Barrier(unsigned n):total(n){}
    void wait() {
        std::unique_lock<std::mutex> lock(mutex);
        unsigned before=generation;
        if(++waiting==total){waiting=0;++generation;condition.notify_all();}
        else condition.wait(lock,[&]{return generation!=before;});
    }
};
thread_local size_t global_id=0,local_id=0,group_id=0,group_size=1;
thread_local Barrier *group_barrier=nullptr;
size_t get_global_id(uint){return global_id;}
size_t get_local_id(uint){return local_id;}
size_t get_group_id(uint){return group_id;}
size_t get_local_size(uint){return group_size;}
void barrier(uint){group_barrier->wait();}
#ifdef __APPLE__
#define SYMBOL(name) asm("_" name)
#else
#define SYMBOL(name) asm(name)
#endif
extern "C" float ocl_cos(float) SYMBOL("_Z3cosf");
extern "C" float ocl_sin(float) SYMBOL("_Z3sinf");
extern "C" float ocl_log(float) SYMBOL("_Z3logf");
extern "C" float ocl_fabs(float) SYMBOL("_Z4fabsf");
extern "C" float ocl_ceil(float) SYMBOL("_Z4ceilf");
extern "C" float ocl_floor(float) SYMBOL("_Z5floorf");
extern "C" float ocl_atan2(float,float) SYMBOL("_Z5atan2ff");
extern "C" float ocl_fmin(float,float) SYMBOL("_Z4fminff");
extern "C" float ocl_fmax(float,float) SYMBOL("_Z4fmaxff");
extern "C" int ocl_abs(int) SYMBOL("_Z3absi");
extern "C" int ocl_isfinite(float) SYMBOL("_Z8isfinitef");
float ocl_cos(float x){return cosf(x);} float ocl_sin(float x){return sinf(x);}
float ocl_log(float x){return logf(x);} float ocl_fabs(float x){return fabsf(x);}
float ocl_ceil(float x){return ceilf(x);} float ocl_floor(float x){return floorf(x);}
float ocl_atan2(float y,float x){return atan2f(y,x);}
float ocl_fmin(float a,float b){return fminf(a,b);} float ocl_fmax(float a,float b){return fmaxf(a,b);}
int ocl_abs(int x){return x<0?-x:x;}
int ocl_isfinite(float x){return __builtin_isfinite(x);}
float clamp(float x,float a,float b){return fminf(fmaxf(x,a),b);}
int min(int a,int b){return std::min(a,b);} int max(int a,int b){return std::max(a,b);}
float dot(float2 a,float2 b){return a.x*b.x+a.y*b.y;}
float length(float2 x){return sqrtf(dot(x,x));}
float2 convert_float2(int2 x){return (float2){(float)x.x,(float)x.y};}
int convert_int_rte(float x){return (int)nearbyintf(x);}
int convert_int_rtp(float x){return (int)ceilf(x);}
uchar convert_uchar_sat_rte(float x){return (uchar)std::min(255,std::max(0,(int)nearbyintf(x)));}
extern "C" uint global_inc(volatile uint*) SYMBOL("_Z10atomic_incPU8CLglobalVj");
extern "C" uint local_inc(volatile uint*) SYMBOL("_Z10atomic_incPU7CLlocalVj");
extern "C" int local_inc_i(volatile int*) SYMBOL("_Z10atomic_incPU7CLlocalVi");
extern "C" uint global_add(volatile uint*,uint) SYMBOL("_Z10atomic_addPU8CLglobalVjj");
extern "C" uint global_and(volatile uint*,uint) SYMBOL("_Z10atomic_andPU8CLglobalVjj");
uint global_inc(volatile uint *p){return __atomic_fetch_add(p,1,__ATOMIC_RELAXED);}
uint local_inc(volatile uint *p){return __atomic_fetch_add(p,1,__ATOMIC_RELAXED);}
int local_inc_i(volatile int *p){return __atomic_fetch_add(p,1,__ATOMIC_RELAXED);}
uint global_add(volatile uint *p,uint n){return __atomic_fetch_add(p,n,__ATOMIC_RELAXED);}
uint global_and(volatile uint *p,uint n){return __atomic_fetch_and(p,n,__ATOMIC_RELAXED);}

extern "C" {
void gaussian_axis(void*,void*,void*,int,int,int,int);
void gaussian_tiled(void*,void*,void*,int,int,int,int,void*,void*);
void area_down4(void*,void*,int,int,int,int);
void linear_up4(void*,void*,int,int,int,int);
void bgr_hsv(void*,void*,void*,void*,int);
void median_parameters(void*,void*,float);
void contrast_device(void*,void*,void*,void*,void*,int);
void sharpen_bgr(void*,void*,void*,void*,void*,void*,int,float,float);
void trim_probe(void*,void*,void*,int,int,int,int);
void trim_morph(void*,void*,void*,int,int,int,int);
void trim_extract(void*,void*,void*,void*,void*,void*,void*,int,int,int,int,int);
void trim_sections(void*,void*,void*,void*,void*,int,int,int,int);
void trim_extract_fast(void*,void*,void*,void*,void*,void*,void*,int,int,int);
void side_support(void*,void*,void*,int,int,int);
void local_contrast(void*,void*,void*,int,int);
void tube_threshold(void*,void*,void*,void*,void*,int);
void morph3(void*,void*,int,int,int);
void combine_masks(void*,void*,void*,void*,int);
void log_signal(void*,void*,int);
void strict_red(void*,void*,void*,void*,int);
void histogram_value(void*,void*,void*,int,void*);
void contrast_value(void*,void*,void*,void*,void*,int,float,float,float,int);
void sharpen_value(void*,void*,void*,void*,int,float,float);
void remap_bgr(void*,void*,void*,void*,int,int);
void compact_foreground(void*,void*,void*,int,int);
void compact_foreground_fast(void*,void*,void*,int,int);
void hough_vote(void*,int,void*,void*,int,int,void*);
void hough_vote_compacted(void*,void*,void*,void*,int,int,void*);
void hough_peaks(void*,void*,void*,int,int,int);
void hough_peaks_compacted(void*,void*,void*,int,int,int);
void hough_runs(void*,void*,void*,void*,void*,int,int,int,int,int,int,int);
void hough_runs_fast(void*,void*,void*,void*,void*,int,int,int,int,int,int,int,int,void*);
void hough_select(void*,void*,void*,void*,void*,int,int,int,int,int);
void hough_select_parallel(void*,void*,void*,void*,void*,int,int,int,int,int,void*);
void fit_sections(void*,void*,void*,void*,void*,int,int,int,int);
void fit_moments(void*,void*,void*,void*,void*,void*,void*,void*,int,int,int,float);
void fit_statistics(void*,void*,void*,void*,int,void*,void*);
void search_merge(void*,void*,void*,int,int,void*);
void search_probe(void*,void*,void*,int,int,int,int);
void search_runs(void*,void*,void*,void*,void*,void*,int,int,int,int,int);
void search_contrast(void*,void*,void*,void*,int,int,int,int,void*,void*);
void search_quads(void*,void*,void*,void*,void*,void*,void*,int,int,int,int,int,int,int,int);
void search_accept(void*,void*,void*,void*,void*,void*,int);
void search_partial_joints(void*,void*,int,int);
void search_partial_color(void*,void*,void*,void*,void*,int,int,int,void*,void*);
void search_side_support(void*,void*,void*,int,int,int,void*);
void resident_copy(void*,void*,int);
void resident_roi(void*,void*,int,int,int,int,int,int,int,int,int,int);
void resident_restrict(void*,void*,void*,int,int,int,int,int,int,int);
void resident_and(void*,void*,void*,int);
void resident_canvas(void*,void*,int,int,int,int,int,int);
void bgr_color(void*,void*,void*,void*,void*,void*,int);
void track_occupancy(void*,void*,void*,int,int,int,void*);
void gaussian_small_fused(void*,void*,void*,int,int,int,void*,void*,void*);
void gaussian_local(void*,void*,void*,void*,int,int,int,int,void*,void*);
void linear_up4_local(void*,void*,void*,int,int,int,int,int);
void sharpen_only(void*,void*,void*,void*,int,int,int,float,void*,void*,void*,void*,void*);
}
struct Arg {void *pointer;int integer;float real;};
#define P(i) args[i].pointer
#define I(i) args[i].integer
#define F(i) args[i].real
void invoke(int kernel,Arg *args) {
    switch(kernel) {
    case 0:gaussian_axis(P(0),P(1),P(2),I(3),I(4),I(5),I(6));break;
    case 1:local_contrast(P(0),P(1),P(2),I(3),I(4));break;
    case 2:tube_threshold(P(0),P(1),P(2),P(3),P(4),I(5));break;
    case 3:morph3(P(0),P(1),I(2),I(3),I(4));break;
    case 4:combine_masks(P(0),P(1),P(2),P(3),I(4));break;
    case 5:log_signal(P(0),P(1),I(2));break;
    case 6:strict_red(P(0),P(1),P(2),P(3),I(4));break;
    case 7:histogram_value(P(0),P(1),P(2),I(3),P(4));break;
    case 8:contrast_value(P(0),P(1),P(2),P(3),P(4),I(5),F(6),F(7),F(8),I(9));break;
    case 9:sharpen_value(P(0),P(1),P(2),P(3),I(4),F(5),F(6));break;
    case 10:remap_bgr(P(0),P(1),P(2),P(3),I(4),I(5));break;
    case 11:compact_foreground(P(0),P(1),P(2),I(3),I(4));break;
    case 12:hough_vote(P(0),I(1),P(2),P(3),I(4),I(5),P(6));break;
    case 13:hough_peaks(P(0),P(1),P(2),I(3),I(4),I(5));break;
    case 14:hough_runs(P(0),P(1),P(2),P(3),P(4),I(5),I(6),I(7),I(8),I(9),I(10),I(11));break;
    case 15:fit_sections(P(0),P(1),P(2),P(3),P(4),I(5),I(6),I(7),I(8));break;
    case 16:fit_moments(P(0),P(1),P(2),P(3),P(4),P(5),P(6),P(7),I(8),I(9),I(10),F(11));break;
    case 17:fit_statistics(P(0),P(1),P(2),P(3),I(4),P(5),P(6));break;
    case 18:hough_select(P(0),P(1),P(2),P(3),P(4),I(5),I(6),I(7),I(8),I(9));break;
    case 19:gaussian_tiled(P(0),P(1),P(2),I(3),I(4),I(5),I(6),P(7),P(8));break;
    case 20:hough_select_parallel(P(0),P(1),P(2),P(3),P(4),I(5),I(6),I(7),I(8),I(9),P(10));break;
    case 21:area_down4(P(0),P(1),I(2),I(3),I(4),I(5));break;
    case 22:linear_up4(P(0),P(1),I(2),I(3),I(4),I(5));break;
    case 23:bgr_hsv(P(0),P(1),P(2),P(3),I(4));break;
    case 24:median_parameters(P(0),P(1),F(2));break;
    case 25:contrast_device(P(0),P(1),P(2),P(3),P(4),I(5));break;
    case 26:sharpen_bgr(P(0),P(1),P(2),P(3),P(4),P(5),I(6),F(7),F(8));break;
    case 27:trim_probe(P(0),P(1),P(2),I(3),I(4),I(5),I(6));break;
    case 28:trim_morph(P(0),P(1),P(2),I(3),I(4),I(5),I(6));break;
    case 29:trim_extract(P(0),P(1),P(2),P(3),P(4),P(5),P(6),I(7),I(8),I(9),I(10),I(11));break;
    case 30:side_support(P(0),P(1),P(2),I(3),I(4),I(5));break;
    case 31:compact_foreground_fast(P(0),P(1),P(2),I(3),I(4));break;
    case 32:hough_runs_fast(P(0),P(1),P(2),P(3),P(4),I(5),I(6),I(7),I(8),I(9),I(10),I(11),I(12),P(13));break;
    case 33:trim_sections(P(0),P(1),P(2),P(3),P(4),I(5),I(6),I(7),I(8));break;
    case 34:trim_extract_fast(P(0),P(1),P(2),P(3),P(4),P(5),P(6),I(7),I(8),I(9));break;
    case 35:hough_vote_compacted(P(0),P(1),P(2),P(3),I(4),I(5),P(6));break;
    case 36:hough_peaks_compacted(P(0),P(1),P(2),I(3),I(4),I(5));break;
    case 37:search_merge(P(0),P(1),P(2),I(3),I(4),P(5));break;
    case 38:search_probe(P(0),P(1),P(2),I(3),I(4),I(5),I(6));break;
    case 39:search_runs(P(0),P(1),P(2),P(3),P(4),P(5),I(6),I(7),I(8),I(9),I(10));break;
    case 40:search_contrast(P(0),P(1),P(2),P(3),I(4),I(5),I(6),I(7),P(8),P(9));break;
    case 41:search_quads(P(0),P(1),P(2),P(3),P(4),P(5),P(6),I(7),I(8),I(9),I(10),I(11),I(12),I(13),I(14));break;
    case 42:search_accept(P(0),P(1),P(2),P(3),P(4),P(5),I(6));break;
    case 43:search_partial_joints(P(0),P(1),I(2),I(3));break;
    case 44:search_partial_color(P(0),P(1),P(2),P(3),P(4),I(5),I(6),I(7),P(8),P(9));break;
    case 45:search_side_support(P(0),P(1),P(2),I(3),I(4),I(5),P(6));break;
    case 46:resident_copy(P(0),P(1),I(2));break;
    case 47:resident_roi(P(0),P(1),I(2),I(3),I(4),I(5),I(6),I(7),I(8),I(9),I(10),I(11));break;
    case 48:resident_restrict(P(0),P(1),P(2),I(3),I(4),I(5),I(6),I(7),I(8),I(9));break;
    case 49:resident_and(P(0),P(1),P(2),I(3));break;
    case 50:resident_canvas(P(0),P(1),I(2),I(3),I(4),I(5),I(6),I(7));break;
    case 51:bgr_color(P(0),P(1),P(2),P(3),P(4),P(5),I(6));break;
    case 52:track_occupancy(P(0),P(1),P(2),I(3),I(4),I(5),P(6));break;
    case 53:gaussian_small_fused(P(0),P(1),P(2),I(3),I(4),I(5),P(6),P(7),P(8));break;
    case 54:gaussian_local(P(0),P(1),P(2),P(3),I(4),I(5),I(6),I(7),P(8),P(9));break;
    case 55:linear_up4_local(P(0),P(1),P(2),I(3),I(4),I(5),I(6),I(7));break;
    case 56:sharpen_only(P(0),P(1),P(2),P(3),I(4),I(5),I(6),F(7),P(8),P(9),P(10),P(11),P(12));break;
    }
}
extern "C" int launch(const char *name,size_t size,size_t local,Arg *args) {
    const char *names[]={"gaussian_axis","local_contrast","tube_threshold","morph3","combine_masks",
        "log_signal","strict_red","histogram_value","contrast_value","sharpen_value","remap_bgr",
        "compact_foreground","hough_vote","hough_peaks","hough_runs","fit_sections","fit_moments","fit_statistics","hough_select","gaussian_tiled","hough_select_parallel","area_down4","linear_up4","bgr_hsv","median_parameters","contrast_device","sharpen_bgr","trim_probe","trim_morph","trim_extract","side_support","compact_foreground_fast","hough_runs_fast","trim_sections","trim_extract_fast","hough_vote_compacted","hough_peaks_compacted",
        "search_merge","search_probe","search_runs","search_contrast","search_quads","search_accept",
        "search_partial_joints","search_partial_color","search_side_support",
        "resident_copy","resident_roi","resident_restrict","resident_and","resident_canvas","bgr_color","track_occupancy","gaussian_small_fused","gaussian_local","linear_up4_local","sharpen_only"};
    int kernel=-1;
    for(int i=0;i<57;++i)if(std::strcmp(names[i],name)==0){kernel=i;break;}
    if(kernel<0)return -1;
    if(!local) {
        for(size_t i=0;i<size;++i){global_id=i;invoke(kernel,args);}
        return 0;
    }
    Barrier sync((unsigned)local);
    std::vector<std::thread> workers;
    for(size_t lid=0;lid<local;++lid)workers.emplace_back([&,lid]{
        local_id=lid;group_size=local;group_barrier=&sync;
        for(size_t group=0;group<size/local;++group) {
            group_id=group;global_id=group*local+lid;
            invoke(kernel,args);
            sync.wait(); // Shared scratch can be reused only after every lane finished.
        }
    });
    for(auto &worker:workers)worker.join();
    return 0;
}
