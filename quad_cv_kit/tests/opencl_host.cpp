// Test-only host execution of the ACTUAL OpenCL source compiled with
// -D__kernel=. This checks arithmetic and barriers without claiming GPU tests.
#include <algorithm>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

extern "C" float cosf(float), sinf(float), logf(float), fabsf(float), ceilf(float), floorf(float),
    sqrtf(float), powf(float,float), nearbyintf(float), atan2f(float,float), fmaxf(float,float), fminf(float,float);
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
extern "C" float ocl_sqrt(float) SYMBOL("_Z4sqrtf");
extern "C" float ocl_pow(float,float) SYMBOL("_Z3powff");
float ocl_sqrt(float x){return sqrtf(x);} float ocl_pow(float x,float y){return powf(x,y);}
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
extern "C" int global_add_i(volatile int*,int) SYMBOL("_Z10atomic_addPU8CLglobalVii");
extern "C" int global_min_i(volatile int*,int) SYMBOL("_Z10atomic_minPU8CLglobalVii");
extern "C" int global_max_i(volatile int*,int) SYMBOL("_Z10atomic_maxPU8CLglobalVii");
int global_add_i(volatile int *p,int n){return __atomic_fetch_add(p,n,__ATOMIC_RELAXED);}
int global_min_i(volatile int *p,int n){int old=__atomic_load_n(p,__ATOMIC_RELAXED);while(n<old&&!__atomic_compare_exchange_n(p,&old,n,false,__ATOMIC_RELAXED,__ATOMIC_RELAXED)){}return old;}
int global_max_i(volatile int *p,int n){int old=__atomic_load_n(p,__ATOMIC_RELAXED);while(n>old&&!__atomic_compare_exchange_n(p,&old,n,false,__ATOMIC_RELAXED,__ATOMIC_RELAXED)){}return old;}
extern "C" int local_add_i(volatile int*,int) SYMBOL("_Z10atomic_addPU7CLlocalVii");
extern "C" int local_min_i(volatile int*,int) SYMBOL("_Z10atomic_minPU7CLlocalVii");
extern "C" int local_max_i(volatile int*,int) SYMBOL("_Z10atomic_maxPU7CLlocalVii");
extern "C" int local_cmp_i(volatile int*,int,int) SYMBOL("_Z14atomic_cmpxchgPU7CLlocalViii");
extern "C" int global_cmp_i(volatile int*,int,int) SYMBOL("_Z14atomic_cmpxchgPU8CLglobalViii");
int local_add_i(volatile int *p,int n){return __atomic_fetch_add(p,n,__ATOMIC_RELAXED);}
int local_min_i(volatile int *p,int n){return global_min_i(p,n);}
int local_max_i(volatile int *p,int n){return global_max_i(p,n);}
int local_cmp_i(volatile int *p,int expected,int value){__atomic_compare_exchange_n(p,&expected,value,false,__ATOMIC_RELAXED,__ATOMIC_RELAXED);return expected;}
int global_cmp_i(volatile int *p,int expected,int value){__atomic_compare_exchange_n(p,&expected,value,false,__ATOMIC_RELAXED,__ATOMIC_RELAXED);return expected;}


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
void rg_target(void*,void*,void*,void*,int,int,int,int,int,int,int,float,float,int,int);
void rg_canvas(void*,void*,void*,void*,void*,void*,int,int,int,int,int,int,int);
void rg_masks(void*,void*,void*,void*,void*,void*,void*,void*,void*);
void rg_cc_init(void*,void*,void*,int,void*);
void rg_cc_link(void*,void*,int,int,void*);
void rg_cc_stats(void*,void*,void*,int,int,void*,void*);
void rg_cc_filter(void*,void*,void*,void*,int,int,int,int,void*);
void rg_peak_top(void*,void*,void*,void*,int,int,void*,void*);
void rg_peak_select(void*,void*,void*,void*,void*,void*,int,void*);
void rg_peak_prepare(void*,void*,void*,void*,void*,int);
void rg_peak_order(void*,void*,void*,void*);
void rg_rotate_bgr(void*,void*,int);
void rg_white_open2(void*,void*,void*);
void rg_hough_vote(void*,void*,void*,void*,void*,int,int,void*);
void rg_runs(void*,void*,void*,void*,void*,void*,int,int,int,int,void*);
void rg_fit(void*,void*,void*,void*,void*,int,int,void*);
void rg_rank(void*,void*,int,int,int,void*,void*);
void rg_rank_tiles(void*,void*,void*,int,int,void*,void*,void*);
void rg_rank_merge(void*,void*,void*,void*,int,int);
void rg_rank_gather(void*,void*,void*,int);
void rg_test_peak_suppression(void*,void*,void*,int);
void morph3_fused(void*,void*,int,int,int,int,void*,void*);
void rg_cc_tile(void*,void*,void*,int,int,void*,void*);
void rg_cc_boundary(void*,void*,int,int,void*);
void rg_cc_stats_hash(void*,void*,void*,int,int,void*,void*);
void rg_peak_top_tiled(void*,void*,void*,void*,int,int,void*,void*);
void rg_clear(void*,int);
void rg_merge(void*,void*,void*,int,void*);
void rg_line_validate(void*,void*,void*,void*,void*,void*,int,void*,void*);
void rg_pairs(void*,int);
void rg_complete(void*,void*,void*,void*,void*,void*,void*,int);
void rg_joints(void*,void*,void*);
void rg_partial(void*,void*,void*,void*,void*,int);
void rg_gray_down(void*,void*,int,int);
void rg_corners(void*,void*,void*,void*,void*,void*);
void rg_feature_rank(void*,void*,int);
void rg_lk(void*,void*,void*,void*,void*,void*,void*,void*,void*,void*,void*);
void rg_ransac(void*,void*,void*,void*);
void rg_motion(void*,void*,void*,void*,void*,void*,void*,int,void*,void*);
void rg_track_validate(void*,void*,void*,void*);
void rg_select(void*,void*,void*,void*,int,void*,void*);
void rg_pose(void*,void*,void*,float,int,int,int);
}
// Resident pipeline declarations are generated from its kernel signatures.
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
    case 57:rg_target(P(0),P(1),P(2),P(3),I(4),I(5),I(6),I(7),I(8),I(9),I(10),F(11),F(12),I(13),I(14));break;
    case 58:rg_canvas(P(0),P(1),P(2),P(3),P(4),P(5),I(6),I(7),I(8),I(9),I(10),I(11),I(12));break;
    case 59:rg_masks(P(0),P(1),P(2),P(3),P(4),P(5),P(6),P(7),P(8));break;
    case 60:rg_cc_init(P(0),P(1),P(2),I(3),P(4));break;
    case 61:rg_cc_link(P(0),P(1),I(2),I(3),P(4));break;
    case 62:rg_cc_stats(P(0),P(1),P(2),I(3),I(4),P(5),P(6));break;
    case 63:rg_cc_filter(P(0),P(1),P(2),P(3),I(4),I(5),I(6),I(7),P(8));break;
    case 64:rg_peak_top(P(0),P(1),P(2),P(3),I(4),I(5),P(6),P(7));break;
    case 65:rg_peak_select(P(0),P(1),P(2),P(3),P(4),P(5),I(6),P(7));break;
    case 66:rg_hough_vote(P(0),P(1),P(2),P(3),P(4),I(5),I(6),P(7));break;
    case 67:rg_runs(P(0),P(1),P(2),P(3),P(4),P(5),I(6),I(7),I(8),I(9),P(10));break;
    case 68:rg_fit(P(0),P(1),P(2),P(3),P(4),I(5),I(6),P(7));break;
    case 69:rg_rank(P(0),P(1),I(2),I(3),I(4),P(5),P(6));break;
    case 70:rg_clear(P(0),I(1));break;
    case 71:rg_merge(P(0),P(1),P(2),I(3),P(4));break;
    case 72:rg_line_validate(P(0),P(1),P(2),P(3),P(4),P(5),I(6),P(7),P(8));break;
    case 73:rg_pairs(P(0),I(1));break;
    case 74:rg_complete(P(0),P(1),P(2),P(3),P(4),P(5),P(6),I(7));break;
    case 75:rg_joints(P(0),P(1),P(2));break;
    case 76:rg_partial(P(0),P(1),P(2),P(3),P(4),I(5));break;
    case 77:rg_gray_down(P(0),P(1),I(2),I(3));break;
    case 78:rg_corners(P(0),P(1),P(2),P(3),P(4),P(5));break;
    case 79:rg_feature_rank(P(0),P(1),I(2));break;
    case 80:rg_lk(P(0),P(1),P(2),P(3),P(4),P(5),P(6),P(7),P(8),P(9),P(10));break;
    case 81:rg_ransac(P(0),P(1),P(2),P(3));break;
    case 82:rg_motion(P(0),P(1),P(2),P(3),P(4),P(5),P(6),I(7),P(8),P(9));break;
    case 83:rg_track_validate(P(0),P(1),P(2),P(3));break;
    case 84:rg_select(P(0),P(1),P(2),P(3),I(4),P(5),P(6));break;
    case 85:rg_pose(P(0),P(1),P(2),F(3),I(4),I(5),I(6));break;
    case 86:rg_rotate_bgr(P(0),P(1),I(2));break;
    case 87:rg_white_open2(P(0),P(1),P(2));break;
    case 88:rg_peak_prepare(P(0),P(1),P(2),P(3),P(4),I(5));break;
    case 89:rg_peak_order(P(0),P(1),P(2),P(3));break;
    case 90:rg_rank_tiles(P(0),P(1),P(2),I(3),I(4),P(5),P(6),P(7));break;
    case 91:rg_rank_merge(P(0),P(1),P(2),P(3),I(4),I(5));break;
    case 92:rg_rank_gather(P(0),P(1),P(2),I(3));break;
    case 93:rg_test_peak_suppression(P(0),P(1),P(2),I(3));break;
    case 94:morph3_fused(P(0),P(1),I(2),I(3),I(4),I(5),P(6),P(7));break;
    case 95:rg_cc_tile(P(0),P(1),P(2),I(3),I(4),P(5),P(6));break;
    case 96:rg_cc_boundary(P(0),P(1),I(2),I(3),P(4));break;
    case 97:rg_cc_stats_hash(P(0),P(1),P(2),I(3),I(4),P(5),P(6));break;
    case 98:rg_peak_top_tiled(P(0),P(1),P(2),P(3),I(4),I(5),P(6),P(7));break;
    }
}
extern "C" int launch(const char *name,size_t size,size_t local,Arg *args) {
    const char *names[]={"gaussian_axis","local_contrast","tube_threshold","morph3","combine_masks",
        "log_signal","strict_red","histogram_value","contrast_value","sharpen_value","remap_bgr",
        "compact_foreground","hough_vote","hough_peaks","hough_runs","fit_sections","fit_moments","fit_statistics","hough_select","gaussian_tiled","hough_select_parallel","area_down4","linear_up4","bgr_hsv","median_parameters","contrast_device","sharpen_bgr","trim_probe","trim_morph","trim_extract","side_support","compact_foreground_fast","hough_runs_fast","trim_sections","trim_extract_fast","hough_vote_compacted","hough_peaks_compacted",
        "search_merge","search_probe","search_runs","search_contrast","search_quads","search_accept",
        "search_partial_joints","search_partial_color","search_side_support",
        "resident_copy","resident_roi","resident_restrict","resident_and","resident_canvas","bgr_color","track_occupancy","gaussian_small_fused","gaussian_local","linear_up4_local","sharpen_only","rg_target","rg_canvas","rg_masks","rg_cc_init","rg_cc_link","rg_cc_stats","rg_cc_filter","rg_peak_top","rg_peak_select","rg_hough_vote","rg_runs","rg_fit","rg_rank","rg_clear","rg_merge","rg_line_validate","rg_pairs","rg_complete","rg_joints","rg_partial","rg_gray_down","rg_corners","rg_feature_rank","rg_lk","rg_ransac","rg_motion","rg_track_validate","rg_select","rg_pose","rg_rotate_bgr","rg_white_open2","rg_peak_prepare","rg_peak_order","rg_rank_tiles","rg_rank_merge","rg_rank_gather","rg_test_peak_suppression","morph3_fused","rg_cc_tile","rg_cc_boundary","rg_cc_stats_hash","rg_peak_top_tiled"};
    int kernel=-1;
    for(size_t i=0;i<sizeof(names)/sizeof(names[0]);++i)if(std::strcmp(names[i],name)==0){kernel=(int)i;break;}
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
