/* Exact CPU peak suppression. Inputs retain Python's sorted float64 values.
 * Build without fast math or FMA contraction to match NumPy operations. */
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

int quad_select_peaks(const double *normals, const double *offsets,
                     const int32_t *horizontal, int count, int limit,
                     double cosine_limit, int32_t *selected) {
    unsigned char *suppressed = calloc((size_t)count, 1);
    if (count && !suppressed) return -1;
    int quotas[2] = {0, 0}, accepted = 0;
    for (int i = 0; i < count; ++i) {
        int orientation = horizontal[i];
        if (suppressed[i] || quotas[orientation] >= limit / 2) continue;
        selected[accepted++] = i;
        ++quotas[orientation];
        for (int j = 0; j < count; ++j) {
            double dot = normals[2*j] * normals[2*i] + normals[2*j+1] * normals[2*i+1];
            if (fabs(dot) > cosine_limit &&
                fabs((dot < 0 ? -offsets[j] : offsets[j]) - offsets[i]) < 4.0)
                suppressed[j] = 1;
        }
        if (accepted >= limit) break;
    }
    free(suppressed);
    return accepted;
}
