
#ifndef __AABB__
#define __AABB__

#include <algorithm>

#include "vec3.h"

class AABB
{
public:
    AABB() {}

    AABB(const vec3 &min, const vec3 &max) : 
        _min(min),
        _max(max)
    {
    }

    ~AABB() {}

    vec3 min() const { return _min; }
    vec3 max() const { return _max; }

    // slab method of intersection polling
    bool hit(const Ray &ray, double tmin, double tmax) const
    {
        // poles vector inverse lerp method for each dimension to test intersection:
        // x = A + t*B | A = ray start, B = ray direction
        // => t = (x - A) / B
        for (int i = 0; i < 3; ++i)
        {
            double inv_dir = 1.0 / ray.direction()[i];
            double t0 = (_min[i] - ray.origin()[i]) * inv_dir;
            double t1 = (_max[i] - ray.origin()[i]) * inv_dir;

            if (inv_dir < 0.0) // special case: ray goes in negative direction
                std::swap(t0, t1);

            tmin = (t0 > tmin) ? t0 : tmin; // only test within acceptable intersection regions.
            tmax = (t1 < tmax) ? t1 : tmax;

            // either the intersection's outside of test bounds or the ray is parallel to plane.
            if (tmax <= tmin) return false;
        }

        return true;
    }

private:
    vec3 _min;
    vec3 _max;
};

AABB surrounding_box(AABB l, AABB r)
{
    vec3 small(std::min(l.min().x(), r.min().x()),
               std::min(l.min().y(), r.min().y()),
               std::min(l.min().z(), r.min().z()));
    vec3 large(std::max(l.max().x(), r.max().x()),
               std::max(l.max().y(), r.max().y()),
               std::max(l.max().z(), r.max().z()));
    return AABB(small, large);
}

#endif