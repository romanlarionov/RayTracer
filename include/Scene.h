
#ifndef __SCENE__
#define __SCENE__

#include <vector>

#include "vec3.h"
#include "Material.h"
#include "BVH.h"

class Scene
{
public:

    Scene() {}

    // Assumes an iterative search through entity list with bounding box collision detection
    Scene(std::vector<Entity *> entities)
    {
        _entities = entities;
        _use_bvh = false;
    }

    // Assumes a logn search through entity list via bounding volume hierarchy
    Scene(BVHNode *bvh)
    {
        _bvh = bvh;
        _use_bvh = true;
    }

    ~Scene()
    {
        if (_use_bvh)
            delete _bvh;
    }

    bool hit(const Ray &ray, double tmin, double tmax, HitRecord &hit_record) const
    {
        // if bvh has been provided, use it
        if (_use_bvh)
            return _bvh->hit(ray, tmin, tmax, hit_record);

        // otherwise perform iterative search
        HitRecord temp_record;
        bool hit_anything = false;
        double closest = tmax;

        //for (auto &e : _entities)
        for (size_t i = 0; i < _entities.size(); ++i)
        {
            if (_entities[i]->hit(ray, tmin, closest, temp_record))
            {
                hit_anything = true;
                closest = temp_record.t;
                hit_record = temp_record;
            }
        }
        return hit_anything;
    }

private:
    std::vector<Entity *> _entities;
    BVHNode *_bvh;

    bool _use_bvh;
};

#endif