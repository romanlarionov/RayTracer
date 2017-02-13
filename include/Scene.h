
#ifndef __SCENE__
#define __SCENE__

#include "Material.h"
#include "BVH.h"

class Scene
{
public:
    Scene() {}

    // Assumes an iterative search through entity list with bounding box collision detection
    Scene(EntityList *entities) :
        _entities(entities),
        _use_bvh(false)
    {
    }

    // Assumes a logn search through entity list via bounding volume hierarchy
    Scene(BVHNode *bvh) :
        _bvh(bvh),
        _use_bvh(true)
    {
    }

    ~Scene()
    {
        if (_use_bvh)
            delete _bvh;
        else
            delete _entities;
    }

    bool hit(const Ray &ray, double tmin, double tmax, HitRecord &hit_record) const
    {
        if (_use_bvh)
            return _bvh->hit(ray, tmin, tmax, hit_record);

        return _entities->hit(ray, tmin, tmax, hit_record);
    }
    
private:
    EntityList *_entities;
    BVHNode *_bvh;

    bool _use_bvh;
};

#endif