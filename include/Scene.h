
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

    Scene(std::vector<Entity *> entities)
    {
        if (!entities.empty())
        {
            _bvh = new BVHNode(entities.data(), entities.size());
            _entities = entities;
        }
    }

    ~Scene()
    {
        //for (auto &e : entities)
        //    delete e;
        delete _bvh;
    }

    bool hit(const Ray &ray, double tmin, double tmax, HitRecord &hit_record) const
    {
        /*HitRecord temp_record;
        bool hit_anything = false;
        double closest = tmax;

        for (auto &e : entities)
        {
            if (e->hit(ray, tmin, closest, temp_record))
            {
                hit_anything = true;
                closest = temp_record.t;
                hit_record = temp_record;
            }
        }
        return hit_anything;*/
        return _bvh->hit(ray, tmin, tmax, hit_record);
    }

    // returns bounding box around all entities
    bool bounding_box(AABB &box) const
    {
        if (_entities.empty()) return false;

        AABB temp_box;
        if (!_entities[0]->bounding_box(temp_box)) return false;

        box = temp_box;
        for (int i = 1; i < _entities.size(); ++i)
        {
            if (_entities[0]->bounding_box(temp_box))
                box = surrounding_box(box, temp_box);
            else // breaks the assumption that every node in the bounded volumetric hierarchy is represented with an AABB.
                return false;
        }

        return true;
    }

private:
    std::vector<Entity *> _entities;
    BVHNode *_bvh;
};

#endif