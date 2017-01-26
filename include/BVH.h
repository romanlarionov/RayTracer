
#ifndef __BVH__
#define __BVH__

#include <cstdlib>

#include "vec3.h"
#include "Entity.h"

int box_x_compare(const void *l, const void *r)
{
    AABB box_left, box_right;
    Entity *left = *const_cast<Entity **>(static_cast<const Entity * const *>(l));
    Entity *right = *const_cast<Entity **>(static_cast<const Entity * const *>(r));
    if (!left->bounding_box(box_left) || !right->bounding_box(box_right))
        std::cerr << "no bounding box in bvh comparison\n";

    if (box_left.min().x() - box_right.min().x() < 0.0)
        return -1;

    return 1;
}

int box_y_compare(const void *l, const void *r)
{
    AABB box_left, box_right;
    Entity *left = *const_cast<Entity **>(static_cast<const Entity * const *>(l));
    Entity *right = *const_cast<Entity **>(static_cast<const Entity * const *>(r));
    if (!left->bounding_box(box_left) || !right->bounding_box(box_right))
        std::cerr << "no bounding box in bvh comparison\n";

    if (box_left.min().y() - box_right.min().y() < 0.0)
        return -1;

    return 1;
}

int box_z_compare(const void *l, const void *r)
{
    AABB box_left, box_right;
    Entity *left = *const_cast<Entity **>(static_cast<const Entity * const *>(l));
    Entity *right = *const_cast<Entity **>(static_cast<const Entity * const *>(r));
    if (!left->bounding_box(box_left) || !right->bounding_box(box_right))
        std::cerr << "no bounding box in bvh comparison\n";

    if (box_left.min().z() - box_right.min().z() < 0.0)
        return -1;

    return 1;
}

class BVHNode : public Entity
{
public:
    BVHNode() {}

    BVHNode(Entity **entities, int n)
    {
        int axis = int(3 * drand48());

        if (axis == 0)
            qsort(entities, n, sizeof(Entity *), box_x_compare);
        else if (axis == 1)
            qsort(entities, n, sizeof(Entity *), box_y_compare);
        else
            qsort(entities, n, sizeof(Entity *), box_z_compare);

        if (n == 1) // two leaf nodes. avoids dealing with null pointers.
           { _left = _right = entities[0]; }
        else if (n == 2)
        {
            _left = entities[0];
            _right = entities[1];
        }
        else
        {
            _left = new BVHNode(entities, n / 2);
            _right = new BVHNode(entities + n / 2, n - n/2);
        }

        AABB box_left, box_right;
        if (!_left->bounding_box(box_left) || !_right->bounding_box(box_right))
            std::cerr << "no bounding box in bvh constructor\n";

        _box = surrounding_box(box_left, box_right);
    }

    virtual ~BVHNode()
    {
        if (_left) delete _left;
        if (_right) delete _right; // todo: causes seg fault
    }

    virtual bool hit(const Ray &ray, double tmin, double tmax, HitRecord &hit_record) const
    {
        // checks if anything contained within the bounding box is hit
        if (_box.hit(ray, tmin, tmax))
        {
            HitRecord left_record = {};
            HitRecord right_record = {};
            bool left_success = _left->hit(ray, tmin, tmax, left_record);
            bool right_success = _right->hit(ray, tmin, tmax, right_record);

            if (left_success && right_success)
            {
                if (left_record.t < right_record.t)
                    hit_record = left_record;
                else
                    hit_record = right_record;

                return true;
            }
            else if (left_success)
            {
                hit_record = left_record;
                return true;
            }
            else if (right_success)
            {
                hit_record = right_record;
                return true;
            }

            else return false;
        }

        else return false;
    }

    virtual bool bounding_box(AABB &box) const
    {
        box = _box;
        return true;
    }

private:
    Entity *_left;
    Entity *_right;
    AABB _box;
};

#endif
