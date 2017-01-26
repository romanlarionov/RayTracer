
#ifndef __ENTITY__
#define __ENTITY__

#include "vec3.h"
#include "Material.h"
#include "AABB.h"

class Entity
{
public:
    virtual ~Entity() {}
    virtual bool hit(const Ray &ray, double tmin, double tmax, HitRecord &hit_record) const = 0;
    virtual bool bounding_box(AABB &box) const = 0;
};

class Sphere : public Entity
{
public:
    Sphere() {}
    
    Sphere(vec3 center, double radius, Material *material) :
        _center(center),
        _radius(radius),
        _material(material)
    {
    }

    virtual ~Sphere()
    {
        if (_material) delete _material;
    }

    // geometric equation for sphere: (x - cx)^2 + (y - cy)^2 + (z - cz)^2 = R^2
    // vector form: dot(origin + t*direction - sphere_center, origin + t*direction - sphere_center) = R^2
    // => dot(A + t*B - C, A + t*B - C) - R^2 = 0
    // => dot(B,B)*t^2 + 2*dot(B,A - C)*t + dot(A - C, A - C) - R^2 = 0
    // solve via quadratic equation for value under sqrt.
    virtual bool hit(const Ray &ray, double tmin, double tmax, HitRecord &hit_record) const
    {
        vec3 oc = ray.origin() - _center;
        double a = dot(ray.direction(), ray.direction());
        double b = dot(ray.direction(), oc);
        double c = dot(oc, oc) - _radius * _radius;
        double discriminant = b * b - a * c;
        if (discriminant > 0) // no complex numbers
        {
            // test all possible roots
            double temp = (-b - sqrt(discriminant)) / a;
            if (temp < tmax && temp > tmin)
            {
                hit_record.t = temp;
                hit_record.position = ray.pointAtParameter(hit_record.t);
                hit_record.normal = (hit_record.position - _center) / _radius;
                hit_record.material = _material;
                return true;
            }

            temp = (-b + sqrt(discriminant)) / a;
            if (temp < tmax && temp > tmin)
            {
                hit_record.t = temp;
                hit_record.position = ray.pointAtParameter(hit_record.t);
                hit_record.normal = (hit_record.position - _center) / _radius;
                hit_record.material = _material;
                return true;
            }
        }

        return false;
    }

    virtual bool bounding_box(AABB &box) const
    {
        box = AABB(_center - vec3(_radius), _center + vec3(_radius));
        return true;
    }

private:
    vec3 _center;
    double _radius;
    Material *_material;
};

#endif