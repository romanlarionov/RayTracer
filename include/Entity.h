
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


void getSphereUV(const vec3 &p, double &u, double &v)
{
    double phi = atan2(p.z(), p.x());
    double theta = asin(p.y());
    u = 1 - (phi + M_PI) / (2 * M_PI);
    v = (theta + M_PI/2) / M_PI;
}

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
                getSphereUV(hit_record.normal, hit_record.u, hit_record.v);

                return true;
            }

            temp = (-b + sqrt(discriminant)) / a;
            if (temp < tmax && temp > tmin)
            {
                hit_record.t = temp;
                hit_record.position = ray.pointAtParameter(hit_record.t);
                hit_record.normal = (hit_record.position - _center) / _radius;
                hit_record.material = _material;
                getSphereUV(hit_record.normal, hit_record.u, hit_record.v);
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

class xy_Rect : public Entity
{
public:
    xy_Rect() {}

    xy_Rect(double x0, double x1, double y0, double y1, double k, vec3 normal, Material *material)
    {
        _x0 = x0; _x1 = x1; _y0 = y0; _y1 = y1;
        _k = k;
        _normal = normal;
        _material = material;
    }

    /*Rect(vec3 bottom_left, vec3 top_right, vec3 normal, Material *material) :
        _bottom_left(bottom_left),
        _top_right(top_right),
        _normal(normal.normalize()),
        _material(material)
    {
    }*/

    virtual ~xy_Rect() {}

    // https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-plane-and-ray-disk-intersection
    /*virtual bool hit(const Ray &ray, double tmin, double tmax, HitRecord &hit_record)
    {
        double VdotN = dot(createUnitVector(ray.direction()), _normal);
        if (VdotN < 0.000001) return false;

        double t = dot(_origin - ray.origin(), _normal) / VdotN;
        vec3 hit_point = ray.pointAtParameter(t);

        if (t < 0.0 || hit_point.x() < bottom_left.x()) ||
            hit_point.x() > _top_right.x() ||
            hit_point.y() < _bottom_left.y() ||
            hit_point.y() > _top_right.y() ||
            )
            return false;

        hit_record.t = t;
        hit_record.position = ray.pointAtParameter(hit_record.t);
        hit_record.normal = _normal;
        hit_record.material = _material;
        hit_record.u = (x - x0) / (x1 - x0);
        hit_record.v = (y - y0) / (y1 - y0);
        return true;
    }*/

    virtual bool hit(const Ray &ray, double tmin, double tmax, HitRecord &hit_record) const
    {
        double t = (_k - ray.origin().z()) / ray.direction().z();
        double x = ray.origin().x() + t * ray.direction().x();
        double y = ray.origin().y() + t * ray.direction().y();

        if (x < _x0 || x > _x1 || y < _y0 || y > _y1) return false;

        hit_record.t = t;
        hit_record.position = ray.pointAtParameter(hit_record.t);
        hit_record.normal = _normal;
        hit_record.material = _material;
        hit_record.u = (x - _x0) / (_x1 - _x0);
        hit_record.v = (y - _y0) / (_y1 - _y0);
        return true;
    }

    virtual bool bounding_box(AABB &box) const
    {
        box = AABB(vec3(_x0, _y0, _k - 0.0001), vec3(_x1, _y1, _k + 0.0001));
        return true;
    }

private:
    double _x0, _x1, _y0, _y1, _k;
    //double _width, _height;
    //vec3 _bottom_left;
    //vec3 _top_right;
    vec3 _normal;
    Material *_material;
};


class xz_Rect : public Entity
{
public:
    xz_Rect() {}

    xz_Rect(double x0, double x1, double z0, double z1, double k, vec3 normal, Material *material)
    {
        _x0 = x0; _x1 = x1;
        _z0 = z0; _z1 = z1;
        _k = k;
        _normal = normal;
        _material = material;
    }

    virtual ~xz_Rect() {}

    virtual bool hit(const Ray &ray, double tmin, double tmax, HitRecord &hit_record) const
    {
        double t = (_k - ray.origin().y()) / ray.direction().y();
        double x = ray.origin().x() + t * ray.direction().x();
        double z = ray.origin().z() + t * ray.direction().z();

        if (x < _x0 || x > _x1 || z < _z0 || z > _z1) return false;

        hit_record.t = t;
        hit_record.position = ray.pointAtParameter(hit_record.t);
        hit_record.normal = _normal;
        hit_record.material = _material;
        hit_record.u = (x - _x0) / (_x1 - _x0);
        hit_record.v = (z - _z0) / (_z1 - _z0);
        return true;
    }

    virtual bool bounding_box(AABB &box) const
    {
        box = AABB(vec3(_x0, _k - 0.0001, _z0), vec3(_x1, _k + 0.0001, _z1));
        return true;
    }

private:
    double _x0, _x1, _z0, _z1, _k;
    vec3 _normal;
    Material *_material;
};

class yz_Rect : public Entity
{
public:
    yz_Rect() {}

    yz_Rect(double y0, double y1, double z0, double z1, double k, vec3 normal, Material *material) :
        _k(k),
        _material(material)
    {
        _y0 = y0;
        _y1 = y1;
        _z0 = z0;
        _z1 = z1;
        _normal = normal;
    }

    virtual ~yz_Rect() {}

    virtual bool hit(const Ray &ray, double tmin, double tmax, HitRecord &hit_record) const
    {
        double t = (_k - ray.origin().x()) / ray.direction().x();
        double y = (ray.origin().y()) + t * ray.direction().y();
        double z = ray.origin().z() + t * ray.direction().z();

        if (y < _y0 || y > _y1 || z < _z0 || z > _z1) return false;

        hit_record.t = t;
        hit_record.position = ray.pointAtParameter(hit_record.t);
        hit_record.normal = _normal;
        hit_record.material = _material;
        hit_record.u = (y - _y0) / (_y1 - _y0);
        hit_record.v = (z - _z0) / (_z1 - _z0);
        return true;
    }

    virtual bool bounding_box(AABB &box) const
    {
        box = AABB(vec3(_k - 0.0001, _y0, _z0), vec3(_k + 0.0001, _y1, _z1));
        return true;
    }

private:
    double _y0, _y1, _z0, _z1, _k;
    vec3 _normal;
    Material *_material;
};

#endif