
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

class EntityList : public Entity
{
public:
    EntityList() {}
    
    EntityList(std::vector<Entity *> entities) : 
        _entities(entities)
    {
    }

    virtual ~EntityList()
    {
        for (auto &e : _entities)
            delete e;
    }

    virtual bool hit(const Ray &ray, double tmin, double tmax, HitRecord &hit_record) const
    {
        HitRecord temp_record;
        bool hit_anything = false;
        double closest = tmax;

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

    virtual bool bounding_box(AABB &box) const
    {
        if (_entities.empty()) return false;

        AABB temp_box;
        if (!_entities[0]->bounding_box(temp_box)) return false;

        box = temp_box;
        for (int i = 1; i < _entities.size(); ++i)
        {
            if (_entities[0]->bounding_box(temp_box))
                box = surrounding_box(box, temp_box);
            else
                return false;
        }

        return true;
    }

private:
    std::vector<Entity *> _entities;
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

    virtual ~xy_Rect() {}

    virtual bool hit(const Ray &ray, double tmin, double tmax, HitRecord &hit_record) const
    {
        double t = (_k - ray.origin().z()) / ray.direction().z();
        if (t < tmin || t > tmax) return false;

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
        if (t < tmin || t > tmax) return false;

        double x = ray.origin().x() + t * ray.direction().x();
        double z = ray.origin().z() + t * ray.direction().z();

        if (x < _x0 || x > _x1 || z < _z0 || z > _z1) return false;

        hit_record.t = t;
        hit_record.position = ray.pointAtParameter(t);
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
        if (t < tmin || t > tmax) return false;

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

class AxisAlignedBox : public Entity
{
public:
    AxisAlignedBox(vec3 p0, vec3 p1, Material *material) :
        _p0(p0),
        _p1(p1)
    {
        std::vector<Entity *> entities;
        entities.emplace_back(new xy_Rect(p0.x(), p1.x(), p0.y(), p1.y(), p1.z(), vec3(0.0, 0.0, 1.0), material));
        entities.emplace_back(new xy_Rect(p0.x(), p1.x(), p0.y(), p1.y(), p0.z(), vec3(0.0, 0.0, -1.0), material));
        entities.emplace_back(new xz_Rect(p0.x(), p1.x(), p0.z(), p1.z(), p1.y(), vec3(0.0, 1.0, 0.0), material));
        entities.emplace_back(new xz_Rect(p0.x(), p1.x(), p0.z(), p1.z(), p0.y(), vec3(0.0, -1.0, 0.0), material));
        entities.emplace_back(new yz_Rect(p0.y(), p1.y(), p0.z(), p1.z(), p1.x(), vec3(1.0, 0.0, 0.0), material));
        entities.emplace_back(new yz_Rect(p0.y(), p1.y(), p0.z(), p1.z(), p0.x(), vec3(-1.0, 0.0, 0.0), material));
        _box_sides = new EntityList(entities);
    }

    ~AxisAlignedBox()
    {
        delete _box_sides;
    }

    virtual bool hit(const Ray &ray, double tmin, double tmax, HitRecord &hit_record) const
    {
        return _box_sides->hit(ray, tmin, tmax, hit_record);
    }

    virtual bool bounding_box(AABB &box) const
    {
        box = AABB(_p0, _p1);
        return true;
    }

private:
    vec3 _p0, _p1;
    EntityList *_box_sides;
};

#endif