
#ifndef __MATERIAL__
#define __MATERIAL__

#include <cstdlib>

#include "vec3.h"

class Material;

struct HitRecord
{
    double t;
    vec3 position;
    vec3 normal;
    Material *material;
};

inline double schlick_fresnel(double cosine, double refractive_index)
{
    double f0 = (1.0 - refractive_index) / (1.0 + refractive_index);
    f0 = f0 * f0;
    return f0 + (1.0 - f0) * pow(1.0 - cosine, 5);
}

class Material
{
public:
    virtual ~Material() {}
    // determines how an incoming light ray interacts with a surface by affecting the scattered outgoing ray.
    virtual bool scatter(const Ray &incident, HitRecord &hit_record, vec3 &attenuation, Ray &scattered) = 0;

protected:  
    vec3 sampleUnitSphere()
    {
        vec3 vec;
        do
        {
            vec = 2.0 * vec3(drand48(), drand48(), drand48()) - 1.0;
        } while (vec.squaredMag() >= 1.0); // equation for sphere. tests if point is inside sphere volume.

        return vec;
    }
};


class Lambertian : public Material
{
public:
    Lambertian(const vec3 &albedo) : _albedo(albedo) {}
    virtual ~Lambertian() {}

    virtual bool scatter(const Ray &incident, HitRecord &hit_record, vec3 &attenuation, Ray &scattered)
    {
        vec3 target = hit_record.position + hit_record.normal + sampleUnitSphere();
        scattered = Ray(hit_record.position, target - hit_record.position);
        attenuation = _albedo;
        return true;
    }

private:
    vec3 _albedo;
};


class Metallic : public Material
{
public:
    Metallic(const vec3 &albedo, double fuzziness)
    {
        _albedo = albedo;
        _fuzziness = (fuzziness <= 1.0) ? fuzziness : 1.0;
    }

    virtual ~Metallic() {}

    virtual bool scatter(const Ray &incident, HitRecord &hit_record, vec3 &attenuation, Ray &scattered)
    {
        vec3 reflected = reflect(createUnitVector(incident.direction()), hit_record.normal);
        scattered = Ray(hit_record.position, reflected + _fuzziness * sampleUnitSphere());
        attenuation = _albedo;
        return (dot(scattered.direction(), hit_record.normal) > 0.0); // can't reflect underneath surface
    }

private:
    vec3 _albedo;
    double _fuzziness;
};


class Dielectric : public Material
{
public:
    Dielectric(const vec3 &albedo, double refractive_index) : _albedo(albedo), _refractive_index(refractive_index) {}
    virtual ~Dielectric() {}

    virtual bool scatter(const Ray &incident, HitRecord &hit_record, vec3 &attenuation, Ray &scattered)
    {
        vec3 outward_normal;
        double refractive_index_ratio;
        //attenuation = _albedo; // clear glass should be 1.0,1.0,1.0. stained glass should attenuate only certain color channels.
        attenuation = vec3(1.0, 1.0, 1.0);
        vec3 refraction_direction;

        vec3 reflection_direction = reflect(createUnitVector(incident.direction()), hit_record.normal);
        double NdotL = dot(hit_record.normal, incident.direction());
        double cosine;
        double reflection_probability = 0.0;

        // 
        if (NdotL > 0.0)
        {
            outward_normal = -hit_record.normal;
            refractive_index_ratio = _refractive_index;
            cosine = _refractive_index * NdotL / incident.direction().mag();
        }
        else
        {
            outward_normal = hit_record.normal;
            refractive_index_ratio = 1.0 / _refractive_index;
            cosine = -NdotL / incident.direction().mag();
        }

        // dielectrics can reflect and refract. reflection is based on fresnel probability.
        if (refract(incident.direction(), outward_normal, refractive_index_ratio, refraction_direction))
            reflection_probability = schlick_fresnel(cosine, _refractive_index);
        else
        {
            scattered = Ray(hit_record.position, reflection_direction);
            reflection_probability = 1.0;
        }

        if (drand48() < reflection_probability)
            scattered = Ray(hit_record.position, reflection_direction);
        else
            scattered = Ray(hit_record.position, refraction_direction);

        return true;
    }

private:
    vec3 _albedo;
    double _refractive_index;
};

#endif