
#ifndef __MATERIAL__
#define __MATERIAL__

#include <cstdlib>
#include <algorithm>

#include "vec3.h"
#include "Texture.h"
#include "Utilities.h"

class Material;

struct HitRecord
{
    double t;
    double u;
    double v;
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
    virtual bool scatter(const Ray &incident, HitRecord &hit_record, vec3 &attenuation, double &pdf, Ray &scattered) = 0;

    virtual double scatteredPdf(const Ray &incident, HitRecord &hit_record, const Ray &scattered) const
    {
        return 0.0;
    }

    virtual vec3 emitted(double u, double v, const vec3 &p) const
    {
        return vec3(0.0);
    }

protected:  
    vec3 sampleUnitSphere()
    {
        vec3 vec;
        do
        {
            vec = 2.0 * vec3(getUnitRandom(), getUnitRandom(), getUnitRandom()) - 1.0;
        } while (vec.squaredMag() >= 1.0); // equation for sphere. tests if point is inside sphere volume.

        return vec;
    }
};


class Lambertian : public Material
{
public:
    Lambertian(Texture *albedo) : _albedo(albedo) {}
    virtual ~Lambertian() {}

    virtual bool scatter(const Ray &incident, HitRecord &hit_record, vec3 &attenuation, double &pdf, Ray &scattered)
    {
        vec3 target = hit_record.position + hit_record.normal + sampleUnitSphere();
        scattered = Ray(hit_record.position, target - hit_record.position);
        attenuation = _albedo->sample(hit_record.u, hit_record.v, hit_record.position);
        pdf = dot(hit_record.normal, scattered.direction()) / M_PI;
        return true;
    }

    // clamped cosine lobe distribution
    virtual double scatteredPdf(const Ray &incident, HitRecord &hit_record, const Ray &scattered) const
    {
        return std::max(dot(hit_record.normal, createUnitVector(scattered.direction())), 0.0) / M_PI;
    }

private:
    Texture *_albedo;
};


class Metallic : public Material
{
public:
    Metallic(Texture *albedo, double fuzziness)
    {
        _albedo = albedo;
        _fuzziness = (fuzziness < 1.0) ? fuzziness : 1.0;
    }

    virtual ~Metallic() {}

    virtual bool scatter(const Ray &incident, HitRecord &hit_record, vec3 &attenuation, double &pdf, Ray &scattered)
    {
        vec3 reflected = reflect(createUnitVector(incident.direction()), hit_record.normal);
        scattered = Ray(hit_record.position, reflected + _fuzziness * sampleUnitSphere());
        attenuation = _albedo->sample(0.0, 0.0, hit_record.position);
        return (dot(scattered.direction(), hit_record.normal) > 0.0); // can't reflect underneath surface
    }

private:
    Texture *_albedo;
    double _fuzziness;
};


/*class Dielectric : public Material
{
public:
    Dielectric(Texture *albedo, double refractive_index) : _albedo(albedo), _refractive_index(refractive_index) {}
    virtual ~Dielectric() {}

    virtual bool scatter(const Ray &incident, HitRecord &hit_record, vec3 &attenuation, Ray &scattered)
    {
        vec3 outward_normal;
        double refractive_index_ratio;
        attenuation = _albedo->sample(0.0, 0.0, hit_record.position); 
        vec3 refraction_direction;

        vec3 reflection_direction = reflect(createUnitVector(incident.direction()), hit_record.normal);
        double NdotL = dot(hit_record.normal, incident.direction());
        double cosine;
        double reflection_probability = 0.0;

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

        if (getUnitRandom() < reflection_probability)
            scattered = Ray(hit_record.position, reflection_direction);
        else
            scattered = Ray(hit_record.position, refraction_direction);

        return true;
    }

private:
    Texture *_albedo;
    double _refractive_index;
};*/

class DiffuseLight : public Material
{
public:
    DiffuseLight(Texture *emitted) :
        _emitted(emitted)
    {
    }

    ~DiffuseLight()
    {
    }

    virtual bool scatter(const Ray &incident, HitRecord &hit_record, vec3 &attenuation, double &pdf, Ray &scattered)
    {
        return false;
    }

    virtual vec3 emitted(double u, double v, const vec3 &p) const
    {
        return _emitted->sample(u, v, p);
    }

private:
    Texture *_emitted;
};

#endif