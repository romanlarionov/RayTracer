
#ifndef __CAMERA__
#define __CAMERA__

#include <cstdlib>

#include "vec3.hpp"

class Camera
{
public:
    Camera(vec3 origin, vec3 look_at, vec3 up, double fov, double aspect, double aperture, double focus_distance)
    {
        _fov = fov;
        _aspect = aspect;
        _aperture = aperture;
        _focus_distance = focus_distance;
        _origin = origin;

        double theta = fov * M_PI / 180;
        double half_height = tan(theta / 2.0);
        double half_width = aspect * half_height;

        _z = createUnitVector(_origin - look_at);
        _x = createUnitVector(cross(up, _z));
        _y = cross(_z, _x);

        _lower_left_corner = _origin - focus_distance * (half_width * _x + half_height * _y + _z);
        _horizontal = 2.0 * half_width * focus_distance * _x;
        _vertical = 2.0 * half_height * focus_distance * _y;
    }

    Ray getRay(double u, double v)
    {
        vec3 rd = _aperture * sampleUnitDisk() / 2.0;
        vec3 offset = _x * rd.x() + _y * rd.y();
        return Ray(_origin + offset, _lower_left_corner + u * _horizontal + v * _vertical - _origin - offset);
    }

private:
    double _fov; // based on vertical angle
    double _aspect;
    double _aperture;
    double _focus_distance;
    vec3 _origin;
    vec3 _lower_left_corner;
    vec3 _vertical;
    vec3 _horizontal;
    vec3 _x, _y, _z;

    // simulate aperture by emitting rays from disk around camera origin.
    vec3 sampleUnitDisk()
    {
        vec3 p;
        do
        {
            p = 2.0 * vec3(drand48(), drand48(), 0.0) - vec3(1.0, 1.0, 0.0);
        } while (dot(p, p) >= 1.0);

        return p;
    }
};

#endif