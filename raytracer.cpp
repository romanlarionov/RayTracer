
#include <stdlib.h> // drand48
#include <limits> // numeric_limits
#include <vector>
#include <cmath>
#include <cstring> // memset
#include <fstream>

using namespace std;

class vec3
{
public:
	vec3()
	{
		memset(_v, 0, sizeof(double) * 3);
	}
	vec3(double x, double y, double z)
	{
		_v[0] = x;
		_v[1] = y;
		_v[2] = z;
	}
	~vec3() {}

	inline double x() const { return _v[0]; }
	inline double y() const { return _v[1]; }
	inline double z() const { return _v[2]; }
	inline double r() const { return _v[0]; }
	inline double g() const { return _v[1]; }
	inline double b() const { return _v[2]; }

	inline vec3 operator+(const vec3 &o) const { return vec3(_v[0] + o[0], _v[1] + o[1], _v[2] + o[2]); } 
	inline vec3 operator-(const vec3 &o) const { return vec3(_v[0] - o[0], _v[1] - o[1], _v[2] - o[2]); } 
	inline vec3 operator+(double s) const { return vec3(s + _v[0], s + _v[1], s + _v[2]); } 
	inline vec3 operator-(double s) const { return vec3(s - _v[0], s - _v[1], s - _v[2]); } 
	inline vec3 operator*(double s) const { return vec3(s * _v[0], s * _v[1], s * _v[2]); } 
	inline vec3 operator/(double s) const { return vec3(_v[0] / s, _v[1] / s, _v[2] / s); } 
	inline double operator[](int i) const { return _v[i]; }
	inline double& operator[](int i) { return _v[i]; } 

	inline vec3& operator+=(const vec3& o)
	{
		_v[0] += o[0]; _v[1] += o[1]; _v[2] += o[2];
		return *this;
	} 
	inline vec3& operator-=(const vec3& o)
	{
		_v[0] -= o[0]; _v[1] -= o[1]; _v[2] -= o[2];
		return *this;
	} 
	inline vec3& operator*=(const double s)
	{
		_v[0] *= s; _v[1] *= s; _v[2] *= s;
		return *this;
	} 
	inline vec3& operator/=(const double s)
	{
		_v[0] /= s; _v[1] /= s; _v[2] /= s;
		return *this;
	}

	inline double mag() const { return sqrt(_v[0] * _v[0] + _v[1] * _v[1] + _v[2] * _v[2]); }
	inline double squaredMag() const { return _v[0] * _v[0] + _v[1] * _v[1] + _v[2] * _v[2]; }
	inline void normalize()
	{
		double s = 1.0 / sqrt(_v[0] * _v[0] + _v[1] * _v[1] + _v[2] * _v[2]);
		_v[0] *= s; _v[1] *= s; _v[2] *= s;
	}

	inline ostream& operator<<(ostream &os) const
	{
		os << _v[0] << " " << _v[1] << " " << _v[2];
		return os;
	}

private:
	double _v[3];
};

vec3 operator+(double s, const vec3 &o)
{
	return vec3(s + o[0], s + o[1], s + o[2]);
}

vec3 operator-(double s, const vec3 &o)
{
	return vec3(s - o[0], s - o[1], s - o[2]);
}

vec3 operator*(double s, const vec3 &o)
{
	return vec3(s * o[0], s * o[1], s * o[2]);
}

inline vec3 createUnitVector(vec3 v)
{
	return v / v.mag();
}

inline double dot(const vec3 &v, const vec3 &o)
{
	return v[0] * o[0] + v[1] * o[1] + v[2] * o[2];
}

inline vec3 cross(const vec3 &v, const vec3 &o)
{
	return vec3(v[1] * o[2] - v[2] * o[1],
				v[2] * o[0] - v[0] * o[2],
				v[0] * o[1] - v[1] * o[0]);
}

class Ray
{
public:
	Ray() {}
	Ray(const vec3 &origin, const vec3 &direction) { A = origin; B = direction; }
	~Ray() {}

	vec3 origin() const { return A; }
	vec3 direction() const { return B; }
	vec3 pointAtParameter(double t) const { return A + t * B; }

private:
	vec3 A;
	vec3 B;
};

struct HitRecord
{
	double t;
	vec3 p;
	vec3 n;
};

class Entity
{
public:
	virtual bool hit(const Ray &ray, double tmin, double tmax, HitRecord &hit_record) const = 0;
};

class Sphere : public Entity
{
public:
	Sphere() {}
	Sphere(vec3 center, double radius) :
		_center(center),
		_radius(radius)
	{
	}

	~Sphere() {}

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
				hit_record.p = ray.pointAtParameter(hit_record.t);
				hit_record.n = (hit_record.p - _center) / _radius;
				return true;
			}

			temp = (-b + sqrt(discriminant)) / a;
			if (temp < tmax && temp > tmin)
			{
				hit_record.t = temp;
				hit_record.p = ray.pointAtParameter(hit_record.t);
				hit_record.n = (hit_record.p - _center) / _radius;
				return true;
			}
		}

		return false;
	}

private:
	vec3 _center;
	double _radius;
};

class Scene
{
public:
	vector<Entity *> entities;

	Scene() {}
	~Scene() {}

	bool hit(const Ray &ray, double tmin, double tmax, HitRecord &hit_record) const
	{
		HitRecord temp_record;
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
		return hit_anything;
	}
};

class Camera
{
public:
	Camera()
	{
    	_origin = vec3(0.0, 0.0, 0.0);
		_lower_left_corner = vec3(-2.0, -1.0, -1.0);
		_vertical = vec3(0.0, 2.0, 0.0);
		_horizontal = vec3(4.0, 0.0, 0.0);
	}
	Camera(vec3 origin, vec3 lower_left_corner, vec3 vertical, vec3 horizontal)
	{
    	_origin = origin;
		_lower_left_corner = lower_left_corner;
		_vertical = vertical;
		_horizontal = horizontal;
	}

	~Camera() {}

	Ray getRay(double u, double v) const
	{
		return Ray(_origin, _lower_left_corner + u * _horizontal + v * _vertical);
	}

private:
	vec3 _origin;
	vec3 _lower_left_corner;
	vec3 _vertical;
	vec3 _horizontal;
};

// both assume unit normal
inline vec3 normalToTextureSpace(vec3 v)
{
	return 0.5 * (1.0 + v);
}

inline vec3 normalToWorldSpace(vec3 v)
{
	return 2.0 * v - 1.0;
}

inline vec3 lerp(const vec3 &v, const vec3 &o, double t)
{
	return (1.0 - t) * v + t * o;
}

vec3 randomVecInUnitSphere()
{
	vec3 vec;
	do
	{
		vec = 2.0 * vec3(drand48(), drand48(), drand48()) - 1.0;
	} while (vec.squaredMag() >= 1.0); // equation for sphere. tests if point is inside sphere volume.

	return vec;
}

vec3 color(const Ray &r, Scene *scene)
{
	HitRecord hit_record;
	if (scene->hit(r, 0.001, numeric_limits<double>::max(), hit_record))
	{
		vec3 random_sample = hit_record.p + hit_record.n + randomVecInUnitSphere();
		return 0.5 * color(Ray(hit_record.p, random_sample - hit_record.p), scene);
	}
	else
	{
		vec3 dir = createUnitVector(r.direction());
		double t = 0.5 * (dir.y() + 1.0);
		return lerp(vec3(1.0, 1.0, 1.0), vec3(0.5, 0.7, 1.0), t);
	}
}

int main()
{
	// settings
	bool use_antialiasing = true;
	bool use_gamma_correction = true;
	int nx = 800;
	int ny = 400;
	int ns = 100;

	// output to ppm file format
	ofstream image_file("image.ppm");
	image_file << "P3\n" << nx << " " << ny << "\n255\n";

	// setup objects and scene
    Sphere *sphere = new Sphere(vec3(0.0, 0.0, -1.0), 0.5);
    Sphere *sphere2 = new Sphere(vec3(0.0, -100.5, -1.0), 100);

	Scene *scene = new Scene();
	scene->entities.push_back(sphere);
	scene->entities.push_back(sphere2);

	Camera camera;

	// main loop	
	for (int j = ny - 1; j >= 0; --j)
	{
		for (int i = 0; i < nx; ++i)
		{
			vec3 col(0.0, 0.0, 0.0);

			if (use_antialiasing)
			{
				for (int s = 0; s < ns; ++s)
				{
					double u = double(i) / double(nx);
					double v = double(j) / double(ny);
					col += color(camera.getRay(u, v), scene);
				}

				col /= ns;
			}
			else
			{
				double u = double(i) / double(nx);
				double v = double(j) / double(ny);
				col = color(camera.getRay(u, v), scene);
			}

			if (use_gamma_correction)
				col = vec3(sqrt(col.x()), sqrt(col.y()), sqrt(col.z())); // gamma2 correction (raise to power of 1/2)

			int ir = int(255.99 * col[0]);
			int ig = int(255.99 * col[1]);
			int ib = int(255.99 * col[2]);

			image_file << ir << " " << ig << " " << ib << "\n";
		}
	}

	image_file.close();
	return 0;
}