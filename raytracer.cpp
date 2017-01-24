
#include <algorithm>
#include <cstring>
#include <iostream>
#include <cstdlib> // drand48
#include <limits> // numeric_limits
#include <vector>
#include <cmath>
#include <cstring> // memset
#include <fstream>
#include <thread>
#include <chrono>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

using namespace std;

///////////////////////////////////////////////////////////////////////////////////////////////////////////// Linear Algebra

class vec3
{
public:
	vec3()
	{
		memset(_v, 0, sizeof(double) * 3);
	}

	vec3(double val)
	{
		_v[0] = val;
		_v[1] = val;
		_v[2] = val;
	}

	vec3(double x, double y, double z)
	{
		_v[0] = x;
		_v[1] = y;
		_v[2] = z;
	}

	vec3(const vec3 &o)
	{
		_v[0] = o[0];
		_v[1] = o[1];
		_v[2] = o[2];
	}

	inline double x() const { return _v[0]; }
	inline double y() const { return _v[1]; }
	inline double z() const { return _v[2]; }
	inline double r() const { return _v[0]; }
	inline double g() const { return _v[1]; }
	inline double b() const { return _v[2]; }

	inline vec3 operator+(const vec3 &o) const { return vec3(_v[0] + o[0], _v[1] + o[1], _v[2] + o[2]); } 
	inline vec3 operator-(const vec3 &o) const { return vec3(_v[0] - o[0], _v[1] - o[1], _v[2] - o[2]); } 
	inline vec3 operator*(const vec3 &o) const { return vec3(_v[0] * o[0], _v[1] * o[1], _v[2] * o[2]); } 
	inline vec3 operator+(double s) const { return vec3(s + _v[0], s + _v[1], s + _v[2]); } 
	inline vec3 operator-(double s) const { return vec3(s - _v[0], s - _v[1], s - _v[2]); } 
	inline vec3 operator*(double s) const { return vec3(s * _v[0], s * _v[1], s * _v[2]); } 
	inline vec3 operator/(double s) const { return vec3(_v[0] / s, _v[1] / s, _v[2] / s); } 
	inline double operator[](int i) const { return _v[i]; }
	inline double& operator[](int i) { return _v[i]; } 
	inline vec3 operator-() const { return vec3(-_v[0], -_v[1], -_v[2]); } 

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

	vec3 origin() const { return A; }
	vec3 direction() const { return B; }
	vec3 pointAtParameter(double t) const { return A + t * B; }

private:
	vec3 A;
	vec3 B;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////// Helper Functions

unsigned char* framebufferToArray(vector<vector<vec3> > &framebuffer)
{
	int width = (int)framebuffer.size();
	int height = (int)framebuffer[0].size();

	unsigned char *output = new unsigned char[width * height * 3];
	unsigned char *output_ptr = output;

	for (int j = height - 1; j >= 0; --j)
	{
		unsigned char *temp = new unsigned char[3 * width];
		unsigned char *temp_ptr = temp;

		for (int i = 0; i < width; ++i)
		{
			int rgb[3];
			rgb[0] = (unsigned char)(255.99 * framebuffer[i][j].x());
			rgb[1] = (unsigned char)(255.99 * framebuffer[i][j].y());
			rgb[2] = (unsigned char)(255.99 * framebuffer[i][j].z());
			copy(rgb, rgb + 3, temp_ptr);
			temp_ptr += 3;
		}

		copy(temp, temp + (width * 3), output_ptr);
		output_ptr += width * 3;
		delete[] temp;
	}

	return output;
}

// all assume unit normals
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

inline vec3 reflect(const vec3 &v, const vec3 &n)
{
	return v - 2.0 * dot(v, n) * n;
}

bool refract(const vec3 &v, const vec3 &n, double refractive_index_ratio, vec3 &refraction_direction)
{
	vec3 unit_v = createUnitVector(v);
	double NdotL = dot(unit_v, n);
	double discriminant = 1.0 - refractive_index_ratio * refractive_index_ratio * (1.0 - NdotL * NdotL);
	if (discriminant > 0.0)
	{
		refraction_direction = refractive_index_ratio * (unit_v - n * NdotL) - n * sqrt(discriminant);
		return true;
	}

	return false;
}

double schlick_fresnel(double cosine, double refractive_index)
{
	double f0 = (1.0 - refractive_index) / (1.0 + refractive_index);
	f0 = f0 * f0;
	return f0 + (1.0 - f0) * pow(1.0 - cosine, 5);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////// Materials

class Material;

struct HitRecord
{
	double t;
	vec3 position;
	vec3 normal;
	Material *material;
};

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

////////////////////////////////////////////////////////////////////////////////////////////////// Scene & Entities In Scene

class Entity
{
public:
	virtual ~Entity() {}
	virtual bool hit(const Ray &ray, double tmin, double tmax, HitRecord &hit_record) const = 0;
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
		delete _material;
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

private:
	vec3 _center;
	double _radius;
	Material *_material;
};

class Scene
{
public:
	vector<Entity *> entities;

	Scene() {}
	~Scene()
	{
		for (auto &e : entities)
			delete e;
	}

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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////// Camera

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

// recursively compute the color of a pixel given a starting ray
vec3 color(const Ray &r, Scene *scene, int depth)
{
	HitRecord hit_record;
	if (scene->hit(r, 0.001, numeric_limits<double>::max(), hit_record))
	{
		Ray scattered;
		vec3 attenuation;

		// limit the call stack. only proceed when successfully scattered.
		if (depth < 50 && hit_record.material->scatter(r, hit_record, attenuation, scattered))
			return attenuation * color(scattered, scene, depth + 1);
		else
			return vec3(0.0);
	}
	else
	{
		// color the backdrop (arbitrary)
		vec3 dir = createUnitVector(r.direction());
		double t = 0.5 * (dir.y() + 1.0);
		return lerp(vec3(1.0), vec3(0.5, 0.7, 1.0), t);
	}
}

Scene* createRandomScene(int num_entities)
{
	Scene *scene = new Scene();
	scene->entities.push_back(new Sphere(vec3(0.0, -1000.0, 0.0), 1000.0, new Lambertian(vec3(0.5)) ) );

	double dim = sqrt(num_entities) / 2.0;
	for (double a = -dim; a < dim; ++a)
	for (double b = -dim; b < dim; ++b)
	{
		double curr_material = drand48();
		vec3 center(a + 0.9 * drand48(), 0.2, b + 0.9 * drand48());
		if ((center - vec3(4.0, 0.2, 0.0)).mag() > 0.9)
		{
			if (curr_material < 0.8)
				scene->entities.push_back(new Sphere(center, 0.2, new Lambertian(vec3(drand48() * drand48(), drand48() * drand48(), drand48() * drand48()))));
			else if (curr_material < 0.95)
				scene->entities.push_back(new Sphere(center, 0.2, new Metallic(0.5 * vec3(1.0 + drand48(), 1.0 + drand48(), drand48()), 0.5 * drand48())));
			else
				scene->entities.push_back(new Sphere(center, 0.2, new Dielectric(vec3(1.0), 1.5)));
		}
	}

	scene->entities.push_back(new Sphere(vec3(0.0, 1.0, 0.0), 1.0, new Dielectric(vec3(1.0), 1.5)));
	scene->entities.push_back(new Sphere(vec3(-4.0, 1.0, 0.0), 1.0, new Lambertian(vec3(0.4, 0.2, 0.1))));
	scene->entities.push_back(new Sphere(vec3(4.0, 1.0, 0.0), 1.0, new Metallic(vec3(0.7, 0.6, 0.5), 0.0)));

	return scene;
}

struct Settings
{
	bool use_antialiasing;
	bool use_gamma_correction;
	int width;
	int height;
	int num_aa_samples;
};

// threads loop through the frame in an interleaving fashion. each computes and stores RGB vec3 to global framebuffer.
void render(Scene *scene, Camera &camera, Settings &settings, int thread_id, int num_threads, vector<vector<vec3> > &framebuffer)
{
	for (int idx = thread_id; idx < settings.width * settings.height; idx += num_threads)
	{
		int i = idx % settings.width;
		int j = idx / settings.width;
		vec3 col(0.0);

		if (settings.use_antialiasing)
		{
			// apply stochastic sampling approach over evenly spaced sub pixel box regions
			double delta_u = 1.0 / double(settings.width);
			double delta_v = 1.0 / double(settings.height);
			double sub_pixel_dimension = sqrt(settings.num_aa_samples);

			for (double s_x = 0.0; s_x < sub_pixel_dimension; ++s_x)
			{
				for (double s_y = 0.0; s_y < sub_pixel_dimension; ++s_y)
				{
					double u = (double(i) / double(settings.width)) + (delta_u * drand48());// + (drand48() / delta_u);
					double v = (double(j) / double(settings.height)) + (delta_v * drand48());// + (drand48() / delta_v);
					col += color(camera.getRay(u, v), scene, 0);
				}
			}

			// could apply gaussian filter here. but choosing simple box filter (equal weighting)
			col /= settings.num_aa_samples;
		}
		else
		{
			double u = double(i) / double(settings.width);
			double v = double(j) / double(settings.height);
			col = color(camera.getRay(u, v), scene, 0);
		}

		if (settings.use_gamma_correction)
			col = vec3(sqrt(col.x()), sqrt(col.y()), sqrt(col.z())); // raise to power of 1/X, where I use X = 2

		framebuffer[i][j] = vec3(col);

		cout << double(idx) / double(settings.width * settings.height) * 100 << "%\n";
	}
}

int main(int argc, char **argv)
{
	Settings settings;
	settings.use_antialiasing = true;
	settings.use_gamma_correction = true;
	settings.width = 720;
	settings.height = 480;
	settings.num_aa_samples = 36;

	// setup entities and scene
    Scene *scene = createRandomScene(50);

	vec3 camera_center(5.5, 2.0, 1.0);
	vec3 look_at(0.0, 0.0, -1.0);
	double distance_to_focus = (camera_center - look_at).mag();
	double aperture = 0.02;

	Camera camera(camera_center, look_at, vec3(0.0, 1.0, 0.0), 90.0,
				  settings.width / settings.height, aperture, distance_to_focus);

	unsigned int num_threads = std::thread::hardware_concurrency() * 2;
	vector<thread> threads;
	vector<vector<vec3> > framebuffer(settings.width);
	for (int i = 0; i < settings.width; ++i)
		framebuffer[i].resize(settings.height);

	// begin execution
	auto start_time = std::chrono::high_resolution_clock::now();

	for (int i = 0; i < num_threads; ++i)
		threads.emplace_back(thread(render, scene, ref(camera), ref(settings), i, num_threads, ref(framebuffer)));

	for (auto &t : threads)
		t.join();

	// end execution
	auto curr_time = std::chrono::high_resolution_clock::now();
	float time = std::chrono::duration_cast<std::chrono::milliseconds>(curr_time - start_time).count() / 1000.0f;
	cout << "\n\n Execution time: " << time << " (seconds)\n";

	// output to file
	int success = stbi_write_png("image.png", settings.width, settings.height, 3, framebufferToArray(framebuffer), 3 * settings.width);

	delete scene;
}
