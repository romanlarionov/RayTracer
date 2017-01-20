
#include <cmath>
#include <cstring> // memset
#include <fstream>

using namespace std;

class vec3
{
public:
	vec3()
	{
		memset(v, 0, sizeof(double) * 3);
	}
	vec3(double x, double y, double z)
	{
		v[0] = x;
		v[1] = y;
		v[2] = z;
	}
	~vec3() {}

	inline double x() const { return v[0]; }
	inline double y() const { return v[1]; }
	inline double z() const { return v[2]; }
	inline double r() const { return v[0]; }
	inline double g() const { return v[1]; }
	inline double b() const { return v[2]; }

	inline vec3 operator+(const vec3 &o) const { return vec3(v[0] + o[0], v[1] + o[1], v[2] + o[2]); } 
	inline vec3 operator-(const vec3 &o) const { return vec3(v[0] - o[0], v[1] - o[1], v[2] - o[2]); } 
	inline vec3 operator+(double s) const { return vec3(s + v[0], s + v[1], s + v[2]); } 
	inline vec3 operator-(double s) const { return vec3(s - v[0], s - v[1], s - v[2]); } 
	inline vec3 operator*(double s) const { return vec3(s * v[0], s * v[1], s * v[2]); } 
	inline vec3 operator/(double s) const { return vec3(v[0] / s, v[1] / s, v[2] / s); } 
	inline double operator[](int i) const { return v[i]; }
	inline double& operator[](int i) { return v[i]; } 

	inline vec3& operator+=(const vec3& o)
	{
		v[0] += o[0]; v[1] += o[1]; v[2] += o[2];
		return *this;
	} 
	inline vec3& operator-=(const vec3& o)
	{
		v[0] -= o[0]; v[1] -= o[1]; v[2] -= o[2];
		return *this;
	} 
	inline vec3& operator*=(const double s)
	{
		v[0] *= s; v[1] *= s; v[2] *= s;
		return *this;
	} 
	inline vec3& operator/=(const double s)
	{
		v[0] /= s; v[1] /= s; v[2] /= s;
		return *this;
	}

	inline double mag() const { return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }
	inline double squaredMag() const { return v[0] * v[0] + v[1] * v[1] + v[2] * v[2]; }
	inline void normalize()
	{
		double s = 1.0 / sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
		v[0] *= s; v[1] *= s; v[2] *= s;
	}

	inline double dot(const vec3& o) const { return v[0] * o[0] + v[1] * o[1] + v[2] * o[2]; }
	inline vec3 cross(const vec3& o) const
	{
		return vec3(v[1] * o[2] - v[2] * o[1],
					v[2] * o[0] - v[0] * o[2],
					v[0] * o[1] - v[1] * o[0]);
	}

	inline ostream& operator<<(ostream &os) const
	{
		os << v[0] << " " << v[1] << " " << v[2];
		return os;
	}

private:
	double v[3];
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
	Ray(const vec3 &a, const vec3 &b) { A = a; B = b; }
	~Ray() {}

	vec3 origin() const { return A; }
	vec3 direction() const { return B; }
	vec3 pointAtParameter(double t) const { return A + t * B; }

private:
	vec3 A;
	vec3 B;
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

// geometric equation for sphere: (x - cx)^2 + (y - cy)^2 + (z - cz)^2 = R^2
// vector form: dot(origin + t*direction - sphere_center, origin + t*direction - sphere_center) = R^2
// => dot(A + t*B - C, A + t*B - C) - R^2 = 0
// => dot(B,B)*t^2 + 2*dot(B,A - C)*t + dot(A - C, A - C) - R^2 = 0
// solve via quadratic equation for value under sqrt.
double hitSphere(const vec3 &center, double radius, const Ray &ray)
{
	vec3 oc = ray.origin() - center;
	double a = dot(ray.direction(), ray.direction());
	double b = 2.0 * dot(ray.direction(), oc);
	double c = dot(oc, oc) - radius * radius;
	double discriminant = b * b - 4.0 * a * c;
	if (discriminant < 0)
		return -1.0;

	return -b - sqrt(discriminant) / 2.0 * a;
}

vec3 color(const Ray &r)
{
	double t = hitSphere(vec3(0.0, 0.0, -1.0), 0.5, r);
	if (t > 0.0)
		return normalToTextureSpace(createUnitVector(r.pointAtParameter(t) - vec3(0.0, 0.0, -1.0)));

	vec3 dir = createUnitVector(r.direction());
	t = 0.5 * (dir.y() + 1.0);
	return lerp(vec3(1.0, 1.0, 1.0), vec3(0.5, 0.7, 1.0), t);
}

int main()
{
	ofstream image_file("image.ppm");

	// todo: add std_image.h for more output formats
	// output to ppm file format
	int nx = 200;
	int ny = 100;
	image_file << "P3\n" << nx << " " << ny << "\n255\n";

	vec3 bottom_left_corner(-2.0, -1.0, -1.0);
	vec3 horizontal(4.0, 0.0, 0.0);
	vec3 vertical(0.0, 2.0, 0.0);
    vec3 origin(0.0, 0.0, 0.0);

	for (int j = ny - 1; j >= 0; --j)
	{
		for (int i = 0; i < nx; ++i)
		{
			double u = double(i) / double(nx);
			double v = double(j) / double(ny);
			Ray ray(origin, bottom_left_corner + u * horizontal + v * vertical);
			vec3 col = color(ray);
			int ir = int(255.99 * col[0]);
			int ig = int(255.99 * col[1]);
			int ib = int(255.99 * col[2]);

			image_file << ir << " " << ig << " " << ib << "\n";
		}
	}

	image_file.close();
	return 0;
}


