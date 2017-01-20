
#include <cmath>
#include <cstring> // memset
#include <fstream>

using namespace std;

// todo: make template. for now only double
class vec3
{
public:
	vec3()
	{
		//memset(v, 0, sizeof(double) * 3);
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

	inline const vec3& operator+() const { return *this; } 
	inline vec3 operator-() const { return vec3(-v[0], -v[1], -v[2]); } 
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
	//inline vec3& operator*=(const vec3& other) { return ;} 
	//inline vec3& operator/=(const vec3& other) { return ;} 
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
	inline double squared_mag() const { return v[0] * v[0] + v[1] * v[1] + v[2] * v[2]; }
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

	/*inline istream& operator>>(istream &is) const
	{
		is >> v[0] >> v[1] >> v[2];
		return is;
	}*/

	inline ostream& operator<<(ostream &os) const
	{
		os << v[0] << " " << v[1] << " " << v[2];
		return os;
	}

private:
	double v[3];
};

int main()
{
	ofstream image_file("image.ppm");

	// todo: add std_image.h for more output formats
	// output to ppm file format
	int nx = 200;
	int ny = 100;
	image_file << "P3\n" << nx << " " << ny << "\n255\n";
	for (int j = ny - 1; j >= 0; --j)
	{
		for (int i = 0; i < nx; ++i)
		{
			vec3 col(double(i) / double(nx), double(j) / double(ny), 0.2);
			int ir = int(255.99 * col.x());
			int ig = int(255.99 * col.y());
			int ib = int(255.99 * col.z());

			image_file << ir << " " << ig << " " << ib << "\n";
		}
	}

	image_file.close();
	return 0;
}