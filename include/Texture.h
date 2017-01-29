
#ifndef __TEXTURE__
#define __TEXTURE__

class Texture
{
public:
	virtual vec3 sample(double u, double v, const vec3 &p) const = 0;	
};

class ConstantTexture : public Texture
{
public:
	ConstantTexture(vec3 color) :
	 _color(color)
	{
	}

	virtual vec3 sample(double u, double v, const vec3 &p) const
	{
		return _color;
	}

private:
	vec3 _color;
};

#endif