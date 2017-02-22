
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

class CheckerTexture : public Texture
{
public:
	CheckerTexture(Texture *color1, Texture *color2) :
		_color1(color1),
		_color2(color2)
	{
	}

	virtual ~CheckerTexture() {}

	virtual vec3 sample(double u, double v, const vec3 &p) const
	{
		double sines = sin(10.0 * p.x()) * sin(10.0 * p.y()) * sin(10.0 * p.z());
		if (sines > 0.0)
			return _color1->sample(u, v, p);

		return _color2->sample(u, v, p);
	}

private:
	Texture *_color1;
	Texture *_color2;
};

#endif