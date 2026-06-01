//
//  Colors.h
//  hog2
//
//  Created by Nathan Sturtevant on 7/29/17.
//  Copyright © 2017 NS Software. All rights reserved.
//

#ifndef Colors_h
#define Colors_h

#include <iomanip>
#include <string>
#include <sstream>

/**
 * A color; r/g/b are between 0...1
 */
class rgbColor {
public:
	rgbColor() {}
	rgbColor(float rr, float gg, float bb) :r(rr), g(gg), b(bb) {}
	rgbColor &operator=(const rgbColor & ) = default;
	rgbColor &operator*=(float p) { r*=p; g*=p; b*=p; return *this; }
	rgbColor operator*(float p) const { rgbColor tmp(*this); tmp*=p; return tmp; }
	rgbColor operator+(const rgbColor &c) const { return mix(*this, c, 0.5); }
	static rgbColor mix(const rgbColor &c1, const rgbColor &c2, float perc)
	{
		return rgbColor((1-perc)*c1.r+c2.r*perc, (1-perc)*c1.g+c2.g*perc, (1-perc)*c1.b+c2.b*perc);
	}
	static rgbColor hsl(float h, float s, float l)
	{
		float r, g, b;

		if (s == 0)
		{
			r = g = b = l; // achromatic
		}
		else {
			float q = l < 0.5f ? l * (1 + s) : l + s - l * s;
			float p = 2 * l - q;
			r = hue2rgb(p, q, h + 1.0f/3.0f);
			g = hue2rgb(p, q, h);
			b = hue2rgb(p, q, h - 1.0f/3.0f);
		}
		return rgbColor(r, g, b);
	}
	void mix(const rgbColor &c, float perc)
	{
		r = (1-perc)*r+c.r*perc;
		g = (1-perc)*g+c.g*perc;
		b = (1-perc)*b+c.b*perc;
	}
	std::string hex() const
	{
        std::stringstream ss;
        ss << "#" << std::uppercase
            << std::setfill('0') << std::setw(2) << std::hex << static_cast<unsigned>(r * 255)
            << std::setfill('0') << std::setw(2) << std::hex << static_cast<unsigned>(g * 255)
            << std::setfill('0') << std::setw(2) << std::hex << static_cast<unsigned>(b * 255);
        return ss.str();
	}
	void hex(const char *str) // get color from #RRGGBB format
	{
		if (str[0] != '#')
		{
			printf("Conversion failed\n");
			return;
		}
		r = (unhexdigit(str[1])*16+unhexdigit(str[2]))/255.0;
		g = (unhexdigit(str[3])*16+unhexdigit(str[4]))/255.0;
		b = (unhexdigit(str[5])*16+unhexdigit(str[6]))/255.0;
	}
    std::size_t hash() const
    {
        return (static_cast<std::size_t>(r) << 16) |
            (static_cast<std::size_t>(g) << 8) |
            static_cast<std::size_t>(b);
    }
    bool operator==(const rgbColor &other) const
    {
        return r == other.r && g == other.g && b == other.b;
    }
    bool operator!=(const rgbColor &other) const
    {
        return !(*this==other);
    }
	float r,g,b;
private:
	static float hue2rgb(float p, float q, float t)
	{
		if (t < 0) t += 1;
		if (t > 1) t -= 1;
		if (t < 1.0/6.0) return p + (q - p) * 6 * t;
		if (t < 1.0/2.0) return q;
		if (t < 2.0/3.0) return p + (q - p) * (2.0/3.0 - t) * 6;
		return p;
	}

	int unhexdigit(char c)
	{
		switch (c)
		{
			case '0': return 0;
			case '1': return 1;
			case '2': return 2;
			case '3': return 3;
			case '4': return 4;
			case '5': return 5;
			case '6': return 6;
			case '7': return 7;
			case '8': return 8;
			case '9': return 9;
			case 'A': return 10;
			case 'B': return 11;
			case 'C': return 12;
			case 'D': return 13;
			case 'E': return 14;
			case 'F': return 15;
		}
		return 0;
	}
};

template<>
struct std::hash<rgbColor>
{
    std::size_t operator()(const rgbColor &color) const noexcept
    {
        return color.hash();
    }
};

namespace Colors
{
	/**
	 * Given min/max values, get a color from a color schema
	 */
	rgbColor GetColor(float v, float vmin, float vmax, int type);


	const rgbColor black  = {0.0,0.0,0.0};
	const rgbColor white  = {1.0,1.0,1.0};
	const rgbColor gray   = {0.5,0.5,0.5};
	const rgbColor bluegray   = {0.4,0.5,0.6};
	const rgbColor darkgray= {0.25,0.25,0.25};
	const rgbColor darkbluegray= {0.15,0.25,0.35};
	const rgbColor lightgray={0.75,0.75,0.75};
	const rgbColor lightbluegray={0.65,0.75,0.85};

	const rgbColor red    = {1.0,0.0,0.0}; // red
	const rgbColor darkred= {0.5,0.0,0.0}; // red
	const rgbColor lightred= {1.0,0.5,0.5}; // red
	const rgbColor lighterred= {1.0,0.75,0.75}; // red
	const rgbColor brown   = {0.5,0.25,0.0}; // brown
	const rgbColor lightbrown   = {0.75,0.5,0.25}; // brown

	const rgbColor green  = {0.0,1.0,0.0}; // green
	const rgbColor darkgreen= {0.0,0.5,0.0}; // green
	const rgbColor lightgreen= {0.5,1.0,0.5}; // green

	const rgbColor bluegreen   = {0.0f,0.65f,0.5f}; // blue


	const rgbColor blue   = {0.0,0.0,1.0}; // blue
	const rgbColor darkblue   = {0.0,0.0,0.5}; // blue
	const rgbColor lightblue   = {0.5,0.5,1.0}; // blue
	const rgbColor lighterblue   = {0.75,0.75,1.0}; // blue

	const rgbColor lightyellow = {1.0,1.0,0.5}; // yellow
	const rgbColor yellow = {1.0,1.0,0.0}; // yellow
	const rgbColor darkyellow = {0.5,0.5,0.0}; // yellow
	const rgbColor magenta = {1.0,0.0,1.0}; //
	const rgbColor purple = {0.5,0.0,1.0}; // purple
	const rgbColor darkpurple = {0.25,0.0,0.5}; //
	const rgbColor cyan   = {0.0,1.0,1.0}; // cyan

	const rgbColor orange = {1.0,0.5,0.0}; // orange
	const rgbColor pink   = {1.0,0.0,0.5}; // pink

	const rgbColor cb0    = {0, 0, 0}; // black
	const rgbColor cb1    = {0, 114.0f/255.0f, 178/255.0f};   // blue
	const rgbColor cb2    = {204/255.0f, 121/255.0f, 167/255.0f}; // reddish purple
	const rgbColor cb3    = {230/255.0f, 159/255.0f, 0};   // orange
	const rgbColor cb4    = {86/255.0f, 180/255.0f, 233/255.0f};  // sky blue
	const rgbColor cb5    = {213/255.0f, 94/255.0f, 0};   // vermillion
	const rgbColor cb_green  = {26/255.0f, 255/255.0f, 26/255.0f};
	const rgbColor cb_purple  = {75/255.0f,0,146/255.0f};

}


#endif /* Colors_h */
