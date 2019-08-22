#include <stdint.h>
#include <math.h>

template <typename T>
struct Point
{
	T x;
	T y;

	Point(T x=0, T y=0) : x(x), y(y) {}
	template <typename U>
	Point(const Point<U> & other) : x(other.x), y(other.y) {}

	template <typename Y>
	friend Point operator -(const Point<Y> & lhs, const Point & rhs)
	{
		return Point(lhs.x - rhs.x, lhs.y - rhs.y);
	}

};

template <typename T>
struct rgba
{
	T r;
	T g;
	T b;
	T a;

	rgba(T r = 0, T g = 0, T b = 0, T a = 0) : r(r), g(g), b(b), a(a) {}
	template <typename U>
	rgba(const rgba<U> & other) : r(other.r), g(other.g), b(other.b), a(other.a) {}

	template <typename Y>
	friend rgba<Y> operator *(rgba lhs, Y rhs)
	{
		return rgba(((Y)lhs.r)*rhs, ((Y)lhs.g)*rhs, ((Y)lhs.b)*rhs, ((Y)lhs.a)*rhs);
	}

	//template <typename T>
	friend rgba<T> operator +(rgba<T> lhs, const rgba<T> & rhs)
	{
		return rgba(lhs.r + rhs.r, lhs.g + rhs.g, lhs.b + rhs.b, lhs.a + rhs.a);
	}
};


Point<uint8_t> polar_lut[500][32];
Point<float> offset_lut[500][32];
rgba<uint8_t> img[64][64];
uint8_t polar[500][96];

#define min(x, y) (x < y ? x : y)
#define max(x, y) (x > y ? x : y)

void make_polar(rgba<uint8_t> in[64][64], uint8_t out[500][96])
{
	uint32_t line = 0;
	uint32_t px = 0;
	for (line = 0; line < 500; line++)
	{
		for (px = 0; px < 32; px++)
		{
			Point<float> offset;
			Point<uint8_t> tlc;

			offset = offset_lut[line][px];
			tlc = polar_lut[line][px];

			rgba<uint8_t> tl = in[tlc.y][tlc.x];
			rgba<uint8_t> tr = in[tlc.y][min(63, tlc.x + 1)];
			rgba<uint8_t> bl = in[min(63, tlc.y + 1)][tlc.x];
			rgba<uint8_t> br = in[min(63, tlc.y + 1)][min(63, tlc.x + 1)];

			rgba<float> pl = tl * (1 - offset.y) + bl * offset.y;
			rgba<float> pr = tr * (1 - offset.y) + br * offset.y;
			rgba<float> p = pl * (1 - offset.x) + pr * offset.x;
			out[line][px * 3 + 0] = p.r;
			out[line][px * 3 + 1] = p.g;
			out[line][px * 3 + 2] = p.b;
		}
	}
}

			

int main()
{
	uint32_t line = 0;
	uint32_t px = 0;
	for (line = 0; line < 500; line++)
	{
		for (px = 0; px < 32; px++)
		{
			Point<float> p;
			Point<uint8_t> f;

			float phi = 2 * 3.141592654 * line / 500;
			p.x = px * cos(phi) + 32;
			p.y = px * sin(phi) + 32;
			f = Point<uint8_t>(p);
			polar_lut[line][px] = f;
			offset_lut[line][px] = p - f;
		}
	}

	int fd = open("/dev/smi", O_RDWR);

	while (1)
	{
		make_polar(img, polar);
		
		for (line = 0; line < 500; line++)
		{
			write(fd, polar[line], 96);
		}
	}

	return 0;
}
