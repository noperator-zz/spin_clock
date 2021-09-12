#include <stdint.h>
#include <math.h>

template <typename T>
struct Point<T>
{
	T x;
	T y;
	
	Point(T x, T y) : x(x), y(y) {}
	
	Point operator -(Point lhs, const Point & rhs)
	{
		return Point(lhs.x-rhs.x, lhs.y-rhs.y);
	}
		
};

template <typename T>
struct rgba
{
	T r;
	T g;
	T b;
	T a;
	
	rgba operator *(rgba lhs, float rhs)
	{
		return rgba(lhs.r*rhs, lhs.g*rhs, lhs.b*rhs, lhs.a*rhs);
	}
};

Point<uint8_t> polar_lut[500][32];
Point<float> offset_lut[500][32];
rgba img[64][64];
uint8_t polar[500][96];

void make_polar(rgba in[64][64], uint8_t out[500][64])
{
	for (line = 0; line < 500; line++)
	{
		for (px = 0; px < 32; px++)
		{
			Point<float> offset;
			Point<uint8_t> tlc;
			
			offset = offset_lut[line][px];
			tlc = polar_lut[line][px];
			
			rgba tl = in[tlc.y][tlc.x];
			rgba tr = in[tlc.y][min(63, tlc.x + 1)];
			rgba bl = in[min(63, tlc.y + 1)][tlc.x];
			rgba br = in[min(63, tlc.y + 1)][min(63, tlc.x + 1)];
			
            rgba<float> pl = tl * (1 - offset.y) + bl * offset.y
            rgba<float> pr = tr * (1 - offset.y) + br * offset.y
            rgba<float> p = pl * (1 - offset.x) + pr * offset.x
			out[line][px] = p;
			
    op = np.zeros(shape=(500, 32, 3)).astype('uint8')
    for line in range(500):
        for px in range(32):
            ox, oy = offset_lut[line][px]
            nx, ny = polar_lut[line][px]
            tl = img[ny][nx]
            tr = img[ny][min(63, nx+1)]
            bl = img[min(63, ny+1)][nx]
            br = img[min(63, ny+1)][min(63, nx+1)]

            pl = tl * (1 - oy) + bl * oy
            pr = tr * (1 - oy) + br * oy
            p = pl * (1 - ox) + pr * ox
            op[line][px] = p
        print op[line]
    return op


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
			
			float phi = 2 * PI * line / 500;
			p.x = px * cos(phi) + 32;
			p.y = px * sin(phi) + 32;
			f.x = int(x);
			f.y = int(y);
			polar_lut[line][px] = f;
			offset_lut[line][px] = p - f;
		}
	}

	int fd = open("/dev/smi", O_RDWR);
