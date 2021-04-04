#ifndef HELPERS_H
#define HELPERS_H
#include <string>
#include <Godot.hpp>

#define SETGET(x, t)             \
	t x;                         \
	void set_##x(t v) { x = v; } \
	t get_##x() { return x; }

#ifndef MAX
#define MAX(m_a, m_b) (((m_a) > (m_b)) ? (m_a) : (m_b))
#endif

inline unsigned int ilog2(unsigned int v) {
	unsigned int r;
	unsigned int shift;
	r = (v > 0xffff) << 4;
	v >>= r;
	shift = (v > 0xff) << 3;
	v >>= shift;
	r |= shift;
	shift = (v > 0xf) << 2;
	v >>= shift;
	r |= shift;
	shift = (v > 0x3) << 1;
	v >>= shift;
	r |= shift;
	r |= (v >> 1);
	return r;
}

inline unsigned int nextPow2(unsigned int v) {
	v--;
	v |= v >> 1;
	v |= v >> 2;
	v |= v >> 4;
	v |= v >> 8;
	v |= v >> 16;
	v++;
	return v;
}

godot::Color HSVtoRGB(float fH, float fS, float fV);


const std::string CHARS = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";

std::string generateUUID();

#endif