#include "box2d/b2_timer.h"

#if defined(_WIN32)

double b2Timer::s_invFrequency = 0.0;

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <windows.h>

b2Timer::b2Timer()
{
	LARGE_INTEGER largeInteger;

	if (s_invFrequency == 0.0)
	{
		QueryPerformanceFrequency(&largeInteger);
		s_invFrequency = double(largeInteger.QuadPart);
		if (s_invFrequency > 0.0)
		{
			s_invFrequency = 1000.0 / s_invFrequency;
		}
	}

	QueryPerformanceCounter(&largeInteger);
	m_start = double(largeInteger.QuadPart);
}

void b2Timer::Reset()
{
	LARGE_INTEGER largeInteger;
	QueryPerformanceCounter(&largeInteger);
	m_start = double(largeInteger.QuadPart);
}

float b2Timer::GetMilliseconds() const
{
	LARGE_INTEGER largeInteger;
	QueryPerformanceCounter(&largeInteger);
	double count = double(largeInteger.QuadPart);
	float ms = float(s_invFrequency * (count - m_start));
	return ms;
}

#elif defined(__linux__) || defined (__APPLE__)

#include <sys/time.h>

b2Timer::b2Timer()
{
    Reset();
}

void b2Timer::Reset()
{
    timeval t;
    gettimeofday(&t, 0);
    m_start_sec = t.tv_sec;
    m_start_usec = t.tv_usec;
}

float b2Timer::GetMilliseconds() const
{
    timeval t;
    gettimeofday(&t, 0);
	time_t start_sec = m_start_sec;
	suseconds_t start_usec = m_start_usec;
	
	// http://www.gnu.org/software/libc/manual/html_node/Elapsed-Time.html
	if (t.tv_usec < start_usec)
	{
		int nsec = (start_usec - t.tv_usec) / 1000000 + 1;
		start_usec -= 1000000 * nsec;
		start_sec += nsec;
	}
	
	if (t.tv_usec - start_usec > 1000000)
	{
		int nsec = (t.tv_usec - start_usec) / 1000000;
		start_usec += 1000000 * nsec;
		start_sec -= nsec;
	}
	return 1000.0f * (t.tv_sec - start_sec) + 0.001f * (t.tv_usec - start_usec);
}

#else

b2Timer::b2Timer()
{
}

void b2Timer::Reset()
{
}

float b2Timer::GetMilliseconds() const
{
	return 0.0f;
}

#endif
