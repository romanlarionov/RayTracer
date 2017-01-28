
#ifndef __UTILITIES__
#define __UTILITIES__

#include <chrono>
#include <thread>
#include <random>

#ifndef M_PI
	#define M_PI 3.14159265358979323846264338327950288
#endif

#if defined (_MSC_VER)  // Visual studio
    #define thread_local __declspec(thread)
#elif defined (__GCC__) // GCC
    #define thread_local __thread
#endif

// thread-safe, unbiased, cross-platform random number generator.
// warning: function leaks memory. shouldn't be a big deal for this application.
double getUnitRandom()
{
    static thread_local std::mt19937* generator = nullptr;
    if (!generator)
		generator = new std::mt19937(std::chrono::high_resolution_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<double> distribution(0.0, 1.0);
    return distribution(*generator);
}

#endif