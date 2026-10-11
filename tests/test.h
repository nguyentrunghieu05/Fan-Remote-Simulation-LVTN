/* Minimal unit-test helpers: no external framework needed. */
#ifndef TEST_H
#define TEST_H
#include <stdio.h>
#include <math.h>

extern int g_failures, g_checks;

#define CHECK(cond) do { g_checks++; if (!(cond)) { g_failures++; \
    printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define CHECK_EQ(a, b) do { long long _a = (long long)(a), _b = (long long)(b); g_checks++; \
    if (_a != _b) { g_failures++; printf("  FAIL %s:%d: %s == %lld, expected %lld\n", \
    __FILE__, __LINE__, #a, _a, _b); } } while (0)
#define CHECK_NEAR(a, b, tol) do { double _a = (a), _b = (b); g_checks++; \
    if (fabs(_a - _b) > (tol)) { g_failures++; printf("  FAIL %s:%d: %s = %f, expected %f\n", \
    __FILE__, __LINE__, #a, _a, _b); } } while (0)
#define RUN(fn) do { printf("- %s\n", #fn); fn(); } while (0)

#endif
