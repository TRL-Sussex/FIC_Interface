/*
 * fic_gl_pace: cap how often OpenGL windows redraw.
 *
 * On a display without vertical sync (the container's virtual desktop, or any
 * display with software OpenGL) MuJoCo's viewers redraw as fast as they can:
 * the render loop never waits, and Mesa's software renderer spreads each frame
 * over every CPU core. That starves the simulation and the 1 kHz controller of
 * CPU while drawing frames nobody can see (the VNC stream sends ~30 a second).
 *
 * This file is built twice, into wrappers named libGL.so.1 and libGLX.so.0,
 * which are placed first on LD_LIBRARY_PATH (see fic-image-setup). Each wrapper
 * depends on the real library by absolute path, so every GL/GLX function comes
 * from the real library, except glXSwapBuffers: this one waits until at least
 * 1/FIC_GL_MAX_FPS seconds have passed since the thread's previous swap, then
 * swaps. Both GLFW 3.3 (libGL.so.1, used by native GLFW programs) and GLFW 3.4
 * (libGLX.so.0, used by the Python viewers) look glXSwapBuffers up by name in
 * the library they open, so both end up here.
 *
 * FIC_GL_MAX_FPS: frames per second, default 60; 0 turns pacing off.
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#ifndef FIC_REAL_LIB
#error "build with -DFIC_REAL_LIB=\"/usr/lib/<triplet>/libGL.so.1\" (or libGLX.so.0)"
#endif

typedef struct _XDisplay Display;
typedef unsigned long GLXDrawable;
typedef void (*swap_fn)(Display *, GLXDrawable);

static swap_fn real_swap;
static void *real_lib;
static long long interval_ns = -1;

static long long now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void pace(void)
{
    static __thread long long next_ns;

    if (interval_ns < 0) {
        const char *env = getenv("FIC_GL_MAX_FPS");
        long fps = env ? strtol(env, NULL, 10) : 60;
        interval_ns = fps > 0 ? 1000000000LL / fps : 0;
    }
    if (interval_ns == 0)
        return;

    long long t = now_ns();
    if (next_ns > t) {
        long long wait = next_ns - t;
        struct timespec d = { wait / 1000000000LL, wait % 1000000000LL };
        while (nanosleep(&d, &d) != 0)
            ;
        t = next_ns;
    }
    next_ns = t + interval_ns;
}

/* FIC_GL_STATS=1: print the achieved frame rate every 5 s (for tuning). */
static void stats(void)
{
    static __thread long long since_ns;
    static __thread int frames;
    static int enabled = -1;
    if (enabled < 0) {
        const char *env = getenv("FIC_GL_STATS");
        enabled = env && env[0] == '1';
    }
    if (!enabled)
        return;
    long long t = now_ns();
    if (!since_ns)
        since_ns = t;
    frames++;
    if (t - since_ns >= 5000000000LL) {
        fprintf(stderr, "fic_gl_pace: %.1f frames/s\n", frames * 1e9 / (t - since_ns));
        since_ns = t;
        frames = 0;
    }
}

static void *real(const char *name)
{
    if (!real_lib) {
        real_lib = dlopen(FIC_REAL_LIB, RTLD_LAZY | RTLD_NOLOAD);
        if (!real_lib)
            real_lib = dlopen(FIC_REAL_LIB, RTLD_LAZY);
    }
    return real_lib ? dlsym(real_lib, name) : NULL;
}

__attribute__((visibility("default")))
void glXSwapBuffers(Display *dpy, GLXDrawable drawable)
{
    if (!real_swap) {
        real_swap = (swap_fn)real("glXSwapBuffers");
        if (!real_swap) {
            fprintf(stderr, "fic_gl_pace: glXSwapBuffers not found in %s\n", FIC_REAL_LIB);
            return;
        }
    }
    pace();
    real_swap(dpy, drawable);
    stats();
}
