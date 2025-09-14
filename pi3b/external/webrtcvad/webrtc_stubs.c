#include <stdio.h>
#include <stdlib.h>

void rtc_FatalMessage(const char* file, int line, const char* msg) {
    fprintf(stderr, "WebRTC fatal error at %s:%d: %s\n", file, line, msg);
    abort();
}

// Minimal stub for "once"
typedef void (*InitOnceFunction)(void);

int once(InitOnceFunction fn) {
    static int initialized = 0;
    if (!initialized) {
        fn();
        initialized = 1;
    }
    return 0;
}
