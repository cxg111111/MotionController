#include "log.h"
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>
#include <stdbool.h>

#define MAX_CALLBACKS 32

typedef struct {
    log_LogFn fn;
    void *udata;
    int level;
} Callback;

static struct {
    void *udata;
    log_LockFn lock;
    int level;
    bool quiet;
    Callback callbacks[MAX_CALLBACKS];
    CRITICAL_SECTION mutex;     // 临界区用于线程同步
    volatile LONG initialized;  // 原子变量标记初始化状态
} L = {0};

static const char *level_strings[] = {"TRACE", "DEBUG", "INFO", "WARN", "ERROR", "FATAL"};

#ifdef LOG_USE_COLOR
static const char *level_colors[] = {"\x1b[94m", "\x1b[36m", "\x1b[32m", "\x1b[33m", "\x1b[31m", "\x1b[35m"};
#endif

// 确保只初始化一次的函数
static void ensure_initialized(void) {
    if (InterlockedCompareExchange(&L.initialized, 1, 0) == 0) {
        L.level = LOG_INFO;
        L.quiet = false;
        L.lock = NULL;
        L.udata = NULL;
        InitializeCriticalSection(&L.mutex);
        
        // 清空回调数组
        for (int i = 0; i < MAX_CALLBACKS; i++) {
            L.callbacks[i].fn = NULL;
            L.callbacks[i].udata = NULL;
            L.callbacks[i].level = 0;
        }
    }
}

static void lock(void) {
    if (L.lock) {
        L.lock(true, L.udata);
    } else {
        EnterCriticalSection(&L.mutex);
    }
}

static void unlock(void) {
    if (L.lock) {
        L.lock(false, L.udata);
    } else {
        LeaveCriticalSection(&L.mutex);
    }
}

static void stdout_callback(log_Event *ev) {
    char buf[100] = {0};
    time_t timep;
    time(&timep);
    struct tm *pt = gmtime(&timep);
    sprintf_s(buf, sizeof(buf), "%d-%02d-%02d %02d:%02d:%02d", 
              1900 + pt->tm_year, 1 + pt->tm_mon, pt->tm_mday, 
              (8 + pt->tm_hour) % 24, pt->tm_min, pt->tm_sec);
#ifdef LOG_USE_COLOR
    fprintf((FILE *)ev->udata, "%s %s%-5s\x1b[0m \x1b[90m%s:%d:\x1b[0m ", 
            buf, level_colors[ev->level], level_strings[ev->level], ev->file, ev->line);
#else
    fprintf((FILE *)ev->udata, "%s %-5s %s:%d: ", 
            buf, level_strings[ev->level], ev->file, ev->line);
#endif
    vfprintf((FILE *)ev->udata, ev->fmt, ev->ap);
    fprintf((FILE *)ev->udata, "\n");
    fflush((FILE *)ev->udata);
}

static void file_callback(log_Event *ev) {
    char buf[100] = {0};
    time_t timep;
    time(&timep);
    struct tm *pt = gmtime(&timep);
    sprintf_s(buf, sizeof(buf), "%d-%02d-%02d %02d:%02d:%02d", 
              1900 + pt->tm_year, 1 + pt->tm_mon, pt->tm_mday, 
              (8 + pt->tm_hour) % 24, pt->tm_min, pt->tm_sec);
    fprintf((FILE *)ev->udata, "%s %-5s %s:%d: ", 
            buf, level_strings[ev->level], ev->file, ev->line);
    vfprintf((FILE *)ev->udata, ev->fmt, ev->ap);
    fprintf((FILE *)ev->udata, "\n");
    fflush((FILE *)ev->udata);
}

const char *log_level_string(int level) { 
    ensure_initialized();
    return level_strings[level]; 
}

void log_init(void) {
    ensure_initialized();
}

void log_cleanup(void) {
    if (InterlockedCompareExchange(&L.initialized, 0, 1) == 1) {
        DeleteCriticalSection(&L.mutex);
    }
}

void log_set_lock(log_LockFn fn, void *udata) {
    ensure_initialized();
    lock();
    L.lock = fn;
    L.udata = udata;
    unlock();
}

void log_set_level(int level) { 
    ensure_initialized();
    lock();
    L.level = level;
    unlock();
}

void log_set_quiet(bool enable) { 
    ensure_initialized();
    lock();
    L.quiet = enable;
    unlock();
}

int log_add_callback(log_LogFn fn, void *udata, int level) {
    ensure_initialized();
    lock();
    
    for (int i = 0; i < MAX_CALLBACKS; i++) {
        if (!L.callbacks[i].fn) {
            L.callbacks[i].fn = fn;
            L.callbacks[i].udata = udata;
            L.callbacks[i].level = level;
            unlock();
            return 0;
        }
    }
    
    unlock();
    return -1;
}

int log_add_fp(FILE *fp, int level) { 
    return log_add_callback(file_callback, fp, level); 
}

static void init_event(log_Event *ev, void *udata) {
    if (!ev->time) {
        time_t t = time(NULL);
        //ev->time = gmtime(&t);
    }
    ev->udata = udata;
}

void log_log(int level, const char *file, int line, const char *fmt, ...) {
    ensure_initialized();
    
    log_Event ev = {0};
    ev.fmt = fmt;
    ev.file = file;
    ev.line = line;
    ev.level = level;

    lock();

    if (!L.quiet && level >= L.level) {
        init_event(&ev, stderr);
        va_start(ev.ap, fmt);
        stdout_callback(&ev);
        va_end(ev.ap);
    }

    for (int i = 0; i < MAX_CALLBACKS && L.callbacks[i].fn; i++) {
        Callback *cb = &L.callbacks[i];
        if (level >= cb->level) {
            init_event(&ev, cb->udata);
            va_start(ev.ap, fmt);
            cb->fn(&ev);
            va_end(ev.ap);
        }
    }

    unlock();
}