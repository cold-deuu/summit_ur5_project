#include <stdlib.h>
#include <stddef.h>
extern void *__libc_malloc(size_t);
extern void *__libc_calloc(size_t,size_t);
extern void *__libc_realloc(void*,size_t);
extern void *__libc_memalign(size_t,size_t);
extern void __libc_free(void*);
static __thread int active;
static __thread unsigned long count;
void allocation_guard_start(void) { count=0; active=1; }
unsigned long allocation_guard_stop(void) { active=0; return count; }
void *malloc(size_t n) { if(active) ++count; return __libc_malloc(n); }
void *calloc(size_t n,size_t s) { if(active) ++count; return __libc_calloc(n,s); }
void *realloc(void *p,size_t n) { if(active) ++count; return __libc_realloc(p,n); }
void *memalign(size_t a,size_t n) { if(active) ++count; return __libc_memalign(a,n); }
void *aligned_alloc(size_t a,size_t n) { if(active) ++count; return __libc_memalign(a,n); }
int posix_memalign(void **p,size_t a,size_t n) {
    if(active) ++count;
    *p=__libc_memalign(a,n); return *p?0:12;
}
void free(void *p) { __libc_free(p); }
