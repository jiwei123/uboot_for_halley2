/**
 * nanddebug.c
 **/

#include <stdarg.h>
//#include <stdio.h>
//#include <stdarg.h>
enum {
	DUG_LEVEL_INFO = 1,
	DUG_LEVEL_WARNING,
	DUG_LEVEL_DEBUG,
	DUG_LEVEL_ERROR,
	DUG_LEVEL_SYSINFO,
};

int nm_dbg_level = DUG_LEVEL_WARNING;
//int nm_dbg_level = DUG_LEVEL_INFO;
int utils_dbg_level = DUG_LEVEL_WARNING;
int libops_dbg_level = DUG_LEVEL_WARNING;

extern int vprintf(const char *, va_list);
int __ndprint(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);

	return 0;
}

void nd_dump_stack(void) {
}
