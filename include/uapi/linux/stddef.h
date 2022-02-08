/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#include <linux/compiler_types.h>

#ifndef __always_inline
#define __always_inline inline
#endif
/*Young 5.16*/
#define __DECLARE_FLEX_ARRAY(TYPE, NAME)	\
	struct { \
		struct { } __empty_ ## NAME; \
		TYPE NAME[]; \
	}
