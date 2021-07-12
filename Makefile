KMOD=		kvmclock

SRCS=		kvm_clock.c pvclock.c rdtsc_ordered.c
SRCS+=		bus_if.h clock_if.h device_if.h

CFLAGS+=	-Iinclude
CFLAGS+=	-DINVARIANTS -DINVARIANT_SUPPORT

.include <bsd.kmod.mk>
