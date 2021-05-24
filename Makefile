KMOD=		kvmclock

SRCS=		kvm_clock.c pvclock.c
SRCS+=		bus_if.h clock_if.h device_if.h

CFLAGS+=	-Iinclude

.include <bsd.kmod.mk>
