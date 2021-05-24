/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2014 Bryan Venteicher <bryanv@FreeBSD.org>
 * Copyright (c) 2021 Mathieu Chouquet-Stringer
 * Copyright (c) 2021 Juniper Networks, Inc.
 * Copyright (c) 2021 Klara, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * Linux KVM paravirtual clock support
 *
 * References:
 *     - [1] https://www.kernel.org/doc/html/latest/virt/kvm/cpuid.html
 *     - [2] https://www.kernel.org/doc/html/latest/virt/kvm/msr.html
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/smp.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/pvclock.h>
#include <x86/kvm.h>

#include "clock_if.h"

#define	KVM_CLOCK_DEVNAME		"kvmclock"
/*
 * Note: Chosen to be (1) above HPET's value (always 950), (2) above the TSC's
 * default value of 800, and (3) below the TSC's value when it supports the
 * "Invariant TSC" feature and is believed to be synchronized across all CPUs.
 */
#define	KVM_CLOCK_TC_QUALITY		975

struct kvm_clock_softc {
	struct pvclock_softc		 pvcsc;
	struct pvclock_wall_clock	 wc;
	struct pvclock_vcpu_time_info	*timeinfos;
	u_int				 msr_tc;
	u_int				 msr_wc;
};

static devclass_t	kvm_clock_devclass;

static void		kvm_clock_system_time_enable(struct kvm_clock_softc *sc);
static void		kvm_clock_system_time_enable_pcpu(void *arg);

static inline struct pvclock_vcpu_time_info *
kvm_clock_get_curcpu_timeinfo(void *arg)
{
	struct pvclock_vcpu_time_info *timeinfos = arg;

	return (&timeinfos[curcpu]);
}

static void
kvm_clock_system_time_enable(struct kvm_clock_softc *sc)
{
	smp_rendezvous(NULL, kvm_clock_system_time_enable_pcpu, NULL, sc);
}

static void
kvm_clock_system_time_enable_pcpu(void *arg)
{
	struct kvm_clock_softc *sc = arg;

	/*
	 * See [2]; the lsb of this MSR is the system time enable bit.
	 */
	wrmsr(sc->msr_tc, vtophys(&(sc->timeinfos)[curcpu]) | 1);
}

static void
kvm_clock_identify(driver_t *driver, device_t parent)
{
	u_int regs[4];

	kvm_cpuid_get_features(regs);
	if ((regs[0] & KVM_FEATURE_CLOCKSOURCE2) == 0 &&
	    (regs[0] & KVM_FEATURE_CLOCKSOURCE) == 0)
		return;

	if (device_find_child(parent, KVM_CLOCK_DEVNAME, -1))
		return;

	BUS_ADD_CHILD(parent, 0, KVM_CLOCK_DEVNAME, 0);
}

static int
kvm_clock_probe(device_t dev)
{
	device_set_desc(dev, "KVM paravirtual clock");
	return (BUS_PROBE_DEFAULT);
}

static int
kvm_clock_attach(device_t dev)
{
	u_int regs[4];
	struct kvm_clock_softc *sc;
	bool stable_flag_supported;

	sc = device_get_softc(dev);

	/* Process KVM "features" CPUID leaf content: */
	kvm_cpuid_get_features(regs);
	if ((regs[0] & KVM_FEATURE_CLOCKSOURCE2) != 0) {
		sc->msr_tc = KVM_MSR_SYSTEM_TIME_NEW;
		sc->msr_wc = KVM_MSR_WALL_CLOCK_NEW;
	} else if ((regs[0] & KVM_FEATURE_CLOCKSOURCE) != 0) {
		sc->msr_tc = KVM_MSR_SYSTEM_TIME;
		sc->msr_wc = KVM_MSR_WALL_CLOCK;
	} else
		return (ENXIO);

	stable_flag_supported =
	    ((regs[0] & KVM_FEATURE_CLOCKSOURCE_STABLE_BIT) != 0);

	/* Set up 'struct pvclock_vcpu_time_info' page(s): */
	sc->timeinfos = contigmalloc(round_page(mp_ncpus *
	    sizeof(struct pvclock_vcpu_time_info)), M_DEVBUF, M_ZERO | M_NOWAIT,
	    0, ~0, PAGE_SIZE, 0);
	if (sc->timeinfos == NULL)
		return (ENOMEM);

	kvm_clock_system_time_enable(sc);

	/*
	 * Init pvclock softc:
	 *     Regarding 'tc_flags': Since the KVM MSR documentation does not
	 *     specifically discuss suspend/resume scenarios, conservatively
	 *     leave 'TC_FLAGS_SUSPEND_SAFE' cleared and assume the system time
	 *     must be re-inited in such cases.
	 */
	pvclock_softc_init(&sc->pvcsc, KVM_CLOCK_DEVNAME, KVM_CLOCK_TC_QUALITY,
	    0, kvm_clock_get_curcpu_timeinfo, sc->timeinfos, sc->timeinfos,
	    stable_flag_supported);

	/* Attach pvclock; register KVM clock timecounter and wall clock: */
	pvclock_attach(dev, &sc->pvcsc);

	return (0);
}

static int
kvm_clock_detach(device_t dev)
{
	struct kvm_clock_softc *sc;

	sc = device_get_softc(dev);

	return (pvclock_detach(dev, &sc->pvcsc));
}

static int
kvm_clock_suspend(device_t dev)
{
	return (0);
}

static int
kvm_clock_resume(device_t dev)
{
	kvm_clock_system_time_enable(device_get_softc(dev));
	pvclock_resume();
	inittodr(time_second);

	return (0);
}

static int
kvm_clock_gettime(device_t dev, struct timespec *ts)
{
	struct timespec system_ts;
	uint64_t system_nsec;
	struct kvm_clock_softc *sc;

	sc = device_get_softc(dev);

	critical_enter();
	wrmsr(sc->msr_wc, vtophys(&sc->wc));
	pvclock_get_wallclock(&sc->wc, ts);
	system_nsec = pvclock_get_timecount(&(sc->timeinfos)[curcpu]);
	critical_exit();

	system_ts.tv_sec = system_nsec / 1000000000ULL;
	system_ts.tv_nsec = system_nsec % 1000000000ULL;

	timespecadd(ts, &system_ts, ts);

	return (0);
}

static int
kvm_clock_settime(device_t dev, struct timespec *ts)
{
	/*
	 * Even though it is not possible to set the KVM clock's wall clock, to
	 * avoid the possibility of periodic benign error messages from
	 * 'settime_task_func()', report success rather than, e.g., 'ENODEV'.
	 */
	return (0);
}

static device_method_t kvm_clock_methods[] = {
	DEVMETHOD(device_identify,	kvm_clock_identify),
	DEVMETHOD(device_probe,		kvm_clock_probe),
	DEVMETHOD(device_attach,	kvm_clock_attach),
	DEVMETHOD(device_detach,	kvm_clock_detach),
	DEVMETHOD(device_suspend,	kvm_clock_suspend),
	DEVMETHOD(device_resume,	kvm_clock_resume),
	/* clock interface */
	DEVMETHOD(clock_gettime,	kvm_clock_gettime),
	DEVMETHOD(clock_settime,	kvm_clock_settime),

	DEVMETHOD_END
};

static driver_t kvm_clock_driver = {
	KVM_CLOCK_DEVNAME,
	kvm_clock_methods,
	sizeof(struct kvm_clock_softc),
};

DRIVER_MODULE(kvm_clock, nexus, kvm_clock_driver, kvm_clock_devclass, 0, 0);
