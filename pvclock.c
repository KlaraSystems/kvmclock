/*-
 * Copyright (c) 2009 Adrian Chadd
 * Copyright (c) 2012 Spectra Logic Corporation
 * Copyright (c) 2014 Bryan Venteicher
 * All rights reserved.
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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/clock.h>
#include <sys/conf.h>
#include <sys/fcntl.h>
#include <sys/limits.h>
#include <sys/mman.h>
#include <sys/proc.h>
#include <sys/vdso.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/cpufunc.h>
#include <machine/cpu.h>
#include <machine/atomic.h>
#include <machine/pvclock.h>

/*
 * Note: This is the resolution of the pvclock and not the resolution of the
 * TSC. The former is always 1ns and the pvclock consumer is meant to remain
 * agnostic of the latter, simply using the scaling information provided by the
 * pvclock system time interface to convert TSC-based deltas into nanoseconds.
 */
#define	PVCLOCK_RESOLUTION_NS	1
/*
 * Note: Analogous to the note for 'PVCLOCK_RESOLUTION_NS', this is the
 * frequency of the pvclock and is not to be confused with the frequency of the
 * TSC.
 */
#define	PVCLOCK_FREQUENCY_HZ	1000000000ULL

/*
 * Last time; this guarantees a monotonically increasing clock for when
 * a stable TSC is not provided.
 */
static volatile uint64_t pvclock_last_cycles;

static u_int		 pvclock_tc_get_timecount(struct timecounter *tc);
static uint32_t		 pvclock_tc_vdso_timehands(
    struct vdso_timehands *vdso_th, struct timecounter *tc);
#ifdef COMPAT_FREEBSD32
static uint32_t		 pvclock_tc_vdso_timehands32(
    struct vdso_timehands32 *vdso_th, struct timecounter *tc);
#endif

static d_open_t		 pvclock_cdev_open;
static d_mmap_t		 pvclock_cdev_mmap;

static struct cdevsw	 pvclock_cdev_cdevsw = {
	.d_version =	D_VERSION,
	.d_name =	PVCLOCK_CDEVNAME,
	.d_open =	pvclock_cdev_open,
	.d_mmap =	pvclock_cdev_mmap,
};

void
pvclock_resume(void)
{

	atomic_store_rel_64(&pvclock_last_cycles, 0);
}

uint64_t
pvclock_get_last_cycles(void)
{

	return (atomic_load_acq_64(&pvclock_last_cycles));
}

uint64_t
pvclock_tsc_freq(struct pvclock_vcpu_time_info *ti)
{
	uint64_t freq;

	freq = (1000000000ULL << 32) / ti->tsc_to_system_mul;

	if (ti->tsc_shift < 0)
		freq <<= -ti->tsc_shift;
	else
		freq >>= ti->tsc_shift;

	return (freq);
}

uint64_t
pvclock_get_timecount(struct pvclock_vcpu_time_info *ti)
{
	uint64_t now, last;
	uint8_t flags;

	pvclock_read_time_info(ti, &now, &flags);

	if (flags & PVCLOCK_FLAG_TSC_STABLE)
		return (now);

	/*
	 * Enforce a monotonically increasing clock time across all VCPUs.
	 * If our time is too old, use the last time and return. Otherwise,
	 * try to update the last time.
	 */
	do {
		last = atomic_load_acq_64(&pvclock_last_cycles);
		if (last > now)
			return (last);
	} while (!atomic_cmpset_64(&pvclock_last_cycles, last, now));

	return (now);
}

void
pvclock_get_wallclock(struct pvclock_wall_clock *wc, struct timespec *ts)
{
	uint32_t version;

	do {
		version = wc->version;
		rmb();
		ts->tv_sec = wc->sec;
		ts->tv_nsec = wc->nsec;
		rmb();
	} while ((wc->version & 1) != 0 || wc->version != version);
}

static int
pvclock_cdev_open(struct cdev *dev, int oflags, int devtype, struct thread *td)
{
	if (oflags & FWRITE)
		return (EPERM);

	return (0);
}

static int
pvclock_cdev_mmap(struct cdev *dev, vm_ooffset_t offset, vm_paddr_t *paddr,
    int nprot, vm_memattr_t *memattr)
{
	if (offset != 0)
		return (EINVAL);

#ifdef PROT_EXTRACT
	if (PROT_EXTRACT(nprot) != PROT_READ)
#else
	if (nprot != PROT_READ)
#endif
		return (EINVAL);

	*paddr = vtophys(dev->si_drv1);
	*memattr = VM_MEMATTR_DEFAULT;

	return (0);
}

static u_int
pvclock_tc_get_timecount(struct timecounter *tc)
{
	struct pvclock *pvc = tc->tc_priv;
	uint64_t ns;

	critical_enter();
	ns = pvclock_get_timecount(pvc->get_curcpu_ti(pvc->get_curcpu_ti_arg));
	critical_exit();

	return (ns & UINT_MAX);
}

static uint32_t
pvclock_tc_vdso_timehands(struct vdso_timehands *vdso_th,
    struct timecounter *tc)
{
	struct pvclock *pvc = tc->tc_priv;

	vdso_th->th_algo = VDSO_TH_ALGO_X86_PVCLK;
	vdso_th->th_x86_shift = 0;
	vdso_th->th_x86_hpet_idx = 0;
	bzero(vdso_th->th_res, sizeof(vdso_th->th_res));

	return (pvc->cdev != NULL && pvc->stable_flag_supported &&
	    (pvc->ti_vcpu0_page->flags & PVCLOCK_FLAG_TSC_STABLE) != 0);
}

#ifdef COMPAT_FREEBSD32
static uint32_t
pvclock_tc_vdso_timehands32(struct vdso_timehands32 *vdso_th,
    struct timecounter *tc)
{
	struct pvclock *pvc = tc->tc_priv;

	vdso_th->th_algo = VDSO_TH_ALGO_X86_PVCLK;
	vdso_th->th_x86_shift = 0;
	vdso_th->th_x86_hpet_idx = 0;
	bzero(vdso_th->th_res, sizeof(vdso_th->th_res));

	return (pvc->cdev != NULL && pvc->stable_flag_supported &&
	    (pvc->ti_vcpu0_page->flags & PVCLOCK_FLAG_TSC_STABLE) != 0);
}
#endif

void
pvclock_gettime(struct pvclock *pvc, struct timespec *ts)
{
	struct timespec system_ts;
	uint64_t system_ns;

	pvclock_get_wallclock(pvc->get_wallclock(pvc->get_wallclock_arg),
	    ts);

	critical_enter();
	system_ns =
	    pvclock_get_timecount(pvc->get_curcpu_ti(pvc->get_curcpu_ti_arg));
	critical_exit();

	system_ts.tv_sec = system_ns / 1000000000ULL;
	system_ts.tv_nsec = system_ns % 1000000000ULL;

	timespecadd(ts, &system_ts, ts);
}

void
pvclock_init(struct pvclock *pvc, device_t dev, const char *tc_name,
    int tc_quality, u_int tc_flags,
    pvclock_get_curcpu_timeinfo_t *get_curcpu_ti, void *get_curcpu_ti_arg,
    pvclock_get_wallclock_t *get_wallclock, void *get_wallclock_arg,
    struct pvclock_vcpu_time_info *ti_vcpu0_page, bool stable_flag_supported)
{
	struct make_dev_args mda;
	int err;

	KASSERT(((uintptr_t)ti_vcpu0_page & PAGE_MASK) == 0, ("Specified vCPU "
	    "0 time info page address not page-aligned."));

	/* Set up timecounter and timecounter-supporting members: */
	pvc->tc.tc_get_timecount = pvclock_tc_get_timecount;
	pvc->tc.tc_poll_pps = NULL;
	pvc->tc.tc_counter_mask = ~0U;
	pvc->tc.tc_frequency = PVCLOCK_FREQUENCY_HZ;
	pvc->tc.tc_name = tc_name;
	pvc->tc.tc_quality = tc_quality;
	pvc->tc.tc_flags = tc_flags;
	pvc->tc.tc_priv = pvc;
	pvc->tc.tc_fill_vdso_timehands = pvclock_tc_vdso_timehands;
#ifdef COMPAT_FREEBSD32
	pvc->tc.tc_fill_vdso_timehands32 = pvclock_tc_vdso_timehands32;
#endif

	pvc->get_curcpu_ti = get_curcpu_ti;
	pvc->get_curcpu_ti_arg = get_curcpu_ti_arg;

	pvc->get_wallclock = get_wallclock;
	pvc->get_wallclock_arg = get_wallclock_arg;

	pvc->ti_vcpu0_page = ti_vcpu0_page;

	pvc->stable_flag_supported = stable_flag_supported;

	/* Set up cdev for userspace mmapping of vCPU 0 time info page: */
	make_dev_args_init(&mda);
	mda.mda_devsw = &pvclock_cdev_cdevsw;
	mda.mda_uid = UID_ROOT;
	mda.mda_gid = GID_WHEEL;
	mda.mda_mode = 0444;
	mda.mda_si_drv1 = pvc->ti_vcpu0_page;
	err = make_dev_s(&mda, &pvc->cdev, PVCLOCK_CDEVNAME);
	if (err != 0) {
		device_printf(dev, "Could not create /dev/%s, error %d. Fast "
		    "time of day will be unavailable for this timecounter.\n",
		    PVCLOCK_CDEVNAME, err);
		KASSERT(pvc->cdev == NULL, ("Failed make_dev_s() unexpectedly "
		    "inited cdev."));
	}

	/* Register timecounter: */
	tc_init(&pvc->tc);

	/*
	 * Register wallclock:
	 *     The RTC registration API expects a resolution in microseconds;
	 *     'PVCLOCK_RESOLUTION_NS' is rounded up to the nearest 1us.
	 */
	clock_register(dev, (PVCLOCK_RESOLUTION_NS + 1000) / 1000);
}

int
pvclock_destroy(struct pvclock *pvc)
{
	/*
	 * Not currently possible since there is no teardown counterpart of
	 * 'tc_init()'.
	 */
	return (EBUSY);
}
