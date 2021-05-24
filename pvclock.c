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
 * TSC. The former is always nanoseconds and the pvclock consumer is meant to
 * remain agnostic of the latter, simply using the scaling information provided
 * by the pvclock system time interface to convert TSC-based deltas into
 * nanoseconds.
 */
#define	PVCLOCK_RESOLUTION	1
/*
 * Note: Analogous to the note for 'PVCLOCK_RESOLUTION', this is the frequency
 * of the pvclock and is not to be confused with the frequency of the TSC.
 */
#define	PVCLOCK_TC_FREQUENCY	1000000000ULL

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

/*
 * Scale a 64-bit delta by scaling and multiplying by a 32-bit fraction,
 * yielding a 64-bit result.
 */
static inline uint64_t
pvclock_scale_delta(uint64_t delta, uint32_t mul_frac, int shift)
{
	uint64_t product;

	if (shift < 0)
		delta >>= -shift;
	else
		delta <<= shift;

#if defined(__i386__)
	{
		uint32_t tmp1, tmp2;

		/**
		 * For i386, the formula looks like:
		 *
		 *   lower = (mul_frac * (delta & UINT_MAX)) >> 32
		 *   upper = mul_frac * (delta >> 32)
		 *   product = lower + upper
		 */
		__asm__ (
			"mul  %5       ; "
			"mov  %4,%%eax ; "
			"mov  %%edx,%4 ; "
			"mul  %5       ; "
			"xor  %5,%5    ; "
			"add  %4,%%eax ; "
			"adc  %5,%%edx ; "
			: "=A" (product), "=r" (tmp1), "=r" (tmp2)
			: "a" ((uint32_t)delta), "1" ((uint32_t)(delta >> 32)),
			  "2" (mul_frac) );
	}
#elif defined(__amd64__)
	{
		unsigned long tmp;

		__asm__ (
			"mulq %[mul_frac] ; shrd $32, %[hi], %[lo]"
			: [lo]"=a" (product), [hi]"=d" (tmp)
			: "0" (delta), [mul_frac]"rm"((uint64_t)mul_frac));
	}
#else
#error "pvclock: unsupported x86 architecture?"
#endif

	return (product);
}

static uint64_t
pvclock_get_nsec_offset(struct pvclock_vcpu_time_info *ti)
{
	uint64_t delta;

	delta = rdtsc() - ti->tsc_timestamp;

	return (pvclock_scale_delta(delta, ti->tsc_to_system_mul,
	    ti->tsc_shift));
}

static void
pvclock_read_time_info(struct pvclock_vcpu_time_info *ti,
    uint64_t *cycles, uint8_t *flags)
{
	uint32_t version;

	do {
		version = ti->version;
		rmb();
		*cycles = ti->system_time + pvclock_get_nsec_offset(ti);
		*flags = ti->flags;
		rmb();
	} while ((ti->version & 1) != 0 || ti->version != version);
}

static void
pvclock_read_wall_clock(struct pvclock_wall_clock *wc, uint32_t *sec,
    uint32_t *nsec)
{
	uint32_t version;

	do {
		version = wc->version;
		rmb();
		*sec = wc->sec;
		*nsec = wc->nsec;
		rmb();
	} while ((wc->version & 1) != 0 || wc->version != version);
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
	uint32_t sec, nsec;

	pvclock_read_wall_clock(wc, &sec, &nsec);
	ts->tv_sec = sec;
	ts->tv_nsec = nsec;
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

	if (PROT_EXTRACT(nprot) != PROT_READ)
		return (EINVAL);

	*paddr = vtophys(dev->si_drv1);
	*memattr = VM_MEMATTR_DEFAULT;

	return (0);
}

static u_int
pvclock_tc_get_timecount(struct timecounter *tc)
{
	struct pvclock_softc *sc = tc->tc_priv;
	uint64_t time;

	critical_enter();
	time = pvclock_get_timecount(sc->get_curcpu_ti(sc->get_curcpu_ti_arg));
	critical_exit();

	return (time & UINT_MAX);
}

static uint32_t
pvclock_tc_vdso_timehands(struct vdso_timehands *vdso_th,
    struct timecounter *tc)
{
	struct pvclock_softc *sc = tc->tc_priv;

	vdso_th->th_algo = VDSO_TH_ALGO_X86_PVCLK;
	vdso_th->th_x86_shift = 0;
	vdso_th->th_x86_hpet_idx = 0;
	bzero(vdso_th->th_res, sizeof(vdso_th->th_res));

	return (sc->cdev != NULL && sc->stable_flag_supported &&
	    (sc->ti_vcpu0_page->flags & PVCLOCK_FLAG_TSC_STABLE) != 0);
}

#ifdef COMPAT_FREEBSD32
static uint32_t
pvclock_tc_vdso_timehands32(struct vdso_timehands32 *vdso_th,
    struct timecounter *tc)
{
	struct pvclock_softc *sc = tc->tc_priv;

	vdso_th->th_algo = VDSO_TH_ALGO_X86_PVCLK;
	vdso_th->th_x86_shift = 0;
	vdso_th->th_x86_hpet_idx = 0;
	bzero(vdso_th->th_res, sizeof(vdso_th->th_res));

	return (sc->cdev != NULL && sc->stable_flag_supported &&
	    (sc->ti_vcpu0_page->flags & PVCLOCK_FLAG_TSC_STABLE) != 0);
}
#endif

void
pvclock_softc_init(struct pvclock_softc *sc, const char *tc_name,
    int tc_quality, u_int tc_flags,
    pvclock_get_curcpu_timeinfo_t *get_curcpu_ti, void *get_curcpu_ti_arg,
    struct pvclock_vcpu_time_info *ti_vcpu0_page, bool stable_flag_supported)
{
	KASSERT(((uintptr_t)ti_vcpu0_page & PAGE_MASK) == 0, ("Specified vCPU "
	    "0 time info page address not page-aligned."));

	sc->tc.tc_get_timecount = pvclock_tc_get_timecount;
	sc->tc.tc_poll_pps = NULL;
	sc->tc.tc_counter_mask = ~0U;
	sc->tc.tc_frequency = PVCLOCK_TC_FREQUENCY;
	sc->tc.tc_name = tc_name;
	sc->tc.tc_quality = tc_quality;
	sc->tc.tc_flags = tc_flags;
	sc->tc.tc_priv = sc;
	sc->tc.tc_fill_vdso_timehands = pvclock_tc_vdso_timehands;
#ifdef COMPAT_FREEBSD32
	sc->tc.tc_fill_vdso_timehands32 = pvclock_tc_vdso_timehands32;
#endif

	sc->cdev = NULL;

	sc->get_curcpu_ti = get_curcpu_ti;
	sc->get_curcpu_ti_arg = get_curcpu_ti_arg;

	sc->ti_vcpu0_page = ti_vcpu0_page;

	sc->stable_flag_supported = stable_flag_supported;
}

void
pvclock_attach(device_t dev, struct pvclock_softc *sc)
{
	struct make_dev_args mda;
	int err;

	/* Set up cdev for userspace mmapping of vCPU 0 time info page: */
	make_dev_args_init(&mda);
	mda.mda_devsw = &pvclock_cdev_cdevsw;
	mda.mda_uid = UID_ROOT;
	mda.mda_gid = GID_WHEEL;
	mda.mda_mode = 0444;
	mda.mda_si_drv1 = sc->ti_vcpu0_page;
	err = make_dev_s(&mda, &sc->cdev, PVCLOCK_CDEVNAME);
	if (err != 0) {
		device_printf(dev, "Could not create /dev/%s, error %d. Fast "
		    "time of day will be unavailable for this timecounter.\n",
		    PVCLOCK_CDEVNAME, err);
		KASSERT(sc->cdev == NULL, ("Failed make_dev_s() unexpectedly "
		    "inited cdev."));
	}

	/* Register timecounter: */
	tc_init(&sc->tc);

	/* Register wallclock: */
	clock_register(dev, PVCLOCK_RESOLUTION);
}

int
pvclock_detach(device_t dev, struct pvclock_softc *sc)
{
	/*
	 * Not currently possible since there is no teardown counterpart of
	 * 'tc_init()'.
	 */
	return (EBUSY);
}
