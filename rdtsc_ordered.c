/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/pcpu.h>

#include <machine/cputypes.h>
#include <machine/md_var.h>
#include <machine/specialreg.h>

#include <x86/ifunc.h>

static __inline void
lfence(void)
{
	__asm __volatile("lfence" : : : "memory");
}

static __inline void
mfence(void)
{
	__asm __volatile("mfence" : : : "memory");
}

static __inline uint64_t
rdtsc(void)
{
	uint32_t low, high;

	__asm __volatile("rdtsc" : "=a" (low), "=d" (high));
	return (low | ((uint64_t)high << 32));
}

static uint64_t
rdtsc_ordered_lfence(void)
{
	lfence();
	return (rdtsc());
}

static uint64_t
rdtsc_ordered_mfence(void)
{
	mfence();
	return (rdtsc());
}

static uint64_t
rdtscp(void)
{
	uint32_t low, high;

	__asm __volatile("rdtscp" : "=a" (low), "=d" (high) : : "ecx");
	return (low | ((uint64_t)high << 32));
}

#if __FreeBSD_version >= 1300000
DEFINE_IFUNC(, uint64_t, rdtsc_ordered, (void))
#else
DEFINE_IFUNC(, uint64_t, rdtsc_ordered, (void), static)
#endif
{
	bool cpu_is_amd = cpu_vendor_id == CPU_VENDOR_AMD ||
	    cpu_vendor_id == CPU_VENDOR_HYGON;

	if ((amd_feature & AMDID_RDTSCP) != 0)
		return (rdtscp);
	else if ((cpu_feature & CPUID_SSE2) != 0)
		return (cpu_is_amd ? rdtsc_ordered_mfence :
		    rdtsc_ordered_lfence);
	else
		return (rdtsc);
}
