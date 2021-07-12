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
 *
 * $FreeBSD$
 */

#ifndef _X86__RDTSC_ORDERED_H_
#define _X86__RDTSC_ORDERED_H_

#include <sys/types.h>
#include <machine/cpufunc.h>
#include <machine/cputypes.h>
#include <machine/specialreg.h>
#include <x86/ifunc.h>
#include <x86/rdtsc_ordered.h>

#ifdef _KERNEL
#include <sys/pcpu.h>
#include <machine/md_var.h>
#else /* !_KERNEL */
#include <stdbool.h>
#include <string.h>
#endif /* _KERNEL */

#define	DEFINE_RDTSC_ORDERED_COMMON()					\
    static uint64_t							\
    rdtsc_ordered_lfence(void)						\
    {									\
    	lfence();							\
    	return (rdtsc());						\
    }									\
    static uint64_t							\
    rdtsc_ordered_mfence(void)						\
    {									\
    	mfence();							\
    	return (rdtsc());						\
    }									\
    static inline uint64_t (*rdtsc_ordered_select(u_int amd_feature,	\
        u_int cpu_feature, bool cpu_is_amd))(void)			\
    {									\
    	if ((amd_feature & AMDID_RDTSCP) != 0)				\
    		return (rdtscp);					\
    	else if ((cpu_feature & CPUID_SSE2) != 0)			\
    		if (cpu_is_amd)						\
    			return (rdtsc_ordered_mfence);			\
    		else							\
    			return (rdtsc_ordered_lfence);			\
    	else								\
    		return (rdtsc);						\
    }

#ifdef _KERNEL
#define	DEFINE_RDTSC_ORDERED()						\
    DEFINE_RDTSC_ORDERED_COMMON()					\
    DEFINE_IFUNC(, uint64_t, rdtsc_ordered, (void))			\
    {									\
    	bool cpu_is_amd = cpu_vendor_id == CPU_VENDOR_AMD ||		\
    	    cpu_vendor_id == CPU_VENDOR_HYGON;				\
    									\
    	return (rdtsc_ordered_select(amd_feature, cpu_feature,		\
    	    cpu_is_amd));						\
    }
#else /* !_KERNEL */
#define	DEFINE_RDTSC_ORDERED()						\
    DEFINE_RDTSC_ORDERED_COMMON()					\
    DEFINE_UIFUNC(, uint64_t, rdtsc_ordered, (void))			\
    {									\
    	u_int amd_feature, cpu_exthigh, p[4], v[3];			\
    	static const char amd_id[] = "AuthenticAMD";			\
    	static const char hygon_id[] = "HygonGenuine";			\
    	bool cpu_is_amd;						\
    									\
    	do_cpuid(0, p);							\
    	v[0] = p[1];							\
    	v[1] = p[3];							\
    	v[2] = p[2];							\
    	cpu_is_amd = memcmp(v, amd_id, sizeof(amd_id) - 1) == 0 ||	\
    	    memcmp(v, hygon_id, sizeof(hygon_id) - 1) == 0;		\
    	if (cpu_feature != 0) {						\
    		do_cpuid(0x80000000, p);				\
    		cpu_exthigh = p[0];					\
    	} else {							\
    		cpu_exthigh = 0;					\
    	}								\
    	if (cpu_exthigh >= 0x80000001) {				\
    		do_cpuid(0x80000001, p);				\
    		amd_feature = p[3];					\
    	} else {							\
    		amd_feature = 0;					\
    	}								\
    	return (rdtsc_ordered_select(amd_feature, cpu_feature,		\
    	    cpu_is_amd));						\
    }
#endif /* _KERNEL */

#endif /* !_X86__RDTSC_ORDERED_H_ */
