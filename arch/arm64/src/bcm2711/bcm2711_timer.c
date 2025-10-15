/****************************************************************************
 * arch/arm64/src/bcm2711/bcm2711_timer.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/timers/arch_alarm.h>

#include "arm64_arch_timer.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#define MRS(reg) ({\
    uint64_t _temp;\
    asm volatile("mrs %0, " #reg "\n\r" : "=r"(_temp));\
    _temp;\
})

static uint64_t frequency;
static inline uint64_t arm_arch_timer_count(void)
{
  return MRS(CNTVCT_EL0);
}

static inline uint64_t arm_arch_timer_get_cntfrq(void)
{
  return MRS(CNTFRQ_EL0);
}

#include <stdio.h>

unsigned long get_current_nanosecond(void);
unsigned long get_current_nanosecond()
{
  unsigned long ticks = arm_arch_timer_count();
  unsigned long ret = (double) ticks * ((double) 1000000000.0 / (double)frequency);
  //printf("ticks: %lu; frequency: %lu; ret: %lu\n", ticks, frequency, ret);
  return ret;
}

unsigned long get_affinity(void);
unsigned long get_affinity(void)
{
  return MRS(MPIDR_EL1);
}

int get_current_timer_nanoseconds(clockid_t, struct timespec *time)
{
  uint64_t nanoseconds = get_current_nanosecond();
  time->tv_sec = nanoseconds / 1000000000UL;
  time->tv_nsec = nanoseconds % 1000000000UL;

  //printf("nanoseconds: %ld; time->tv_sec: %ld; time->tv_nsec: %ld\n", nanoseconds, time->tv_sec, time->tv_nsec);

  return 0;
}

void up_timer_initialize(void)
{
  frequency = arm_arch_timer_get_cntfrq();
  up_alarm_set_lowerhalf(arm64_oneshot_initialize());
}

void arm64_timer_secondary_init(void)
{
  arm64_oneshot_secondary_init();
}
