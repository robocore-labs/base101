/*
 * zenoh-pico system layer for the Axon board (RP2350, bare metal).
 *
 * Single-threaded (Z_FEATURE_MULTI_THREAD=0): no task/mutex/condvar
 * implementations are required, the core compiles them out.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/rand.h"
#include "pico/time.h"

#include "zenoh-pico/config.h"
#include "zenoh-pico/system/platform.h"
#include "zenoh-pico/utils/result.h"

#include "axon_zenoh.h"

static axon_idle_fn s_idle_poll = NULL;

void axon_zenoh_set_idle_poll(axon_idle_fn fn) { s_idle_poll = fn; }

void axon_zenoh_idle_poll(void) {
    if (s_idle_poll != NULL) {
        s_idle_poll();
    }
}

/*------------------ Random ------------------*/
uint8_t z_random_u8(void) { return (uint8_t)get_rand_32(); }
uint16_t z_random_u16(void) { return (uint16_t)get_rand_32(); }
uint32_t z_random_u32(void) { return get_rand_32(); }
uint64_t z_random_u64(void) { return get_rand_64(); }

void z_random_fill(void *buf, size_t len) {
    uint8_t *p = (uint8_t *)buf;
    for (size_t i = 0; i < len; i++) {
        p[i] = z_random_u8();
    }
}

/*------------------ Memory ------------------*/
void *z_malloc(size_t size) { return (size == 0) ? NULL : malloc(size); }
void *z_realloc(void *ptr, size_t size) { return realloc(ptr, size); }
void z_free(void *ptr) { free(ptr); }

/*------------------ Sleep ------------------*/
// Sleeps keep the rest of the firmware alive through the idle poll.
static z_result_t _axon_sleep_us64(uint64_t us) {
    absolute_time_t deadline = make_timeout_time_us(us);
    while (!time_reached(deadline)) {
        axon_zenoh_idle_poll();
    }
    return _Z_RES_OK;
}

z_result_t z_sleep_us(size_t time) { return _axon_sleep_us64((uint64_t)time); }
z_result_t z_sleep_ms(size_t time) { return _axon_sleep_us64((uint64_t)time * 1000u); }
z_result_t z_sleep_s(size_t time) { return _axon_sleep_us64((uint64_t)time * 1000000u); }

/*------------------ Clock (monotonic, since boot) ------------------*/
static uint64_t _clock_to_us(const z_clock_t *c) {
    return (uint64_t)c->tv_sec * 1000000u + (uint64_t)(c->tv_nsec / 1000);
}

z_clock_t z_clock_now(void) {
    uint64_t us = time_us_64();
    z_clock_t c = {
        .tv_sec = (int64_t)(us / 1000000u),
        .tv_nsec = (int64_t)((us % 1000000u) * 1000u),
    };
    return c;
}

unsigned long zp_clock_elapsed_us_since(z_clock_t *instant, z_clock_t *epoch) {
    (void)epoch;
    return (unsigned long)(time_us_64() - _clock_to_us(instant));
}

unsigned long zp_clock_elapsed_ms_since(z_clock_t *instant, z_clock_t *epoch) {
    return zp_clock_elapsed_us_since(instant, epoch) / 1000u;
}

unsigned long zp_clock_elapsed_s_since(z_clock_t *instant, z_clock_t *epoch) {
    return zp_clock_elapsed_us_since(instant, epoch) / 1000000u;
}

unsigned long z_clock_elapsed_us(z_clock_t *time) {
    return (unsigned long)(time_us_64() - _clock_to_us(time));
}
unsigned long z_clock_elapsed_ms(z_clock_t *time) { return z_clock_elapsed_us(time) / 1000u; }
unsigned long z_clock_elapsed_s(z_clock_t *time) { return z_clock_elapsed_us(time) / 1000000u; }

static void _clock_advance_ns(z_clock_t *clock, uint64_t ns) {
    clock->tv_nsec += (int64_t)(ns % 1000000000u);
    clock->tv_sec += (int64_t)(ns / 1000000000u);
    if (clock->tv_nsec >= 1000000000) {
        clock->tv_nsec -= 1000000000;
        clock->tv_sec += 1;
    }
}

void z_clock_advance_us(z_clock_t *clock, unsigned long duration) {
    _clock_advance_ns(clock, (uint64_t)duration * 1000u);
}
void z_clock_advance_ms(z_clock_t *clock, unsigned long duration) {
    _clock_advance_ns(clock, (uint64_t)duration * 1000000u);
}
void z_clock_advance_s(z_clock_t *clock, unsigned long duration) {
    _clock_advance_ns(clock, (uint64_t)duration * 1000000000u);
}

/*------------------ Time ------------------*/
// No RTC: wall time == time since boot. Good enough for timestamps and
// rate measurement; the ROS host re-stamps where it matters.
z_time_t z_time_now(void) { return time_us_64(); }

const char *z_time_now_as_str(char *const buf, unsigned long buflen) {
    uint64_t us = time_us_64();
    snprintf(buf, buflen, "%llu.%06llu", (unsigned long long)(us / 1000000u),
             (unsigned long long)(us % 1000000u));
    return buf;
}

unsigned long z_time_elapsed_us(z_time_t *time) { return (unsigned long)(time_us_64() - *time); }
unsigned long z_time_elapsed_ms(z_time_t *time) { return z_time_elapsed_us(time) / 1000u; }
unsigned long z_time_elapsed_s(z_time_t *time) { return z_time_elapsed_us(time) / 1000000u; }

z_result_t _z_get_time_since_epoch(_z_time_since_epoch *t) {
    uint64_t us = time_us_64();
    t->secs = (uint32_t)(us / 1000000u);
    t->nanos = (uint32_t)((us % 1000000u) * 1000u);
    return _Z_RES_OK;
}

/*------------------ Socket stubs ------------------*/
// No IP links in this build; these exist only to satisfy the linker if
// some shared code path references them.
z_result_t _z_socket_set_blocking(const _z_sys_net_socket_t *sock, bool blocking) {
    (void)sock;
    (void)blocking;
    return _Z_RES_OK;
}

z_result_t _z_ip_port_to_endpoint(const uint8_t *address, size_t address_len, uint16_t port, char *dst,
                                  size_t dst_len) {
    (void)address;
    (void)address_len;
    (void)port;
    (void)dst;
    (void)dst_len;
    return _Z_ERR_GENERIC;
}

z_result_t _z_socket_get_endpoints(const _z_sys_net_socket_t *sock, char *local, size_t local_len, char *remote,
                                   size_t remote_len) {
    (void)sock;
    (void)local;
    (void)local_len;
    (void)remote;
    (void)remote_len;
    return _Z_ERR_GENERIC;
}

void _z_socket_close(_z_sys_net_socket_t *sock) { sock->_open = false; }
