#include "cfg_console.h"

#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "tusb.h"

#include "usb_descriptors.h"
#include "../dbg.h"
#include "axon_cfg.h"
#include "motor_test.h"

#define LINE_MAX 1024
static char line[LINE_MAX];
static uint32_t line_len = 0;

// ---------------------------------------------------------------------------
// Minimal JSON scanner — only what this command schema needs.
// ---------------------------------------------------------------------------
static const char *skipws(const char *p) {
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    return p;
}

// Parse a JSON string token (p at the opening quote). Copies into out (may be
// NULL to just skip). Returns the cursor past the closing quote, or NULL.
static const char *parse_str(const char *p, char *out, size_t n) {
    p = skipws(p);
    if (*p != '"') return NULL;
    p++;
    size_t i = 0;
    while (*p && *p != '"') {
        char c = *p++;
        if (c == '\\' && *p) {
            char e = *p++;
            c = (e == 'n') ? '\n' : (e == 't') ? '\t' : e;
        }
        if (out && i + 1 < n) out[i++] = c;
    }
    if (*p != '"') return NULL;
    p++;
    if (out) out[i] = '\0';
    return p;
}

static const char *skip_value(const char *p);

static const char *skip_value(const char *p) {
    p = skipws(p);
    if (*p == '"') return parse_str(p, NULL, 0);
    if (*p == '{' || *p == '[') {
        char close = (*p == '{') ? '}' : ']';
        p++;
        p = skipws(p);
        if (*p == close) return p + 1;
        for (;;) {
            if (close == '}') {
                p = parse_str(p, NULL, 0);  // key
                if (!p) return NULL;
                p = skipws(p);
                if (*p != ':') return NULL;
                p++;
            }
            p = skip_value(p);
            if (!p) return NULL;
            p = skipws(p);
            if (*p == ',') { p++; continue; }
            if (*p == close) return p + 1;
            return NULL;
        }
    }
    // primitive: number / true / false / null
    while (*p && *p != ',' && *p != '}' && *p != ']' &&
           *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') p++;
    return p;
}

// In the object at obj (cursor at or before '{'), return the cursor at the
// value for `key`, or NULL if absent.
static const char *obj_find(const char *obj, const char *key) {
    const char *p = skipws(obj);
    if (*p != '{') return NULL;
    p++;
    p = skipws(p);
    if (*p == '}') return NULL;
    for (;;) {
        char k[AXON_JOINT_NAME_LEN + 8];
        const char *kp = parse_str(p, k, sizeof(k));
        if (!kp) return NULL;
        p = skipws(kp);
        if (*p != ':') return NULL;
        p++;
        const char *val = skipws(p);
        if (strcmp(k, key) == 0) return val;
        p = skip_value(val);
        if (!p) return NULL;
        p = skipws(p);
        if (*p == ',') { p = skipws(p + 1); continue; }
        return NULL;  // hit '}' or malformed
    }
}

static bool val_is_true(const char *v) {
    return v && skipws(v)[0] == 't';
}

static long val_int(const char *v) {
    return v ? strtol(skipws(v), NULL, 10) : 0;
}

static double val_num(const char *v) {
    return v ? strtod(skipws(v), NULL) : 0.0;
}

// ---------------------------------------------------------------------------
// Command handling
// ---------------------------------------------------------------------------
static void emit_config(void) {
    dbg_printf("{\"servos_enabled\":%s,\"servo_count\":%u,\"servos\":[",
               g_cfg.servos_enabled ? "true" : "false", g_cfg.servo_count);
    for (uint8_t i = 0; i < g_cfg.servo_count; i++) {
        dbg_printf("%s{\"id\":%u,\"joint\":\"%s\",\"enable\":%s}",
                   i ? "," : "",
                   g_cfg.servos[i].id, g_cfg.servos[i].joint,
                   g_cfg.servos[i].enable ? "true" : "false");
    }
    dbg_printf("]}\n");
}

static void apply_set(const char *json) {
    const char *se = obj_find(json, "servos_enabled");
    if (se) {
        g_cfg.servos_enabled = val_is_true(se);
    }

    const char *arr = obj_find(json, "servos");
    if (arr) {
        arr = skipws(arr);
        if (*arr != '[') {
            dbg_printf("{\"ok\":false,\"msg\":\"servos must be an array\"}\n");
            return;
        }
        arr = skipws(arr + 1);
        uint8_t count = 0;
        while (*arr && *arr != ']' && count < AXON_SERVO_MAX) {
            const char *elem = arr;  // object cursor

            const char *idv = obj_find(elem, "id");
            const char *jnv = obj_find(elem, "joint");
            const char *env = obj_find(elem, "enable");

            axon_servo_cfg_t *s = &g_cfg.servos[count];
            memset(s, 0, sizeof(*s));
            s->id = idv ? (uint8_t)atoi(skipws(idv)) : 0;
            s->enable = env ? val_is_true(env) : true;
            if (jnv) {
                parse_str(jnv, s->joint, sizeof(s->joint));
            } else {
                snprintf(s->joint, sizeof(s->joint), "%u", s->id);
            }
            count++;

            arr = skip_value(arr);
            if (!arr) break;
            arr = skipws(arr);
            if (*arr == ',') { arr = skipws(arr + 1); continue; }
            break;
        }
        g_cfg.servo_count = count;
    }

    dbg_printf("{\"ok\":true,\"msg\":\"set (save+reboot to apply)\"}\n");
    emit_config();
}

static void process_line(const char *json) {
    char cmd[16] = {0};
    const char *cv = obj_find(json, "cmd");
    if (!cv || !parse_str(cv, cmd, sizeof(cmd))) {
        dbg_printf("{\"ok\":false,\"msg\":\"expected {\\\"cmd\\\":...}\"}\n");
        return;
    }

    if (strcmp(cmd, "get") == 0) {
        emit_config();
    } else if (strcmp(cmd, "set") == 0) {
        apply_set(json);
    } else if (strcmp(cmd, "defaults") == 0) {
        axon_cfg_defaults();
        dbg_printf("{\"ok\":true,\"msg\":\"defaults (save+reboot to apply)\"}\n");
        emit_config();
    } else if (strcmp(cmd, "save") == 0) {
        bool ok = axon_cfg_save();
        dbg_printf("{\"ok\":%s,\"msg\":\"%s\"}\n", ok ? "true" : "false",
                   ok ? "saved" : "save failed");
    } else if (strcmp(cmd, "reboot") == 0) {
        dbg_printf("{\"ok\":true,\"msg\":\"rebooting\"}\n");
        sleep_ms(50);
        watchdog_reboot(0, 0, 0);
        while (true) { tight_loop_contents(); }
    } else if (strcmp(cmd, "motors") == 0) {
        motor_test_list_json();
    } else if (strcmp(cmd, "wheel") == 0) {
        motor_test_wheel((int)val_int(obj_find(json, "index")),
                         val_num(obj_find(json, "rpm")));
    } else if (strcmp(cmd, "servo") == 0) {
        int id = (int)val_int(obj_find(json, "id"));
        const char *posv = obj_find(json, "pos");
        const char *delv = obj_find(json, "delta");
        if (posv) {
            motor_test_servo(id, (int)val_int(posv));
        } else if (delv) {
            motor_test_servo_delta(id, (int)val_int(delv));
        } else {
            dbg_printf("{\"ok\":false,\"msg\":\"servo needs pos or delta\"}\n");
        }
    } else if (strcmp(cmd, "stop") == 0) {
        motor_test_stop_all();
    } else if (strcmp(cmd, "setid") == 0) {
        motor_test_set_servo_id((int)val_int(obj_find(json, "from")),
                                (int)val_int(obj_find(json, "to")));
    } else {
        dbg_printf("{\"ok\":false,\"msg\":\"unknown cmd\"}\n");
    }
}

void cfg_console_task(void) {
    if (!tud_cdc_n_available(CDC_IDX_DEBUG)) {
        return;
    }
    uint8_t buf[64];
    uint32_t n = tud_cdc_n_read(CDC_IDX_DEBUG, buf, sizeof(buf));
    for (uint32_t i = 0; i < n; i++) {
        char c = (char)buf[i];
        if (c == '\r') continue;
        if (c == '\n') {
            line[line_len] = '\0';
            if (line_len > 0) process_line(line);
            line_len = 0;
            continue;
        }
        if (line_len < LINE_MAX - 1) {
            line[line_len++] = c;
        } else {
            line_len = 0;  // overflow: drop the line
        }
    }
}
