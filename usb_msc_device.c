#include "bsp/board.h"
#include "tusb.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <hardware/flash.h>
#include <hardware/sync.h>
#include <hardware/pwm.h>
#include <pico/platform.h>
#include <pico/stdlib.h>

// PicoRapidX2GR専用 フラッシュアドレス
#define FLASH_ADDR_IO_SETTING   0x1C1000  // (旧来、GRでは非使用)
#define FLASH_ADDR_BTN_SETTING  0x1C3000  // GRボタン設定

// GRボタンモード定義 (PicoRapidX2GR.c と同じ値)
#define _GR_BTN_MODE_DISABLED     0
#define _GR_BTN_MODE_HOLD         1
#define _GR_BTN_MODE_RAPID_ROTARY 2
#define _GR_BTN_MODE_RAPID_FIXED  3

// Setting.txt のみの簡素なFAT12ディスク
// セクタ構成:
//   0   : ブートレコード
//   1   : FAT1
//   2   : FAT2
//   3   : ルートディレクトリ (1セクタ = 16エントリ)
//   4   : Setting.txt データ (クラスタ2)
#define DISK_SECTOR_SIZE  512
#define DISK_SECTOR_COUNT 8
#define ROOT_DIR_SECTOR   3
#define DATA_START_SECTOR 4

static uint8_t ram_disk[DISK_SECTOR_SIZE * DISK_SECTOR_COUNT];

static bool     s_write_processed          = false;
static uint8_t  s_write_buffer[DISK_SECTOR_SIZE * 2];
static uint32_t s_write_len                = 0;
static bool     s_led_blinking             = false;
static uint32_t s_file_size_hint           = 0;
static uint32_t s_last_write_ms            = 0;
static bool     s_preseeded_buffer         = false;
static uint32_t s_blink_hold_until_ms      = 0;
// 完了後パターン点滅の状態
static bool     s_post_blink_active        = false;
static uint32_t s_post_blink_next_ms       = 0;
static int      s_post_blink_remaining_toggles = 0;
static bool     s_led_manual_state         = false;
// 初回の実データ書込みまで点滅を完全抑止
static bool     s_led_suppress_until_write = true;

// ---- Output label helpers -----------------------------------------------
// GPIO番号 → OUTPUTラベル文字列
static const char* gpio_to_output_label(uint8_t g) {
    if (g == 18) return "A";
    if (g == 17) return "B";
    if (g == 16) return "C";
    if (g == 28) return "RESET";
    if (g == 27) return "START";
    return "A"; // フォールバック
}

// OUTPUTラベル文字列 → GPIO番号 (不明なら0xFF=デフォルト使用)
static uint8_t output_label_to_gpio(const char *label) {
    if (label[0] == 'A' || label[0] == 'a') return 18;
    if (label[0] == 'B' || label[0] == 'b') return 17;
    if (label[0] == 'C' || label[0] == 'c') return 16;
    // RESET / reset
    if ((label[0] == 'R' || label[0] == 'r') &&
        (label[1] == 'E' || label[1] == 'e')) return 28;
    // START / start
    if ((label[0] == 'S' || label[0] == 's') &&
        (label[1] == 'T' || label[1] == 't')) return 27;
    return 0xFF; // 不明
}

// ---- Validation and deletion helpers --------------------------------------
// GR button config header check: "INPUT,OUTPUT,MODE,RAPID" が含まれること
static bool has_expected_header_n(const char *data, size_t len) {
    if (!data || len == 0) return false;
    const char *needle = "INPUT,OUTPUT,MODE,RAPID";
    size_t n = strlen(needle);
    for (size_t i = 0; i + n <= len; i++) {
        if (memcmp(data + i, needle, n) == 0) return true;
    }
    return false;
}

// GR button config データ行の検証:
//   - 先頭が 'A'〜'F' または 'a'〜'f'
//   - カンマ区切り4フィールド (INPUT,OUTPUT,MODE,RAPID)
static bool validate_settings_lines_n(const char *data, size_t len) {
    if (!data || len == 0) return false;
    const char *p = data;
    const char *end = data + len;
    int valid_rows = 0;
    for (int line = 0; line < 32 && p < end; line++) {
        const char *line_start = p;
        while (p < end && *p != '\n' && *p != '\r') p++;
        const char *line_end = p;
        while (p < end && (*p == '\r' || *p == '\n')) p++;

        // trim
        while (line_start < line_end && (*line_start == ' ' || *line_start == '\t')) line_start++;
        while (line_end > line_start && (line_end[-1] == ' ' || line_end[-1] == '\t')) line_end--;
        if (line_start >= line_end) continue;

        // '#' で始まる行はコメント
        if (line_start[0] == '#') continue;

        // データ行は 'A'〜'F' または 'a'〜'f' で始まる
        char c = line_start[0];
        bool is_slot = (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f');
        if (!is_slot) continue;

        // コンマが3つあること (4フィールド)
        int commas = 0;
        for (const char *q = line_start; q < line_end; ++q) if (*q == ',') commas++;
        if (commas != 3) return false;
        valid_rows++;
    }
    return valid_rows > 0;
}

static void delete_setting_file_from_ramdisk(void) {
    // ルートエントリを削除扱いに
    uint8_t *root = ram_disk + DISK_SECTOR_SIZE * ROOT_DIR_SECTOR;
    root[0] = 0xE5; // deleted mark
    // FATのクラスタ2を解放
    uint8_t *fat1 = ram_disk + DISK_SECTOR_SIZE * 1;
    uint8_t *fat2 = ram_disk + DISK_SECTOR_SIZE * 3;
    fat1[3] = 0x00; fat1[4] = 0x00;
    fat2[3] = 0x00; fat2[4] = 0x00;
    // ファイルサイズも0に
    memset(root + 28, 0, 4);
}

// LED blink helpers (PWM-based, ~8 Hz, 50% duty)
static inline void led_blink_start(void) {
    const uint LED_PIN = 25;
    gpio_set_function(LED_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(LED_PIN);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 256.0f);
    pwm_config_set_wrap(&cfg, 61035); // ~8 Hz at 125 MHz
    pwm_init(slice, &cfg, true);
    pwm_set_gpio_level(LED_PIN, 61035 / 2);
}

static inline void led_blink_stop(void) {
    const uint LED_PIN = 25;
    uint slice = pwm_gpio_to_slice_num(LED_PIN);
    pwm_set_enabled(slice, false);
    gpio_set_function(LED_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);
}

static inline void led_on(void) {
    const uint LED_PIN = 25;
    gpio_set_function(LED_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);
}

static inline void led_off(void) {
    const uint LED_PIN = 25;
    gpio_set_function(LED_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);
}

static inline void led_toggle(void) {
    s_led_manual_state = !s_led_manual_state;
    if (s_led_manual_state) led_on(); else led_off();
}

static inline void start_post_blink_pattern(void) {
    if (s_led_suppress_until_write) {
        return; // 接続直後など、実書込み前は一切点滅しない
    }
    // PWM点滅中なら停止し、手動トグルで3回点滅に移行
    if (s_led_blinking) {
        led_blink_stop();
        s_led_blinking = false;
    }
    s_post_blink_active = true;
    s_post_blink_remaining_toggles = 6; // ON/OFF 6回 = 3回点滅
    s_led_manual_state = false; // 消灯から開始
    led_off();
    s_post_blink_next_ms = to_ms_since_boot(get_absolute_time()); // 直ちに1回目のトグル
}

static void build_fat12_image(void) {
    memset(ram_disk, 0, sizeof(ram_disk));

    // ---- ブートセクタ (セクタ0) ----
    uint8_t *b = ram_disk;
    b[0] = 0xEB; b[1] = 0x3C; b[2] = 0x90;
    memcpy(&b[3], "MSDOS5.0", 8);
    uint16_t bps = DISK_SECTOR_SIZE;   memcpy(&b[11], &bps, 2);
    b[13] = 1;                          // sectors per cluster
    uint16_t rsv = 1;   memcpy(&b[14], &rsv, 2);
    b[16] = 2;                          // number of FATs
    uint16_t rents = 16; memcpy(&b[17], &rents, 2);
    uint16_t tots = DISK_SECTOR_COUNT; memcpy(&b[19], &tots, 2);
    b[21] = 0xF8;
    uint16_t spf = 1;   memcpy(&b[22], &spf, 2);
    uint16_t spt = 1;   memcpy(&b[24], &spt, 2);
    uint16_t hds = 1;   memcpy(&b[26], &hds, 2);
    b[36] = 0x29;
    memcpy(&b[43], "NO NAME    ", 11);
    memcpy(&b[54], "FAT12   ", 8);

    // ---- FAT1 (セクタ1) ----
    uint8_t *fat1 = ram_disk + DISK_SECTOR_SIZE * 1;
    fat1[0] = 0xF8; fat1[1] = 0xFF; fat1[2] = 0xFF;
    // クラスタ2 = Setting.txt (EOC = 0xFFF)
    fat1[3] = 0xFF; fat1[4] = (fat1[4] & 0xF0) | 0x0F;

    // ---- FAT2 (セクタ2) ----
    memcpy(ram_disk + DISK_SECTOR_SIZE * 2, fat1, DISK_SECTOR_SIZE);

    // ---- ルートディレクトリ (セクタ3) ----
    uint8_t *root = ram_disk + DISK_SECTOR_SIZE * ROOT_DIR_SECTOR;
    memcpy(root, "SETTING TXT", 11);
    root[11] = 0x20;
    uint16_t sc = 2; memcpy(root + 26, &sc, 2);

    // ---- Setting.txt データ (セクタ4) / GRボタン設定 ----
    char *text = (char *)(ram_disk + DISK_SECTOR_SIZE * DATA_START_SECTOR);
    memset(text, 0, DISK_SECTOR_SIZE);
    int pos = 0;

    // デフォルト値テーブル (LoadButtonConfig と同じ)
    static const struct { uint8_t mode; uint8_t gpio; uint8_t rapid_off; } btn_default[6] = {
        {_GR_BTN_MODE_HOLD,         18, 1},
        {_GR_BTN_MODE_HOLD,         17, 1},
        {_GR_BTN_MODE_HOLD,         16, 1},
        {_GR_BTN_MODE_RAPID_ROTARY, 18, 1},
        {_GR_BTN_MODE_HOLD,         17, 1},
        {_GR_BTN_MODE_HOLD,         16, 1},
    };
    const char *slot_names[6] = {
        "TOP_L", "TOP_M", "TOP_R", "BTM_L", "BTM_M", "BTM_R"
    };
    // フラッシュから現在の設定を読む
    const uint8_t *fp = (const uint8_t *)(XIP_BASE + FLASH_ADDR_BTN_SETTING);

    // ヘッダコメント
    pos += snprintf(text + pos, DISK_SECTOR_SIZE - pos,
        "# PicoRapidX2GR Button Configuration\r\n"
        "# INPUT : A=TOP_L  B=TOP_M  C=TOP_R  D=BTM_L  E=BTM_M  F=BTM_R\r\n"
        "# OUTPUT: A=GP18   B=GP17   C=GP16   RESET=GP28  START=GP27\r\n"
        "# MODE  : 0=DISABLED  1=HOLD  2=RAPID_ROTARY  3=RAPID_FIXED\r\n"
        "# RAPID : 1=8.6/s  2=10/s  3=12/s  4=15/s  5=20/s  6=30/s\r\n"
        "#         (RAPID_ROTARY uses rotary switch, RAPID field ignored)\r\n"
        "# INPUT,OUTPUT,MODE,RAPID\r\n"
        "#\r\n");

    for (int i = 0; i < 6 && pos < DISK_SECTOR_SIZE - 40; i++) {
        uint8_t mode      = fp[i * 3 + 0];
        uint8_t gpio_pin  = fp[i * 3 + 1];
        uint8_t rapid_off = fp[i * 3 + 2];
        // 未初期化(0xFF)はデフォルト値を使用
        if (mode > _GR_BTN_MODE_RAPID_FIXED || mode == 0xFF) {
            mode      = btn_default[i].mode;
            gpio_pin  = btn_default[i].gpio;
            rapid_off = btn_default[i].rapid_off;
        } else {
            if (gpio_pin > 28  || gpio_pin  == 0xFF) gpio_pin  = btn_default[i].gpio;
            if (rapid_off == 0 || rapid_off  > 6 || rapid_off == 0xFF) rapid_off = 1;
        }
        // rapid_off(1-6) → RAPID段階(1-6): RAPID = 7 - rapid_off
        int rapid_level = 7 - (int)rapid_off;  // 1→6(8.6/s)...6→1(30/s) の逆
        if (rapid_level < 1 || rapid_level > 6) rapid_level = 3;
        pos += snprintf(text + pos, DISK_SECTOR_SIZE - pos,
            "%c,%s,%d,%d  # %s\r\n",
            'A' + i, gpio_to_output_label(gpio_pin), mode, rapid_level, slot_names[i]);
    }

    uint32_t fsize = (uint32_t)pos;
    memcpy(root + 28, &fsize, 4);
}

// TinyUSB MSC callbacks ------------------------------------------------------
void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4]) {
    (void) lun;
    memcpy(vendor_id,  "PICO   ", 8);
    const char prod[] = "RAM DISK MSC";
    memset(product_id, ' ', 16);
    memcpy(product_id, prod, sizeof(prod)-1);
    memcpy(product_rev, "1.0", 4);
}

bool tud_msc_test_unit_ready_cb(uint8_t lun) { (void) lun; return true; }

void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size) {
    (void) lun; *block_count = DISK_SECTOR_COUNT; *block_size = DISK_SECTOR_SIZE;
}

bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject) {
    (void) lun; (void) power_condition; (void) start; (void) load_eject; return true;
}

int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize) {
    (void) lun;
    if (lba >= DISK_SECTOR_COUNT) return -1;
    uint32_t available = DISK_SECTOR_SIZE - offset;
    if (bufsize > available) bufsize = available;
    memcpy(buffer, ram_disk + lba * DISK_SECTOR_SIZE + offset, bufsize);
    return (int32_t) bufsize;
}

// GRボタン設定CSVをパースしてフラッシュに書き込む
// 形式: INPUT,OUTPUT,MODE,RAPID  (行末の #コメントは無視)
//   INPUT : A-F (スロット0-5)
//   OUTPUT: A(GP18) B(GP17) C(GP16) RESET(GP28) START(GP27)
//   MODE  : 0=DISABLED 1=HOLD 2=RAPID_ROTARY 3=RAPID_FIXED
//   RAPID : 1(8.6/s)〜6(30/s) → rapid_off = 7 - RAPID
static void parse_settings_csv_n(const char *csv_data, size_t csv_len) {
    // デフォルト値テーブル
    static const struct { uint8_t mode; uint8_t gpio; uint8_t rapid_off; } btn_default[6] = {
        {_GR_BTN_MODE_HOLD,         18, 1},
        {_GR_BTN_MODE_HOLD,         17, 1},
        {_GR_BTN_MODE_HOLD,         16, 1},
        {_GR_BTN_MODE_RAPID_ROTARY, 18, 1},
        {_GR_BTN_MODE_HOLD,         17, 1},
        {_GR_BTN_MODE_HOLD,         16, 1},
    };

    // 現在のフラッシュ内容をベースにし、書き込み済みスロットだけ上書き
    uint8_t new_settings[FLASH_PAGE_SIZE];
    memset(new_settings, 0xFF, sizeof(new_settings));
    const uint8_t *fp = (const uint8_t *)(XIP_BASE + FLASH_ADDR_BTN_SETTING);
    memcpy(new_settings, fp, 18);  // 既存の18バイトをコピー

    const char *p = csv_data;
    const char *end = csv_data + csv_len;

    while (p < end) {
        const char *line_start = p;
        while (p < end && *p != '\r' && *p != '\n') p++;
        const char *line_end = p;
        while (p < end && (*p == '\r' || *p == '\n')) p++;

        // trim
        while (line_start < line_end && (*line_start == ' ' || *line_start == '\t')) line_start++;
        while (line_end > line_start && (line_end[-1] == ' ' || line_end[-1] == '\t')) line_end--;
        if (line_start >= line_end) continue;

        // '#' コメント行・空行をスキップ
        if (line_start[0] == '#') continue;
        // データ行は 'A'〜'F' または 'a'〜'f' で始まる
        char first = line_start[0];
        bool is_slot_char = (first >= 'A' && first <= 'F') || (first >= 'a' && first <= 'f');
        if (!is_slot_char) continue;

        // バッファにコピーし '#' 以降を切り捨て
        size_t line_len = (size_t)(line_end - line_start);
        char buf[64];
        if (line_len >= sizeof(buf)) line_len = sizeof(buf) - 1;
        memcpy(buf, line_start, line_len);
        buf[line_len] = '\0';
        for (int k = 0; buf[k]; k++) { if (buf[k] == '#') { buf[k] = '\0'; break; } }

        // INPUT(A-F) をスロット番号(0-5)に変換
        char *s = buf;
        char *f0 = s; while (*f0 == ' ' || *f0 == '\t') f0++;
        char slot_char = f0[0];
        int slot = (slot_char >= 'a') ? (slot_char - 'a') : (slot_char - 'A');
        if (slot < 0 || slot > 5) continue;
        // 最初のカンマまで進める
        while (*s && *s != ',') s++;
        if (*s != ',') continue;
        s++; // カンマの次へ

        // 4フィールドを文字列のまま分割: OUTPUT, MODE, RAPID
        char *fields[3] = {NULL, NULL, NULL};
        int field_count = 0;
        char *fp_field = s;
        for (int k = 0; s[k] != '\0' && field_count < 3; k++) {
            if (s[k] == ',') {
                s[k] = '\0';
                char *f = fp_field; while (*f == ' ' || *f == '\t') f++;
                fields[field_count++] = f;
                fp_field = &s[k + 1];
            }
        }
        // 最後のフィールド
        if (field_count < 3) {
            char *f = fp_field; while (*f == ' ' || *f == '\t') f++;
            fields[field_count++] = f;
        }
        if (field_count < 3 || !fields[0] || !fields[1] || !fields[2]) continue;

        // OUTPUT ラベル → GPIO番号
        uint8_t gpio_pin = output_label_to_gpio(fields[0]);
        if (gpio_pin == 0xFF) gpio_pin = btn_default[slot].gpio; // 不明はデフォルト

        // MODE (0-3)
        int mode = atoi(fields[1]);
        if (mode < 0 || mode > 3) mode = (int)btn_default[slot].mode;

        // RAPID 1-6 → rapid_off = 7 - RAPID
        int rapid_level = atoi(fields[2]);
        int rapid_off;
        if (rapid_level >= 1 && rapid_level <= 6) {
            rapid_off = 7 - rapid_level;  // 1→6, 2→5, 3→4, 4→3, 5→2, 6→1
        } else {
            rapid_off = (int)btn_default[slot].rapid_off;
        }

        new_settings[slot * 3 + 0] = (uint8_t)mode;
        new_settings[slot * 3 + 1] = gpio_pin;
        new_settings[slot * 3 + 2] = (uint8_t)rapid_off;
    }

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_ADDR_BTN_SETTING, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_ADDR_BTN_SETTING, new_settings, FLASH_PAGE_SIZE);
    restore_interrupts(ints);

    __dsb();
    __isb();
    sleep_us(1000);
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize) {
    (void) lun;
    if (lba >= DISK_SECTOR_COUNT) return -1;
    uint32_t available = DISK_SECTOR_SIZE - offset;
    if (bufsize > available) bufsize = available;
    memcpy(ram_disk + DISK_SECTOR_SIZE * lba + offset, buffer, bufsize);

    // ルートディレクトリ更新時にファイルサイズヒントを取得
    if (lba == ROOT_DIR_SECTOR) {
        uint8_t *root = ram_disk + DISK_SECTOR_SIZE * ROOT_DIR_SECTOR;
        if (root[0] != 0x00 && root[0] != 0xE5 && memcmp(root, "SETTING TXT", 11) == 0) {
            uint32_t fsz = 0; memcpy(&fsz, root + 28, 4);
            s_file_size_hint = fsz;
        }
    }

    // Setting.txt のデータセクタ書込み (セクタ4)
    if (lba == DATA_START_SECTOR) {
        if (!s_led_blinking) {
            s_post_blink_active = false;
            s_post_blink_remaining_toggles = 0;
            led_off();
            s_led_suppress_until_write = false;
            led_blink_start();
            s_led_blinking = true;
            s_write_processed = false;
            s_write_len = 0;
            s_preseeded_buffer = false;
            // 既存内容をプリシード
            uint32_t preload_len = s_file_size_hint;
            if (preload_len == 0 || preload_len > sizeof(s_write_buffer)) preload_len = sizeof(s_write_buffer);
            memcpy(s_write_buffer, ram_disk + DISK_SECTOR_SIZE * DATA_START_SECTOR, preload_len);
            s_write_len = preload_len;
            s_preseeded_buffer = true;
        }
        s_last_write_ms = to_ms_since_boot(get_absolute_time());
        uint32_t pos = offset;
        if (pos < sizeof(s_write_buffer)) {
            uint32_t cpy = bufsize;
            if (pos + cpy > sizeof(s_write_buffer)) cpy = sizeof(s_write_buffer) - pos;
            memcpy(s_write_buffer + pos, buffer, cpy);
            if (pos + cpy > s_write_len) s_write_len = pos + cpy;
        }
    }

    return (int32_t) bufsize;
}

// 書き込み完了時に検証・適用を行う
void tud_msc_write10_complete_cb(uint8_t lun) {
    (void)lun;
    // Setting.txt の書き込み完了処理
    if (!s_write_processed) {
        uint8_t *root = ram_disk + DISK_SECTOR_SIZE * ROOT_DIR_SECTOR;
        if (root[0] != 0x00 && root[0] != 0xE5 && memcmp(root, "SETTING TXT", 11) == 0) {
            uint32_t fsz = 0; memcpy(&fsz, root + 28, 4);
            if (fsz > 0) {
                uint32_t sz = fsz;
                if (sz > sizeof(s_write_buffer)) sz = sizeof(s_write_buffer);
                memcpy(s_write_buffer, ram_disk + DISK_SECTOR_SIZE * DATA_START_SECTOR, sz);
                parse_settings_csv_n((const char*)s_write_buffer, sz);
                s_write_processed = true;
                start_post_blink_pattern();
            }
        }
    }
}

bool tud_msc_is_writable_cb (uint8_t lun) { (void) lun; return true; }

int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void* buffer, uint16_t bufsize) {
    (void) lun; (void) scsi_cmd; (void) buffer; (void) bufsize; return 0;
}

// Public helpers for main app ------------------------------------------------
bool usb_msc_is_connected(void) { return tud_connected(); }
bool usb_msc_is_mounted(void) { return tud_mounted(); }

// USB状態の文字列を取得（デバッグ用）
const char* usb_msc_get_status_string(void) {
    if (!tud_inited()) return "NotInit";
    if (!tud_connected()) return "NotConn"; 
    if (!tud_mounted()) return "NotMount";
    return "Mounted";
}

void usb_msc_start(void) {
    static bool s_inited = false;
    build_fat12_image();
    s_write_processed  = false;
    s_led_blinking     = false;
    s_write_len        = 0;
    s_file_size_hint   = 0;
    s_last_write_ms    = 0;
    s_preseeded_buffer = false;
    s_blink_hold_until_ms = 0;
    s_post_blink_active   = false;
    s_post_blink_remaining_toggles = 0;
    s_led_manual_state    = false;
    s_led_suppress_until_write = true;
    led_blink_stop();
    led_off();
    memset(s_write_buffer, 0, sizeof(s_write_buffer));
    if (!s_inited) { board_init(); s_inited = true; }
    tud_disconnect();
    sleep_ms(100);
    tud_init(0);
    tud_connect();
}

void usb_msc_task(void) {
    tud_task();

    uint32_t now = to_ms_since_boot(get_absolute_time());

    // タイムアウトフォールバック: 書込みが途絶えて1.5秒以上なら完了処理
    if (s_led_blinking && !s_write_processed && s_write_len > 0 && s_last_write_ms != 0 && (now - s_last_write_ms) > 1500) {
        // タイムアウト時も即適用（パーサが安全側で取り込み）
        parse_settings_csv_n((const char*)s_write_buffer, s_write_len);
        s_write_processed = true;
        start_post_blink_pattern();
    }

    // 完了後のパターン点滅（0.5秒間隔で3回）
    if (s_post_blink_active && s_post_blink_remaining_toggles > 0 && now >= s_post_blink_next_ms) {
        led_toggle();
        s_post_blink_remaining_toggles--;
        s_post_blink_next_ms = now + 500; // 0.5秒ごとにトグル
        if (s_post_blink_remaining_toggles == 0) {
            led_off();
            s_post_blink_active = false;
        }
    }
}

// Optional: visibility on host state
void tud_mount_cb(void) {
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    // MSC接続中はLED消灯
    led_blink_stop();
    led_off();
    s_led_blinking = false;
    s_blink_hold_until_ms = 0;
    s_post_blink_active = false;
    s_post_blink_remaining_toggles = 0;
    s_led_manual_state = false;
    s_led_suppress_until_write = true;
}

void tud_umount_cb(void) {
    // アンマウント時も消灯を維持
    led_blink_stop();
    led_off();
    s_led_blinking = false;
    s_blink_hold_until_ms = 0;
    s_post_blink_active = false;
    s_post_blink_remaining_toggles = 0;
    s_led_manual_state = false;
    s_led_suppress_until_write = true;
}
