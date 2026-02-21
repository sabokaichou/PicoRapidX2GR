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

#define DISK_SECTOR_SIZE 512
#define DISK_SECTOR_COUNT 200  // 13ファイル × 最大8セクタ + 管理領域
static uint8_t ram_disk[DISK_SECTOR_SIZE * DISK_SECTOR_COUNT];

#define ROOT_DIR_SECTOR       9  // ルートディレクトリはセクタ9-10
#define DATA_START_SECTOR     11  // データ領域の開始セクタ
// 書き込み完了後、目視できる時間だけ点滅を維持
#define MIN_BLINK_VISIBLE_MS 600
static bool s_write_processed = false;
static uint8_t s_write_buffer[DISK_SECTOR_SIZE * 8];
static uint32_t s_write_len = 0;
static bool s_led_blinking = false;
static uint32_t s_file_size_hint = 0;
static uint32_t s_last_write_ms = 0;
static bool s_preseeded_buffer = false;
static uint32_t s_blink_hold_until_ms = 0;
// 完了後パターン点滅の状態
static bool s_post_blink_active = false;
static uint32_t s_post_blink_next_ms = 0;
static int s_post_blink_remaining_toggles = 0; // 6トグル=3回点滅
static bool s_led_manual_state = false;
// 初回の実データ書込み(Setting.txtのデータセクタ)まで点滅を完全抑止
static bool s_led_suppress_until_write = true;

// マクロファイル書き込み用のバッファ (各マクロファイル用、最大10セクタ = 5120バイト)
static uint8_t s_macro_write_buffer[DISK_SECTOR_SIZE * 10];
static int s_macro_write_number = -1;  // 現在書き込み中のマクロ番号 (-1 = なし)
static bool s_macro_write_pending = false;

// ---- Validation and deletion helpers --------------------------------------
static bool has_expected_header_n(const char *data, size_t len) {
    if (!data || len == 0) return false;
    const char *needle1 = "Format: INPUT_NO,RAPID_TYPE,REVERSE,OUT_FRAME,IN_FRAME,OUTPUT_PINS";
    const char *needle2 = "RAPID: 1=Norm 2=R30 3=R30Rev 4=Custom 5=Macro 6=R15 7=R15Rev";
    size_t n1 = strlen(needle1), n2 = strlen(needle2);
    bool f1 = false, f2 = false;
    for (size_t i = 0; i + n1 <= len; i++) { if (memcmp(data + i, needle1, n1) == 0) { f1 = true; break; } }
    for (size_t i = 0; i + n2 <= len; i++) { if (memcmp(data + i, needle2, n2) == 0) { f2 = true; break; } }
    return f1 && f2;
}

static bool validate_settings_lines_n(const char *data, size_t len) {
    if (!data || len == 0) return false;
    const char *p = data;
    const char *end = data + len;
    int valid_rows = 0;
    for (int line = 0; line < 64 && p < end; line++) {
        const char *line_start = p;
        while (p < end && *p != '\n' && *p != '\r') p++;
        const char *line_end = p;
        while (p < end && (*p == '\r' || *p == '\n')) p++;

        // trim
        while (line_start < line_end && (*line_start == ' ' || *line_start == '\t')) line_start++;
        while (line_end > line_start && (line_end[-1] == ' ' || line_end[-1] == '\t')) line_end--;
        if (line_start >= line_end) continue; // empty

        // non-data lines (headers/comments) are skipped
        if (!(line_start[0] >= '0' && line_start[0] <= '9')) continue;

        int commas = 0; for (const char *q = line_start; q < line_end; ++q) if (*q == ',') commas++;
        if (commas < 5) return false; // data line must have 5 commas

        // check last field has 12 binary digits (ignore spaces/tabs)
        const char *last_comma = NULL;
        for (const char *q = line_start; q < line_end; ++q) if (*q == ',') last_comma = q;
        if (!last_comma) return false;
        int digits = 0;
        for (const char *q = last_comma + 1; q < line_end; ++q) {
            if (*q == '0' || *q == '1') digits++;
            else if (*q == ' ' || *q == '\t') {/* skip */}
            else return false;
            if (digits > 12) return false; // too many
        }
        if (digits != 12) return false;
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

// マクロデータをテキスト形式で生成するヘルパー関数
static void generate_macro_text(int macro_no, char *buffer, size_t buffer_size) {
    // フラッシュからマクロデータを読み込む
    const uint32_t macro_offsets[] = {
        0x1E0000, 0x1E1000, 0x1E2000, 0x1E3000, 0x1E4000, 0x1E5000,
        0x1E6000, 0x1E7000, 0x1E8000, 0x1E9000, 0x1EA000, 0x1EB000
    };
    
    if (macro_no < 0 || macro_no >= 12 || !buffer || buffer_size == 0) {
        if (buffer && buffer_size > 0) buffer[0] = '\0';
        return;
    }
    
    const uint8_t *flash_ptr = (const uint8_t *)(XIP_BASE + macro_offsets[macro_no]);
    int pos = 0;
    
    // ヘッダー
    pos += snprintf(buffer + pos, buffer_size - pos,
        "Macro %d Data\r\n"
        "Format: FRAME,OUT1,OUT2,OUT3,OUT4,OUT5,OUT6,OUT7,OUT8,OUT9,OUT10,OUT11,OUT12\r\n"
        "\r\n",
        macro_no);
    
    if (pos >= (int)buffer_size - 10) return;
    
    // マクロデータは4096バイト (1セクタ)
    // 256フレーム × 16バイト = 4096バイト
    // 各フレームの16バイト: バイト0=有効フラグ、バイト1-15=出力1-15 (出力は12個まで使用)
    // 注意: フラッシュのインデックス0はダミー、実際のフレーム1はインデックス1から
    for (int frame = 0; frame < 150; frame++) {
        // 各フレームは16バイト、ただしインデックス0はスキップ(ダミーフレーム)
        int byte_offset = (frame + 1) * 16;  // +1でインデックス0をスキップ
        if (byte_offset >= 4096) break;
        
        // バイト0 = 有効フラグ
        bool frame_valid = (flash_ptr[byte_offset] != 0);
        
        if (!frame_valid) continue; // 無効フレームはスキップ
        
        // バッファサイズチェック (1行あたり約30バイト必要)
        if (pos >= (int)buffer_size - 35) break;
        
        // フレーム番号
        pos += snprintf(buffer + pos, buffer_size - pos, "%d,", frame + 1);
        
        // 12個の出力値 (バイト1-12)
        for (int i = 0; i < 12 && pos < (int)buffer_size - 3; i++) {
            int bit_val = flash_ptr[byte_offset + 1 + i];  // バイト1から出力1
            pos += snprintf(buffer + pos, buffer_size - pos, "%d", bit_val ? 1 : 0);
            if (i < 11) {
                buffer[pos++] = ',';
            }
        }
        
        if (pos < (int)buffer_size - 2) {
            buffer[pos++] = '\r';
            buffer[pos++] = '\n';
        }
    }
    
    if (pos < (int)buffer_size) {
        buffer[pos] = '\0';
    }
}

static void build_fat12_image(void) {
    for (uint i = 0; i < sizeof(ram_disk); ++i) ram_disk[i] = 0;

    uint8_t *b = ram_disk;
    memset(b, 0, DISK_SECTOR_SIZE);
    b[0] = 0xEB; b[1] = 0x3C; b[2] = 0x90;
    memcpy(&b[3], "MSDOS5.0", 8);
    uint16_t bytes_per_sector = DISK_SECTOR_SIZE; memcpy(&b[11], &bytes_per_sector, 2);
    b[13] = 1; // sectors per cluster
    uint16_t reserved = 1; memcpy(&b[14], &reserved, 2);
    b[16] = 2; // number of FATs
    uint16_t root_entries = 32; memcpy(&b[17], &root_entries, 2);
    uint16_t total_sectors = DISK_SECTOR_COUNT; memcpy(&b[19], &total_sectors, 2);
    b[21] = 0xF8; // media
    uint16_t sectors_per_fat = 4; memcpy(&b[22], &sectors_per_fat, 2);  // FATサイズを4セクタに拡張
    uint16_t sectors_per_track = 1; memcpy(&b[24], &sectors_per_track, 2);
    uint16_t heads = 1; memcpy(&b[26], &heads, 2);
    b[36] = 0x29;
    memcpy(&b[43], "NO NAME    ", 8);
    memcpy(&b[54], "FAT12   ", 8);

    // FAT領域の設定 (セクタ1-4とセクタ5-8の2つのFATテーブル)
    uint8_t *fat1 = ram_disk + DISK_SECTOR_SIZE * 1;
    uint8_t *fat2 = ram_disk + DISK_SECTOR_SIZE * 5;
    memset(fat1, 0, DISK_SECTOR_SIZE * 4);
    memset(fat2, 0, DISK_SECTOR_SIZE * 4);
    fat1[0] = 0xF8; fat1[1] = 0xFF; fat1[2] = 0xFF; // media + EOC
    
    // FAT12エントリを設定
    for (int c = 0; c < 100; c++) {
        int offset = c * 3 / 2;
        if (c % 2 == 0) {
            fat1[offset] = 0;
            fat1[offset + 1] &= 0xF0;
        } else {
            fat1[offset] &= 0x0F;
            fat1[offset + 1] = 0;
        }
    }
    
    // クラスタ2: Setting.txt (1セクタ、EOC)
    {
        int offset = 2 * 3 / 2;  // cluster 2
        fat1[offset] = 0xFF;
        fat1[offset + 1] = (fat1[offset + 1] & 0xF0) | 0x0F;
    }
    
    // クラスタ3以降: MACRO_00.txt - MACRO_11.txt (各10セクタ)
    int cluster = 3;
    for (int macro = 0; macro < 12; macro++) {
        // 10セクタのチェーンを作成
        for (int sec = 0; sec < 10; sec++) {
            int offset = cluster * 3 / 2;
            uint16_t next_val = (sec < 9) ? (cluster + 1) : 0xFFF;
            
            if (cluster % 2 == 0) {
                fat1[offset] = next_val & 0xFF;
                fat1[offset + 1] = (fat1[offset + 1] & 0xF0) | ((next_val >> 8) & 0x0F);
            } else {
                fat1[offset] = (fat1[offset] & 0x0F) | ((next_val & 0x0F) << 4);
                fat1[offset + 1] = (next_val >> 4) & 0xFF;
            }
            cluster++;
        }
    }
    
    memcpy(fat2, fat1, DISK_SECTOR_SIZE * 4);

    // ルートディレクトリはセクタ9-10 (32エントリ = 2セクタ)
    uint8_t *root = ram_disk + DISK_SECTOR_SIZE * 9;
    memset(root, 0, DISK_SECTOR_SIZE * 2);
    
    // Setting.txt エントリ
    memcpy(root + 0, "SETTING TXT", 11);
    root[11] = 0x20;
    uint16_t start_cluster = 2; memcpy(root + 26, &start_cluster, 2);

    // MACRO_00.txt - MACRO_11.txt エントリ
    cluster = 3;
    for (int i = 0; i < 12; i++) {
        uint8_t *entry = root + (i + 1) * 32;
        char filename[12];
        snprintf(filename, sizeof(filename), "MACRO_%02dTXT", i);
        memcpy(entry, filename, 11);
        entry[11] = 0x20;
        uint16_t file_cluster = cluster;
        memcpy(entry + 26, &file_cluster, 2);
        cluster += 10; // 次のマクロファイルは10クラスタ後
    }

    // データ領域の開始はセクタ11 (ルート後)
    const int actual_data_start = 11;
    
    // Setting.txt の内容を生成 (クラスタ2 = セクタ11)
    char *text_buffer = (char *)(ram_disk + DISK_SECTOR_SIZE * actual_data_start);
    memset(text_buffer, 0, DISK_SECTOR_SIZE);

    const uint8_t *flash_ptr = (const uint8_t *) (XIP_BASE + 0x1F0000);
    int text_pos = 0;
    
    // 設定ファイルのヘッダ
    const char *header =
        "Format: INPUT_NO,RAPID_TYPE,REVERSE,OUT_FRAME,IN_FRAME,OUTPUT_PINS(0:OFF 1:ON)\r\n"
        "RAPID: 1=Norm 2=R30 3=R30Rev 4=Custom 5=Macro 6=R15 7=R15Rev\r\n"
        "\r\n";
    
    int header_len = strlen(header);
    memcpy(text_buffer + text_pos, header, header_len);
    text_pos += header_len;
    
    // 12個の入力設定をCSV形式で出力
    for (int input = 0; input < 12; input++) {
        int base = input * 16;
        uint8_t rapid_type = flash_ptr[base];
        bool reverse = (rapid_type >= 10);
        if (reverse) rapid_type -= 10;
        
        // 容量チェック - 最後の行まで確実に収まるように
        if (text_pos > DISK_SECTOR_SIZE - 30) break;
        
        // INPUT_NO,RAPID_TYPE,REVERSE,OUTPUT_FRAME,INTERVAL_FRAME
        int written = snprintf(text_buffer + text_pos, DISK_SECTOR_SIZE - text_pos,
            "%d,%d,%d,%d,%d,",
            input,
            rapid_type,
            reverse ? 1 : 0,
            flash_ptr[base + 1],  // OUTPUT_FRAME
            flash_ptr[base + 2]   // INTERVAL_FRAME
        );
        
        if (written < 0) break;
        text_pos += written;
        
        // OUTPUT_PINS (12 values) - カンマなしで連続出力
        for (int pin = 0; pin < 12; pin++) {
            if (text_pos >= DISK_SECTOR_SIZE - 3) break; // 安全マージン
            written = snprintf(text_buffer + text_pos, DISK_SECTOR_SIZE - text_pos, "%d", flash_ptr[base + 4 + pin]);
            if (written < 0) break;
            text_pos += written;
        }
        
        // 行終端（容量チェック）
        if (text_pos < DISK_SECTOR_SIZE - 2) {
            text_buffer[text_pos++] = '\r';
            text_buffer[text_pos++] = '\n';
        }
    }
    
    // Setting.txt のファイルサイズを設定
    uint32_t fsize = text_pos; memcpy(root + 28, &fsize, 4);
    
    // MACRO_00.txt - MACRO_11.txt の内容を生成 (各10セクタ)
    for (int macro_no = 0; macro_no < 12; macro_no++) {
        // 各マクロファイルは10セクタ = 5120バイト使用 (150フレーム分には十分)
        int macro_start_sector = actual_data_start + 1 + (macro_no * 10);
        char *macro_buffer = (char *)(ram_disk + DISK_SECTOR_SIZE * macro_start_sector);
        memset(macro_buffer, 0, DISK_SECTOR_SIZE * 10);
        
        generate_macro_text(macro_no, macro_buffer, DISK_SECTOR_SIZE * 10);
        
        // ファイルサイズを設定
        uint32_t macro_size = strlen(macro_buffer);
        uint8_t *macro_entry = root + (macro_no + 1) * 32;
        memcpy(macro_entry + 28, &macro_size, 4);
    }
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

// マクロCSVファイルをパースしてフラッシュに書き込む
static void parse_macro_csv_n(int macro_no, const char *csv_data, size_t csv_len) {
    if (macro_no < 0 || macro_no >= 12) return;
    
    const uint32_t macro_offsets[] = {
        0x1E0000, 0x1E1000, 0x1E2000, 0x1E3000, 0x1E4000, 0x1E5000,
        0x1E6000, 0x1E7000, 0x1E8000, 0x1E9000, 0x1EA000, 0x1EB000
    };
    
    // 4096バイト = 1セクタ分のマクロデータ
    // 256フレーム × 16バイト = 4096バイト
    // 各フレームの16バイト: バイト0=有効フラグ、バイト1-15=出力1-15 (出力は12個まで使用)
    uint8_t new_macro_data[4096];
    
    // 全セクタを0でクリア
    memset(new_macro_data, 0, sizeof(new_macro_data));
    
    int max_frame_parsed = 0;  // 最大フレーム番号を記録
    
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
        if (line_start >= line_end) continue; // empty
        if (!(line_start[0] >= '0' && line_start[0] <= '9')) continue; // only data lines
        
        size_t len = (size_t)(line_end - line_start);
        char buf[160];
        if (len >= sizeof(buf)) len = sizeof(buf) - 1;
        memcpy(buf, line_start, len);
        buf[len] = '\0';
        
        // フレーム番号を取得
        int frame_no = 0;
        char *s = buf;
        char *comma = strchr(s, ',');
        if (!comma) continue;
        *comma = '\0';
        frame_no = atoi(s);
        
        if (frame_no < 1 || frame_no > 150) continue; // フレーム1-150まで有効
        int frame_index = frame_no - 1;
        
        // 各フレームは16バイトで格納
        // バイト0 = 有効フラグ, バイト1-12 = 出力1-12
        
        // 12個の出力値を取得
        s = comma + 1;
        int output_count = 0;
        uint8_t outputs[12] = {0};
        
        for (int i = 0; i < 12 && *s != '\0'; i++) {
            while (*s == ' ' || *s == '\t') s++;
            if (*s == '\0') break;
            
            if (*s == '1') {
                outputs[i] = 1;
            } else if (*s != '0') {
                break; // 不正な値
            }
            
            output_count++;
            s++;
            // カンマをスキップ
            while (*s == ' ' || *s == '\t') s++;
            if (*s == ',') s++;
        }
        
        if (output_count == 12) {
            // 16バイト形式でデータを格納
            // インデックス0はダミーなので、フレーム1はインデックス1に格納
            int byte_offset = (frame_index + 1) * 16;  // +1でインデックス0をスキップ
            if (byte_offset < 4096 - 15) {
                new_macro_data[byte_offset] = 1;  // バイト0: 有効フラグ
                for (int i = 0; i < 12; i++) {
                    new_macro_data[byte_offset + 1 + i] = outputs[i];  // バイト1-12: 出力1-12
                }
                
                // 最大フレーム番号を更新
                if (frame_no > max_frame_parsed) {
                    max_frame_parsed = frame_no;
                }
            }
        }
    }
    
    // CSVに書かれている最大フレーム番号までが有効範囲とする
    // 空白フレームを補完 (+1でインデックス0をスキップ)
    for (int i = 0; i < max_frame_parsed; i++) {
        int byte_offset = (i + 1) * 16;  // +1でインデックス0をスキップ
        
        // まだ無効(バイト0=0)のフレームは、有効フラグだけセット(全出力OFF)
        if (new_macro_data[byte_offset] == 0) {
            new_macro_data[byte_offset] = 1;  // バイト0=1 (有効フラグ)
            // バイト1-12は既に0なので全出力OFF
        }
    }
    
    // インデックス0(ダミーフレーム)に有効フラグを設定
    new_macro_data[0] = (max_frame_parsed > 0) ? 1 : 0;
    
    // フラッシュに書き込み
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(macro_offsets[macro_no], FLASH_SECTOR_SIZE);
    flash_range_program(macro_offsets[macro_no], new_macro_data, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);
    
    __dsb();
    __isb();
    sleep_us(1000);
    
    // マクロファイル保存時の自動設定
    const uint8_t *settings_flash = (const uint8_t *)(XIP_BASE + 0x1F0000);
    uint8_t settings_data[256];
    memcpy(settings_data, settings_flash, 256);
    
    // 対応する入力番号の設定を更新 (macro_no = 入力番号)
    int setting_offset = macro_no * 16;
    uint8_t current_rapid_type = settings_data[setting_offset];
    bool is_reverse = (current_rapid_type >= 10);
    
    if (max_frame_parsed > 0) {
        // 有効なフレームがある場合: RAPID_TYPE = 5 (Macro) に設定
        settings_data[setting_offset] = is_reverse ? 15 : 5;
    } else {
        // 空のマクロの場合: RAPID_TYPE = 1 (R10) に設定
        settings_data[setting_offset] = is_reverse ? 11 : 1;
    }
    
    // 設定をフラッシュに書き込み
    ints = save_and_disable_interrupts();
    flash_range_erase(0x1F0000, FLASH_SECTOR_SIZE);
    flash_range_program(0x1F0000, settings_data, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);
    
    __dsb();
    __isb();
    sleep_us(1000);
}

// CSVファイルをパースして設定に反映する
static void parse_settings_csv_n(const char *csv_data, size_t csv_len) {
    uint8_t new_settings[256];
    const uint8_t *flash_ptr = (const uint8_t *)(XIP_BASE + 0x1F0000);
    memcpy(new_settings, flash_ptr, 256);

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
        if (line_start >= line_end) continue; // empty
        if (!(line_start[0] >= '0' && line_start[0] <= '9')) continue; // only data lines

        size_t len = (size_t)(line_end - line_start);
        char buf[160];
        if (len >= sizeof(buf)) len = sizeof(buf) - 1;
        memcpy(buf, line_start, len);
        buf[len] = '\0';

        // split first 5 numeric fields
        int values[5] = {0};
        int value_count = 0;
        char *s = buf;
        char *field = s;
        for (size_t i = 0; s[i] != '\0'; ++i) {
            if (s[i] == ',') {
                s[i] = '\0';
                // trim field
                char *f = field; while (*f == ' ' || *f == '\t') f++;
                values[value_count++] = atoi(f);
                field = &s[i + 1];
                if (value_count == 5) break;
            }
        }
        if (value_count < 5) continue;

        // remaining is pins
        char *pins = field;
        int input_no = values[0];
        if (input_no < 0 || input_no >= 12) continue;
        int base = input_no * 16;

        int rapid_type = values[1];
        int reverse = values[2];
        new_settings[base] = (uint8_t)(rapid_type + (reverse ? 10 : 0));
        new_settings[base + 1] = (uint8_t)values[3];
        new_settings[base + 2] = (uint8_t)values[4];
        if (rapid_type == 5) {
            new_settings[base + 3] = (uint8_t)input_no; // Macro: CMD=自分の番号
        }

        int filled = 0;
        for (char *q = pins; *q != '\0' && filled < 12; ++q) {
            if (*q == '0' || *q == '1') {
                new_settings[base + 4 + filled] = (*q == '1') ? 1 : 0;
                filled++;
            } else if (*q == ' ' || *q == '\t') {
                // skip
            } else {
                break;
            }
        }
        if (filled != 12) continue; // 不完全行は適用しない
    }

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(0x1F0000, FLASH_SECTOR_SIZE);
    flash_range_program(0x1F0000, new_settings, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
    
    // フラッシュ書込み後に同期処理（重要）
    __dsb(); // データ同期バリア
    __isb(); // 命令同期バリア
    
    // 少し待機してフラッシュ書込み完了を確実に
    sleep_us(1000);
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize) {
    (void) lun;
    if (lba >= DISK_SECTOR_COUNT) return -1;
    uint32_t available = DISK_SECTOR_SIZE - offset;
    if (bufsize > available) bufsize = available;
    memcpy(ram_disk + DISK_SECTOR_SIZE * lba + offset, buffer, bufsize);

    // ルートディレクトリ更新時にファイルを識別
    if (lba == ROOT_DIR_SECTOR || lba == ROOT_DIR_SECTOR + 1) {
        uint8_t *root = ram_disk + DISK_SECTOR_SIZE * ROOT_DIR_SECTOR;
        
        // Setting.txt のチェック
        if (root[0] != 0x00 && root[0] != 0xE5) {
            if (memcmp(root + 0, "SETTING TXT", 11) == 0) {
                uint32_t fsz = 0; memcpy(&fsz, root + 28, 4);
                s_file_size_hint = fsz;
                if (!s_write_processed && s_write_len > 0) {
                    parse_settings_csv_n((const char*)s_write_buffer, s_write_len);
                    s_write_processed = true;
                    start_post_blink_pattern();
                }
            }
        }
        
        // MACRO_XX.txt のチェック (エントリ1-12)
        for (int i = 0; i < 12; i++) {
            uint8_t *entry = root + (i + 1) * 32;
            if (entry[0] != 0x00 && entry[0] != 0xE5) {
                char expected_name[12];
                snprintf(expected_name, sizeof(expected_name), "MACRO_%02dTXT", i);
                if (memcmp(entry, expected_name, 11) == 0) {
                    uint32_t fsz = 0; memcpy(&fsz, entry + 28, 4);
                    if (fsz > 0 && s_macro_write_number == i && s_macro_write_pending) {
                        // マクロファイルの書き込みを適用
                        parse_macro_csv_n(i, (const char*)s_macro_write_buffer, fsz);
                        s_macro_write_pending = false;
                        s_macro_write_number = -1;
                        start_post_blink_pattern();
                    }
                }
            }
        }
    }
    
    // Setting.txt のデータセクタ書込み (セクタ7)
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
            uint8_t *root = ram_disk + DISK_SECTOR_SIZE * ROOT_DIR_SECTOR;
            uint32_t current_fsize = 0; memcpy(&current_fsize, root + 28, 4);
            if (s_file_size_hint > 0) current_fsize = s_file_size_hint;
            uint32_t preload_len = current_fsize;
            if (preload_len == 0 || preload_len > sizeof(s_write_buffer)) preload_len = sizeof(s_write_buffer);
            memcpy(s_write_buffer, ram_disk + DISK_SECTOR_SIZE * DATA_START_SECTOR, preload_len);
            s_write_len = preload_len;
            s_preseeded_buffer = true;
        }
        s_last_write_ms = to_ms_since_boot(get_absolute_time());
        uint32_t pos = (lba - DATA_START_SECTOR) * DISK_SECTOR_SIZE + offset;
        if (pos < sizeof(s_write_buffer)) {
            uint32_t cpy = bufsize;
            if (pos + cpy > sizeof(s_write_buffer)) cpy = sizeof(s_write_buffer) - pos;
            memcpy(s_write_buffer + pos, buffer, cpy);
            if (pos + cpy > s_write_len) s_write_len = pos + cpy;
        }
    }
    
    // MACRO_XX.txt のデータセクタ書込み (各マクロファイルは10セクタ)
    for (int i = 0; i < 12; i++) {
        int macro_start_sector = DATA_START_SECTOR + 1 + (i * 10);
        int macro_end_sector = macro_start_sector + 10;
        
        if (lba >= macro_start_sector && lba < macro_end_sector) {
            if (!s_led_blinking) {
                s_post_blink_active = false;
                s_post_blink_remaining_toggles = 0;
                led_off();
                s_led_suppress_until_write = false;
                led_blink_start();
                s_led_blinking = true;
                s_macro_write_number = i;
                memset(s_macro_write_buffer, 0, sizeof(s_macro_write_buffer));
            }
            s_last_write_ms = to_ms_since_boot(get_absolute_time());
            
            // マクロ書き込みバッファにコピー (10セクタ分)
            int sector_offset = lba - macro_start_sector;
            uint32_t buf_pos = sector_offset * DISK_SECTOR_SIZE + offset;
            if (buf_pos + bufsize <= sizeof(s_macro_write_buffer)) {
                memcpy(s_macro_write_buffer + buf_pos, buffer, bufsize);
                s_macro_write_number = i;
                s_macro_write_pending = true;
            }
            break;
        }
    }
    
    return (int32_t) bufsize;
}

// 書き込み完了時に検証・適用/削除を行う
void tud_msc_write10_complete_cb(uint8_t lun) {
    (void)lun;
    
    // Setting.txt の書き込み完了処理
    if (!s_write_processed) {
        uint8_t *root = ram_disk + DISK_SECTOR_SIZE * ROOT_DIR_SECTOR;
        if (root[0] != 0x00 && root[0] != 0xE5 && memcmp(root + 0, "SETTING TXT", 11) == 0) {
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
    
    // MACRO_XX.txt の書き込み完了処理
    if (s_macro_write_pending && s_macro_write_number >= 0 && s_macro_write_number < 12) {
        uint8_t *root = ram_disk + DISK_SECTOR_SIZE * ROOT_DIR_SECTOR;
        uint8_t *entry = root + (s_macro_write_number + 1) * 32;
        if (entry[0] != 0x00 && entry[0] != 0xE5) {
            // ルートディレクトリのファイルサイズではなく、実際のバッファから直接読み取る
            int macro_start_sector = DATA_START_SECTOR + 1 + (s_macro_write_number * 10);
            
            // バッファ全体をコピー (最大5120バイト)
            memcpy(s_macro_write_buffer, ram_disk + DISK_SECTOR_SIZE * macro_start_sector, sizeof(s_macro_write_buffer));
            
            // 実際のファイルサイズを計算 (NULL終端または最大サイズまで)
            size_t actual_size = 0;
            for (size_t i = 0; i < sizeof(s_macro_write_buffer); i++) {
                if (s_macro_write_buffer[i] == '\0') {
                    actual_size = i;
                    break;
                }
            }
            if (actual_size == 0) actual_size = sizeof(s_macro_write_buffer);
            
            if (actual_size > 0) {
                parse_macro_csv_n(s_macro_write_number, (const char*)s_macro_write_buffer, actual_size);
                s_macro_write_pending = false;
                s_macro_write_number = -1;
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
    s_write_processed = false;
    s_led_blinking = false;
    s_write_len = 0;
    s_file_size_hint = 0;
    s_last_write_ms = 0;
    s_preseeded_buffer = false;
    s_blink_hold_until_ms = 0;
    // 接続直後は点滅しないようLEDとパターン状態を明示的にリセット
    s_post_blink_active = false;
    s_post_blink_remaining_toggles = 0;
    s_led_manual_state = false;
    s_led_suppress_until_write = true;
    // マクロ書き込み用の変数を初期化
    s_macro_write_number = -1;
    s_macro_write_pending = false;
    memset(s_macro_write_buffer, 0, sizeof(s_macro_write_buffer));
    led_blink_stop();
    led_off();
    memset(s_write_buffer, 0, sizeof(s_write_buffer));
    if (!s_inited) {
        board_init();
        s_inited = true;
    }
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
