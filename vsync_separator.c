/*
 * V-Sync Separator - 複合同期信号から垂直同期信号を分離
 * PIOステートマシンを使用した高精度パルス幅測定
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "vsync_separator.h"
#include "vsync_separator.pio.h"

// ピン定義
#define CSYNC_INPUT_PIN 28      // 複合同期信号入力（GP28に変更）

// デフォルト出力ピン（初期化時に使用）
static const uint8_t DEFAULT_OUTPUT_PINS[] = {
    1,   // 60Hz常時出力
    6,   // 30Hz奇数
    7,   // 30Hz偶数
    8,   // 15Hz前半
    9,   // 15Hz後半
    10,  // 40Hz (2on-1off)
    11,  // 20Hz (1on-2off)
    12,  // 15Hz (1on-3off)
    13,  // 12Hz (1on-4off)
    14,  // 10Hz (1on-5off)
    15   // 8.6Hz (1on-6off)
};
static const output_pattern_t DEFAULT_OUTPUT_PATTERNS[] = {
    OUTPUT_ALWAYS,      // GP1: 60Hz
    OUTPUT_ODD,         // GP6: 30Hz奇数
    OUTPUT_EVEN,        // GP7: 30Hz偶数
    OUTPUT_QUARTER_01,  // GP8: 15Hz前半
    OUTPUT_QUARTER_23,  // GP9: 15Hz後半
    OUTPUT_2ON_1OFF,    // GP10: 40Hz
    OUTPUT_1ON_2OFF,    // GP11: 20Hz
    OUTPUT_1ON_3OFF,    // GP12: 15Hz
    OUTPUT_1ON_4OFF,    // GP13: 12Hz
    OUTPUT_1ON_5OFF,    // GP14: 10Hz
    OUTPUT_1ON_6OFF     // GP15: 8.6Hz
};
#define DEFAULT_OUTPUT_COUNT 11

// 設定可能な出力ピン配列
static uint8_t output_pins[MAX_OUTPUT_PINS];
static output_pattern_t output_patterns[MAX_OUTPUT_PINS];
static uint8_t output_pin_count = 0;

// 垂直同期カウンタ（出力パターン判定用）
static uint32_t vsync_counter = 0;

// 削除: H-Syncカウント速度検出は使用しない

// パルス幅閾値(サンプル数単位） - 1μs/サンプル (PIOクロック1MHz)
#define VSYNC_HIGH_MIN 10        // V-Sync開始検出最小HIGHパルス幅（10μs以上）
#define VSYNC_LOW_TOLERANCE 5    // V-Sync継続中の短いLOW許容幅（5μs以下）
#define VSYNC_TOTAL_MIN 30       // V-Sync全体の最小幅（30μs以上でV-Sync確定）

// ステートマシンの状態
typedef enum {
    STATE_IDLE,              // 待機
    STATE_HSYNC,             // 水平同期検出中
    STATE_EQUALIZING,        // 等化パルス検出中
    STATE_VSYNC_SERRATION,   // 垂直同期鋸歯パルス検出中
    STATE_VSYNC_ACTIVE       // 垂直同期信号出力中
} sync_state_t;

// グローバル変数
static PIO pio = pio0;
static uint sm = 0;
static sync_state_t current_state = STATE_IDLE;
static uint32_t equalizing_count = 0;
static uint32_t serration_count = 0;
static bool vsync_output = false;
static uint32_t high_count = 0;  // HIGHパルス幅カウンタ
static uint32_t low_count = 0;   // LOWパルス幅カウンタ
static uint32_t total_vsync_width = 0;  // V-Sync全体の幅（HIGH+短いLOW含む）
static bool in_vsync_candidate = false;  // V-Sync候補期間中フラグ

// コールバック関数
static void (*user_vsync_callback)(void) = NULL;

// 統計情報
static sync_stats_t stats = {0};

/*
 * V-Sync分離器の初期化
 */
void vsync_separator_init(void) {
    vsync_separator_init_with_callback(NULL);
}

/*
 * V-Sync分離器の初期化（コールバック付き）
 */
void vsync_separator_init_with_callback(void (*callback)(void)) {
    user_vsync_callback = callback;
    
    // PicoRapidX2では出力ピンは使用しない（GP12-15は入力ピンとして使用するため）
    // vsync_separator_set_output_pins(DEFAULT_OUTPUT_PINS, DEFAULT_OUTPUT_PATTERNS, DEFAULT_OUTPUT_COUNT);
    output_pin_count = 0;  // 出力ピンなし
    
    // GP28を入力として明示的に初期化（PIO初期化前）
    gpio_init(CSYNC_INPUT_PIN);
    gpio_set_dir(CSYNC_INPUT_PIN, GPIO_IN);
    gpio_disable_pulls(CSYNC_INPUT_PIN);  // プルアップ無効化（外部バッファ回路使用のため）
    
    // PIOプログラムロード
    uint offset = pio_add_program(pio, &csync_measure_program);
    csync_measure_program_init(pio, sm, offset, CSYNC_INPUT_PIN);
    
    printf("V-Sync Separator initialized (with transistor buffer circuit)\n");
    printf("C-Sync Input: GP%d\n", CSYNC_INPUT_PIN);
    printf("Initial pin state: %d\n", gpio_get(CSYNC_INPUT_PIN));
    printf("Output Pins:\n");
    for (uint8_t i = 0; i < output_pin_count; i++) {
        const char* pattern_name;
        switch(output_patterns[i]) {
            case OUTPUT_ALWAYS: pattern_name = "ALWAYS(60Hz)"; break;
            case OUTPUT_ODD: pattern_name = "ODD(30Hz)"; break;
            case OUTPUT_EVEN: pattern_name = "EVEN(30Hz)"; break;
            case OUTPUT_QUARTER_01: pattern_name = "QUARTER_01(15Hz)"; break;
            case OUTPUT_QUARTER_23: pattern_name = "QUARTER_23(15Hz)"; break;
            case OUTPUT_2ON_1OFF: pattern_name = "2ON_1OFF(40Hz)"; break;
            case OUTPUT_1ON_2OFF: pattern_name = "1ON_2OFF(20Hz)"; break;
            case OUTPUT_1ON_3OFF: pattern_name = "1ON_3OFF(15Hz)"; break;
            case OUTPUT_1ON_4OFF: pattern_name = "1ON_4OFF(12Hz)"; break;
            case OUTPUT_1ON_5OFF: pattern_name = "1ON_5OFF(10Hz)"; break;
            case OUTPUT_1ON_6OFF: pattern_name = "1ON_6OFF(8.6Hz)"; break;
            default: pattern_name = "UNKNOWN"; break;
        }
        printf("  GP%d: %s\n", output_pins[i], pattern_name);
    }
}

/*
 * パルス幅を分類
 */
static inline bool is_vsync_high(uint32_t width) {
    return (width >= VSYNC_HIGH_MIN);
}

static inline bool is_hsync_pulse(uint32_t width) {
    // H-Syncパルス: 10-20サンプル程度（1MHzサンプリング）
    return (width >= 10 && width < VSYNC_HIGH_MIN);
}

static inline bool is_equalizing_pulse(uint32_t width_us) {
    return false;  // 未使用
}

static inline bool is_vsync_serration(uint32_t width_us) {
    return false;  // 未使用
}

/*
 * パターンに基づいて出力するかどうかを判定
 */
static inline bool should_output(output_pattern_t pattern, uint32_t counter) {
    uint32_t mod;
    
    switch (pattern) {
        case OUTPUT_ALWAYS:
            return true;
        case OUTPUT_ODD:
            return (counter % 2) == 1;
        case OUTPUT_EVEN:
            return (counter % 2) == 0;
        case OUTPUT_QUARTER_01:
            return (counter % 4) <= 1;
        case OUTPUT_QUARTER_23:
            return (counter % 4) >= 2;
        case OUTPUT_2ON_1OFF:
            // 3回周期: 1,2がオン、3がオフ
            mod = (counter - 1) % 3;
            return mod < 2;
        case OUTPUT_1ON_2OFF:
            // 3回周期: 1がオン、2,3がオフ
            mod = (counter - 1) % 3;
            return mod == 0;
        case OUTPUT_1ON_3OFF:
            // 4回周期: 1がオン、2,3,4がオフ
            mod = (counter - 1) % 4;
            return mod == 0;
        case OUTPUT_1ON_4OFF:
            // 5回周期: 1がオン、2,3,4,5がオフ
            mod = (counter - 1) % 5;
            return mod == 0;
        case OUTPUT_1ON_5OFF:
            // 6回周期: 1がオン、2,3,4,5,6がオフ
            mod = (counter - 1) % 6;
            return mod == 0;
        case OUTPUT_1ON_6OFF:
            // 7回周期: 1がオン、2,3,4,5,6,7がオフ
            mod = (counter - 1) % 7;
            return mod == 0;
        default:
            return false;
    }
}

/*
 * 垂直同期信号を出力
 */
static void set_vsync_output(bool active) {
    vsync_output = active;
    
    if (active) {
        // 垂直同期開始時：カウンタをインクリメント
        vsync_counter++;
        
        // 各ピンのパターンに応じて出力
        for (uint8_t i = 0; i < output_pin_count; i++) {
            bool output = should_output(output_patterns[i], vsync_counter);
            gpio_put(output_pins[i], output ? 1 : 0);
        }
    } else {
        // 垂直同期終了時：すべてのピンをOFF
        for (uint8_t i = 0; i < output_pin_count; i++) {
            gpio_put(output_pins[i], 0);
        }
    }
    
    stats.vsync_active = active;
}

/*
 * ステートマシン処理（CPS2専用: 長いHIGH検出方式）
 */
static void process_sync_state(bool active) {
    stats.current_state = current_state;

    if (active) {
        stats.vsync_count++;
        stats.vsync_active = true;
        stats.last_high_width = high_count;  // デバッグ: 検出したHIGH幅を記録
        
        // コールバック呼び出し
        if (user_vsync_callback != NULL) {
            user_vsync_callback();
        }
    }
    else
    {
        stats.vsync_active = false;
    }
}

/*
 * V-Sync分離タスク（メインループで呼び出し）
 * 10μs以上のHIGHを検出後、5μs以下の短いLOWを許容して継続カウント
 */
void vsync_separator_task(void) {
    static uint32_t last_pin_state = 1;  // 前回のピン状態（初期値HIGH）
    
    // PIO FIFOからデータ読み取り
    while (!pio_sm_is_rx_fifo_empty(pio, sm)) {
        
        // PIOから1ビット読み取り（1MHzサンプリング、GP28のみ）
        uint32_t data = pio_sm_get_blocking(pio, sm);
        
        // 最下位ビットがGP28の状態
        uint32_t pin_state = (data & 1);
        
        // エッジ検出
        if (pin_state != last_pin_state) {
            if (pin_state == 0) {
                // HIGH→LOW遷移
                
                // 10μs以上のHIGHを検出したらV-Sync候補開始
                if (!in_vsync_candidate && high_count >= VSYNC_HIGH_MIN) {
                    in_vsync_candidate = true;
                    total_vsync_width = high_count;  // 初期幅
                }
                // V-Sync候補期間中なら幅に加算
                else if (in_vsync_candidate) {
                    total_vsync_width += high_count;
                }
                
                low_count = 1;
                high_count = 0;
                
            } else {
                // LOW→HIGH遷移
                
                // V-Sync候補期間中の処理
                if (in_vsync_candidate) {
                    // 短いLOW（5μs以下）なら継続
                    if (low_count <= VSYNC_LOW_TOLERANCE) {
                        total_vsync_width += low_count;  // LOW幅も含める
                        // V-Sync期間継続
                    }
                    // 長いLOWならV-Sync期間終了
                    else {
                        // 全体が30μs以上ならV-Sync確定
                        if (total_vsync_width >= VSYNC_TOTAL_MIN) {
                            process_sync_state(true);
                            stats.last_high_width = total_vsync_width;  // デバッグ用
                        } else {
                            process_sync_state(false);
                        }
                        
                        // リセット
                        in_vsync_candidate = false;
                        total_vsync_width = 0;
                    }
                }
                
                high_count = 1;
                low_count = 0;
            }
            
            last_pin_state = pin_state;
        } else {
            // エッジなし、カウント増加
            if (pin_state == 1) {
                high_count++;
            } else {
                low_count++;
                
                // V-Sync候補期間中に長いLOWになった場合も終了判定
                if (in_vsync_candidate && low_count > VSYNC_LOW_TOLERANCE) {
                    if (total_vsync_width >= VSYNC_TOTAL_MIN) {
                        process_sync_state(true);
                        stats.last_high_width = total_vsync_width;
                    } else {
                        process_sync_state(false);
                    }
                    
                    in_vsync_candidate = false;
                    total_vsync_width = 0;
                }
            }
        }
    }
    
    // デバッグ用: ステート情報を格納
    stats.debug_hsync_count_in_period = current_state;
    stats.debug_prev_hsync_count = total_vsync_width;  // 現在の累積幅
}

/*
 * V-Sync分離タスクCore1専用（無限ループ）
 * Core1で専用実行され、PIO FIFOを常時ポーリング
 */
void vsync_separator_task_core1(void) {
    printf("Core1: V-Sync detection task started\n");
    
    while (true) {
        vsync_separator_task();
        // tight_loop_contents()は不要（最大速度で回す）
    }
}

/*
 * Core1でV-Sync検出タスクを起動
 */
void vsync_separator_start_core1(void) {
    printf("Starting V-Sync detection on Core1...\n");
    multicore_launch_core1(vsync_separator_task_core1);
    printf("Core1 launched\n");
}

/*
 * 出力ピンを設定
 */
void vsync_separator_set_output_pins(const uint8_t* pins, const output_pattern_t* patterns, uint8_t count) {
    // 既存の出力ピンをクリア
    for (uint8_t i = 0; i < output_pin_count; i++) {
        gpio_put(output_pins[i], 0);
        gpio_deinit(output_pins[i]);
    }
    
    // 新しい出力ピンを設定
    output_pin_count = (count > MAX_OUTPUT_PINS) ? MAX_OUTPUT_PINS : count;
    
    for (uint8_t i = 0; i < output_pin_count; i++) {
        output_pins[i] = pins[i];
        output_patterns[i] = patterns[i];
        gpio_init(output_pins[i]);
        gpio_set_dir(output_pins[i], GPIO_OUT);
        gpio_put(output_pins[i], 0);
    }
    
    // カウンタをリセット
    vsync_counter = 0;
}

/*
 * 簡易設定：全ピン同じパターン
 */
void vsync_separator_set_output_pins_simple(const uint8_t* pins, uint8_t count, output_pattern_t pattern) {
    output_pattern_t patterns[MAX_OUTPUT_PINS];
    uint8_t actual_count = (count > MAX_OUTPUT_PINS) ? MAX_OUTPUT_PINS : count;
    
    for (uint8_t i = 0; i < actual_count; i++) {
        patterns[i] = pattern;
    }
    
    vsync_separator_set_output_pins(pins, patterns, actual_count);
}

const char* vsync_separator_get_state_string(void) {
    switch (current_state) {
        case STATE_IDLE: return "IDLE";
        case STATE_HSYNC: return "H-SYNC";
        case STATE_EQUALIZING: return "EQUAL";
        case STATE_VSYNC_SERRATION: return "V-SERR";
        case STATE_VSYNC_ACTIVE: return "V-SYNC";
        default: return "UNKNOWN";
    }
}

const sync_stats_t* vsync_separator_get_stats(void) {
    return &stats;
}


