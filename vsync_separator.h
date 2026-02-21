/*
 * V-Sync Separator Header
 * 複合同期信号から垂直同期信号を分離
 */

#ifndef VSYNC_SEPARATOR_H
#define VSYNC_SEPARATOR_H

#include <stdint.h>
#include <stdbool.h>

// 最大出力ピン数（16ピンまで拡張可能）
#define MAX_OUTPUT_PINS 16

// 出力パターンタイプ
typedef enum {
    OUTPUT_ALWAYS,          // 常に出力（全信号：60回/秒）
    OUTPUT_ODD,             // 奇数回のみ出力（30回/秒）
    OUTPUT_EVEN,            // 偶数回のみ出力（30回/秒）
    OUTPUT_QUARTER_01,      // カウント%4が0,1でオン、2,3でオフ（15回/秒）
    OUTPUT_QUARTER_23,      // カウント%4が2,3でオン、0,1でオフ（15回/秒）
    OUTPUT_2ON_1OFF,        // 2回オン、1回オフ（40回/秒）
    OUTPUT_1ON_2OFF,        // 1回オン、2回オフ（20回/秒）
    OUTPUT_1ON_3OFF,        // 1回オン、3回オフ（15回/秒）
    OUTPUT_1ON_4OFF,        // 1回オン、4回オフ（12回/秒）
    OUTPUT_1ON_5OFF,        // 1回オン、5回オフ（10回/秒）
    OUTPUT_1ON_6OFF         // 1回オン、6回オフ（約8.6回/秒）
} output_pattern_t;

// 統計情報構造体
typedef struct {
    uint32_t hsync_count;
    uint32_t vsync_count;
    uint32_t last_high_width;
    uint32_t last_low_width;
    int current_state;
    bool vsync_active;
    uint32_t debug_hsync_count_in_period;  // デバッグ: 現在の期間内H-Syncカウント
    uint32_t debug_prev_hsync_count;       // デバッグ: 前回の期間内H-Syncカウント
} sync_stats_t;

/*
 * V-Sync分離器の初期化
 */
void vsync_separator_init(void);

/*
 * V-Sync分離器の初期化（コールバック付き）
 */
void vsync_separator_init_with_callback(void (*callback)(void));

/*
 * V-Sync分離タスク（メインループで定期的に呼び出し）
 */
void vsync_separator_task(void);

/*
 * V-Sync分離タスクCore1専用（Core1で回し続ける）
 */
void vsync_separator_task_core1(void);

/*
 * Core1でV-Sync検出タスクを起動
 */
void vsync_separator_start_core1(void);

/*
 * 統計情報取得
 */
const sync_stats_t* vsync_separator_get_stats(void);

/*
 * 現在の状態を文字列で取得
 */
const char* vsync_separator_get_state_string(void);

/*
 * 出力ピンを設定（最大MAX_OUTPUT_PINS個）
 * pins: 出力するGPIOピン番号の配列
 * patterns: 各ピンの出力パターンの配列
 * count: ピン数
 */
void vsync_separator_set_output_pins(const uint8_t* pins, const output_pattern_t* patterns, uint8_t count);

/*
 * 簡易設定：全ピン同じパターン
 */
void vsync_separator_set_output_pins_simple(const uint8_t* pins, uint8_t count, output_pattern_t pattern);

#endif // VSYNC_SEPARATOR_H
