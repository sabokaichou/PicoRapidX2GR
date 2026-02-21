/*
 * PicoVSyncSeparator - Main Program
 * 複合同期信号から垂直同期信号を分離し、SSD1306ディスプレイに表示
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "vsync_separator.h"

// I2C/ディスプレイ設定
#define I2C_PORT i2c0
#define SDA_PIN 4
#define SCL_PIN 5
#define SSD1306_I2C_ADDR 0x3C

// SSD1306簡易表示関数（既存のコードから流用）
// OLEDライブラリが見つからないため一時的にコメントアウト
// extern bool Ssd1306_Init(void);
// extern unsigned char* Ssd1306_Get_Draw_Canvas(void);
// extern void Ssd1306_Update_Frame(void);
// extern void DrawMessage(int row_pos, char *Message, bool reverse_flg);
// extern void SetCharPattern(void);

// static unsigned char *canvas;

/*
 * メインループ
 */
int main() {
    // 標準入出力初期化
    stdio_init_all();
    
    // 少し待機（安定化）
    sleep_ms(100);
    
    printf("PicoVSyncSeparator Starting...\n");
    
    // I2C初期化（OLED用）
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    
    // OLED初期化（一時的にコメントアウト）
    // printf("Initializing OLED...\n");
    // Ssd1306_Init();
    // SetCharPattern();
    // canvas = Ssd1306_Get_Draw_Canvas();
    
    // 起動メッセージ
    // DrawMessage(0, "V-Sync", false);
    // DrawMessage(1, "Separator", false);
    // Ssd1306_Update_Frame();
    // sleep_ms(2000);
    
    // V-Sync分離器初期化
    printf("Initializing V-Sync Separator...\n");
    vsync_separator_init();
    
    // 出力ピンをカスタマイズする場合（オプション）
    
    // 例1: GP15=常時出力(60Hz), GP16=奇数のみ(30Hz)
    // const uint8_t pins1[] = {15, 16};
    // const output_pattern_t patterns1[] = {OUTPUT_ALWAYS, OUTPUT_ODD};
    // vsync_separator_set_output_pins(pins1, patterns1, 2);
    
    // 例2: 5つのピンで異なるパターン
    // const uint8_t pins2[] = {15, 16, 17, 18, 19};
    // const output_pattern_t patterns2[] = {
    //     OUTPUT_ALWAYS,      // GP15: 60Hz
    //     OUTPUT_ODD,         // GP16: 30Hz(奇数)
    //     OUTPUT_EVEN,        // GP17: 30Hz(偶数)
    //     OUTPUT_QUARTER_01,  // GP18: 15Hz(0,1)
    //     OUTPUT_QUARTER_23   // GP19: 15Hz(2,3)
    // };
    // vsync_separator_set_output_pins(pins2, patterns2, 5);
    
    // 例2.5: カスタム連射パターンを含む10ピン
    // const uint8_t pins25[] = {15, 16, 17, 18, 19, 20, 21, 22, 26, 27};
    // const output_pattern_t patterns25[] = {
    //     OUTPUT_ALWAYS,      // GP15: 60Hz
    //     OUTPUT_ODD,         // GP16: 30Hz
    //     OUTPUT_EVEN,        // GP17: 30Hz
    //     OUTPUT_QUARTER_01,  // GP18: 15Hz
    //     OUTPUT_QUARTER_23,  // GP19: 15Hz
    //     OUTPUT_2ON_1OFF,    // GP20: 40Hz
    //     OUTPUT_1ON_2OFF,    // GP21: 20Hz
    //     OUTPUT_1ON_3OFF,    // GP22: 15Hz
    //     OUTPUT_1ON_4OFF,    // GP26: 12Hz
    //     OUTPUT_1ON_5OFF     // GP27: 10Hz
    // };
    // vsync_separator_set_output_pins(pins25, patterns25, 10);
    
    // 例3: すべて同じパターン（簡易設定）
    // const uint8_t pins3[] = {15, 16, 17};
    // vsync_separator_set_output_pins_simple(pins3, 3, OUTPUT_ODD);
    
    // 例4: 最大16ピン使用（大量のボタン制御）
    // const uint8_t pins4[] = {15, 16, 17, 18, 19, 20, 21, 22, 26, 27, 28, 6, 7, 8, 9, 10};
    // const output_pattern_t patterns4[] = {
    //     OUTPUT_ALWAYS, OUTPUT_ODD, OUTPUT_EVEN, OUTPUT_QUARTER_01, OUTPUT_QUARTER_23,
    //     OUTPUT_ODD, OUTPUT_EVEN, OUTPUT_ODD, OUTPUT_EVEN, OUTPUT_QUARTER_01,
    //     OUTPUT_QUARTER_23, OUTPUT_ODD, OUTPUT_EVEN, OUTPUT_ODD, OUTPUT_EVEN, OUTPUT_ALWAYS
    // };
    // vsync_separator_set_output_pins(pins4, patterns4, 16);
    
    // DrawMessage(0, "Ready", false);
    // DrawMessage(1, "Waiting...", false);
    // Ssd1306_Update_Frame();
    // sleep_ms(1000);
    
    // メインループ
    uint32_t last_display_update = 0;
    char line0[16];
    char line1[16];
    char line2[16];
    char line3[16];
    
    while (true) {
        // V-Sync分離タスク実行（高頻度）
        vsync_separator_task();
        
        // ディスプレイ更新（100ms周期）
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_display_update > 100) {
            last_display_update = now;
            
            const sync_stats_t* stats = vsync_separator_get_stats();
            
            // 状態表示
            snprintf(line0, sizeof(line0), "St:%s", vsync_separator_get_state_string());
            snprintf(line1, sizeof(line1), "H:%lu", stats->hsync_count);
            snprintf(line2, sizeof(line2), "V:%lu", stats->vsync_count);
            snprintf(line3, sizeof(line3), "P:%lu/%lu", stats->last_high_width, stats->last_low_width);
            
            // DrawMessage(0, line0, stats->vsync_active);
            // DrawMessage(1, line1, false);
            // DrawMessage(2, line2, false);
            // DrawMessage(3, line3, false);
            // Ssd1306_Update_Frame();
            
            // シリアル出力（デバッグ用）
            printf("State:%s H:%lu V:%lu Pulse:%lu/%lu VSync:%s\n",
                   vsync_separator_get_state_string(),
                   stats->hsync_count,
                   stats->vsync_count,
                   stats->last_high_width,
                   stats->last_low_width,
                   stats->vsync_active ? "ACTIVE" : "IDLE");
        }
        
        // 短時間待機（CPU負荷軽減）
        sleep_us(10);
    }
    
    return 0;
}
