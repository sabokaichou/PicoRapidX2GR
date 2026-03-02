#include <stdio.h>
#include <string.h>
#include <pico/stdlib.h>
#include <stdint.h>
#include <pico/multicore.h>
#include <pico/time.h>
#include <hardware/flash.h>
#include <hardware/sync.h>
#include <hardware/gpio.h>
#include <hardware/i2c.h>
#include "bsp/board.h"
#include "vsync_separator.h"

void usb_msc_start(void);
void usb_msc_task(void);
bool usb_msc_is_connected(void);
bool usb_msc_is_mounted(void);
const char* usb_msc_get_status_string(void);

// GPIO関連
#define Sync_Pin 2   // CSync入力 (GP02)
#define VBUS_PIN 24  // VBUS検出ピン（USB接続検知用）

#define Sync_IRQ_enable   gpio_set_irq_enabled(Sync_Pin, 0x4u, true);
#define Sync_IRQ_disable  gpio_set_irq_enabled(Sync_Pin, 0x4u, false);

bool InputStatus[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

uint8_t boardMode = 0;

int Output_Pin[] = {11, 10, 9, 8, 6, 7, 5, 4, 3};
int Input_Pin[]  = {15, 14, 13, 12, 16, 17, 18, 19, 20, 21, 22, 26};

// PicoRapidX2GR 出力ピン定義 (JAMMAコネクタ対応)
// INDEX: 0=リセット 1=スタート 2=上 3=下 4=左(JAMMA) 5=右 6=A 7=B 8=C
const int8_t Output_Pin_GR[] = {28, 27, 26, 22, 21, 20, 18, 17, 16};

// PicoRapidX2GR 入力ピン定義
#define GR_PIN_TOP_L  4   // 上段左  (スイッチOFF時=A, ON時=D)
#define GR_PIN_TOP_M  5   // 上段中  (スイッチOFF時=B, ON時=E)
#define GR_PIN_TOP_R  6   // 上段右  (スイッチOFF時=C, ON時=F)
#define GR_PIN_BTM_L  13  // 下段左  (スイッチOFF時=D, ON時=A)
#define GR_PIN_BTM_M  14  // 下段中  (スイッチOFF時=E, ON時=B)
#define GR_PIN_BTM_R  15  // 下段右  (スイッチOFF時=F, ON時=C)
#define GR_PIN_SWAP   1   // 上下切替スイッチ (LOW=ON=スワップ)
#define GR_PIN_RESET  0   // リセットボタン (1秒長押しで動作)
#define GR_PIN_ROT1   7   // ロータリー位置1 (8.6/s: on1/off6)
#define GR_PIN_ROT2   8   // ロータリー位置2 (10/s:  on1/off5)
#define GR_PIN_ROT3   9   // ロータリー位置3 (12/s:  on1/off4)
#define GR_PIN_ROT4   10  // ロータリー位置4 (15/s:  on1/off3)
#define GR_PIN_ROT5   11  // ロータリー位置5 (20/s:  on1/off2)
#define GR_PIN_ROT6   12  // ロータリー位置6 (30/s:  on1/off1)

// 出力インデックス定義
#define GR_OUT_RESET  0
#define GR_OUT_START  1
#define GR_OUT_UP     2
#define GR_OUT_DOWN   3
#define GR_OUT_LEFT   4
#define GR_OUT_RIGHT  5
#define GR_OUT_A      6
#define GR_OUT_B      7
#define GR_OUT_C      8

// ロータリースイッチの連射OFFフレーム数 [位置0-5]
static const uint8_t GR_RapidOffFrames[6] = {6, 5, 4, 3, 2, 1};

// リセット長押し判定フレーム数 (約1秒 = 60フレーム)
#define GR_RESET_HOLD_FRAMES 60

// GRボタン動作モード定義
#define GR_BTN_MODE_DISABLED      0  // 無効
#define GR_BTN_MODE_HOLD          1  // 単純ホールド
#define GR_BTN_MODE_RAPID_ROTARY  2  // 連射(ロータリー速度使用)
#define GR_BTN_MODE_RAPID_FIXED   3  // 連射(固定速度)

// マクロ出力先の特殊値 (gpio フィールドで識別、実GPIOではない)
#define GR_GPIO_MACRO_START       100  // STARTマクロ (GP27へ出力)
#define GR_GPIO_MACRO_RESET       101  // RESETマクロ (GP28へ出力)
#define GR_GPIO_MACRO_RESETSTART  102  // RESET+STARTマクロ (GP28+GP27へ出力)

// ---- プログラム埋め込みマクロシステム ----
// MacroStep: outputs=0=待機, outputs=0xFFFF=終端マーカ, それ以外=出力ビット列
typedef struct {
    uint16_t outputs;  // 出力ビット (Output_Pin_GR[]インデックす対応)
    uint8_t  frames;   // 保持/待機フレーム数
} MacroStep;

// 出力ビット定義 (Output_Pin_GR[] のインデックス照射)
#define MOUT_RESET  (1u<<0)   // GP28 (index 0 = GR_OUT_RESET)
#define MOUT_START  (1u<<1)   // GP27 (index 1 = GR_OUT_START)
#define MOUT_UP     (1u<<2)   // GP26 (index 2)
#define MOUT_DOWN   (1u<<3)   // GP22 (index 3)
#define MOUT_LEFT   (1u<<4)   // GP21 (index 4)
#define MOUT_RIGHT  (1u<<5)   // GP20 (index 5)
#define MOUT_A      (1u<<6)   // GP18 (index 6)
#define MOUT_B      (1u<<7)   // GP17 (index 7)
#define MOUT_C      (1u<<8)   // GP16 (index 8)

// マクロシーケンス記述ヘルパー
#define MSTEP_OUT(b)      {(uint16_t)(b), 1}    // 出力 b を1フレーム
#define MSTEP_HOLD(b, n)  {(uint16_t)(b), (n)}  // 出力 b をnフレーム
#define MSTEP_WAIT(n)     {0x0000,        (n)}  // nフレーム何もしない
#define MSTEP_END         {0xFFFF,        0}    // 終端マーカー

// ボタン設定構造体
// フラッシュ(0x1C3000)レイアウト (18バイト):
//   [0..2]  : スロット0 = TOP_Left (mode, gpio, rapid_off)
//   [3..5]  : スロット1 = TOP_Center
//   [6..8]  : スロット2 = TOP_Right
//   [9..11] : スロット3 = BTM_Left
//   [12..14]: スロット4 = BTM_Center
//   [15..17]: スロット5 = BTM_Right
// デフォルト (未初期化=0xFF の場合):
//   slot0=HOLD/GP18, slot1=HOLD/GP17, slot2=HOLD/GP16
//   slot3=RAPID_ROTARY/GP18, slot4=HOLD/GP17, slot5=HOLD/GP16
typedef struct {
    uint8_t mode;       // GR_BTN_MODE_*
    uint8_t gpio;       // 出力先GPIO番号 (0-28)
    uint8_t rapid_off;  // OFFフレーム数 (1-6, RAPID_FIXED時のみ使用)
} GR_ButtonConfig;

GR_ButtonConfig GR_Btn_Config[6];  // [0]=TOP_Left [1]=TOP_Center [2]=TOP_Right [3]=BTM_Left [4]=BTM_Center [5]=BTM_Right

// GR入力状態 (スワップ適用後の論理スロット単位)
static bool     GR_BtnState[6] = {false};  // [0..5] 各スロットの押下状態
static bool     GR_BtnStatePrev[6] = {false}; // 前フレームの押下状態 (立ち上がりエッジ検出用)
static uint32_t GR_ResetFrameCount = 0;
static uint8_t  GR_CurrentRapidOff = 0;    // 現在のOFFフレーム数 (0=連射なし)
static uint8_t  GR_RapidCnt[6] = {0};      // 各スロットの連射カウンタ

// マクロ実行状態
typedef struct {
    const MacroStep *steps;   // 実行中シーケンス
    uint8_t          step;    // 現在のステップインデックス
    uint8_t          frame;   // 現ステップ内フレームカウンタ
    bool             active;  // 実行中フラグ
} GR_MacroRunState;
static GR_MacroRunState GR_MacroState[6];  // 各スロットのマクロ実行状態

// GPIO関連(設定)
bool SettingMode = false;
#define LED_PIN 25 // 標準LED

bool UsbConnected = false;  // USB接続検出フラグ

uint8_t OnFrame = 1;
uint8_t OffFrame = 1;

// グローバル変数として前回の実行時間を追加
static absolute_time_t last_sync_time;

// 選択状態
unsigned char DispPanel[9] = {'0', '0', '0', '0', '0', '0', '0', '0', '0'};

// flash関連
// PicoRapidX2GR専用アドレス (Block28内の空き領域を使用)
// 0x1C0000: VSyncSeparator設定 (他プロジェクト)
// 0x1C1000: PicoRapidX2GR IO設定 (専用)
// 0x1C2000: PicoRapidX2GR ボード設定 (専用)
// 0x1C3000: PicoRapidX2GR EF設定 (専用)
// 0x1D0000: IO_Board (PicoRapidX/PicoRapidX2 共用)
// 0x1E0000-0x1EF000: Macro 0-15 (全プロジェクト共用)
// 0x1F0000: IO_Setting (PicoRapidX/PicoRapidX2 共用)
const uint32_t FLASH_TARGET_OFFSET_IO_Setting = 0x1C1000;
const uint32_t FLASH_TARGET_OFFSET_IO_Board   = 0x1C2000;
const uint32_t FLASH_TARGET_OFFSET_EF_Setting = 0x1C3000;
uint8_t g_read_io_data[FLASH_PAGE_SIZE];
uint8_t g_save_io_data[FLASH_PAGE_SIZE];
uint8_t g_read_macro_data[FLASH_PAGE_SIZE];
uint8_t g_read_macro_repeat_data[FLASH_PAGE_SIZE];
uint8_t g_save_macro_repeat_data[FLASH_PAGE_SIZE];

// マクロの保存ブロック
const uint32_t FLASH_TARGET_OFFSET_Macro_0 = 0x1E0000;
const uint32_t FLASH_TARGET_OFFSET_Macro_1 = 0x1E1000;
const uint32_t FLASH_TARGET_OFFSET_Macro_2 = 0x1E2000;
const uint32_t FLASH_TARGET_OFFSET_Macro_3 = 0x1E3000;
const uint32_t FLASH_TARGET_OFFSET_Macro_4 = 0x1E4000;
const uint32_t FLASH_TARGET_OFFSET_Macro_5 = 0x1E5000;
const uint32_t FLASH_TARGET_OFFSET_Macro_6 = 0x1E6000;
const uint32_t FLASH_TARGET_OFFSET_Macro_7 = 0x1E7000;
const uint32_t FLASH_TARGET_OFFSET_Macro_8 = 0x1E8000;
const uint32_t FLASH_TARGET_OFFSET_Macro_9 = 0x1E9000;
const uint32_t FLASH_TARGET_OFFSET_Macro_10 = 0x1EA000;
const uint32_t FLASH_TARGET_OFFSET_Macro_11 = 0x1EB000;
const uint32_t FLASH_TARGET_OFFSET_Macro_12 = 0x1EC000;
const uint32_t FLASH_TARGET_OFFSET_Macro_13 = 0x1ED000;
const uint32_t FLASH_TARGET_OFFSET_Macro_14 = 0x1EE000;
const uint32_t FLASH_TARGET_OFFSET_Macro_15 = 0x1EF000;

// 入出力設定
bool Rapid = false; // 表/裏管理用
int8_t SyncCount_15 = -1; // 15連表/裏管理用

const int IOCount = 9;

// 入力内容に対応した出力内容を設定する構造体
typedef struct {
    bool Reverse;           // 押下反転フラグ

    uint8_t RapidType;          // 連射設定 1..7
    uint8_t CommandType;        // コマンドNo(0-11)

    uint8_t RepeatMode;         // リピート回数 0:無制限
    uint8_t RepeatCount;        // 現在のリピートカウント

    uint8_t OutputFrame;        // 出力フレーム数
    uint8_t IntervalFrame;      // インターバルフレーム数

    uint8_t OutputFrameCount;   // 出力中フレーム数
    uint8_t IntervalFrameCount; // インターバル中フレーム数

    int16_t OutputGPIONo[9];    // 出力するGPIO番号を格納（-1 を含むため符号付）

    bool OutputNo[9];       // 出力するPinのID

    int16_t CurrentPin;         // 現在処理しているOutputGPIONoを格納
} IOSettingDef;
IOSettingDef IOSetting[9];

// 設定モード関連
IOSettingDef IOSetting_Current;
uint8_t SelectInputNo = 0;

bool GPIOStatusOn[9] = {false, false, false, false, false, false, false, false, false}; // 出力中ステータス

// コマンド管理用宣言
uint8_t ExecuteInputNo = 0; // 実行中の入力番号を格納
uint8_t LastFrameCount[9];  // 各入力番号で登録されたコマンドのフレーム数
const uint16_t MaxMacroFrame = 120; // 256以上にする場合はアドレスが重複してしまうので処理を変更する必要あり
// マクロ設定/実行セットを16ビットにビットパック（bit0=有効フレーム, bit1-15=出力1-15）
uint16_t MacroSettingBits[9][150];
uint8_t MacroFrame = 1;
uint8_t MacroFrame_Total = 1;
uint8_t SelectMacroNo = 0;

typedef struct {
    bool LastButtonOn; // 前のフレームでのオン/オフ
    uint8_t CurrentFrame;  // 現在実行中のフレーム番号
} ButtonCommand;
ButtonCommand buttonCommands[9];   // 各入力に割り当てられたコマンド管理変数
uint16_t CommandSetBits[9][150];

// ビット操作ヘルパー
static inline bool bit_get_u16(const uint16_t v, int bit) { return (v >> bit) & 1u; }
static inline void bit_set_u16(uint16_t *v, int bit, bool on) {
    if (on) { *v |= (uint16_t)(1u << bit); }
    else    { *v &= (uint16_t)~(1u << bit); }
}

// マルチコア制御
//static semaphore_t sem;

void SetBoardMode();
void InitGPIO();
void InitGPIOSync();
void callback_sync(uint gpio, uint32_t events);
void SetCommandData(int InputNo);
void GetInput_GR();
void InputExecute_GR();
static void ProcessButton_GR(int slot, bool pressed, uint8_t *counter);
void LoadButtonConfig();
void SaveButtonConfig();
void InputNormal(int InputNo);
void InputCommand(int InputNo);
void load_io_setting_from_flash(uint32_t load_address, uint8_t *read_data);
static void save_io_setting_to_flash(uint32_t save_address, uint8_t *save_data, uint32_t flash_size);
void InitGPIO_Macro();
void callback_check(uint gpio, uint32_t events);
void SetIOData();
void ModeSelect();
void EnterPush();
void DispRapidMessage();
static void vsync_callback(void);

int main() {
    // 基本クロック/USBなどボード初期化
    board_init();

    // ボード設定の読込
    SetBoardMode();

    // VBUS検出（GPIO24）でUSB接続を判定
    gpio_init(VBUS_PIN);
    gpio_set_dir(VBUS_PIN, GPIO_IN);
    busy_wait_ms(5);  // 安定化待ち
    UsbConnected = gpio_get(VBUS_PIN);

    if (UsbConnected) {
        // === USB接続モード: MSCデバイスとして動作 ===
        InitGPIO();

        // MSC初期化
        usb_msc_start();

        // MSCメインループ
        while (true) {
            tud_task();
            usb_msc_task();
            sleep_ms(1);
        }
    }
        
    // === USB未接続モード: VSync検出 + 連射 ===
    InitGPIO();

    // VSync分離器初期化（コールバック登録）
    vsync_separator_init_with_callback(vsync_callback);
    
    // メインループ: VSync検出タスク実行
    while (true) {
        vsync_separator_task();
    }
    return 0;
}

// 使用ボードの設定
void SetBoardMode() {
    for (int i = 0; i < IOCount; i++) {
        Output_Pin[i] = Output_Pin_GR[i];  // GR専用出力ピン固定
    }
}

// 入力・設定側初期化
void InitGPIO() {
    // 使用GPIOを標準SIOモードにリセット
    gpio_set_function(26, GPIO_FUNC_SIO);  // GP26: 上 (出力)
    gpio_set_function(27, GPIO_FUNC_SIO);  // GP27: スタート (出力)
    gpio_set_function(28, GPIO_FUNC_SIO);  // GP28: リセット (出力)

    // 出力ピンを個別に初期化
    for (int i = 0; i < IOCount; i++) {
        gpio_init(Output_Pin_GR[i]);
        gpio_set_dir(Output_Pin_GR[i], GPIO_OUT);
        gpio_put(Output_Pin_GR[i], 0);
        gpio_set_drive_strength(Output_Pin_GR[i], GPIO_DRIVE_STRENGTH_2MA);
    }

    // GRボタン入力ピン初期化 (プルアップ)
    const uint8_t gr_btn_pins[] = {
        GR_PIN_TOP_L, GR_PIN_TOP_M, GR_PIN_TOP_R,
        GR_PIN_BTM_L, GR_PIN_BTM_M, GR_PIN_BTM_R,
        GR_PIN_SWAP, GR_PIN_RESET
    };
    for (int i = 0; i < 8; i++) {
        gpio_init(gr_btn_pins[i]);
        gpio_set_dir(gr_btn_pins[i], GPIO_IN);
        gpio_pull_up(gr_btn_pins[i]);
    }

    // ロータリースイッチピン初期化 (プルアップ)
    for (int p = GR_PIN_ROT1; p <= GR_PIN_ROT6; p++) {
        gpio_init(p);
        gpio_set_dir(p, GPIO_IN);
        gpio_pull_up(p);
    }

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_set_drive_strength(LED_PIN, GPIO_DRIVE_STRENGTH_2MA);

    // ボタン設定をフラッシュから読み込み、対応出力GPIOを初期化
    LoadButtonConfig();
    for (int i = 0; i < 6; i++) {
        if (GR_Btn_Config[i].mode != GR_BTN_MODE_DISABLED) {
            uint8_t g = GR_Btn_Config[i].gpio;
            // マクロ出力先(GP27/GP28)はOutput_Pin_GRの初期化ループで既に設定済み
            if (g != GR_GPIO_MACRO_START &&
                g != GR_GPIO_MACRO_RESET &&
                g != GR_GPIO_MACRO_RESETSTART) {
                gpio_init(g);
                gpio_set_dir(g, GPIO_OUT);
                gpio_put(g, 0);
                gpio_set_drive_strength(g, GPIO_DRIVE_STRENGTH_2MA);
            }
        }
    }
    gpio_put(LED_PIN, 1); // 初期状態でLED ON（VSync検出待ち）

}

// VSync検出コールバック
static void vsync_callback(void) {
    // VSync 2フレームごとにLEDをトグル（約30Hzで点滅）
    // ただし偶数回検出時の相殺を防ぐため1フレーム分のON/OFFパルスで実現
    static uint32_t frame_count = 0;
    frame_count++;
    // 偶数フレーム: ON / 奇数フレーム: OFF → 2フレームで1周期
    gpio_put(LED_PIN, (frame_count & 1) ? 1 : 0);

    // GR入力取得 → 出力処理
    GetInput_GR();
    InputExecute_GR();
}

// GR専用入力取得
// スワップスイッチの状態に応じて物理ボタンをスロット0-5に割り当てる
// スワップOFF: TOP_L→slot0, TOP_M→slot1, TOP_R→slot2, BTM_L→slot3, BTM_M→slot4, BTM_R→slot5
// スワップON:  BTM_L→slot0, BTM_M→slot1, BTM_R→slot2, TOP_L→slot3, TOP_M→slot4, TOP_R→slot5
void GetInput_GR() {
    uint32_t val = gpio_get_all();

    // 物理ボタンの状態を読み取る (LOW=押下)
    bool top_l = ((val >> GR_PIN_TOP_L) & 1) == 0;
    bool top_m = ((val >> GR_PIN_TOP_M) & 1) == 0;
    bool top_r = ((val >> GR_PIN_TOP_R) & 1) == 0;
    bool btm_l = ((val >> GR_PIN_BTM_L) & 1) == 0;
    bool btm_m = ((val >> GR_PIN_BTM_M) & 1) == 0;
    bool btm_r = ((val >> GR_PIN_BTM_R) & 1) == 0;

    // 上下切替スイッチ (LOW=ON=スワップ)
    bool swap = ((val >> GR_PIN_SWAP) & 1) == 0;

    if (!swap) {
        GR_BtnState[0] = top_l;
        GR_BtnState[1] = top_m;
        GR_BtnState[2] = top_r;
        GR_BtnState[3] = btm_l;
        GR_BtnState[4] = btm_m;
        GR_BtnState[5] = btm_r;
    } else {
        GR_BtnState[0] = btm_l;
        GR_BtnState[1] = btm_m;
        GR_BtnState[2] = btm_r;
        GR_BtnState[3] = top_l;
        GR_BtnState[4] = top_m;
        GR_BtnState[5] = top_r;
    }

    // リセットボタン (LOW=押下)
    bool reset_pressed = ((val >> GR_PIN_RESET) & 1) == 0;
    if (reset_pressed) {
        GR_ResetFrameCount++;
    } else {
        GR_ResetFrameCount = 0;
    }

    // ロータリースイッチ読み取り (LOW=選択中)
    GR_CurrentRapidOff = 0;  // デフォルト=連射なし
    for (int i = 0; i < 6; i++) {
        if (((val >> (GR_PIN_ROT1 + i)) & 1) == 0) {
            GR_CurrentRapidOff = GR_RapidOffFrames[i];
            break;
        }
    }
}

// GPIO番号またはマクロ番号に対してオン/オフを出力する
static inline void gr_output_set(uint8_t gpio_val, bool on) {
    if (gpio_val == GR_GPIO_MACRO_START) {
        gpio_put(Output_Pin_GR[GR_OUT_START], on ? 1 : 0);
    } else if (gpio_val == GR_GPIO_MACRO_RESET) {
        gpio_put(Output_Pin_GR[GR_OUT_RESET], on ? 1 : 0);
    } else if (gpio_val == GR_GPIO_MACRO_RESETSTART) {
        gpio_put(Output_Pin_GR[GR_OUT_RESET], on ? 1 : 0);
        gpio_put(Output_Pin_GR[GR_OUT_START], on ? 1 : 0);
    } else {
        gpio_put(gpio_val, on ? 1 : 0);
    }
}

// ---- 埋め込みマクロシーケンス定義 ----
// ここのシーケンスを編集してマクロ動作をカスタマイズすること。

// STARTマクロ (GR_GPIO_MACRO_START=100)
static const MacroStep GR_MacroSeq_Start[] = {
    MSTEP_HOLD(MOUT_UP,    2), MSTEP_WAIT(2),  // ↑
    MSTEP_HOLD(MOUT_UP,    2), MSTEP_WAIT(2),  // ↑
    MSTEP_HOLD(MOUT_DOWN,  2), MSTEP_WAIT(2),  // ↓
    MSTEP_HOLD(MOUT_DOWN,  2), MSTEP_WAIT(2),  // ↓
    MSTEP_HOLD(MOUT_LEFT,  2), MSTEP_WAIT(2),  // ←
    MSTEP_HOLD(MOUT_RIGHT, 2), MSTEP_WAIT(2),  // →
    MSTEP_HOLD(MOUT_LEFT,  2), MSTEP_WAIT(2),  // ←
    MSTEP_HOLD(MOUT_RIGHT, 2), MSTEP_WAIT(2),  // →
    MSTEP_HOLD(MOUT_A,     2), MSTEP_WAIT(2),  // A
    MSTEP_HOLD(MOUT_B,     2), MSTEP_WAIT(2),  // B
    MSTEP_HOLD(MOUT_C,     2),                 // C
    MSTEP_END,
};

// RESETマクロ (GR_GPIO_MACRO_RESET=101)
static const MacroStep GR_MacroSeq_Reset[] = {
    MSTEP_OUT(MOUT_RESET),              // Frame 1: GP28 ON
    MSTEP_WAIT(1),                      // Frame 2: 待機
    MSTEP_OUT(MOUT_A | MOUT_B | MOUT_C), // Frame 3: GP18/17/16(A/B/C) ON
    MSTEP_END,
};

// RESET+STARTマクロ (GR_GPIO_MACRO_RESETSTART=102)
static const MacroStep GR_MacroSeq_ResetStart[] = {
    // --- RESET シーケンス ---
    MSTEP_OUT(MOUT_RESET),               // Frame 1: GP28(RESET) ON
    MSTEP_WAIT(1),                       // Frame 2: 待機
    MSTEP_OUT(MOUT_A | MOUT_B | MOUT_C), // Frame 3: A/B/C ON
    // --- 2700フレーム待機 (255×10 + 150) ---
    MSTEP_WAIT(255), MSTEP_WAIT(255), MSTEP_WAIT(255), MSTEP_WAIT(255), MSTEP_WAIT(255),
    MSTEP_WAIT(255), MSTEP_WAIT(255), MSTEP_WAIT(255), MSTEP_WAIT(255), MSTEP_WAIT(255),
    MSTEP_WAIT(150),
    // --- START シーケンス (コナミコマンド) ---
    MSTEP_HOLD(MOUT_UP,    2), MSTEP_WAIT(2),  // ↑
    MSTEP_HOLD(MOUT_UP,    2), MSTEP_WAIT(2),  // ↑
    MSTEP_HOLD(MOUT_DOWN,  2), MSTEP_WAIT(2),  // ↓
    MSTEP_HOLD(MOUT_DOWN,  2), MSTEP_WAIT(2),  // ↓
    MSTEP_HOLD(MOUT_LEFT,  2), MSTEP_WAIT(2),  // ←
    MSTEP_HOLD(MOUT_RIGHT, 2), MSTEP_WAIT(2),  // →
    MSTEP_HOLD(MOUT_LEFT,  2), MSTEP_WAIT(2),  // ←
    MSTEP_HOLD(MOUT_RIGHT, 2), MSTEP_WAIT(2),  // →
    MSTEP_HOLD(MOUT_A,     2), MSTEP_WAIT(2),  // A
    MSTEP_HOLD(MOUT_B,     2), MSTEP_WAIT(2),  // B
    MSTEP_HOLD(MOUT_C,     2),                 // C
    MSTEP_END,
};

// マクロステップの出力ビットを Output_Pin_GR に適用する
static void gr_macro_apply(uint16_t bits, bool on) {
    for (int i = 0; i < 9; i++) {
        if (bits & (1u << i)) {
            gpio_put(Output_Pin_GR[i], on ? 1 : 0);
        }
    }
}

// いずれかのスロットでマクロが実行中かどうか
static inline bool GR_AnyMacroActive(void) {
    for (int i = 0; i < 6; i++) {
        if (GR_MacroState[i].active) return true;
    }
    return false;
}

// スロットのマクロを開始する (実行中の場合は先頭から再スタート)
static void gr_start_macro(int slot, uint8_t macro_type) {
    const MacroStep *seq;
    if      (macro_type == GR_GPIO_MACRO_START) seq = GR_MacroSeq_Start;
    else if (macro_type == GR_GPIO_MACRO_RESET) seq = GR_MacroSeq_Reset;
    else                                        seq = GR_MacroSeq_ResetStart;

    // 初回起動時 (他のマクロも未実行) はボタン入力出力を全クリア
    if (!GR_AnyMacroActive()) {
        for (int i = 0; i < 6; i++) {
            const GR_ButtonConfig *cfg = &GR_Btn_Config[i];
            if (cfg->mode != GR_BTN_MODE_DISABLED &&
                cfg->gpio != GR_GPIO_MACRO_START &&
                cfg->gpio != GR_GPIO_MACRO_RESET &&
                cfg->gpio != GR_GPIO_MACRO_RESETSTART) {
                gr_output_set(cfg->gpio, false);
            }
            GR_RapidCnt[i] = 0;
        }
    }

    GR_MacroState[slot].steps  = seq;
    GR_MacroState[slot].step   = 0;
    GR_MacroState[slot].frame  = 0;
    GR_MacroState[slot].active = true;
}

// アクティブな全スロットのマクロを 1フレーム進める
static void TickMacros_GR(void) {
    for (int slot = 0; slot < 6; slot++) {
        GR_MacroRunState *st = &GR_MacroState[slot];
        if (!st->active) continue;

        const MacroStep *cur = &st->steps[st->step];

        // 終端チェック
        if (cur->outputs == 0xFFFF) {
            st->active = false;
            continue;
        }

        // 出力ありステップ: GPIOをアサート
        if (cur->outputs != 0) {
            gr_macro_apply(cur->outputs, true);
        }
        // outputs==0（WAIT）のときは何もしない（前ステップ終了時に解除済み）

        // フレームカウンタ更新
        st->frame++;
        if (st->frame >= cur->frames) {
            st->frame = 0;
            // 出力ステップ終了時に解除
            if (cur->outputs != 0) {
                gr_macro_apply(cur->outputs, false);
            }
            st->step++;
        }
    }
}

// 単一スロットのボタン出力処理ヘルパー
static void ProcessButton_GR(int slot, bool pressed, uint8_t *counter) {
    const GR_ButtonConfig *cfg = &GR_Btn_Config[slot];

    // マクロ出力先: 立ち上がりエッジでシングルショット起動 (モード問わず)
    if (cfg->gpio == GR_GPIO_MACRO_START ||
        cfg->gpio == GR_GPIO_MACRO_RESET ||
        cfg->gpio == GR_GPIO_MACRO_RESETSTART) {
        if (pressed && !GR_BtnStatePrev[slot]) {
            gr_start_macro(slot, cfg->gpio);
        }
        *counter = 0;
        return;
    }

    switch (cfg->mode) {
        case GR_BTN_MODE_DISABLED:
            // 何もしない
            break;
        case GR_BTN_MODE_HOLD:
            gr_output_set(cfg->gpio, pressed);
            *counter = 0;
            break;
        case GR_BTN_MODE_RAPID_ROTARY:
            if (pressed) {
                if (GR_CurrentRapidOff == 0) {
                    gr_output_set(cfg->gpio, true);
                } else {
                    gr_output_set(cfg->gpio, *counter == 0);
                    (*counter)++;
                    if (*counter > GR_CurrentRapidOff) *counter = 0;
                }
            } else {
                gr_output_set(cfg->gpio, false);
                *counter = 0;
            }
            break;
        case GR_BTN_MODE_RAPID_FIXED:
            if (pressed) {
                if (cfg->rapid_off == 0) {
                    gr_output_set(cfg->gpio, true);
                } else {
                    gr_output_set(cfg->gpio, *counter == 0);
                    (*counter)++;
                    if (*counter > cfg->rapid_off) *counter = 0;
                }
            } else {
                gr_output_set(cfg->gpio, false);
                *counter = 0;
            }
            break;
    }
}

// GR専用出力処理
void InputExecute_GR() {
    // リセット: GR_RESET_HOLD_FRAMES 継続押下でアサート
    gpio_put(Output_Pin_GR[GR_OUT_RESET],
             (GR_ResetFrameCount >= GR_RESET_HOLD_FRAMES) ? 1 : 0);

    // アクティブなマクロを 1フレーム進める
    TickMacros_GR();

    // マクロ実行中は再押下・他の入力をすべて無視する
    if (!GR_AnyMacroActive()) {
        for (int i = 0; i < 6; i++) {
            ProcessButton_GR(i, GR_BtnState[i], &GR_RapidCnt[i]);
        }
    }

    // 前フレームのボタン状態を保存 (立ち上がりエッジ検出用)
    for (int i = 0; i < 6; i++) {
        GR_BtnStatePrev[i] = GR_BtnState[i];
    }
}

void load_io_setting_from_flash(uint32_t load_address, uint8_t *read_data)
{
    // XIP_BASE(0x10000000)はflash.hで定義済み
    const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + load_address);
    memcpy(read_data, flash_target_contents, 256);
}

static void save_io_setting_to_flash(uint32_t save_address, uint8_t *save_data, uint32_t flash_size)
{
    printf("savestart!\n");
    gpio_put(LED_PIN, 1);
    // 割り込み無効にする
    uint32_t ints = save_and_disable_interrupts();
    printf("stop_interrupts!\n");
    busy_wait_ms(200);
    // Flash消去。
    // 消去単位はflash.hで定義されている FLASH_SECTOR_SIZE(4096Byte) の倍数とする
    flash_range_erase(save_address, FLASH_SECTOR_SIZE);
    gpio_put(LED_PIN, 0);
    printf("erase_flash!\n");
    busy_wait_ms(100);
    // Flash書き込み。
    gpio_put(LED_PIN, 0);
    // 書込単位はflash.hで定義されている FLASH_PAGE_SIZE(256Byte) の倍数とする
    flash_range_program(save_address, save_data, flash_size);
    printf("write_flash!\n");
    busy_wait_ms(200);
    // 割り込みフラグを戻す
    gpio_put(LED_PIN, 1);
    restore_interrupts(ints);
    printf("start_interrupts!\n");
    busy_wait_ms(200);
    gpio_put(LED_PIN, 0);

    printf("saved!\n");
}

// EF設定をフラッシュから読み込む
// フラッシュレイアウト(0x1C3000):
//   [0] E mode, [1] E gpio, [2] E rapid_off
//   [3] F mode, [4] F gpio, [5] F rapid_off
// ボタン設定のデフォルト値
// スロット: 0=TOP_Left, 1=TOP_Center, 2=TOP_Right, 3=BTM_Left, 4=BTM_Center, 5=BTM_Right
static const GR_ButtonConfig GR_Btn_Default[6] = {
    {GR_BTN_MODE_HOLD,         18, 1},  // slot0: TOP_Left  → GP18 ホールド
    {GR_BTN_MODE_HOLD,         17, 1},  // slot1: TOP_Center → GP17 ホールド
    {GR_BTN_MODE_HOLD,         16, 1},  // slot2: TOP_Right → GP16 ホールド
    {GR_BTN_MODE_RAPID_ROTARY, 18, 1},  // slot3: BTM_Left  → GP18 連射(ロータリー)
    {GR_BTN_MODE_HOLD,         17, 1},  // slot4: BTM_Center → GP17 ホールド
    {GR_BTN_MODE_HOLD,         16, 1},  // slot5: BTM_Right → GP16 ホールド
};

// ボタン設定をフラッシュから読み込む
void LoadButtonConfig() {
    uint8_t buf[FLASH_PAGE_SIZE];
    load_io_setting_from_flash(FLASH_TARGET_OFFSET_EF_Setting, buf);

    for (int i = 0; i < 6; i++) {
        uint8_t mode      = buf[i * 3 + 0];
        uint8_t gpio_pin  = buf[i * 3 + 1];
        uint8_t rapid_off = buf[i * 3 + 2];

        // 未初期化(0xFF)またはモード範囲外はデフォルト値を使用
        if (mode > GR_BTN_MODE_RAPID_FIXED || mode == 0xFF) {
            GR_Btn_Config[i] = GR_Btn_Default[i];
        } else {
            // 旧フォーマット互換: 27(START)→100, 28/99(RESET)→101 に変換
            if (gpio_pin == 27) {
                gpio_pin = GR_GPIO_MACRO_START;
            } else if (gpio_pin == 28 || gpio_pin == 99) {
                gpio_pin = GR_GPIO_MACRO_RESET;
            } else if (gpio_pin != 18 && gpio_pin != 17 && gpio_pin != 16 &&
                       gpio_pin != GR_GPIO_MACRO_START &&
                       gpio_pin != GR_GPIO_MACRO_RESET &&
                       gpio_pin != GR_GPIO_MACRO_RESETSTART) {
                gpio_pin = GR_Btn_Default[i].gpio;
            }
            if (rapid_off == 0 || rapid_off > 6 || rapid_off == 0xFF) rapid_off = 1;
            GR_Btn_Config[i].mode      = mode;
            GR_Btn_Config[i].gpio      = gpio_pin;
            GR_Btn_Config[i].rapid_off = rapid_off;
        }
    }
}

// ボタン設定をフラッシュに保存する
void SaveButtonConfig() {
    uint8_t buf[FLASH_PAGE_SIZE];
    memset(buf, 0xFF, sizeof(buf));

    for (int i = 0; i < 6; i++) {
        buf[i * 3 + 0] = GR_Btn_Config[i].mode;
        buf[i * 3 + 1] = GR_Btn_Config[i].gpio;
        buf[i * 3 + 2] = GR_Btn_Config[i].rapid_off;
    }

    save_io_setting_to_flash(FLASH_TARGET_OFFSET_EF_Setting, buf, FLASH_PAGE_SIZE);
}
