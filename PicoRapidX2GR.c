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

// ディスプレイ関連
#define _SSD1306_H_

#define SSD1306_COLUMNS (128)
#define SSD1306_ROWS (64)
#define SSD1306_PAGES (SSD1306_ROWS / 8)
#define SSD1306_CANVAS_SIZE (SSD1306_COLUMNS * SSD1306_PAGES)

extern bool Ssd1306_Init();
extern unsigned char* Ssd1306_Get_Draw_Canvas();
extern void Ssd1306_Update_Frame();

#define SSD1306_DEV_ADR (0x3C)
#define SSD1306_CONFIG_MUX_RATIO_CMD (0xA8)
#define SSD1306_CONFIG_MUX_RATIO_A (0x3F)
#define SSD1306_CONFIG_DISPLAY_OFFSET_CMD (0xD3)
#define SSD1306_CONFIG_DISPLAY_OFFSET_A (0x0)
#define SSD1306_CONFIG_DISPLAY_START_LINE (0x40)
#define SSD1306_CONFIG_SEGMENT_REMAP (0xA1)
#define SSD1306_CONFIG_COM_OUT_DIRECTION (0xC8)
#define SSD1306_CONFIG_COM_PIN_CONFIG_CMD (0xDA)
#define SSD1306_CONFIG_COM_PIN_CONFIG_A (0x12)
#define SSD1306_CONFIG_CONTRAST_CMD (0x81)
#define SSD1306_CONFIG_CONTRAST_A (0x7F)
#define SSD1306_CONFIG_ENTIRE_DISPLAY_ON (0xA4)
#define SSD1306_CONFIG_DISPLAY_PIX_MODE (0xA6)
#define SSD1306_CONFIG_DISPLAY_FREQ_CMD (0xD5)
#define SSD1306_CONFIG_DISPLAY_FREQ_A (0xF0)
#define SSD1306_CONFIG_ADDRESSING_MODE_CMD (0x20)
#define SSD1306_CONFIG_ADDRESSING_MODE_A (0x0)
#define SSD1306_CONFIG_CHARGE_PUMP_CMD (0x8D)
#define SSD1306_CONFIG_CHARGE_PUMP_A (0x14)
#define SSD1306_CONFIG_DISPLAY_OFF (0xAE)
#define SSD1306_CONFIG_DISPLAY_ON (0xAF)
#define SSD1306_CTRL_BYTE_CMD_SINGLE (0b00000000)
#define SSD1306_CTRL_BYTE_CMD_STREAM (0b10000000)
#define SSD1306_CTRL_BYTE_DATA_SINGLE (0b01000000)
#define SSD1306_CTRL_BYTE_DATA_STREAM (0b11000000)

#define SSD1306_DRAW_CANVAS_PAYLOAD_SIZE (SSD1306_COLUMNS * SSD1306_PAGES + 1) //+1はコントロールバイト

/* 送信バッファ（キャンバス含む） */
static unsigned char Draw_Canvas_Payload[SSD1306_DRAW_CANVAS_PAYLOAD_SIZE] = {0};

/* キャンバスポインタ（送信バッファの所定アドレス） */
static unsigned char *p_Draw_Canvas = &Draw_Canvas_Payload[1];

/* 初期設定コマンドセット */
static const unsigned char SSD1306_Init_Config[] = {
    SSD1306_CONFIG_MUX_RATIO_CMD,
    SSD1306_CONFIG_MUX_RATIO_A,
    SSD1306_CONFIG_DISPLAY_OFFSET_CMD,
    SSD1306_CONFIG_DISPLAY_OFFSET_A,
    SSD1306_CONFIG_DISPLAY_START_LINE,
    SSD1306_CONFIG_SEGMENT_REMAP,
    SSD1306_CONFIG_COM_OUT_DIRECTION,
    SSD1306_CONFIG_COM_PIN_CONFIG_CMD,
    SSD1306_CONFIG_COM_PIN_CONFIG_A,
    SSD1306_CONFIG_CONTRAST_CMD,
    SSD1306_CONFIG_CONTRAST_A,
    SSD1306_CONFIG_ENTIRE_DISPLAY_ON,
    SSD1306_CONFIG_DISPLAY_PIX_MODE,
    SSD1306_CONFIG_DISPLAY_FREQ_CMD,
    SSD1306_CONFIG_DISPLAY_FREQ_A,
    SSD1306_CONFIG_ADDRESSING_MODE_CMD,
    SSD1306_CONFIG_ADDRESSING_MODE_A,
    SSD1306_CONFIG_CHARGE_PUMP_CMD,
    SSD1306_CONFIG_CHARGE_PUMP_A,
};
#define NUM_OF_SSD1306_CONFIG (sizeof(SSD1306_Init_Config)/sizeof(unsigned char))

typedef struct {
    char dispChar;
    unsigned char pattern_head[16];
    unsigned char pattern_foot[16];
} DispPatternDef;
DispPatternDef DispPattern[100];

static bool Ssd1306_Write(unsigned char *data, unsigned long length);

unsigned char *canvas;

const uint16_t col_byte = 16;
const uint16_t row_byte = 256;
const uint16_t row_byte_half = 128;

#define I2C_PORT i2c0
//#define SDA_PIN 26 // GP26
//#define SCL_PIN 27 // GP27
#define SDA_PIN 20 // GP20
#define SCL_PIN 21 // GP21

// GPIO関連
#define Sync_Pin 2   // CSync入力 (GP02)
#define VBUS_PIN 24  // VBUS検出ピン（USB接続検知用）

#define Sync_IRQ_enable   gpio_set_irq_enabled(Sync_Pin, 0x4u, true);
#define Sync_IRQ_disable  gpio_set_irq_enabled(Sync_Pin, 0x4u, false);

const int32_t maskGPIO   = 0b11101111111111111111111111111; // GP28-GP0 (GP28,27,26,22-0)
const int32_t maskIO     = 0b00000000000000000111111111111; // GP28-GP0 (GP11-0)

bool InputStatus[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

uint8_t boardMode = 0;

int Output_Pin[] = {11, 10, 9, 8, 6, 7, 5, 4, 3};
int Input_Pin[]  = {15, 14, 13, 12, 16, 17, 18, 19, 20, 21, 22, 26};

//int8_t Output_Pin[] = {4, 3, 11, 10, 9, 8, 7, 6, 5};
//int8_t Input_Pin[]  = {19, 20, 12, 13, 14, 15, 16, 17, 18, 21, 22, 26};

const int8_t Input_Pin_A[]  = {15, 14, 13, 12, 16, 17, 18, 19, 20, 21, 22, 26};

const int8_t Input_Pin_B[]  = {19, 20, 12, 13, 14, 15, 16, 17, 18, 21, 22, 26};

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
const int32_t maskGPIO_Macro = 0b1110000011110011111111111100; // GP19-GP0
const int32_t maskIO_Macro   = 0b0000000000000000000000000000; // GP19-GP0

const uint8_t Input_Pin_Macro[]  = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 16, 17, 18, 19, 26, 27};
const uint8_t GPIO_InputNo_Macro[] = {2, 3, 5, 4, 6, 7, 8, 9, 10, 11, 0, 1};

#define ModeSW_Pin 27
#define EnterSW_Pin 26
#define SettingSW_UL_Pin 19
#define SettingSW_UR_Pin 18
#define SettingSW_DL_Pin 17
#define SettingSW_DR_Pin 16

#define ModeSW_IRQ_enable   gpio_set_irq_enabled(ModeSW_Pin, 0x4u, true);
#define ModeSW_IRQ_disable  gpio_set_irq_enabled(ModeSW_Pin, 0x4u, false);
#define EnterSW_IRQ_enable   gpio_set_irq_enabled(EnterSW_Pin, 0x4u, true);
#define EnterSW_IRQ_disable  gpio_set_irq_enabled(EnterSW_Pin, 0x4u, false);
#define SettingSW_UL_IRQ_enable   gpio_set_irq_enabled(SettingSW_UL_Pin, 0x4u, true);
#define SettingSW_UL_IRQ_disable  gpio_set_irq_enabled(SettingSW_UL_Pin, 0x4u, false);
#define SettingSW_UR_IRQ_enable   gpio_set_irq_enabled(SettingSW_UR_Pin, 0x4u, true);
#define SettingSW_UR_IRQ_disable  gpio_set_irq_enabled(SettingSW_UR_Pin, 0x4u, false);
#define SettingSW_DL_IRQ_enable   gpio_set_irq_enabled(SettingSW_DL_Pin, 0x4u, true);
#define SettingSW_DL_IRQ_disable  gpio_set_irq_enabled(SettingSW_DL_Pin, 0x4u, false);
#define SettingSW_DR_IRQ_enable   gpio_set_irq_enabled(SettingSW_DR_Pin, 0x4u, true);
#define SettingSW_DR_IRQ_disable  gpio_set_irq_enabled(SettingSW_DR_Pin, 0x4u, false);

// モード定義 (enum化: Cで整数定数式として扱えるようにする)
enum SelectMode {
    SelectMode_Normal = 0,
    SelectMode_Input,
    SelectMode_Output,
    SelectMode_Rapid,
    SelectMode_Custom,
    SelectMode_Save,
    SelectMode_Macro,
    SelectMode_Repeat,
    SelectMode_USB
};

enum SelectRapid {
    SelectRapid_Normal = 0,
    SelectRapid_Sync30,
    SelectRapid_Sync30R,
    SelectRapid_20,
    SelectRapid_15,
    SelectRapid_12,
    SelectRapid_10,
    SelectRapid_75,
    SelectRapid_Custom,
    SelectRapid_Sync15,
    SelectRapid_Sync15R
};

enum ConfirmMode {
    ConfirmMode_Nothing = 0,
    ConfirmMode_Execute,
    ConfirmMode_Cancel
};

bool ModeSW_flg = false;
bool EnterSW_flg = false;
bool Button_Flg = false;
bool Check_FrameDelete_Flg = false;
bool Check_FrameInsert_Flg = false;
bool ChangeBoard_Flg = false;
bool UsbConnected = false;  // USB接続検出フラグ
bool FrameToggle = false;  // ループごとにtrue/falseを繰り返す

uint8_t SelectMode = SelectMode_Normal;
uint8_t SelectRapid = SelectRapid_Normal;

// YES/NO 選択 (順番: No=0, Yes=1)
enum SelectYesNo {
    SelectNo = 0,
    SelectYes = 1
};

bool jumpMode = false;

uint8_t OnFrame = 1;
uint8_t OffFrame = 1;

uint8_t Confirm_Mode = ConfirmMode_Nothing;
uint8_t Confirm_Flg = SelectYes;

bool SetMacroMode = false;

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

void core1_main();
void SetBoardMode();
void InitGPIO();
void InitGPIOSync();
void callback_sync(uint gpio, uint32_t events);
void SetIOSetting();
void SetCommandData(int InputNo);
void GetInput();
void InputExecute();
void GetInput_GR();
void InputExecute_GR();
static void ProcessButton_GR(int slot, bool pressed, uint8_t *counter);
void LoadButtonConfig();
void SaveButtonConfig();
void InputNormal(int InputNo);
void InputCommand(int InputNo);
void load_io_setting_from_flash(uint32_t load_address, uint8_t *read_data);
static void save_io_setting_to_flash(uint32_t save_address, uint8_t *save_data, uint32_t flash_size);
void StartSettingMode();
void InitGPIO_Macro();
void callback_check(uint gpio, uint32_t events);
void StartSetting();
void SetIOData();
void ModeSelect();
void EnterPush();
void DispRapidMessage();
void Setting_SW_UL();
void Setting_SW_UR();
void Setting_SW_DL();
void Setting_SW_DR();
void Input_SW_Push(int gpio);
void InitSetting();
void SetBoardSetting();
void SetOutputMode();
void SetRapidMode();
void SaveSetting();
void SaveMacro();
void DispConfirm(char *Message, int SelectSide);
void DispChange();
int CountMacroEnableFrame();
void DispMacroSetting();
void DeleteFrame();
void InsertFrame();
bool Ssd1306_Init();
static bool Ssd1306_Write(unsigned char *data, unsigned long length);
unsigned char* Ssd1306_Get_Draw_Canvas();
void Ssd1306_Update_Frame();
bool I2C_WriteData(unsigned char dev_adr, void *buf, unsigned long buf_length);
void DrawMessage(int row_pos, char *Message, bool reverse_flg);
DispPatternDef getcharPattern(char DispChar);
void DrawControlPanel(char InputState[9]);
void SetCharPattern(); 
void SetCharPattern_Number();
void SetCharPattern_ALPHABET();
void SetCharPattern_alphabet();
void SetCharPattern_symbol();
static void vsync_callback(void);

int main() {
    // 基本クロック/USBなどボード初期化
    board_init();

    // ボード設定の読込
    load_io_setting_from_flash(FLASH_TARGET_OFFSET_IO_Board, g_read_io_data);
    SetBoardMode();

    // VBUS検出（GPIO24）でUSB接続を判定
    gpio_init(VBUS_PIN);
    gpio_set_dir(VBUS_PIN, GPIO_IN);
    busy_wait_ms(5);  // 安定化待ち
    UsbConnected = gpio_get(VBUS_PIN);

    if (UsbConnected) {
        // === USB接続モード: MSCデバイスとして動作 ===
        InitGPIO();
        SetIOSetting();

        // I2C初期化（OLED用）
        i2c_init(I2C_PORT, 400 * 1000);
        gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
        gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
        gpio_pull_up(SDA_PIN);
        gpio_pull_up(SCL_PIN);

        // OLED初期化
        Ssd1306_Init();
        SetCharPattern();
        canvas = Ssd1306_Get_Draw_Canvas();
        DrawMessage(0, "USB Mode", false);
        DrawMessage(1, "MSC Ready", false);
        Ssd1306_Update_Frame();

        // MSC初期化
        usb_msc_start();

        // USB接続モードでは出力として使っていたGP26/GP27を入力として再設定
        gpio_init(ModeSW_Pin);
        gpio_set_dir(ModeSW_Pin, GPIO_IN);
        gpio_set_input_hysteresis_enabled(ModeSW_Pin, true);
        gpio_pull_up(ModeSW_Pin);
        gpio_set_irq_enabled_with_callback(ModeSW_Pin, 0x4u, true, callback_sync);
        gpio_init(EnterSW_Pin);
        gpio_set_dir(EnterSW_Pin, GPIO_IN);
        gpio_pull_up(EnterSW_Pin);

        // 起動時のボタンチェック（設定モード用）
        busy_wait_ms(2);

        bool mode_pressed = (gpio_get(ModeSW_Pin) == 0);
        if (mode_pressed) {
            // Modeボタン押下: 設定モードへ
            StartSettingMode();
        }

        // MSCメインループ
        while (true) {
            tud_task();
            usb_msc_task();

            // OLED表示更新（接続状態）
            static bool last_mounted = false;
            bool mounted = tud_mounted();
            if (mounted != last_mounted) {
                last_mounted = mounted;
                DrawMessage(1, mounted ? "MSC Ready" : "Wait USB...", false);
                Ssd1306_Update_Frame();
            }
            sleep_ms(1);
        }
    }
        
    // === USB未接続モード: VSync検出 + 連射 ===
    InitGPIO();
    SetIOSetting();

    // VSync分離器初期化（コールバック登録）
    vsync_separator_init_with_callback(vsync_callback);
    
    // メインループ: VSync検出タスク実行
    while (true) {
        vsync_separator_task();
    }
    return 0;
}

// 入力取得スレッド
void core1_main() {
    if (SettingMode) return;
    while (true) {
        GetInput();
    }
    return;
}

// 使用ボードの設定
void SetBoardMode() {
    boardMode = g_read_io_data[0];
    if (boardMode > 1) boardMode = 0;

    for (int i = 0; i < IOCount; i++) {
        Input_Pin[i] = (boardMode == 0) ? Input_Pin_A[i] : Input_Pin_B[i];
        Output_Pin[i] = Output_Pin_GR[i];  // GR専用出力ピン固定
    }
}

// 入力・設定側初期化
void InitGPIO() {
    // 使用GPIOを標準SIOモードにリセット
    gpio_set_function(2,  GPIO_FUNC_SIO);  // GP02: CSync入力
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
}

// VSync検出コールバック
static void vsync_callback(void) {
    // LED点滅
    static bool led_state = false;
    led_state = !led_state;
    gpio_put(LED_PIN, led_state);

    // GR入力取得 → 出力処理
    GetInput_GR();
    InputExecute_GR();
}

// 同期信号入力ピン初期化
void InitGPIOSync() {
    gpio_set_input_hysteresis_enabled(Sync_Pin, true);
    gpio_set_irq_enabled_with_callback(Sync_Pin, 0x4u, true, callback_sync);

    // 前回の実行時間を初期化
    last_sync_time = get_absolute_time();
}

// 入力コールバック
void callback_sync(uint gpio, uint32_t events) {
    if (events == 4u) {
        switch (gpio)
        {
        case Sync_Pin:
        {
            // ブロックを追加して宣言をcase直後にまとめる（C90互換のため）
            // 同期信号の入力
            absolute_time_t now = get_absolute_time();              // 現在の時間
            int64_t time_diff_us = absolute_time_diff_us(last_sync_time, now); // 経過時間(us)

            if (time_diff_us < 15000) {
                // 15ms未満の場合、残りの時間待機 (約16ms周期に揃える)
                busy_wait_us(16000 - time_diff_us);
            }

            // 前回の実行時間を更新
            last_sync_time = get_absolute_time();

            Sync_IRQ_disable;
            InputExecute();
            Sync_IRQ_enable;
            //gpio_put(LED_PIN, Rapid ? GPIO_IN : GPIO_OUT);
            break; // ブロック内breakでcaseを抜ける
        }
        case ModeSW_Pin:
            // 設定モードボタンの入力
            Sync_IRQ_disable
            ModeSW_IRQ_disable
            StartSettingMode();            
            ModeSW_IRQ_enable
            break;
        default:
            break;
        }
    }
}

// InputOutputDataの有効ボタンを設定する
void SetIOSetting() {
    int i, j, k,row_start;

    load_io_setting_from_flash(FLASH_TARGET_OFFSET_IO_Setting, g_read_io_data);

    // 初期化
    for (i = 0; i < IOCount; i++) {
        // 出力対象外設定
        for (j = 0; j < IOCount; j++) {
            IOSetting[i].OutputGPIONo[j] = -1;
        }
        
        k = 0;
        for (j = 0; j < 16; j++) {
            row_start = (i * 16) + j;
            switch (j) {
                case 0:
                    if (g_read_io_data[row_start] < 10) {
                        IOSetting[i].RapidType = g_read_io_data[row_start];
                        IOSetting[i].Reverse = false;
                    } else {
                        IOSetting[i].RapidType = g_read_io_data[row_start] - 10;
                        IOSetting[i].Reverse = true;
                    }
                    break;
                case 1:
                    IOSetting[i].OutputFrame = g_read_io_data[row_start]; // 出力フレーム数・連射設定
                    break;
                case 2:
                    IOSetting[i].IntervalFrame = g_read_io_data[row_start]; // インターバルフレーム数
                    break;
                case 3:
                    IOSetting[i].CommandType = g_read_io_data[row_start]; // コマンドNo
                    break;
                default:
                    if (j - 4 < IOCount) {
                        IOSetting[i].OutputNo[j - 4] = g_read_io_data[row_start];
                        // 少しでもループを減らしたいので対象のピン番号を前に詰める
                        if (g_read_io_data[row_start] == 1) {
                            IOSetting[i].OutputGPIONo[k] = Output_Pin[j - 4]; // 出力PIN
                            k++;
                        }
                    }
                    break;
            }
        }
    }

    // マクロ用変数の初期化
    memset(MacroSettingBits, 0, sizeof(MacroSettingBits));
    memset(CommandSetBits, 0, sizeof(CommandSetBits));
    for (int i = 0; i < IOCount; i++) {
        bit_set_u16(&MacroSettingBits[i][0], 0, true); // 1フレーム目は有効
    }

    uint32_t read_address = FLASH_TARGET_OFFSET_Macro_0;
    for (int i = 0; i < IOCount; i++) {
        for (int j = 0; j < 16; j++) {
            load_io_setting_from_flash(read_address, g_read_macro_data);
            for (int k = 0; k < 16; k++) {
                for (int l = 0; l < 16; l++) {
                    if (((j * 16) + k) < MaxMacroFrame) {
                        bit_set_u16(&MacroSettingBits[i][(j * 16) + k], l, g_read_macro_data[(k * 16) + l] != 0);
                    }
                }
            }
            read_address += FLASH_PAGE_SIZE;
        }
        if (IOSetting[i].RapidType == 5) {
            SetCommandData(i);
        }
    }

    read_address = FLASH_TARGET_OFFSET_Macro_15;
    load_io_setting_from_flash(read_address, g_read_macro_repeat_data);
    for (int i = 0; i < IOCount; i++) {
        IOSetting[i].RepeatMode = g_read_macro_repeat_data[i];
    }
}

// コマンド設定
void SetCommandData(int InputNo) {
    int i;    
    for (i = 1; i < 150; i++) {
        if (!bit_get_u16(MacroSettingBits[InputNo][i], 0)) {
            break;
        } else {
            for (int j = 1; j < 16; j++) {
                int k = j;
                if (boardMode == 1) {
                    if (j == 5) {
                        k = j + 1;
                    } else if (j == 6) {
                        k = j - 1;
                    }
                }
                bit_set_u16(&CommandSetBits[InputNo][i - 1], k - 1, bit_get_u16(MacroSettingBits[InputNo][i], j));
            }
        }
    }
    
    LastFrameCount[InputNo] = i - 1;
    
    // マクロが読み込まれたかはここで確認
    //if (CommandSet[7][2][7]== 1) gpio_put(LED_PIN, GPIO_OUT);
    //if (LastFrameCount[7] == 3) gpio_put(LED_PIN, GPIO_OUT);
}

// ボタンの状態を取得 (旧実装: 設定モード用)
void GetInput() {
    int32_t InputValue = gpio_get_all();
    for (int i = 0; i < IOCount; i++) {
        InputStatus[i] = (((InputValue >> Input_Pin[i]) & 1) == 0) ? true : false;
        if (IOSetting[i].Reverse == true) InputStatus[i] = !InputStatus[i];
    }
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
    MSTEP_HOLD(MOUT_RESET | MOUT_START, 3),  // RESET+START 同時押下 3フレーム
    MSTEP_WAIT(3),                           // 3フレーム待機
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

// 同期信号入力発生時の処理 (旧実装: 設定モード用)
void InputExecute() {
    Rapid = !Rapid;

    SyncCount_15++;
    if (SyncCount_15 == 4) SyncCount_15 = -1;

    int i;
    for (i = 0; i < IOCount; i++) {
        GPIOStatusOn[i] = false; 
    }

    for (i = 0; i < IOCount; i++) {
        // コマンドボタン
        if ((IOSetting[i].RapidType == 5) && (InputStatus[i] == true)) {
            // コマンドボタンが入力された
            if (buttonCommands[IOSetting[i].CommandType].LastButtonOn == false) {
                // 前フレームでボタンを押していない
                buttonCommands[IOSetting[i].CommandType].LastButtonOn = true;
                ExecuteInputNo = i;
                InputCommand(i); // マクロを実行
                return;
            }
        } else if ((IOSetting[i].RapidType == 5) 
                    && (InputStatus[i] == false) 
                    && (buttonCommands[IOSetting[ExecuteInputNo].CommandType].CurrentFrame == 0)) {
            buttonCommands[IOSetting[i].CommandType].LastButtonOn = false;
        }
    }

    // コマンドボタン2フレーム目以降
    if (buttonCommands[IOSetting[ExecuteInputNo].CommandType].CurrentFrame > 0) {
        // 実行中
        InputCommand(ExecuteInputNo);
        if (buttonCommands[IOSetting[ExecuteInputNo].CommandType].CurrentFrame == 0) ExecuteInputNo = 0;
        return;
    }

    // 通常ボタンが押された
    for (i = 0; i < IOCount; i++) {
        InputNormal(i);
    }
    return;
}

// 通常ボタンの処理
void InputNormal(int InputNo) {
    if (InputStatus[InputNo] == true) {
        // 出力確認はここのコメントを外す    
        // if (InputNo == 6) {
        //     gpio_put(LED_PIN, GPIO_OUT);
        // }
        if (IOSetting[InputNo].RapidType == 1) {
            // 押しっぱなし
            for (int i = 0; i < IOCount; i++) {
                if (IOSetting[InputNo].OutputGPIONo[i] == -1) break;
                gpio_put(IOSetting[InputNo].OutputGPIONo[i], GPIO_OUT);
                GPIOStatusOn[IOSetting[InputNo].OutputGPIONo[i]] = true;
            }
        } else if ((IOSetting[InputNo].RapidType == 2) || (IOSetting[InputNo].RapidType == 3)) {
            // 表裏連射(OutputFrameに2を入れると奇数フレームでON、3だと偶数フレームでON)
            if (((IOSetting[InputNo].RapidType == 2) && (Rapid == true)) || ((IOSetting[InputNo].RapidType == 3) && (Rapid == false))) {
                for (int i = 0; i < IOCount; i++) {
                    if (IOSetting[InputNo].OutputGPIONo[i] == -1) break;
                    gpio_put(IOSetting[InputNo].OutputGPIONo[i], GPIO_OUT);
                    GPIOStatusOn[IOSetting[InputNo].OutputGPIONo[i]] = true;
                }
            } else {
                for (int i = 0; i < IOCount; i++) {
                    if (IOSetting[InputNo].OutputGPIONo[i] == -1) break;
                    if (GPIOStatusOn[IOSetting[InputNo].OutputGPIONo[i]] == true) break;
                    gpio_put(IOSetting[InputNo].OutputGPIONo[i], GPIO_IN);
                    GPIOStatusOn[IOSetting[InputNo].OutputGPIONo[i]] = false;
                }
            }
        } else if (IOSetting[InputNo].RapidType == 4) {
            if (IOSetting[InputNo].OutputFrameCount < IOSetting[InputNo].OutputFrame) {
                // 指定フレーム数だけ出力
                for (int i = 0; i < IOCount; i++) {
                    if (IOSetting[InputNo].OutputGPIONo[i] == -1) break;
                    gpio_put(IOSetting[InputNo].OutputGPIONo[i], GPIO_OUT);
                    GPIOStatusOn[IOSetting[InputNo].OutputGPIONo[i]] = true;
                }
                // 出力フレームカウントアップ
                IOSetting[InputNo].OutputFrameCount++;
            } else {
                // インターバルフレームをカウント
                IOSetting[InputNo].IntervalFrameCount++;

                // 指定フレーム数に達したらオフ
                for (int i = 0; i < IOCount; i++) {
                    if (IOSetting[InputNo].OutputGPIONo[i] == -1) break;
                    if (GPIOStatusOn[IOSetting[InputNo].OutputGPIONo[i]] == true) break;
                    gpio_put(IOSetting[InputNo].OutputGPIONo[i], GPIO_IN);
                    GPIOStatusOn[IOSetting[InputNo].OutputGPIONo[i]] = false;
                }

                if (IOSetting[InputNo].IntervalFrameCount == IOSetting[InputNo].IntervalFrame) {
                    // インターバルフレーム数に達したらカウントリセット
                    IOSetting[InputNo].OutputFrameCount = 0;
                    IOSetting[InputNo].IntervalFrameCount = 0;
                }
            }
        } else if ((IOSetting[InputNo].RapidType == 6) || (IOSetting[InputNo].RapidType == 7)) {
            // 表裏15連
            if (((IOSetting[InputNo].RapidType == 6) && (SyncCount_15 < 2)) || ((IOSetting[InputNo].RapidType == 7) && (SyncCount_15 > 2))) {
                for (int i = 0; i < IOCount; i++) {
                    if (IOSetting[InputNo].OutputGPIONo[i] == -1) break;
                    gpio_put(IOSetting[InputNo].OutputGPIONo[i], GPIO_OUT);
                    GPIOStatusOn[IOSetting[InputNo].OutputGPIONo[i]] = true;
                }
            } else {
                for (int i = 0; i < IOCount; i++) {
                    if (IOSetting[InputNo].OutputGPIONo[i] == -1) break;
                    if (GPIOStatusOn[IOSetting[InputNo].OutputGPIONo[i]] == true) break;
                    gpio_put(IOSetting[InputNo].OutputGPIONo[i], GPIO_IN);
                    GPIOStatusOn[IOSetting[InputNo].OutputGPIONo[i]] = false;
                }
            }
        }
    } else {
        for (int i = 0; i < IOCount; i++) {
            if (IOSetting[InputNo].OutputGPIONo[i] == -1) break;
            if (GPIOStatusOn[IOSetting[InputNo].OutputGPIONo[i]] != true) {
                gpio_put(IOSetting[InputNo].OutputGPIONo[i], GPIO_IN);
                GPIOStatusOn[IOSetting[InputNo].OutputGPIONo[i]] = false;
            }
        }
        IOSetting[InputNo].CurrentPin = 0;
        IOSetting[InputNo].IntervalFrameCount = 0;
        // 出力確認はここのコメントを外す    
        // if (InputNo == 11) {
        //     gpio_put(LED_PIN, GPIO_IN);
        // }
    }
}

// コマンドボタンの実行
void InputCommand(int InputNo)
{
    for (int i = 0; i < IOCount; i++) {
        bool on = bit_get_u16(CommandSetBits[InputNo][buttonCommands[IOSetting[InputNo].CommandType].CurrentFrame], i);
        gpio_put(Output_Pin[i], on ? GPIO_OUT : GPIO_IN);
        GPIOStatusOn[i] = on;
    }
    buttonCommands[IOSetting[InputNo].CommandType].CurrentFrame++;

    // 最終フレームに到達したら
    if (buttonCommands[IOSetting[InputNo].CommandType].CurrentFrame == LastFrameCount[InputNo]) {
        buttonCommands[IOSetting[InputNo].CommandType].CurrentFrame = 0;
        if (IOSetting[InputNo].RepeatMode == 0) {
            buttonCommands[IOSetting[InputNo].CommandType].LastButtonOn = false; // 押しっぱなしで繰り返す
        }
    }

    for (int i = 0; i < IOCount; i++) {
        if (IOSetting[i].RapidType == 5) {
            if (i != InputNo) buttonCommands[IOSetting[i].CommandType].LastButtonOn = false;
            if (i != InputNo) buttonCommands[IOSetting[i].CommandType].CurrentFrame = 0;
        }
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

// ここからマクロ設定用
void StartSettingMode() {
    // USB未接続時は設定モードに入らない
    if (!UsbConnected) {
        return;
    }
    
    SettingMode = true;
    multicore_reset_core1();

    jumpMode = false;

    stdio_init_all();
    setup_default_uart();

    InitGPIO_Macro();

    Ssd1306_Init();

    SetCharPattern();
    canvas = Ssd1306_Get_Draw_Canvas();

    DrawMessage(0, "Start", false);
    DrawMessage(1, "Pico", false);
    DrawMessage(2, "Rapid", false);
    DrawMessage(3, "Setting", false);

    StartSetting();

    return;
}

// 入力・設定側初期化
void InitGPIO_Macro() {
    // 初期モードが違うため標準IOに設定
    gpio_set_function(26, GPIO_FUNC_SIO);
    gpio_set_function(27, GPIO_FUNC_SIO);
    gpio_set_function(28, GPIO_FUNC_SIO);

    // ディスプレイ用
    int f = i2c_init(i2c0, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    gpio_init_mask(maskIO_Macro);
    gpio_set_dir_masked(maskGPIO_Macro, maskIO_Macro);

    for (int i = 0; i < 18; i++) {
        gpio_set_drive_strength(Input_Pin_Macro[i], GPIO_DRIVE_STRENGTH_2MA);
        gpio_set_input_hysteresis_enabled(Input_Pin_Macro[i], true);
        gpio_pull_up(Input_Pin_Macro[i]);
        gpio_set_irq_enabled_with_callback(Input_Pin_Macro[i], 0x4u, true, callback_check);
    }

    // 標準GPIO
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

}

// 割り込み処理
void callback_check(uint gpio, uint32_t events) {
    if (events == 4u) {
        switch (gpio) {
        case ModeSW_Pin: // モードボタン
            ModeSelect();
            break;
        case EnterSW_Pin: // 決定ボタン
            EnterPush();
            break;
        case SettingSW_UL_Pin: // 設定ボタン(上段・左)
            Setting_SW_UL();
            break;
        case SettingSW_UR_Pin: // 設定ボタン(上段・右)
            Setting_SW_UR();
            break;
        case SettingSW_DL_Pin: // 設定ボタン(下段・左)
            Setting_SW_DL();
            break;
        case SettingSW_DR_Pin: // 設定ボタン(下段・右)
            Setting_SW_DR();
            break;
        default: // 入力ボタン(割り込みは設定モードの時のみ発生させる)
            Input_SW_Push(gpio);
            break;
        }
    }
}

void StartSetting() {
    // マクロ用変数の初期化
    for (int i = 0; i < IOCount; i++) {
        for (int j = 0; j < 150; j++) {
            MacroSettingBits[i][j] = 0;
            if (j == 0) bit_set_u16(&MacroSettingBits[i][j], 0, true); // 1フレーム目は有効にしておく
        }
    }

    // 設定の読込
    load_io_setting_from_flash(FLASH_TARGET_OFFSET_IO_Setting, g_read_io_data);
    SetIOData();

    uint32_t read_address = FLASH_TARGET_OFFSET_Macro_0;
    for (int i = 0; i < IOCount; i++) {
        for (int j = 0; j < 16; j++) {
            load_io_setting_from_flash(read_address, g_read_macro_data);
            for (int k = 0; k < 16; k++) {
                for (int l = 0; l < 16; l++) {
                    if (((j * 16) + k) < MaxMacroFrame) {
                        bit_set_u16(&MacroSettingBits[i][(j * 16) + k], l, g_read_macro_data[(k * 16) + l] != 0);
                    }
                }
            }
            read_address += FLASH_PAGE_SIZE;
        }
    }

    read_address = FLASH_TARGET_OFFSET_Macro_15;
    load_io_setting_from_flash(read_address, g_read_macro_repeat_data);
    for (int i = 0; i < IOCount; i++) {
        IOSetting[i].RepeatMode = g_read_macro_repeat_data[i];
    }

    ModeSW_flg = false;
    EnterSW_flg = false;
    Button_Flg = false;
    Check_FrameDelete_Flg = false;
    Check_FrameInsert_Flg = false;
    ChangeBoard_Flg = false;

    SelectMode = SelectMode_Normal;
    SelectRapid = SelectRapid_Normal;
    
    OnFrame = 1;
    OffFrame = 1;
    MacroFrame = 1;
    MacroFrame_Total = 1;
    SelectMacroNo = 0;

    Confirm_Mode = ConfirmMode_Nothing;
    Confirm_Flg = SelectYes;

    SetMacroMode = false;

    for (int i = 0; i < IOCount; i++) {
        DispPanel[i] = '0';
    }
    
    busy_wait_ms(1000);

    DrawMessage(0, "PushMode", false);
    DrawMessage(1, "Button", false);

    DrawControlPanel(DispPanel);

}

// 読み取った入出力データを展開する
void SetIOData() {
    int i, j, k, row_start;

    // 初期化
    for (i = 0; i < IOCount; i++) {
        k = 0;
        for (j = 0; j < 16; j++) {
            row_start = (i * 16) + j;
            switch (j) {
                case 0:
                    if (g_read_io_data[row_start] < 10) {
                        IOSetting[i].RapidType = g_read_io_data[row_start];
                        IOSetting[i].Reverse = false;
                    } else {
                        IOSetting[i].RapidType = g_read_io_data[row_start] - 10;
                        IOSetting[i].Reverse = true;
                    }
                    break;
                case 1:
                    IOSetting[i].OutputFrame = g_read_io_data[row_start]; // 出力フレーム数・連射設定
                    break;
                case 2:
                    IOSetting[i].IntervalFrame = g_read_io_data[row_start]; // インターバルフレーム数
                    break;
                case 3:
                    IOSetting[i].CommandType = g_read_io_data[row_start]; // コマンドNo
                    break;
                default:
                    if (j - 4 < IOCount) {
                        IOSetting[i].OutputNo[j - 4] = g_read_io_data[row_start];
                    }
                    break;
            }
            printf("%d", g_read_io_data[row_start]);
        }
        printf("\n");
    }
}

void ModeSelect() {
    char Message[8];

    if (ModeSW_flg == true) return;
    if (Confirm_Mode != ConfirmMode_Nothing) return;

    ModeSW_flg = true;
    SelectMode++;
    if (SelectMode > SelectMode_USB) SelectMode = SelectMode_Save;
    if (SelectMode == SelectMode_Normal) SelectRapid = SelectRapid_Normal;
    busy_wait_ms(300);
    ModeSW_IRQ_disable
    switch (SelectMode)
    {
    case SelectMode_Input: // 入力選択
        DrawMessage(0, "Select", false);
        DrawMessage(1, "Input", false);
        DrawControlPanel(DispPanel);
        Input_SW_Push(6); // 未選択を防ぐためAボタンを選択状態にする
        break;
    case SelectMode_Output: // 出力選択
        SetOutputMode();
        DrawMessage(0, "Select", false);
        DrawMessage(1, "Output", false);
        break;
    case SelectMode_Rapid: // 連射選択
        DrawMessage(0, "SetRapid", false);
        SetRapidMode();
        break;
    case SelectMode_Repeat: // マクロのリピート
        Confirm_Mode = ConfirmMode_Execute;
        Confirm_Flg = (IOSetting_Current.RepeatMode == 0) ? SelectYes : SelectNo;
        DispConfirm("Repeat?", Confirm_Flg); // 保存
        break;
    case SelectMode_Custom: // カスタム連射設定
        if (SelectRapid == SelectRapid_Custom) {
            OnFrame = IOSetting_Current.OutputFrame;
            OffFrame = IOSetting_Current.IntervalFrame;
            sprintf(Message, "Cst On:%d", OnFrame);
            DrawMessage(0, Message, false);
            sprintf(Message, "   Off:%d", OffFrame);
            DrawMessage(1, Message, false);
            break;
        }
        SelectMode = SelectMode_Save;
    case SelectMode_Save:
        Confirm_Mode = ConfirmMode_Execute;
        Confirm_Flg = SelectYes;
        DispConfirm("Save?", Confirm_Flg); // 保存
        break;
    default:
        SelectMode--;
        break;
    }

    ModeSW_flg = false;
    ModeSW_IRQ_enable
}

void EnterPush() {
    char Message[8];

    if (EnterSW_flg == true) return;
    EnterSW_flg = true;
    busy_wait_ms(300);
    EnterSW_IRQ_disable

    // 決定ボタンとして使用
    if ((Confirm_Mode == ConfirmMode_Execute) || (Confirm_Mode == ConfirmMode_Cancel)) {
        switch (SelectMode) {
        case SelectMode_Normal: // 初期化
            Confirm_Mode = ConfirmMode_Nothing;
            if (ChangeBoard_Flg == true) {
                // 出力ボード切り替え
                SetBoardSetting();

                DrawMessage(0, "Change", false);
                DrawMessage(1, "Finish", false);
                busy_wait_ms(500);
                DrawMessage(0, "Change", true);
                DrawMessage(1, "Finish", true);
                busy_wait_ms(500);
                StartSetting();
            } else if (Confirm_Flg == SelectYes) {
                // 初期化
                InitSetting();

                DrawMessage(0, "Init", false);
                DrawMessage(1, "Finish", false);
                busy_wait_ms(500);
                DrawMessage(0, "Init", true);
                DrawMessage(1, "Finish", true);
                busy_wait_ms(500);
                StartSetting();
            } else {
                SelectMode = SelectMode_Normal;
                ChangeBoard_Flg = false;
                DrawMessage(0, "PushMode", false);
                DrawMessage(1, "Button", false);
            }
            break;            
        case SelectMode_Macro: // マクロ設定時のフレーム削除・挿入
            Confirm_Mode = ConfirmMode_Nothing;
            if (Confirm_Flg == SelectYes) {
                if (Check_FrameDelete_Flg == true) {
                    // フレーム削除
                    DeleteFrame();
                } else {
                    // フレーム挿入
                    InsertFrame();
                }
                DispMacroSetting();
            }
            Check_FrameDelete_Flg = false;
            Check_FrameInsert_Flg = false;
            Confirm_Mode = ConfirmMode_Nothing;
            sprintf(Message, "Macro  %s", jumpMode ? "j" : "");
            DrawMessage(0, Message , false);
            sprintf(Message, "%d/%d", MacroFrame, MacroFrame_Total);
            DrawMessage(1, Message, false);
            break;
        case SelectMode_Custom:
            if (Confirm_Mode == ConfirmMode_Execute) {
                if (Confirm_Flg == SelectYes) {
                    SaveSetting();
                } else {
                    Confirm_Mode = ConfirmMode_Cancel;
                    Confirm_Flg = SelectNo;
                    DispConfirm("Cancel?", Confirm_Flg);
                }
            } else if (Confirm_Mode == ConfirmMode_Cancel) {
                if (Confirm_Flg == SelectYes) {
                    // カスタム連射設定表示
                    Confirm_Mode = ConfirmMode_Nothing;
                    SelectMode = SelectMode_Custom;
                    sprintf(Message, "Cst On:%d", OnFrame);
                    DrawMessage(0, Message, false);
                    sprintf(Message, "   Off:%d", OffFrame);
                    DrawMessage(1, Message, false);
                } else {
                    // 初期画面に戻る
                    StartSetting();
                }
            }
            break;
        case SelectMode_Repeat: // マクロのリピート
            IOSetting_Current.RepeatMode = (Confirm_Flg == SelectYes) ? 0 : 1;
            SelectMode = SelectMode_Save;
            Confirm_Mode = ConfirmMode_Execute;
            Confirm_Flg = SelectYes;
            DispConfirm("Save?", Confirm_Flg); // 保存
            break;
        case SelectMode_Save:
            if (Confirm_Mode == ConfirmMode_Execute) {
                if (Confirm_Flg == SelectYes) {
                    SaveSetting();
                } else {
                    Confirm_Mode = ConfirmMode_Cancel;
                    Confirm_Flg = SelectNo;
                    DispConfirm("Cancel?", Confirm_Flg);
                }
            } else if (Confirm_Mode == ConfirmMode_Cancel) {
                if (Confirm_Flg == SelectYes) {
                    // 初期画面に戻る
                    StartSetting();
                } else {
                    // 連射設定表示
                    Confirm_Mode = ConfirmMode_Nothing;
                    SelectMode = SelectMode_Rapid;
                    DrawMessage(0, "SetRapid", false);
                    DispRapidMessage();
                }
            }
            break;
        default: // 現在の設定を保存
            break;
        }
    } else {
        switch (SelectMode)
        {
        case SelectMode_Input:
            IOSetting_Current.Reverse = !IOSetting_Current.Reverse;
            sprintf(Message, "Input  %s", IOSetting_Current.Reverse ? "R" : "");
            DrawMessage(1, Message , false);
            break;
        case SelectMode_Macro:
            jumpMode = !jumpMode;
            sprintf(Message, "Macro  %s", jumpMode ? "j" : "");
            DrawMessage(0, Message , false);
            sprintf(Message, "%d/%d", MacroFrame, MacroFrame_Total);
            DrawMessage(1, Message, false);
            break;
        default:
            break;
        }
    }
    EnterSW_flg = false;
    EnterSW_IRQ_enable
}

void DispRapidMessage() {
    switch (SelectRapid)
    {
    case SelectRapid_Normal:
        DrawMessage(1, "Normal", false);
        break;
    case SelectRapid_Sync30:
        DrawMessage(1, "Sync30", false);
        break;
    case SelectRapid_Sync30R:
        DrawMessage(1, "Sync30R", false);
        break;
    case SelectRapid_20:
        DrawMessage(1, "20", false);
        break;
    case SelectRapid_15:
        DrawMessage(1, "15", false);
        break;
    case SelectRapid_12:
        DrawMessage(1, "12", false);
        break;
    case SelectRapid_10:
        DrawMessage(1, "10", false);
        break;
    case SelectRapid_75:
        DrawMessage(1, "7.5", false);
        break;
    case SelectRapid_Custom:
        DrawMessage(1, "Custom", false);
        break;
    case SelectRapid_Sync15:
        DrawMessage(1, "Sync15", false);
        break;
    case SelectRapid_Sync15R:
        DrawMessage(1, "Sync15R", false);
        break;
    default:
        break;
    }
}

void Setting_SW_UL() {
    char Message[8];

    busy_wait_ms(300);
    SettingSW_UL_IRQ_disable
    switch (SelectMode)
    {
    case SelectMode_Normal:
        // 初期化
        if (Confirm_Mode == ConfirmMode_Nothing) {
            Confirm_Mode = ConfirmMode_Execute;
            Confirm_Flg = SelectNo;
            DispConfirm("Init?", Confirm_Flg);
        }
        break;
    case SelectMode_Input:
        SelectMode = SelectMode_Macro;
        SetMacroMode = true;
        IOSetting_Current.RapidType = 0;
        IOSetting_Current.CommandType = SelectInputNo;
        IOSetting_Current.Reverse = false;
        MacroFrame = 1; // 最初は1フレーム目を表示
        MacroFrame_Total = CountMacroEnableFrame(); // 現在のマクロ設定の最大フレーム
        sprintf(Message, "Macro  %s", jumpMode ? "j" : "");
        DrawMessage(0, Message , false);
        sprintf(Message, "%d/%d", MacroFrame, MacroFrame_Total);
        DrawMessage(1, Message, false);
        DispMacroSetting();
        break;
    case SelectMode_Custom:
        if ((OnFrame > 1) && (Confirm_Mode == ConfirmMode_Nothing)) {
            OnFrame--;
            IOSetting_Current.OutputFrame = OnFrame;
            sprintf(Message, "Cst On:%d", OnFrame);
            DrawMessage(0, Message, false);
        }
        break;    
    case SelectMode_Macro:
        // フレームの削除
        if (Check_FrameDelete_Flg == false) {
            Check_FrameDelete_Flg = true;
            Confirm_Mode = ConfirmMode_Execute;
            Confirm_Flg = SelectYes;
            DispConfirm("Delete?", Confirm_Flg);
        }
    default:
        break;
    }
    SettingSW_UL_IRQ_enable
}

void Setting_SW_UR() {
    char Message[8];

    busy_wait_ms(300);
    SettingSW_UR_IRQ_disable
    switch (SelectMode)
    {
    case SelectMode_Normal:
        // ボード切り替え
        if (Confirm_Mode == ConfirmMode_Nothing) {
            ChangeBoard_Flg = true;
            Confirm_Mode = ConfirmMode_Execute;
            Confirm_Flg = SelectNo;
            DispChange();
        }
        break;
    case SelectMode_Input:
        SelectMode = SelectMode_Macro;
        SetMacroMode = true;
        IOSetting_Current.RapidType = 0;
        IOSetting_Current.CommandType = SelectInputNo;
        IOSetting_Current.Reverse = false;
        MacroFrame = 1; // 最初は1フレーム目を表示
        MacroFrame_Total = CountMacroEnableFrame(); // 現在のマクロ設定の最大フレーム
        sprintf(Message, "Macro  %s", jumpMode ? "j" : "");
        DrawMessage(0, Message , false);
        sprintf(Message, "%d/%d", MacroFrame, MacroFrame_Total);
        DrawMessage(1, Message, false);
        DispMacroSetting();
        break;
    case SelectMode_Custom:
        if ((OnFrame < 9) && (Confirm_Mode == ConfirmMode_Nothing)) {
            OnFrame++;
            IOSetting_Current.OutputFrame = OnFrame;
            sprintf(Message, "Cst On:%d", OnFrame);
            DrawMessage(0, Message, false);
        }
        break;
    case SelectMode_Macro:
        // フレームの挿入
        if (Check_FrameInsert_Flg == false) {
            Check_FrameInsert_Flg = true;
            Confirm_Mode = ConfirmMode_Execute;
            Confirm_Flg = SelectYes;
            DispConfirm("Insert?", Confirm_Flg);
        }
        break;            
    default:
        break;
    }
    SettingSW_UR_IRQ_enable
}

void Setting_SW_DL() {
    char Message[8];

    busy_wait_ms(300);
    SettingSW_DL_IRQ_disable
    switch (SelectMode)
    {
    case SelectMode_Normal:
        if (ChangeBoard_Flg == true) {
            boardMode = 0;
            DispChange();
        }
        else if (Confirm_Mode == ConfirmMode_Execute) {
            Confirm_Flg = SelectYes;
            DispConfirm("Init?", Confirm_Flg);
        }
        break;
    case SelectMode_Rapid:
        SelectRapid--;
        if (SelectRapid < SelectRapid_Normal) SelectRapid = SelectRapid_Sync15R;
        DispRapidMessage();
        break;
    case SelectMode_Custom:
        if (Confirm_Mode == ConfirmMode_Execute) {
            Confirm_Flg = SelectYes;
            DispConfirm("Save?", Confirm_Flg);
        } else if (Confirm_Mode == ConfirmMode_Cancel) {
            Confirm_Flg = SelectYes;
            DispConfirm("Cancel?", Confirm_Flg);
        } else if (OffFrame > 1) {
            OffFrame--;
            IOSetting_Current.IntervalFrame = OffFrame;
            sprintf(Message, "   Off:%d", OffFrame);
            DrawMessage(1, Message, false);
        }
        break;
    case SelectMode_Repeat:
        // マクロの繰り返し
        Confirm_Flg = SelectYes;
        Confirm_Mode = ConfirmMode_Execute;
        DispConfirm("Repeat?", Confirm_Flg);
        break;
    case SelectMode_Save:
        if (Confirm_Mode == ConfirmMode_Execute) {
            Confirm_Flg = SelectYes;
            DispConfirm("Save?", Confirm_Flg);
        } else if (Confirm_Mode == ConfirmMode_Cancel) {
            Confirm_Flg = SelectYes;
            DispConfirm("Cancel?", Confirm_Flg);
        }
        break;
    case SelectMode_Macro:
        if ((Check_FrameDelete_Flg == true) || (Check_FrameInsert_Flg == true)) {
            Confirm_Flg = SelectYes;
            DispConfirm((Check_FrameDelete_Flg) ? "Delete?" : "Insert?", Confirm_Flg);
        } else if (MacroFrame >= 1) {
            if (jumpMode == true) {
                if (MacroFrame == 1) {
                    MacroFrame = MacroFrame_Total; // 先頭フレームだったら最終フレームに移動
                } else if (MacroFrame <= 10) {
                    MacroFrame = 1; // 10フレーム以内だったら先頭フレームに移動
                } else {
                    MacroFrame = MacroFrame - 10; // 10フレーム以上だったら-10フレーム
                }
            } else {
                if (MacroFrame > 1) MacroFrame--;               
            }

            sprintf(Message, "%d/%d", MacroFrame, MacroFrame_Total);
            DrawMessage(1, Message, false);
            DispMacroSetting();
        }
        break;
    default:
        break;
    }
    SettingSW_DL_IRQ_enable
}

void Setting_SW_DR() {
    char Message[8];

    busy_wait_ms(300);
    SettingSW_DR_IRQ_disable
    switch (SelectMode)
    {
    case SelectMode_Normal:
        if (ChangeBoard_Flg == true) {
            boardMode = 1;
            DispChange();
        }
        else if (Confirm_Mode == ConfirmMode_Execute) {
            Confirm_Flg = SelectYes;
            DispConfirm("Init?", Confirm_Flg);
        }
        break;
    case SelectMode_Rapid:
        SelectRapid++;
        if (SelectRapid > SelectRapid_Sync15R) SelectRapid = SelectRapid_Normal;
        DispRapidMessage();
        break;
    case SelectMode_Custom:
        if (Confirm_Mode == ConfirmMode_Execute) {
            Confirm_Flg = SelectNo;
            DispConfirm("Save?", Confirm_Flg);
        } else if (Confirm_Mode == ConfirmMode_Cancel) {
            Confirm_Flg = SelectNo;
            DispConfirm("Cancel?", Confirm_Flg);
        } else if (OffFrame < 9) {
            OffFrame++;
            IOSetting_Current.IntervalFrame = OffFrame;
            sprintf(Message, "   Off:%d", OffFrame);
            DrawMessage(1, Message, false);
        }
        break;
    case SelectMode_Repeat:
        // マクロの繰り返し
        Confirm_Flg = SelectNo;
        Confirm_Mode = ConfirmMode_Execute;
        DispConfirm("Repeat?", Confirm_Flg);
        break;
    case SelectMode_Save:
        if (Confirm_Mode == ConfirmMode_Execute) {
            Confirm_Flg = SelectNo;
            DispConfirm("Save?", Confirm_Flg);
        } else if (Confirm_Mode == ConfirmMode_Cancel) {
            Confirm_Flg = SelectNo;
            DispConfirm("Cancel?", Confirm_Flg);
        }
        break;
    case SelectMode_Macro:
        if ((Check_FrameDelete_Flg == true) || (Check_FrameInsert_Flg == true)) {
            Confirm_Flg = SelectNo;
            DispConfirm((Check_FrameDelete_Flg) ? "Delete?" : "Insert?", Confirm_Flg);
        } else if (MacroFrame < MaxMacroFrame) {
            if (jumpMode == true) {                
                if (MacroFrame == MacroFrame_Total) {
                    MacroFrame = 1; // 最終フレームだったら先頭フレームに移動
                } else if ((MacroFrame + 10) > MacroFrame_Total) {
                    MacroFrame = MacroFrame_Total; // 10フレーム以内だったら最終フレームに移動
                } else {
                    MacroFrame = MacroFrame + 10; // フレーム数を+10
                }
            } else {
                MacroFrame++;
                if (MacroFrame > MacroFrame_Total) MacroFrame_Total = MacroFrame;
            }
            
            sprintf(Message, "%d/%d", MacroFrame, MacroFrame_Total);
            DrawMessage(1, Message, false);
            DispMacroSetting();
        }
        break;            
    default:
        break;
    }
    SettingSW_DR_IRQ_enable
}

void Input_SW_Push(int gpio) {
    busy_wait_ms(300);
    gpio_set_irq_enabled(gpio, 0x4u, false);
    switch (SelectMode) {
    case SelectMode_Input:
        SelectInputNo = GPIO_InputNo_Macro[gpio - 2];
        for (int i = 0; i < IOCount; i++) {
            DispPanel[i] = (SelectInputNo == i) ? '1' : '0';
        }
        IOSetting_Current = IOSetting[SelectInputNo];
        DrawControlPanel(DispPanel);

        if (IOSetting_Current.RapidType == 5) {
            DrawMessage(1, "Input  M", false);
        } else if (IOSetting_Current.Reverse == true) {
            DrawMessage(1, "Input  R", false);
        } else {
            DrawMessage(1, "Input", false);
        }

        break;
    case SelectMode_Output:
        for (int i = 0; i < IOCount; i++) {
            if (i == (GPIO_InputNo_Macro[gpio - 2])) {
                DispPanel[i] = (DispPanel[i] == '0') ? '1' : '0'; 
                IOSetting_Current.OutputNo[i] = (DispPanel[i] == '0') ? 0 : 1;
            }
        }
        DrawControlPanel(DispPanel);
        break;
    case SelectMode_Macro:
        for (int i = 0; i < IOCount; i++) {
            if (i == (GPIO_InputNo_Macro[gpio - 2])) {
                DispPanel[i] = (DispPanel[i] == '0') ? '1' : '0';
                bit_set_u16(&MacroSettingBits[SelectInputNo][MacroFrame], i + 1, (DispPanel[i] != '0'));
            }
        }
        DrawControlPanel(DispPanel);
        break;
    default:
        break;
    }
    gpio_set_irq_enabled(gpio, 0x4u, true);            
}

// 入出力設定初期化
void InitSetting() {
    printf("init_start\n");
    int i, j, row_start;
    int8_t save_data[256];
    for (i = 0; i < 16; i++) {
        for (j = 0; j < 16; j++) {
            row_start = i * 16;
            switch (j) {
                case 0:
                    save_data[row_start] = 1;
                    continue;
                case 1:
                case 2:
                case 3:
                    save_data[row_start + j] = 0;
                    continue;
                default:
                    save_data[row_start + j] = (j == (i + 4)) ? 1 : 0;
                    continue;
            }
        }
    }

    save_io_setting_to_flash(FLASH_TARGET_OFFSET_IO_Setting, save_data, FLASH_PAGE_SIZE);

    uint8_t save_macrodata[4096];
    uint32_t save_address = FLASH_TARGET_OFFSET_Macro_0;
    for (int i = 0; i < 12; i++) {
        for (int j = 0; j < 256; j++) {
            for (int k = 0; k < 16; k++) {
                save_macrodata[(j * 16) + k] = 0;
            }
        }
        save_io_setting_to_flash(save_address, save_macrodata, FLASH_SECTOR_SIZE);
        save_address += FLASH_SECTOR_SIZE;
    }

    int8_t save_repeat_data[256];
    save_address = FLASH_TARGET_OFFSET_Macro_15;
    for (i = 0; i < 256; i++) {
        save_repeat_data[i] = 0;
    }
    save_io_setting_to_flash(save_address, save_repeat_data, FLASH_PAGE_SIZE);
}

// 使用ボード設定
void SetBoardSetting() {
    int i;
    int8_t save_data[256];
    save_data[0] = boardMode;
    for (i = 1; i < 256; i++) {
        save_data[i] = 0;
    }
    save_io_setting_to_flash(FLASH_TARGET_OFFSET_IO_Board, save_data, FLASH_PAGE_SIZE);
}

void SetOutputMode() {
    IOSetting_Current.CommandType = 0;

    // 現在の出力を表示する
    for (int i = 0; i < 12; i++) {
        DispPanel[i] = (IOSetting_Current.OutputNo[i] == 0) ? '0' : '1';
    }
    DrawControlPanel(DispPanel);
}

void SetRapidMode() {
    // 現在の出力を表示する
    switch (IOSetting_Current.RapidType)
    {
    case 1:
        SelectRapid = SelectRapid_Normal;
        break;
    case 2:
        SelectRapid = SelectRapid_Sync30;
        break;
    case 3:
        SelectRapid = SelectRapid_Sync30R;
        break;
    case 4:
        printf("interval:%d\n", IOSetting_Current.IntervalFrame);
        if (IOSetting_Current.OutputFrame > 1) {
            SelectRapid = SelectRapid_Custom;
        } else {
            switch (IOSetting_Current.IntervalFrame)
            {
            case 1: // 1:1の30連指定は任意
                SelectRapid = SelectRapid_Custom;
                break;
            case 2: // 20連(1:2)
                SelectRapid = SelectRapid_20;
                break;
            case 3: // 15連(1:3)
                SelectRapid = SelectRapid_15;
                break;
            case 4: // 12連(1:4)
                SelectRapid = SelectRapid_12;
                break;
            case 5: // 10連(1:5)
                SelectRapid = SelectRapid_10;
                break;
            case 7: // 7.5連(1:7)
                SelectRapid = SelectRapid_75;
                break;
            default:
                SelectRapid = SelectRapid_Custom;
                break;
            }
        }
        break;
    case 6:
        SelectRapid = SelectRapid_Sync15;
        break;
    case 7:
        SelectRapid = SelectRapid_Sync15R;
        break;
    default:
        break;
    }
    DispRapidMessage();
}

void SaveSetting() {
    switch (SelectRapid)
    {
    case 0: // 連なし
        IOSetting[SelectInputNo].RapidType     = 1;
        IOSetting[SelectInputNo].OutputFrame   = 0;
        IOSetting[SelectInputNo].IntervalFrame = 0;
        break;
    case 1: // 30連(表)
        IOSetting[SelectInputNo].RapidType     = 2;
        IOSetting[SelectInputNo].OutputFrame   = 0;
        IOSetting[SelectInputNo].IntervalFrame = 0;
        break;
    case 2: // 30連(裏)
        IOSetting[SelectInputNo].RapidType     = 3;
        IOSetting[SelectInputNo].OutputFrame   = 0;
        IOSetting[SelectInputNo].IntervalFrame = 0;
        break;
    case 3: // 20連(1on/2off)
        IOSetting[SelectInputNo].RapidType     = 4;
        IOSetting[SelectInputNo].OutputFrame   = 1;
        IOSetting[SelectInputNo].IntervalFrame = 2;
        break;
    case 4: // 15連(1on/3off)
        IOSetting[SelectInputNo].RapidType     = 4;
        IOSetting[SelectInputNo].OutputFrame   = 1;
        IOSetting[SelectInputNo].IntervalFrame = 3;
        break;
    case 5: // 12連(1on/4off)
        IOSetting[SelectInputNo].RapidType     = 4;
        IOSetting[SelectInputNo].OutputFrame   = 1;
        IOSetting[SelectInputNo].IntervalFrame = 4;
        break;
    case 6: // 10連(1on/5off)
        IOSetting[SelectInputNo].RapidType     = 4;
        IOSetting[SelectInputNo].OutputFrame   = 1;
        IOSetting[SelectInputNo].IntervalFrame = 5;
        break;
    case 7: // 7.5連(1on/7off)
        IOSetting[SelectInputNo].RapidType     = 4;
        IOSetting[SelectInputNo].OutputFrame   = 1;
        IOSetting[SelectInputNo].IntervalFrame = 7;
        break;
    case 8: // 任意
        IOSetting[SelectInputNo].RapidType     = 4;
        IOSetting[SelectInputNo].OutputFrame   = IOSetting_Current.OutputFrame;
        IOSetting[SelectInputNo].IntervalFrame = IOSetting_Current.IntervalFrame;
        break;
    case 9: // 15連(表)
        IOSetting[SelectInputNo].RapidType     = 6;
        IOSetting[SelectInputNo].OutputFrame   = 0;
        IOSetting[SelectInputNo].IntervalFrame = 0;
        break;
    case 10: // 15連(裏)
        IOSetting[SelectInputNo].RapidType     = 7;
        IOSetting[SelectInputNo].OutputFrame   = 0;
        IOSetting[SelectInputNo].IntervalFrame = 0;
        break;
    default:
        break;
    }
    IOSetting[SelectInputNo].CommandType = IOSetting_Current.CommandType;
    if (SetMacroMode) IOSetting[SelectInputNo].RapidType = 5; // マクロ有効
    // RAPID_TYPE=5 の場合は CMD_TYPE を入力番号へ強制
    if ((IOSetting[SelectInputNo].RapidType % 10) == 5) {
        IOSetting[SelectInputNo].CommandType = SelectInputNo;
    }
    IOSetting[SelectInputNo].RepeatMode = IOSetting_Current.RepeatMode;

    if (IOSetting_Current.Reverse == true) IOSetting[SelectInputNo].RapidType = IOSetting[SelectInputNo].RapidType + 10;

    int i, j, row_start;
    for (i = 0; i < 12; i++) {
        IOSetting[SelectInputNo].OutputNo[i] = IOSetting_Current.OutputNo[i];
    }

    for (i = 0; i < 128; i++) {
        g_save_io_data[i] = 0;
    }
    for (i = 0; i < 12; i++) {
        for (j = 0; j < 16; j++) {
            row_start = i * 16;
            switch (j) {
                case 0:
                    g_save_io_data[row_start] = IOSetting[i].RapidType;
                    break;
                case 1:
                    g_save_io_data[row_start + j] = IOSetting[i].OutputFrame;
                    break;
                case 2:
                    g_save_io_data[row_start + j] = IOSetting[i].IntervalFrame;
                    break;
                case 3:
                    // RapidType=5の場合はCMD_TYPEを入力番号に固定
                    g_save_io_data[row_start + j] = ((IOSetting[i].RapidType % 10) == 5) ? i : IOSetting[i].CommandType;
                    break;
                default:
                    g_save_io_data[row_start + j] = (IOSetting[i].OutputNo[j - 4]) ? 1 : 0;
                    break;
            }
            printf("%d", g_save_io_data[row_start + j]);
        }
        printf("\n");
    }
    save_io_setting_to_flash(FLASH_TARGET_OFFSET_IO_Setting, g_save_io_data, FLASH_PAGE_SIZE);

    SaveMacro();

    DrawMessage(0, "Save", false);
    DrawMessage(1, "Finish", false);
    busy_wait_ms(500);
    DrawMessage(0, "Save", true);
    DrawMessage(1, "Finish", true);
    busy_wait_ms(500);

    StartSetting();

}

void SaveMacro() {
    uint8_t save_macrodata[4096];
    uint32_t save_address;

    switch (IOSetting_Current.CommandType) {
        case 0:
            save_address = FLASH_TARGET_OFFSET_Macro_0;
            break;
        case 1:
            save_address = FLASH_TARGET_OFFSET_Macro_1;
            break;
        case 2:
            save_address = FLASH_TARGET_OFFSET_Macro_2;
            break;
        case 3:
            save_address = FLASH_TARGET_OFFSET_Macro_3;
            break;
        case 4:
            save_address = FLASH_TARGET_OFFSET_Macro_4;
            break;
        case 5:
            save_address = FLASH_TARGET_OFFSET_Macro_5;
            break;
        case 6:
            save_address = FLASH_TARGET_OFFSET_Macro_6;
            break;
        case 7:
            save_address = FLASH_TARGET_OFFSET_Macro_7;
            break;
        case 8:
            save_address = FLASH_TARGET_OFFSET_Macro_8;
            break;
        case 9:
            save_address = FLASH_TARGET_OFFSET_Macro_9;
            break;
        case 10:
            save_address = FLASH_TARGET_OFFSET_Macro_10;
            break;
        case 11:
            save_address = FLASH_TARGET_OFFSET_Macro_11;
            break;
        default:
            break;
    }

    for (int i = 0; i < 256; i++) {
        for (int j = 0; j < 16; j++) {
            if (j == 0) {
                save_macrodata[(i * 16) + j] = (i <= MacroFrame_Total) ? 1 : 0;
            } else {
                save_macrodata[(i * 16) + j] = (i >= MaxMacroFrame) ? 0 : (bit_get_u16(MacroSettingBits[IOSetting_Current.CommandType][i], j) ? 1 : 0);
            }
            printf("%d", save_macrodata[(i * 16) + j]);
        }
        printf("\n");
    }
    save_io_setting_to_flash(save_address, save_macrodata, FLASH_SECTOR_SIZE);

    int8_t save_repeat_data[256];
    save_address = FLASH_TARGET_OFFSET_Macro_15;
    for (int i = 0; i < 255; i++) {
        if (i < 12) {
            printf("%d", IOSetting[i].RepeatMode);
            save_repeat_data[i] = IOSetting[i].RepeatMode;
        } else {
            save_repeat_data[i] = 0;
        }
    }
    save_io_setting_to_flash(save_address, save_repeat_data, FLASH_PAGE_SIZE);
}

// 確認メッセージ
void DispConfirm(char *Message, int SelectSide) {
    int MessageCount = strlen(Message);
    DispPatternDef dispCharPattern;

    if (MessageCount > 8) MessageCount = 8;

    for (int row_pos = 0; row_pos < 2; row_pos++) {
        for (int col_pos = 0; col_pos < 8; col_pos++) {
            for (int i = 0; i < col_byte; i++) {
                canvas[(row_byte * row_pos) + (col_byte * col_pos) + i] = 0x00;
                canvas[(row_byte * row_pos) + (col_byte * col_pos) + row_byte_half + i] = 0x00;
            }
        }
    }

    // 確認メッセージ(1行目)
    for (int col_pos = 0; col_pos < MessageCount; col_pos++) {
        dispCharPattern = getcharPattern(Message[col_pos]);
        for (int i = 0; i < col_byte; i++) {

            canvas[(col_byte * col_pos) + i] = dispCharPattern.pattern_head[i];
            canvas[(col_byte * col_pos) + row_byte_half + i] = dispCharPattern.pattern_foot[i];
        }
    }

    char *SelectStr = "Yes";
    for (int col_pos = 0; col_pos < 3; col_pos++) {
        dispCharPattern = getcharPattern(SelectStr[col_pos]);
        for (int i = 0; i < col_byte; i++) {

            if (SelectSide == SelectYes) {
                dispCharPattern.pattern_head[i] = 0xFF - dispCharPattern.pattern_head[i];
                dispCharPattern.pattern_foot[i] = 0xFF - dispCharPattern.pattern_foot[i]; 
            }

            canvas[(row_byte) + (col_byte * col_pos) + i] = dispCharPattern.pattern_head[i];
            canvas[(row_byte) + (col_byte * col_pos) + row_byte_half + i] = dispCharPattern.pattern_foot[i];
        }
    }

    SelectStr = "No";
    for (int col_pos = 4; col_pos < 6; col_pos++) {
        dispCharPattern = getcharPattern(SelectStr[col_pos - 4]);
        for (int i = 0; i < col_byte; i++) {

            if (SelectSide == SelectNo) {
                dispCharPattern.pattern_head[i] = 0xFF - dispCharPattern.pattern_head[i];
                dispCharPattern.pattern_foot[i] = 0xFF - dispCharPattern.pattern_foot[i]; 
            }

            canvas[(row_byte) + (col_byte * col_pos) + i] = dispCharPattern.pattern_head[i];
            canvas[(row_byte) + (col_byte * col_pos) + row_byte_half + i] = dispCharPattern.pattern_foot[i];
        }
    }

	Ssd1306_Update_Frame();
}

// 切り替え確認メッセージ
void DispChange() {
    char *Message = "Select";
    int MessageCount = strlen(Message);
    DispPatternDef dispCharPattern;

    if (MessageCount > 8) MessageCount = 8;

    for (int row_pos = 0; row_pos < 2; row_pos++) {
        for (int col_pos = 0; col_pos < 8; col_pos++) {
            for (int i = 0; i < col_byte; i++) {
                canvas[(row_byte * row_pos) + (col_byte * col_pos) + i] = 0x00;
                canvas[(row_byte * row_pos) + (col_byte * col_pos) + row_byte_half + i] = 0x00;
            }
        }
    }

    // 確認メッセージ(1行目)
    for (int col_pos = 0; col_pos < MessageCount; col_pos++) {
        dispCharPattern = getcharPattern(Message[col_pos]);
        for (int i = 0; i < col_byte; i++) {

            canvas[(col_byte * col_pos) + i] = dispCharPattern.pattern_head[i];
            canvas[(col_byte * col_pos) + row_byte_half + i] = dispCharPattern.pattern_foot[i];
        }
    }

    char *SelectStr = "Org";
    for (int col_pos = 0; col_pos < 3; col_pos++) {
        dispCharPattern = getcharPattern(SelectStr[col_pos]);
        for (int i = 0; i < col_byte; i++) {

            if (boardMode == 0) {
                dispCharPattern.pattern_head[i] = 0xFF - dispCharPattern.pattern_head[i];
                dispCharPattern.pattern_foot[i] = 0xFF - dispCharPattern.pattern_foot[i]; 
            }

            canvas[(row_byte) + (col_byte * col_pos) + i] = dispCharPattern.pattern_head[i];
            canvas[(row_byte) + (col_byte * col_pos) + row_byte_half + i] = dispCharPattern.pattern_foot[i];
        }
    }

    SelectStr = "MAXX";
    for (int col_pos = 4; col_pos < 8; col_pos++) {
        dispCharPattern = getcharPattern(SelectStr[col_pos - 4]);
        for (int i = 0; i < col_byte; i++) {

            if (boardMode == 1) {
                dispCharPattern.pattern_head[i] = 0xFF - dispCharPattern.pattern_head[i];
                dispCharPattern.pattern_foot[i] = 0xFF - dispCharPattern.pattern_foot[i]; 
            }

            canvas[(row_byte) + (col_byte * col_pos) + i] = dispCharPattern.pattern_head[i];
            canvas[(row_byte) + (col_byte * col_pos) + row_byte_half + i] = dispCharPattern.pattern_foot[i];
        }
    }

	Ssd1306_Update_Frame();
}

// マクロで有効になっている最終フレームを取得する
int CountMacroEnableFrame()
{
    for (int i = 1; i < MaxMacroFrame; i++) {
        if (!bit_get_u16(MacroSettingBits[SelectInputNo][i], 0)) {
            if (i == 1) return 1;
            return (i - 1);
        }
    }
}

// 現在のマクロ設定を画面表示する
void DispMacroSetting()
{
    for (int i = 0; i < 12; i++) {
        DispPanel[i] = (bit_get_u16(MacroSettingBits[SelectInputNo][MacroFrame], i + 1) ? '1' : '0');
    }
    DrawControlPanel(DispPanel);
}

// フレームの削除
void DeleteFrame() {
    if (MacroFrame_Total > 1) {
        MacroFrame_Total--;
    }
    for (int i = MacroFrame; i < MaxMacroFrame + 1; i++) {
        MacroSettingBits[SelectInputNo][i] = MacroSettingBits[SelectInputNo][i + 1];
    }
    if (MacroFrame > MacroFrame_Total) MacroFrame = MacroFrame_Total;
}

// フレームの挿入
void InsertFrame() {
    if (MacroFrame_Total > 1) {
        MacroFrame_Total++;
    }
    for (int i = MaxMacroFrame; i > MacroFrame - 1; i--) {
        MacroSettingBits[SelectInputNo][i] = MacroSettingBits[SelectInputNo][i - 1];
    }
    MacroSettingBits[SelectInputNo][MacroFrame] = 0;
    if (MacroFrame_Total > MaxMacroFrame) MacroFrame = MaxMacroFrame;
}

bool Ssd1306_Init()
{
    bool status = true;
    unsigned char cmd_buf[BUFSIZ];

    // 一度画面をオフ
    cmd_buf[0] = SSD1306_CTRL_BYTE_CMD_SINGLE;
    unsigned char *p = &cmd_buf[1];
    for(unsigned long i=0; i < NUM_OF_SSD1306_CONFIG; i++)
    {
        *(p++) = SSD1306_Init_Config[i];
    }
    *(p++) = SSD1306_CONFIG_DISPLAY_OFF;
    status = Ssd1306_Write(cmd_buf, NUM_OF_SSD1306_CONFIG + 2);
    busy_wait_ms(100);

    /* 動作設定 */
    cmd_buf[0] = SSD1306_CTRL_BYTE_CMD_SINGLE;
    p = &cmd_buf[1];
    for(unsigned long i=0; i < NUM_OF_SSD1306_CONFIG; i++)
    {
        *(p++) = SSD1306_Init_Config[i];
    }
    *(p++) = SSD1306_CONFIG_DISPLAY_ON;
    Ssd1306_Write(cmd_buf, NUM_OF_SSD1306_CONFIG + 2);
    busy_wait_ms(100);

    /* 画面初期化(黒塗り) */
    Draw_Canvas_Payload[0] = SSD1306_CTRL_BYTE_DATA_SINGLE;
    Ssd1306_Write(Draw_Canvas_Payload, SSD1306_DRAW_CANVAS_PAYLOAD_SIZE);
    busy_wait_ms(100);
    return status;
}

static bool Ssd1306_Write(unsigned char *data, unsigned long length)
{
    bool status = true;
    status = I2C_WriteData(SSD1306_DEV_ADR, data, length);
    return status;
}

unsigned char* Ssd1306_Get_Draw_Canvas()
{
    return p_Draw_Canvas;
}

void Ssd1306_Update_Frame()
{
    Draw_Canvas_Payload[0] = SSD1306_CTRL_BYTE_DATA_SINGLE;
    Ssd1306_Write(Draw_Canvas_Payload, SSD1306_DRAW_CANVAS_PAYLOAD_SIZE);
}

bool I2C_WriteData(unsigned char dev_adr, void *buf, unsigned long buf_length) {
    i2c_write_blocking(I2C_PORT, dev_adr, buf, buf_length, false);
}

void DrawMessage(int row_pos, char *Message, bool reverse_flg)
{
    int MessageCount = strlen(Message);

    if (MessageCount > 8) MessageCount = 8;

    for (int col_pos = 0; col_pos < 8; col_pos++) {
        for (int i = 0; i < col_byte; i++) {
            canvas[(row_byte * row_pos) + (col_byte * col_pos) + i] = 0x00;
            canvas[(row_byte * row_pos) + (col_byte * col_pos) + row_byte_half + i] = 0x00;
        }
    }

    DispPatternDef dispCharPattern;
    for (int col_pos = 0; col_pos < MessageCount; col_pos++) {
        dispCharPattern = getcharPattern(Message[col_pos]);
        for (int i = 0; i < col_byte; i++) {

            if (reverse_flg) {
                dispCharPattern.pattern_head[i] = 0xFF - dispCharPattern.pattern_head[i];
                dispCharPattern.pattern_foot[i] = 0xFF - dispCharPattern.pattern_foot[i]; 
            }

            canvas[(row_byte * row_pos) + (col_byte * col_pos) + i] = dispCharPattern.pattern_head[i];
            canvas[(row_byte * row_pos) + (col_byte * col_pos) + row_byte_half + i] = dispCharPattern.pattern_foot[i];
        }
    }
	Ssd1306_Update_Frame();
}

DispPatternDef getcharPattern(char DispChar) {
    for (int i = 0; i < 100; i++) {
        if (DispPattern[i].dispChar == DispChar) {
            return DispPattern[i];
            break;
        }
    }
}

void DrawControlPanel(char InputState[9])
{
    int i;
    int row_pos = 2;
    int col_pos;
    for (col_pos = 0; col_pos < 8; col_pos++) {
        for (i = 0; i < col_byte; i++) {
            canvas[(row_byte * row_pos) + (col_byte * col_pos) + i] = 0x00;
            canvas[(row_byte * row_pos) + (col_byte * col_pos) + row_byte_half + i] = 0x00;
            canvas[(row_byte * (row_pos + 1)) + (col_byte * col_pos) + i] = 0x00;
            canvas[(row_byte * (row_pos + 1)) + (col_byte * col_pos) + row_byte_half + i] = 0x00;
        }
    }

    DispPatternDef dispCharPattern;

    // ←
    row_pos = 2;
    col_pos = 0;
    dispCharPattern = (InputState[5] == '0') ? DispPattern[94] : DispPattern[95];
    for (i = 0; i < col_byte; i++) {
        canvas[(row_byte * row_pos) + (col_byte * col_pos) + row_byte_half + i] = dispCharPattern.pattern_head[i];
        canvas[(row_byte * row_pos) + (col_byte * col_pos) + row_byte + i] = dispCharPattern.pattern_foot[i];
    }
    // ↑
    row_pos = 2;
    col_pos = 1;
    dispCharPattern = (InputState[2] == '0') ? DispPattern[96] : DispPattern[97];
    for (i = 0; i < col_byte; i++) {
        canvas[(row_byte * row_pos) + (col_byte * col_pos) + i] = dispCharPattern.pattern_head[i];
        canvas[(row_byte * row_pos) + (col_byte * col_pos) + row_byte_half + i] = dispCharPattern.pattern_foot[i];
    }
    // ↓
    col_pos = 1;
    row_pos = 3;
    dispCharPattern = (InputState[3] == '0') ? DispPattern[98] : DispPattern[99];
    for (i = 0; i < col_byte; i++) {
        canvas[(row_byte * row_pos) + (col_byte * col_pos) + i] = dispCharPattern.pattern_head[i];
        canvas[(row_byte * row_pos) + (col_byte * col_pos) + row_byte_half + i] = dispCharPattern.pattern_foot[i];
    }
    // →
    row_pos = 2;
    col_pos = 2;
    dispCharPattern = (InputState[4] == '0') ? DispPattern[92] : DispPattern[93];
    for (i = 0; i < col_byte; i++) {
        canvas[(row_byte * row_pos) + (col_byte * col_pos) + row_byte_half + i] = dispCharPattern.pattern_head[i];
        canvas[(row_byte * row_pos) + (col_byte * col_pos) + row_byte + i] = dispCharPattern.pattern_foot[i];
    }

    row_pos = 2;
    int j = 6;
    for (row_pos = 2;row_pos < 4; row_pos++) {
        for (col_pos = 3; col_pos < 6; col_pos++) {
            dispCharPattern = (InputState[j] == '0') ? DispPattern[90] : DispPattern[91];
            for (i = 0; i < col_byte; i++) {
                canvas[(row_byte * row_pos) + (col_byte * col_pos) + i] = dispCharPattern.pattern_head[i];
                canvas[(row_byte * row_pos) + (col_byte * col_pos) + row_byte_half + i] = dispCharPattern.pattern_foot[i];
            }
            j++;
        }
    }

    // Credit
    row_pos = 2;
    col_pos = 7;
    dispCharPattern = (InputState[0] == '0') ? DispPattern[86] : DispPattern[87];
    for (i = 0; i < col_byte; i++) {
        canvas[(row_byte * row_pos) + (col_byte * col_pos) + i] = dispCharPattern.pattern_head[i];
        canvas[(row_byte * row_pos) + (col_byte * col_pos) + row_byte_half + i] = dispCharPattern.pattern_foot[i];
    }

    // Start
    row_pos = 3;
    col_pos = 7;
    dispCharPattern = (InputState[1] == '0') ? DispPattern[88] : DispPattern[89];
    for (i = 0; i < col_byte; i++) {
        canvas[(row_byte * row_pos) + (col_byte * col_pos) + i] = dispCharPattern.pattern_head[i];
        canvas[(row_byte * row_pos) + (col_byte * col_pos) + row_byte_half + i] = dispCharPattern.pattern_foot[i];
    }

	Ssd1306_Update_Frame();
}

void SetCharPattern() {
    SetCharPattern_Number();
    SetCharPattern_ALPHABET();
    SetCharPattern_alphabet();
    SetCharPattern_symbol(); 
}

void SetCharPattern_Number() 
{
    DispPattern[0].dispChar = '0';
    DispPattern[0].pattern_head[0] = 0x00;
    DispPattern[0].pattern_head[1] = 0x00;
    DispPattern[0].pattern_head[2] = 0x00;
    DispPattern[0].pattern_head[3] = 0xE0;
    DispPattern[0].pattern_head[4] = 0x38;
    DispPattern[0].pattern_head[5] = 0x04;
    DispPattern[0].pattern_head[6] = 0x02;
    DispPattern[0].pattern_head[7] = 0x02;
    DispPattern[0].pattern_head[8] = 0x02;
    DispPattern[0].pattern_head[9] = 0x02;
    DispPattern[0].pattern_head[10] = 0x04;
    DispPattern[0].pattern_head[11] = 0x38;
    DispPattern[0].pattern_head[12] = 0xE0;
    DispPattern[0].pattern_head[13] = 0x00;
    DispPattern[0].pattern_head[14] = 0x00;
    DispPattern[0].pattern_head[15] = 0x00;
    DispPattern[0].pattern_foot[0] = 0x00;
    DispPattern[0].pattern_foot[1] = 0x00;
    DispPattern[0].pattern_foot[2] = 0x00;
    DispPattern[0].pattern_foot[3] = 0x07;
    DispPattern[0].pattern_foot[4] = 0x1C;
    DispPattern[0].pattern_foot[5] = 0x20;
    DispPattern[0].pattern_foot[6] = 0x40;
    DispPattern[0].pattern_foot[7] = 0x40;
    DispPattern[0].pattern_foot[8] = 0x40;
    DispPattern[0].pattern_foot[9] = 0x40;
    DispPattern[0].pattern_foot[10] = 0x20;
    DispPattern[0].pattern_foot[11] = 0x1C;
    DispPattern[0].pattern_foot[12] = 0x07;
    DispPattern[0].pattern_foot[13] = 0x00;
    DispPattern[0].pattern_foot[14] = 0x00;
    DispPattern[0].pattern_foot[15] = 0x00;

    DispPattern[1].dispChar = '1';
    DispPattern[1].pattern_head[0] = 0x00;
    DispPattern[1].pattern_head[1] = 0x00;
    DispPattern[1].pattern_head[2] = 0x00;
    DispPattern[1].pattern_head[3] = 0x00;
    DispPattern[1].pattern_head[4] = 0x10;
    DispPattern[1].pattern_head[5] = 0x08;
    DispPattern[1].pattern_head[6] = 0x04;
    DispPattern[1].pattern_head[7] = 0xFE;
    DispPattern[1].pattern_head[8] = 0x00;
    DispPattern[1].pattern_head[9] = 0x00;
    DispPattern[1].pattern_head[10] = 0x00;
    DispPattern[1].pattern_head[11] = 0x00;
    DispPattern[1].pattern_head[12] = 0x00;
    DispPattern[1].pattern_head[13] = 0x00;
    DispPattern[1].pattern_head[14] = 0x00;
    DispPattern[1].pattern_head[15] = 0x00;
    DispPattern[1].pattern_foot[0] = 0x00;
    DispPattern[1].pattern_foot[1] = 0x00;
    DispPattern[1].pattern_foot[2] = 0x00;
    DispPattern[1].pattern_foot[3] = 0x00;
    DispPattern[1].pattern_foot[4] = 0x00;
    DispPattern[1].pattern_foot[5] = 0x40;
    DispPattern[1].pattern_foot[6] = 0x40;
    DispPattern[1].pattern_foot[7] = 0x7F;
    DispPattern[1].pattern_foot[8] = 0x40;
    DispPattern[1].pattern_foot[9] = 0x40;
    DispPattern[1].pattern_foot[10] = 0x00;
    DispPattern[1].pattern_foot[11] = 0x00;
    DispPattern[1].pattern_foot[12] = 0x00;
    DispPattern[1].pattern_foot[13] = 0x00;
    DispPattern[1].pattern_foot[14] = 0x00;
    DispPattern[1].pattern_foot[15] = 0x00;

    DispPattern[2].dispChar ='2';
    DispPattern[2].pattern_head[0] = 0x00;
    DispPattern[2].pattern_head[1] = 0x00;
    DispPattern[2].pattern_head[2] = 0x00;
    DispPattern[2].pattern_head[3] = 0x18;
    DispPattern[2].pattern_head[4] = 0x0C;
    DispPattern[2].pattern_head[5] = 0x06;
    DispPattern[2].pattern_head[6] = 0x02;
    DispPattern[2].pattern_head[7] = 0x02;
    DispPattern[2].pattern_head[8] = 0x02;
    DispPattern[2].pattern_head[9] = 0x02;
    DispPattern[2].pattern_head[10] = 0x82;
    DispPattern[2].pattern_head[11] = 0xC4;
    DispPattern[2].pattern_head[12] = 0x78;
    DispPattern[2].pattern_head[13] = 0x00;
    DispPattern[2].pattern_head[14] = 0x00;
    DispPattern[2].pattern_head[15] = 0x00;
    DispPattern[2].pattern_foot[0] = 0x00;
    DispPattern[2].pattern_foot[1] = 0x00;
    DispPattern[2].pattern_foot[2] = 0x00;
    DispPattern[2].pattern_foot[3] = 0x40;
    DispPattern[2].pattern_foot[4] = 0x60;
    DispPattern[2].pattern_foot[5] = 0x70;
    DispPattern[2].pattern_foot[6] = 0x58;
    DispPattern[2].pattern_foot[7] = 0x4C;
    DispPattern[2].pattern_foot[8] = 0x46;
    DispPattern[2].pattern_foot[9] = 0x43;
    DispPattern[2].pattern_foot[10] = 0x41;
    DispPattern[2].pattern_foot[11] = 0x40;
    DispPattern[2].pattern_foot[12] = 0x40;
    DispPattern[2].pattern_foot[13] = 0x00;
    DispPattern[2].pattern_foot[14] = 0x00;
    DispPattern[2].pattern_foot[15] = 0x00;

    DispPattern[3].dispChar ='3';
    DispPattern[3].pattern_head[0] = 0x00;
    DispPattern[3].pattern_head[1] = 0x00;
    DispPattern[3].pattern_head[2] = 0x00;
    DispPattern[3].pattern_head[3] = 0x08;
    DispPattern[3].pattern_head[4] = 0x0C;
    DispPattern[3].pattern_head[5] = 0x84;
    DispPattern[3].pattern_head[6] = 0x86;
    DispPattern[3].pattern_head[7] = 0x82;
    DispPattern[3].pattern_head[8] = 0x82;
    DispPattern[3].pattern_head[9] = 0x82;
    DispPattern[3].pattern_head[10] = 0x82;
    DispPattern[3].pattern_head[11] = 0x44;
    DispPattern[3].pattern_head[12] = 0x38;
    DispPattern[3].pattern_head[13] = 0x00;
    DispPattern[3].pattern_head[14] = 0x00;
    DispPattern[3].pattern_head[15] = 0x00;
    DispPattern[3].pattern_foot[0] = 0x00;
    DispPattern[3].pattern_foot[1] = 0x00;
    DispPattern[3].pattern_foot[2] = 0x00;
    DispPattern[3].pattern_foot[3] = 0x40;
    DispPattern[3].pattern_foot[4] = 0x40;
    DispPattern[3].pattern_foot[5] = 0x40;
    DispPattern[3].pattern_foot[6] = 0x40;
    DispPattern[3].pattern_foot[7] = 0x40;
    DispPattern[3].pattern_foot[8] = 0x40;
    DispPattern[3].pattern_foot[9] = 0x60;
    DispPattern[3].pattern_foot[10] = 0x20;
    DispPattern[3].pattern_foot[11] = 0x31;
    DispPattern[3].pattern_foot[12] = 0x1E;
    DispPattern[3].pattern_foot[13] = 0x00;
    DispPattern[3].pattern_foot[14] = 0x00;
    DispPattern[3].pattern_foot[15] = 0x00;

    DispPattern[4].dispChar ='4';
    DispPattern[4].pattern_head[0] = 0x00;
    DispPattern[4].pattern_head[1] = 0x00;
    DispPattern[4].pattern_head[2] = 0x00;
    DispPattern[4].pattern_head[3] = 0x80;
    DispPattern[4].pattern_head[4] = 0xC0;
    DispPattern[4].pattern_head[5] = 0x60;
    DispPattern[4].pattern_head[6] = 0x30;
    DispPattern[4].pattern_head[7] = 0x18;
    DispPattern[4].pattern_head[8] = 0x0C;
    DispPattern[4].pattern_head[9] = 0x06;
    DispPattern[4].pattern_head[10] = 0xFE;
    DispPattern[4].pattern_head[11] = 0x00;
    DispPattern[4].pattern_head[12] = 0x00;
    DispPattern[4].pattern_head[13] = 0x00;
    DispPattern[4].pattern_head[14] = 0x00;
    DispPattern[4].pattern_head[15] = 0x00;
    DispPattern[4].pattern_foot[0] = 0x00;
    DispPattern[4].pattern_foot[1] = 0x00;
    DispPattern[4].pattern_foot[2] = 0x07;
    DispPattern[4].pattern_foot[3] = 0x05;
    DispPattern[4].pattern_foot[4] = 0x04;
    DispPattern[4].pattern_foot[5] = 0x04;
    DispPattern[4].pattern_foot[6] = 0x04;
    DispPattern[4].pattern_foot[7] = 0x04;
    DispPattern[4].pattern_foot[8] = 0x44;
    DispPattern[4].pattern_foot[9] = 0x44;
    DispPattern[4].pattern_foot[10] = 0x7F;
    DispPattern[4].pattern_foot[11] = 0x44;
    DispPattern[4].pattern_foot[12] = 0x44;
    DispPattern[4].pattern_foot[13] = 0x04;
    DispPattern[4].pattern_foot[14] = 0x00;
    DispPattern[4].pattern_foot[15] = 0x00;

    DispPattern[5].dispChar ='5';
    DispPattern[5].pattern_head[0] = 0x00;
    DispPattern[5].pattern_head[1] = 0x00;
    DispPattern[5].pattern_head[2] = 0x00;
    DispPattern[5].pattern_head[3] = 0xFE;
    DispPattern[5].pattern_head[4] = 0x82;
    DispPattern[5].pattern_head[5] = 0x42;
    DispPattern[5].pattern_head[6] = 0x42;
    DispPattern[5].pattern_head[7] = 0x42;
    DispPattern[5].pattern_head[8] = 0x42;
    DispPattern[5].pattern_head[9] = 0x42;
    DispPattern[5].pattern_head[10] = 0x42;
    DispPattern[5].pattern_head[11] = 0x82;
    DispPattern[5].pattern_head[12] = 0x02;
    DispPattern[5].pattern_head[13] = 0x00;
    DispPattern[5].pattern_head[14] = 0x00;
    DispPattern[5].pattern_head[15] = 0x00;
    DispPattern[5].pattern_foot[0] = 0x00;
    DispPattern[5].pattern_foot[1] = 0x00;
    DispPattern[5].pattern_foot[2] = 0x00;
    DispPattern[5].pattern_foot[3] = 0x41;
    DispPattern[5].pattern_foot[4] = 0x40;
    DispPattern[5].pattern_foot[5] = 0x40;
    DispPattern[5].pattern_foot[6] = 0x40;
    DispPattern[5].pattern_foot[7] = 0x40;
    DispPattern[5].pattern_foot[8] = 0x60;
    DispPattern[5].pattern_foot[9] = 0x20;
    DispPattern[5].pattern_foot[10] = 0x30;
    DispPattern[5].pattern_foot[11] = 0x18;
    DispPattern[5].pattern_foot[12] = 0x0F;
    DispPattern[5].pattern_foot[13] = 0x00;
    DispPattern[5].pattern_foot[14] = 0x00;
    DispPattern[5].pattern_foot[15] = 0x00;

    DispPattern[6].dispChar ='6';
    DispPattern[6].pattern_head[0] = 0x00;
    DispPattern[6].pattern_head[1] = 0x00;
    DispPattern[6].pattern_head[2] = 0x00;
    DispPattern[6].pattern_head[3] = 0xC0;
    DispPattern[6].pattern_head[4] = 0x70;
    DispPattern[6].pattern_head[5] = 0x98;
    DispPattern[6].pattern_head[6] = 0x4C;
    DispPattern[6].pattern_head[7] = 0x44;
    DispPattern[6].pattern_head[8] = 0x46;
    DispPattern[6].pattern_head[9] = 0x42;
    DispPattern[6].pattern_head[10] = 0x82;
    DispPattern[6].pattern_head[11] = 0x82;
    DispPattern[6].pattern_head[12] = 0x00;
    DispPattern[6].pattern_head[13] = 0x00;
    DispPattern[6].pattern_head[14] = 0x00;
    DispPattern[6].pattern_head[15] = 0x00;
    DispPattern[6].pattern_foot[0] = 0x00;
    DispPattern[6].pattern_foot[1] = 0x00;
    DispPattern[6].pattern_foot[2] = 0x00;
    DispPattern[6].pattern_foot[3] = 0x0F;
    DispPattern[6].pattern_foot[4] = 0x39;
    DispPattern[6].pattern_foot[5] = 0x20;
    DispPattern[6].pattern_foot[6] = 0x40;
    DispPattern[6].pattern_foot[7] = 0x40;
    DispPattern[6].pattern_foot[8] = 0x40;
    DispPattern[6].pattern_foot[9] = 0x40;
    DispPattern[6].pattern_foot[10] = 0x20;
    DispPattern[6].pattern_foot[11] = 0x31;
    DispPattern[6].pattern_foot[12] = 0x1F;
    DispPattern[6].pattern_foot[13] = 0x00;
    DispPattern[6].pattern_foot[14] = 0x00;
    DispPattern[6].pattern_foot[15] = 0x00;

    DispPattern[7].dispChar ='7';
    DispPattern[7].pattern_head[0] = 0x00;
    DispPattern[7].pattern_head[1] = 0x00;
    DispPattern[7].pattern_head[2] = 0x00;
    DispPattern[7].pattern_head[3] = 0x1E;
    DispPattern[7].pattern_head[4] = 0x02;
    DispPattern[7].pattern_head[5] = 0x02;
    DispPattern[7].pattern_head[6] = 0x02;
    DispPattern[7].pattern_head[7] = 0x02;
    DispPattern[7].pattern_head[8] = 0x02;
    DispPattern[7].pattern_head[9] = 0x82;
    DispPattern[7].pattern_head[10] = 0xE2;
    DispPattern[7].pattern_head[11] = 0x3A;
    DispPattern[7].pattern_head[12] = 0x0E;
    DispPattern[7].pattern_head[13] = 0x02;
    DispPattern[7].pattern_head[14] = 0x00;
    DispPattern[7].pattern_head[15] = 0x00;
    DispPattern[7].pattern_foot[0] = 0x00;
    DispPattern[7].pattern_foot[1] = 0x00;
    DispPattern[7].pattern_foot[2] = 0x00;
    DispPattern[7].pattern_foot[3] = 0x00;
    DispPattern[7].pattern_foot[4] = 0x00;
    DispPattern[7].pattern_foot[5] = 0x00;
    DispPattern[7].pattern_foot[6] = 0x00;
    DispPattern[7].pattern_foot[7] = 0x70;
    DispPattern[7].pattern_foot[8] = 0x1E;
    DispPattern[7].pattern_foot[9] = 0x03;
    DispPattern[7].pattern_foot[10] = 0x00;
    DispPattern[7].pattern_foot[11] = 0x00;
    DispPattern[7].pattern_foot[12] = 0x00;
    DispPattern[7].pattern_foot[13] = 0x00;
    DispPattern[7].pattern_foot[14] = 0x00;
    DispPattern[7].pattern_foot[15] = 0x00;

    DispPattern[8].dispChar ='8';
    DispPattern[8].pattern_head[0] = 0x00;
    DispPattern[8].pattern_head[1] = 0x00;
    DispPattern[8].pattern_head[2] = 0x00;
    DispPattern[8].pattern_head[3] = 0x38;
    DispPattern[8].pattern_head[4] = 0x4C;
    DispPattern[8].pattern_head[5] = 0x84;
    DispPattern[8].pattern_head[6] = 0x82;
    DispPattern[8].pattern_head[7] = 0x82;
    DispPattern[8].pattern_head[8] = 0x82;
    DispPattern[8].pattern_head[9] = 0x82;
    DispPattern[8].pattern_head[10] = 0x84;
    DispPattern[8].pattern_head[11] = 0x4C;
    DispPattern[8].pattern_head[12] = 0x38;
    DispPattern[8].pattern_head[13] = 0x00;
    DispPattern[8].pattern_head[14] = 0x00;
    DispPattern[8].pattern_head[15] = 0x00;
    DispPattern[8].pattern_foot[0] = 0x00;
    DispPattern[8].pattern_foot[1] = 0x00;
    DispPattern[8].pattern_foot[2] = 0x00;
    DispPattern[8].pattern_foot[3] = 0x1E;
    DispPattern[8].pattern_foot[4] = 0x31;
    DispPattern[8].pattern_foot[5] = 0x20;
    DispPattern[8].pattern_foot[6] = 0x40;
    DispPattern[8].pattern_foot[7] = 0x40;
    DispPattern[8].pattern_foot[8] = 0x40;
    DispPattern[8].pattern_foot[9] = 0x40;
    DispPattern[8].pattern_foot[10] = 0x20;
    DispPattern[8].pattern_foot[11] = 0x31;
    DispPattern[8].pattern_foot[12] = 0x1E;
    DispPattern[8].pattern_foot[13] = 0x00;
    DispPattern[8].pattern_foot[14] = 0x00;
    DispPattern[8].pattern_foot[15] = 0x00;

    DispPattern[9].dispChar ='9';
    DispPattern[9].pattern_head[0] = 0x00;
    DispPattern[9].pattern_head[1] = 0x00;
    DispPattern[9].pattern_head[2] = 0x00;
    DispPattern[9].pattern_head[3] = 0xF8;
    DispPattern[9].pattern_head[4] = 0x8C;
    DispPattern[9].pattern_head[5] = 0x04;
    DispPattern[9].pattern_head[6] = 0x02;
    DispPattern[9].pattern_head[7] = 0x02;
    DispPattern[9].pattern_head[8] = 0x02;
    DispPattern[9].pattern_head[9] = 0x02;
    DispPattern[9].pattern_head[10] = 0x04;
    DispPattern[9].pattern_head[11] = 0x9C;
    DispPattern[9].pattern_head[12] = 0xF0;
    DispPattern[9].pattern_head[13] = 0x00;
    DispPattern[9].pattern_head[14] = 0x00;
    DispPattern[9].pattern_head[15] = 0x00;
    DispPattern[9].pattern_foot[0] = 0x00;
    DispPattern[9].pattern_foot[1] = 0x00;
    DispPattern[9].pattern_foot[2] = 0x00;
    DispPattern[9].pattern_foot[3] = 0x00;
    DispPattern[9].pattern_foot[4] = 0x41;
    DispPattern[9].pattern_foot[5] = 0x41;
    DispPattern[9].pattern_foot[6] = 0x42;
    DispPattern[9].pattern_foot[7] = 0x62;
    DispPattern[9].pattern_foot[8] = 0x22;
    DispPattern[9].pattern_foot[9] = 0x32;
    DispPattern[9].pattern_foot[10] = 0x19;
    DispPattern[9].pattern_foot[11] = 0x0E;
    DispPattern[9].pattern_foot[12] = 0x03;
    DispPattern[9].pattern_foot[13] = 0x00;
    DispPattern[9].pattern_foot[14] = 0x00;
    DispPattern[9].pattern_foot[15] = 0x00;

}

void SetCharPattern_ALPHABET() 
{
    DispPattern[17].dispChar ='A';
    DispPattern[17].pattern_head[0] = 0x00;
    DispPattern[17].pattern_head[1] = 0x00;
    DispPattern[17].pattern_head[2] = 0x00;
    DispPattern[17].pattern_head[3] = 0x00;
    DispPattern[17].pattern_head[4] = 0xC0;
    DispPattern[17].pattern_head[5] = 0x70;
    DispPattern[17].pattern_head[6] = 0x1C;
    DispPattern[17].pattern_head[7] = 0x06;
    DispPattern[17].pattern_head[8] = 0x1C;
    DispPattern[17].pattern_head[9] = 0x70;
    DispPattern[17].pattern_head[10] = 0xC0;
    DispPattern[17].pattern_head[11] = 0x00;
    DispPattern[17].pattern_head[12] = 0x00;
    DispPattern[17].pattern_head[13] = 0x00;
    DispPattern[17].pattern_head[14] = 0x00;
    DispPattern[17].pattern_head[15] = 0x00;
    DispPattern[17].pattern_foot[0] = 0x20;
    DispPattern[17].pattern_foot[1] = 0x20;
    DispPattern[17].pattern_foot[2] = 0x38;
    DispPattern[17].pattern_foot[3] = 0x2E;
    DispPattern[17].pattern_foot[4] = 0x23;
    DispPattern[17].pattern_foot[5] = 0x01;
    DispPattern[17].pattern_foot[6] = 0x01;
    DispPattern[17].pattern_foot[7] = 0x01;
    DispPattern[17].pattern_foot[8] = 0x01;
    DispPattern[17].pattern_foot[9] = 0x01;
    DispPattern[17].pattern_foot[10] = 0x23;
    DispPattern[17].pattern_foot[11] = 0x2E;
    DispPattern[17].pattern_foot[12] = 0x38;
    DispPattern[17].pattern_foot[13] = 0x20;
    DispPattern[17].pattern_foot[14] = 0x20;
    DispPattern[17].pattern_foot[15] = 0x00;

    DispPattern[18].dispChar ='B';
    DispPattern[18].pattern_head[0] = 0x00;
    DispPattern[18].pattern_head[1] = 0x00;
    DispPattern[18].pattern_head[2] = 0x02;
    DispPattern[18].pattern_head[3] = 0x02;
    DispPattern[18].pattern_head[4] = 0xFE;
    DispPattern[18].pattern_head[5] = 0x82;
    DispPattern[18].pattern_head[6] = 0x82;
    DispPattern[18].pattern_head[7] = 0x82;
    DispPattern[18].pattern_head[8] = 0x82;
    DispPattern[18].pattern_head[9] = 0x82;
    DispPattern[18].pattern_head[10] = 0x82;
    DispPattern[18].pattern_head[11] = 0x44;
    DispPattern[18].pattern_head[12] = 0x38;
    DispPattern[18].pattern_head[13] = 0x00;
    DispPattern[18].pattern_head[14] = 0x00;
    DispPattern[18].pattern_head[15] = 0x00;
    DispPattern[18].pattern_foot[0] = 0x00;
    DispPattern[18].pattern_foot[1] = 0x00;
    DispPattern[18].pattern_foot[2] = 0x20;
    DispPattern[18].pattern_foot[3] = 0x20;
    DispPattern[18].pattern_foot[4] = 0x3F;
    DispPattern[18].pattern_foot[5] = 0x20;
    DispPattern[18].pattern_foot[6] = 0x20;
    DispPattern[18].pattern_foot[7] = 0x20;
    DispPattern[18].pattern_foot[8] = 0x20;
    DispPattern[18].pattern_foot[9] = 0x20;
    DispPattern[18].pattern_foot[10] = 0x20;
    DispPattern[18].pattern_foot[11] = 0x21;
    DispPattern[18].pattern_foot[12] = 0x11;
    DispPattern[18].pattern_foot[13] = 0x0E;
    DispPattern[18].pattern_foot[14] = 0x00;
    DispPattern[18].pattern_foot[15] = 0x00;

    DispPattern[19].dispChar ='C';
    DispPattern[19].pattern_head[0] = 0x00;
    DispPattern[19].pattern_head[1] = 0x00;
    DispPattern[19].pattern_head[2] = 0xE0;
    DispPattern[19].pattern_head[3] = 0x38;
    DispPattern[19].pattern_head[4] = 0x0C;
    DispPattern[19].pattern_head[5] = 0x04;
    DispPattern[19].pattern_head[6] = 0x02;
    DispPattern[19].pattern_head[7] = 0x02;
    DispPattern[19].pattern_head[8] = 0x02;
    DispPattern[19].pattern_head[9] = 0x02;
    DispPattern[19].pattern_head[10] = 0x04;
    DispPattern[19].pattern_head[11] = 0x08;
    DispPattern[19].pattern_head[12] = 0x3E;
    DispPattern[19].pattern_head[13] = 0x00;
    DispPattern[19].pattern_head[14] = 0x00;
    DispPattern[19].pattern_head[15] = 0x00;
    DispPattern[19].pattern_foot[0] = 0x00;
    DispPattern[19].pattern_foot[1] = 0x00;
    DispPattern[19].pattern_foot[2] = 0x03;
    DispPattern[19].pattern_foot[3] = 0x0E;
    DispPattern[19].pattern_foot[4] = 0x18;
    DispPattern[19].pattern_foot[5] = 0x10;
    DispPattern[19].pattern_foot[6] = 0x20;
    DispPattern[19].pattern_foot[7] = 0x20;
    DispPattern[19].pattern_foot[8] = 0x20;
    DispPattern[19].pattern_foot[9] = 0x20;
    DispPattern[19].pattern_foot[10] = 0x10;
    DispPattern[19].pattern_foot[11] = 0x18;
    DispPattern[19].pattern_foot[12] = 0x0C;
    DispPattern[19].pattern_foot[13] = 0x00;
    DispPattern[19].pattern_foot[14] = 0x00;
    DispPattern[19].pattern_foot[15] = 0x00;

    DispPattern[20].dispChar ='D';
    DispPattern[20].pattern_head[0] = 0x00;
    DispPattern[20].pattern_head[1] = 0x00;
    DispPattern[20].pattern_head[2] = 0x02;
    DispPattern[20].pattern_head[3] = 0x02;
    DispPattern[20].pattern_head[4] = 0xFE;
    DispPattern[20].pattern_head[5] = 0x02;
    DispPattern[20].pattern_head[6] = 0x02;
    DispPattern[20].pattern_head[7] = 0x02;
    DispPattern[20].pattern_head[8] = 0x02;
    DispPattern[20].pattern_head[9] = 0x02;
    DispPattern[20].pattern_head[10] = 0x04;
    DispPattern[20].pattern_head[11] = 0x0C;
    DispPattern[20].pattern_head[12] = 0x38;
    DispPattern[20].pattern_head[13] = 0xE0;
    DispPattern[20].pattern_head[14] = 0x00;
    DispPattern[20].pattern_head[15] = 0x00;
    DispPattern[20].pattern_foot[0] = 0x00;
    DispPattern[20].pattern_foot[1] = 0x00;
    DispPattern[20].pattern_foot[2] = 0x20;
    DispPattern[20].pattern_foot[3] = 0x20;
    DispPattern[20].pattern_foot[4] = 0x3F;
    DispPattern[20].pattern_foot[5] = 0x20;
    DispPattern[20].pattern_foot[6] = 0x20;
    DispPattern[20].pattern_foot[7] = 0x20;
    DispPattern[20].pattern_foot[8] = 0x20;
    DispPattern[20].pattern_foot[9] = 0x20;
    DispPattern[20].pattern_foot[10] = 0x10;
    DispPattern[20].pattern_foot[11] = 0x18;
    DispPattern[20].pattern_foot[12] = 0x0E;
    DispPattern[20].pattern_foot[13] = 0x03;
    DispPattern[20].pattern_foot[14] = 0x00;
    DispPattern[20].pattern_foot[15] = 0x00;

    DispPattern[21].dispChar ='E';
    DispPattern[21].pattern_head[0] = 0x00;
    DispPattern[21].pattern_head[1] = 0x00;
    DispPattern[21].pattern_head[2] = 0x02;
    DispPattern[21].pattern_head[3] = 0x02;
    DispPattern[21].pattern_head[4] = 0xFE;
    DispPattern[21].pattern_head[5] = 0x82;
    DispPattern[21].pattern_head[6] = 0x82;
    DispPattern[21].pattern_head[7] = 0x82;
    DispPattern[21].pattern_head[8] = 0x82;
    DispPattern[21].pattern_head[9] = 0xE2;
    DispPattern[21].pattern_head[10] = 0x02;
    DispPattern[21].pattern_head[11] = 0x02;
    DispPattern[21].pattern_head[12] = 0x06;
    DispPattern[21].pattern_head[13] = 0x08;
    DispPattern[21].pattern_head[14] = 0x00;
    DispPattern[21].pattern_head[15] = 0x00;
    DispPattern[21].pattern_foot[0] = 0x00;
    DispPattern[21].pattern_foot[1] = 0x00;
    DispPattern[21].pattern_foot[2] = 0x20;
    DispPattern[21].pattern_foot[3] = 0x20;
    DispPattern[21].pattern_foot[4] = 0x3F;
    DispPattern[21].pattern_foot[5] = 0x20;
    DispPattern[21].pattern_foot[6] = 0x20;
    DispPattern[21].pattern_foot[7] = 0x20;
    DispPattern[21].pattern_foot[8] = 0x20;
    DispPattern[21].pattern_foot[9] = 0x23;
    DispPattern[21].pattern_foot[10] = 0x20;
    DispPattern[21].pattern_foot[11] = 0x20;
    DispPattern[21].pattern_foot[12] = 0x30;
    DispPattern[21].pattern_foot[13] = 0x08;
    DispPattern[21].pattern_foot[14] = 0x00;
    DispPattern[21].pattern_foot[15] = 0x00;

    DispPattern[22].dispChar ='F';
    DispPattern[22].pattern_head[0] = 0x00;
    DispPattern[22].pattern_head[1] = 0x00;
    DispPattern[22].pattern_head[2] = 0x00;
    DispPattern[22].pattern_head[3] = 0x02;
    DispPattern[22].pattern_head[4] = 0x02;
    DispPattern[22].pattern_head[5] = 0xFE;
    DispPattern[22].pattern_head[6] = 0x82;
    DispPattern[22].pattern_head[7] = 0x82;
    DispPattern[22].pattern_head[8] = 0x82;
    DispPattern[22].pattern_head[9] = 0x82;
    DispPattern[22].pattern_head[10] = 0xE2;
    DispPattern[22].pattern_head[11] = 0x02;
    DispPattern[22].pattern_head[12] = 0x02;
    DispPattern[22].pattern_head[13] = 0x0E;
    DispPattern[22].pattern_head[14] = 0x00;
    DispPattern[22].pattern_head[15] = 0x00;
    DispPattern[22].pattern_foot[0] = 0x00;
    DispPattern[22].pattern_foot[1] = 0x00;
    DispPattern[22].pattern_foot[2] = 0x00;
    DispPattern[22].pattern_foot[3] = 0x20;
    DispPattern[22].pattern_foot[4] = 0x20;
    DispPattern[22].pattern_foot[5] = 0x3F;
    DispPattern[22].pattern_foot[6] = 0x20;
    DispPattern[22].pattern_foot[7] = 0x20;
    DispPattern[22].pattern_foot[8] = 0x00;
    DispPattern[22].pattern_foot[9] = 0x00;
    DispPattern[22].pattern_foot[10] = 0x03;
    DispPattern[22].pattern_foot[11] = 0x00;
    DispPattern[22].pattern_foot[12] = 0x00;
    DispPattern[22].pattern_foot[13] = 0x00;
    DispPattern[22].pattern_foot[14] = 0x00;
    DispPattern[22].pattern_foot[15] = 0x00;

    DispPattern[23].dispChar ='G';
    DispPattern[23].pattern_head[0] = 0x00;
    DispPattern[23].pattern_head[1] = 0x00;
    DispPattern[23].pattern_head[2] = 0xE0;
    DispPattern[23].pattern_head[3] = 0x38;
    DispPattern[23].pattern_head[4] = 0x0C;
    DispPattern[23].pattern_head[5] = 0x04;
    DispPattern[23].pattern_head[6] = 0x02;
    DispPattern[23].pattern_head[7] = 0x02;
    DispPattern[23].pattern_head[8] = 0x02;
    DispPattern[23].pattern_head[9] = 0x02;
    DispPattern[23].pattern_head[10] = 0x04;
    DispPattern[23].pattern_head[11] = 0x04;
    DispPattern[23].pattern_head[12] = 0x1E;
    DispPattern[23].pattern_head[13] = 0x00;
    DispPattern[23].pattern_head[14] = 0x00;
    DispPattern[23].pattern_head[15] = 0x00;
    DispPattern[23].pattern_foot[0] = 0x00;
    DispPattern[23].pattern_foot[1] = 0x00;
    DispPattern[23].pattern_foot[2] = 0x03;
    DispPattern[23].pattern_foot[3] = 0x0E;
    DispPattern[23].pattern_foot[4] = 0x18;
    DispPattern[23].pattern_foot[5] = 0x10;
    DispPattern[23].pattern_foot[6] = 0x20;
    DispPattern[23].pattern_foot[7] = 0x20;
    DispPattern[23].pattern_foot[8] = 0x20;
    DispPattern[23].pattern_foot[9] = 0x20;
    DispPattern[23].pattern_foot[10] = 0x11;
    DispPattern[23].pattern_foot[11] = 0x09;
    DispPattern[23].pattern_foot[12] = 0x3F;
    DispPattern[23].pattern_foot[13] = 0x01;
    DispPattern[23].pattern_foot[14] = 0x01;
    DispPattern[23].pattern_foot[15] = 0x00;

    DispPattern[24].dispChar ='H';
    DispPattern[24].pattern_head[0] = 0x00;
    DispPattern[24].pattern_head[1] = 0x02;
    DispPattern[24].pattern_head[2] = 0x02;
    DispPattern[24].pattern_head[3] = 0xFE;
    DispPattern[24].pattern_head[4] = 0x82;
    DispPattern[24].pattern_head[5] = 0x82;
    DispPattern[24].pattern_head[6] = 0x80;
    DispPattern[24].pattern_head[7] = 0x80;
    DispPattern[24].pattern_head[8] = 0x80;
    DispPattern[24].pattern_head[9] = 0x80;
    DispPattern[24].pattern_head[10] = 0x82;
    DispPattern[24].pattern_head[11] = 0x82;
    DispPattern[24].pattern_head[12] = 0xFE;
    DispPattern[24].pattern_head[13] = 0x02;
    DispPattern[24].pattern_head[14] = 0x02;
    DispPattern[24].pattern_head[15] = 0x00;
    DispPattern[24].pattern_foot[0] = 0x00;
    DispPattern[24].pattern_foot[1] = 0x20;
    DispPattern[24].pattern_foot[2] = 0x20;
    DispPattern[24].pattern_foot[3] = 0x3F;
    DispPattern[24].pattern_foot[4] = 0x20;
    DispPattern[24].pattern_foot[5] = 0x20;
    DispPattern[24].pattern_foot[6] = 0x00;
    DispPattern[24].pattern_foot[7] = 0x00;
    DispPattern[24].pattern_foot[8] = 0x00;
    DispPattern[24].pattern_foot[9] = 0x00;
    DispPattern[24].pattern_foot[10] = 0x20;
    DispPattern[24].pattern_foot[11] = 0x20;
    DispPattern[24].pattern_foot[12] = 0x3F;
    DispPattern[24].pattern_foot[13] = 0x20;
    DispPattern[24].pattern_foot[14] = 0x20;
    DispPattern[24].pattern_foot[15] = 0x00;

    DispPattern[25].dispChar ='I';
    DispPattern[25].pattern_head[0] = 0x00;
    DispPattern[25].pattern_head[1] = 0x00;
    DispPattern[25].pattern_head[2] = 0x00;
    DispPattern[25].pattern_head[3] = 0x00;
    DispPattern[25].pattern_head[4] = 0x00;
    DispPattern[25].pattern_head[5] = 0x02;
    DispPattern[25].pattern_head[6] = 0x02;
    DispPattern[25].pattern_head[7] = 0xFE;
    DispPattern[25].pattern_head[8] = 0x02;
    DispPattern[25].pattern_head[9] = 0x02;
    DispPattern[25].pattern_head[10] = 0x00;
    DispPattern[25].pattern_head[11] = 0x00;
    DispPattern[25].pattern_head[12] = 0x00;
    DispPattern[25].pattern_head[13] = 0x00;
    DispPattern[25].pattern_head[14] = 0x00;
    DispPattern[25].pattern_head[15] = 0x00;
    DispPattern[25].pattern_foot[0] = 0x00;
    DispPattern[25].pattern_foot[1] = 0x00;
    DispPattern[25].pattern_foot[2] = 0x00;
    DispPattern[25].pattern_foot[3] = 0x00;
    DispPattern[25].pattern_foot[4] = 0x00;
    DispPattern[25].pattern_foot[5] = 0x20;
    DispPattern[25].pattern_foot[6] = 0x20;
    DispPattern[25].pattern_foot[7] = 0x3F;
    DispPattern[25].pattern_foot[8] = 0x20;
    DispPattern[25].pattern_foot[9] = 0x20;
    DispPattern[25].pattern_foot[10] = 0x00;
    DispPattern[25].pattern_foot[11] = 0x00;
    DispPattern[25].pattern_foot[12] = 0x00;
    DispPattern[25].pattern_foot[13] = 0x00;
    DispPattern[25].pattern_foot[14] = 0x00;
    DispPattern[25].pattern_foot[15] = 0x00;

    DispPattern[26].dispChar ='J';
    DispPattern[26].pattern_head[0] = 0x00;
    DispPattern[26].pattern_head[1] = 0x00;
    DispPattern[26].pattern_head[2] = 0x00;
    DispPattern[26].pattern_head[3] = 0x00;
    DispPattern[26].pattern_head[4] = 0x00;
    DispPattern[26].pattern_head[5] = 0x00;
    DispPattern[26].pattern_head[6] = 0x00;
    DispPattern[26].pattern_head[7] = 0x00;
    DispPattern[26].pattern_head[8] = 0x02;
    DispPattern[26].pattern_head[9] = 0x02;
    DispPattern[26].pattern_head[10] = 0xFE;
    DispPattern[26].pattern_head[11] = 0x02;
    DispPattern[26].pattern_head[12] = 0x02;
    DispPattern[26].pattern_head[13] = 0x00;
    DispPattern[26].pattern_head[14] = 0x00;
    DispPattern[26].pattern_head[15] = 0x00;
    DispPattern[26].pattern_foot[0] = 0x00;
    DispPattern[26].pattern_foot[1] = 0x00;
    DispPattern[26].pattern_foot[2] = 0x00;
    DispPattern[26].pattern_foot[3] = 0x18;
    DispPattern[26].pattern_foot[4] = 0x30;
    DispPattern[26].pattern_foot[5] = 0x20;
    DispPattern[26].pattern_foot[6] = 0x20;
    DispPattern[26].pattern_foot[7] = 0x20;
    DispPattern[26].pattern_foot[8] = 0x20;
    DispPattern[26].pattern_foot[9] = 0x10;
    DispPattern[26].pattern_foot[10] = 0x0F;
    DispPattern[26].pattern_foot[11] = 0x00;
    DispPattern[26].pattern_foot[12] = 0x00;
    DispPattern[26].pattern_foot[13] = 0x00;
    DispPattern[26].pattern_foot[14] = 0x00;
    DispPattern[26].pattern_foot[15] = 0x00;

    DispPattern[27].dispChar ='K';
    DispPattern[27].pattern_head[0] = 0x00;
    DispPattern[27].pattern_head[1] = 0x00;
    DispPattern[27].pattern_head[2] = 0x02;
    DispPattern[27].pattern_head[3] = 0x02;
    DispPattern[27].pattern_head[4] = 0xFE;
    DispPattern[27].pattern_head[5] = 0x82;
    DispPattern[27].pattern_head[6] = 0xC2;
    DispPattern[27].pattern_head[7] = 0xE0;
    DispPattern[27].pattern_head[8] = 0x30;
    DispPattern[27].pattern_head[9] = 0x1A;
    DispPattern[27].pattern_head[10] = 0x0E;
    DispPattern[27].pattern_head[11] = 0x06;
    DispPattern[27].pattern_head[12] = 0x02;
    DispPattern[27].pattern_head[13] = 0x02;
    DispPattern[27].pattern_head[14] = 0x00;
    DispPattern[27].pattern_head[15] = 0x00;
    DispPattern[27].pattern_foot[0] = 0x00;
    DispPattern[27].pattern_foot[1] = 0x00;
    DispPattern[27].pattern_foot[2] = 0x20;
    DispPattern[27].pattern_foot[3] = 0x20;
    DispPattern[27].pattern_foot[4] = 0x3F;
    DispPattern[27].pattern_foot[5] = 0x20;
    DispPattern[27].pattern_foot[6] = 0x20;
    DispPattern[27].pattern_foot[7] = 0x01;
    DispPattern[27].pattern_foot[8] = 0x03;
    DispPattern[27].pattern_foot[9] = 0x06;
    DispPattern[27].pattern_foot[10] = 0x2C;
    DispPattern[27].pattern_foot[11] = 0x38;
    DispPattern[27].pattern_foot[12] = 0x30;
    DispPattern[27].pattern_foot[13] = 0x20;
    DispPattern[27].pattern_foot[14] = 0x20;
    DispPattern[27].pattern_foot[15] = 0x00;

    DispPattern[28].dispChar ='L';
    DispPattern[28].pattern_head[0] = 0x00;
    DispPattern[28].pattern_head[1] = 0x00;
    DispPattern[28].pattern_head[2] = 0x00;
    DispPattern[28].pattern_head[3] = 0x02;
    DispPattern[28].pattern_head[4] = 0x02;
    DispPattern[28].pattern_head[5] = 0xFE;
    DispPattern[28].pattern_head[6] = 0x02;
    DispPattern[28].pattern_head[7] = 0x02;
    DispPattern[28].pattern_head[8] = 0x00;
    DispPattern[28].pattern_head[9] = 0x00;
    DispPattern[28].pattern_head[10] = 0x00;
    DispPattern[28].pattern_head[11] = 0x00;
    DispPattern[28].pattern_head[12] = 0x00;
    DispPattern[28].pattern_head[13] = 0x00;
    DispPattern[28].pattern_head[14] = 0x00;
    DispPattern[28].pattern_head[15] = 0x00;
    DispPattern[28].pattern_foot[0] = 0x00;
    DispPattern[28].pattern_foot[1] = 0x00;
    DispPattern[28].pattern_foot[2] = 0x00;
    DispPattern[28].pattern_foot[3] = 0x20;
    DispPattern[28].pattern_foot[4] = 0x20;
    DispPattern[28].pattern_foot[5] = 0x3F;
    DispPattern[28].pattern_foot[6] = 0x20;
    DispPattern[28].pattern_foot[7] = 0x20;
    DispPattern[28].pattern_foot[8] = 0x20;
    DispPattern[28].pattern_foot[9] = 0x20;
    DispPattern[28].pattern_foot[10] = 0x20;
    DispPattern[28].pattern_foot[11] = 0x20;
    DispPattern[28].pattern_foot[12] = 0x20;
    DispPattern[28].pattern_foot[13] = 0x3C;
    DispPattern[28].pattern_foot[14] = 0x00;
    DispPattern[28].pattern_foot[15] = 0x00;

    DispPattern[29].dispChar ='M';
    DispPattern[29].pattern_head[0] = 0x02;
    DispPattern[29].pattern_head[1] = 0x02;
    DispPattern[29].pattern_head[2] = 0xFE;
    DispPattern[29].pattern_head[3] = 0x0E;
    DispPattern[29].pattern_head[4] = 0x78;
    DispPattern[29].pattern_head[5] = 0xC0;
    DispPattern[29].pattern_head[6] = 0x00;
    DispPattern[29].pattern_head[7] = 0x00;
    DispPattern[29].pattern_head[8] = 0x00;
    DispPattern[29].pattern_head[9] = 0xC0;
    DispPattern[29].pattern_head[10] = 0x78;
    DispPattern[29].pattern_head[11] = 0x0E;
    DispPattern[29].pattern_head[12] = 0xFE;
    DispPattern[29].pattern_head[13] = 0x02;
    DispPattern[29].pattern_head[14] = 0x02;
    DispPattern[29].pattern_head[15] = 0x00;
    DispPattern[29].pattern_foot[0] = 0x20;
    DispPattern[29].pattern_foot[1] = 0x20;
    DispPattern[29].pattern_foot[2] = 0x3F;
    DispPattern[29].pattern_foot[3] = 0x20;
    DispPattern[29].pattern_foot[4] = 0x00;
    DispPattern[29].pattern_foot[5] = 0x03;
    DispPattern[29].pattern_foot[6] = 0x0E;
    DispPattern[29].pattern_foot[7] = 0x38;
    DispPattern[29].pattern_foot[8] = 0x0E;
    DispPattern[29].pattern_foot[9] = 0x03;
    DispPattern[29].pattern_foot[10] = 0x00;
    DispPattern[29].pattern_foot[11] = 0x20;
    DispPattern[29].pattern_foot[12] = 0x3F;
    DispPattern[29].pattern_foot[13] = 0x20;
    DispPattern[29].pattern_foot[14] = 0x20;
    DispPattern[29].pattern_foot[15] = 0x00;

    DispPattern[30].dispChar ='N';
    DispPattern[30].pattern_head[0] = 0x00;
    DispPattern[30].pattern_head[1] = 0x02;
    DispPattern[30].pattern_head[2] = 0x02;
    DispPattern[30].pattern_head[3] = 0xFE;
    DispPattern[30].pattern_head[4] = 0x06;
    DispPattern[30].pattern_head[5] = 0x1C;
    DispPattern[30].pattern_head[6] = 0x30;
    DispPattern[30].pattern_head[7] = 0xE0;
    DispPattern[30].pattern_head[8] = 0x80;
    DispPattern[30].pattern_head[9] = 0x00;
    DispPattern[30].pattern_head[10] = 0x02;
    DispPattern[30].pattern_head[11] = 0x02;
    DispPattern[30].pattern_head[12] = 0xFE;
    DispPattern[30].pattern_head[13] = 0x02;
    DispPattern[30].pattern_head[14] = 0x02;
    DispPattern[30].pattern_head[15] = 0x00;
    DispPattern[30].pattern_foot[0] = 0x00;
    DispPattern[30].pattern_foot[1] = 0x20;
    DispPattern[30].pattern_foot[2] = 0x20;
    DispPattern[30].pattern_foot[3] = 0x3F;
    DispPattern[30].pattern_foot[4] = 0x20;
    DispPattern[30].pattern_foot[5] = 0x00;
    DispPattern[30].pattern_foot[6] = 0x00;
    DispPattern[30].pattern_foot[7] = 0x00;
    DispPattern[30].pattern_foot[8] = 0x01;
    DispPattern[30].pattern_foot[9] = 0x07;
    DispPattern[30].pattern_foot[10] = 0x0C;
    DispPattern[30].pattern_foot[11] = 0x18;
    DispPattern[30].pattern_foot[12] = 0x3F;
    DispPattern[30].pattern_foot[13] = 0x00;
    DispPattern[30].pattern_foot[14] = 0x00;
    DispPattern[30].pattern_foot[15] = 0x00;

    DispPattern[31].dispChar ='O';
    DispPattern[31].pattern_head[0] = 0x00;
    DispPattern[31].pattern_head[1] = 0x00;
    DispPattern[31].pattern_head[2] = 0xE0;
    DispPattern[31].pattern_head[3] = 0x38;
    DispPattern[31].pattern_head[4] = 0x0C;
    DispPattern[31].pattern_head[5] = 0x04;
    DispPattern[31].pattern_head[6] = 0x02;
    DispPattern[31].pattern_head[7] = 0x02;
    DispPattern[31].pattern_head[8] = 0x02;
    DispPattern[31].pattern_head[9] = 0x02;
    DispPattern[31].pattern_head[10] = 0x04;
    DispPattern[31].pattern_head[11] = 0x0C;
    DispPattern[31].pattern_head[12] = 0x38;
    DispPattern[31].pattern_head[13] = 0xE0;
    DispPattern[31].pattern_head[14] = 0x00;
    DispPattern[31].pattern_head[15] = 0x00;
    DispPattern[31].pattern_foot[0] = 0x00;
    DispPattern[31].pattern_foot[1] = 0x00;
    DispPattern[31].pattern_foot[2] = 0x03;
    DispPattern[31].pattern_foot[3] = 0x0E;
    DispPattern[31].pattern_foot[4] = 0x18;
    DispPattern[31].pattern_foot[5] = 0x10;
    DispPattern[31].pattern_foot[6] = 0x20;
    DispPattern[31].pattern_foot[7] = 0x20;
    DispPattern[31].pattern_foot[8] = 0x20;
    DispPattern[31].pattern_foot[9] = 0x20;
    DispPattern[31].pattern_foot[10] = 0x10;
    DispPattern[31].pattern_foot[11] = 0x18;
    DispPattern[31].pattern_foot[12] = 0x0E;
    DispPattern[31].pattern_foot[13] = 0x03;
    DispPattern[31].pattern_foot[14] = 0x00;
    DispPattern[31].pattern_foot[15] = 0x00;

    DispPattern[32].dispChar ='P';
    DispPattern[32].pattern_head[0] = 0x00;
    DispPattern[32].pattern_head[1] = 0x00;
    DispPattern[32].pattern_head[2] = 0x00;
    DispPattern[32].pattern_head[3] = 0x02;
    DispPattern[32].pattern_head[4] = 0x02;
    DispPattern[32].pattern_head[5] = 0xFE;
    DispPattern[32].pattern_head[6] = 0x82;
    DispPattern[32].pattern_head[7] = 0x82;
    DispPattern[32].pattern_head[8] = 0x82;
    DispPattern[32].pattern_head[9] = 0x82;
    DispPattern[32].pattern_head[10] = 0x82;
    DispPattern[32].pattern_head[11] = 0x82;
    DispPattern[32].pattern_head[12] = 0x44;
    DispPattern[32].pattern_head[13] = 0x38;
    DispPattern[32].pattern_head[14] = 0x00;
    DispPattern[32].pattern_head[15] = 0x00;
    DispPattern[32].pattern_foot[0] = 0x00;
    DispPattern[32].pattern_foot[1] = 0x00;
    DispPattern[32].pattern_foot[2] = 0x00;
    DispPattern[32].pattern_foot[3] = 0x20;
    DispPattern[32].pattern_foot[4] = 0x20;
    DispPattern[32].pattern_foot[5] = 0x3F;
    DispPattern[32].pattern_foot[6] = 0x20;
    DispPattern[32].pattern_foot[7] = 0x20;
    DispPattern[32].pattern_foot[8] = 0x00;
    DispPattern[32].pattern_foot[9] = 0x00;
    DispPattern[32].pattern_foot[10] = 0x00;
    DispPattern[32].pattern_foot[11] = 0x00;
    DispPattern[32].pattern_foot[12] = 0x00;
    DispPattern[32].pattern_foot[13] = 0x00;
    DispPattern[32].pattern_foot[14] = 0x00;
    DispPattern[32].pattern_foot[15] = 0x00;

    DispPattern[33].dispChar ='Q';
    DispPattern[33].pattern_head[0] = 0x00;
    DispPattern[33].pattern_head[1] = 0x00;
    DispPattern[33].pattern_head[2] = 0xE0;
    DispPattern[33].pattern_head[3] = 0x38;
    DispPattern[33].pattern_head[4] = 0x0C;
    DispPattern[33].pattern_head[5] = 0x04;
    DispPattern[33].pattern_head[6] = 0x02;
    DispPattern[33].pattern_head[7] = 0x02;
    DispPattern[33].pattern_head[8] = 0x02;
    DispPattern[33].pattern_head[9] = 0x02;
    DispPattern[33].pattern_head[10] = 0x04;
    DispPattern[33].pattern_head[11] = 0x0C;
    DispPattern[33].pattern_head[12] = 0x38;
    DispPattern[33].pattern_head[13] = 0xE0;
    DispPattern[33].pattern_head[14] = 0x00;
    DispPattern[33].pattern_head[15] = 0x00;
    DispPattern[33].pattern_foot[0] = 0x00;
    DispPattern[33].pattern_foot[1] = 0x00;
    DispPattern[33].pattern_foot[2] = 0x03;
    DispPattern[33].pattern_foot[3] = 0x0E;
    DispPattern[33].pattern_foot[4] = 0x18;
    DispPattern[33].pattern_foot[5] = 0x10;
    DispPattern[33].pattern_foot[6] = 0x28;
    DispPattern[33].pattern_foot[7] = 0x24;
    DispPattern[33].pattern_foot[8] = 0x24;
    DispPattern[33].pattern_foot[9] = 0x28;
    DispPattern[33].pattern_foot[10] = 0x10;
    DispPattern[33].pattern_foot[11] = 0x38;
    DispPattern[33].pattern_foot[12] = 0x4E;
    DispPattern[33].pattern_foot[13] = 0x43;
    DispPattern[33].pattern_foot[14] = 0x20;
    DispPattern[33].pattern_foot[15] = 0x00;

    DispPattern[34].dispChar ='R';
    DispPattern[34].pattern_head[0] = 0x00;
    DispPattern[34].pattern_head[1] = 0x00;
    DispPattern[34].pattern_head[2] = 0x02;
    DispPattern[34].pattern_head[3] = 0x02;
    DispPattern[34].pattern_head[4] = 0xFE;
    DispPattern[34].pattern_head[5] = 0x82;
    DispPattern[34].pattern_head[6] = 0x82;
    DispPattern[34].pattern_head[7] = 0x82;
    DispPattern[34].pattern_head[8] = 0x82;
    DispPattern[34].pattern_head[9] = 0x82;
    DispPattern[34].pattern_head[10] = 0x82;
    DispPattern[34].pattern_head[11] = 0x44;
    DispPattern[34].pattern_head[12] = 0x38;
    DispPattern[34].pattern_head[13] = 0x00;
    DispPattern[34].pattern_head[14] = 0x00;
    DispPattern[34].pattern_head[15] = 0x00;
    DispPattern[34].pattern_foot[0] = 0x00;
    DispPattern[34].pattern_foot[1] = 0x00;
    DispPattern[34].pattern_foot[2] = 0x20;
    DispPattern[34].pattern_foot[3] = 0x20;
    DispPattern[34].pattern_foot[4] = 0x3F;
    DispPattern[34].pattern_foot[5] = 0x20;
    DispPattern[34].pattern_foot[6] = 0x20;
    DispPattern[34].pattern_foot[7] = 0x00;
    DispPattern[34].pattern_foot[8] = 0x00;
    DispPattern[34].pattern_foot[9] = 0x00;
    DispPattern[34].pattern_foot[10] = 0x23;
    DispPattern[34].pattern_foot[11] = 0x2E;
    DispPattern[34].pattern_foot[12] = 0x38;
    DispPattern[34].pattern_foot[13] = 0x20;
    DispPattern[34].pattern_foot[14] = 0x20;
    DispPattern[34].pattern_foot[15] = 0x00;

    DispPattern[35].dispChar ='S';
    DispPattern[35].pattern_head[0] = 0x00;
    DispPattern[35].pattern_head[1] = 0x00;
    DispPattern[35].pattern_head[2] = 0x00;
    DispPattern[35].pattern_head[3] = 0x38;
    DispPattern[35].pattern_head[4] = 0x44;
    DispPattern[35].pattern_head[5] = 0x42;
    DispPattern[35].pattern_head[6] = 0x82;
    DispPattern[35].pattern_head[7] = 0x82;
    DispPattern[35].pattern_head[8] = 0x82;
    DispPattern[35].pattern_head[9] = 0x82;
    DispPattern[35].pattern_head[10] = 0x04;
    DispPattern[35].pattern_head[11] = 0x08;
    DispPattern[35].pattern_head[12] = 0x3E;
    DispPattern[35].pattern_head[13] = 0x00;
    DispPattern[35].pattern_head[14] = 0x00;
    DispPattern[35].pattern_head[15] = 0x00;
    DispPattern[35].pattern_foot[0] = 0x00;
    DispPattern[35].pattern_foot[1] = 0x00;
    DispPattern[35].pattern_foot[2] = 0x00;
    DispPattern[35].pattern_foot[3] = 0x3E;
    DispPattern[35].pattern_foot[4] = 0x08;
    DispPattern[35].pattern_foot[5] = 0x10;
    DispPattern[35].pattern_foot[6] = 0x20;
    DispPattern[35].pattern_foot[7] = 0x20;
    DispPattern[35].pattern_foot[8] = 0x20;
    DispPattern[35].pattern_foot[9] = 0x20;
    DispPattern[35].pattern_foot[10] = 0x21;
    DispPattern[35].pattern_foot[11] = 0x11;
    DispPattern[35].pattern_foot[12] = 0x0E;
    DispPattern[35].pattern_foot[13] = 0x00;
    DispPattern[35].pattern_foot[14] = 0x00;
    DispPattern[35].pattern_foot[15] = 0x00;

    DispPattern[36].dispChar ='T';
    DispPattern[36].pattern_head[0] = 0x00;
    DispPattern[36].pattern_head[1] = 0x0E;
    DispPattern[36].pattern_head[2] = 0x02;
    DispPattern[36].pattern_head[3] = 0x02;
    DispPattern[36].pattern_head[4] = 0x02;
    DispPattern[36].pattern_head[5] = 0x02;
    DispPattern[36].pattern_head[6] = 0x02;
    DispPattern[36].pattern_head[7] = 0xFE;
    DispPattern[36].pattern_head[8] = 0x02;
    DispPattern[36].pattern_head[9] = 0x02;
    DispPattern[36].pattern_head[10] = 0x02;
    DispPattern[36].pattern_head[11] = 0x02;
    DispPattern[36].pattern_head[12] = 0x02;
    DispPattern[36].pattern_head[13] = 0x0E;
    DispPattern[36].pattern_head[14] = 0x00;
    DispPattern[36].pattern_head[15] = 0x00;
    DispPattern[36].pattern_foot[0] = 0x00;
    DispPattern[36].pattern_foot[1] = 0x00;
    DispPattern[36].pattern_foot[2] = 0x00;
    DispPattern[36].pattern_foot[3] = 0x00;
    DispPattern[36].pattern_foot[4] = 0x00;
    DispPattern[36].pattern_foot[5] = 0x20;
    DispPattern[36].pattern_foot[6] = 0x20;
    DispPattern[36].pattern_foot[7] = 0x3F;
    DispPattern[36].pattern_foot[8] = 0x20;
    DispPattern[36].pattern_foot[9] = 0x20;
    DispPattern[36].pattern_foot[10] = 0x00;
    DispPattern[36].pattern_foot[11] = 0x00;
    DispPattern[36].pattern_foot[12] = 0x00;
    DispPattern[36].pattern_foot[13] = 0x00;
    DispPattern[36].pattern_foot[14] = 0x00;
    DispPattern[36].pattern_foot[15] = 0x00;

    DispPattern[37].dispChar ='U';
    DispPattern[37].pattern_head[0] = 0x00;
    DispPattern[37].pattern_head[1] = 0x02;
    DispPattern[37].pattern_head[2] = 0x02;
    DispPattern[37].pattern_head[3] = 0xFE;
    DispPattern[37].pattern_head[4] = 0x02;
    DispPattern[37].pattern_head[5] = 0x02;
    DispPattern[37].pattern_head[6] = 0x00;
    DispPattern[37].pattern_head[7] = 0x00;
    DispPattern[37].pattern_head[8] = 0x00;
    DispPattern[37].pattern_head[9] = 0x00;
    DispPattern[37].pattern_head[10] = 0x02;
    DispPattern[37].pattern_head[11] = 0x02;
    DispPattern[37].pattern_head[12] = 0xFE;
    DispPattern[37].pattern_head[13] = 0x02;
    DispPattern[37].pattern_head[14] = 0x02;
    DispPattern[37].pattern_head[15] = 0x00;
    DispPattern[37].pattern_foot[0] = 0x00;
    DispPattern[37].pattern_foot[1] = 0x00;
    DispPattern[37].pattern_foot[2] = 0x00;
    DispPattern[37].pattern_foot[3] = 0x0F;
    DispPattern[37].pattern_foot[4] = 0x18;
    DispPattern[37].pattern_foot[5] = 0x10;
    DispPattern[37].pattern_foot[6] = 0x20;
    DispPattern[37].pattern_foot[7] = 0x20;
    DispPattern[37].pattern_foot[8] = 0x20;
    DispPattern[37].pattern_foot[9] = 0x20;
    DispPattern[37].pattern_foot[10] = 0x10;
    DispPattern[37].pattern_foot[11] = 0x18;
    DispPattern[37].pattern_foot[12] = 0x0F;
    DispPattern[37].pattern_foot[13] = 0x00;
    DispPattern[37].pattern_foot[14] = 0x00;
    DispPattern[37].pattern_foot[15] = 0x00;

    DispPattern[38].dispChar ='V';
    DispPattern[38].pattern_head[0] = 0x00;
    DispPattern[38].pattern_head[1] = 0x02;
    DispPattern[38].pattern_head[2] = 0x02;
    DispPattern[38].pattern_head[3] = 0x0E;
    DispPattern[38].pattern_head[4] = 0x7A;
    DispPattern[38].pattern_head[5] = 0xC2;
    DispPattern[38].pattern_head[6] = 0x00;
    DispPattern[38].pattern_head[7] = 0x00;
    DispPattern[38].pattern_head[8] = 0x00;
    DispPattern[38].pattern_head[9] = 0x00;
    DispPattern[38].pattern_head[10] = 0xE2;
    DispPattern[38].pattern_head[11] = 0x3A;
    DispPattern[38].pattern_head[12] = 0x0E;
    DispPattern[38].pattern_head[13] = 0x02;
    DispPattern[38].pattern_head[14] = 0x02;
    DispPattern[38].pattern_head[15] = 0x00;
    DispPattern[38].pattern_foot[0] = 0x00;
    DispPattern[38].pattern_foot[1] = 0x00;
    DispPattern[38].pattern_foot[2] = 0x00;
    DispPattern[38].pattern_foot[3] = 0x00;
    DispPattern[38].pattern_foot[4] = 0x00;
    DispPattern[38].pattern_foot[5] = 0x03;
    DispPattern[38].pattern_foot[6] = 0x1E;
    DispPattern[38].pattern_foot[7] = 0x30;
    DispPattern[38].pattern_foot[8] = 0x1C;
    DispPattern[38].pattern_foot[9] = 0x07;
    DispPattern[38].pattern_foot[10] = 0x01;
    DispPattern[38].pattern_foot[11] = 0x00;
    DispPattern[38].pattern_foot[12] = 0x00;
    DispPattern[38].pattern_foot[13] = 0x00;
    DispPattern[38].pattern_foot[14] = 0x00;
    DispPattern[38].pattern_foot[15] = 0x00;

    DispPattern[39].dispChar ='W';
    DispPattern[39].pattern_head[0] = 0x02;
    DispPattern[39].pattern_head[1] = 0x02;
    DispPattern[39].pattern_head[2] = 0x3E;
    DispPattern[39].pattern_head[3] = 0xE2;
    DispPattern[39].pattern_head[4] = 0x02;
    DispPattern[39].pattern_head[5] = 0x80;
    DispPattern[39].pattern_head[6] = 0xF0;
    DispPattern[39].pattern_head[7] = 0x1E;
    DispPattern[39].pattern_head[8] = 0x1E;
    DispPattern[39].pattern_head[9] = 0xF0;
    DispPattern[39].pattern_head[10] = 0x80;
    DispPattern[39].pattern_head[11] = 0x02;
    DispPattern[39].pattern_head[12] = 0xE2;
    DispPattern[39].pattern_head[13] = 0x3E;
    DispPattern[39].pattern_head[14] = 0x02;
    DispPattern[39].pattern_head[15] = 0x02;
    DispPattern[39].pattern_foot[0] = 0x00;
    DispPattern[39].pattern_foot[1] = 0x00;
    DispPattern[39].pattern_foot[2] = 0x00;
    DispPattern[39].pattern_foot[3] = 0x07;
    DispPattern[39].pattern_foot[4] = 0x3C;
    DispPattern[39].pattern_foot[5] = 0x0F;
    DispPattern[39].pattern_foot[6] = 0x00;
    DispPattern[39].pattern_foot[7] = 0x00;
    DispPattern[39].pattern_foot[8] = 0x00;
    DispPattern[39].pattern_foot[9] = 0x00;
    DispPattern[39].pattern_foot[10] = 0x0F;
    DispPattern[39].pattern_foot[11] = 0x3C;
    DispPattern[39].pattern_foot[12] = 0x07;
    DispPattern[39].pattern_foot[13] = 0x00;
    DispPattern[39].pattern_foot[14] = 0x00;
    DispPattern[39].pattern_foot[15] = 0x00;

    DispPattern[40].dispChar ='X';
    DispPattern[40].pattern_head[0] = 0x00;
    DispPattern[40].pattern_head[1] = 0x02;
    DispPattern[40].pattern_head[2] = 0x02;
    DispPattern[40].pattern_head[3] = 0x0E;
    DispPattern[40].pattern_head[4] = 0x1A;
    DispPattern[40].pattern_head[5] = 0x32;
    DispPattern[40].pattern_head[6] = 0x60;
    DispPattern[40].pattern_head[7] = 0xC0;
    DispPattern[40].pattern_head[8] = 0x60;
    DispPattern[40].pattern_head[9] = 0x32;
    DispPattern[40].pattern_head[10] = 0x1A;
    DispPattern[40].pattern_head[11] = 0x0E;
    DispPattern[40].pattern_head[12] = 0x02;
    DispPattern[40].pattern_head[13] = 0x02;
    DispPattern[40].pattern_head[14] = 0x00;
    DispPattern[40].pattern_head[15] = 0x00;
    DispPattern[40].pattern_foot[0] = 0x00;
    DispPattern[40].pattern_foot[1] = 0x20;
    DispPattern[40].pattern_foot[2] = 0x20;
    DispPattern[40].pattern_foot[3] = 0x38;
    DispPattern[40].pattern_foot[4] = 0x2C;
    DispPattern[40].pattern_foot[5] = 0x06;
    DispPattern[40].pattern_foot[6] = 0x03;
    DispPattern[40].pattern_foot[7] = 0x01;
    DispPattern[40].pattern_foot[8] = 0x03;
    DispPattern[40].pattern_foot[9] = 0x26;
    DispPattern[40].pattern_foot[10] = 0x2C;
    DispPattern[40].pattern_foot[11] = 0x38;
    DispPattern[40].pattern_foot[12] = 0x20;
    DispPattern[40].pattern_foot[13] = 0x20;
    DispPattern[40].pattern_foot[14] = 0x00;
    DispPattern[40].pattern_foot[15] = 0x00;

    DispPattern[41].dispChar ='Y';
    DispPattern[41].pattern_head[0] = 0x00;
    DispPattern[41].pattern_head[1] = 0x02;
    DispPattern[41].pattern_head[2] = 0x02;
    DispPattern[41].pattern_head[3] = 0x0E;
    DispPattern[41].pattern_head[4] = 0x3A;
    DispPattern[41].pattern_head[5] = 0xE2;
    DispPattern[41].pattern_head[6] = 0x80;
    DispPattern[41].pattern_head[7] = 0x00;
    DispPattern[41].pattern_head[8] = 0x80;
    DispPattern[41].pattern_head[9] = 0xE2;
    DispPattern[41].pattern_head[10] = 0x3A;
    DispPattern[41].pattern_head[11] = 0x0E;
    DispPattern[41].pattern_head[12] = 0x02;
    DispPattern[41].pattern_head[13] = 0x02;
    DispPattern[41].pattern_head[14] = 0x00;
    DispPattern[41].pattern_head[15] = 0x00;
    DispPattern[41].pattern_foot[0] = 0x00;
    DispPattern[41].pattern_foot[1] = 0x00;
    DispPattern[41].pattern_foot[2] = 0x00;
    DispPattern[41].pattern_foot[3] = 0x00;
    DispPattern[41].pattern_foot[4] = 0x00;
    DispPattern[41].pattern_foot[5] = 0x20;
    DispPattern[41].pattern_foot[6] = 0x21;
    DispPattern[41].pattern_foot[7] = 0x3F;
    DispPattern[41].pattern_foot[8] = 0x21;
    DispPattern[41].pattern_foot[9] = 0x20;
    DispPattern[41].pattern_foot[10] = 0x00;
    DispPattern[41].pattern_foot[11] = 0x00;
    DispPattern[41].pattern_foot[12] = 0x00;
    DispPattern[41].pattern_foot[13] = 0x00;
    DispPattern[41].pattern_foot[14] = 0x00;
    DispPattern[41].pattern_foot[15] = 0x00;

    DispPattern[42].dispChar ='Z';
    DispPattern[42].pattern_head[0] = 0x00;
    DispPattern[42].pattern_head[1] = 0x00;
    DispPattern[42].pattern_head[2] = 0x00;
    DispPattern[42].pattern_head[3] = 0x0E;
    DispPattern[42].pattern_head[4] = 0x02;
    DispPattern[42].pattern_head[5] = 0x02;
    DispPattern[42].pattern_head[6] = 0x02;
    DispPattern[42].pattern_head[7] = 0xC2;
    DispPattern[42].pattern_head[8] = 0x62;
    DispPattern[42].pattern_head[9] = 0x3A;
    DispPattern[42].pattern_head[10] = 0x0E;
    DispPattern[42].pattern_head[11] = 0x06;
    DispPattern[42].pattern_head[12] = 0x02;
    DispPattern[42].pattern_head[13] = 0x00;
    DispPattern[42].pattern_head[14] = 0x00;
    DispPattern[42].pattern_head[15] = 0x00;
    DispPattern[42].pattern_foot[0] = 0x00;
    DispPattern[42].pattern_foot[1] = 0x00;
    DispPattern[42].pattern_foot[2] = 0x20;
    DispPattern[42].pattern_foot[3] = 0x30;
    DispPattern[42].pattern_foot[4] = 0x38;
    DispPattern[42].pattern_foot[5] = 0x2E;
    DispPattern[42].pattern_foot[6] = 0x23;
    DispPattern[42].pattern_foot[7] = 0x21;
    DispPattern[42].pattern_foot[8] = 0x20;
    DispPattern[42].pattern_foot[9] = 0x20;
    DispPattern[42].pattern_foot[10] = 0x20;
    DispPattern[42].pattern_foot[11] = 0x20;
    DispPattern[42].pattern_foot[12] = 0x38;
    DispPattern[42].pattern_foot[13] = 0x00;
    DispPattern[42].pattern_foot[14] = 0x00;
    DispPattern[42].pattern_foot[15] = 0x00;
}

void SetCharPattern_alphabet() 
{
	DispPattern[49].dispChar ='a';
	DispPattern[49].pattern_head[0] = 0x00;
	DispPattern[49].pattern_head[1] = 0x00;
	DispPattern[49].pattern_head[2] = 0x00;
	DispPattern[49].pattern_head[3] = 0x00;
	DispPattern[49].pattern_head[4] = 0x00;
	DispPattern[49].pattern_head[5] = 0x40;
	DispPattern[49].pattern_head[6] = 0x40;
	DispPattern[49].pattern_head[7] = 0x40;
	DispPattern[49].pattern_head[8] = 0x40;
	DispPattern[49].pattern_head[9] = 0x40;
	DispPattern[49].pattern_head[10] = 0x80;
	DispPattern[49].pattern_head[11] = 0x00;
	DispPattern[49].pattern_head[12] = 0x00;
	DispPattern[49].pattern_head[13] = 0x00;
	DispPattern[49].pattern_head[14] = 0x00;
	DispPattern[49].pattern_head[15] = 0x00;
	DispPattern[49].pattern_foot[0] = 0x00;
	DispPattern[49].pattern_foot[1] = 0x00;
	DispPattern[49].pattern_foot[2] = 0x00;
	DispPattern[49].pattern_foot[3] = 0x00;
	DispPattern[49].pattern_foot[4] = 0x1C;
	DispPattern[49].pattern_foot[5] = 0x22;
	DispPattern[49].pattern_foot[6] = 0x22;
	DispPattern[49].pattern_foot[7] = 0x22;
	DispPattern[49].pattern_foot[8] = 0x22;
	DispPattern[49].pattern_foot[9] = 0x12;
	DispPattern[49].pattern_foot[10] = 0x1F;
	DispPattern[49].pattern_foot[11] = 0x20;
	DispPattern[49].pattern_foot[12] = 0x30;
	DispPattern[49].pattern_foot[13] = 0x00;
	DispPattern[49].pattern_foot[14] = 0x00;
	DispPattern[49].pattern_foot[15] = 0x00;

	DispPattern[50].dispChar ='b';
	DispPattern[50].pattern_head[0] = 0x00;
	DispPattern[50].pattern_head[1] = 0x00;
	DispPattern[50].pattern_head[2] = 0x00;
	DispPattern[50].pattern_head[3] = 0x02;
	DispPattern[50].pattern_head[4] = 0x02;
	DispPattern[50].pattern_head[5] = 0xFE;
	DispPattern[50].pattern_head[6] = 0x80;
	DispPattern[50].pattern_head[7] = 0x40;
	DispPattern[50].pattern_head[8] = 0x40;
	DispPattern[50].pattern_head[9] = 0x40;
	DispPattern[50].pattern_head[10] = 0x40;
	DispPattern[50].pattern_head[11] = 0x80;
	DispPattern[50].pattern_head[12] = 0x00;
	DispPattern[50].pattern_head[13] = 0x00;
	DispPattern[50].pattern_head[14] = 0x00;
	DispPattern[50].pattern_head[15] = 0x00;
	DispPattern[50].pattern_foot[0] = 0x00;
	DispPattern[50].pattern_foot[1] = 0x00;
	DispPattern[50].pattern_foot[2] = 0x00;
	DispPattern[50].pattern_foot[3] = 0x00;
	DispPattern[50].pattern_foot[4] = 0x00;
	DispPattern[50].pattern_foot[5] = 0x3F;
	DispPattern[50].pattern_foot[6] = 0x10;
	DispPattern[50].pattern_foot[7] = 0x20;
	DispPattern[50].pattern_foot[8] = 0x20;
	DispPattern[50].pattern_foot[9] = 0x20;
	DispPattern[50].pattern_foot[10] = 0x20;
	DispPattern[50].pattern_foot[11] = 0x10;
	DispPattern[50].pattern_foot[12] = 0x0F;
	DispPattern[50].pattern_foot[13] = 0x00;
	DispPattern[50].pattern_foot[14] = 0x00;
	DispPattern[50].pattern_foot[15] = 0x00;

	DispPattern[51].dispChar ='c';
	DispPattern[51].pattern_head[0] = 0x00;
	DispPattern[51].pattern_head[1] = 0x00;
	DispPattern[51].pattern_head[2] = 0x00;
	DispPattern[51].pattern_head[3] = 0x00;
	DispPattern[51].pattern_head[4] = 0x00;
	DispPattern[51].pattern_head[5] = 0x80;
	DispPattern[51].pattern_head[6] = 0x40;
	DispPattern[51].pattern_head[7] = 0x40;
	DispPattern[51].pattern_head[8] = 0x40;
	DispPattern[51].pattern_head[9] = 0x40;
	DispPattern[51].pattern_head[10] = 0x40;
	DispPattern[51].pattern_head[11] = 0x80;
	DispPattern[51].pattern_head[12] = 0x00;
	DispPattern[51].pattern_head[13] = 0x00;
	DispPattern[51].pattern_head[14] = 0x00;
	DispPattern[51].pattern_head[15] = 0x00;
	DispPattern[51].pattern_foot[0] = 0x00;
	DispPattern[51].pattern_foot[1] = 0x00;
	DispPattern[51].pattern_foot[2] = 0x00;
	DispPattern[51].pattern_foot[3] = 0x00;
	DispPattern[51].pattern_foot[4] = 0x0F;
	DispPattern[51].pattern_foot[5] = 0x10;
	DispPattern[51].pattern_foot[6] = 0x20;
	DispPattern[51].pattern_foot[7] = 0x20;
	DispPattern[51].pattern_foot[8] = 0x20;
	DispPattern[51].pattern_foot[9] = 0x20;
	DispPattern[51].pattern_foot[10] = 0x20;
	DispPattern[51].pattern_foot[11] = 0x10;
	DispPattern[51].pattern_foot[12] = 0x00;
	DispPattern[51].pattern_foot[13] = 0x00;
	DispPattern[51].pattern_foot[14] = 0x00;
	DispPattern[51].pattern_foot[15] = 0x00;

	DispPattern[52].dispChar ='d';
	DispPattern[52].pattern_head[0] = 0x00;
	DispPattern[52].pattern_head[1] = 0x00;
	DispPattern[52].pattern_head[2] = 0x00;
	DispPattern[52].pattern_head[3] = 0x00;
	DispPattern[52].pattern_head[4] = 0x80;
	DispPattern[52].pattern_head[5] = 0x40;
	DispPattern[52].pattern_head[6] = 0x40;
	DispPattern[52].pattern_head[7] = 0x40;
	DispPattern[52].pattern_head[8] = 0x42;
	DispPattern[52].pattern_head[9] = 0x82;
	DispPattern[52].pattern_head[10] = 0xFE;
	DispPattern[52].pattern_head[11] = 0x00;
	DispPattern[52].pattern_head[12] = 0x00;
	DispPattern[52].pattern_head[13] = 0x00;
	DispPattern[52].pattern_head[14] = 0x00;
	DispPattern[52].pattern_head[15] = 0x00;
	DispPattern[52].pattern_foot[0] = 0x00;
	DispPattern[52].pattern_foot[1] = 0x00;
	DispPattern[52].pattern_foot[2] = 0x00;
	DispPattern[52].pattern_foot[3] = 0x0F;
	DispPattern[52].pattern_foot[4] = 0x10;
	DispPattern[52].pattern_foot[5] = 0x20;
	DispPattern[52].pattern_foot[6] = 0x20;
	DispPattern[52].pattern_foot[7] = 0x20;
	DispPattern[52].pattern_foot[8] = 0x20;
	DispPattern[52].pattern_foot[9] = 0x10;
	DispPattern[52].pattern_foot[10] = 0x3F;
	DispPattern[52].pattern_foot[11] = 0x20;
	DispPattern[52].pattern_foot[12] = 0x20;
	DispPattern[52].pattern_foot[13] = 0x00;
	DispPattern[52].pattern_foot[14] = 0x00;
	DispPattern[52].pattern_foot[15] = 0x00;

	DispPattern[53].dispChar ='e';
	DispPattern[53].pattern_head[0] = 0x00;
	DispPattern[53].pattern_head[1] = 0x00;
	DispPattern[53].pattern_head[2] = 0x00;
	DispPattern[53].pattern_head[3] = 0x00;
	DispPattern[53].pattern_head[4] = 0x00;
	DispPattern[53].pattern_head[5] = 0x80;
	DispPattern[53].pattern_head[6] = 0x40;
	DispPattern[53].pattern_head[7] = 0x40;
	DispPattern[53].pattern_head[8] = 0x40;
	DispPattern[53].pattern_head[9] = 0x40;
	DispPattern[53].pattern_head[10] = 0x80;
	DispPattern[53].pattern_head[11] = 0x00;
	DispPattern[53].pattern_head[12] = 0x00;
	DispPattern[53].pattern_head[13] = 0x00;
	DispPattern[53].pattern_head[14] = 0x00;
	DispPattern[53].pattern_head[15] = 0x00;
	DispPattern[53].pattern_foot[0] = 0x00;
	DispPattern[53].pattern_foot[1] = 0x00;
	DispPattern[53].pattern_foot[2] = 0x00;
	DispPattern[53].pattern_foot[3] = 0x00;
	DispPattern[53].pattern_foot[4] = 0x0F;
	DispPattern[53].pattern_foot[5] = 0x12;
	DispPattern[53].pattern_foot[6] = 0x22;
	DispPattern[53].pattern_foot[7] = 0x22;
	DispPattern[53].pattern_foot[8] = 0x22;
	DispPattern[53].pattern_foot[9] = 0x22;
	DispPattern[53].pattern_foot[10] = 0x22;
	DispPattern[53].pattern_foot[11] = 0x13;
	DispPattern[53].pattern_foot[12] = 0x00;
	DispPattern[53].pattern_foot[13] = 0x00;
	DispPattern[53].pattern_foot[14] = 0x00;
	DispPattern[53].pattern_foot[15] = 0x00;

	DispPattern[54].dispChar ='f';
	DispPattern[54].pattern_head[0] = 0x00;
	DispPattern[54].pattern_head[1] = 0x00;
	DispPattern[54].pattern_head[2] = 0x00;
	DispPattern[54].pattern_head[3] = 0x00;
	DispPattern[54].pattern_head[4] = 0x40;
	DispPattern[54].pattern_head[5] = 0x40;
	DispPattern[54].pattern_head[6] = 0x40;
	DispPattern[54].pattern_head[7] = 0xF8;
	DispPattern[54].pattern_head[8] = 0x44;
	DispPattern[54].pattern_head[9] = 0x42;
	DispPattern[54].pattern_head[10] = 0x42;
	DispPattern[54].pattern_head[11] = 0x02;
	DispPattern[54].pattern_head[12] = 0x06;
	DispPattern[54].pattern_head[13] = 0x00;
	DispPattern[54].pattern_head[14] = 0x00;
	DispPattern[54].pattern_head[15] = 0x00;
	DispPattern[54].pattern_foot[0] = 0x00;
	DispPattern[54].pattern_foot[1] = 0x00;
	DispPattern[54].pattern_foot[2] = 0x00;
	DispPattern[54].pattern_foot[3] = 0x00;
	DispPattern[54].pattern_foot[4] = 0x00;
	DispPattern[54].pattern_foot[5] = 0x20;
	DispPattern[54].pattern_foot[6] = 0x20;
	DispPattern[54].pattern_foot[7] = 0x3F;
	DispPattern[54].pattern_foot[8] = 0x20;
	DispPattern[54].pattern_foot[9] = 0x20;
	DispPattern[54].pattern_foot[10] = 0x00;
	DispPattern[54].pattern_foot[11] = 0x00;
	DispPattern[54].pattern_foot[12] = 0x00;
	DispPattern[54].pattern_foot[13] = 0x00;
	DispPattern[54].pattern_foot[14] = 0x00;
	DispPattern[54].pattern_foot[15] = 0x00;

	DispPattern[55].dispChar ='g';
	DispPattern[55].pattern_head[0] = 0x00;
	DispPattern[55].pattern_head[1] = 0x00;
	DispPattern[55].pattern_head[2] = 0x00;
	DispPattern[55].pattern_head[3] = 0x00;
	DispPattern[55].pattern_head[4] = 0x80;
	DispPattern[55].pattern_head[5] = 0x40;
	DispPattern[55].pattern_head[6] = 0x40;
	DispPattern[55].pattern_head[7] = 0x40;
	DispPattern[55].pattern_head[8] = 0x40;
	DispPattern[55].pattern_head[9] = 0x40;
	DispPattern[55].pattern_head[10] = 0x80;
	DispPattern[55].pattern_head[11] = 0x40;
	DispPattern[55].pattern_head[12] = 0x40;
	DispPattern[55].pattern_head[13] = 0x00;
	DispPattern[55].pattern_head[14] = 0x00;
	DispPattern[55].pattern_head[15] = 0x00;
	DispPattern[55].pattern_foot[0] = 0x00;
	DispPattern[55].pattern_foot[1] = 0x00;
	DispPattern[55].pattern_foot[2] = 0x00;
	DispPattern[55].pattern_foot[3] = 0x60;
	DispPattern[55].pattern_foot[4] = 0x9B;
	DispPattern[55].pattern_foot[5] = 0x94;
	DispPattern[55].pattern_foot[6] = 0x94;
	DispPattern[55].pattern_foot[7] = 0x94;
	DispPattern[55].pattern_foot[8] = 0x94;
	DispPattern[55].pattern_foot[9] = 0x94;
	DispPattern[55].pattern_foot[10] = 0x93;
	DispPattern[55].pattern_foot[11] = 0x90;
	DispPattern[55].pattern_foot[12] = 0x60;
	DispPattern[55].pattern_foot[13] = 0x00;
	DispPattern[55].pattern_foot[14] = 0x00;
	DispPattern[55].pattern_foot[15] = 0x00;

	DispPattern[56].dispChar ='h';
	DispPattern[56].pattern_head[0] = 0x00;
	DispPattern[56].pattern_head[1] = 0x00;
	DispPattern[56].pattern_head[2] = 0x00;
	DispPattern[56].pattern_head[3] = 0x02;
	DispPattern[56].pattern_head[4] = 0x02;
	DispPattern[56].pattern_head[5] = 0xFE;
	DispPattern[56].pattern_head[6] = 0x80;
	DispPattern[56].pattern_head[7] = 0x40;
	DispPattern[56].pattern_head[8] = 0x40;
	DispPattern[56].pattern_head[9] = 0x40;
	DispPattern[56].pattern_head[10] = 0xC0;
	DispPattern[56].pattern_head[11] = 0x80;
	DispPattern[56].pattern_head[12] = 0x00;
	DispPattern[56].pattern_head[13] = 0x00;
	DispPattern[56].pattern_head[14] = 0x00;
	DispPattern[56].pattern_head[15] = 0x00;
	DispPattern[56].pattern_foot[0] = 0x00;
	DispPattern[56].pattern_foot[1] = 0x00;
	DispPattern[56].pattern_foot[2] = 0x00;
	DispPattern[56].pattern_foot[3] = 0x20;
	DispPattern[56].pattern_foot[4] = 0x20;
	DispPattern[56].pattern_foot[5] = 0x3F;
	DispPattern[56].pattern_foot[6] = 0x20;
	DispPattern[56].pattern_foot[7] = 0x00;
	DispPattern[56].pattern_foot[8] = 0x00;
	DispPattern[56].pattern_foot[9] = 0x00;
	DispPattern[56].pattern_foot[10] = 0x20;
	DispPattern[56].pattern_foot[11] = 0x3F;
	DispPattern[56].pattern_foot[12] = 0x20;
	DispPattern[56].pattern_foot[13] = 0x20;
	DispPattern[56].pattern_foot[14] = 0x00;
	DispPattern[56].pattern_foot[15] = 0x00;

	DispPattern[57].dispChar ='i';
	DispPattern[57].pattern_head[0] = 0x00;
	DispPattern[57].pattern_head[1] = 0x00;
	DispPattern[57].pattern_head[2] = 0x00;
	DispPattern[57].pattern_head[3] = 0x00;
	DispPattern[57].pattern_head[4] = 0x00;
	DispPattern[57].pattern_head[5] = 0x00;
	DispPattern[57].pattern_head[6] = 0x40;
	DispPattern[57].pattern_head[7] = 0x4C;
	DispPattern[57].pattern_head[8] = 0xCC;
	DispPattern[57].pattern_head[9] = 0x00;
	DispPattern[57].pattern_head[10] = 0x00;
	DispPattern[57].pattern_head[11] = 0x00;
	DispPattern[57].pattern_head[12] = 0x00;
	DispPattern[57].pattern_head[13] = 0x00;
	DispPattern[57].pattern_head[14] = 0x00;
	DispPattern[57].pattern_head[15] = 0x00;
	DispPattern[57].pattern_foot[0] = 0x00;
	DispPattern[57].pattern_foot[1] = 0x00;
	DispPattern[57].pattern_foot[2] = 0x00;
	DispPattern[57].pattern_foot[3] = 0x00;
	DispPattern[57].pattern_foot[4] = 0x00;
	DispPattern[57].pattern_foot[5] = 0x00;
	DispPattern[57].pattern_foot[6] = 0x20;
	DispPattern[57].pattern_foot[7] = 0x20;
	DispPattern[57].pattern_foot[8] = 0x3F;
	DispPattern[57].pattern_foot[9] = 0x20;
	DispPattern[57].pattern_foot[10] = 0x20;
	DispPattern[57].pattern_foot[11] = 0x00;
	DispPattern[57].pattern_foot[12] = 0x00;
	DispPattern[57].pattern_foot[13] = 0x00;
	DispPattern[57].pattern_foot[14] = 0x00;
	DispPattern[57].pattern_foot[15] = 0x00;

	DispPattern[58].dispChar ='j';
	DispPattern[58].pattern_head[0] = 0x00;
	DispPattern[58].pattern_head[1] = 0x00;
	DispPattern[58].pattern_head[2] = 0x00;
	DispPattern[58].pattern_head[3] = 0x00;
	DispPattern[58].pattern_head[4] = 0x00;
	DispPattern[58].pattern_head[5] = 0x00;
	DispPattern[58].pattern_head[6] = 0x00;
	DispPattern[58].pattern_head[7] = 0x40;
	DispPattern[58].pattern_head[8] = 0x4C;
	DispPattern[58].pattern_head[9] = 0xCC;
	DispPattern[58].pattern_head[10] = 0x00;
	DispPattern[58].pattern_head[11] = 0x00;
	DispPattern[58].pattern_head[12] = 0x00;
	DispPattern[58].pattern_head[13] = 0x00;
	DispPattern[58].pattern_head[14] = 0x00;
	DispPattern[58].pattern_head[15] = 0x00;
	DispPattern[58].pattern_foot[0] = 0x00;
	DispPattern[58].pattern_foot[1] = 0x00;
	DispPattern[58].pattern_foot[2] = 0x00;
	DispPattern[58].pattern_foot[3] = 0xC0;
	DispPattern[58].pattern_foot[4] = 0x80;
	DispPattern[58].pattern_foot[5] = 0x80;
	DispPattern[58].pattern_foot[6] = 0x80;
	DispPattern[58].pattern_foot[7] = 0x80;
	DispPattern[58].pattern_foot[8] = 0x40;
	DispPattern[58].pattern_foot[9] = 0x3F;
	DispPattern[58].pattern_foot[10] = 0x00;
	DispPattern[58].pattern_foot[11] = 0x00;
	DispPattern[58].pattern_foot[12] = 0x00;
	DispPattern[58].pattern_foot[13] = 0x00;
	DispPattern[58].pattern_foot[14] = 0x00;
	DispPattern[58].pattern_foot[15] = 0x00;

	DispPattern[59].dispChar ='k';
	DispPattern[59].pattern_head[0] = 0x00;
	DispPattern[59].pattern_head[1] = 0x00;
	DispPattern[59].pattern_head[2] = 0x02;
	DispPattern[59].pattern_head[3] = 0x02;
	DispPattern[59].pattern_head[4] = 0xFE;
	DispPattern[59].pattern_head[5] = 0x00;
	DispPattern[59].pattern_head[6] = 0x00;
	DispPattern[59].pattern_head[7] = 0x00;
	DispPattern[59].pattern_head[8] = 0x40;
	DispPattern[59].pattern_head[9] = 0xC0;
	DispPattern[59].pattern_head[10] = 0xC0;
	DispPattern[59].pattern_head[11] = 0x40;
	DispPattern[59].pattern_head[12] = 0x40;
	DispPattern[59].pattern_head[13] = 0x00;
	DispPattern[59].pattern_head[14] = 0x00;
	DispPattern[59].pattern_head[15] = 0x00;
	DispPattern[59].pattern_foot[0] = 0x00;
	DispPattern[59].pattern_foot[1] = 0x00;
	DispPattern[59].pattern_foot[2] = 0x20;
	DispPattern[59].pattern_foot[3] = 0x20;
	DispPattern[59].pattern_foot[4] = 0x3F;
	DispPattern[59].pattern_foot[5] = 0x22;
	DispPattern[59].pattern_foot[6] = 0x22;
	DispPattern[59].pattern_foot[7] = 0x05;
	DispPattern[59].pattern_foot[8] = 0x09;
	DispPattern[59].pattern_foot[9] = 0x28;
	DispPattern[59].pattern_foot[10] = 0x30;
	DispPattern[59].pattern_foot[11] = 0x30;
	DispPattern[59].pattern_foot[12] = 0x20;
	DispPattern[59].pattern_foot[13] = 0x20;
	DispPattern[59].pattern_foot[14] = 0x00;
	DispPattern[59].pattern_foot[15] = 0x00;

	DispPattern[60].dispChar ='l';
	DispPattern[60].pattern_head[0] = 0x00;
	DispPattern[60].pattern_head[1] = 0x00;
	DispPattern[60].pattern_head[2] = 0x00;
	DispPattern[60].pattern_head[3] = 0x00;
	DispPattern[60].pattern_head[4] = 0x00;
	DispPattern[60].pattern_head[5] = 0x02;
	DispPattern[60].pattern_head[6] = 0x02;
	DispPattern[60].pattern_head[7] = 0xFE;
	DispPattern[60].pattern_head[8] = 0x00;
	DispPattern[60].pattern_head[9] = 0x00;
	DispPattern[60].pattern_head[10] = 0x00;
	DispPattern[60].pattern_head[11] = 0x00;
	DispPattern[60].pattern_head[12] = 0x00;
	DispPattern[60].pattern_head[13] = 0x00;
	DispPattern[60].pattern_head[14] = 0x00;
	DispPattern[60].pattern_head[15] = 0x00;
	DispPattern[60].pattern_foot[0] = 0x00;
	DispPattern[60].pattern_foot[1] = 0x00;
	DispPattern[60].pattern_foot[2] = 0x00;
	DispPattern[60].pattern_foot[3] = 0x00;
	DispPattern[60].pattern_foot[4] = 0x00;
	DispPattern[60].pattern_foot[5] = 0x20;
	DispPattern[60].pattern_foot[6] = 0x20;
	DispPattern[60].pattern_foot[7] = 0x3F;
	DispPattern[60].pattern_foot[8] = 0x20;
	DispPattern[60].pattern_foot[9] = 0x20;
	DispPattern[60].pattern_foot[10] = 0x00;
	DispPattern[60].pattern_foot[11] = 0x00;
	DispPattern[60].pattern_foot[12] = 0x00;
	DispPattern[60].pattern_foot[13] = 0x00;
	DispPattern[60].pattern_foot[14] = 0x00;
	DispPattern[60].pattern_foot[15] = 0x00;

	DispPattern[61].dispChar ='m';
	DispPattern[61].pattern_head[0] = 0x00;
	DispPattern[61].pattern_head[1] = 0x00;
	DispPattern[61].pattern_head[2] = 0x40;
	DispPattern[61].pattern_head[3] = 0xC0;
	DispPattern[61].pattern_head[4] = 0x80;
	DispPattern[61].pattern_head[5] = 0x40;
	DispPattern[61].pattern_head[6] = 0x40;
	DispPattern[61].pattern_head[7] = 0x40;
	DispPattern[61].pattern_head[8] = 0x80;
	DispPattern[61].pattern_head[9] = 0x80;
	DispPattern[61].pattern_head[10] = 0x40;
	DispPattern[61].pattern_head[11] = 0x40;
	DispPattern[61].pattern_head[12] = 0x40;
	DispPattern[61].pattern_head[13] = 0x80;
	DispPattern[61].pattern_head[14] = 0x00;
	DispPattern[61].pattern_head[15] = 0x00;
	DispPattern[61].pattern_foot[0] = 0x00;
	DispPattern[61].pattern_foot[1] = 0x00;
	DispPattern[61].pattern_foot[2] = 0x20;
	DispPattern[61].pattern_foot[3] = 0x3F;
	DispPattern[61].pattern_foot[4] = 0x20;
	DispPattern[61].pattern_foot[5] = 0x00;
	DispPattern[61].pattern_foot[6] = 0x00;
	DispPattern[61].pattern_foot[7] = 0x20;
	DispPattern[61].pattern_foot[8] = 0x3F;
	DispPattern[61].pattern_foot[9] = 0x20;
	DispPattern[61].pattern_foot[10] = 0x00;
	DispPattern[61].pattern_foot[11] = 0x00;
	DispPattern[61].pattern_foot[12] = 0x20;
	DispPattern[61].pattern_foot[13] = 0x3F;
	DispPattern[61].pattern_foot[14] = 0x20;
	DispPattern[61].pattern_foot[15] = 0x00;

	DispPattern[62].dispChar ='n';
	DispPattern[62].pattern_head[0] = 0x00;
	DispPattern[62].pattern_head[1] = 0x00;
	DispPattern[62].pattern_head[2] = 0x00;
	DispPattern[62].pattern_head[3] = 0x40;
	DispPattern[62].pattern_head[4] = 0x40;
	DispPattern[62].pattern_head[5] = 0xC0;
	DispPattern[62].pattern_head[6] = 0x80;
	DispPattern[62].pattern_head[7] = 0x40;
	DispPattern[62].pattern_head[8] = 0x40;
	DispPattern[62].pattern_head[9] = 0x40;
	DispPattern[62].pattern_head[10] = 0xC0;
	DispPattern[62].pattern_head[11] = 0x80;
	DispPattern[62].pattern_head[12] = 0x00;
	DispPattern[62].pattern_head[13] = 0x00;
	DispPattern[62].pattern_head[14] = 0x00;
	DispPattern[62].pattern_head[15] = 0x00;
	DispPattern[62].pattern_foot[0] = 0x00;
	DispPattern[62].pattern_foot[1] = 0x00;
	DispPattern[62].pattern_foot[2] = 0x00;
	DispPattern[62].pattern_foot[3] = 0x20;
	DispPattern[62].pattern_foot[4] = 0x20;
	DispPattern[62].pattern_foot[5] = 0x3F;
	DispPattern[62].pattern_foot[6] = 0x20;
	DispPattern[62].pattern_foot[7] = 0x00;
	DispPattern[62].pattern_foot[8] = 0x00;
	DispPattern[62].pattern_foot[9] = 0x00;
	DispPattern[62].pattern_foot[10] = 0x20;
	DispPattern[62].pattern_foot[11] = 0x3F;
	DispPattern[62].pattern_foot[12] = 0x20;
	DispPattern[62].pattern_foot[13] = 0x20;
	DispPattern[62].pattern_foot[14] = 0x00;
	DispPattern[62].pattern_foot[15] = 0x00;

	DispPattern[63].dispChar ='o';
	DispPattern[63].pattern_head[0] = 0x00;
	DispPattern[63].pattern_head[1] = 0x00;
	DispPattern[63].pattern_head[2] = 0x00;
	DispPattern[63].pattern_head[3] = 0x00;
	DispPattern[63].pattern_head[4] = 0x00;
	DispPattern[63].pattern_head[5] = 0x80;
	DispPattern[63].pattern_head[6] = 0x40;
	DispPattern[63].pattern_head[7] = 0x40;
	DispPattern[63].pattern_head[8] = 0x40;
	DispPattern[63].pattern_head[9] = 0x40;
	DispPattern[63].pattern_head[10] = 0x80;
	DispPattern[63].pattern_head[11] = 0x00;
	DispPattern[63].pattern_head[12] = 0x00;
	DispPattern[63].pattern_head[13] = 0x00;
	DispPattern[63].pattern_head[14] = 0x00;
	DispPattern[63].pattern_head[15] = 0x00;
	DispPattern[63].pattern_foot[0] = 0x00;
	DispPattern[63].pattern_foot[1] = 0x00;
	DispPattern[63].pattern_foot[2] = 0x00;
	DispPattern[63].pattern_foot[3] = 0x00;
	DispPattern[63].pattern_foot[4] = 0x0F;
	DispPattern[63].pattern_foot[5] = 0x10;
	DispPattern[63].pattern_foot[6] = 0x20;
	DispPattern[63].pattern_foot[7] = 0x20;
	DispPattern[63].pattern_foot[8] = 0x20;
	DispPattern[63].pattern_foot[9] = 0x20;
	DispPattern[63].pattern_foot[10] = 0x10;
	DispPattern[63].pattern_foot[11] = 0x0F;
	DispPattern[63].pattern_foot[12] = 0x00;
	DispPattern[63].pattern_foot[13] = 0x00;
	DispPattern[63].pattern_foot[14] = 0x00;
	DispPattern[63].pattern_foot[15] = 0x00;

	DispPattern[64].dispChar ='p';
	DispPattern[64].pattern_head[0] = 0x00;
	DispPattern[64].pattern_head[1] = 0x00;
	DispPattern[64].pattern_head[2] = 0x00;
	DispPattern[64].pattern_head[3] = 0x40;
	DispPattern[64].pattern_head[4] = 0x40;
	DispPattern[64].pattern_head[5] = 0xC0;
	DispPattern[64].pattern_head[6] = 0x80;
	DispPattern[64].pattern_head[7] = 0x40;
	DispPattern[64].pattern_head[8] = 0x40;
	DispPattern[64].pattern_head[9] = 0x40;
	DispPattern[64].pattern_head[10] = 0x40;
	DispPattern[64].pattern_head[11] = 0x80;
	DispPattern[64].pattern_head[12] = 0x00;
	DispPattern[64].pattern_head[13] = 0x00;
	DispPattern[64].pattern_head[14] = 0x00;
	DispPattern[64].pattern_head[15] = 0x00;
	DispPattern[64].pattern_foot[0] = 0x00;
	DispPattern[64].pattern_foot[1] = 0x00;
	DispPattern[64].pattern_foot[2] = 0x00;
	DispPattern[64].pattern_foot[3] = 0x80;
	DispPattern[64].pattern_foot[4] = 0x80;
	DispPattern[64].pattern_foot[5] = 0xFF;
	DispPattern[64].pattern_foot[6] = 0x88;
	DispPattern[64].pattern_foot[7] = 0x90;
	DispPattern[64].pattern_foot[8] = 0x10;
	DispPattern[64].pattern_foot[9] = 0x10;
	DispPattern[64].pattern_foot[10] = 0x10;
	DispPattern[64].pattern_foot[11] = 0x08;
	DispPattern[64].pattern_foot[12] = 0x07;
	DispPattern[64].pattern_foot[13] = 0x00;
	DispPattern[64].pattern_foot[14] = 0x00;
	DispPattern[64].pattern_foot[15] = 0x00;

	DispPattern[65].dispChar ='q';
	DispPattern[65].pattern_head[0] = 0x00;
	DispPattern[65].pattern_head[1] = 0x00;
	DispPattern[65].pattern_head[2] = 0x00;
	DispPattern[65].pattern_head[3] = 0x00;
	DispPattern[65].pattern_head[4] = 0x80;
	DispPattern[65].pattern_head[5] = 0x40;
	DispPattern[65].pattern_head[6] = 0x40;
	DispPattern[65].pattern_head[7] = 0x40;
	DispPattern[65].pattern_head[8] = 0x40;
	DispPattern[65].pattern_head[9] = 0x80;
	DispPattern[65].pattern_head[10] = 0xC0;
	DispPattern[65].pattern_head[11] = 0x00;
	DispPattern[65].pattern_head[12] = 0x00;
	DispPattern[65].pattern_head[13] = 0x00;
	DispPattern[65].pattern_head[14] = 0x00;
	DispPattern[65].pattern_head[15] = 0x00;
	DispPattern[65].pattern_foot[0] = 0x00;
	DispPattern[65].pattern_foot[1] = 0x00;
	DispPattern[65].pattern_foot[2] = 0x00;
	DispPattern[65].pattern_foot[3] = 0x07;
	DispPattern[65].pattern_foot[4] = 0x08;
	DispPattern[65].pattern_foot[5] = 0x10;
	DispPattern[65].pattern_foot[6] = 0x10;
	DispPattern[65].pattern_foot[7] = 0x10;
	DispPattern[65].pattern_foot[8] = 0x90;
	DispPattern[65].pattern_foot[9] = 0x88;
	DispPattern[65].pattern_foot[10] = 0xFF;
	DispPattern[65].pattern_foot[11] = 0x80;
	DispPattern[65].pattern_foot[12] = 0x80;
	DispPattern[65].pattern_foot[13] = 0x00;
	DispPattern[65].pattern_foot[14] = 0x00;
	DispPattern[65].pattern_foot[15] = 0x00;

	DispPattern[66].dispChar ='r';
	DispPattern[66].pattern_head[0] = 0x00;
	DispPattern[66].pattern_head[1] = 0x00;
	DispPattern[66].pattern_head[2] = 0x00;
	DispPattern[66].pattern_head[3] = 0x00;
	DispPattern[66].pattern_head[4] = 0x40;
	DispPattern[66].pattern_head[5] = 0x40;
	DispPattern[66].pattern_head[6] = 0xC0;
	DispPattern[66].pattern_head[7] = 0x00;
	DispPattern[66].pattern_head[8] = 0x80;
	DispPattern[66].pattern_head[9] = 0x40;
	DispPattern[66].pattern_head[10] = 0x40;
	DispPattern[66].pattern_head[11] = 0xC0;
	DispPattern[66].pattern_head[12] = 0x00;
	DispPattern[66].pattern_head[13] = 0x00;
	DispPattern[66].pattern_head[14] = 0x00;
	DispPattern[66].pattern_head[15] = 0x00;
	DispPattern[66].pattern_foot[0] = 0x00;
	DispPattern[66].pattern_foot[1] = 0x00;
	DispPattern[66].pattern_foot[2] = 0x00;
	DispPattern[66].pattern_foot[3] = 0x00;
	DispPattern[66].pattern_foot[4] = 0x20;
	DispPattern[66].pattern_foot[5] = 0x20;
	DispPattern[66].pattern_foot[6] = 0x3F;
	DispPattern[66].pattern_foot[7] = 0x21;
	DispPattern[66].pattern_foot[8] = 0x20;
	DispPattern[66].pattern_foot[9] = 0x00;
	DispPattern[66].pattern_foot[10] = 0x00;
	DispPattern[66].pattern_foot[11] = 0x00;
	DispPattern[66].pattern_foot[12] = 0x00;
	DispPattern[66].pattern_foot[13] = 0x00;
	DispPattern[66].pattern_foot[14] = 0x00;
	DispPattern[66].pattern_foot[15] = 0x00;

	DispPattern[67].dispChar ='s';
	DispPattern[67].pattern_head[0] = 0x00;
	DispPattern[67].pattern_head[1] = 0x00;
	DispPattern[67].pattern_head[2] = 0x00;
	DispPattern[67].pattern_head[3] = 0x00;
	DispPattern[67].pattern_head[4] = 0x80;
	DispPattern[67].pattern_head[5] = 0x40;
	DispPattern[67].pattern_head[6] = 0x40;
	DispPattern[67].pattern_head[7] = 0x40;
	DispPattern[67].pattern_head[8] = 0x40;
	DispPattern[67].pattern_head[9] = 0x40;
	DispPattern[67].pattern_head[10] = 0x80;
	DispPattern[67].pattern_head[11] = 0xC0;
	DispPattern[67].pattern_head[12] = 0x00;
	DispPattern[67].pattern_head[13] = 0x00;
	DispPattern[67].pattern_head[14] = 0x00;
	DispPattern[67].pattern_head[15] = 0x00;
	DispPattern[67].pattern_foot[0] = 0x00;
	DispPattern[67].pattern_foot[1] = 0x00;
	DispPattern[67].pattern_foot[2] = 0x00;
	DispPattern[67].pattern_foot[3] = 0x00;
	DispPattern[67].pattern_foot[4] = 0x39;
	DispPattern[67].pattern_foot[5] = 0x12;
	DispPattern[67].pattern_foot[6] = 0x22;
	DispPattern[67].pattern_foot[7] = 0x22;
	DispPattern[67].pattern_foot[8] = 0x22;
	DispPattern[67].pattern_foot[9] = 0x24;
	DispPattern[67].pattern_foot[10] = 0x24;
	DispPattern[67].pattern_foot[11] = 0x19;
	DispPattern[67].pattern_foot[12] = 0x00;
	DispPattern[67].pattern_foot[13] = 0x00;
	DispPattern[67].pattern_foot[14] = 0x00;
	DispPattern[67].pattern_foot[15] = 0x00;

	DispPattern[68].dispChar ='t';
	DispPattern[68].pattern_head[0] = 0x00;
	DispPattern[68].pattern_head[1] = 0x00;
	DispPattern[68].pattern_head[2] = 0x00;
	DispPattern[68].pattern_head[3] = 0x40;
	DispPattern[68].pattern_head[4] = 0x40;
	DispPattern[68].pattern_head[5] = 0x40;
	DispPattern[68].pattern_head[6] = 0xFC;
	DispPattern[68].pattern_head[7] = 0x40;
	DispPattern[68].pattern_head[8] = 0x40;
	DispPattern[68].pattern_head[9] = 0x40;
	DispPattern[68].pattern_head[10] = 0x40;
	DispPattern[68].pattern_head[11] = 0x00;
	DispPattern[68].pattern_head[12] = 0x00;
	DispPattern[68].pattern_head[13] = 0x00;
	DispPattern[68].pattern_head[14] = 0x00;
	DispPattern[68].pattern_head[15] = 0x00;
	DispPattern[68].pattern_foot[0] = 0x00;
	DispPattern[68].pattern_foot[1] = 0x00;
	DispPattern[68].pattern_foot[2] = 0x00;
	DispPattern[68].pattern_foot[3] = 0x00;
	DispPattern[68].pattern_foot[4] = 0x00;
	DispPattern[68].pattern_foot[5] = 0x00;
	DispPattern[68].pattern_foot[6] = 0x0F;
	DispPattern[68].pattern_foot[7] = 0x10;
	DispPattern[68].pattern_foot[8] = 0x20;
	DispPattern[68].pattern_foot[9] = 0x20;
	DispPattern[68].pattern_foot[10] = 0x20;
	DispPattern[68].pattern_foot[11] = 0x30;
	DispPattern[68].pattern_foot[12] = 0x00;
	DispPattern[68].pattern_foot[13] = 0x00;
	DispPattern[68].pattern_foot[14] = 0x00;
	DispPattern[68].pattern_foot[15] = 0x00;

	DispPattern[69].dispChar ='u';
	DispPattern[69].pattern_head[0] = 0x00;
	DispPattern[69].pattern_head[1] = 0x00;
	DispPattern[69].pattern_head[2] = 0x00;
	DispPattern[69].pattern_head[3] = 0x40;
	DispPattern[69].pattern_head[4] = 0x40;
	DispPattern[69].pattern_head[5] = 0xC0;
	DispPattern[69].pattern_head[6] = 0x00;
	DispPattern[69].pattern_head[7] = 0x00;
	DispPattern[69].pattern_head[8] = 0x00;
	DispPattern[69].pattern_head[9] = 0x40;
	DispPattern[69].pattern_head[10] = 0x40;
	DispPattern[69].pattern_head[11] = 0xC0;
	DispPattern[69].pattern_head[12] = 0x00;
	DispPattern[69].pattern_head[13] = 0x00;
	DispPattern[69].pattern_head[14] = 0x00;
	DispPattern[69].pattern_head[15] = 0x00;
	DispPattern[69].pattern_foot[0] = 0x00;
	DispPattern[69].pattern_foot[1] = 0x00;
	DispPattern[69].pattern_foot[2] = 0x00;
	DispPattern[69].pattern_foot[3] = 0x00;
	DispPattern[69].pattern_foot[4] = 0x00;
	DispPattern[69].pattern_foot[5] = 0x1F;
	DispPattern[69].pattern_foot[6] = 0x30;
	DispPattern[69].pattern_foot[7] = 0x20;
	DispPattern[69].pattern_foot[8] = 0x20;
	DispPattern[69].pattern_foot[9] = 0x20;
	DispPattern[69].pattern_foot[10] = 0x10;
	DispPattern[69].pattern_foot[11] = 0x3F;
	DispPattern[69].pattern_foot[12] = 0x20;
	DispPattern[69].pattern_foot[13] = 0x20;
	DispPattern[69].pattern_foot[14] = 0x00;
	DispPattern[69].pattern_foot[15] = 0x00;

	DispPattern[70].dispChar ='v';
	DispPattern[70].pattern_head[0] = 0x00;
	DispPattern[70].pattern_head[1] = 0x00;
	DispPattern[70].pattern_head[2] = 0x40;
	DispPattern[70].pattern_head[3] = 0x40;
	DispPattern[70].pattern_head[4] = 0xC0;
	DispPattern[70].pattern_head[5] = 0x40;
	DispPattern[70].pattern_head[6] = 0x00;
	DispPattern[70].pattern_head[7] = 0x00;
	DispPattern[70].pattern_head[8] = 0x00;
	DispPattern[70].pattern_head[9] = 0x40;
	DispPattern[70].pattern_head[10] = 0xC0;
	DispPattern[70].pattern_head[11] = 0x40;
	DispPattern[70].pattern_head[12] = 0x40;
	DispPattern[70].pattern_head[13] = 0x00;
	DispPattern[70].pattern_head[14] = 0x00;
	DispPattern[70].pattern_head[15] = 0x00;
	DispPattern[70].pattern_foot[0] = 0x00;
	DispPattern[70].pattern_foot[1] = 0x00;
	DispPattern[70].pattern_foot[2] = 0x00;
	DispPattern[70].pattern_foot[3] = 0x00;
	DispPattern[70].pattern_foot[4] = 0x01;
	DispPattern[70].pattern_foot[5] = 0x07;
	DispPattern[70].pattern_foot[6] = 0x1C;
	DispPattern[70].pattern_foot[7] = 0x30;
	DispPattern[70].pattern_foot[8] = 0x1C;
	DispPattern[70].pattern_foot[9] = 0x07;
	DispPattern[70].pattern_foot[10] = 0x01;
	DispPattern[70].pattern_foot[11] = 0x00;
	DispPattern[70].pattern_foot[12] = 0x00;
	DispPattern[70].pattern_foot[13] = 0x00;
	DispPattern[70].pattern_foot[14] = 0x00;
	DispPattern[70].pattern_foot[15] = 0x00;

	DispPattern[71].dispChar ='w';
	DispPattern[71].pattern_head[0] = 0x00;
	DispPattern[71].pattern_head[1] = 0x40;
	DispPattern[71].pattern_head[2] = 0x40;
	DispPattern[71].pattern_head[3] = 0xC0;
	DispPattern[71].pattern_head[4] = 0x40;
	DispPattern[71].pattern_head[5] = 0x00;
	DispPattern[71].pattern_head[6] = 0x00;
	DispPattern[71].pattern_head[7] = 0xC0;
	DispPattern[71].pattern_head[8] = 0xC0;
	DispPattern[71].pattern_head[9] = 0x00;
	DispPattern[71].pattern_head[10] = 0x00;
	DispPattern[71].pattern_head[11] = 0x40;
	DispPattern[71].pattern_head[12] = 0xC0;
	DispPattern[71].pattern_head[13] = 0x40;
	DispPattern[71].pattern_head[14] = 0x40;
	DispPattern[71].pattern_head[15] = 0x00;
	DispPattern[71].pattern_foot[0] = 0x00;
	DispPattern[71].pattern_foot[1] = 0x00;
	DispPattern[71].pattern_foot[2] = 0x00;
	DispPattern[71].pattern_foot[3] = 0x01;
	DispPattern[71].pattern_foot[4] = 0x0F;
	DispPattern[71].pattern_foot[5] = 0x38;
	DispPattern[71].pattern_foot[6] = 0x0F;
	DispPattern[71].pattern_foot[7] = 0x01;
	DispPattern[71].pattern_foot[8] = 0x01;
	DispPattern[71].pattern_foot[9] = 0x0F;
	DispPattern[71].pattern_foot[10] = 0x38;
	DispPattern[71].pattern_foot[11] = 0x0F;
	DispPattern[71].pattern_foot[12] = 0x01;
	DispPattern[71].pattern_foot[13] = 0x00;
	DispPattern[71].pattern_foot[14] = 0x00;
	DispPattern[71].pattern_foot[15] = 0x00;

	DispPattern[72].dispChar ='x';
	DispPattern[72].pattern_head[0] = 0x00;
	DispPattern[72].pattern_head[1] = 0x00;
	DispPattern[72].pattern_head[2] = 0x00;
	DispPattern[72].pattern_head[3] = 0x40;
	DispPattern[72].pattern_head[4] = 0x40;
	DispPattern[72].pattern_head[5] = 0xC0;
	DispPattern[72].pattern_head[6] = 0x40;
	DispPattern[72].pattern_head[7] = 0x00;
	DispPattern[72].pattern_head[8] = 0x00;
	DispPattern[72].pattern_head[9] = 0x40;
	DispPattern[72].pattern_head[10] = 0xC0;
	DispPattern[72].pattern_head[11] = 0x40;
	DispPattern[72].pattern_head[12] = 0x40;
	DispPattern[72].pattern_head[13] = 0x00;
	DispPattern[72].pattern_head[14] = 0x00;
	DispPattern[72].pattern_head[15] = 0x00;
	DispPattern[72].pattern_foot[0] = 0x00;
	DispPattern[72].pattern_foot[1] = 0x00;
	DispPattern[72].pattern_foot[2] = 0x20;
	DispPattern[72].pattern_foot[3] = 0x20;
	DispPattern[72].pattern_foot[4] = 0x30;
	DispPattern[72].pattern_foot[5] = 0x28;
	DispPattern[72].pattern_foot[6] = 0x25;
	DispPattern[72].pattern_foot[7] = 0x02;
	DispPattern[72].pattern_foot[8] = 0x02;
	DispPattern[72].pattern_foot[9] = 0x25;
	DispPattern[72].pattern_foot[10] = 0x28;
	DispPattern[72].pattern_foot[11] = 0x30;
	DispPattern[72].pattern_foot[12] = 0x20;
	DispPattern[72].pattern_foot[13] = 0x20;
	DispPattern[72].pattern_foot[14] = 0x00;
	DispPattern[72].pattern_foot[15] = 0x00;

	DispPattern[73].dispChar ='y';
	DispPattern[73].pattern_head[0] = 0x00;
	DispPattern[73].pattern_head[1] = 0x00;
	DispPattern[73].pattern_head[2] = 0x00;
	DispPattern[73].pattern_head[3] = 0x40;
	DispPattern[73].pattern_head[4] = 0x40;
	DispPattern[73].pattern_head[5] = 0xC0;
	DispPattern[73].pattern_head[6] = 0x40;
	DispPattern[73].pattern_head[7] = 0x00;
	DispPattern[73].pattern_head[8] = 0x00;
	DispPattern[73].pattern_head[9] = 0x00;
	DispPattern[73].pattern_head[10] = 0x40;
	DispPattern[73].pattern_head[11] = 0xC0;
	DispPattern[73].pattern_head[12] = 0x40;
	DispPattern[73].pattern_head[13] = 0x40;
	DispPattern[73].pattern_head[14] = 0x00;
	DispPattern[73].pattern_head[15] = 0x00;
	DispPattern[73].pattern_foot[0] = 0x00;
	DispPattern[73].pattern_foot[1] = 0x00;
	DispPattern[73].pattern_foot[2] = 0x00;
	DispPattern[73].pattern_foot[3] = 0xC0;
	DispPattern[73].pattern_foot[4] = 0x80;
	DispPattern[73].pattern_foot[5] = 0x81;
	DispPattern[73].pattern_foot[6] = 0xC7;
	DispPattern[73].pattern_foot[7] = 0x6C;
	DispPattern[73].pattern_foot[8] = 0x30;
	DispPattern[73].pattern_foot[9] = 0x1C;
	DispPattern[73].pattern_foot[10] = 0x07;
	DispPattern[73].pattern_foot[11] = 0x01;
	DispPattern[73].pattern_foot[12] = 0x00;
	DispPattern[73].pattern_foot[13] = 0x00;
	DispPattern[73].pattern_foot[14] = 0x00;
	DispPattern[73].pattern_foot[15] = 0x00;

	DispPattern[74].dispChar ='z';
	DispPattern[74].pattern_head[0] = 0x00;
	DispPattern[74].pattern_head[1] = 0x00;
	DispPattern[74].pattern_head[2] = 0x00;
	DispPattern[74].pattern_head[3] = 0x00;
	DispPattern[74].pattern_head[4] = 0xC0;
	DispPattern[74].pattern_head[5] = 0x40;
	DispPattern[74].pattern_head[6] = 0x40;
	DispPattern[74].pattern_head[7] = 0x40;
	DispPattern[74].pattern_head[8] = 0x40;
	DispPattern[74].pattern_head[9] = 0x40;
	DispPattern[74].pattern_head[10] = 0xC0;
	DispPattern[74].pattern_head[11] = 0x40;
	DispPattern[74].pattern_head[12] = 0x00;
	DispPattern[74].pattern_head[13] = 0x00;
	DispPattern[74].pattern_head[14] = 0x00;
	DispPattern[74].pattern_head[15] = 0x00;
	DispPattern[74].pattern_foot[0] = 0x00;
	DispPattern[74].pattern_foot[1] = 0x00;
	DispPattern[74].pattern_foot[2] = 0x00;
	DispPattern[74].pattern_foot[3] = 0x00;
	DispPattern[74].pattern_foot[4] = 0x21;
	DispPattern[74].pattern_foot[5] = 0x30;
	DispPattern[74].pattern_foot[6] = 0x28;
	DispPattern[74].pattern_foot[7] = 0x24;
	DispPattern[74].pattern_foot[8] = 0x22;
	DispPattern[74].pattern_foot[9] = 0x21;
	DispPattern[74].pattern_foot[10] = 0x20;
	DispPattern[74].pattern_foot[11] = 0x38;
	DispPattern[74].pattern_foot[12] = 0x00;
	DispPattern[74].pattern_foot[13] = 0x00;
	DispPattern[74].pattern_foot[14] = 0x00;
	DispPattern[74].pattern_foot[15] = 0x00;
}

void SetCharPattern_symbol() 
{
    DispPattern[80].dispChar =' ';
    DispPattern[80].pattern_head[0] = 0x00;
    DispPattern[80].pattern_head[1] = 0x00;
    DispPattern[80].pattern_head[2] = 0x00;
    DispPattern[80].pattern_head[3] = 0x00;
    DispPattern[80].pattern_head[4] = 0x00;
    DispPattern[80].pattern_head[5] = 0x00;
    DispPattern[80].pattern_head[6] = 0x00;
    DispPattern[80].pattern_head[7] = 0x00;
    DispPattern[80].pattern_head[8] = 0x00;
    DispPattern[80].pattern_head[9] = 0x00;
    DispPattern[80].pattern_head[10] = 0x00;
    DispPattern[80].pattern_head[11] = 0x00;
    DispPattern[80].pattern_head[12] = 0x00;
    DispPattern[80].pattern_head[13] = 0x00;
    DispPattern[80].pattern_head[14] = 0x00;
    DispPattern[80].pattern_head[15] = 0x00;
    DispPattern[80].pattern_foot[0] = 0x00;
    DispPattern[80].pattern_foot[1] = 0x00;
    DispPattern[80].pattern_foot[2] = 0x00;
    DispPattern[80].pattern_foot[3] = 0x00;
    DispPattern[80].pattern_foot[4] = 0x00;
    DispPattern[80].pattern_foot[5] = 0x00;
    DispPattern[80].pattern_foot[6] = 0x00;
    DispPattern[80].pattern_foot[7] = 0x00;
    DispPattern[80].pattern_foot[8] = 0x00;
    DispPattern[80].pattern_foot[9] = 0x00;
    DispPattern[80].pattern_foot[10] = 0x00;
    DispPattern[80].pattern_foot[11] = 0x00;
    DispPattern[80].pattern_foot[12] = 0x00;
    DispPattern[80].pattern_foot[13] = 0x00;
    DispPattern[80].pattern_foot[14] = 0x00;
    DispPattern[80].pattern_foot[15] = 0x00;

    DispPattern[81].dispChar ='/';
    DispPattern[81].pattern_head[0] = 0x00;
    DispPattern[81].pattern_head[1] = 0x00;
    DispPattern[81].pattern_head[2] = 0x00;
    DispPattern[81].pattern_head[3] = 0x00;
    DispPattern[81].pattern_head[4] = 0x00;
    DispPattern[81].pattern_head[5] = 0x00;
    DispPattern[81].pattern_head[6] = 0x00;
    DispPattern[81].pattern_head[7] = 0x00;
    DispPattern[81].pattern_head[8] = 0x80;
    DispPattern[81].pattern_head[9] = 0x40;
    DispPattern[81].pattern_head[10] = 0x20;
    DispPattern[81].pattern_head[11] = 0x10;
    DispPattern[81].pattern_head[12] = 0x08;
    DispPattern[81].pattern_head[13] = 0x04;
    DispPattern[81].pattern_head[14] = 0x02;
    DispPattern[81].pattern_head[15] = 0x01;
    DispPattern[81].pattern_foot[0] = 0x80;
    DispPattern[81].pattern_foot[1] = 0x40;
    DispPattern[81].pattern_foot[2] = 0x20;
    DispPattern[81].pattern_foot[3] = 0x10;
    DispPattern[81].pattern_foot[4] = 0x08;
    DispPattern[81].pattern_foot[5] = 0x04;
    DispPattern[81].pattern_foot[6] = 0x02;
    DispPattern[81].pattern_foot[7] = 0x01;
    DispPattern[81].pattern_foot[8] = 0x00;
    DispPattern[81].pattern_foot[9] = 0x00;
    DispPattern[81].pattern_foot[10] = 0x00;
    DispPattern[81].pattern_foot[11] = 0x00;
    DispPattern[81].pattern_foot[12] = 0x00;
    DispPattern[81].pattern_foot[13] = 0x00;
    DispPattern[81].pattern_foot[14] = 0x00;
    DispPattern[81].pattern_foot[15] = 0x00;

    DispPattern[82].dispChar ='?';
    DispPattern[82].pattern_head[0] = 0x00;
    DispPattern[82].pattern_head[1] = 0x00;
    DispPattern[82].pattern_head[2] = 0x00;
    DispPattern[82].pattern_head[3] = 0x18;
    DispPattern[82].pattern_head[4] = 0x24;
    DispPattern[82].pattern_head[5] = 0x02;
    DispPattern[82].pattern_head[6] = 0x02;
    DispPattern[82].pattern_head[7] = 0x02;
    DispPattern[82].pattern_head[8] = 0x82;
    DispPattern[82].pattern_head[9] = 0x82;
    DispPattern[82].pattern_head[10] = 0x42;
    DispPattern[82].pattern_head[11] = 0x44;
    DispPattern[82].pattern_head[12] = 0x38;
    DispPattern[82].pattern_head[13] = 0x00;
    DispPattern[82].pattern_head[14] = 0x00;
    DispPattern[82].pattern_head[15] = 0x00;
    DispPattern[82].pattern_foot[0] = 0x00;
    DispPattern[82].pattern_foot[1] = 0x00;
    DispPattern[82].pattern_foot[2] = 0x00;
    DispPattern[82].pattern_foot[3] = 0x00;
    DispPattern[82].pattern_foot[4] = 0x00;
    DispPattern[82].pattern_foot[5] = 0x00;
    DispPattern[82].pattern_foot[6] = 0x00;
    DispPattern[82].pattern_foot[7] = 0x33;
    DispPattern[82].pattern_foot[8] = 0x30;
    DispPattern[82].pattern_foot[9] = 0x00;
    DispPattern[82].pattern_foot[10] = 0x00;
    DispPattern[82].pattern_foot[11] = 0x00;
    DispPattern[82].pattern_foot[12] = 0x00;
    DispPattern[82].pattern_foot[13] = 0x00;
    DispPattern[82].pattern_foot[14] = 0x00;
    DispPattern[82].pattern_foot[15] = 0x00;

    DispPattern[83].dispChar ='-';
    DispPattern[83].pattern_head[0] = 0x00;
    DispPattern[83].pattern_head[1] = 0x00;
    DispPattern[83].pattern_head[2] = 0x00;
    DispPattern[83].pattern_head[3] = 0x00;
    DispPattern[83].pattern_head[4] = 0x00;
    DispPattern[83].pattern_head[5] = 0x80;
    DispPattern[83].pattern_head[6] = 0x80;
    DispPattern[83].pattern_head[7] = 0x80;
    DispPattern[83].pattern_head[8] = 0x80;
    DispPattern[83].pattern_head[9] = 0x80;
    DispPattern[83].pattern_head[10] = 0x80;
    DispPattern[83].pattern_head[11] = 0x00;
    DispPattern[83].pattern_head[12] = 0x00;
    DispPattern[83].pattern_head[13] = 0x00;
    DispPattern[83].pattern_head[14] = 0x00;
    DispPattern[83].pattern_head[15] = 0x00;
    DispPattern[83].pattern_foot[0] = 0x00;
    DispPattern[83].pattern_foot[1] = 0x00;
    DispPattern[83].pattern_foot[2] = 0x00;
    DispPattern[83].pattern_foot[3] = 0x00;
    DispPattern[83].pattern_foot[4] = 0x00;
    DispPattern[83].pattern_foot[5] = 0x00;
    DispPattern[83].pattern_foot[6] = 0x00;
    DispPattern[83].pattern_foot[7] = 0x00;
    DispPattern[83].pattern_foot[8] = 0x00;
    DispPattern[83].pattern_foot[9] = 0x00;
    DispPattern[83].pattern_foot[10] = 0x00;
    DispPattern[83].pattern_foot[11] = 0x00;
    DispPattern[83].pattern_foot[12] = 0x00;
    DispPattern[83].pattern_foot[13] = 0x00;
    DispPattern[83].pattern_foot[14] = 0x00;
    DispPattern[83].pattern_foot[15] = 0x00;

    DispPattern[84].dispChar =':';
    DispPattern[84].pattern_head[0] = 0x00;
    DispPattern[84].pattern_head[1] = 0x00;
    DispPattern[84].pattern_head[2] = 0x00;
    DispPattern[84].pattern_head[3] = 0x00;
    DispPattern[84].pattern_head[4] = 0x00;
    DispPattern[84].pattern_head[5] = 0x00;
    DispPattern[84].pattern_head[6] = 0x00;
    DispPattern[84].pattern_head[7] = 0xC0;
    DispPattern[84].pattern_head[8] = 0xC0;
    DispPattern[84].pattern_head[9] = 0x00;
    DispPattern[84].pattern_head[10] = 0x00;
    DispPattern[84].pattern_head[11] = 0x00;
    DispPattern[84].pattern_head[12] = 0x00;
    DispPattern[84].pattern_head[13] = 0x00;
    DispPattern[84].pattern_head[14] = 0x00;
    DispPattern[84].pattern_head[15] = 0x00;
    DispPattern[84].pattern_foot[0] = 0x00;
    DispPattern[84].pattern_foot[1] = 0x00;
    DispPattern[84].pattern_foot[2] = 0x00;
    DispPattern[84].pattern_foot[3] = 0x00;
    DispPattern[84].pattern_foot[4] = 0x00;
    DispPattern[84].pattern_foot[5] = 0x00;
    DispPattern[84].pattern_foot[6] = 0x00;
    DispPattern[84].pattern_foot[7] = 0x30;
    DispPattern[84].pattern_foot[8] = 0x30;
    DispPattern[84].pattern_foot[9] = 0x00;
    DispPattern[84].pattern_foot[10] = 0x00;
    DispPattern[84].pattern_foot[11] = 0x00;
    DispPattern[84].pattern_foot[12] = 0x00;
    DispPattern[84].pattern_foot[13] = 0x00;
    DispPattern[84].pattern_foot[14] = 0x00;
    DispPattern[84].pattern_foot[15] = 0x00;

    DispPattern[85].dispChar ='.';
    DispPattern[85].pattern_head[0] = 0x00;
    DispPattern[85].pattern_head[1] = 0x00;
    DispPattern[85].pattern_head[2] = 0x00;
    DispPattern[85].pattern_head[3] = 0x00;
    DispPattern[85].pattern_head[4] = 0x00;
    DispPattern[85].pattern_head[5] = 0x00;
    DispPattern[85].pattern_head[6] = 0x00;
    DispPattern[85].pattern_head[7] = 0x00;
    DispPattern[85].pattern_head[8] = 0x00;
    DispPattern[85].pattern_head[9] = 0x00;
    DispPattern[85].pattern_head[10] = 0x00;
    DispPattern[85].pattern_head[11] = 0x00;
    DispPattern[85].pattern_head[12] = 0x00;
    DispPattern[85].pattern_head[13] = 0x00;
    DispPattern[85].pattern_head[14] = 0x00;
    DispPattern[85].pattern_head[15] = 0x00;
    DispPattern[85].pattern_foot[0] = 0x00;
    DispPattern[85].pattern_foot[1] = 0x30;
    DispPattern[85].pattern_foot[2] = 0x30;
    DispPattern[85].pattern_foot[3] = 0x00;
    DispPattern[85].pattern_foot[4] = 0x00;
    DispPattern[85].pattern_foot[5] = 0x00;
    DispPattern[85].pattern_foot[6] = 0x00;
    DispPattern[85].pattern_foot[7] = 0x00;
    DispPattern[85].pattern_foot[8] = 0x00;
    DispPattern[85].pattern_foot[9] = 0x00;
    DispPattern[85].pattern_foot[10] = 0x00;
    DispPattern[85].pattern_foot[11] = 0x00;
    DispPattern[85].pattern_foot[12] = 0x00;
    DispPattern[85].pattern_foot[13] = 0x00;
    DispPattern[85].pattern_foot[14] = 0x00;
    DispPattern[85].pattern_foot[15] = 0x00;

    // Credit
    DispPattern[86].pattern_head[0] = 0x00;
    DispPattern[86].pattern_head[1] = 0xC0;
    DispPattern[86].pattern_head[2] = 0x30;
    DispPattern[86].pattern_head[3] = 0xC8;
    DispPattern[86].pattern_head[4] = 0x24;
    DispPattern[86].pattern_head[5] = 0x24;
    DispPattern[86].pattern_head[6] = 0x42;
    DispPattern[86].pattern_head[7] = 0x02;
    DispPattern[86].pattern_head[8] = 0xE2;
    DispPattern[86].pattern_head[9] = 0x22;
    DispPattern[86].pattern_head[10] = 0x24;
    DispPattern[86].pattern_head[11] = 0xC4;
    DispPattern[86].pattern_head[12] = 0x08;
    DispPattern[86].pattern_head[13] = 0x30;
    DispPattern[86].pattern_head[14] = 0xC0;
    DispPattern[86].pattern_head[15] = 0x00;
    DispPattern[86].pattern_foot[0] = 0x00;
    DispPattern[86].pattern_foot[1] = 0x03;
    DispPattern[86].pattern_foot[2] = 0x0C;
    DispPattern[86].pattern_foot[3] = 0x13;
    DispPattern[86].pattern_foot[4] = 0x24;
    DispPattern[86].pattern_foot[5] = 0x24;
    DispPattern[86].pattern_foot[6] = 0x42;
    DispPattern[86].pattern_foot[7] = 0x40;
    DispPattern[86].pattern_foot[8] = 0x47;
    DispPattern[86].pattern_foot[9] = 0x41;
    DispPattern[86].pattern_foot[10] = 0x21;
    DispPattern[86].pattern_foot[11] = 0x26;
    DispPattern[86].pattern_foot[12] = 0x10;
    DispPattern[86].pattern_foot[13] = 0x0C;
    DispPattern[86].pattern_foot[14] = 0x03;
    DispPattern[86].pattern_foot[15] = 0x00;

    DispPattern[87].pattern_head[0] = 0x00;
    DispPattern[87].pattern_head[1] = 0xC0;
    DispPattern[87].pattern_head[2] = 0xF0;
    DispPattern[87].pattern_head[3] = 0x38;
    DispPattern[87].pattern_head[4] = 0xDC;
    DispPattern[87].pattern_head[5] = 0xDC;
    DispPattern[87].pattern_head[6] = 0xBE;
    DispPattern[87].pattern_head[7] = 0xFE;
    DispPattern[87].pattern_head[8] = 0x1E;
    DispPattern[87].pattern_head[9] = 0xDE;
    DispPattern[87].pattern_head[10] = 0xDC;
    DispPattern[87].pattern_head[11] = 0x3C;
    DispPattern[87].pattern_head[12] = 0xF8;
    DispPattern[87].pattern_head[13] = 0xF0;
    DispPattern[87].pattern_head[14] = 0xC0;
    DispPattern[87].pattern_head[15] = 0x00;
    DispPattern[87].pattern_foot[0] = 0x00;
    DispPattern[87].pattern_foot[1] = 0x03;
    DispPattern[87].pattern_foot[2] = 0x0F;
    DispPattern[87].pattern_foot[3] = 0x1C;
    DispPattern[87].pattern_foot[4] = 0x3B;
    DispPattern[87].pattern_foot[5] = 0x3B;
    DispPattern[87].pattern_foot[6] = 0x7D;
    DispPattern[87].pattern_foot[7] = 0x7F;
    DispPattern[87].pattern_foot[8] = 0x78;
    DispPattern[87].pattern_foot[9] = 0x7E;
    DispPattern[87].pattern_foot[10] = 0x3E;
    DispPattern[87].pattern_foot[11] = 0x39;
    DispPattern[87].pattern_foot[12] = 0x1F;
    DispPattern[87].pattern_foot[13] = 0x0F;
    DispPattern[87].pattern_foot[14] = 0x03;
    DispPattern[87].pattern_foot[15] = 0x00;

   // Start
    DispPattern[88].pattern_head[0] = 0x00;
    DispPattern[88].pattern_head[1] = 0xC0;
    DispPattern[88].pattern_head[2] = 0x30;
    DispPattern[88].pattern_head[3] = 0x08;
    DispPattern[88].pattern_head[4] = 0xC4;
    DispPattern[88].pattern_head[5] = 0x24;
    DispPattern[88].pattern_head[6] = 0x22;
    DispPattern[88].pattern_head[7] = 0x42;
    DispPattern[88].pattern_head[8] = 0x02;
    DispPattern[88].pattern_head[9] = 0x22;
    DispPattern[88].pattern_head[10] = 0xE4;
    DispPattern[88].pattern_head[11] = 0x24;
    DispPattern[88].pattern_head[12] = 0x08;
    DispPattern[88].pattern_head[13] = 0x30;
    DispPattern[88].pattern_head[14] = 0xC0;
    DispPattern[88].pattern_head[15] = 0x00;
    DispPattern[88].pattern_foot[0] = 0x00;
    DispPattern[88].pattern_foot[1] = 0x03;
    DispPattern[88].pattern_foot[2] = 0x0C;
    DispPattern[88].pattern_foot[3] = 0x10;
    DispPattern[88].pattern_foot[4] = 0x24;
    DispPattern[88].pattern_foot[5] = 0x29;
    DispPattern[88].pattern_foot[6] = 0x49;
    DispPattern[88].pattern_foot[7] = 0x46;
    DispPattern[88].pattern_foot[8] = 0x40;
    DispPattern[88].pattern_foot[9] = 0x40;
    DispPattern[88].pattern_foot[10] = 0x2F;
    DispPattern[88].pattern_foot[11] = 0x20;
    DispPattern[88].pattern_foot[12] = 0x10;
    DispPattern[88].pattern_foot[13] = 0x0C;
    DispPattern[88].pattern_foot[14] = 0x03;
    DispPattern[88].pattern_foot[15] = 0x00;

    DispPattern[89].pattern_head[0] = 0x00;
    DispPattern[89].pattern_head[1] = 0xC0;
    DispPattern[89].pattern_head[2] = 0xF0;
    DispPattern[89].pattern_head[3] = 0xF8;
    DispPattern[89].pattern_head[4] = 0x3C;
    DispPattern[89].pattern_head[5] = 0xDC;
    DispPattern[89].pattern_head[6] = 0xDE;
    DispPattern[89].pattern_head[7] = 0xBE;
    DispPattern[89].pattern_head[8] = 0xFE;
    DispPattern[89].pattern_head[9] = 0xDE;
    DispPattern[89].pattern_head[10] = 0x1C;
    DispPattern[89].pattern_head[11] = 0xDC;
    DispPattern[89].pattern_head[12] = 0xF8;
    DispPattern[89].pattern_head[13] = 0xF0;
    DispPattern[89].pattern_head[14] = 0xC0;
    DispPattern[89].pattern_head[15] = 0x00;
    DispPattern[89].pattern_foot[0] = 0x00;
    DispPattern[89].pattern_foot[1] = 0x03;
    DispPattern[89].pattern_foot[2] = 0x0F;
    DispPattern[89].pattern_foot[3] = 0x1F;
    DispPattern[89].pattern_foot[4] = 0x3B;
    DispPattern[89].pattern_foot[5] = 0x36;
    DispPattern[89].pattern_foot[6] = 0x76;
    DispPattern[89].pattern_foot[7] = 0x79;
    DispPattern[89].pattern_foot[8] = 0x7F;
    DispPattern[89].pattern_foot[9] = 0x7F;
    DispPattern[89].pattern_foot[10] = 0x30;
    DispPattern[89].pattern_foot[11] = 0x3F;
    DispPattern[89].pattern_foot[12] = 0x1F;
    DispPattern[89].pattern_foot[13] = 0x0F;
    DispPattern[89].pattern_foot[14] = 0x03;
    DispPattern[89].pattern_foot[15] = 0x00;

    // 〇
    DispPattern[90].pattern_head[0] = 0x00;
    DispPattern[90].pattern_head[1] = 0xC0;
    DispPattern[90].pattern_head[2] = 0x30;
    DispPattern[90].pattern_head[3] = 0x08;
    DispPattern[90].pattern_head[4] = 0x04;
    DispPattern[90].pattern_head[5] = 0x04;
    DispPattern[90].pattern_head[6] = 0x02;
    DispPattern[90].pattern_head[7] = 0x02;
    DispPattern[90].pattern_head[8] = 0x02;
    DispPattern[90].pattern_head[9] = 0x02;
    DispPattern[90].pattern_head[10] = 0x04;
    DispPattern[90].pattern_head[11] = 0x04;
    DispPattern[90].pattern_head[12] = 0x08;
    DispPattern[90].pattern_head[13] = 0x30;
    DispPattern[90].pattern_head[14] = 0xC0;
    DispPattern[90].pattern_head[15] = 0x00;
    DispPattern[90].pattern_foot[0] = 0x00;
    DispPattern[90].pattern_foot[1] = 0x03;
    DispPattern[90].pattern_foot[2] = 0x0C;
    DispPattern[90].pattern_foot[3] = 0x10;
    DispPattern[90].pattern_foot[4] = 0x20;
    DispPattern[90].pattern_foot[5] = 0x20;
    DispPattern[90].pattern_foot[6] = 0x40;
    DispPattern[90].pattern_foot[7] = 0x40;
    DispPattern[90].pattern_foot[8] = 0x40;
    DispPattern[90].pattern_foot[9] = 0x40;
    DispPattern[90].pattern_foot[10] = 0x20;
    DispPattern[90].pattern_foot[11] = 0x20;
    DispPattern[90].pattern_foot[12] = 0x10;
    DispPattern[90].pattern_foot[13] = 0x0C;
    DispPattern[90].pattern_foot[14] = 0x03;
    DispPattern[90].pattern_foot[15] = 0x00;

    // ●
    DispPattern[91].pattern_head[0] = 0x00;
    DispPattern[91].pattern_head[1] = 0xC0;
    DispPattern[91].pattern_head[2] = 0xF0;
    DispPattern[91].pattern_head[3] = 0xF8;
    DispPattern[91].pattern_head[4] = 0xFC;
    DispPattern[91].pattern_head[5] = 0xFC;
    DispPattern[91].pattern_head[6] = 0xFE;
    DispPattern[91].pattern_head[7] = 0xFE;
    DispPattern[91].pattern_head[8] = 0xFE;
    DispPattern[91].pattern_head[9] = 0xFE;
    DispPattern[91].pattern_head[10] = 0xFC;
    DispPattern[91].pattern_head[11] = 0xFC;
    DispPattern[91].pattern_head[12] = 0xF8;
    DispPattern[91].pattern_head[13] = 0xF0;
    DispPattern[91].pattern_head[14] = 0xC0;
    DispPattern[91].pattern_head[15] = 0x00;
    DispPattern[91].pattern_foot[0] = 0x00;
    DispPattern[91].pattern_foot[1] = 0x03;
    DispPattern[91].pattern_foot[2] = 0x0F;
    DispPattern[91].pattern_foot[3] = 0x1F;
    DispPattern[91].pattern_foot[4] = 0x3F;
    DispPattern[91].pattern_foot[5] = 0x3F;
    DispPattern[91].pattern_foot[6] = 0x7F;
    DispPattern[91].pattern_foot[7] = 0x7F;
    DispPattern[91].pattern_foot[8] = 0x7F;
    DispPattern[91].pattern_foot[9] = 0x7F;
    DispPattern[91].pattern_foot[10] = 0x3F;
    DispPattern[91].pattern_foot[11] = 0x3F;
    DispPattern[91].pattern_foot[12] = 0x1F;
    DispPattern[91].pattern_foot[13] = 0x0F;
    DispPattern[91].pattern_foot[14] = 0x03;
    DispPattern[91].pattern_foot[15] = 0x00;

    // →
    DispPattern[92].pattern_head[0] = 0xE0;
    DispPattern[92].pattern_head[1] = 0x20;
    DispPattern[92].pattern_head[2] = 0x20;
    DispPattern[92].pattern_head[3] = 0x20;
    DispPattern[92].pattern_head[4] = 0x20;
    DispPattern[92].pattern_head[5] = 0x20;
    DispPattern[92].pattern_head[6] = 0x20;
    DispPattern[92].pattern_head[7] = 0x20;
    DispPattern[92].pattern_head[8] = 0x20;
    DispPattern[92].pattern_head[9] = 0x20;
    DispPattern[92].pattern_head[10] = 0x3C;
    DispPattern[92].pattern_head[11] = 0x08;
    DispPattern[92].pattern_head[12] = 0x10;
    DispPattern[92].pattern_head[13] = 0x20;
    DispPattern[92].pattern_head[14] = 0x40;
    DispPattern[92].pattern_head[15] = 0x80;
    DispPattern[92].pattern_foot[0] = 0x03;
    DispPattern[92].pattern_foot[1] = 0x02;
    DispPattern[92].pattern_foot[2] = 0x02;
    DispPattern[92].pattern_foot[3] = 0x02;
    DispPattern[92].pattern_foot[4] = 0x02;
    DispPattern[92].pattern_foot[5] = 0x02;
    DispPattern[92].pattern_foot[6] = 0x02;
    DispPattern[92].pattern_foot[7] = 0x02;
    DispPattern[92].pattern_foot[8] = 0x02;
    DispPattern[92].pattern_foot[9] = 0x02;
    DispPattern[92].pattern_foot[10] = 0x1E;
    DispPattern[92].pattern_foot[11] = 0x08;
    DispPattern[92].pattern_foot[12] = 0x04;
    DispPattern[92].pattern_foot[13] = 0x02;
    DispPattern[92].pattern_foot[14] = 0x01;
    DispPattern[92].pattern_foot[15] = 0x00;

    // →選択
    DispPattern[93].pattern_head[0] = 0xE0;
    DispPattern[93].pattern_head[1] = 0xE0;
    DispPattern[93].pattern_head[2] = 0xE0;
    DispPattern[93].pattern_head[3] = 0xE0;
    DispPattern[93].pattern_head[4] = 0xE0;
    DispPattern[93].pattern_head[5] = 0xE0;
    DispPattern[93].pattern_head[6] = 0xE0;
    DispPattern[93].pattern_head[7] = 0xE0;
    DispPattern[93].pattern_head[8] = 0xE0;
    DispPattern[93].pattern_head[9] = 0xE0;
    DispPattern[93].pattern_head[10] = 0xFC;
    DispPattern[93].pattern_head[11] = 0xF8;
    DispPattern[93].pattern_head[12] = 0xF0;
    DispPattern[93].pattern_head[13] = 0xE0;
    DispPattern[93].pattern_head[14] = 0xC0;
    DispPattern[93].pattern_head[15] = 0x80;
    DispPattern[93].pattern_foot[0] = 0x03;
    DispPattern[93].pattern_foot[1] = 0x03;
    DispPattern[93].pattern_foot[2] = 0x03;
    DispPattern[93].pattern_foot[3] = 0x03;
    DispPattern[93].pattern_foot[4] = 0x03;
    DispPattern[93].pattern_foot[5] = 0x03;
    DispPattern[93].pattern_foot[6] = 0x03;
    DispPattern[93].pattern_foot[7] = 0x03;
    DispPattern[93].pattern_foot[8] = 0x03;
    DispPattern[93].pattern_foot[9] = 0x03;
    DispPattern[93].pattern_foot[10] = 0x1F;
    DispPattern[93].pattern_foot[11] = 0x0F;
    DispPattern[93].pattern_foot[12] = 0x07;
    DispPattern[93].pattern_foot[13] = 0x03;
    DispPattern[93].pattern_foot[14] = 0x01;
    DispPattern[93].pattern_foot[15] = 0x00;

    // ←
    DispPattern[94].pattern_head[0] = 0x80;
    DispPattern[94].pattern_head[1] = 0x40;
    DispPattern[94].pattern_head[2] = 0x20;
    DispPattern[94].pattern_head[3] = 0x10;
    DispPattern[94].pattern_head[4] = 0x08;
    DispPattern[94].pattern_head[5] = 0x3C;
    DispPattern[94].pattern_head[6] = 0x20;
    DispPattern[94].pattern_head[7] = 0x20;
    DispPattern[94].pattern_head[8] = 0x20;
    DispPattern[94].pattern_head[9] = 0x20;
    DispPattern[94].pattern_head[10] = 0x20;
    DispPattern[94].pattern_head[11] = 0x20;
    DispPattern[94].pattern_head[12] = 0x20;
    DispPattern[94].pattern_head[13] = 0x20;
    DispPattern[94].pattern_head[14] = 0x20;
    DispPattern[94].pattern_head[15] = 0xE0;
    DispPattern[94].pattern_foot[0] = 0x00;
    DispPattern[94].pattern_foot[1] = 0x01;
    DispPattern[94].pattern_foot[2] = 0x02;
    DispPattern[94].pattern_foot[3] = 0x04;
    DispPattern[94].pattern_foot[4] = 0x08;
    DispPattern[94].pattern_foot[5] = 0x1E;
    DispPattern[94].pattern_foot[6] = 0x02;
    DispPattern[94].pattern_foot[7] = 0x02;
    DispPattern[94].pattern_foot[8] = 0x02;
    DispPattern[94].pattern_foot[9] = 0x02;
    DispPattern[94].pattern_foot[10] = 0x02;
    DispPattern[94].pattern_foot[11] = 0x02;
    DispPattern[94].pattern_foot[12] = 0x02;
    DispPattern[94].pattern_foot[13] = 0x02;
    DispPattern[94].pattern_foot[14] = 0x02;
    DispPattern[94].pattern_foot[15] = 0x03;

    // ←選択
    DispPattern[95].pattern_head[0] = 0x80;
    DispPattern[95].pattern_head[1] = 0xC0;
    DispPattern[95].pattern_head[2] = 0xE0;
    DispPattern[95].pattern_head[3] = 0xF0;
    DispPattern[95].pattern_head[4] = 0xF8;
    DispPattern[95].pattern_head[5] = 0xFC;
    DispPattern[95].pattern_head[6] = 0xE0;
    DispPattern[95].pattern_head[7] = 0xE0;
    DispPattern[95].pattern_head[8] = 0xE0;
    DispPattern[95].pattern_head[9] = 0xE0;
    DispPattern[95].pattern_head[10] = 0xE0;
    DispPattern[95].pattern_head[11] = 0xE0;
    DispPattern[95].pattern_head[12] = 0xE0;
    DispPattern[95].pattern_head[13] = 0xE0;
    DispPattern[95].pattern_head[14] = 0xE0;
    DispPattern[95].pattern_head[15] = 0xE0;
    DispPattern[95].pattern_foot[0] = 0x00;
    DispPattern[95].pattern_foot[1] = 0x01;
    DispPattern[95].pattern_foot[2] = 0x03;
    DispPattern[95].pattern_foot[3] = 0x07;
    DispPattern[95].pattern_foot[4] = 0x0F;
    DispPattern[95].pattern_foot[5] = 0x1F;
    DispPattern[95].pattern_foot[6] = 0x03;
    DispPattern[95].pattern_foot[7] = 0x03;
    DispPattern[95].pattern_foot[8] = 0x03;
    DispPattern[95].pattern_foot[9] = 0x03;
    DispPattern[95].pattern_foot[10] = 0x03;
    DispPattern[95].pattern_foot[11] = 0x03;
    DispPattern[95].pattern_foot[12] = 0x03;
    DispPattern[95].pattern_foot[13] = 0x03;
    DispPattern[95].pattern_foot[14] = 0x03;
    DispPattern[95].pattern_foot[15] = 0x03;

    // ↑
    DispPattern[96].pattern_head[0] = 0x00;
    DispPattern[96].pattern_head[1] = 0x00;
    DispPattern[96].pattern_head[2] = 0x20;
    DispPattern[96].pattern_head[3] = 0x30;
    DispPattern[96].pattern_head[4] = 0x28;
    DispPattern[96].pattern_head[5] = 0xE4;
    DispPattern[96].pattern_head[6] = 0x02;
    DispPattern[96].pattern_head[7] = 0x01;
    DispPattern[96].pattern_head[8] = 0x02;
    DispPattern[96].pattern_head[9] = 0xE4;
    DispPattern[96].pattern_head[10] = 0x28;
    DispPattern[96].pattern_head[11] = 0x30;
    DispPattern[96].pattern_head[12] = 0x20;
    DispPattern[96].pattern_head[13] = 0x00;
    DispPattern[96].pattern_head[14] = 0x00;
    DispPattern[96].pattern_head[15] = 0x00;
    DispPattern[96].pattern_foot[0] = 0x00;
    DispPattern[96].pattern_foot[1] = 0x00;
    DispPattern[96].pattern_foot[2] = 0x00;
    DispPattern[96].pattern_foot[3] = 0x00;
    DispPattern[96].pattern_foot[4] = 0x00;
    DispPattern[96].pattern_foot[5] = 0xFF;
    DispPattern[96].pattern_foot[6] = 0x80;
    DispPattern[96].pattern_foot[7] = 0x80;
    DispPattern[96].pattern_foot[8] = 0x80;
    DispPattern[96].pattern_foot[9] = 0xFF;
    DispPattern[96].pattern_foot[10] = 0x00;
    DispPattern[96].pattern_foot[11] = 0x00;
    DispPattern[96].pattern_foot[12] = 0x00;
    DispPattern[96].pattern_foot[13] = 0x00;
    DispPattern[96].pattern_foot[14] = 0x00;
    DispPattern[96].pattern_foot[15] = 0x00;

    // ↑選択
    DispPattern[97].pattern_head[0] = 0x00;
    DispPattern[97].pattern_head[1] = 0x00;
    DispPattern[97].pattern_head[2] = 0x20;
    DispPattern[97].pattern_head[3] = 0x30;
    DispPattern[97].pattern_head[4] = 0x38;
    DispPattern[97].pattern_head[5] = 0xFC;
    DispPattern[97].pattern_head[6] = 0xFE;
    DispPattern[97].pattern_head[7] = 0xFF;
    DispPattern[97].pattern_head[8] = 0xFE;
    DispPattern[97].pattern_head[9] = 0xFC;
    DispPattern[97].pattern_head[10] = 0x38;
    DispPattern[97].pattern_head[11] = 0x30;
    DispPattern[97].pattern_head[12] = 0x20;
    DispPattern[97].pattern_head[13] = 0x00;
    DispPattern[97].pattern_head[14] = 0x00;
    DispPattern[97].pattern_head[15] = 0x00;
    DispPattern[97].pattern_foot[0] = 0x00;
    DispPattern[97].pattern_foot[1] = 0x00;
    DispPattern[97].pattern_foot[2] = 0x00;
    DispPattern[97].pattern_foot[3] = 0x00;
    DispPattern[97].pattern_foot[4] = 0x00;
    DispPattern[97].pattern_foot[5] = 0xFF;
    DispPattern[97].pattern_foot[6] = 0xFF;
    DispPattern[97].pattern_foot[7] = 0xFF;
    DispPattern[97].pattern_foot[8] = 0xFF;
    DispPattern[97].pattern_foot[9] = 0xFF;
    DispPattern[97].pattern_foot[10] = 0x00;
    DispPattern[97].pattern_foot[11] = 0x00;
    DispPattern[97].pattern_foot[12] = 0x00;
    DispPattern[97].pattern_foot[13] = 0x00;
    DispPattern[97].pattern_foot[14] = 0x00;
    DispPattern[97].pattern_foot[15] = 0x00;

    // ↓
    DispPattern[98].pattern_head[0] = 0x00;
    DispPattern[98].pattern_head[1] = 0x00;
    DispPattern[98].pattern_head[2] = 0x00;
    DispPattern[98].pattern_head[3] = 0x00;
    DispPattern[98].pattern_head[4] = 0x00;
    DispPattern[98].pattern_head[5] = 0xFF;
    DispPattern[98].pattern_head[6] = 0x01;
    DispPattern[98].pattern_head[7] = 0x01;
    DispPattern[98].pattern_head[8] = 0x01;
    DispPattern[98].pattern_head[9] = 0xFF;
    DispPattern[98].pattern_head[10] = 0x00;
    DispPattern[98].pattern_head[11] = 0x00;
    DispPattern[98].pattern_head[12] = 0x00;
    DispPattern[98].pattern_head[13] = 0x00;
    DispPattern[98].pattern_head[14] = 0x00;
    DispPattern[98].pattern_head[15] = 0x00;
    DispPattern[98].pattern_foot[0] = 0x00;
    DispPattern[98].pattern_foot[1] = 0x00;
    DispPattern[98].pattern_foot[2] = 0x04;
    DispPattern[98].pattern_foot[3] = 0x0C;
    DispPattern[98].pattern_foot[4] = 0x14;
    DispPattern[98].pattern_foot[5] = 0x27;
    DispPattern[98].pattern_foot[6] = 0x40;
    DispPattern[98].pattern_foot[7] = 0x80;
    DispPattern[98].pattern_foot[8] = 0x40;
    DispPattern[98].pattern_foot[9] = 0x27;
    DispPattern[98].pattern_foot[10] = 0x14;
    DispPattern[98].pattern_foot[11] = 0x0C;
    DispPattern[98].pattern_foot[12] = 0x04;
    DispPattern[98].pattern_foot[13] = 0x00;
    DispPattern[98].pattern_foot[14] = 0x00;
    DispPattern[98].pattern_foot[15] = 0x00;

    // ↓選択
    DispPattern[99].pattern_head[0] = 0x00;
    DispPattern[99].pattern_head[1] = 0x00;
    DispPattern[99].pattern_head[2] = 0x00;
    DispPattern[99].pattern_head[3] = 0x00;
    DispPattern[99].pattern_head[4] = 0x00;
    DispPattern[99].pattern_head[5] = 0xFF;
    DispPattern[99].pattern_head[6] = 0xFF;
    DispPattern[99].pattern_head[7] = 0xFF;
    DispPattern[99].pattern_head[8] = 0xFF;
    DispPattern[99].pattern_head[9] = 0xFF;
    DispPattern[99].pattern_head[10] = 0x00;
    DispPattern[99].pattern_head[11] = 0x00;
    DispPattern[99].pattern_head[12] = 0x00;
    DispPattern[99].pattern_head[13] = 0x00;
    DispPattern[99].pattern_head[14] = 0x00;
    DispPattern[99].pattern_head[15] = 0x00;
    DispPattern[99].pattern_foot[0] = 0x00;
    DispPattern[99].pattern_foot[1] = 0x00;
    DispPattern[99].pattern_foot[2] = 0x04;
    DispPattern[99].pattern_foot[3] = 0x0C;
    DispPattern[99].pattern_foot[4] = 0x1C;
    DispPattern[99].pattern_foot[5] = 0x3F;
    DispPattern[99].pattern_foot[6] = 0x7F;
    DispPattern[99].pattern_foot[7] = 0xFF;
    DispPattern[99].pattern_foot[8] = 0x7F;
    DispPattern[99].pattern_foot[9] = 0x3F;
    DispPattern[99].pattern_foot[10] = 0x1C;
    DispPattern[99].pattern_foot[11] = 0x0C;
    DispPattern[99].pattern_foot[12] = 0x04;
    DispPattern[99].pattern_foot[13] = 0x00;
    DispPattern[99].pattern_foot[14] = 0x00;
    DispPattern[99].pattern_foot[15] = 0x00;

}
