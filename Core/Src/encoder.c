#include "encoder.h"

// _______________________________________
// Internal State
// _______________________________________

static TIM_HandleTypeDef *encoder_htim;
static EncoderMode_t current_mode = ENCODER_MODE_DISABLED;
static EncoderMode_t mode_changed = 0;
#define DEBOUNCE_DELAY_MS 50
static uint8_t  btn_raw_last   = 1;  // last raw reading
static uint8_t  btn_debounced  = 1;  // stable debounced state
static uint32_t btn_change_tick = 0; // time of last raw change

// _______________________________________
// CUBEMX TIMER SETUP:
// -Combined channels: Encoder Mode
// - Encoder Mode: Encoder Mode T1 (counts clock edges only)
// - Prescaler: 0 (count every edge)
// - Counter Period (ARR): 39 (0-39 counts = 40 counts per revolution)
// -CLK -> (TIM3_CH1) -> A pin of encoder
// DT -> (TIM3_CH2) -> B pin of encoder
// SW -> GPIO input with pullup,
// _______________________________________
static float accumulated_angle = 90.0f;
static int32_t last_counter = 0;

void encoder_init(TIM_HandleTypeDef *htim) {
    encoder_htim = htim;
    HAL_TIM_Encoder_Start(encoder_htim, TIM_CHANNEL_3);

    __HAL_TIM_SET_COUNTER(encoder_htim, 0);

}

void encoder_update(void) {
    mode_changed = 0;

    uint8_t btn_raw = HAL_GPIO_ReadPin(ENCODER_SW_PORT, ENCODER_SW_PIN);
    uint32_t now = HAL_GetTick();

    // Any raw change resets the stability timer
    if (btn_raw != btn_raw_last) {
        btn_change_tick = now;
        btn_raw_last = btn_raw;
    }



    // Only act once input has been stable for the full debounce window
    if ((now - btn_change_tick) >= DEBOUNCE_DELAY_MS && btn_raw != btn_debounced) {
        btn_debounced = btn_raw;
        if (btn_debounced == GPIO_PIN_RESET) {
            current_mode = (current_mode == ENCODER_MODE_DISABLED)
                ? ENCODER_MODE_ENABLED
                : ENCODER_MODE_DISABLED;
            mode_changed = 1;
        }

    }

    // --- delta tracking ---
    int32_t counter = (int32_t)__HAL_TIM_GET_COUNTER(encoder_htim);
    int32_t delta = counter - last_counter;

    if (current_mode == ENCODER_MODE_DISABLED) return; // 

    // handle wraparound at ARR boundary
    if (delta >  (ENCODER_MAX_COUNT / 2)) delta -= ENCODER_MAX_COUNT;
    if (delta < -(ENCODER_MAX_COUNT / 2)) delta += ENCODER_MAX_COUNT;

    last_counter = counter;

    accumulated_angle += delta * ANGLE_STEP_PER_COUNT;
    if (accumulated_angle < 0.0f)   accumulated_angle = 0.0f;
    if (accumulated_angle > 180.0f) accumulated_angle = 180.0f;
}

float encoder_get_angle(void) {
    return accumulated_angle;
}


EncoderMode_t encoder_get_mode(void) {
    return current_mode;
}   

uint8_t encoder_mode_changed(void) {
    return mode_changed;
}


