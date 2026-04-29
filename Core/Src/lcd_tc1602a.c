#include "lcd_tc1602a.h"
// ─────────────────────────────────────────────
// HD44780 COMMANDS
// ─────────────────────────────────────────────
#define LCD_CLEAR 0x01
#define LCD_HOME 0x02
#define LCD_ENTRY_MODE 0x06 // Cursor moves right, no display shift
#define LCD_DISPLAY_ON 0x0C // Display on, cursor off, blink off
#define LCD_FUNCTION_SET 0x28 // 4-bit mode, 2 lines, 5x8 font
#define LCD_ROW2_OFFSET 0x40 // DDRAM address of second row
// ─────────────────────────────────────────────
// INTERNAL HELPERS
// ─────────────────────────────────────────────
// Pulse the Enable pin to latch data into the LCD
static void lcd_pulse_enable(void) {
HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_SET);
HAL_Delay(1);
HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_RESET);
HAL_Delay(1);
}
// Write 4 bits to D4-D7
static void lcd_write_nibble(uint8_t nibble) {
HAL_GPIO_WritePin(LCD_D4_PORT, LCD_D4_PIN, ((nibble >> 0) & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
HAL_GPIO_WritePin(LCD_D5_PORT, LCD_D5_PIN, ((nibble >> 1) & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
HAL_GPIO_WritePin(LCD_D6_PORT, LCD_D6_PIN, ((nibble >> 2) & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
HAL_GPIO_WritePin(LCD_D7_PORT, LCD_D7_PIN, ((nibble >> 3) & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
lcd_pulse_enable();
}
// Send a full byte as two nibbles (upper first, then lower)
// rs=0 → command, rs=1 → character data
static void lcd_send_byte(uint8_t byte, uint8_t rs) {
HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, rs);
lcd_write_nibble(byte >> 4); // Upper nibble
lcd_write_nibble(byte & 0x0F); // Lower nibble
HAL_Delay(2); // Most commands take <1.6ms
}
// ─────────────────────────────────────────────
// PUBLIC IMPLEMENTATION

// ─────────────────────────────────────────────
void lcd_init(void) {
HAL_Delay(100);
// Ensure E and RS start low
HAL_GPIO_WritePin(LCD_E_PORT,  LCD_E_PIN,  GPIO_PIN_RESET);
HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, GPIO_PIN_RESET);

// HD44780 4-bit init sequence — send 0x3 three times in 8-bit style
lcd_write_nibble(0x03); HAL_Delay(10);
lcd_write_nibble(0x03); HAL_Delay(5);
lcd_write_nibble(0x03); HAL_Delay(2);
lcd_write_nibble(0x02); HAL_Delay(2); // switch to 4-bit

// Standard init order from HD44780 datasheet
lcd_send_byte(LCD_FUNCTION_SET, 0); // 4-bit, 2 lines, 5x8
lcd_send_byte(0x08, 0);             // display OFF
lcd_send_byte(LCD_CLEAR, 0);        // clear display
HAL_Delay(5);                       // clear needs >1.52ms
lcd_send_byte(LCD_ENTRY_MODE, 0);   // cursor moves right
lcd_send_byte(LCD_DISPLAY_ON, 0);   // display ON
}
void lcd_clear(void) {
lcd_send_byte(LCD_CLEAR, 0);
HAL_Delay(2);
}
void lcd_home(void) {
lcd_send_byte(LCD_HOME, 0);
HAL_Delay(2);
}
void lcd_set_cursor(uint8_t row, uint8_t col) {
    if (row > 1) row = 1;
    if (col > 15) col = 15;

    uint8_t address = col + (row == 1 ? LCD_ROW2_OFFSET : 0x00);
    lcd_send_byte(0x80 | address, 0);
}
void lcd_print_char(char c) {
lcd_send_byte((uint8_t)c, 1); // RS=1 means data, not command
}
void lcd_print(const char *str) {
while (*str) {
lcd_print_char(*str++);
}
}