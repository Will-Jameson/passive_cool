#ifndef LCD_TC1602A_H
#define LCD_TC1602A_H
#include "stm32f3xx_hal.h"  
#include <stdint.h>
// ─────────────────────────────────────────────
// PIN CONFIGURATION

// ─────────────────────────────────────────────
#define LCD_RS_PORT GPIOA
#define LCD_RS_PIN GPIO_PIN_0
#define LCD_E_PORT GPIOA
#define LCD_E_PIN GPIO_PIN_1
#define LCD_D4_PORT GPIOA
#define LCD_D4_PIN GPIO_PIN_4
#define LCD_D5_PORT GPIOA
#define LCD_D5_PIN GPIO_PIN_5
#define LCD_D6_PORT GPIOA
#define LCD_D6_PIN GPIO_PIN_6
#define LCD_D7_PORT GPIOA
#define LCD_D7_PIN GPIO_PIN_7
// ─────────────────────────────────────────────
// PUBLIC API
// ─────────────────────────────────────────────
void lcd_init(void);
void lcd_clear(void);
void lcd_home(void);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_print(const char *str);
void lcd_print_char(char c);
#endif // LCD_TC1602A_H
