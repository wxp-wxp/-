#include "main.h"
#define H GPIO_PIN_SET
#define L GPIO_PIN_RESET
#define out 0x12
#define in  0x13
#define yack 0x14
#define nack 0x15
#define led2 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6)
#define	led3 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7)
const uint8_t F16x16[];
const uint8_t DPH[];
const uint8_t DPL[];
const uint8_t english[];
const uint8_t  F10x10[];
void delay_1(void);
void delay_bus(uint16_t dat);
void delay_5us(void);
void SDA_write(uint8_t dat);
void SCL_write(uint8_t dat);
void OLED_start(void);
void OLED_stop(void);
void OLED_write_byte(uint8_t dat);
void OLED_write_dat(uint8_t dat);
void OLED_write_cmd(uint8_t dat);
void OLED_cls(void);
void OLED_set_pos(uint8_t x,uint8_t y);
void OLED_fill(uint8_t dat);
void OLED_init(void);
void OLED_english(uint8_t x,uint8_t y,uint8_t n);
void OLED_16X16(uint8_t x,uint8_t y,uint8_t n);
void OLED_H(uint8_t x,uint8_t y,uint8_t n);
void OLED_L(uint8_t x,uint8_t y,uint8_t n);

const uint8_t DPH[] = {0x00,0x00,0x00,0xFE,0x02,0x02,0x02,0x02,0x02,0x02,0x02,
0x02,0xFE,0x00,0x00,0x00};
const uint8_t DPL[] = {0x00,0x00,0x00,0x7E,0x40,0x40,0x40,0x40,0x40,0x40,0x40,
0x40,0x7E,0x00,0x00,0x00};
const uint8_t F16x16[] = {
		
0x00,0x00,0xE0,0xF0,0x30,0x08,0x08,0x08,0x08,0x08,0x08,0x30,0xF0,0xE0,0x00,0x00,
0x00,0x00,0x0F,0x1F,0x18,0x30,0x20,0x20,0x20,0x20,0x20,0x18,0x0F,0x07,0x00,0x00,/*"0",0*/

0x00,0x00,0x00,0x00,0x00,0x10,0x10,0xF0,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x20,0x20,0x20,0x3F,0x3F,0x20,0x20,0x20,0x20,0x00,0x00,0x00,/*"1",1*/

0x00,0x00,0x30,0x70,0x28,0x08,0x08,0x08,0x08,0x08,0x08,0xD8,0xF0,0x20,0x00,0x00,
0x00,0x00,0x30,0x30,0x28,0x24,0x24,0x22,0x22,0x21,0x21,0x20,0x30,0x18,0x00,0x00,/*"2",2*/

0x00,0x00,0x30,0x30,0x28,0x08,0x08,0x08,0x08,0x88,0x88,0x70,0x70,0x00,0x00,0x00,
0x00,0x00,0x18,0x18,0x28,0x20,0x20,0x21,0x21,0x21,0x21,0x13,0x1E,0x0C,0x00,0x00,/*"3",3*/

0x00,0x00,0x00,0x00,0x80,0x80,0x40,0x20,0x10,0xF0,0xF8,0xF8,0x00,0x00,0x00,0x00,
0x00,0x04,0x06,0x05,0x04,0x04,0x24,0x24,0x24,0x3F,0x3F,0x3F,0x24,0x24,0x24,0x00,/*"4",4*/

0x00,0x00,0x00,0xF8,0x08,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x08,0x00,0x00,0x00,
0x00,0x00,0x18,0x19,0x29,0x20,0x20,0x20,0x20,0x20,0x20,0x11,0x1F,0x0E,0x00,0x00,/*"5",5*/

0x00,0x00,0xC0,0xE0,0x10,0x88,0x88,0x88,0x88,0x88,0x88,0x98,0x10,0x00,0x00,0x00,
0x00,0x00,0x0F,0x1F,0x11,0x20,0x20,0x20,0x20,0x20,0x20,0x10,0x1F,0x0F,0x00,0x00,/*"6",6*/

0x00,0x00,0x30,0x18,0x08,0x08,0x08,0x08,0x08,0x88,0x48,0x28,0x18,0x08,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x3C,0x3E,0x13,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"7",7*/

0x00,0x00,0x70,0x70,0xC8,0x88,0x88,0x88,0x08,0x08,0x88,0x88,0x70,0x20,0x00,0x00,
0x00,0x0C,0x1E,0x12,0x21,0x21,0x20,0x21,0x21,0x21,0x23,0x22,0x1E,0x0C,0x00,0x00,/*"8",8*/

0x00,0x40,0xF0,0xF0,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x90,0xF0,0xE0,0x00,0x00,
0x00,0x00,0x11,0x11,0x33,0x22,0x22,0x22,0x22,0x22,0x11,0x1C,0x0F,0x07,0x00,0x00,/*"9",9*/

0x00,0x00,0xF8,0x88,0x88,0x88,0x88,0xFF,0x88,0x88,0x88,0x88,0xF8,0x00,0x00,0x00,
0x00,0x00,0x1F,0x08,0x08,0x08,0x08,0x7F,0x88,0x88,0x88,0x88,0x9F,0x80,0xF0,0x00,/*��*/
	
0x00,0x00,0xFE,0x02,0x82,0x82,0x82,0x82,0xFA,0x82,0x82,0x82,0x82,0x82,0x02,0x00,	
0x80,0x60,0x1F,0x40,0x40,0x40,0x40,0x40,0x7F,0x40,0x40,0x44,0x58,0x40,0x40,0x00,/*ѹ*/
	
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x36,0x36,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*:*/

0x00,0x08,0x08,0x38,0xF8,0xC0,0x00,0x00,0x00,0x00,0x80,0x60,0x18,0x08,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x03,0x0F,0x3C,0x18,0x06,0x01,0x00,0x00,0x00,0x00,0x00,/*V*/

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0xE0,0xE0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*.*/

  };
const uint8_t english[]={
	
	
	
	

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*" ",0*/

0x00,0x00,0x00,0x7F,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x06,0x00,0x00,0x00,0x00,/*"!",1*/

0x00,0x02,0x01,0x00,0x02,0x01,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*""",2*/

0x00,0x88,0xF8,0x8F,0x88,0xF8,0x8F,0x00,
0x00,0x00,0x07,0x00,0x00,0x07,0x00,0x00,/*"#",3*/

0x00,0x0E,0x11,0x11,0xFF,0x21,0xC6,0x00,
0x00,0x03,0x04,0x04,0x1F,0x04,0x03,0x00,/*"$",4*/

0x1E,0x21,0x9E,0x70,0xCC,0x23,0xC0,0x00,
0x00,0x06,0x01,0x00,0x03,0x04,0x03,0x00,/*"%",5*/

0xC0,0x3E,0x61,0x91,0x2E,0xE0,0x20,0x00,
0x03,0x04,0x04,0x05,0x03,0x04,0x04,0x02,/*"&",6*/

0x00,0x02,0x01,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"'",7*/

0x00,0x00,0x00,0xFC,0x03,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x03,0x04,0x08,0x00,/*"(",8*/

0x00,0x00,0x00,0x03,0xFC,0x00,0x00,0x00,
0x00,0x08,0x04,0x03,0x00,0x00,0x00,0x00,/*")",9*/

0x48,0x48,0x30,0xFE,0x30,0x48,0x48,0x00,
0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,/*"*",10*/

0x00,0x20,0x20,0x20,0xFC,0x20,0x20,0x20,
0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,/*"+",11*/

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x12,0x0E,0x00,0x00,0x00,0x00,0x00,/*",",12*/

0x00,0x20,0x20,0x20,0x20,0x20,0x20,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"-",13*/

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x06,0x06,0x00,0x00,0x00,0x00,0x00,/*".",14*/

0x00,0x00,0x00,0xE0,0x18,0x07,0x00,0x00,
0x00,0x0C,0x03,0x00,0x00,0x00,0x00,0x00,/*"/",15*/

0x00,0xFC,0x02,0x01,0x01,0x02,0xFC,0x00,
0x00,0x01,0x02,0x04,0x04,0x02,0x01,0x00,/*"0",16*/

0x00,0x00,0x02,0x02,0xFF,0x00,0x00,0x00,
0x00,0x00,0x04,0x04,0x07,0x04,0x04,0x00,/*"1",17*/

0x00,0x0E,0x01,0x81,0x41,0x21,0x1E,0x00,
0x00,0x06,0x05,0x04,0x04,0x04,0x06,0x00,/*"2",18*/

0x00,0x06,0x01,0x21,0x21,0x51,0x8E,0x00,
0x00,0x03,0x04,0x04,0x04,0x04,0x03,0x00,/*"3",19*/

0x00,0xC0,0xB0,0x88,0x86,0xFF,0x80,0x80,
0x00,0x00,0x00,0x04,0x04,0x07,0x04,0x04,/*"4",20*/

0x00,0x3F,0x11,0x11,0x11,0x21,0xC1,0x00,
0x00,0x03,0x04,0x04,0x04,0x02,0x01,0x00,/*"5",21*/

0x00,0xFC,0x22,0x11,0x11,0x12,0xE0,0x00,
0x00,0x01,0x02,0x04,0x04,0x04,0x03,0x00,/*"6",22*/

0x00,0x03,0x01,0xC1,0x31,0x0D,0x03,0x00,
0x00,0x00,0x00,0x07,0x00,0x00,0x00,0x00,/*"7",23*/

0x00,0x8E,0x51,0x21,0x21,0x51,0x8E,0x00,
0x00,0x03,0x04,0x04,0x04,0x04,0x03,0x00,/*"8",24*/

0x00,0x3E,0x41,0x41,0x41,0x22,0xFC,0x00,
0x00,0x00,0x02,0x04,0x04,0x02,0x01,0x00,/*"9",25*/

0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,
0x00,0x00,0x00,0x06,0x06,0x00,0x00,0x00,/*":",26*/

0x00,0x00,0x00,0x10,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x1C,0x00,0x00,0x00,0x00,/*";",27*/

0x00,0x20,0x50,0x88,0x04,0x02,0x01,0x00,
0x00,0x00,0x00,0x00,0x01,0x02,0x04,0x00,/*"<",28*/

0x00,0x48,0x48,0x48,0x48,0x48,0x48,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"=",29*/

0x00,0x01,0x02,0x04,0x88,0x50,0x20,0x00,
0x00,0x04,0x02,0x01,0x00,0x00,0x00,0x00,/*">",30*/

0x00,0x0E,0x09,0x01,0xE1,0x11,0x0E,0x00,
0x00,0x00,0x00,0x06,0x06,0x00,0x00,0x00,/*"?",31*/

0xF8,0x06,0xF9,0x05,0xFD,0x02,0xFC,0x00,
0x00,0x03,0x04,0x05,0x05,0x05,0x02,0x00,/*"@",32*/

0x00,0x80,0x78,0x47,0x5C,0xE0,0x00,0x00,
0x04,0x07,0x04,0x00,0x00,0x04,0x07,0x04,/*"A",33*/

0x01,0xFF,0x11,0x11,0x11,0x2E,0xC0,0x00,
0x04,0x07,0x04,0x04,0x04,0x02,0x01,0x00,/*"B",34*/

0xF8,0x06,0x01,0x01,0x01,0x01,0x07,0x00,
0x00,0x03,0x04,0x04,0x04,0x02,0x01,0x00,/*"C",35*/

0x01,0xFF,0x01,0x01,0x01,0x02,0xFC,0x00,
0x04,0x07,0x04,0x04,0x04,0x02,0x01,0x00,/*"D",36*/

0x01,0xFF,0x11,0x11,0x7D,0x01,0x02,0x00,
0x04,0x07,0x04,0x04,0x04,0x04,0x03,0x00,/*"E",37*/

0x01,0xFF,0x11,0x11,0x7D,0x01,0x02,0x00,
0x04,0x07,0x04,0x00,0x00,0x00,0x00,0x00,/*"F",38*/

0xF8,0x06,0x01,0x01,0x41,0xC7,0x40,0x00,
0x00,0x03,0x04,0x04,0x04,0x03,0x00,0x00,/*"G",39*/

0x01,0xFF,0x21,0x20,0x20,0x21,0xFF,0x01,
0x04,0x07,0x04,0x00,0x00,0x04,0x07,0x04,/*"H",40*/

0x00,0x01,0x01,0xFF,0x01,0x01,0x00,0x00,
0x00,0x04,0x04,0x07,0x04,0x04,0x00,0x00,/*"I",41*/

0x00,0x00,0x01,0x01,0xFF,0x01,0x01,0x00,
0x18,0x10,0x10,0x10,0x0F,0x00,0x00,0x00,/*"J",42*/

0x01,0xFF,0x11,0x38,0xC5,0x03,0x01,0x00,
0x04,0x07,0x04,0x00,0x04,0x07,0x04,0x00,/*"K",43*/

0x01,0xFF,0x01,0x00,0x00,0x00,0x00,0x00,
0x04,0x07,0x04,0x04,0x04,0x04,0x06,0x00,/*"L",44*/

0x01,0xFF,0x3F,0xC0,0x3F,0xFF,0x01,0x00,
0x04,0x07,0x00,0x07,0x00,0x07,0x04,0x00,/*"M",45*/

0x01,0xFF,0x06,0x18,0xE0,0x01,0xFF,0x01,
0x04,0x07,0x04,0x00,0x00,0x03,0x07,0x00,/*"N",46*/

0xFC,0x02,0x01,0x01,0x01,0x02,0xFC,0x00,
0x01,0x02,0x04,0x04,0x04,0x02,0x01,0x00,/*"O",47*/

0x01,0xFF,0x21,0x21,0x21,0x21,0x1E,0x00,
0x04,0x07,0x04,0x00,0x00,0x00,0x00,0x00,/*"P",48*/

0xFC,0x02,0x01,0x01,0x01,0x02,0xFC,0x00,
0x01,0x02,0x05,0x05,0x06,0x0A,0x09,0x00,/*"Q",49*/

0x01,0xFF,0x11,0x11,0x71,0x91,0x0E,0x00,
0x04,0x07,0x04,0x00,0x00,0x01,0x06,0x04,/*"R",50*/

0x00,0x0E,0x11,0x21,0x21,0x41,0x87,0x00,
0x00,0x07,0x04,0x04,0x04,0x04,0x03,0x00,/*"S",51*/

0x03,0x01,0x01,0xFF,0x01,0x01,0x03,0x00,
0x00,0x00,0x04,0x07,0x04,0x00,0x00,0x00,/*"T",52*/

0x01,0xFF,0x01,0x00,0x00,0x01,0xFF,0x01,
0x00,0x03,0x04,0x04,0x04,0x04,0x03,0x00,/*"U",53*/

0x01,0x0F,0xF1,0x00,0xC0,0x39,0x07,0x01,
0x00,0x00,0x00,0x07,0x01,0x00,0x00,0x00,/*"V",54*/

0x01,0x7F,0xC0,0x3F,0xC0,0x7F,0x01,0x00,
0x00,0x00,0x07,0x00,0x07,0x00,0x00,0x00,/*"W",55*/

0x01,0x03,0x8D,0x70,0x70,0x8D,0x03,0x01,
0x04,0x06,0x05,0x00,0x00,0x05,0x06,0x04,/*"X",56*/

0x01,0x07,0x19,0xE0,0x19,0x07,0x01,0x00,
0x00,0x00,0x04,0x07,0x04,0x00,0x00,0x00,/*"Y",57*/

0x02,0x01,0xC1,0x21,0x19,0x07,0x01,0x00,
0x04,0x07,0x04,0x04,0x04,0x04,0x03,0x00,/*"Z",58*/

0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x0F,0x08,0x08,0x08,0x00,/*"[",59*/

0x00,0x00,0x07,0x38,0xC0,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x07,0x18,0x00,/*"\",60*/

0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,
0x00,0x08,0x08,0x08,0x0F,0x00,0x00,0x00,/*"]",61*/

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"^",62*/

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,/*"_",63*/

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"`",64*/

0x00,0x20,0x90,0x90,0x50,0xE0,0x00,0x00,
0x00,0x03,0x04,0x04,0x02,0x07,0x04,0x00,/*"a",65*/

0x02,0xFE,0x20,0x10,0x10,0x20,0xC0,0x00,
0x00,0x07,0x02,0x04,0x04,0x02,0x01,0x00,/*"b",66*/

0x00,0xC0,0x20,0x10,0x10,0x10,0x20,0x00,
0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x00,/*"c",67*/

0x00,0xE0,0x10,0x10,0x10,0x12,0xFE,0x00,
0x00,0x03,0x04,0x04,0x04,0x02,0x07,0x04,/*"d",68*/

0x00,0xE0,0x90,0x90,0x90,0x90,0xE0,0x00,
0x00,0x03,0x04,0x04,0x04,0x04,0x02,0x00,/*"e",69*/

0x00,0x10,0x10,0xFC,0x12,0x12,0x04,0x00,
0x00,0x04,0x04,0x07,0x04,0x04,0x00,0x00,/*"f",70*/

0x00,0x60,0x90,0x90,0x90,0x70,0x10,0x00,
0x00,0x0D,0x12,0x12,0x12,0x12,0x0C,0x00,/*"g",71*/

0x02,0xFE,0x20,0x10,0x10,0x10,0xE0,0x00,
0x04,0x07,0x04,0x00,0x00,0x04,0x07,0x04,/*"h",72*/

0x00,0x10,0x13,0xF3,0x00,0x00,0x00,0x00,
0x00,0x04,0x04,0x07,0x04,0x04,0x00,0x00,/*"i",73*/

0x00,0x00,0x00,0x10,0x13,0xF3,0x00,0x00,
0x00,0x18,0x10,0x10,0x10,0x0F,0x00,0x00,/*"j",74*/

0x02,0xFE,0x80,0xC0,0x30,0x10,0x10,0x00,
0x04,0x07,0x04,0x00,0x05,0x06,0x04,0x00,/*"k",75*/

0x00,0x02,0x02,0xFF,0x00,0x00,0x00,0x00,
0x00,0x04,0x04,0x07,0x04,0x04,0x00,0x00,/*"l",76*/

0x10,0xF0,0x10,0x10,0xF0,0x10,0x10,0xE0,
0x04,0x07,0x04,0x00,0x07,0x04,0x00,0x07,/*"m",77*/

0x10,0xF0,0x20,0x10,0x10,0x10,0xE0,0x00,
0x04,0x07,0x04,0x00,0x00,0x04,0x07,0x04,/*"n",78*/

0x00,0xE0,0x10,0x10,0x10,0x10,0xE0,0x00,
0x00,0x03,0x04,0x04,0x04,0x04,0x03,0x00,/*"o",79*/

0x10,0xF0,0x20,0x10,0x10,0x20,0xC0,0x00,
0x10,0x1F,0x12,0x04,0x04,0x02,0x01,0x00,/*"p",80*/

0x00,0xC0,0x20,0x10,0x10,0x20,0xF0,0x00,
0x00,0x01,0x02,0x04,0x04,0x12,0x1F,0x10,/*"q",81*/

0x10,0x10,0xF0,0x20,0x10,0x10,0x30,0x00,
0x04,0x04,0x07,0x04,0x04,0x00,0x00,0x00,/*"r",82*/

0x00,0x60,0x90,0x90,0x90,0x90,0x30,0x00,
0x00,0x06,0x04,0x04,0x04,0x04,0x03,0x00,/*"s",83*/

0x00,0x10,0x10,0xFC,0x10,0x10,0x00,0x00,
0x00,0x00,0x00,0x03,0x04,0x04,0x02,0x00,/*"t",84*/

0x10,0xF0,0x00,0x00,0x00,0x10,0xF0,0x00,
0x00,0x03,0x04,0x04,0x04,0x02,0x07,0x04,/*"u",85*/

0x10,0x70,0x90,0x00,0x90,0x70,0x10,0x00,
0x00,0x00,0x01,0x06,0x01,0x00,0x00,0x00,/*"v",86*/

0x30,0xD0,0x00,0x90,0xF0,0x00,0xD0,0x30,
0x00,0x01,0x06,0x01,0x00,0x07,0x00,0x00,/*"w",87*/

0x00,0x10,0x30,0xD0,0xC0,0x30,0x10,0x00,
0x00,0x04,0x06,0x01,0x05,0x06,0x04,0x00,/*"x",88*/

0x10,0x30,0xD0,0x00,0x00,0xD0,0x30,0x10,
0x00,0x10,0x10,0x0F,0x03,0x00,0x00,0x00,/*"y",89*/

0x00,0x30,0x10,0x90,0x50,0x30,0x10,0x00,
0x00,0x04,0x06,0x05,0x04,0x04,0x06,0x00,/*"z",90*/

0x00,0x00,0x00,0x00,0x20,0xDF,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x07,0x08,0x08,/*"{",91*/

0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x1F,0x00,0x00,0x00,/*"|",92*/

0x00,0x00,0xDF,0x20,0x00,0x00,0x00,0x00,
0x08,0x08,0x07,0x00,0x00,0x00,0x00,0x00,/*"}",93*/

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"~",94*/

};

const uint8_t hz[]={
0x10,0x60,0x02,0x8C,0x00,0x44,0x64,0x54,
0x4D,0x46,0x44,0x54,0x64,0xC4,0x04,0x00,
0x04,0x04,0x7E,0x01,0x80,0x40,0x3E,0x00,
0x00,0xFE,0x00,0x00,0x7E,0x80,0xE0,0x00,/*"?",0;*/

0x00,0xFC,0x44,0x44,0x44,0xFC,0x00,0x00,
0xFE,0x22,0x22,0x22,0x22,0xFE,0x00,0x00,
0x00,0x0F,0x04,0x04,0x04,0x8F,0x40,0x30,
0x0F,0x02,0x02,0x42,0x82,0x7F,0x00,0x00,/*"?",1;*/

0x40,0x40,0x42,0xCC,0x00,0x40,0xA0,0x9E,
0x82,0x82,0x82,0x9E,0xA0,0x20,0x20,0x00,
0x00,0x00,0x00,0x3F,0x90,0x88,0x40,0x43,
0x2C,0x10,0x28,0x46,0x41,0x80,0x80,0x00,/*"?",2;*/

0x00,0x17,0x15,0xD5,0x55,0x57,0x55,0x7D,
0x55,0x57,0x55,0xD5,0x15,0x17,0x00,0x00,
0x40,0x40,0x40,0x7F,0x55,0x55,0x55,0x55,
0x55,0x55,0x55,0x7F,0x40,0x40,0x40,0x00,/*"?",3;*/
	
0x00,0xFC,0x84,0x84,0x84,0xFC,0x00,0x10,
0x10,0x10,0x10,0x10,0xFF,0x10,0x10,0x00,
0x00,0x3F,0x10,0x10,0x10,0x3F,0x00,0x00,
0x01,0x06,0x40,0x80,0x7F,0x00,0x00,0x00,/*"?",0;*/

0x00,0xF8,0x01,0x06,0x00,0xF0,0x12,0x12,
0x12,0xF2,0x02,0x02,0x02,0xFE,0x00,0x00,
0x00,0xFF,0x00,0x00,0x00,0x1F,0x11,0x11,
0x11,0x1F,0x00,0x40,0x80,0x7F,0x00,0x00,/*"?",1;*/

0x40,0x40,0x42,0xCC,0x00,0x40,0xA0,0x9E,
0x82,0x82,0x82,0x9E,0xA0,0x20,0x20,0x00,
0x00,0x00,0x00,0x3F,0x90,0x88,0x40,0x43,
0x2C,0x10,0x28,0x46,0x41,0x80,0x80,0x00,/*"?",2;*/

0x00,0x17,0x15,0xD5,0x55,0x57,0x55,0x7D,
0x55,0x57,0x55,0xD5,0x15,0x17,0x00,0x00,
0x40,0x40,0x40,0x7F,0x55,0x55,0x55,0x55,
0x55,0x55,0x55,0x7F,0x40,0x40,0x40,0x00,/*"?",3;*/


0x00,0xF8,0x01,0x22,0x20,0x22,0x2A,0xF2,
0x22,0x22,0x22,0x22,0x02,0xFE,0x00,0x00,
0x00,0xFF,0x00,0x00,0x1F,0x01,0x01,0x7F,
0x09,0x11,0x0F,0x40,0x80,0x7F,0x00,0x00,/*"?",0;*/

0x20,0x10,0x2C,0xE7,0x24,0x24,0x00,0xF0,
0x10,0x10,0xFF,0x10,0x10,0xF0,0x00,0x00,
0x01,0x01,0x01,0x7F,0x21,0x11,0x00,0x07,
0x02,0x02,0xFF,0x02,0x02,0x07,0x00,0x00,/*"?",1;*/

0x40,0x40,0x42,0xCC,0x00,0x40,0xA0,0x9E,
0x82,0x82,0x82,0x9E,0xA0,0x20,0x20,0x00,
0x00,0x00,0x00,0x3F,0x90,0x88,0x40,0x43,
0x2C,0x10,0x28,0x46,0x41,0x80,0x80,0x00,/*"?",2;*/

0x00,0x17,0x15,0xD5,0x55,0x57,0x55,0x7D,
0x55,0x57,0x55,0xD5,0x15,0x17,0x00,0x00,
0x40,0x40,0x40,0x7F,0x55,0x55,0x55,0x55,
0x55,0x55,0x55,0x7F,0x40,0x40,0x40,0x00,/*"?",3;*/


0x00,0x00,0x00,0xFE,0x02,0x04,0x08,0x10,
0x20,0x40,0x80,0x80,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x7F,0x40,0x20,0x10,0x08,
0x04,0x02,0x01,0x01,0x00,0x00,0x00,0x00,/*"?????",0;*/

};
const uint8_t  F10x10[] =
 {
		0xF8,0x04,0x04,0xF8,0x00,0x00,0x01,0x01,0x00,0x00,/*"0",0*/

		0x00,0x08,0xFC,0x00,0x00,0x00,0x01,0x01,0x01,0x00,/*"1",1*/

		0x00,0x8C,0x64,0x1C,0x00,0x00,0x01,0x01,0x01,0x00,/*"2",2*/

		0x00,0x8C,0x24,0xDC,0x00,0x00,0x01,0x01,0x01,0x00,/*"3",3*/

		0x60,0x50,0x48,0xFC,0x40,0x00,0x00,0x01,0x01,0x01,/*"4",4*/

		0x00,0x9C,0x14,0xF4,0x00,0x00,0x01,0x01,0x00,0x00,/*"5",5*/

		0xF8,0x14,0x14,0xE4,0x00,0x00,0x01,0x01,0x00,0x00,/*"6",6*/

		0x00,0x04,0xE4,0x1C,0x00,0x00,0x00,0x01,0x00,0x00,/*"7",7*/

		0xD8,0x24,0x24,0xD8,0x00,0x00,0x01,0x01,0x00,0x00,/*"8",8*/

		0x78,0x44,0xC4,0xF8,0x00,0x01,0x01,0x01,0x00,0x00,/*"9",9*/

		0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x03,0x00,0x00,/*";",10*/

 };

 
const uint8_t F32x48[]= {
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,
0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0xE0,0xF8,0xFE,0x1F,0x0F,0x07,
0x07,0x0F,0x1F,0xFC,0xF8,0xC0,0x00,0x00,
0x00,0xFE,0xFF,0xFF,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x01,0xFF,0xFF,0x00,0x00,
0x00,0x7F,0xFF,0xFF,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x80,0xFF,0xFF,0x00,0x00,
0x00,0x00,0x07,0x1F,0x7F,0x78,0xF0,0xE0,
0xE0,0xF0,0x7C,0x3F,0x1F,0x03,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"0",0*/

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0xC0,0xE0,0xF0,0x7C,0xFE,
0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x03,0x01,0x00,0x00,0xFF,
0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,
0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,
0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"1",1*/

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,
0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0xC0,0xF8,0xFC,0x1E,0x0F,0x07,0x07,
0x07,0x0F,0x1F,0xFE,0xFC,0xE0,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x80,0xF0,0xFF,0x3F,0x07,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0xC0,0xE0,0xF8,
0x7E,0x1F,0x07,0x01,0x00,0x00,0x00,0x00,
0x00,0x80,0xF8,0xFC,0xFF,0xFF,0xF3,0xF0,
0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"2",2*/

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0xF0,0xFC,0x3E,0x0F,0x07,0x07,
0x07,0x07,0x0F,0xFE,0xFC,0xF0,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0xC0,0xC0,
0xE0,0xE0,0xF8,0x3F,0x1F,0x03,0x00,0x00,
0x00,0x00,0x80,0x80,0x00,0x00,0x00,0x01,
0x01,0x03,0x07,0xBF,0xFE,0xF8,0x00,0x00,
0x00,0x01,0x0F,0x3F,0x7C,0xF0,0xE0,0xE0,
0xE0,0xF0,0x78,0x7F,0x1F,0x07,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"3",3*/

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC0,
0xF0,0xFC,0xFF,0xFF,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x80,0xE0,0xFC,0x3F,0x0F,
0x03,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,
0xE0,0xF8,0xFE,0xDF,0xC3,0xC0,0xC0,0xC0,
0xC0,0xC0,0xFF,0xFF,0xC0,0xC0,0xC0,0x00,
0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
0x01,0x01,0xFF,0xFF,0x01,0x01,0x01,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"4",4*/

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x80,0xFF,0xFF,0x0F,0x0F,0x0F,
0x0F,0x0F,0x0F,0x0F,0x0F,0x00,0x00,0x00,
0x00,0xC0,0xFF,0xFF,0x7B,0x38,0x38,0x3C,
0x38,0x78,0xF8,0xF0,0xE0,0x00,0x00,0x00,
0x00,0x00,0x81,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0xC7,0xFF,0xFF,0x00,0x00,
0x00,0x0F,0x3F,0x7C,0xF0,0xE0,0xE0,0xE0,
0xE0,0xF0,0x7C,0x3F,0x1F,0x03,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"5",5*/

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x80,0xE0,0xF8,
0x7F,0x1F,0x03,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x80,0xF0,0xFC,0xFF,0x7F,0x71,
0x78,0x78,0x70,0xF0,0xE0,0xC0,0x00,0x00,
0x00,0xFC,0xFF,0xFF,0x03,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0x00,
0x00,0x01,0x1F,0x3F,0x7C,0xF0,0xE0,0xE0,
0xE0,0xE0,0xF0,0x78,0x7F,0x1F,0x07,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"6",6*/

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,
0x0F,0x0F,0x0F,0xEF,0xFF,0x7F,0x0F,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xC0,0xF8,0x7F,0x0F,0x01,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFE,
0x7F,0x07,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0xC0,0xFE,0xFF,0x1F,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"7",7*/

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0xF8,0xFC,0x1E,0x0F,0x07,0x07,
0x07,0x07,0x1F,0xFE,0xFC,0xE0,0x00,0x00,
0x00,0x00,0x1F,0xBF,0xF8,0xF0,0xE0,0xE0,
0xE0,0xF0,0xF8,0x3F,0x1F,0x03,0x00,0x00,
0x00,0xF8,0xFE,0x3F,0x07,0x03,0x01,0x01,
0x01,0x03,0x07,0x0F,0xFF,0xFC,0x00,0x00,
0x00,0x0F,0x3F,0x7F,0xF0,0xF0,0xE0,0xE0,
0xE0,0xF0,0xF0,0x7E,0x3F,0x0F,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"8",8*/

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0xE0,0xF8,0xFE,0x1F,0x0F,0x07,0x07,
0x07,0x0F,0x1F,0xFE,0xF8,0xE0,0x00,0x00,
0x00,0xFF,0xFF,0xE0,0x80,0x00,0x00,0x00,
0x00,0x00,0xC0,0xFF,0xFF,0x3F,0x00,0x00,
0x00,0x00,0x03,0x07,0x0F,0x0F,0x0E,0x8E,
0xFF,0xFF,0x7F,0x0F,0x01,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x80,0xE0,0xFC,0x7F,
0x0F,0x01,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"9",9*/
};

 void GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */ 
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9|GPIO_PIN_8, GPIO_PIN_RESET);
  /*Configure GPIO pins : PE3 PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}
void delay_bus(uint16_t dat)
{
	while(dat)
	{	
		dat--;	
	}
}	
void delay_5us(void)
{
	uint32_t n;
	n = 5000000;
	while(n)
		n--;
}
void SDA_write(uint8_t dat)
{
  if(dat == H)
	 HAL_GPIO_WritePin(oled_sda_GPIO_Port, oled_sda_Pin,H);
  if(dat == L)
	  HAL_GPIO_WritePin(oled_sda_GPIO_Port, oled_sda_Pin,L);
}
void SCL_write(uint8_t dat)
{
  if(dat == H)
	 HAL_GPIO_WritePin(oled_sck_GPIO_Port, oled_sck_Pin,H);
	if(dat == L)
	 HAL_GPIO_WritePin(oled_sck_GPIO_Port, oled_sck_Pin,L);
}
void OLED_start(void)
{
	SDA_write(H);
	SCL_write(H);delay_bus(10);
	SDA_write(L);delay_bus(10);
	SCL_write(L);delay_bus(10);
}
void OLED_stop(void)
{
  SCL_write(L);delay_bus(10);	
	SDA_write(L);delay_bus(10);
	SCL_write(H);delay_bus(10);
	SDA_write(H);delay_bus(10);
	
}
void OLED_write_byte(uint8_t dat)
{
  uint8_t i;
 for(i=0;i<8;i++)
 {
   if(dat &0x80)
	 {
    	SDA_write(H);delay_bus(10); 
	 }
   else
	 {
      SDA_write(L);delay_bus(10);
	 }		 
	 SCL_write(H);delay_bus(10);
   SCL_write(L);delay_bus(10);
	 dat <<= 1;delay_bus(10);
 }
 SDA_write(H);delay_bus(10);
 SCL_write(H);delay_bus(10);
 SCL_write(L);delay_bus(10);
}
void OLED_write_dat(uint8_t dat)
{
	OLED_start();
	OLED_write_byte(0x78);
	OLED_write_byte(0x40);
	OLED_write_byte(dat);
	OLED_stop();
}
void OLED_write_cmd(uint8_t dat)
{
	OLED_start();
	OLED_write_byte(0x78);
	OLED_write_byte(0x00);
	OLED_write_byte(dat);
	OLED_stop();
}
void OLED_cls(void)
{
	uint8_t y,x;
	for(y=0;y<8;y++)
	{
		OLED_write_cmd(0xb0+y);
		OLED_write_cmd(0x01);
		OLED_write_cmd(0x10);
		for(x=0;x<128;x++)
		  OLED_write_dat(0x00);
	}	
}
void OLED_set_pos(uint8_t x,uint8_t y)
{
	OLED_write_cmd(0xb0+y);
	OLED_write_cmd(((x&0xf0)>>4)|0x10);
	OLED_write_cmd((x&0x0f)|0x01);	
}
void OLED_fill(uint8_t dat)
{
	uint8_t y,x;
	for(y=0;y<8;y++)
	 {
		OLED_write_cmd(0xb0+y); 
		OLED_write_cmd(0x01); 
		OLED_write_cmd(0x10); 
		for(x=0;x<128;x++)
		   OLED_write_dat(dat); 
	 }
}
void OLED_init(void)
{
	GPIO_Init();
	delay_5us();
	OLED_write_cmd(0xae);
	OLED_write_cmd(0x00);
	OLED_write_cmd(0x10);
	OLED_write_cmd(0x40);
	OLED_write_cmd(0x81);
	OLED_write_cmd(0xbf);
	OLED_write_cmd(0xa1);
	OLED_write_cmd(0xc8);
	OLED_write_cmd(0xa6);
	OLED_write_cmd(0xa8);
	OLED_write_cmd(0x3f);
	OLED_write_cmd(0xd3);
	OLED_write_cmd(0x00);
	OLED_write_cmd(0xd5);
	OLED_write_cmd(0x80);
	OLED_write_cmd(0xd9);
	OLED_write_cmd(0xf1);
	OLED_write_cmd(0xda);
	OLED_write_cmd(0x12);
	OLED_write_cmd(0xdb);
	OLED_write_cmd(0x40);
	OLED_write_cmd(0x20);
	OLED_write_cmd(0x02);
	OLED_write_cmd(0x8d);
	OLED_write_cmd(0x14);
	OLED_write_cmd(0xa4);
	OLED_write_cmd(0xa6);
	OLED_write_cmd(0xaf);
	OLED_fill(0xff);
	OLED_set_pos(0,0);	
	OLED_fill(0x00);
	OLED_set_pos(0,0);	
	
	
}
void OLED_P10x10Ch(uint8_t x,uint8_t y,uint8_t n)
  {
	unsigned char wm=0,i;
	unsigned int adder=10*n;  //????????????
  for(i=0;i<2;i++)
    {
    	OLED_set_pos(x , y);
   	for(wm = 0;wm <5;wm++)
   	  {
	   	OLED_write_dat(F10x10[adder]);
		   adder += 1;  
	     }
	 y++;
	}	
 }
void OLED_english(uint8_t x,uint8_t y,uint8_t n)
{
	uint8_t wm = 0,i;
	uint16_t adder = 16*(n-32);
	OLED_set_pos(0,0);	
	for(i=0;i<2;i++)
	{
		OLED_set_pos(x,y);	
	for(wm = 0;wm<8;wm++)
		 {
			 OLED_write_dat(english[adder]);  
			 adder += 1;
		 }
	 y++;
	 }
}
void OLED_16X16(uint8_t x,uint8_t y,uint8_t n)
{
	uint8_t wm = 0,i;
	uint32_t adder = 32*n;
	OLED_set_pos(0,0);	
	for(i=0;i<2;i++)
	{
		OLED_set_pos(x,y);	
	for(wm = 0;wm<16;wm++)
		 {
			 OLED_write_dat(F16x16[adder]);  
			 adder += 1;
		 }
	 y++;
	 }
}
void OLED_32X48(uint8_t x,uint8_t y,uint8_t n)
{
	uint8_t wm = 0,i;
	uint32_t adder = 96*n;
	OLED_set_pos(0,0);	
	for(i=0;i<6;i++)
	{
		OLED_set_pos(x,y);	
	for(wm = 0;wm<16;wm++)
		 {
			 OLED_write_dat(F32x48[adder]);  
			 adder += 1;
		 }
	 y++;
	 }
}
void OLED_hz(uint8_t x,uint8_t y,uint8_t n)
{
	uint8_t  wm = 0,i;
	uint32_t adder = 32*n;
	for(i=0;i<2;i++)
	{
		OLED_set_pos(x,y);	
	for(wm = 0;wm<16;wm++)
		 {
			 OLED_write_dat(hz[adder]);  
			 adder += 1;
		 }
	 y++;
	 }
}


void OLED_H(uint8_t x,uint8_t y,uint8_t n)
{
	uint8_t wm = 0,i;
	uint32_t adder = 16*n;
	for(i=0;i<1;i++)
	{
		OLED_set_pos(x,y);	
	for(wm = 0;wm<16;wm++)
		 {
			 OLED_write_dat(DPH[adder]);  
			 adder += 1;
		 }
	 y++;
	 }
}
void OLED_L(uint8_t x,uint8_t y,uint8_t n)
{
	uint8_t wm = 0,i;
	uint32_t adder = 16*n;
	for(i=0;i<1;i++)
	{
		OLED_set_pos(x,y);	
	for(wm = 0;wm<16;wm++)
		 {
			 OLED_write_dat(DPL[adder]);  
			 adder += 1;
		 }
	 y++;
	 }
}
