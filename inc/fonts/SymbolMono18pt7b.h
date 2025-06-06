const uint8_t SymbolMono18pt7bBitmaps[] PROGMEM = {
//05 Power symbol
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| . . . . . . . X . . . . . . . . |*/  0x01,0x00, 
/*| . . . . . . . X . . . . . . . . |*/  0x01,0x00, 
/*| . . . . . . . X . . . . . . . . |*/  0x01,0x00, 
/*| . . . . . X . X . X . . . . . . |*/  0x05,0x40, 
/*| . . . X X . . X . . X X . . . . |*/  0x19,0x30,
/*| . . X . . . . X . . . . X . . . |*/  0x21,0x08, 
/*| . X . . . . . X . . . . . X . . |*/  0x41,0x04, 
/*| X . . . . . . X . . . . . . X . |*/  0x81,0x02, 
/*| X . . . . . . X . . . . . . X . |*/  0x81,0x02, 
/*| X . . . . . . X . . . . . . X . |*/  0x81,0x02,
/*| X . . . . . . , . . . . . . X . |*/  0x80,0x02, 
/*| X . . . . . . , . . . . . . X . |*/  0x80,0x02, 
/*| . X . . . . . , . . . . . X . . |*/  0x40,0x04, 
/*| . . X . . . . , . . . . X . . . |*/  0x20,0x08, 
/*| . . . X X . . , . . X X . . . . |*/  0x18,0x30,
/*| . . . . . X X X X X . . . . . . |*/  0x07,0xc0,
//06 Skip left
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| X X . . . . . , . . . . . X . . |*/  0xc0,0x04, 
/*| X X . . . . . , . . . X X X . . |*/  0xc0,0x1c, 
/*| X X . . . . . , . X X X X X . . |*/  0xc0,0x7c, 
/*| X X . . . . . X X X X X X X . . |*/  0xc1,0xfc, 
/*| X X . . . X X X X X X X X X . . |*/  0xc7,0xfc,
/*| X X X X X X X X X X X X X X . . |*/  0xff,0xfc, 
/*| X X . . . X X X X X X X X X . . |*/  0xc7,0xfc, 
/*| X X . . . . . X X X X X X X . . |*/  0xc1,0xfc, 
/*| X X . . . . . , . X X X X X . . |*/  0xc0,0x7c, 
/*| X X . . . . . , . . . X X X . . |*/  0xc0,0x1c,
/*| X X . . . . . , . . . . . X . . |*/  0xc0,0x04,
//07 Rewind
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| . . . . . . X X . . . . . . X X |*/  0x03,0x03, 
/*| . . . . . X X X . . . . . X X X |*/  0x07,0x07, 
/*| . . . . X X X X . . . . X X X X |*/  0x0f,0x0f, 
/*| . . . X X X X X . . . X X X X X |*/  0x1f,0x1f, 
/*| . . X X X X X X . . X X X X X X |*/  0x3f,0x3f,
/*| . X X X X X X X . X X X X X X X |*/  0x7f,0x7f, 
/*| X X X X X X X X X X X X X X X X |*/  0xff,0xff, 
/*| . X X X X X X X . X X X X X X X |*/  0x7f,0x7f, 
/*| . . X X X X X X . . X X X X X X |*/  0x3f,0x3f, 
/*| . . . X X X X X . . . X X X X X |*/  0x1f,0x1f,
/*| . . . . X X X X . . . . X X X X |*/  0x0f,0x0f, 
/*| . . . . . X X X . . . . . X X X |*/  0x07,0x07, 
/*| . . . . . . X X . . . . . . X X |*/  0x03,0x03,
//08 Play
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| X . . . . . . , . . . . . . . . |*/  0x80,0x00, 
/*| X X X . . . . , . . . . . . . . |*/  0xe0,0x00, 
/*| X X X X X . . , . . . . . . . . |*/  0xf8,0x00, 
/*| X X X X X X X , . . . . . . . . |*/  0xfe,0x00, 
/*| X X X X X X X X X . . . . . . . |*/  0xff,0x80,
/*| X X X X X X X X X X X . . . . . |*/  0xff,0xe0, 
/*| X X X X X X X X X X X X X . . . |*/  0xff,0xf8, 
/*| X X X X X X X X X X X . . . . . |*/  0xff,0xe0, 
/*| X X X X X X X X X . . . . . . . |*/  0xff,0x80, 
/*| X X X X X X X , . . . . . . . . |*/  0xfe,0x00,
/*| X X X X X . . , . . . . . . . . |*/  0xf8,0x00, 
/*| X X X . . . . , . . . . . . . . |*/  0xe0,0x00, 
/*| X . . . . . . , . . . . . . . . |*/  0x80,0x00,
//09 ff
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| X X . . . . . , X X . . . . . . |*/  0xc0,0xc0, 
/*| X X X . . . . , X X X . . . . . |*/  0xe0,0xe0, 
/*| X X X X . . . , X X X X . . . . |*/  0xf0,0xf0, 
/*| X X X X X . . , X X X X X . . . |*/  0xf8,0xf8, 
/*| X X X X X X . , X X X X X X . . |*/  0xfc,0xfc,
/*| X X X X X X X , X X X X X X X . |*/  0xfe,0xfe, 
/*| X X X X X X X X X X X X X X X X |*/  0xff,0xff, 
/*| X X X X X X X , X X X X X X X . |*/  0xfe,0xfe, 
/*| X X X X X X . , X X X X X X . . |*/  0xfc,0xfc, 
/*| X X X X X . . , X X X X X . . . |*/  0xf8,0xf8,
/*| X X X X . . . , X X X X . . . . |*/  0xf0,0xf0, 
/*| X X X . . . . , X X X . . . . . |*/  0xe0,0xe0, 
/*| X X . . . . . , X X . . . . . . |*/  0xc0,0xc0,
//10 Skip Right
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| . . X . . . . , . . . . . . X X |*/  0x20,0x03, 
/*| . . X X X . . , . . . . . . X X |*/  0x38,0x03, 
/*| . . X X X X X , . . . . . . X X |*/  0x3e,0x03, 
/*| . . X X X X X X X . . . . . X X |*/  0x3f,0x83, 
/*| . . X X X X X X X X X . . . X X |*/  0x3f,0xe3,
/*| . . X X X X X X X X X X X X X X |*/  0x3f,0xff, 
/*| . . X X X X X X X X X . . . X X |*/  0x3f,0xe3, 
/*| . . X X X X X X X . . . . . X X |*/  0x3f,0x83, 
/*| . . X X X X X , . . . . . . X X |*/  0x3e,0x03, 
/*| . . X X X . . , . . . . . . X X |*/  0x38,0x03,
/*| . . X . . . . , . . . . . . X X |*/  0x30,0x03,
//11 pause
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| . . . X X X . , X X X . . . . . |*/  0x1c,0xe0, 
/*| . . . X X X . , X X X . . . . . |*/  0x1c,0xe0, 
/*| . . . X X X . , X X X . . . . . |*/  0x1c,0xe0, 
/*| . . . X X X . , X X X . . . . . |*/  0x1c,0xe0, 
/*| . . . X X X . , X X X . . . . . |*/  0x1c,0xe0,
/*| . . . X X X . , X X X . . . . . |*/  0x1c,0xe0, 
/*| . . . X X X . , X X X . . . . . |*/  0x1c,0xe0, 
/*| . . . X X X . , X X X . . . . . |*/  0x1c,0xe0, 
/*| . . . X X X . , X X X . . . . . |*/  0x1c,0xe0, 
/*| . . . X X X . , X X X . . . . . |*/  0x1c,0xe0,
//12 Stop
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| . . X X X X X X X X X X . . . . |*/  0x3f,0xf0, 
/*| . . X X X X X X X X X X . . . . |*/  0x3f,0xf0, 
/*| . . X X X X X X X X X X . . . . |*/  0x3f,0xf0, 
/*| . . X X X X X X X X X X . . . . |*/  0x3f,0xf0, 
/*| . . X X X X X X X X X X . . . . |*/  0x3f,0xf0, 
/*| . . X X X X X X X X X X . . . . |*/  0x3f,0xf0, 
/*| . . X X X X X X X X X X . . . . |*/  0x3f,0xf0, 
/*| . . X X X X X X X X X X . . . . |*/  0x3f,0xf0, 
/*| . . X X X X X X X X X X . . . . |*/  0x3f,0xf0, 
/*| . . X X X X X X X X X X . . . . |*/  0x3f,0xf0, 
//13 ch up
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| . . . . . . . X . . . . . . . . |*/  0x01,0x00, 
/*| . . . . . . X X X . . . . . . . |*/  0x03,0x80, 
/*| . . . . . X X X X X . . . . . . |*/  0x07,0xc0, 
/*| . . . . X X X X X X X . . . . . |*/  0x0f,0xe0, 
/*| . . . X X X X X X X X X . . . . |*/  0x1f,0xf0,
/*| . . X X X X X X X X X X X . . . |*/  0x3f,0xf8, 
/*| . X X X X X X X X X X X X X . . |*/  0x7f,0xfc, 
/*| X X X X X X X X X X X X X X X . |*/  0xff,0xfe,
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
//14 ch down
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| X X X X X X X X X X X X X X X . |*/  0xff,0xfe, 
/*| . X X X X X X X X X X X X X . . |*/  0x7f,0xfc, 
/*| . . X X X X X X X X X X X . . . |*/  0x3f,0xf8, 
/*| . . . X X X X X X X X X . . . . |*/  0x1f,0xf0, 
/*| . . . . X X X X X X X . . . . . |*/  0x0f,0xe0,
/*| . . . . . X X X X X . . . . . . |*/  0x07,0xc0, 
/*| . . . . . . X X X . . . . . . . |*/  0x03,0x80, 
/*| . . . . . . . X . . . . . . . . |*/  0x01,0x00,
//15 Page up
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| . . . . . . . X . . . . . . . . |*/  0x01,0x00, 
/*| . . . . . . X , X . . . . . . . |*/  0x02,0x80, 
/*| . . . . . X . , . X . . . . . . |*/  0x04,0x40, 
/*| . . . . X . . , . . X . . . . . |*/  0x08,0x20, 
/*| . . . X . . . , . . . X . . . . |*/  0x10,0x10,
/*| . . X . . . . , . . . . X . . . |*/  0x20,0x08, 
/*| . X . . . . . X . . . . . X . . |*/  0x41,0x04, 
/*| X . . . . . X , X . . . . . X . |*/  0x82,0x82, 
/*| . . . . . X . , . X . . . . . . |*/  0x04,0x40, 
/*| . . . . X . . , . . X . . . . . |*/  0x08,0x20,
/*| . . . X . . . , . . . X . . . . |*/  0x10,0x10, 
/*| . . X . . . . , . . . . X . . . |*/  0x20,0x08, 
/*| . X . . . . . , . . . . . X . . |*/  0x40,0x04, 
/*| X . . . . . . , . . . . . . X . |*/  0x80,0x02,
//16 Page down
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| X . . . . . . , . . . . . . X . |*/  0x80,0x02, 
/*| . X . . . . . , . . . . . X . . |*/  0x40,0x04, 
/*| . . X . . . . , . . . . X . . . |*/  0x20,0x08, 
/*| . . . X . . . , . . . X . . . . |*/  0x10,0x10, 
/*| . . . . X . . , . . X . . . . . |*/  0x08,0x20,
/*| . . . . . X . , . X . . . . . . |*/  0x04,0x40, 
/*| X . . . . . X , X . . . . . X . |*/  0x82,0x82, 
/*| . X . . . . . X . . . . . X . . |*/  0x41,0x04, 
/*| . . X . . . . , . . . . X . . . |*/  0x20,0x08, 
/*| . . . X . . . , . . . X . . . . |*/  0x10,0x10,
/*| . . . . X . . , . . X . . . . . |*/  0x08,0x20, 
/*| . . . . . X . , . X . . . . . . |*/  0x04,0x40, 
/*| . . . . . . x , X . . . . . . . |*/  0x02,0x80, 
/*| . . . . . . . X . . . . . . . . |*/  0x01,0x00,
//17 Bluetooth
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| . . . . . . . X X . . . . . . . |*/  0x01,0x80, 
/*| . . . . . . . X . X . . . . . . |*/  0x01,0x40, 
/*| . . . . . . . X . . X . . . . . |*/  0x01,0x20, 
/*| . . . . . . . X . . . X . . . . |*/  0x01,0x10, 
/*| . . X . . . . X . . . . X . . . |*/  0x21,0x08,
/*| . . . X . . . X . . . X . . . . |*/  0x11,0x10, 
/*| . . . . X . . X . . X . . . . . |*/  0x09,0x20, 
/*| . . . . . X . X . X . . . . . . |*/  0x05,0x40, 
/*| . . . . . . X X X . . . . . . . |*/  0x03,0x80, 
/*| . . . . . X . X . X . . . . . . |*/  0x05,0x40,
/*| . . . . X . . X . . X . . . . . |*/  0x09,0x20, 
/*| . . . X . . . X . . . X . . . . |*/  0x11,0x10, 
/*| . . X . . . . X . . . . X . . . |*/  0x21,0x08, 
/*| . . . . . . . X . . . X . . . . |*/  0x01,0x10, 
/*| . . . . . . . X . . X . . . . . |*/  0x01,0x20,
/*| . . . . . . . X . X . . . . . . |*/  0x01,0x40, 
/*| . . . . . . . X X . . . . . . . |*/  0x01,0x80,
//18 Up arrow
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| . . . X X . . . . . . . . . . . |*/  0x18,0x00, 
/*| . . . X . X . . . . . . . . . . |*/  0x14,0x00,
/*| x . . X . . X . . . . . . . . . |*/  0x92,0x00, 
/*| . X . X . X . . . . . . . . . . |*/  0x54,0x00,
/*| . . X X X . . . . . . . . . . . |*/  0x38,0x00, 
/*| . x . X . X . . . . . . . . . . |*/  0x54,0x00, 
/*| x . . X . . X . . . . . . . . . |*/  0x92,0x00, 
/*| . . . X . X . . . . . . . . . . |*/  0x14,0x00, 
/*| . . . X X . . . . . . . . . . . |*/  0x18,0x00, 

//19 Down arrow
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| . . . . . . . X . . . . . . . . |*/  0x01,0x00, 
/*| . . . . . . . X . . . . . . . . |*/  0x01,0x00, 
/*| . . . . . . . X . . . . . . . . |*/  0x01,0x00, 
/*| . . . . . . . X . . . . . . . . |*/  0x01,0x00, 
/*| . . . . . . . X . . . . . . . . |*/  0x01,0x00,
/*| . . . . . . . X . . . . . . . . |*/  0x01,0x00, 
/*| . . . . . . . X . . . . . . . . |*/  0x01,0x00, 
/*| X . . . . . . X . . . . . . X . |*/  0x81,0x02, 
/*| . X . . . . . X . . . . . X . . |*/  0x41,0x04, 
/*| . . X . . . . X . . . . X . . . |*/  0x21,0x08,
/*| . . . X . . . X . . . X . . . . |*/  0x11,0x10, 
/*| . . . . X . . X . . X . . . . . |*/  0x09,0x20, 
/*| . . . . . X . X . X . . . . . . |*/  0x05,0x40, 
/*| . . . . . . X X X . . . . . . . |*/  0x03,0x80, 
/*| . . . . . . . X . . . . . . . . |*/  0x01,0x00,
//20 Left arrow
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| . . . . . . . X . . . . . . . . |*/  0x01,0x00, 
/*| . . . . . . X , . . . . . . . . |*/  0x02,0x00, 
/*| . . . . . X . , . . . . . . . . |*/  0x04,0x00, 
/*| . . . . X . . , . . . . . . . . |*/  0x08,0x00, 
/*| . . . X . . . , . . . . . . . . |*/  0x10,0x00,
/*| . . X . . . . , . . . . . . . . |*/  0x20,0x00, 
/*| . X . . . . . , . . . . . . . . |*/  0x40,0x00, 
/*| X X X X X X X X X X X X X X X X |*/  0xff,0xff, 
/*| . X . . . . . , . . . . . . . . |*/  0x40,0x00, 
/*| . . X . . . . , . . . . . . . . |*/  0x20,0x00,
/*| . . . X . . . , . . . . . . . . |*/  0x10,0x00, 
/*| . . . . X . . , . . . . . . . . |*/  0x08,0x00, 
/*| . . . . . X . , . . . . . . . . |*/  0x04,0x00, 
/*| . . . . . . X , . . . . . . . . |*/  0x02,0x00, 
/*| . . . . . . . X . . . . . . . . |*/  0x01,0x00,
//21 Right arrow
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| . . . . . . . , X . . . . . . . |*/  0x00,0x80, 
/*| . . . . . . . , . X . . . . . . |*/  0x00,0x40, 
/*| . . . . . . . , . . X . . . . . |*/  0x00,0x20, 
/*| . . . . . . . , . . . X . . . . |*/  0x00,0x10, 
/*| . . . . . . . , . . . . X . . . |*/  0x00,0x08,
/*| . . . . . . . , . . . . . X . . |*/  0x00,0x04, 
/*| . . . . . . . , . . . . . . X . |*/  0x00,0x02, 
/*| X X X X X X X X X X X X X X X X |*/  0xff,0xff, 
/*| . . . . . . . , . . . . . . X . |*/  0x00,0x02, 
/*| . . . . . . . , . . . . . X . . |*/  0x00,0x04,
/*| . . . . . . . , . . . . X . . . |*/  0x00,0x08, 
/*| . . . . . . . , . . . X . . . . |*/  0x00,0x10, 
/*| . . . . . . . , . . X . . . . . |*/  0x00,0x20, 
/*| . . . . . . . , . X . . . . . . |*/  0x00,0x40, 
/*| . . . . . . . , X . . . . . . . |*/  0x00,0x80,
//22 Return
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| . . . X . . . , . . . . . . . X |*/  0x10,0x01, 
/*| . . X X . . . , . . . . . . . X |*/  0x30,0x01, 
/*| . X X X . . . , . . . . . . . X |*/  0x70,0x01, 
/*| X X X X X X X X X X X X X X X X |*/  0xff,0xff, 
/*| . X X X . . . , . . . . . . . . |*/  0x70,0x00,
/*| . . X X . . . , . . . . . . . . |*/  0x30,0x00, 
/*| . . . X . . . , . . . . . . . . |*/  0x10,0x00,
//23 Backspace
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| . . . . . . . , X X X X X X X X |*/  0x00,0xff, 
/*| . . . . . . . X . . . . . . . X |*/  0x01,0x01, 
/*| . . . . . . X , . . . . . . . X |*/  0x02,0x01, 
/*| . . . . . X . , . . . . . . . X |*/  0x04,0x01, 
/*| . . . . X . . , . . . . . . . X |*/  0x08,0x01,
/*| . . . X . . . , . . . . . . . X |*/  0x10,0x01, 
/*| . . X . . . . X . . . X . . . X |*/  0x21,0x11, 
/*| . X . . . . . , X . X . . . . X |*/  0x40,0xa1, 
/*| X . . . . . . , . X . . . . . X |*/  0x80,0x41, 
/*| . X . . . . . , X . X . . . . X |*/  0x40,0xa1,
/*| . . X . . . . X . . . X . . . X |*/  0x21,0x11, 
/*| . . . X . . . , . . . . . . . X |*/  0x10,0x01, 
/*| . . . . X . . , . . . . . . . X |*/  0x08,0x01, 
/*| . . . . . X . , . . . . . . . X |*/  0x04,0x01, 
/*| . . . . . . X , . . . . . . . X |*/  0x02,0x01,
/*| . . . . . . . X . . . . . . . X |*/  0x01,0x01, 
/*| . . . . . . . , X X X X X X X X |*/  0x00,0xff,
//24 Delete
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| . X X X . . . , . . . . . X . . |*/  0x70,0x04, 
/*| . X X X X X . , . . . X X . . . |*/  0x7c,0x18, 
/*| . . . X X X X , . . X X . . . . |*/  0x1e,0x30, 
/*| . . . . . X X X . X X . . . . . |*/  0x07,0x60, 
/*| . . . . . . . X X X . . . . . . |*/  0x01,0xc0,
/*| . . . . . . . X X X . . . . . . |*/  0x01,0xc0, 
/*| . . . . . . X X . X X . . . . . |*/  0x03,0x60, 
/*| . . . . X X X , . . X X . . . . |*/  0x0e,0x30, 
/*| . . . X X X . , . . . X X . . . |*/  0x1c,0x18, 
/*| . . X X X . . , . . . . X X . . |*/  0x38,0x0c,
/*| . . X X X . . , . . . . . . X . |*/  0x38,0x02,
//25 Function 1
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| X X X X X . . , . . . X . . . . |*/  0xf8,0x10, 
/*| X . . . . . . , . . X X . . . . |*/  0x80,0x30, 
/*| X . . . . . . , . X X X . . . . |*/  0x80,0x70, 
/*| X . . . . . . , . . . X . . . . |*/  0x80,0x10, 
/*| X . . . . . . , . . . X . . . . |*/  0x80,0x10,
/*| X X X X X . . , . . . X . . . . |*/  0xf8,0x10, 
/*| X . . . . . . , . . . X . . . . |*/  0x80,0x10, 
/*| X . . . . . . , . . . X . . . . |*/  0x80,0x10, 
/*| X . . . . . . , . . . X . . . . |*/  0x80,0x10, 
/*| X . . . . . . , . . . X . . . . |*/  0x80,0x10,
/*| X . . . . . . , . . . X . . . . |*/  0x80,0x7c,
//26 Function 2
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| X X X X X . . , . . X X X . . . |*/  0xf8,0x38, 
/*| X . . . . . . , . X . . . X . . |*/  0x80,0x44, 
/*| X . . . . . . , . . . . . X . . |*/  0x80,0x04, 
/*| X . . . . . . , . . . . . X . . |*/  0x80,0x04, 
/*| X . . . . . . , . . . . . X . . |*/  0x80,0x04,
/*| X X X X X . . , . . . . X . . . |*/  0xf8,0x08, 
/*| X . . . . . . , . . . X . . . . |*/  0x80,0x10, 
/*| X . . . . . . , . . X . . . . . |*/  0x80,0x20, 
/*| X . . . . . . , . X . . . . . . |*/  0x80,0x40, 
/*| X . . . . . . , . X . . . . . . |*/  0x80,0x40,
/*| X . . . . . . , . X X X X X . . |*/  0x80,0x7c,
//27 Function 3
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| X X X X X . . , . . X X X . . . |*/  0xf8,0x38, 
/*| X . . . . . . , . X . . . X . . |*/  0x80,0x44, 
/*| X . . . . . . , . . . . . X . . |*/  0x80,0x04, 
/*| X . . . . . . , . . . . . X . . |*/  0x80,0x04, 
/*| X . . . . . . , . . . . . X . . |*/  0x80,0x04,
/*| X X X X X . . , . . X X X . . . |*/  0xf8,0x38, 
/*| X . . . . . . , . . . . . X . . |*/  0x80,0x04, 
/*| X . . . . . . , . . . . . X . . |*/  0x80,0x04, 
/*| X . . . . . . , . . . . . X . . |*/  0x80,0x04, 
/*| X . . . . . . , . X . . . X . . |*/  0x80,0x44,
/*| X . . . . . . , . . X X X . . . |*/  0x80,0x38,
//28 Function 4
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| X X X X X . . , . . . . . X . . |*/  0xf8,0x04, 
/*| X . . . . . . , . . . . X X . . |*/  0x80,0x0c, 
/*| X . . . . . . , . . . X . X . . |*/  0x80,0x14, 
/*| X . . . . . . , . . X . . X . . |*/  0x80,0x24, 
/*| X . . . . . . , . X . . . X . . |*/  0x80,0x44,
/*| X X X X X . . , X X X X X X . . |*/  0xf8,0xfc, 
/*| X . . . . . . , . . . . . X . . |*/  0x80,0x04, 
/*| X . . . . . . , . . . . . X . . |*/  0x80,0x04, 
/*| X . . . . . . , . . . . . X . . |*/  0x80,0x04, 
/*| X . . . . . . , . . . . . X . . |*/  0x80,0x04,
/*| X . . . . . . , . . . . . X . . |*/  0x80,0x04,
//29 Function 5
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| X X X X X . . , . X X X X X . . |*/  0xf8,0x7c, 
/*| X . . . . . . , . X . . . . . . |*/  0x80,0x40, 
/*| X . . . . . . , . X . . . . . . |*/  0x80,0x40, 
/*| X . . . . . . , . X . . . . . . |*/  0x80,0x40, 
/*| X . . . . . . , . X . . . . . . |*/  0x80,0x40,
/*| X X X X X . . , . X X X X . . . |*/  0xf8,0x78, 
/*| X . . . . . . , . . . . . X . . |*/  0x80,0x04, 
/*| X . . . . . . , . . . . . X . . |*/  0x80,0x04, 
/*| X . . . . . . , . . . . . X . . |*/  0x80,0x04, 
/*| X . . . . . . , . . . . . X . . |*/  0x80,0x04,
/*| X . . . . . . , . X X X X . . . |*/  0x80,0x78,
//30 Function 6
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| X X X X X . . , . . X X X . . . |*/  0xf8,0x38, 
/*| X . . . . . . , . X . . . X . . |*/  0x80,0x44, 
/*| X . . . . . . , . X . . . . . . |*/  0x80,0x40, 
/*| X . . . . . . , . X . . . . . . |*/  0x80,0x40, 
/*| X . . . . . . , . X . . . . . . |*/  0x80,0x40,
/*| X X X X X . . , . X X X X . . . |*/  0xf8,0x78, 
/*| X . . . . . . , . X . . . X . . |*/  0x80,0x44, 
/*| X . . . . . . , . X . . . X . . |*/  0x80,0x44, 
/*| X . . . . . . , . X . . . X . . |*/  0x80,0x44, 
/*| X . . . . . . , . X . . . X . . |*/  0x80,0x44,
/*| X . . . . . . , . . X X X . . . |*/  0x80,0x38,
//31 Function 7
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| X X X X X . . , . X X X X X X . |*/  0xf8,0x7e, 
/*| X . . . . . . , . . . . . X . . |*/  0x80,0x04, 
/*| X . . . . . . , . . . . . X . . |*/  0x80,0x04, 
/*| X . . . . . . , . . . . X . . . |*/  0x80,0x08, 
/*| X . . . . . . , . . . . X . . . |*/  0x80,0x08,
/*| X X X X X . . , . . . X . . . . |*/  0xf8,0x10, 
/*| X . . . . . . , . . . X . . . . |*/  0x80,0x10, 
/*| X . . . . . . , . . X . . . . . |*/  0x80,0x20, 
/*| X . . . . . . , . . X . . . . . |*/  0x80,0x20, 
/*| X . . . . . . , . X . . . . . . |*/  0x80,0x40,
/*| X . . . . . . , . X . . . . . . |*/  0x80,0x40,
//127 Function 8
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| X X X X X . . , . . X X X . . . |*/  0xf8,0x38, 
/*| X . . . . . . , . X . . . X . . |*/  0x80,0x44, 
/*| X . . . . . . , . X . . . X . . |*/  0x80,0x44, 
/*| X . . . . . . , . X . . . X . . |*/  0x80,0x44, 
/*| X . . . . . . , . X . . . X . . |*/  0x80,0x44,
/*| X X X X X . . , . . X X X . . . |*/  0xf8,0x38, 
/*| X . . . . . . , . X . . . X . . |*/  0x80,0x44, 
/*| X . . . . . . , . X . . . X . . |*/  0x80,0x44, 
/*| X . . . . . . , . X . . . X . . |*/  0x80,0x44, 
/*| X . . . . . . , . X . . . X . . |*/  0x80,0x44,
/*| X . . . . . . , . . X X X . . . |*/  0x80,0x38,
//128 Function 9
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| X X X X X . . , . . X X X . . . |*/  0xf8,0x38, 
/*| X . . . . . . , . X . . . X . . |*/  0x80,0x44, 
/*| X . . . . . . , . X . . . X . . |*/  0x80,0x44, 
/*| X . . . . . . , . X . . . X . . |*/  0x80,0x44, 
/*| X . . . . . . , . X . . . X . . |*/  0x80,0x44,
/*| X X X X X . . , . . X X X X . . |*/  0xf8,0x3c, 
/*| X . . . . . . , . . . . . X . . |*/  0x80,0x04, 
/*| X . . . . . . , . . . . . X . . |*/  0x80,0x04, 
/*| X . . . . . . , . . . . . X . . |*/  0x80,0x04, 
/*| X . . . . . . , . . . . . X . . |*/  0x80,0x04,
/*| X . . . . . . , . X X X X . . . |*/  0x80,0x78,
//129 Function 10
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| X X X X X . . . x . . . X X X . |*/  0xf8,0x8e, 
/*| X . . . . . . X X . . X . . . X |*/  0x81,0x91, 
/*| X . . . . . X X X . . X . . . X |*/  0x83,0x91, 
/*| X . . . . . . . X . . X . . . X |*/  0x80,0x91, 
/*| X . . . . . . . X . . X . . . X |*/  0x80,0x91,
/*| X X X X X . . . X . . X . . . X |*/  0xf0,0x91, 
/*| X . . . . . . . X . . X . . . X |*/  0x80,0x91, 
/*| X . . . . . . . X . . X . . . X |*/  0x80,0x91, 
/*| X . . . . . . . X . . X . . . X |*/  0x80,0x91, 
/*| X . . . . . . . X . . X . . . X |*/  0x80,0x91,
/*| X . . . . . . . X . . . X X X . |*/  0x80,0x8e,
//130 Function 11
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| X X X X X . . . x . . . . . X . |*/  0xf8,0x82, 
/*| X . . . . . . X X . . . . X X . |*/  0x81,0x86, 
/*| X . . . . . X X X . . . X X X . |*/  0x83,0x8e, 
/*| X . . . . . . . X . . . . . X . |*/  0x80,0x82, 
/*| X . . . . . . . X . . . . . X . |*/  0x80,0x82,
/*| X X X X X . . . X . . . . . X . |*/  0xf0,0x82, 
/*| X . . . . . . . X . . . . . X . |*/  0x80,0x82, 
/*| X . . . . . . . X . . . . . X . |*/  0x80,0x82, 
/*| X . . . . . . . X . . . . . X . |*/  0x80,0x82, 
/*| X . . . . . . . X . . . . . X . |*/  0x80,0x82,
/*| X . . . . . . . X . . . . . X . |*/  0x80,0x82,
//131 Function 12
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| X X X X X . . . x . . . X X X . |*/  0xf8,0x8e, 
/*| X . . . . . . X X . . X . . . X |*/  0x81,0x91, 
/*| X . . . . . X X X . . . . . . X |*/  0x83,0x81, 
/*| X . . . . . . . X . . . . . . X |*/  0x80,0x81, 
/*| X . . . . . . . X . . . . . X . |*/  0x80,0x82,
/*| X X X X X . . . X . . . . X . . |*/  0xf0,0x84, 
/*| X . . . . . . . X . . . X . . . |*/  0x80,0x88, 
/*| X . . . . . . . X . . X . . . . |*/  0x80,0x90, 
/*| X . . . . . . . X . . X . . . . |*/  0x80,0x90, 
/*| X . . . . . . . X . . X . . . . |*/  0x80,0x90,
/*| X . . . . . . . X . . X X X X X |*/  0x80,0x9f,
//132 Double less than  
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| . . . . . X . , . . . . . X . . |*/  0x04,0x04, 
/*| . . . . X . . , . . . . X . . . |*/  0x08,0x08, 
/*| . . . X . . . , . . . X . . . . |*/  0x10,0x10, 
/*| . . X . . . . , . . X . . . . . |*/  0x20,0x20, 
/*| . X . . . . . , . X . . . . . . |*/  0x40,0x40,
/*| X . . . . . . , X . . . . . . . |*/  0x80,0x80, 
/*| . X . . . . . , . X . . . . . . |*/  0x40,0x40, 
/*| . . X . . . . , . . X . . . . . |*/  0x20,0x20, 
/*| . . . X . . . , . . . X . . . . |*/  0x10,0x10, 
/*| . . . . X . . , . . . . X . . . |*/  0x08,0x08,
/*| . . . . . X . , . . . . . X . . |*/  0x04,0x04,
//132 Double greater than
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| X . . . . . . , X . . . . . . . |*/  0x80,0x80, 
/*| . X . . . . . , . X . . . . . . |*/  0x40,0x40, 
/*| . . X . . . . , . . X . . . . . |*/  0x20,0x20, 
/*| . . . X . . . , . . . X . . . . |*/  0x10,0x10, 
/*| . . . . X . . , . . . . X . . . |*/  0x08,0x08,
/*| . . . . . X . , . . . . . X . . |*/  0x04,0x04, 
/*| . . . . X . . , . . . . X . . . |*/  0x08,0x08, 
/*| . . . X . . . , . . . X . . . . |*/  0x10,0x10, 
/*| . . X . . . . , . . X . . . . . |*/  0x20,0x20, 
/*| . X . . . . . , . X . . . . . . |*/  0x40,0x40,
/*| X . . . . . . , X . . . . . . . |*/  0x80,0x80,
//133 Space Bar
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| X . . . . . . , . . . . . . . X |*/  0x80,0x01, 
/*| X . . . . . . , . . . . . . . X |*/  0x80,0x01, 
/*| X X X X X X X X X X X X X X X X |*/  0xff,0xff,
//   Empty array for future expansion
/*| 8 4 2 1 8 4 2 1 8 4 2 1 8 4 2 1 |*/
/*| . . . . . . . , . . . . . . . . |*/  0x00,0x00,
/*| . . . . . . . , . . . . . . . . |*/  0x00,0x00,
/*| . . . . . . . , . . . . . . . . |*/  0x00,0x00,
/*| . . . . . . . , . . . . . . . . |*/  0x00,0x00,
/*| . . . . . . . , . . . . . . . . |*/  0x00,0x00,
/*| . . . . . . . , . . . . . . . . |*/  0x00,0x00,
/*| . . . . . . . , . . . . . . . . |*/  0x00,0x00,
/*| . . . . . . . , . . . . . . . . |*/  0x00,0x00,
/*| . . . . . . . , . . . . . . . . |*/  0x00,0x00,
/*| . . . . . . . , . . . . . . . . |*/  0x00,0x00,
0x00};//One more byte just in case

const GFXglyph SymbolMono18pt7bGlyphs[] PROGMEM = {
  //Index,  W, H,xAdv,dX, dY
  {     0, 16,21, 21, 3,-19}, // 00 test square
  {    42, 16,15, 21, 3,-18}, // 01 Upper_Left_Arrow
  {    72, 16,15, 21, 3,-18}, // 02 Upper_Right_Arrow
  {   102, 16,15, 21, 3,-16}, // 03 lower left arrow
  {   132, 16,15, 21, 3,-16}, // 04 lower right arrow
  {   162, 16,16, 21, 3,-17}, // 05 power symbol
  {   194, 16,11, 21, 3,-15}, // 06 skip left
  {   216, 16,13, 21, 3,-16}, // 07 rewind
  {   242, 16,13, 21, 3,-16}, // 08 play
  {   268, 16,13, 21, 3,-16}, // 09 fast-forward
  {   294, 16,11, 21, 3,-15}, // 10 skip right
  {   316, 16,10, 21, 3,-15}, // 11 pause
  {   336, 16,10, 21, 3,-15}, // 12 Stop
  {   356, 16, 8, 21, 3,-15}, // 13 ch up
  {   372, 16, 8, 21, 3,-13}, // 14 ch down
  {   388, 16,14, 21, 3,-17}, // 15 page up
  {   416, 16,14, 21, 3,-15}, // 16 page down
  {   444, 16,17, 21, 3,-18}, // 17 Bluetooth
  {   478, 16,15, 21, 3,-18}, // 18 up Arrow
  {   508, 16,15, 21, 3,-18}, // 19 down arrow
  {   538, 16,15, 21, 3,-18}, // 20 left arrow
  {   568, 16,15, 21, 3,-18}, // 21 right arrow
  {   598, 16, 7, 21, 3,-13}, // 22 Return
  {   612, 16,17, 21, 3,-17}, // 23 backspace
  {   646, 16,11, 21, 3,-14}, // 24 delete
  {   668, 16,11, 21, 4,-13}, // 25 F1
  {   690, 16,11, 21, 4,-13}, // 26 F2
  {   712, 16,11, 21, 4,-13}, // 27 F3
  {   734, 16,11, 21, 4,-13}, // 28 F4
  {   756, 16,11, 21, 4,-13}, // 29 F5
  {   778, 16,11, 21, 4,-13}, // 30 F6
  {   800, 16,11, 21, 4,-13}, // 31 F7
  {   822, 16,11, 21, 4,-13}, //127 F8
  {   844, 16,11, 21, 4,-13}, //128 F9
  {   866, 16,11, 21, 2,-13}, //129 F10
  {   888, 16,11, 21, 2,-13}, //130 F11
  {   910, 16,11, 21, 2,-13}, //131 F12
  {   932, 16,11, 21, 3,-13}, //132 double less than
  {   954, 16,11, 21, 3,-13}, //133 double greater than
  {   976, 16, 3, 21, 2,- 2}, //134 space bar
  {     0, 16,21, 21, 3,-19}, //135
  {     0, 16,21, 21, 3,-19}, //136
  {     0, 16,21, 21, 3,-19}, //137
  {     0, 16,21, 21, 3,-19}, //138
  {     0, 16,21, 21, 3,-19}, //139
  {     0, 16,21, 21, 3,-19}, //140
  {     0, 16,21, 21, 3,-19}, //141
  {     0, 16,21, 21, 3,-19}, //142
  {     0, 16,21, 21, 3,-19}, //143
  {    42, 16,15, 21, 3,-18}};//144
  //Index,  W, H,xAdv,dX, dY
const GFXfont SymbolMono18pt7b PROGMEM = {
  (uint8_t  *)SymbolMono18pt7bBitmaps,
  (GFXglyph *)SymbolMono18pt7bGlyphs,
  0,48, 35 //ASCII start, ASCII stop,y Advance
};
#define MY_TEST_SQUARE 0
#define MY_UPPER_LEFT_ARROW 1
#define MY_UPPER_RIGHT_ARROW 2
#define MY_LOWER_LEFT_ARROW 3
#define MY_LOWER_RIGHT_ARROW 4
#define MY_POWER_SYMBOL 5
#define MY_SKIP_LEFT 6
#define MY_REWIND 7
#define MY_PLAY 8
#define MY_FF 9
#define MY_SKIP_RIGHT 10
#define MY_PAUSE 11
#define MY_STOP 12
#define MY_CH_UP 13
#define MY_CH_DOWN 14
#define MY_PAGE_UP 15
#define MY_PAGE_DOWN 16
#define MY_BLUETOOTH 17
#define MY_UP_ARROW 18
#define MY_DOWN_ARROW 19
#define MY_LEFT_ARROW 20
#define MY_RIGHT_ARROW 21
#define MY_RETURN 22
#define MY_BACKSPACE 23
#define MY_DELETE 24
#define MY_F1 25
#define MY_F2 26
#define MY_F3 27
#define MY_F4 28
#define MY_F5 29
#define MY_F6 30
#define MY_F7 31
#define MY_F8 127
#define MY_F9 128
#define MY_F10 129
#define MY_F11 130
#define MY_F12 131
#define MY_DOUBLE_LESS_THAN 132
#define MY_DOUBLE_GREATER_THAN 133
#define MY_SPACE_BAR 134

const GFXglyph CompactSymbolMono18pt7bGlyphs[] PROGMEM = {
  //Index,  W, H,xAdv,dX, dY
  {  0, 16,16, 21, 3,-17 }, // 00 power symbol
  {  32, 16, 17, 21, 3,-18}, // 01 Bluetooth
  {  478, 8, 9, 21, 3,-2 } // 02 smaller bluetooth
};

const GFXfont CompactSymbolMono18pt7b PROGMEM = {
  (uint8_t  *)SymbolMono18pt7bBitmaps,
  (GFXglyph *)CompactSymbolMono18pt7bGlyphs,
  0, 48, 35 //ASCII start, ASCII stop, y Advance
};
