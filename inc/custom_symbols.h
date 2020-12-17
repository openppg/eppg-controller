
const uint8_t SymbolBitmaps[] PROGMEM = {
// Power symbol
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
// Bluetooth
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
// Bluetooth mini
/*| 8 4 2 1 8 4 2 1 |*/
/*| . . . X X . . . |*/  0x18,
/*| . . . X . X . . |*/  0x14,
/*| x . . X . . X . |*/  0x92,
/*| . X . X . X . . |*/  0x54,
/*| . . X X X . . . |*/  0x38,
/*| . x . X . X . . |*/  0x54,
/*| x . . X . . X . |*/  0x92,
/*| . . . X . X . . |*/  0x14,
/*| . . . X X . . . |*/  0x18,

0x00};//One more byte just in case

const GFXglyph SymbolGlyphs[] PROGMEM = {
  //Index,  W, H,xAdv,dX, dY
  {  0, 16,16, 21, 3,-17 }, // 00 power symbol
  {  32, 16, 17, 21, 3,-18}, // 01 Bluetooth
  {  66, 8, 9, 21, 3,-2 } // 02 smaller bluetooth
};
//Index,  W, H,xAdv,dX, dY
const GFXfont Symbol18 PROGMEM = {
  (uint8_t  *)SymbolBitmaps,
  (GFXglyph *)SymbolGlyphs,
  0,48, 35 //ASCII start, ASCII stop,y Advance
};

void drawSymbol(uint16_t x, uint16_t y, uint8_t c, uint16_t color, uint16_t bg, uint8_t char_size) {
  if ( ( c >= 32 ) && ( c <= 126 ) ) { // If it's 33-126 then use standard mono 18 font
      display.setFont();
  } else {
    display.setFont(&Symbol18);  // Otherwise use special symbol font
    if ( c > 126 ) {  // Remap anything above 126 to be in the range 32 and upwards
      c-=(127-32);
    }
  }
  display.drawChar(x, y, c, color, bg, char_size);
}
