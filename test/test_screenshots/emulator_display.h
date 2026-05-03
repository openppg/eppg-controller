#ifndef TEST_SCREENSHOTS_EMULATOR_DISPLAY_H_
#define TEST_SCREENSHOTS_EMULATOR_DISPLAY_H_

#include <lvgl.h>
#include <stdint.h>

// Display dimensions (must match real hardware)
#define SCREEN_WIDTH 160
#define SCREEN_HEIGHT 128

// Framebuffer access
uint16_t* emulator_get_framebuffer();

// Initialize LVGL and create a framebuffer-backed display
// Returns the created display object
lv_disp_t* emulator_init_display(bool darkMode);

// Force a full render of the current LVGL screen into the framebuffer
void emulator_render_frame();

// Save the current framebuffer as a 24-bit BMP file
// Returns true on success
bool emulator_save_bmp(const char* filename);

// Compare two BMP files pixel-by-pixel.
// Returns the number of differing pixels (0 = identical).
int emulator_compare_bmp(const char* file_a, const char* file_b);

// Save a 3-panel side-by-side diff BMP: [reference | output | diff-highlighted].
// Differing pixels are shown as magenta in the diff panel; matching pixels are dimmed.
// Returns true on success.
bool emulator_save_diff_bmp(const char* ref_path, const char* out_path,
                             const char* diff_path);

// Clean up LVGL state between tests
void emulator_teardown();

#endif  // TEST_SCREENSHOTS_EMULATOR_DISPLAY_H_
