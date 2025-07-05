# ESP LCD GC9D01

[![Component Registry](https://components.espressif.com/components/hwzlovedz/esp_lcd_gc9d01/badge.svg)](https://components.espressif.com/components/hwzlovedz/esp_lcd_gc9d01)

Implementation of the GC9D01 LCD controller with esp_lcd component.

| LCD controller | Communication interface | Component name | Link to datasheet |
| :------------: | :---------------------: | :------------: | :---------------: |
| GC9D01         | SPI                     | esp_lcd_gc9d01     | [GALAXYCORE](https://files.waveshare.com/wiki/0.71inch-LCD-Module/GC9D01N_DataSheet_V1.1.pdf) |

## Add to project

Packages from this repository are uploaded to [Espressif's component service](https://components.espressif.com/).
You can add them to your project via `idf.py add-dependancy`, e.g.
```
    idf.py add-dependency "hwzlovedz/esp_lcd_gc9d01^0.0.1"
```

Alternatively, you can create `idf_component.yml`. More is in [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).

## Example use

```c
    ESP_LOGI(TAG, "Initialize SPI bus");
    const spi_bus_config_t bus_config = GC9D01_PANEL_BUS_SPI_CONFIG(EXAMPLE_PIN_NUM_LCD_PCLK, EXAMPLE_PIN_NUM_LCD_MOSI,
                                                                    EXAMPLE_LCD_H_RES * 80 * sizeof(uint16_t));
    ESP_ERROR_CHECK(spi_bus_initialize(EXAMPLE_LCD_HOST, &bus_config, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_config = GC9D01_PANEL_IO_SPI_CONFIG(EXAMPLE_PIN_NUM_LCD_CS, EXAMPLE_PIN_NUM_LCD_DC,
                                                                               example_callback, &example_callback_ctx);
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)EXAMPLE_LCD_HOST, &io_config, &io_handle));

/**
 * Uncomment these lines if use custom initialization commands.
 * The array should be declared as "static const" and positioned outside the function.
 */
// static const gc9d01_lcd_init_cmd_t lcd_init_cmds[] = {
// //  {cmd, { data }, data_size, delay_ms}
//     {0xfe, (uint8_t []){0x00}, 0, 0},
//     {0xef, (uint8_t []){0x00}, 0, 0},
//     {0xeb, (uint8_t []){0x14}, 1, 0},
//     ...
// };

    ESP_LOGI(TAG, "Install GC9D01 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    // gc9d01_vendor_config_t vendor_config = {  // Uncomment these lines if use custom initialization commands
    //     .init_cmds = lcd_init_cmds,
    //     .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(gc9d01_lcd_init_cmd_t),
    // };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,      // Set to -1 if not use
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
#else
        .rgb_endian = LCD_RGB_ENDIAN_RGB,
#endif
        .bits_per_pixel = 16,                           // Implemented by LCD command `3Ah` (16/18)
        // .vendor_config = &vendor_config,            // Uncomment this line if use custom initialization commands
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_gc9d01(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    ESP_ERROR_CHECK(esp_lcd_panel_disp_off(panel_handle, false));
#else
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
#endif
```

There is an example in ESP-IDF with this LCD controller. Please follow this [link](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/lcd/spi_lcd_touch).

*** 
## 推荐与 `esp_lvgl_port` 组件配合使用，点屏如呼吸
```c
/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"
// #include "esp_lcd_gc9a01.h"
#include "esp_lcd_gc9d01.h"
#include "lv_examples.h"

// #include "esp_lcd_touch_tt21100.h"

/* LCD size */
#define EXAMPLE_LCD_H_RES   (160)
#define EXAMPLE_LCD_V_RES   (160)

/* LCD settings */
#define EXAMPLE_LCD_SPI_NUM         (SPI2_HOST)
#define EXAMPLE_LCD_PIXEL_CLK_HZ    (SPI_MASTER_FREQ_80M)
#define EXAMPLE_LCD_CMD_BITS        (8)
#define EXAMPLE_LCD_PARAM_BITS      (8)
#define EXAMPLE_LCD_COLOR_SPACE     (ESP_LCD_COLOR_SPACE_BGR)
#define EXAMPLE_LCD_BITS_PER_PIXEL  (16)
#define EXAMPLE_LCD_DRAW_BUFF_DOUBLE (1)
#define EXAMPLE_LCD_DRAW_BUFF_HEIGHT (160)  // 全刷缓冲区必须比分辨率高，局部刷新可以小于分辨率
// #define EXAMPLE_LCD_BL_ON_LEVEL     (1)

/* LCD pins */
#define EXAMPLE_LCD_GPIO_SCLK       (GPIO_NUM_39)
#define EXAMPLE_LCD_GPIO_MOSI       (GPIO_NUM_38)
#define EXAMPLE_LCD_GPIO_RST        (GPIO_NUM_45)
#define EXAMPLE_LCD_GPIO_DC         (GPIO_NUM_40)
#define EXAMPLE_LCD_GPIO_CS0         (GPIO_NUM_47)
#define EXAMPLE_LCD_GPIO_CS1         (GPIO_NUM_48)
// #define EXAMPLE_LCD_GPIO_BL         (GPIO_NUM_NC)

/* Touch settings */
// #define EXAMPLE_TOUCH_I2C_NUM       (0)
// #define EXAMPLE_TOUCH_I2C_CLK_HZ    (400000)

/* LCD touch pins */
// #define EXAMPLE_TOUCH_I2C_SCL       (GPIO_NUM_18)
// #define EXAMPLE_TOUCH_I2C_SDA       (GPIO_NUM_8)
// #define EXAMPLE_TOUCH_GPIO_INT      (GPIO_NUM_3)

static const char *TAG = "EXAMPLE";

// LVGL image declare
// LV_IMG_DECLARE(esp_logo)

/* LCD IO and panel */
static esp_lcd_panel_io_handle_t lcd_io = NULL;
static esp_lcd_panel_handle_t lcd_panel = NULL;
// static esp_lcd_touch_handle_t touch_handle = NULL;

/* LVGL display and touch */
static lv_display_t *lvgl_disp = NULL;
// static lv_indev_t *lvgl_touch_indev = NULL;

static esp_err_t app_lcd_init(void)
{
    esp_err_t ret = ESP_OK;

    /* LCD backlight */
    // gpio_config_t bk_gpio_config = {
    //     .mode = GPIO_MODE_OUTPUT,
    //     .pin_bit_mask = 1ULL << EXAMPLE_LCD_GPIO_BL
    // };
    // ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    /* LCD initialization */
    ESP_LOGI(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = {
        .sclk_io_num = EXAMPLE_LCD_GPIO_SCLK,
        .mosi_io_num = EXAMPLE_LCD_GPIO_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_DRAW_BUFF_HEIGHT * sizeof(uint16_t),
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(EXAMPLE_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO), TAG, "SPI init failed");

    ESP_LOGI(TAG, "Install panel IO");
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = EXAMPLE_LCD_GPIO_DC,
        .cs_gpio_num = EXAMPLE_LCD_GPIO_CS0,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)EXAMPLE_LCD_SPI_NUM, &io_config, &lcd_io), err, TAG, "New panel IO failed");

    ESP_LOGI(TAG, "Install LCD driver");
    printf(" _______      ______       ______       ______       ______       ____        \r\n");
    printf("/______/\\    /_____/\\     /_____/\\     /_____/\\     /_____/\\     /___/\\       \r\n");
    printf("\\::::__\\/__  \\:::__\\/     \\:::_:\\ \\    \\:::_ \\ \\    \\:::_ \\ \\    \\_::\\ \\      \r\n");
    printf(" \\:\\ /____/\\  \\:\\ \\  __    \\:\\_\\:\\ \\    \\:\\ \\ \\ \\    \\:\\ \\ \\ \\     \\::\\ \\     \r\n");
    printf("  \\:\\\\_  _\\/   \\:\\ \\/_/\\    \\::__:\\ \\    \\:\\ \\ \\ \\    \\:\\ \\ \\ \\    _\\: \\ \\__  \r\n");
    printf("   \\:\\_\\ \\ \\    \\:\\_\\ \\ \\        \\ \\ \\    \\:\\/.:| |    \\:\\_\\ \\ \\  /__\\: \\__/\\ \r\n");
    printf("    \\_____\\/     \\_____\\/         \\_\\/     \\____/_/     \\_____\\/  \\________\\/ \r\n");
    printf("                                                                              \r\n");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_LCD_GPIO_RST,
        .color_space = EXAMPLE_LCD_COLOR_SPACE,
        .bits_per_pixel = EXAMPLE_LCD_BITS_PER_PIXEL,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_gc9d01(lcd_io, &panel_config, &lcd_panel), err, TAG, "New panel failed");

    esp_lcd_panel_reset(lcd_panel);
    esp_lcd_panel_init(lcd_panel);
    esp_lcd_panel_invert_color(lcd_panel, false);
    esp_lcd_panel_mirror(lcd_panel, true, true);
    esp_lcd_panel_disp_on_off(lcd_panel, true);

    /* LCD backlight on */
    // ESP_ERROR_CHECK(gpio_set_level(EXAMPLE_LCD_GPIO_BL, EXAMPLE_LCD_BL_ON_LEVEL));

    return ret;

err:
    if (lcd_panel) {
        esp_lcd_panel_del(lcd_panel);
    }
    if (lcd_io) {
        esp_lcd_panel_io_del(lcd_io);
    }
    spi_bus_free(EXAMPLE_LCD_SPI_NUM);
    return ret;
}

// static esp_err_t app_touch_init(void)
// {
//     /* Initilize I2C */
//     const i2c_config_t i2c_conf = {
//         .mode = I2C_MODE_MASTER,
//         .sda_io_num = EXAMPLE_TOUCH_I2C_SDA,
//         .sda_pullup_en = GPIO_PULLUP_DISABLE,
//         .scl_io_num = EXAMPLE_TOUCH_I2C_SCL,
//         .scl_pullup_en = GPIO_PULLUP_DISABLE,
//         .master.clk_speed = EXAMPLE_TOUCH_I2C_CLK_HZ
//     };
//     ESP_RETURN_ON_ERROR(i2c_param_config(EXAMPLE_TOUCH_I2C_NUM, &i2c_conf), TAG, "I2C configuration failed");
//     ESP_RETURN_ON_ERROR(i2c_driver_install(EXAMPLE_TOUCH_I2C_NUM, i2c_conf.mode, 0, 0, 0), TAG, "I2C initialization failed");

//     /* Initialize touch HW */
//     const esp_lcd_touch_config_t tp_cfg = {
//         .x_max = EXAMPLE_LCD_H_RES,
//         .y_max = EXAMPLE_LCD_V_RES,
//         .rst_gpio_num = GPIO_NUM_NC, // Shared with LCD reset
//         .int_gpio_num = EXAMPLE_TOUCH_GPIO_INT,
//         .levels = {
//             .reset = 0,
//             .interrupt = 0,
//         },
//         .flags = {
//             .swap_xy = 0,
//             .mirror_x = 1,
//             .mirror_y = 0,
//         },
//     };
//     esp_lcd_panel_io_handle_t tp_io_handle = NULL;
//     const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_TT21100_CONFIG();
//     ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)EXAMPLE_TOUCH_I2C_NUM, &tp_io_config, &tp_io_handle), TAG, "");
//     return esp_lcd_touch_new_i2c_tt21100(tp_io_handle, &tp_cfg, &touch_handle);
// }

static esp_err_t app_lvgl_init(void)
{ 
    /* Initialize LVGL */
    const lvgl_port_cfg_t lvgl_cfg = {
        .task_priority = 4,         /* LVGL task priority */
        .task_stack = 4096,         /* LVGL task stack size */
        .task_affinity = -1,        /* LVGL task pinned to core (-1 is no affinity) */
        .task_max_sleep_ms = 500,   /* Maximum sleep in LVGL task */
        .timer_period_ms = 5        /* LVGL timer tick period in ms */
    };
    ESP_RETURN_ON_ERROR(lvgl_port_init(&lvgl_cfg), TAG, "LVGL port initialization failed");

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_io,
        .panel_handle = lcd_panel,
        .buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_DRAW_BUFF_HEIGHT,
        .double_buffer = EXAMPLE_LCD_DRAW_BUFF_DOUBLE,
        .hres = EXAMPLE_LCD_H_RES,
        .vres = EXAMPLE_LCD_V_RES,
        .monochrome = false,
#if LVGL_VERSION_MAJOR >= 9
        .color_format = LV_COLOR_FORMAT_RGB565,
#endif
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = true,
            .buff_spiram = true,    // 双缓冲+DMA使用外部PSRAM
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes = true,
#endif
            .full_refresh = true,   // 这个屌屏幕驱动局部刷新会有乱点
        }
    };
    lvgl_disp = lvgl_port_add_disp(&disp_cfg);

    // /* Add touch input (for selected screen) */
    // const lvgl_port_touch_cfg_t touch_cfg = {
    //     .disp = lvgl_disp,
    //     .handle = touch_handle,
    // };
    // lvgl_touch_indev = lvgl_port_add_touch(&touch_cfg);

    return ESP_OK;
}

static void app_main_display(void)
{
    lv_obj_t *scr = lv_scr_act();

    /* Task lock */
    lvgl_port_lock(0);

    /* LCD HW rotation */
    // lv_disp_set_rotation(lvgl_disp, LV_DISPLAY_ROTATION_0);

    /* Your LVGL objects code here .... */

    lv_example_get_started_1();

    /* Task unlock */
    lvgl_port_unlock();
}

void app_main(void)
{
    /* LCD HW initialization */
    ESP_ERROR_CHECK(app_lcd_init());

    /* Touch initialization */
    // ESP_ERROR_CHECK(app_touch_init());

    /* LVGL initialization */
    ESP_ERROR_CHECK(app_lvgl_init());

    /* Show LVGL objects */
    app_main_display();
}
```
## 终端打印
```
I (26) boot: ESP-IDF v5.4.1 2nd stage bootloader
I (27) boot: compile time Jul  6 2025 00:09:55
I (27) boot: Multicore bootloader
I (27) boot: chip revision: v0.2
I (30) boot: efuse block revision: v1.3
I (33) qio_mode: Enabling default flash chip QIO
I (38) boot.esp32s3: Boot SPI Speed : 80MHz
I (42) boot.esp32s3: SPI Mode       : QIO
I (45) boot.esp32s3: SPI Flash Size : 16MB
I (49) boot: Enabling RNG early entropy source...
I (54) boot: Partition Table:
I (56) boot: ## Label            Usage          Type ST Offset   Length
I (63) boot:  0 nvs              WiFi data        01 02 00009000 00006000
I (69) boot:  1 phy_init         RF data          01 01 0000f000 00001000
I (76) boot:  2 factory          factory app      00 00 00010000 00177000
I (82) boot: End of partition table
I (85) esp_image: segment 0: paddr=00010020 vaddr=3c060020 size=133c0h ( 78784) map
I (105) esp_image: segment 1: paddr=000233e8 vaddr=3fc96d00 size=03090h ( 12432) load
I (107) esp_image: segment 2: paddr=00026480 vaddr=40374000 size=09b98h ( 39832) load
I (116) esp_image: segment 3: paddr=00030020 vaddr=42000020 size=55600h (349696) map
I (168) esp_image: segment 4: paddr=00085628 vaddr=4037db98 size=090c4h ( 37060) load
I (176) esp_image: segment 5: paddr=0008e6f4 vaddr=600fe100 size=0001ch (    28) load
I (183) boot: Loaded app from partition at offset 0x10000
I (183) boot: Disabling RNG early entropy source...
I (194) octal_psram: vendor id    : 0x0d (AP)
I (194) octal_psram: dev id       : 0x02 (generation 3)
I (194) octal_psram: density      : 0x03 (64 Mbit)
I (196) octal_psram: good-die     : 0x01 (Pass)
I (200) octal_psram: Latency      : 0x01 (Fixed)
I (205) octal_psram: VCC          : 0x01 (3V)
I (209) octal_psram: SRF          : 0x01 (Fast Refresh)
I (214) octal_psram: BurstType    : 0x01 (Hybrid Wrap)
I (219) octal_psram: BurstLen     : 0x01 (32 Byte)
I (223) octal_psram: Readlatency  : 0x02 (10 cycles@Fixed)
I (228) octal_psram: DriveStrength: 0x00 (1/1)
I (233) MSPI Timing: PSRAM timing tuning index: 4
I (237) esp_psram: Found 8MB PSRAM device
I (241) esp_psram: Speed: 80MHz
I (244) cpu_start: Multicore app
I (673) esp_psram: SPI SRAM memory test OK
I (681) cpu_start: Pro cpu start user code
I (681) cpu_start: cpu freq: 240000000 Hz
I (681) app_init: Application information:
I (682) app_init: Project name:     LVGL9_LCD_TOUCH_TEST_0.71
I (687) app_init: App version:      1
I (690) app_init: Compile time:     Jul  6 2025 00:09:01
I (695) app_init: ELF file SHA256:  2813ddf0b...
I (700) app_init: ESP-IDF:          v5.4.1
I (703) efuse_init: Min chip rev:     v0.0
I (707) efuse_init: Max chip rev:     v0.99
I (711) efuse_init: Chip rev:         v0.2
I (715) heap_init: Initializing. RAM available for dynamic allocation:
I (721) heap_init: At 3FCAAA68 len 0003ECA8 (251 KiB): RAM
I (726) heap_init: At 3FCE9710 len 00005724 (21 KiB): RAM
I (732) heap_init: At 3FCF0000 len 00008000 (32 KiB): DRAM
I (737) heap_init: At 600FE11C len 00001ECC (7 KiB): RTCRAM
I (742) esp_psram: Adding pool of 8192K of PSRAM memory to heap allocator
I (749) spi_flash: detected chip: generic
I (752) spi_flash: flash io: qio
I (755) sleep_gpio: Configure to isolate all GPIO pins in sleep state
I (762) sleep_gpio: Enable automatic switching of GPIO sleep configuration
I (768) main_task: Started on CPU0
I (778) esp_psram: Reserving pool of 64K of internal memory for DMA/internal allocations
I (778) main_task: Calling app_main()
I (778) EXAMPLE: Initialize SPI bus
I (788) EXAMPLE: Install panel IO
I (788) gpio: GPIO[40]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (798) EXAMPLE: Install LCD driver
 _______      ______       ______       ______       ______       ____
/______/\    /_____/\     /_____/\     /_____/\     /_____/\     /___/\
\::::__\/__  \:::__\/     \:::_:\ \    \:::_ \ \    \:::_ \ \    \_::\ \
 \:\ /____/\  \:\ \  __    \:\_\:\ \    \:\ \ \ \    \:\ \ \ \     \::\ \
  \:\\_  _\/   \:\ \/_/\    \::__:\ \    \:\ \ \ \    \:\ \ \ \    _\: \ \__
   \:\_\ \ \    \:\_\ \ \        \ \ \    \:\/.:| |    \:\_\ \ \  /__\: \__/\
    \_____\/     \_____\/         \_\/     \____/_/     \_____\/  \________\/

I (858) gpio: GPIO[45]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (868) gc9d01: LCD panel create success, version: 0.0.1
W (988) gc9d01: The 3Ah command has been used and will be overwritten by external initialization sequence
W (988) gc9d01: The 36h command has been used and will be overwritten by external initialization sequence
I (1188) LVGL: Starting LVGL task
I (1198) main_task: Returned from app_main()
```