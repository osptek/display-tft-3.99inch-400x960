#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_ldo_regulator.h"

/* LVGL v9 头文件 */
#include "lvgl.h"
#include "lv_demos.h"

/* 驱动头文件 */
#include "esp_lcd_axs15260.h"
#include "axs15260_init_cmds.h"

/* 屏幕分辨率 */
#define EXAMPLE_LCD_H_RES   (400)
#define EXAMPLE_LCD_V_RES   (960)

/* 色深配置：24位 (RGB888) */
#define EXAMPLE_LCD_BPP     (24)
#define MIPI_DPI_PX_FORMAT  (LCD_COLOR_FMT_RGB888)
#define LV_COLOR_FORMAT     (LV_COLOR_FORMAT_RGB888)

/* MIPI DSI LDO 电源配置 */
#define EXAMPLE_MIPI_DSI_PHY_PWR_LDO_CHAN       (3)
#define EXAMPLE_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV (2500)

#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL           (1)
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL          (!EXAMPLE_LCD_BK_LIGHT_ON_LEVEL)
#define EXAMPLE_PIN_NUM_BK_LIGHT                (20)
#define EXAMPLE_BK_LIGHT_OFF_TEST               (0) // 1: 背光保持关闭（INT 脉冲对照测试），0: 正常开背光
#define EXAMPLE_PIN_NUM_LCD_RST                 (5)
/* TP_RST = GPIO6。AXS15260D 是 INCELL，显示和触摸共用一颗芯片，板上 TP_RST 无上拉，
 * 由 app_lcd_init() 在显示复位前拉高并一直保持；触摸驱动不再单独复位，否则会打乱显示 */
#define EXAMPLE_PIN_NUM_TP_RST                  (GPIO_NUM_6)

/* 1 = 只循环显示 DSI 控制器内置 BIST 图案，不跑 LVGL 和触摸 */
#define EXAMPLE_LCD_DSI_BIST (0)

/* 1 = 屏端 (AXS15260) BIST：A2[0]=0x21，屏自己生成测试图案，不跑 LVGL 和触摸 */
#define EXAMPLE_LCD_PANEL_BIST (0)

/* 触摸功能总开关 */
#define EXAMPLE_USE_TOUCH (1)

#if EXAMPLE_USE_TOUCH
/* 触摸引脚定义 */
#define EXAMPLE_TOUCH_I2C_SCL       (GPIO_NUM_8)
#define EXAMPLE_TOUCH_I2C_SDA       (GPIO_NUM_7)
#define EXAMPLE_TOUCH_RST           (GPIO_NUM_NC)
#define EXAMPLE_TOUCH_INT           (GPIO_NUM_21) // 触摸中断引脚
#define EXAMPLE_STATIC_SCREEN_TEST  (0) // 1: 开机只显示纯黑静态画面（INT 脉冲对照测试），0: lv_demo_widgets

#define EXAMPLE_TOUCH_I2C_NUM       (0)
#define EXAMPLE_TOUCH_I2C_CLK_HZ    (400000)
#endif

static const char *TAG = "EXAMPLE";

/* LCD 组件句柄 */
static esp_lcd_panel_handle_t lcd_panel = NULL;
static esp_lcd_panel_io_handle_t io_handle = NULL;

#if EXAMPLE_USE_TOUCH
static esp_lcd_touch_handle_t touch_handle = NULL;
static SemaphoreHandle_t touch_mux = NULL;
static void example_touch_isr_cb(esp_lcd_touch_handle_t tp);
#endif

/* LVGL 全局互斥锁与显示句柄 */
static SemaphoreHandle_t lvgl_mux = NULL;
static lv_display_t *lvgl_disp = NULL;

/* 1. 开启 MIPI DSI PHY 电源 */
static void example_bsp_enable_dsi_phy_power(void)
{
    esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
#ifdef EXAMPLE_MIPI_DSI_PHY_PWR_LDO_CHAN
    esp_ldo_channel_config_t ldo_mipi_phy_config = {
        .chan_id = EXAMPLE_MIPI_DSI_PHY_PWR_LDO_CHAN,
        .voltage_mv = EXAMPLE_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_mipi_phy_config, &ldo_mipi_phy));
    ESP_LOGI(TAG, "MIPI DSI PHY Powered on");
#endif
}

static void example_bsp_init_lcd_backlight(void)
{
#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT};
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
#endif
}

static void example_bsp_set_lcd_backlight(uint32_t level)
{
#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, level);
#endif
}

/* 2. 初始化 MIPI DSI 屏 */
static esp_err_t app_lcd_init(void)
{
    esp_err_t ret = ESP_OK;

    const gpio_config_t tp_rst_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_TP_RST,
    };
    ESP_ERROR_CHECK(gpio_config(&tp_rst_config));
    ESP_ERROR_CHECK(gpio_set_level(EXAMPLE_PIN_NUM_TP_RST, 1));

    example_bsp_enable_dsi_phy_power();
    example_bsp_init_lcd_backlight();
    example_bsp_set_lcd_backlight(EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL);

    esp_lcd_dsi_bus_handle_t mipi_dsi_bus;
    esp_lcd_dsi_bus_config_t bus_config = {
        .bus_id = 0,
        .num_data_lanes = 2,
        .phy_clk_src = 0,
        .lane_bit_rate_mbps = 850,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus), err, TAG, "DSI bus init failed");

    ESP_LOGI(TAG, "Install MIPI DSI LCD control panel IO");
    esp_lcd_dbi_io_config_t dbi_config = {
        .virtual_channel = 0,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &io_handle), err, TAG, "Panel IO DBI failed");

    esp_lcd_dpi_panel_config_t dpi_config = {
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = 34,
        .virtual_channel = 0,
        .in_color_format = MIPI_DPI_PX_FORMAT, // 适配 RGB888
        .num_fbs = 1,
        .video_timing = {
            .h_size = EXAMPLE_LCD_H_RES,
            .v_size = EXAMPLE_LCD_V_RES,
            .hsync_back_porch = 100,
            .hsync_pulse_width = 10,
            .hsync_front_porch = 100,
            .vsync_back_porch = 60,
            .vsync_pulse_width = 4,
            .vsync_front_porch = 120,
        },
    };

    axs15260_vendor_config_t vendor_config = {
        .init_cmds = lcd_init_cmds,
        .init_cmds_size = lcd_init_cmds_size,
        .flags.use_mipi_interface = 1,
        .mipi_config = {
            .dsi_bus = mipi_dsi_bus,
            .dpi_config = &dpi_config,
        },
    };

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = EXAMPLE_LCD_BPP, // 24位
        .vendor_config = &vendor_config,
    };

    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_axs15260(io_handle, &panel_config, &lcd_panel), err, TAG, "New panel failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_reset(lcd_panel), err, TAG, "Panel reset failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_init(lcd_panel), err, TAG, "Panel init failed");
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_panel, true));

    example_bsp_set_lcd_backlight(EXAMPLE_BK_LIGHT_OFF_TEST ? EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL : EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);

    return ret;

err:
    if (lcd_panel) {
        esp_lcd_panel_del(lcd_panel);
    }
    return ret;
}

#if EXAMPLE_USE_TOUCH
/* 3. 初始化触摸芯片及中断信号量 */
static esp_err_t app_touch_init(void)
{
    const i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = EXAMPLE_TOUCH_I2C_NUM,
        .sda_io_num = EXAMPLE_TOUCH_I2C_SDA,
        .scl_io_num = EXAMPLE_TOUCH_I2C_SCL,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &i2c_bus));
    
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_AXS15260_CONFIG();
    tp_io_config.scl_speed_hz = EXAMPLE_TOUCH_I2C_CLK_HZ;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &tp_io_config, &tp_io_handle));

    touch_mux = xSemaphoreCreateBinary();
    assert(touch_mux);

    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_H_RES,
        .y_max = EXAMPLE_LCD_V_RES,
        .rst_gpio_num = EXAMPLE_TOUCH_RST,
        .int_gpio_num = EXAMPLE_TOUCH_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0, // 低电平触发
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
        .interrupt_callback = example_touch_isr_cb,
    };
    
    return esp_lcd_touch_new_i2c_axs15260(tp_io_handle, &tp_cfg, &touch_handle);
}

/* 4. 触摸中断 ISR 触发信号量 */
static volatile uint32_t s_touch_int_cnt = 0; /* INT 下降沿计数，用于检测固定脉冲 */

static void touch_int_monitor_task(void *arg)
{
    uint32_t last = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        uint32_t now = s_touch_int_cnt;
        ESP_LOGI("TOUCH_INT", "GPIO%d falling edges/s = %u, level = %d",
                 (int)EXAMPLE_TOUCH_INT, (unsigned)(now - last), gpio_get_level(EXAMPLE_TOUCH_INT));
        last = now;
    }
}

static void example_touch_isr_cb(esp_lcd_touch_handle_t tp)
{
    s_touch_int_cnt++;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(touch_mux, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

/* 5. LVGL 触摸读取回调 */
static void lvgl_touch_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    esp_lcd_touch_handle_t tp = (esp_lcd_touch_handle_t)lv_indev_get_user_data(indev);
    assert(tp);

    esp_lcd_touch_point_data_t point;
    uint8_t tp_cnt = 0;

    if (xSemaphoreTake(touch_mux, 0) == pdTRUE) {
        esp_lcd_touch_read_data(tp);
    }

    esp_err_t err = esp_lcd_touch_get_data(tp, &point, &tp_cnt, 1);
    if (err == ESP_OK && tp_cnt > 0) {
        data->point.x = point.x;
        data->point.y = point.y;
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}
#endif

/* 6. LVGL 刷屏回调 */
static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)lv_display_get_user_data(disp);
    esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, px_map);
    lv_display_flush_ready(disp);
}

/* 7. LVGL Tick 定时器 */
static void lvgl_increase_tick(void *arg)
{
    lv_tick_inc(2);
}

/* 8. LVGL 后台任务循环 */
static void lvgl_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL v9 task");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (xSemaphoreTake(lvgl_mux, portMAX_DELAY) == pdTRUE) {
            lv_timer_handler();
            xSemaphoreGive(lvgl_mux);
        }
    }
}

/* 9. 手动初始化 LVGL v9（24位色深 + 双缓冲） */
static esp_err_t app_lvgl_init(void)
{
    lvgl_mux = xSemaphoreCreateMutex();
    assert(lvgl_mux);

    lv_init();

    lvgl_disp = lv_display_create(EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES);
    assert(lvgl_disp != NULL);

    /* 设置颜色格式为 RGB888（24位） */
    lv_display_set_color_format(lvgl_disp, LV_COLOR_FORMAT);

    /* 计算 24 位色深下单缓冲大小 (400 * 960 * 3 字节 ≈ 1.15 MB) */
    size_t buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * 3;
    
    /* 在 PSRAM 中分配双缓冲所需的内存（buf1 与 buf2） */
    void *buf1 = heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(buf1 != NULL);

    void *buf2 = heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(buf2 != NULL);

    /* 注册双缓冲配置 */
    lv_display_set_buffers(lvgl_disp, buf1, buf2, buffer_size, LV_DISPLAY_RENDER_MODE_PARTIAL);
    
    lv_display_set_user_data(lvgl_disp, lcd_panel);
    lv_display_set_flush_cb(lvgl_disp, lvgl_flush_cb);

#if EXAMPLE_USE_TOUCH
    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, lvgl_touch_cb);
    lv_indev_set_user_data(indev, touch_handle);
    lv_indev_set_display(indev, lvgl_disp);
#endif

    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &lvgl_increase_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, 2000)); // 2ms

    xTaskCreatePinnedToCore(lvgl_task, "LVGL_Task", 8192, NULL, 5, NULL, tskNO_AFFINITY);

    return ESP_OK;
}

#if EXAMPLE_LCD_PANEL_BIST
/* 屏端 BIST：A2 par0 默认 0x00，改 0x21 进入 BIST 模式，图案由 AXS15260 自己生成，不依赖 DSI 视频数据。
 * A2[17..18] 按同厂 AXS15231 的注释应为 cr_pat_sel[15:8] / [7:0]，初始化表里是 FF FF */
static void panel_write_a2(const uint8_t *a2, size_t len)
{
    static const uint8_t unlock[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5A, 0xA5};
    static const uint8_t lock[]   = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0xBB, unlock, sizeof(unlock)));
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0xA2, a2, len));
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0xBB, lock, sizeof(lock)));
}

static void panel_bist(void)
{
    static const char *const pat_name[16] = {
        "red", "green", "blue", "white", "black", "chess", "mid gray", "gray col",
        "gray row", "strp col", "strp row", "data partial", "partial inverse", "display rec", "data one", "crosstalk",
    };
    const axs15260_lcd_init_cmd_t *a2_cmd = NULL;
    for (int i = 0; i < lcd_init_cmds_size; i++) {
        if (lcd_init_cmds[i].cmd == 0xA2) {
            a2_cmd = &lcd_init_cmds[i];
        }
    }
    assert(a2_cmd && a2_cmd->data_bytes > 18 && a2_cmd->data_bytes <= 64);
    const size_t len = a2_cmd->data_bytes;
    uint8_t buf[64];
    memcpy(buf, a2_cmd->data, len);
    buf[0] = 0x21;

    ESP_LOGW(TAG, "Panel BIST: A2[0]=0x21, pat_sel=0x%02X%02X (from init table)", buf[17], buf[18]);
    panel_write_a2(buf, len);
    vTaskDelay(pdMS_TO_TICKS(10000));

    while (1) {
        for (int p = 0; p < 16; p++) {
            uint16_t sel = 1u << p;
            buf[17] = sel >> 8;
            buf[18] = sel & 0xFF;
            ESP_LOGI(TAG, "Panel BIST: pattern %d (%s), A2[17..18]=0x%02X%02X", p, pat_name[p], buf[17], buf[18]);
            panel_write_a2(buf, len);
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }
}
#endif

void app_main(void)
{
    /* 1. 初始化 MIPI LCD */
    ESP_ERROR_CHECK(app_lcd_init());

#if EXAMPLE_LCD_PANEL_BIST
    panel_bist(); /* 不返回 */
#endif

#if EXAMPLE_LCD_DSI_BIST
    /* 图案由 DSI 控制器硬件生成，不经过 PSRAM 帧缓冲和 LVGL */
    static const struct { mipi_dsi_pattern_type_t type; const char *name; } bist[] = {
        {MIPI_DSI_PATTERN_BAR_VERTICAL,   "vertical color bars"},
        {MIPI_DSI_PATTERN_BAR_HORIZONTAL, "horizontal color bars"},
        {MIPI_DSI_PATTERN_BER_VERTICAL,   "vertical BER pattern"},
    };
    while (1) {
        for (int i = 0; i < sizeof(bist) / sizeof(bist[0]); i++) {
            ESP_LOGI(TAG, "DSI BIST: %s", bist[i].name);
            ESP_ERROR_CHECK(esp_lcd_dpi_panel_set_pattern(lcd_panel, bist[i].type));
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }
#endif

#if EXAMPLE_USE_TOUCH
    /* 2. 初始化触摸及中断 */
    ESP_ERROR_CHECK(app_touch_init());
    xTaskCreate(touch_int_monitor_task, "touch_int_mon", 3072, NULL, 3, NULL);
#endif

    /* 3. 初始化 LVGL（24位色深双缓冲） */
    ESP_ERROR_CHECK(app_lvgl_init());

    /* 4. 创建 UI 界面 */
    if (xSemaphoreTake(lvgl_mux, portMAX_DELAY) == pdTRUE) {
#if EXAMPLE_STATIC_SCREEN_TEST
        lv_obj_t *scr = lv_screen_active();
        lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
#else
        lv_demo_widgets();
#endif
        xSemaphoreGive(lvgl_mux);
    }
}
