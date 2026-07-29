#include <stdio.h>
#include <string.h>
#include <unistd.h>
 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"

static const char *TAG = "Neptune camera module";

uint8_t neptuneCameraModuleError = 0;

static QueueHandle_t LTEframeQueue;

static StaticSemaphore_t sem_buf;
static SemaphoreHandle_t LTEready;

// camera pins
#define CAM_PIN_PWDN -1
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK 14
#define CAM_PIN_SCCB_SDA 18
#define CAM_PIN_SCCB_SCL 8
 
#define CAM_PIN_D7 13
#define CAM_PIN_D6 21
#define CAM_PIN_D5 47
#define CAM_PIN_D4 38
#define CAM_PIN_D3 40
#define CAM_PIN_D2 42
#define CAM_PIN_D1 41
#define CAM_PIN_D0 39
#define CAM_PIN_VSYNC 10
#define CAM_PIN_HREF 12
#define CAM_PIN_PCLK 48

// microSD pins
#define SD_PIN_CLK GPIO_NUM_7
#define SD_PIN_CMD GPIO_NUM_15
#define SD_PIN_D0 GPIO_NUM_6

#define MOUNT_POINT "/sdcard"
#define SEGMENT_US (30 * 1000 * 1000) // 30 s per file
#define FSYNC_US (2 * 1000 * 1000) // flush FAT every 2 s

// definging error codes
#define cameraInitFail (1U << 0) // Camera init failed bit
#define sdcardMountFail (1U << 1) // SDcard mount failed bit
#define fileOpenFail (1U << 2) // file open fail bit
#define frameGrabFail (1U << 3) // Frame grab fail bit
#define writeError (1U << 4) // Write error fail
#define recordingError (1U << 5) // Recording stopped

// 12 bytes per frame, written to the .IDX sidecar
typedef struct
{
    uint32_t len; // JPEG length in bytes
    uint32_t ts_ms; // esp_timer_get_time() at capture
} frame_rec_t;

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SCCB_SDA,
    .pin_sccb_scl = CAM_PIN_SCCB_SCL,
 
    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href  = CAM_PIN_HREF,
    .pin_pclk  = CAM_PIN_PCLK,
 
    // XCLK freq: 20 MHz
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
 
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_HD, // 1280x720
    .jpeg_quality = 12,
    .fb_count = 5,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

static esp_err_t camera_init(void)
{
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_camera_init failed: 0x%x", err);
        neptuneCameraModuleError = neptuneCameraModuleError | cameraInitFail;

        return err;
    }
    else
    {
        neptuneCameraModuleError = (neptuneCameraModuleError) & (~cameraInitFail);
    }
 
    sensor_t *s = esp_camera_sensor_get();
    ESP_LOGI(TAG, "sensor PID = 0x%04x", s->id.PID); // 0x5640
 
    s->set_vflip(s, 0);
    s->set_hmirror(s, 0);
 
    s->set_brightness(s, 0);
    s->set_contrast(s, 0);
    s->set_saturation(s, 0);
    s->set_whitebal(s, 1);
    s->set_gain_ctrl(s, 1);
    s->set_exposure_ctrl(s, 1);
 
    return ESP_OK;
}

static sdmmc_card_t *s_card;

static esp_err_t sd_init(void)
{
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT;
    host.slot = SDMMC_HOST_SLOT_1;
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED; // 40 MHz
 
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.width = 1;
    slot.clk = SD_PIN_CLK;
    slot.cmd = SD_PIN_CMD;
    slot.d0 = SD_PIN_D0;
    slot.flags = SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
 
    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files = 3,
        .allocation_unit_size = 32 * 1024,
    };
 
    esp_err_t err = esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot, &mount_cfg, &s_card);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "mount failed: %s", esp_err_to_name(err));
        neptuneCameraModuleError = neptuneCameraModuleError | sdcardMountFail;

        return err;
    }
    else
    {
        neptuneCameraModuleError = (neptuneCameraModuleError) & (~sdcardMountFail);
    }

    sdmmc_card_print_info(stdout, s_card);
    return ESP_OK;
}

/* Find the lowest unused VIDnnnnn index so a reboot doesn't overwrite. */
static uint32_t next_segment_index(void)
{
    char path[64];
    for (uint32_t i = 1; i < 100000; i++)
    {
        snprintf(path, sizeof(path), MOUNT_POINT "/VID%05lu.MJP", (unsigned long)i);
        if (access(path, F_OK) != 0)
        {
            return i;
        }
    }
    return 0;
}

typedef struct {
    FILE *mjp;
    FILE *idx;
    int64_t opened_us;
    uint32_t frames;
    uint64_t bytes;
} segment_t;

static esp_err_t segment_open(segment_t *seg, uint32_t index)
{
    char path[64];
 
    snprintf(path, sizeof(path), MOUNT_POINT "/VID%05lu.MJP", (unsigned long)index);
    seg->mjp = fopen(path, "wb");
    if (!seg->mjp)
    {
        ESP_LOGE(TAG, "cannot open %s", path);
        neptuneCameraModuleError = neptuneCameraModuleError | fileOpenFail;

        return ESP_FAIL;
    }
    else
    {
        neptuneCameraModuleError = (neptuneCameraModuleError) & (~fileOpenFail);
    }
 
    snprintf(path, sizeof(path), MOUNT_POINT "/VID%05lu.IDX", (unsigned long)index);
    seg->idx = fopen(path, "wb");
    if (!seg->idx)
    {
        fclose(seg->mjp);
        seg->mjp = NULL;

        ESP_LOGE(TAG, "cannot open %s", path);
        neptuneCameraModuleError = neptuneCameraModuleError | fileOpenFail;

        return ESP_FAIL;
    }
    else
    {
        neptuneCameraModuleError = (neptuneCameraModuleError) & (~fileOpenFail);
    }
 
    seg->opened_us = esp_timer_get_time();
    seg->frames = 0;
    seg->bytes = 0;
    ESP_LOGI(TAG, "segment %lu open", (unsigned long)index);
    return ESP_OK;
}

static void segment_close(segment_t *seg)
{
    if (seg->mjp)
    {
        fclose(seg->mjp);
        seg->mjp = NULL;
    }
    if (seg->idx)
    {
        fclose(seg->idx);
        seg->idx = NULL;
    }

    ESP_LOGI(TAG, "segment closed: %lu frames, %llu bytes", (unsigned long)seg->frames, (unsigned long long)seg->bytes);
}

static void record_task(void *arg)
{
    segment_t seg = {0};
    uint32_t index = next_segment_index();
    int64_t last_fsync = esp_timer_get_time();
 
    if (segment_open(&seg, index) != ESP_OK)
    {
        vTaskDelete(NULL);
    }
 
    while (1)
    {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb)
        {
            neptuneCameraModuleError = neptuneCameraModuleError | frameGrabFail;
            ESP_LOGW(TAG, "frame grab failed");
            continue;
        }
        else
        {
            neptuneCameraModuleError = (neptuneCameraModuleError) & (~frameGrabFail);
        }
 
        int64_t now = esp_timer_get_time();
 
        // Append the raw JPEG. No container, no seeking, no header patching.
        size_t written = fwrite(fb->buf, 1, fb->len, seg.mjp);
        if (written != fb->len)
        {
            neptuneCameraModuleError = neptuneCameraModuleError | writeError;
            ESP_LOGE(TAG, "short write: %u/%u", (unsigned)written, (unsigned)fb->len);
        }
        else
        {
            neptuneCameraModuleError = (neptuneCameraModuleError) & (~writeError);
        }
 
        frame_rec_t rec = {
            .len = fb->len,
            .ts_ms = (uint32_t)(now / 1000)
        };

        fwrite(&rec, sizeof(rec), 1, seg.idx);
 
        seg.frames++;
        seg.bytes += fb->len;

        // if the LTE task is not transmitting, give it a new frame (the LTE task should give back the buffer to the camera driver)
        if (SemaphoreTake(LTEready, 0) == pdTRUE)
        {
            if (xQueueSend(LTEframeQueue, &fb, 0) == pdTRUE)
            {
                // frame is given to the LTE task
            }
            else
            {
                // won't ever get there
            }
        }
        else
        {
            esp_camera_fb_return(fb);
        }
 
        // Bound the data loss window on power cut.
        if (now - last_fsync > FSYNC_US)
        {
            fflush(seg.mjp); fsync(fileno(seg.mjp));
            fflush(seg.idx); fsync(fileno(seg.idx));
            last_fsync = now;
        }
 
        // Rotate segments so a power cut costs at most one file.
        if (now - seg.opened_us > SEGMENT_US)
        {
            segment_close(&seg);
            index++;
            if (segment_open(&seg, index) != ESP_OK)
            {
                break;
            }
        }
    }
 
    segment_close(&seg);
    neptuneCameraModuleError = neptuneCameraModuleError | recordingError;
    vTaskDelete(NULL);
}

static void LTEsend(void *arg)
{
    camera_fb_t *LTEframe;

    while (1)
    {
        // set the binary semphore so the SD task knows this task is ready to take a new frame to transmit
        xSemaphoreGive(LTEready);

        xQueueReceive(LTEframeQueue, &LTEframe, portMAX_DELAY); // getting the new frame

        // sending the frame

        // giving back the frambuffer
        esp_camera_fb_return(LTEframe);
    }
}

void app_main(void)
{
    LTEframeQueue = xQueueCreate(1, sizeof(camera_fb_t *)); // used to give a frame to the LTE task

    LTEready = xSemaphoreCreateBinaryStatic(&sem_buf); // for LTE task frame grab protocol

    ESP_ERROR_CHECK(camera_init()); // Camera init
    ESP_ERROR_CHECK(sd_init()); // SD init
    // LTE init
 
    xTaskCreatePinnedToCore(record_task, "record", 8192, NULL, 5, NULL, 1); // Create camera + sd task
    // Create LTE task
}
