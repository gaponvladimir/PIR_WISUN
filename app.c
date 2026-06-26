/* app.c — Wi-SUN SoC Empty + PIR/Vibration sensor task
 *
 * Hardware (BRD2705A, EFR32xG28):
 *   YS312 DOCI     -> PC01 (breakout pad C01)
 *   Potentiometer  -> PA05 (breakout pad A05, IADC0 single positive input)
 *   Buzzer         -> PC02 (breakout pad C02)
 *   MPU9265 SDA    -> PC03 (I2C0_SDA)
 *   MPU9265 SCL    -> PC04 (I2C0_SCL)
 *   LED1           -> on-board LED (active-high)
 *
 * Architecture:
 *   app_init.c creates the "AppMain" FreeRTOS thread which calls app_task().
 *   All sensor logic runs inside app_task() — no additional threads needed.
 */

#include "app.h"
#include "YS312.h"
#include "motion.h"
#include "vibration.h"
#include "mpu9265.h"
#include "beep.h"

#include "cmsis_os2.h"
#include "sl_simple_led_instances.h"
#include "sl_sleeptimer.h"
#include "em_iadc.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "app_log.h"

#include "sl_wisun_event_mgr.h"

#include <stdio.h>

#include "socket/socket.h"
#include "arpa/inet.h"

#include "sl_wisun_app_core.h"


/* Border Router link-local IPv6 address */
#define BR_IPV6_ADDR    "fe80::8e8b:48ff:fe23:17ce"
#define BR_UDP_PORT     5000U

/* ------------------------------------------------------------------ */
/*  Private state                                                       */
/* ------------------------------------------------------------------ */
static MotionDetector    s_md;
static VibrationDetector s_vd;
static uint32_t          s_start_tick = 0;

static volatile bool s_wisun_connected = false;

/* ------------------------------------------------------------------ */
/*  IADC init — potentiometer on PA05                                  */
/* ------------------------------------------------------------------ */
static void iadc_init(void)
{
    /* Enable clocks */
    CMU_ClockEnable(cmuClock_IADC0, true);
    CMU_ClockEnable(cmuClock_GPIO,  true);

    /* Select FSRCO (20 MHz) as IADC clock source */
    CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_FSRCO);

    /* Configure PA05 as analog input */
    GPIO_PinModeSet(gpioPortA, 5, gpioModeInput, 0);

    /* Allocate analog bus for PA05 (port A, odd pin) to ADC0 */
    GPIO->ABUSALLOC |= GPIO_ABUSALLOC_AODD0_ADC0;

    /* IADC init — 10 MHz ADC clock */
    IADC_Init_t init = IADC_INIT_DEFAULT;
    init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, 10000000, 0);

    /* Config 0: VDD x 0.8 reference (2.64V at 3.3V supply) */
    IADC_AllConfigs_t allConfigs = IADC_ALLCONFIGS_DEFAULT;
    allConfigs.configs[0].reference      = iadcCfgReferenceVddX0P8Buf;
    allConfigs.configs[0].vRef           = 2640;
    allConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(
        IADC0, 1000000, 0, iadcCfgModeNormal, init.srcClkPrescale);

    /* Single conversion: PA05 positive, GND negative */
    IADC_InitSingle_t  initSingle  = IADC_INITSINGLE_DEFAULT;
    IADC_SingleInput_t singleInput = IADC_SINGLEINPUT_DEFAULT;
    singleInput.posInput = iadcPosInputPortAPin5;
    singleInput.negInput = iadcNegInputGnd;

    IADC_reset(IADC0);
    IADC_init(IADC0, &init, &allConfigs);
    IADC_initSingle(IADC0, &initSingle, &singleInput);
}

/* ------------------------------------------------------------------ */
/*  Read potentiometer via IADC single conversion (polling)            */
/* ------------------------------------------------------------------ */
static uint16_t adc_read(void)
{
    IADC_command(IADC0, iadcCmdStartSingle);
    while (IADC_getSingleFifoCnt(IADC0) == 0u) {}
    IADC_Result_t result = IADC_pullSingleFifoResult(IADC0);
    return (uint16_t)(result.data & 0x0FFFu);
}


static void wisun_join_state_cb(sl_wisun_evt_t *evt)
{
    app_log_info("[Wi-SUN] join state: %lu\r\n",
                 (unsigned long)evt->evt.join_state.join_state);
}

static void wisun_connected_cb(sl_wisun_evt_t *evt)
{
    (void)evt;
    app_log_info("[Wi-SUN] connected!\r\n");
    s_wisun_connected = true;
}

/* ------------------------------------------------------------------ */
/*  UDP socket                                                          */
/* ------------------------------------------------------------------ */
static int32_t s_udp_sock = -1;

static void udp_init(void)
{
    s_udp_sock = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
    if (s_udp_sock < 0) {
        app_log_error("[udp] socket create failed: %ld\r\n",
                      (long)s_udp_sock);
    } else {
        app_log_info("[udp] socket OK: %ld\r\n", (long)s_udp_sock);
    }
}

static void udp_send_event(bool motion, bool vibration,
                           int16_t pir, uint32_t energy)
{
    if (s_udp_sock < 0) {
        return;
    }

    /* Build JSON payload */
    char buf[128];
    int len = snprintf(buf, sizeof(buf),
                       "{\"motion\":%d,\"vibration\":%d,"
                       "\"pir\":%d,\"energy\":%lu}",
                       (int)motion, (int)vibration,
                       (int)pir, (unsigned long)energy);

    /* Destination: Border Router */
    sockaddr_in6_t dst = { 0 };
    dst.sin6_family = AF_INET6;
    dst.sin6_port   = htons(BR_UDP_PORT);
    inet_pton(AF_INET6, BR_IPV6_ADDR, &dst.sin6_addr);

    int32_t ret = sendto(s_udp_sock, buf, len, 0,
                         (struct sockaddr *)&dst, sizeof(dst));
    if (ret < 0) {
        app_log_error("[udp] send failed: %ld\r\n", (long)ret);
    } else {
        app_log_info("[udp] sent %d bytes\r\n", len);
    }
}

/* ------------------------------------------------------------------ */
/*  app_task — entry point called by app_init.c FreeRTOS thread        */
/* ------------------------------------------------------------------ */
void app_task(void *arg)
{
    (void)arg;

    /* --- Hardware init --- */
    iadc_init();
    app_log_info("[app] IADC OK\r\n");

    Beep_Init();
    app_log_info("[app] Beep OK\r\n");

    YS312_Init();
    app_log_info("[app] YS312 OK\r\n");

    if (MPU9265_Init()) {
        app_log_info("[app] MPU9265 OK\r\n");
        MPU9265_TestMagnetometer();
    } else {
        app_log_error("[app] MPU9265 init FAILED\r\n");
    }

    /* --- Detector init --- */
    MotionDetector_Init(&s_md);
    VibrationDetector_Init(&s_vd);
    app_log_info("[app] detectors OK\r\n");

#ifdef WISUN_ENABLE
    // Wi-SUN callback initialization and UDP
    app_wisun_em_custom_callback_register(SL_WISUN_MSG_CONNECTED_IND_ID,
                                      wisun_connected_cb);
    app_wisun_em_custom_callback_register(SL_WISUN_MSG_JOIN_STATE_IND_ID,
                                      wisun_join_state_cb);

    /* Start Wi-SUN connection */
    sl_wisun_app_core_network_connect();
    udp_init();
#endif
    /* Record start time for warmup gate */
    s_start_tick = sl_sleeptimer_get_tick_count();

    /* Short beep + LED blink — successful init indicator */
    sl_led_turn_on(&sl_led_led1);
    Beep(100u);
    sl_led_turn_off(&sl_led_led1);

    app_log_info("[app] init done, warmup %lu ms...\r\n",
                 (unsigned long)WARMUP_TIME_MS);

    /* --- Main sensor loop --- */
    while (1) {
        /* 200 ms poll interval — yields CPU to Wi-SUN stack tasks */
        osDelay(SENSOR_READING_PERIOD_MS);

        uint16_t adc_val = 0;
#ifdef SENSIVITY_CONTROL_ENABLE
        /* Read potentiometer and update sensitivity for both detectors */
        adc_val = adc_read();
#endif
        MotionDetector_SetSensitivity(&s_md, adc_val);
        VibrationDetector_SetSensitivity(&s_vd, adc_val);

        bool motion = false;
        YS312_Result pir = {.valid = false, .value = 0};
#ifdef MOTION_ENABLE
        /* Read PIR sensor frame */
        pir = YS312_Read();

        /* Feed PIR sample into motion detection algorithm */
        if (pir.valid) {
            motion = MotionDetector_Update(&s_md, pir.value);
        } else {
            app_log_error("[sensor] YS312 read error\r\n");
        }
#endif

        bool vibration = false;
#ifdef VIBRATION_ENABLE
        /* Read IMU and feed into vibration detector */
        MPU9265_Data imu;
        if (MPU9265_ReadData(&imu)) {
            vibration = VibrationDetector_Update(&s_vd, &imu);
        } else {
            app_log_error("[sensor] MPU9265 read error\r\n");
        }
#endif

        /* Warmup gate — suppress alerts while baselines converge */
        uint32_t elapsed_ms = sl_sleeptimer_tick_to_ms(
            sl_sleeptimer_get_tick_count() - s_start_tick);

        if (elapsed_ms > WARMUP_TIME_MS) {
            if (motion || vibration) {
                sl_led_turn_on(&sl_led_led1);
                Beep(100u);
                sl_led_turn_off(&sl_led_led1);
#ifdef WISUN_ENABLE
                /* Send event over Wi-SUN UDP */
                if (s_wisun_connected) {
                    udp_send_event(motion, vibration, pir.value, s_vd.energy);
                }
#endif
            }
        }
/*
        // Debug output 
        app_log_info("PIR: %6d thr:%4d mot:%s | VIB: e=%6lu thr:%6lu vib:%s\r\n",
                     pir.value, s_md.threshold, motion ? "Y" : "N",
                     (unsigned long)s_vd.energy,
                     (unsigned long)s_vd.threshold,
                     vibration ? "Y" : "N");
*/
    }
}

/* ------------------------------------------------------------------ */
/*  app_process_action — not used in FreeRTOS project                  */
/* ------------------------------------------------------------------ */
void app_process_action(void)
{
}
