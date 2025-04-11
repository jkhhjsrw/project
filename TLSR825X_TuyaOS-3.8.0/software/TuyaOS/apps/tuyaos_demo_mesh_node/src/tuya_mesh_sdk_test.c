/**
 * @file tuya_mesh_sdk_test.c
 * @brief This is tuya_mesh_sdk_test file
 * @version 1.0
 * @date 2021-09-10
 *
 * @copyright Copyright 2021-2023 Tuya Inc. All Rights Reserved.
 *
 */

#include "tuya_sdk_test.h"

#if (TUYA_SDK_TEST)

//tal system
#include "tal_log.h"
#include "tal_memory.h"
#include "tal_sleep.h"
#include "tal_ota.h"
#include "tal_queue.h"
#include "tal_system.h"
#include "tal_sw_timer.h"
//tal driver
#include "tal_rtc.h"
#include "tal_utc.h"
#include "tal_uart.h"
#include "tal_flash.h"
#include "tal_adc.h"
#include "tal_gpio.h"
#include "tal_pwm.h"
#include "tal_spi.h"
#include "tal_i2c.h"
#include "tal_watchdog.h"
#include "tal_oled.h"
//tal bluetooth
#include "tal_bluetooth.h"
#include "tal_bluetooth_mesh_device.h"
#include "tal_bluetooth_mesh_firmware_infor_inner.h"

#include "tal_util.h"
#include "tal_mesh_factory_test.h"

#include "tkl_wakeup.h"
#include "tkl_sleep.h"
#include "board.h"

/***********************************************************************
 ********************* constant ( macro and enum ) *********************
 **********************************************************************/
#define     TEST_GROUP_VARIABLE \
                OPERATE_RET ret  = 0; \
                UINT32_T    idx  = 0; \
                UINT8_T     *rsp = p_rsp_data;

#define     TEST_RSP \
                rsp[idx++] = (ret >> 24) & 0xFF; \
                rsp[idx++] = (ret >> 16) & 0xFF; \
                rsp[idx++] = (ret >> 8) & 0xFF; \
                rsp[idx++] = (ret) & 0xFF;

/***********************************************************************
 ********************* struct ******************************************
 **********************************************************************/
#pragma pack(1)
typedef struct {
    UINT8_T  type;
    UINT8_T  group_id;
    UINT8_T  cmd_id;
    UINT8_T  value[];
} mesh_test_cmd_t;
#pragma pack()

/***********************************************************************
 ********************* variable ****************************************
 **********************************************************************/
STATIC TIMER_ID reboot_timer_id = NULL;
STATIC TIMER_ID mesh_test_enter_sleep_timer_id = NULL;

/***********************************************************************
 ********************* function ****************************************
 **********************************************************************/
VOID_T test_reboot_timer_callback(TIMER_ID timer_id, VOID_T *arg)
{
    tal_system_reset();
}

VOID_T test_enter_sleep_callback(TIMER_ID timer_id, VOID_T *arg)
{
    tkl_cpu_allow_sleep();
}

VOID_T test_mesh_time_request(VOID_T)
{
    UINT8_T data[] = {0x02, 0x01, 0x07};
    tal_mesh_data_send(0, 0xD000, TAL_MESH_OPCODE_DATA, data, 3);
}

VOID_T tal_sdk_test_get_time_rsp(tal_utc_date_t *date)
{
    UINT8_T  rsp_data[10] = {0};
    UINT16_T rsp_len = 9;

    rsp_data[0] = date->year;
    rsp_data[1] = date->month;
    rsp_data[2] = date->day;
    rsp_data[3] = date->hour;
    rsp_data[4] = date->min;
    rsp_data[5] = date->sec;
    rsp_data[6] = date->dayIndex;
    rsp_data[7] = tal_utc_get_time_zone() >> 8;
    rsp_data[8] = tal_utc_get_time_zone();

    test_cmd_send(TEST_ID_GET(TEST_GID_SYSTEM, TEST_CID_GET_TIME), rsp_data, rsp_len);
}

VOID_T tal_sdk_test_mesh_data_write(UINT8_T dp_id, UINT8_T dp_type, UINT8_T *dp_data, UINT16_T dp_len)
{
    UINT8_T dp_data_t[64] = {0};

    dp_data_t[0] = dp_id;
    dp_data_t[1] = dp_type;
    dp_data_t[2] = dp_len >> 8;
    dp_data_t[3] = dp_len & 0xFF;

    memcpy(&dp_data_t[4], dp_data, dp_len);

    test_cmd_send(TEST_ID_GET(TEST_GID_DATA, TEST_CID_MESH_DP_WRITE), dp_data_t, dp_len+4);
}

OPERATE_RET test_group_system(UINT8_T cmd, UINT8_T *cmd_data, UINT32_T cmd_data_len, UINT8_T *p_rsp_data)
{
    TEST_GROUP_VARIABLE

    switch (cmd) {
        case TEST_CID_SHAKE_HAND: {
        } break;

        case TEST_CID_RESET: {
            if (NULL == reboot_timer_id) {
                tal_sw_timer_create(test_reboot_timer_callback, NULL, &reboot_timer_id);
            }
            tal_sw_timer_start(reboot_timer_id, 500, TAL_TIMER_ONCE);
            ret = OPRT_OK;
            TEST_RSP
        } break;

        case TEST_CID_FACTORY_RESET: {
            if (cmd_data[0] == 0) {
                ret = tal_mesh_network_state_set(MESH_NETWORK_RESET_WITH_RECOVER);
            } else if (cmd_data[0] == 1) {
                ret = tal_mesh_network_state_set(MESH_NETWORK_RESET);
            } else if (cmd_data[0] == 2) {
                ret = tal_mesh_network_state_set(MESH_NETWORK_RECOVER);
            }
            TEST_RSP
        } break;

        case TEST_CID_GET_TIME: {
            TIME_T time_sec = 0;
            tal_rtc_time_get(&time_sec);

            tal_utc_date_t date = {0};
            tal_utc_timestamp2date(time_sec, &date, false);

            INT16_T time_zone = tal_utc_get_time_zone();

            rsp[0] = date.year-2000;
            rsp[1] = date.month;
            rsp[2] = date.day;
            rsp[3] = date.hour;
            rsp[4] = date.min;
            rsp[5] = date.sec;
            rsp[6] = date.dayIndex;
            rsp[7] = time_zone >> 8;
            rsp[8] = time_zone;
            idx += 9;
        } break;

        case TEST_CID_REQ_TIME: {
            if (tal_get_if_prov_success()) {
                test_mesh_time_request();
                ret = OPRT_OK;
            } else {
                ret = OPRT_NETWORK_ERROR;
            }
            TEST_RSP
        } break;



        default: {
        } break;
    }

    return idx;
}

OPERATE_RET test_group_device_info(UINT8_T cmd, UINT8_T *cmd_data, UINT32_T cmd_data_len, UINT8_T *p_rsp_data)
{
    TEST_GROUP_VARIABLE

    switch (cmd) {
        case TEST_CID_SET_MAC: {
            ret = OPRT_NOT_SUPPORTED;
            TEST_RSP
        } break;

        case TEST_CID_GET_MAC: {
            TAL_BLE_ADDR_T addr;
            tal_ble_address_get(&addr);

            memcpy(&rsp[idx], addr.addr, 6);
            idx += 6;
        } break;

        case TEST_CID_SET_PID: {
            ret = OPRT_NOT_SUPPORTED;
            TEST_RSP
        } break;

        case TEST_CID_GET_PID: {
            UINT8_T data[8] = {0};
            if (tal_get_firmware_key_or_pid(data)) {
                rsp[idx] = 1;
            } else {
                rsp[idx] = 0;
            }
            idx += 1;

            rsp[idx] = 8;
            idx += 1;

            memcpy(&rsp[idx], data, 8);
            idx += 8;
        } break;

        case TEST_CID_SET_VERSION: {
            ret = OPRT_NOT_SUPPORTED;
            TEST_RSP
        } break;

        case TEST_CID_GET_VERSION: {
            rsp[0] = 0x00;
            rsp[1] = 0x00;
            rsp[2] = tal_util_str_hexchar2int(tal_get_firmware_version() & 0xFF);
            rsp[3] = tal_util_str_hexchar2int(tal_get_firmware_version() >> 8);
            rsp[4] = 0x00;
            rsp[5] = 0x00;
            rsp[6] = 0x00;
            rsp[7] = 0x00;
            idx += 8;
        } break;

        default: {
        } break;
    }

    return idx;
}

OPERATE_RET test_group_adv(UINT8_T cmd, UINT8_T *cmd_data, UINT32_T cmd_data_len, UINT8_T *p_rsp_data)
{
    TEST_GROUP_VARIABLE

    switch (cmd) {
        case TEST_CID_ADV_ENABLE: {
            if (0 == cmd_data[0]) {
                tal_mesh_node_provision_enable(MESH_PROVISION_DISABLE);
            } else {
                tal_mesh_node_provision_enable(MESH_PB_GATT_AND_PB_ADV);
            }
            ret = OPRT_OK;
            TEST_RSP
        }
        break;
        case TEST_CID_MESH_FAST_PROV: {
            if (0 == cmd_data[0]) {
                tal_mesh_fast_prov_enable(0);
            } else {
                tal_mesh_fast_prov_enable(1);
            }
            ret = OPRT_OK;
            TEST_RSP
        }
        break;
        default: {
            ret = OPRT_NOT_SUPPORTED;
            TEST_RSP
        } break;
    }

    return idx;
}

OPERATE_RET test_group_scan(UINT8_T cmd, UINT8_T *cmd_data, UINT32_T cmd_data_len, UINT8_T *p_rsp_data)
{
    TEST_GROUP_VARIABLE

    switch (cmd) {
        default: {
            ret = OPRT_NOT_SUPPORTED;
            TEST_RSP
        } break;
    }

    return idx;
}

OPERATE_RET test_group_conn(UINT8_T cmd, UINT8_T *cmd_data, UINT32_T cmd_data_len, UINT8_T *p_rsp_data)
{
    TEST_GROUP_VARIABLE

    switch (cmd) {
        case TEST_CID_CONN: {
        } break;

        case TEST_CID_SET_CONN_INTERVAL: {
            UINT16_T conn_interval_min = (cmd_data[0]<<8) + cmd_data[1];
            UINT16_T conn_interval_max = (cmd_data[2]<<8) + cmd_data[3];

            if (conn_interval_max > 500 && conn_interval_min < 20) {
                ret = OPRT_NOT_SUPPORTED;
            } else {
                TAL_BLE_PEER_INFO_T peer_info = {0};
                peer_info.conn_handle = 0x00;
                TAL_BLE_CONN_PARAMS_T conn_param = {0};
                conn_param.min_conn_interval = conn_interval_min*4/5;
                conn_param.max_conn_interval = conn_interval_max*4/5;
                conn_param.latency = 0;
                conn_param.conn_sup_timeout = 6000/10;
                conn_param.connection_timeout = 0;
                ret = tal_ble_conn_param_update(peer_info, &conn_param);
            }
            TEST_RSP
        } break;

        case TEST_CID_SET_CONN_PARAM: {
            UINT16_T conn_interval_min = (cmd_data[0]<<8) + cmd_data[1];
            UINT16_T conn_interval_max = (cmd_data[2]<<8) + cmd_data[3];

            if (conn_interval_max > 500 && conn_interval_min < 20) {
                ret = OPRT_NOT_SUPPORTED;
            } else {
                TAL_BLE_PEER_INFO_T peer_info = {0};
                peer_info.conn_handle = 0x00;
                TAL_BLE_CONN_PARAMS_T conn_param = {0};
                conn_param.min_conn_interval = conn_interval_min*4/5;
                conn_param.max_conn_interval = conn_interval_max*4/5;
                conn_param.latency = 0;
                conn_param.conn_sup_timeout = 6000/10;
                conn_param.connection_timeout = 0;
                ret = tal_ble_conn_param_update(peer_info, &conn_param);
            }
            TEST_RSP
        } break;

        case TEST_CID_DISCONN: {
            TAL_BLE_PEER_INFO_T info = {0};
            info.conn_handle = 0x00;
            ret = tal_ble_disconnect(info);
            TEST_RSP
        } break;

        case TEST_CID_NET_STATE: {
            if (cmd_data[0] == 1) { //mesh
                extern UINT8_T app_ble_connect_state_get(VOID);
                if (tal_get_if_prov_success()) {
                    if (app_ble_connect_state_get()) {
                        rsp[idx] = 0x03;
                    } else {
                        rsp[idx] = 0x01;
                    }
                } else {
                    if (app_ble_connect_state_get()) {
                        rsp[idx] = 0x02;
                    }
                    else {
                        rsp[idx] = 0x00;
                    }
                }
            } else {
                rsp[idx] = 0xFF;
            }
            idx++;
        } break;

        case TEST_CID_GET_MESH_ADDR: {
            if (0 == cmd_data[0]) {
                rsp[idx] = tal_primary_ele_addr_get() >> 8;
                idx++;
                rsp[idx] = tal_primary_ele_addr_get() & 0xFF;
                idx++;
            } else if (1 == cmd_data[0]) {
                UINT16_T *sub_list = tal_group_addr_sub_list_get(0, TAL_MODEL_ID_VENDOR_SERVER);
                for (int i = 0; i < 32; i++) {
                    if (MESH_IS_GROUP_ADDR(sub_list[i]) && 0xFFFF != sub_list[i]) {
                        rsp[idx] = sub_list[i] >> 8;
                        idx++;
                        rsp[idx] = sub_list[i] & 0xFF;
                        idx++;
                    }
                }
            }
        }
        break;

        default: {
        } break;
    }

    return idx;
}

OPERATE_RET test_group_data(UINT8_T cmd, UINT8_T *cmd_data, UINT32_T cmd_data_len, UINT8_T *p_rsp_data)
{
    TEST_GROUP_VARIABLE

    switch (cmd) {
        case TEST_CID_MESH_DP_REPORT: {
            switch (cmd_data[0]) {
                case 1: {
                    TAL_MESH_GENERIC_ONOFF_STATUS_T onoff_status = {0};
                    onoff_status.present = cmd_data[4];
                    tal_mesh_data_send(0, 0xD000, TAL_MESH_OPCODE_ON_OFF_STAT, (UINT8_T*)&onoff_status, 1);
                }
                break;
                case 2: {
                    UINT8_T light_mode = cmd_data[4];
                    tal_mesh_data_send(0, 0xD000, TAL_MESH_OPCODE_DATA, &light_mode, 1);
                }
                break;
                case 3: {
                    TAL_MESH_LIGHT_LIGHTNESS_STATUS_T lightness_status = {0};
                    UINT32_T lightness_t = 0;
                    lightness_t += cmd_data[4] << 24;
                    lightness_t += cmd_data[5] << 16;
                    lightness_t += cmd_data[6] << 8;
                    lightness_t += cmd_data[7] & 0xFF;
                    lightness_status.present = ((float)lightness_t / 1000) * 65535;
                    tal_mesh_data_send(0, 0xD000, TAL_MESH_OPCODE_LIGHTNESS_STAT, (UINT8_T*)&lightness_status, sizeof(USHORT_T));
                }
                break;
                case 4: {
                    TAL_MESH_LIGHT_CTL_TEMP_STATUS_T temp_status = {0};
                    UINT32_T tempture_t = 0;
                    tempture_t += cmd_data[4] << 24;
                    tempture_t += cmd_data[5] << 16;
                    tempture_t += cmd_data[6] << 8;
                    tempture_t += cmd_data[7] & 0xFF;
                    temp_status.present_temp = ((float)tempture_t / 1000) * 19200 + 800;
                    tal_mesh_data_send(0, 0xD000, TAL_MESH_OPCODE_LIGHT_CTL_TEMP_STAT, (UINT8_T*)&temp_status, sizeof(USHORT_T)*2);
                }
                break;
                case 5: {
                    TAL_MESH_LIGHT_HSL_STATUS_T hsl_status = {0};
                    UINT16_T hue = 0;
                    UINT16_T sat = 0;
                    UINT16_T lightness = 0;
                    hue += cmd_data[4] << 8;
                    hue += cmd_data[5] & 0xFF;
                    sat += cmd_data[6] << 8;
                    sat += cmd_data[7] & 0xFF;
                    lightness += cmd_data[8] << 8;
                    lightness += cmd_data[9] & 0xFF;
                    hsl_status.lightness = ((float)lightness / 1000) * 65535;
                    hsl_status.hue = ((float)hue / 360) * 65535;
                    hsl_status.sat = ((float)sat / 1000) * 65535;
                    tal_mesh_data_send(0, 0xD000, TAL_MESH_OPCODE_LIGHT_HSL_STAT, (UINT8_T*)&hsl_status, sizeof(USHORT_T)*3);
                }
                break;
                case 6: {
                    UINT16_T data_len = 0;
                    data_len += cmd_data[2] << 8;
                    data_len += cmd_data[3] & 0xFF;

                    UINT8_T data[50] = {0};
                    data[0] = 0x01;
                    data[1] = 0x06;
                    data[2] = 0x00;
                    data[3] = data_len & 0xFF; //len
                    memcpy(&data[4], &cmd_data[4], data_len);

                    tal_mesh_data_send(0, 0xD000, TAL_MESH_OPCODE_DATA, data, data_len+4);
                }
                break;
            }
            ret = OPRT_OK;
            TEST_RSP
        }
        break;
        case TEST_CID_MESH_DP_REPORT_RELIABLE: {
            UINT8_T trans_mode = cmd_data[0];
            switch (cmd_data[1]) {
                case 1: {
                    TAL_MESH_GENERIC_ONOFF_STATUS_T onoff_status = {0};
                    onoff_status.present = cmd_data[5];
                    tal_mesh_vendor_dp_report_reliable(1, DP_TYPE_BOOL, (UINT8_T*)&onoff_status, 1, trans_mode);
                }
                break;
                case 2: {
                    UINT8_T light_mode = cmd_data[5];
                    tal_mesh_vendor_dp_report_reliable(2, DP_TYPE_ENUM, (UINT8_T*)&light_mode, 1, trans_mode);
                }
                break;
                case 3: {
                    tal_mesh_vendor_dp_report_reliable(3, DP_TYPE_VALUE, (UINT8_T*)&cmd_data[5], 4, trans_mode);
                }
                break;
                case 4: {
                    tal_mesh_vendor_dp_report_reliable(4, DP_TYPE_VALUE, (UINT8_T*)&cmd_data[5], 4, trans_mode);
                }
                break;
//                case 5: {
//                    UINT16_T hue = 0;
//                    UINT16_T sat = 0;
//                    UINT16_T lightness = 0;
//                    hue += cmd_data[5] << 8;
//                    hue += cmd_data[6] & 0xFF;
//                    sat += cmd_data[7] << 8;
//                    sat += cmd_data[8] & 0xFF;
//                    lightness += cmd_data[9] << 8;
//                    lightness += cmd_data[10] & 0xFF;
//                    UINT16_T hsv_h = 0, hsv_s = 1000, hsv_v = 1000;
//                    tuya_light_tools_hsl2hsv(hue, sat, lightness, &hsv_h, &hsv_s, &hsv_v);
//                    UINT8_T dp_data[12] = {0};
//                    sprintf((char*)dp_data, "%04x", hsv_h);
//                    sprintf((char*)dp_data+4, "%04x", hsv_s);
//                    sprintf((char*)dp_data+8, "%04x", hsv_v);
//                    tal_mesh_vendor_dp_report_reliable(5, DP_TYPE_STRING, dp_data, 12, trans_mode);
//                }
//                break;
                case 6: {
                    UINT16_T data_len = 0;
                    data_len += cmd_data[3] << 8;
                    data_len += cmd_data[4] & 0xFF;

                    UINT8_T data[50] = {0};
                    data[0] = 0x01;
                    data[1] = 0x06;
                    data[2] = 0x00;
                    data[3] = data_len & 0xFF; //len
                    memcpy(&data[4], &cmd_data[5], data_len);

                    tal_mesh_data_send(0, 0xD000, TAL_MESH_OPCODE_DATA, data, data_len+4);
                }
                break;
            }
            ret = OPRT_OK;
            TEST_RSP
        }
        break;
        default: {
            ret = OPRT_NOT_SUPPORTED;
            TEST_RSP
        } break;
    }

    return idx;
}

OPERATE_RET test_group_else(UINT8_T cmd, UINT8_T *cmd_data, UINT32_T cmd_data_len, UINT8_T *p_rsp_data)
{
    TEST_GROUP_VARIABLE

    switch (cmd) {
        default: {
            ret = OPRT_NOT_SUPPORTED;
            TEST_RSP
        } break;
    }

    return idx;
}

TUYA_WEAK_ATTRIBUTE OPERATE_RET test_group_gpio(UINT8_T cmd, UINT8_T *cmd_data, UINT32_T cmd_data_len, UINT8_T *p_rsp_data)
{
    TEST_GROUP_VARIABLE

    TUYA_GPIO_NUM_E pin_id = (TUYA_GPIO_NUM_E)cmd_data[0];
    switch (cmd) {
        case TEST_CID_PIN_DEINIT: {
            ret = tal_gpio_deinit(pin_id);
            TEST_RSP
        } break;

        case TEST_CID_OUTPUT_HIGH: {
            TUYA_GPIO_BASE_CFG_T gpio_cfg = {
                .mode = TUYA_GPIO_PUSH_PULL,
                .direct = TUYA_GPIO_OUTPUT,
                .level = TUYA_GPIO_LEVEL_HIGH,
            };
            tal_gpio_init(pin_id, &gpio_cfg);

            ret = tal_gpio_write(pin_id, TUYA_GPIO_LEVEL_HIGH);
            TEST_RSP
        } break;

        case TEST_CID_OUTPUT_LOW: {
            TUYA_GPIO_BASE_CFG_T gpio_cfg = {
                .mode = TUYA_GPIO_PUSH_PULL,
                .direct = TUYA_GPIO_OUTPUT,
                .level = TUYA_GPIO_LEVEL_LOW,
            };
            tal_gpio_init(pin_id, &gpio_cfg);

            ret = tal_gpio_write(pin_id, TUYA_GPIO_LEVEL_LOW);
            TEST_RSP
        } break;

        case TEST_CID_PIN_READ: {
            TUYA_GPIO_BASE_CFG_T gpio_cfg = {
                .mode = TUYA_GPIO_PULLUP,
                .direct = TUYA_GPIO_INPUT,
                .level = TUYA_GPIO_LEVEL_LOW,
            };
            tal_gpio_init(pin_id, &gpio_cfg);

            UINT32_T gpio_level = TUYA_GPIO_LEVEL_LOW;
            ret = tal_gpio_read(pin_id, (TUYA_GPIO_LEVEL_E*)&gpio_level);
            rsp[idx++] = (gpio_level >> 24) & 0xFF;
            rsp[idx++] = (gpio_level >> 16) & 0xFF;
            rsp[idx++] = (gpio_level >> 8) & 0xFF;
            rsp[idx++] = (gpio_level) & 0xFF;
        } break;

        default: {
        } break;
    }

    return idx;
}

TUYA_WEAK_ATTRIBUTE OPERATE_RET test_group_uart(UINT8_T cmd, UINT8_T *cmd_data, UINT32_T cmd_data_len, UINT8_T *p_rsp_data)
{
    TEST_GROUP_VARIABLE

    switch (cmd) {
        case TEST_CID_SET_BAUDRATE: {
            TUYA_UART_NUM_E port_num = (TUYA_UART_NUM_E)((cmd_data[0]<<24) + (cmd_data[1]<<16) + (cmd_data[2]<<8) + cmd_data[3]);
            if (1 != port_num) {
                ret = OPRT_NOT_SUPPORTED;
            } else {
                TAL_UART_CFG_T tal_uart_cfg;
                tal_uart_cfg.base_cfg.baudrate = (cmd_data[4]<<24) + (cmd_data[5]<<16) + (cmd_data[6]<<8) + cmd_data[7];
                tal_uart_cfg.rx_buffer_size = 256,
                tal_uart_cfg.base_cfg.parity = TUYA_UART_PARITY_TYPE_NONE,
                tal_uart_cfg.base_cfg.databits = TUYA_UART_DATA_LEN_8BIT,
                tal_uart_cfg.base_cfg.stopbits = TUYA_UART_STOP_LEN_1BIT,
                tal_uart_cfg.base_cfg.flowctrl = TUYA_UART_FLOWCTRL_NONE,

                tal_uart_deinit(port_num);
                ret = tal_uart_init(port_num, &tal_uart_cfg);
            }
            TEST_RSP
        } break;

        case TEST_CID_TX_UART_DATA: {
            TUYA_UART_NUM_E port_num = (TUYA_UART_NUM_E)((cmd_data[0]<<24) + (cmd_data[1]<<16) + (cmd_data[2]<<8) + cmd_data[3]);

            if (1 != port_num) {
                ret = OPRT_NOT_SUPPORTED;
            } else {
                ret = tal_uart_write(port_num, cmd_data + 4, cmd_data_len - 4);
            }
            TEST_RSP
        } break;

        default: {
        } break;
    }

    return idx;
}

TUYA_WEAK_ATTRIBUTE OPERATE_RET test_group_pwm(UINT8_T cmd, UINT8_T *cmd_data, UINT32_T cmd_data_len, UINT8_T *p_rsp_data)
{
    TEST_GROUP_VARIABLE

    switch (cmd) {
        case TEST_CID_PWM_DEINIT: {
            TUYA_PWM_NUM_E channel = (TUYA_PWM_NUM_E)((cmd_data[0] << 24) | (cmd_data[1] << 16) | (cmd_data[2] << 8) | cmd_data[3]);
            ret = tal_pwm_stop(channel);
            TEST_RSP
        } break;

        case TEST_CID_SET_FREQ_DUTY: {
            TUYA_PWM_NUM_E channel = (TUYA_PWM_NUM_E)((cmd_data[0] << 24) | (cmd_data[1] << 16) | (cmd_data[2] << 8) | cmd_data[3]);
            UINT32_T frequency = (cmd_data[4] << 24) | (cmd_data[5] << 16) | (cmd_data[6] << 8) | cmd_data[7];
            UINT32_T duty = (cmd_data[8] << 24) | (cmd_data[9] << 16) | (cmd_data[10] << 8) | cmd_data[11];
            UINT32_T cycle = (cmd_data[12] << 24) | (cmd_data[13] << 16) | (cmd_data[14] << 8) | cmd_data[15];
            TUYA_PWM_BASE_CFG_T pwm_cfg;
            pwm_cfg.polarity = TUYA_PWM_POSITIVE;
            pwm_cfg.duty = duty;
            pwm_cfg.frequency = frequency;
            pwm_cfg.cycle = cycle;
            tal_pwm_init(channel, &pwm_cfg);
            tal_pwm_start(channel);
            ret = OPRT_OK;
            TEST_RSP
        } break;

        default: {
        } break;
    }

    return idx;
}

TUYA_WEAK_ATTRIBUTE OPERATE_RET test_group_adc(UINT8_T cmd, UINT8_T *cmd_data, UINT32_T cmd_data_len, UINT8_T *p_rsp_data)
{
    TEST_GROUP_VARIABLE

    switch (cmd) {
        case TEST_CID_ADC_DEINIT: {
            ret = tal_adc_deinit(TUYA_ADC_NUM_0);
            TEST_RSP
        } break;

        case TEST_CID_READ_ADC_DATA: {
            UINT8_T channel = (TUYA_ADC_NUM_E)cmd_data[3];
            UINT8_T  width = cmd_data[4];
            INT32_T adc_value = 0;
            tal_adc_deinit(TUYA_ADC_NUM_0);
            TUYA_ADC_BASE_CFG_T adc_cfg = {
                .ch_nums = 1,
                .type = TUYA_ADC_EXTERNAL_SAMPLE_VOL,
                .width = width,
            };
            adc_cfg.ch_list.data |= BIT(channel);

            ret = tal_adc_init(TUYA_ADC_NUM_0, &adc_cfg);
            if (ret == OPRT_OK) {
                ret = tal_adc_read_single_channel(TUYA_ADC_NUM_0, channel, &adc_value);
                if (ret == OPRT_OK) {
                    test_cmd_send(TEST_ID_GET(TEST_GID_ADC, TEST_CID_READ_ADC_DATA_RSP), (UINT8_T*)&adc_value, SIZEOF(UINT32_T));
                }
            }

            TEST_RSP
        } break;

        default: {
        } break;
    }

    return idx;
}

TUYA_WEAK_ATTRIBUTE OPERATE_RET test_group_spi(UINT8_T cmd, UINT8_T *cmd_data, UINT32_T cmd_data_len, UINT8_T *p_rsp_data)
{
    TEST_GROUP_VARIABLE

    switch (cmd) {
        case TEST_CID_TX_SPI_DATA: {
            UINT32_T channel        = (cmd_data[0] << 24) | (cmd_data[1] << 16) | (cmd_data[2] << 8) | cmd_data[3];
            UINT32_T frequency      = (cmd_data[4] << 24) | (cmd_data[5] << 16) | (cmd_data[6] << 8) | cmd_data[7];
            UINT8_T  *spi_data      = cmd_data + 8;
            UINT32_T spi_data_len   = cmd_data_len - 8;

            tal_spi_deinit(channel);

            TUYA_SPI_BASE_CFG_T spi_cfg = {
                .mode = TUYA_SPI_MODE0,
                .type = TUYA_SPI_SOFT_TYPE,
                .databits = TUYA_SPI_DATA_BIT8,
                .freq_hz = frequency
            };
            ret = tal_spi_init(channel, &spi_cfg);
            if (ret == OPRT_OK) {
                UINT8_T *buf = tal_malloc(spi_data_len);
                if (buf) {
                    ret = tal_spi_xfer(channel, spi_data, buf, spi_data_len);
                    if (ret == OPRT_OK) {
                        if (memcmp(spi_data, buf, spi_data_len) == 0) {
                            test_cmd_send(TEST_ID_GET(TEST_GID_SPI, TEST_CID_RX_SPI_DATA), buf, spi_data_len);
                        } else {
                            ret = OPRT_RECV_DA_NOT_ENOUGH;
                        }
                    }

                    tal_free(buf);
                } else {
                    ret = OPRT_MALLOC_FAILED;
                }
            }

            TEST_RSP
        } break;

        case TEST_CID_RX_SPI_DATA: {
        } break;

        default: {
        } break;
    }

    switch (cmd) {
        default: {
            ret = OPRT_NOT_SUPPORTED;
            TEST_RSP
        } break;
    }

    return idx;
}

TUYA_WEAK_ATTRIBUTE OPERATE_RET test_group_iic(UINT8_T cmd, UINT8_T *cmd_data, UINT32_T cmd_data_len, UINT8_T *p_rsp_data)
{
    TEST_GROUP_VARIABLE

    switch (cmd) {
        case TEST_CID_TX_IIC_DATA: {
            UINT32_T channel        = (cmd_data[0] << 24) | (cmd_data[1] << 16) | (cmd_data[2] << 8) | cmd_data[3];
            UINT8_T  *iic_data      = cmd_data + 4;
            UINT32_T iic_data_len   = cmd_data_len - 4;

            if (iic_data_len < 14) {
                if (tal_oled_check_i2c_port_num() != channel) {
                    ret = OPRT_NOT_SUPPORTED;
                } else {
                    UINT8_T buf[14] = {0};
                    memcpy(buf, iic_data, iic_data_len);
                    buf[iic_data_len] = '\0';

                    tal_oled_clear();
                    ret = tal_oled_show_string(12, 1, (void*)buf, 16);
                }
            } else {
                ret = OPRT_INVALID_PARM;
            }
            TEST_RSP
        } break;

        case TEST_CID_RX_IIC_DATA: {
        } break;

        default: {
        } break;
    }

    return idx;
}

TUYA_WEAK_ATTRIBUTE OPERATE_RET test_group_rtc(UINT8_T cmd, UINT8_T *cmd_data, UINT32_T cmd_data_len, UINT8_T *p_rsp_data)
{
    TEST_GROUP_VARIABLE

    switch (cmd) {
        case TEST_CID_SET_RTC_TIME: {
            UINT32_T time_sec = (cmd_data[0] << 24) | (cmd_data[1] << 16) | (cmd_data[2] << 8) | cmd_data[3];
            ret = tal_rtc_time_set(time_sec);
            TEST_RSP
        } break;

        case TEST_CID_GET_RTC_TIME: {
            TIME_T time_sec = 0;
            ret = tal_rtc_time_get(&time_sec);

            rsp[0] = time_sec >> 24;
            rsp[1] = time_sec >> 16;
            rsp[2] = time_sec >> 8;
            rsp[3] = time_sec & 0xFF;
            idx += 4;
        } break;

        case TEST_CID_START_RTC: {
            ret = tal_rtc_init();
            TEST_RSP
        } break;

        case TEST_CID_STOP_RTC: {
            ret = tal_rtc_deinit(); // RTC affects the soft timer, which affects uart rx
            TEST_RSP
        } break;

        default: {
        } break;
    }

    return idx;
}

TUYA_WEAK_ATTRIBUTE OPERATE_RET test_group_flash(UINT8_T cmd, UINT8_T *cmd_data, UINT32_T cmd_data_len, UINT8_T *p_rsp_data)
{
    TEST_GROUP_VARIABLE

    switch (cmd) {
        case TEST_CID_READ_FLASH_DATA: {
            UINT32_T addr = (cmd_data[0] << 24) | (cmd_data[1] << 16) | (cmd_data[2] << 8) | cmd_data[3];
            UINT32_T len = (cmd_data[4] << 24) | (cmd_data[5] << 16) | (cmd_data[6] << 8) | cmd_data[7];

            tal_flash_read(addr, &rsp[idx], len);
            idx += len;
        } break;

        case TEST_CID_ERASE_FLASH_DATA: {
            UINT32_T addr = (cmd_data[0] << 24) | (cmd_data[1] << 16) | (cmd_data[2] << 8) | cmd_data[3];
            UINT32_T len = (cmd_data[4] << 24) | (cmd_data[5] << 16) | (cmd_data[6] << 8) | cmd_data[7];

            ret = tal_flash_erase(addr, len);
            TEST_RSP
        } break;

        case TEST_CID_WRITE_FLASH_DATA: {
            UINT32_T addr = (cmd_data[0] << 24) | (cmd_data[1] << 16) | (cmd_data[2] << 8) | cmd_data[3];
            UINT8_T  *flash_data = cmd_data + 4;
            UINT32_T flash_data_len = cmd_data_len - 4;

            ret = tal_flash_write(addr, flash_data, flash_data_len);
            TEST_RSP
        } break;

        default: {
        } break;
    }

    return idx;
}

TUYA_WEAK_ATTRIBUTE OPERATE_RET test_group_watchdog(UINT8_T cmd, UINT8_T *cmd_data, UINT32_T cmd_data_len, UINT8_T *p_rsp_data)
{
    TEST_GROUP_VARIABLE

    switch (cmd) {
        case TEST_CID_START_WDG: {
            UINT32_T interval_ms = (cmd_data[0] << 24) | (cmd_data[1] << 16) | (cmd_data[2] << 8) | cmd_data[3];

            TUYA_WDOG_BASE_CFG_T wdog_cfg = {
                .interval_ms = interval_ms,
            };
            tal_watchdog_start(&wdog_cfg);
            ret = OPRT_OK;
            TEST_RSP
        } break;

        case TEST_CID_FEED_WDG: {
            ret = tal_watchdog_refresh();
            TEST_RSP
        } break;

        case TEST_CID_STOP_WDG: {
            ret = tal_watchdog_stop();
            TEST_RSP
        } break;

        default: {
        } break;
    }

    return idx;
}

TUYA_WEAK_ATTRIBUTE OPERATE_RET test_group_powermanger(UINT8_T cmd, UINT8_T *cmd_data, UINT32_T cmd_data_len, UINT8_T *p_rsp_data)
{
    TEST_GROUP_VARIABLE

    switch (cmd) {
        case TEST_CID_ENTER_SLEEP: {
            UINT8_T mode = cmd_data[0];
            if (0 == mode) {
                tkl_cpu_sleep_mode_set(1, TUYA_CPU_DEEP_SLEEP);
                ret = OPRT_OK;
            } else if (1 == mode) {
                tkl_cpu_sleep_mode_set(1, TUYA_CPU_SLEEP);
                ret = OPRT_OK;
            } else {
                ret = OPRT_NOT_SUPPORTED;
            }

            if (OPRT_OK == ret) {
                if (NULL == mesh_test_enter_sleep_timer_id) {
                    tal_sw_timer_create(test_enter_sleep_callback, NULL, &mesh_test_enter_sleep_timer_id);
                }
                tal_sw_timer_start(mesh_test_enter_sleep_timer_id, 200, TAL_TIMER_ONCE);
            }

            TEST_RSP
        } break;

        case TEST_CID_WAKEUP_SRC_SET: {
            UINT8_T wakeup_type = cmd_data[0];
            TUYA_WAKEUP_SOURCE_BASE_CFG_T param;
            if (0 == wakeup_type) {
                param.source = TUYA_WAKEUP_SOURCE_GPIO;
                param.wakeup_para.gpio_param.gpio_num = cmd_data[1];
                param.wakeup_para.gpio_param.level = cmd_data[2];
                ret = OPRT_OK;
            } else if (1 == wakeup_type) {
                param.source = TUYA_WAKEUP_SOURCE_TIMER;
                param.wakeup_para.timer_param.timer_num = cmd_data[1];
                param.wakeup_para.timer_param.mode = cmd_data[2];
                param.wakeup_para.timer_param.ms = (cmd_data[3] << 24) + (cmd_data[4] << 16) + (cmd_data[5] << 8) + cmd_data[6];
                ret = OPRT_OK;
            } else {
                ret = OPRT_NOT_SUPPORTED;
            }

            if (OPRT_OK == ret) {
                tkl_wakeup_source_set(&param);
            }

            TEST_RSP
        } break;

        default: {
        } break;
    }

    switch (cmd) {
        default: {
            ret = OPRT_NOT_SUPPORTED;
            TEST_RSP
        } break;
    }
    return idx;
}

TUYA_WEAK_ATTRIBUTE OPERATE_RET test_cmd_send(UINT16_T cmdId, UINT8_T* buf, UINT16_T size)
{
    UINT8_T *data_buf = (UINT8_T*)tal_malloc(size + 3);
    data_buf[0] = 0x03;
    data_buf[1] = cmdId >> 8;
    data_buf[2] = cmdId & 0xFF;
    if (size > 0) {
        memcpy(data_buf + 3, buf, size);
    }

    tal_mesh_factory_test_send_cmd(CMD_UART_CMD_SERVER_FOR_BLE_SDK_TEST_EXTEND, data_buf, size + 3);

    tal_free(data_buf);

    return OPRT_OK;
}

VOID_T tuya_mesh_app_sdk_test_process(UINT8_T *p_in_data, UINT16_T in_len)
{
    if (!((p_in_data[7] == 0x01) && (p_in_data[8] == 0x05))) {
//        TAL_PR_HEXDUMP_INFO("test_cmd", p_in_data + 7, in_len - 8);
    }

    mesh_test_cmd_t* cmd = (VOID_T*)p_in_data;

    if ((cmd->type != 3) || (in_len < 3)) {
        return;
    }

    UINT16_T cmd_data_len = in_len - 3;

    BOOL_T   rsp_flag = TRUE;
    UINT8_T  rsp_data[256] = {0};
    UINT32_T rsp_len = 0;

    switch (cmd->group_id) {
        case TEST_GID_SYSTEM: {
            rsp_len = test_group_system(cmd->cmd_id, cmd->value, cmd_data_len, rsp_data);
        } break;

        case TEST_GID_DEVICE_INFO: {
            rsp_len = test_group_device_info(cmd->cmd_id, cmd->value, cmd_data_len, rsp_data);
        } break;

        case TEST_GID_ADV: {
            rsp_len = test_group_adv(cmd->cmd_id, cmd->value, cmd_data_len, rsp_data);
        } break;

        case TEST_GID_SCAN: {
            rsp_len = test_group_scan(cmd->cmd_id, cmd->value, cmd_data_len, rsp_data);
        } break;

        case TEST_GID_CONN: {
            rsp_len = test_group_conn(cmd->cmd_id, cmd->value, cmd_data_len, rsp_data);
        } break;

        case TEST_GID_DATA: {
            rsp_len = test_group_data(cmd->cmd_id, cmd->value, cmd_data_len, rsp_data);
        } break;

        case TEST_GID_ELSE: {
            rsp_len = test_group_else(cmd->cmd_id, cmd->value, cmd_data_len, rsp_data);
        } break;

        case TEST_GID_GPIO: {
            rsp_len = test_group_gpio(cmd->cmd_id, cmd->value, cmd_data_len, rsp_data);
        } break;

        case TEST_GID_UART: {
            rsp_len = test_group_uart(cmd->cmd_id, cmd->value, cmd_data_len, rsp_data);
        } break;

        case TEST_GID_PWM: {
            rsp_len = test_group_pwm(cmd->cmd_id, cmd->value, cmd_data_len, rsp_data);
        } break;

        case TEST_GID_ADC: {
            rsp_len = test_group_adc(cmd->cmd_id, cmd->value, cmd_data_len, rsp_data);
        } break;

        case TEST_GID_SPI: {
            rsp_len = test_group_spi(cmd->cmd_id, cmd->value, cmd_data_len, rsp_data);
        } break;

        case TEST_GID_IIC: {
            rsp_len = test_group_iic(cmd->cmd_id, cmd->value, cmd_data_len, rsp_data);
        } break;

        case TEST_GID_RTC: {
            rsp_len = test_group_rtc(cmd->cmd_id, cmd->value, cmd_data_len, rsp_data);
        } break;

        case TEST_GID_FLASH: {
            rsp_len = test_group_flash(cmd->cmd_id, cmd->value, cmd_data_len, rsp_data);
        } break;

        case TEST_GID_WATCHDOG: {
            rsp_len = test_group_watchdog(cmd->cmd_id, cmd->value, cmd_data_len, rsp_data);
        } break;

        case TEST_GID_POWERMANGER: {
            rsp_len = test_group_powermanger(cmd->cmd_id, cmd->value, cmd_data_len, rsp_data);
        } break;


        default: {
        } break;
    }

    if (rsp_flag) {
        UINT16_T id = (cmd->group_id<<8) + cmd->cmd_id;
        test_cmd_send(id, rsp_data, rsp_len);
    }

}

#endif


