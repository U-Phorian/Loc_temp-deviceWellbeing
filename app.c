/***************************************************************************//**
 * @file app.c
 * @brief Smart Cold-Chain & Proximity Geofence BLE Node (BRD2709A / EFR32MG26)
 *        Works with Web Bluetooth Website + Simplicity Connect App
 ******************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"
#include "sl_simple_button_instances.h"
#include "sl_simple_led_instances.h"
#include "sl_sleeptimer.h"
#include "tempdrv.h"
#include "app_log.h"

// Operating Modes controlled bi-directionally from the Website
typedef enum {
  MODE_AUTO_THERMAL   = 0, // Auto-activates LED0 cooling when Temp >= Threshold
  MODE_MANUAL_CONTROL = 1, // Website manually toggles LED0 ON/OFF
  MODE_PROXIMITY_LOCK = 2, // Auto-locks (LED0 ON) when Phone walks > 2m away (RSSI < -65 dBm)
  MODE_LOCKDOWN_ALARM = 3  // Emergency Lockdown: Forces LED0 ON
} system_mode_t;

// BLE Handles
static uint8_t blinky_adv_handle  = 0xFF;
static uint8_t active_conn_handle = 0xFF;
static bool notifications_enabled = false;

// Periodic Telemetry Timer
static sl_sleeptimer_timer_handle_t telemetry_timer;
static volatile bool tick_telemetry = false;
static uint8_t frame_cycle = 0;

// Real-Time Hardware State
static system_mode_t current_mode = MODE_AUTO_THERMAL;
static int8_t current_temp_c      = 25;
static int8_t temp_threshold_c    = 30;
static int8_t current_rssi_dbm    = -55;
static bool cooling_led0_on       = false;
static bool alarm_active          = false;
static bool btn0_tamper_pressed   = false;

static const uint8_t blinky_advertising_data[] = {
  0x02, 0x01, 0x06,
  0x11, 0x07,
  0x24, 0x12, 0xB5, 0xCB, 0xD4, 0x60, 0x80, 0x0C,
  0x15, 0xC3, 0x9B, 0xA9, 0xAC, 0x5A, 0x8A, 0xDE,
  0x06, 0x09, 'E', 'F', 'R', '3', '2'
};

// Forward declarations
static void evaluate_hardware_logic(void);
static void send_tagged_byte_to_web(uint8_t payload);
static void send_status_frame_now(void);

// Timer Callback (triggers every 250 ms -> full 4-frame cycle every 1.0 s)
static void on_telemetry_timer(sl_sleeptimer_timer_handle_t *handle, void *data)
{
  (void)handle;
  (void)data;
  tick_telemetry = true;
}

// Helper to drive LED0 (and LED1 if installed)
static void update_leds(void)
{
  if (cooling_led0_on || alarm_active) {
    sl_led_turn_on(&sl_led_led0);
  } else {
    sl_led_turn_off(&sl_led_led0);
  }
}

// Evaluate Operating Modes & Alarms
static void evaluate_hardware_logic(void)
{
  bool over_temp = (current_temp_c >= temp_threshold_c);

  switch (current_mode) {
    case MODE_AUTO_THERMAL:
      cooling_led0_on = over_temp;
      alarm_active    = over_temp || btn0_tamper_pressed;
      break;

    case MODE_MANUAL_CONTROL:
      // cooling_led0_on is controlled directly by user from Website
      alarm_active = over_temp || btn0_tamper_pressed;
      break;

    case MODE_PROXIMITY_LOCK:
      // Auto-Lock (LED0 ON) if phone walks > ~2.0m away (RSSI < -65 dBm)
      cooling_led0_on = (current_rssi_dbm < -65);
      alarm_active    = cooling_led0_on || over_temp || btn0_tamper_pressed;
      break;

    case MODE_LOCKDOWN_ALARM:
      cooling_led0_on = true;
      alarm_active    = true;
      break;
  }

  update_leds();

  // Keep GATT LED Characteristic synced
  uint8_t led_state_byte = cooling_led0_on ? 0x01 : 0x00;
  sl_bt_gatt_server_write_attribute_value(gattdb_led_control,
                                          0,
                                          sizeof(led_state_byte),
                                          &led_state_byte);
}

// Send 1-byte Tagged Frame over gattdb_report_button (Zero GATT Config changes needed!)
static void send_tagged_byte_to_web(uint8_t payload)
{
  sl_bt_gatt_server_write_attribute_value(gattdb_report_button,
                                          0,
                                          sizeof(payload),
                                          &payload);

  if (active_conn_handle != 0xFF && notifications_enabled) {
    sl_bt_gatt_server_send_notification(active_conn_handle,
                                        gattdb_report_button,
                                        sizeof(payload),
                                        &payload);
  }
}

// Frame Type 00 (0x00..0x3F): [00 | Mode(2b) | Alarm(1b) | LED0(1b) | BTN0(1b)]
static void send_status_frame_now(void)
{
  uint8_t status_byte = 0x00; // Top 2 bits = 00
  status_byte |= (btn0_tamper_pressed ? (1 << 0) : 0);
  status_byte |= (cooling_led0_on     ? (1 << 1) : 0);
  status_byte |= (alarm_active        ? (1 << 2) : 0);
  status_byte |= (((uint8_t)current_mode & 0x03) << 3);

  send_tagged_byte_to_web(status_byte);
}

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
void app_init(void)
{
  TEMPDRV_Init();
  current_temp_c   = TEMPDRV_GetTemp();
  temp_threshold_c = current_temp_c + 2; // Auto-set threshold 2C above ambient room temp

  evaluate_hardware_logic();

  sl_sleeptimer_start_periodic_timer_ms(&telemetry_timer,
                                        250,
                                        on_telemetry_timer,
                                        NULL,
                                        0,
                                        0);

  app_log_info("=== Smart Cold-Chain & Geofence Node Ready (BRD2709A) ===\r\n");
  app_log_info("Ambient Temp: %d C | Initial Threshold: %d C\r\n",
               current_temp_c, temp_threshold_c);
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
void app_process_action(void)
{
  if (!tick_telemetry) {
    return;
  }
  tick_telemetry = false;

  // 1. Read Real Internal Silicon Temperature
  current_temp_c = TEMPDRV_GetTemp();

  // 2. Read Real Connection RSSI (Phone-to-Board Signal Strength)
  if (active_conn_handle != 0xFF) {
    int8_t measured_rssi = -55;
    if (sl_bt_connection_get_median_rssi(active_conn_handle, &measured_rssi) == SL_STATUS_OK) {
      if (measured_rssi < -20 && measured_rssi > -105) {
        current_rssi_dbm = measured_rssi;
      }
    }
  }

  // 3. Evaluate Modes, Thresholds, and Hardware Outputs
  evaluate_hardware_logic();

  // 4. Stream 4-Frame Tagged Telemetry over BLE Notifications (1 frame every 250ms)
  frame_cycle = (frame_cycle + 1) & 0x03;

  if (frame_cycle == 0) {
    // Frame 00: System Status (BTN0, LED0, Alarm, Active Mode)
    send_status_frame_now();
  } else if (frame_cycle == 1) {
    // Frame 01 (0x40 | temp): Live Silicon Temperature (0..63 C)
    uint8_t clamped_temp = (current_temp_c < 0) ? 0 : ((current_temp_c > 63) ? 63 : (uint8_t)current_temp_c);
    send_tagged_byte_to_web(0x40 | (clamped_temp & 0x3F));
  } else if (frame_cycle == 2) {
    // Frame 10 (0x80 | rssi_offset): Live RSSI (-30 dBm to -93 dBm)
    int32_t abs_rssi = (current_rssi_dbm < 0) ? -current_rssi_dbm : 55;
    int32_t offset   = abs_rssi - 30;
    if (offset < 0)  offset = 0;
    if (offset > 63) offset = 63;
    send_tagged_byte_to_web(0x80 | ((uint8_t)offset & 0x3F));
  } else {
    // Frame 11 (0xC0 | threshold): Active Alarm Threshold (0..63 C)
    uint8_t clamped_thr = (temp_threshold_c < 0) ? 0 : ((temp_threshold_c > 63) ? 63 : (uint8_t)temp_threshold_c);
    send_tagged_byte_to_web(0xC0 | (clamped_thr & 0x3F));

    app_log_info("[LIVE] Mode:%d | Temp:%dC (Lim:%dC) | RSSI:%ddBm | LED0:%d | Alarm:%d\r\n",
                 (int)current_mode, current_temp_c, temp_threshold_c,
                 current_rssi_dbm, cooling_led0_on, alarm_active);
  }
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;

  switch (SL_BT_MSG_ID(evt->header)) {
    case sl_bt_evt_system_boot_id:
      // 1. Start Connectable GATT Advertiser (Blinky Example)
      sc = sl_bt_advertiser_create_set(&blinky_adv_handle);
      app_assert_status(sc);
      sc = sl_bt_legacy_advertiser_set_data(blinky_adv_handle,
                                            sl_bt_advertiser_advertising_data_packet,
                                            sizeof(blinky_advertising_data),
                                            blinky_advertising_data);
      app_assert_status(sc);
      sc = sl_bt_advertiser_set_timing(blinky_adv_handle, 160, 160, 0, 0);
      app_assert_status(sc);
      sc = sl_bt_legacy_advertiser_start(blinky_adv_handle,
                                         sl_bt_legacy_advertiser_connectable);
      app_assert_status(sc);
      break;

    case sl_bt_evt_connection_opened_id:
      active_conn_handle    = evt->data.evt_connection_opened.connection;
      notifications_enabled = false;
      app_log_info("BLE Client Connected (Handle: %d)\r\n", active_conn_handle);
      break;

    case sl_bt_evt_connection_closed_id:
      active_conn_handle    = 0xFF;
      notifications_enabled = false;
      app_log_info("BLE Client Disconnected - Restarting GATT Advertiser\r\n");
      sl_bt_legacy_advertiser_start(blinky_adv_handle,
                                    sl_bt_legacy_advertiser_connectable);
      break;

    // Bi-Directional Command Received from Website (or Simplicity Connect App)
    case sl_bt_evt_gatt_server_attribute_value_id:
      if (evt->data.evt_gatt_server_attribute_value.attribute == gattdb_led_control) {
        uint8_t cmd = evt->data.evt_gatt_server_attribute_value.value.data[0];

        if (cmd == 0x00) {
          // Turn Cooling LED0 OFF (switches to Manual Mode so user sees immediate effect)
          current_mode    = MODE_MANUAL_CONTROL;
          cooling_led0_on = false;
        } else if (cmd == 0x01) {
          // Turn Cooling LED0 ON
          current_mode    = MODE_MANUAL_CONTROL;
          cooling_led0_on = true;
        } else if (cmd == 0x10) {
          current_mode = MODE_AUTO_THERMAL;
        } else if (cmd == 0x11) {
          current_mode = MODE_MANUAL_CONTROL;
        } else if (cmd == 0x12) {
          current_mode = MODE_PROXIMITY_LOCK;
        } else if (cmd == 0x13) {
          current_mode = MODE_LOCKDOWN_ALARM;
        } else if (cmd == 0x20) {
          // Remote Auto-Calibrate Threshold (Ambient + 2C)
          temp_threshold_c = TEMPDRV_GetTemp() + 2;
        } else if ((cmd & 0xC0) == 0x40) {
          // Set Custom Temperature Threshold (0x40 | target_temp_c)
          temp_threshold_c = (int8_t)(cmd & 0x3F);
        }

        evaluate_hardware_logic();
        send_status_frame_now();
        app_log_info("[WEB CMD] Received 0x%02X -> Mode:%d, LED0:%d, Limit:%dC\r\n",
                     cmd, (int)current_mode, cooling_led0_on, temp_threshold_c);
      }
      break;

    case sl_bt_evt_gatt_server_characteristic_status_id:
      if (evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_report_button) {
        if (evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_client_config) {
          notifications_enabled = (evt->data.evt_gatt_server_characteristic_status.client_config_flags != sl_bt_gatt_disable);
          if (notifications_enabled) {
            send_status_frame_now();
          }
        }
      }
      break;

    default:
      break;
  }
}

/**************************************************************************//**
 * Hardware Push-Button Callback (BTN0 = Tamper Switch, BTN1 = Calibrate)
 *****************************************************************************/
void sl_button_on_change(const sl_button_t *handle)
{
  if (handle == &sl_button_btn0) {
    btn0_tamper_pressed = (sl_button_get_state(handle) == SL_SIMPLE_BUTTON_PRESSED);
    evaluate_hardware_logic();
    send_status_frame_now();
  }
}
