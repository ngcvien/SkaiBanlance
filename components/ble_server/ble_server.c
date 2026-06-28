#include "ble_server.h"
#include "robot_cmds.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "pid_controller.h"

extern pid_context_t balance_pid;

// --- KHAI BÁO 128-BIT UUID (Giao thức Nordic UART) ---
// NimBLE yêu cầu byte mảng UUID phải đảo ngược (Little-Endian)
// Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
static const ble_uuid128_t gatt_svr_svc_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E);

// RX (Ghi lệnh): 6E400002-B5A3-F393-E0A9-E50E24DCCA9E
static const ble_uuid128_t gatt_svr_chr_rx_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E);

// TX (Gửi Telemetry): 6E400003-B5A3-F393-E0A9-E50E24DCCA9E
static const ble_uuid128_t gatt_svr_chr_tx_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E);

// --- BIẾN TOÀN CỤC ---
static uint8_t own_addr_type;
static uint16_t ble_conn_handle = BLE_HS_CONN_HANDLE_NONE; // Lưu ID kết nối với điện thoại
uint16_t tx_handle; // Lưu ID của cổng TX để gửi Notify
static volatile bool telemetry_notify_enabled = false;

static void ble_app_advertise(void);

// --- HÀM NHẬN LỆNH TỪ APP ---
static int gatt_svr_chr_write(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    char rx_str[64];
    int len = ctxt->om->om_len > 63 ? 63 : ctxt->om->om_len;
    memcpy(rx_str, ctxt->om->om_data, len);
    rx_str[len] = '\0';

    // Cắt bỏ ký tự \n hoặc \r ở cuối chuỗi để phân tích chính xác
    if (len > 0 && (rx_str[len-1] == '\n' || rx_str[len-1] == '\r')) rx_str[len-1] = '\0';
    if (len > 1 && (rx_str[len-2] == '\n' || rx_str[len-2] == '\r')) rx_str[len-2] = '\0';

    printf(">>> BLE NHAN DUOC: %s\n", rx_str);

    // Phân tích Lệnh điều hướng
    if (strncmp(rx_str, "CMD:", 4) == 0) {
        char cmd_char = rx_str[4];
        robot_cmd_t robot_cmd = CMD_NONE;

        switch (cmd_char) {
            case 'F': robot_cmd = CMD_FORWARD; break;
            case 'B': robot_cmd = CMD_BACKWARD; break;
            case 'L': robot_cmd = CMD_TURN_LEFT; break;
            case 'R': robot_cmd = CMD_TURN_RIGHT; break;
            case 'S': robot_cmd = CMD_STOP; break;
        }

        if (robot_cmd != CMD_NONE) {
            if (!robot_cmd_send(robot_cmd)) {
                printf(">>> BLE: Khong the gui lenh vao queue\n");
            }
        }
    }
    // Phân tích Lệnh PID
    else if (strncmp(rx_str, "PID:", 4) == 0) {
        float kp, ki, kd;
        if (sscanf(rx_str + 4, "%f,%f,%f", &kp, &ki, &kd) == 3) {
            printf(">>> UPDATE PID -> Kp: %.2f | Ki: %.2f | Kd: %.2f\n", kp, ki, kd);
            
            // CẬP NHẬT TRỰC TIẾP VÀO HỆ THỐNG
            balance_pid.kp = kp;
            balance_pid.ki = ki;
            balance_pid.kd = kd;

            // QUAN TRỌNG: Reset PID để xóa các sai số tích lũy cũ (error_sum)
            // Nếu không reset, robot có thể bị giật mạnh ngay khi đổi số
            pid_reset(&balance_pid); 
        }
    }

    return 0;
}

// Hàm bù nhìn bắt buộc cho cổng TX
static int gatt_svr_chr_dummy_cb(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    return 0; // Không làm gì cả
}
// --- BẢNG SERVICE GATT ---
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {   // Cổng RX (Nhận từ App)
                .uuid = &gatt_svr_chr_rx_uuid.u,
                .access_cb = gatt_svr_chr_write,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {   // Cổng TX (Gửi lên App)
                .uuid = &gatt_svr_chr_tx_uuid.u,
                .access_cb = gatt_svr_chr_dummy_cb, // Notify không cần hàm callback đọc/ghi
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &tx_handle, // Lấy handle để dùng cho việc gửi
            },
            { 0 }
        }
    },
    { 0 }
};

// --- QUẢN LÝ SỰ KIỆN KẾT NỐI ---
static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ble_conn_handle = event->connect.conn_handle;
                telemetry_notify_enabled = false;
                printf(">>> BLE: App Android da ket noi thanh cong!\n");
            } else {
                ble_app_advertise();
            }
            break;
        case BLE_GAP_EVENT_DISCONNECT:
            printf(">>> BLE: App ngat ket noi! Dang phat song lai...\n");
            ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            telemetry_notify_enabled = false;
            robot_cmd_send(CMD_STOP);
            ble_app_advertise();
            break;
        case BLE_GAP_EVENT_SUBSCRIBE:
            if (event->subscribe.attr_handle == tx_handle) {
                telemetry_notify_enabled = event->subscribe.cur_notify != 0;
                printf(">>> BLE: Telemetry Notify %s\n",
                       telemetry_notify_enabled ? "BAT" : "TAT");
            }
            break;
    }
    return 0;
}

// --- PHÁT SÓNG BLE ---
static void ble_app_advertise(void) {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    struct ble_hs_adv_fields rsp_fields; // Thêm gói phản hồi để chứa tên
    
    memset(&fields, 0, sizeof fields);
    memset(&rsp_fields, 0, sizeof rsp_fields);

    // 1. Gói tin chính (Fields): Chỉ chứa Cờ và UUID để tiết kiệm chỗ
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = (ble_uuid128_t*)&gatt_svr_svc_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    // 2. Gói tin phản hồi (Response Fields): Chứa tên thiết bị
    const char *name = ble_svc_gap_device_name();
    rsp_fields.name = (uint8_t *)name;
    rsp_fields.name_len = strlen(name);
    rsp_fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&rsp_fields); // Đăng ký gói phản hồi

    // 3. Bắt đầu phát sóng
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    
    // Lưu ý: tham số thứ 5 là ble_gap_event như đã sửa ở bước trước
    ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}
static void ble_app_on_sync(void) {
    ble_hs_id_infer_auto(0, &own_addr_type);
    ble_app_advertise();
    printf(">>> BLE DA KHOI TAO. DANG PHAT SONG 'SkaiBalance_BLE'...\n");
}

static void ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_server_init(void) {
    nimble_port_init();
    ble_svc_gap_device_name_set("SkaiBalance_BLE");
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_add_svcs(gatt_svr_svcs);
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(ble_host_task);
}

// --- HÀM PUBLIC ĐỂ ESP GỬI DỮ LIỆU LÊN APP ---
void ble_server_send_roll(float angle) {
    // Chỉ gửi khi App đang mở và kết nối
    if (ble_conn_handle != BLE_HS_CONN_HANDLE_NONE && telemetry_notify_enabled) {
        char tx_str[32];
        snprintf(tx_str, sizeof(tx_str), "ROLL:%.1f", angle);
        
        // Đóng gói thành mbuf và gửi qua Notify
        struct os_mbuf *om = ble_hs_mbuf_from_flat(tx_str, strlen(tx_str));
        if (om) {
            ble_gatts_notify_custom(ble_conn_handle, tx_handle, om);
        }
    }
}
