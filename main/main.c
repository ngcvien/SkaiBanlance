/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "nvs_flash.h"

#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_board_init.h"
// #include "speech_commands_action.h"
#include "model_path.h"
#include "esp_process_sdkconfig.h"

#include "robot_cmds.h"
#include "ble_server.h"

#include "stepper_driver.h"
#include "robot_control.h"

#include "mpu6050_driver.h"
#include "pid_controller.h"

pid_context_t balance_pid;

int wakeup_flag = 0;
static const esp_afe_sr_iface_t *afe_handle = NULL;
static volatile int task_flag = 0;
srmodel_list_t *models = NULL;

static const char *get_afe_input_format(int board_feed_channel)
{
    // Match AFE input format with board capture layout.
    // 4ch boards: MMNR (2 mic + 1 unknown + 1 reference)
    // 2ch boards: MM
    // 1ch boards: M
    switch (board_feed_channel) {
        case 4:
            return "MMNR";
        case 2:
            return "MM";
        case 1:
            return "M";
        default:
            return "M";
    }
}

void feed_Task(void *arg)
{
    esp_afe_sr_data_t *afe_data = arg;
    int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);
    int nch = afe_handle->get_feed_channel_num(afe_data);
    int feed_channel = esp_get_feed_channel();
    if (nch != feed_channel) {
        printf("AFE/Board channel mismatch: afe=%d, board=%d\n", nch, feed_channel);
        task_flag = 0;
        vTaskDelete(NULL);
        return;
    }
    int16_t *i2s_buff = malloc(audio_chunksize * sizeof(int16_t) * feed_channel);
    if (i2s_buff == NULL) {
        printf("Failed to allocate i2s buffer\n");
        task_flag = 0;
        vTaskDelete(NULL);
        return;
    }

    while (task_flag) {
        esp_get_feed_data(true, i2s_buff, audio_chunksize * sizeof(int16_t) * feed_channel);

        afe_handle->feed(afe_data, i2s_buff);
    }
    if (i2s_buff) {
        free(i2s_buff);
        i2s_buff = NULL;
    }
    vTaskDelete(NULL);
}

void detect_Task(void *arg)
{
    esp_afe_sr_data_t *afe_data = arg;
    int afe_chunksize = afe_handle->get_fetch_chunksize(afe_data);
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_ENGLISH);
    printf("multinet:%s\n", mn_name);
    esp_mn_iface_t *multinet = esp_mn_handle_from_name(mn_name);
    model_iface_data_t *model_data = multinet->create(mn_name, 6000);
    int mu_chunksize = multinet->get_samp_chunksize(model_data);
    esp_mn_commands_update_from_sdkconfig(multinet, model_data); // Add speech commands from sdkconfig
    if (mu_chunksize != afe_chunksize) {
        printf("Chunk size mismatch: multinet=%d, afe=%d\n", mu_chunksize, afe_chunksize);
        task_flag = 0;
        vTaskDelete(NULL);
        return;
    }
    //print active speech commands
    multinet->print_active_speech_commands(model_data);

    printf("------------detect start------------\n");
    while (task_flag) {
        afe_fetch_result_t* res = afe_handle->fetch(afe_data); 
        if (!res || res->ret_value == ESP_FAIL) {
            printf("fetch error!\n");
            break;
        }

        if (res->wakeup_state == WAKENET_DETECTED) {
            printf("WAKEWORD DETECTED\n");
            robot_activity_led_pulse();
	        multinet->clean(model_data);
        }

        if (res->raw_data_channels == 1 && res->wakeup_state == WAKENET_DETECTED) {
            wakeup_flag = 1;
        } else if (res->raw_data_channels > 1 && res->wakeup_state == WAKENET_CHANNEL_VERIFIED) {
            // For a multi-channel AFE, it is necessary to wait for the channel to be verified.
            printf("AFE_FETCH_CHANNEL_VERIFIED, channel index: %d\n", res->trigger_channel_id);
            wakeup_flag = 1;
        }

        if (wakeup_flag == 1) {
            esp_mn_state_t mn_state = multinet->detect(model_data, res->data);

            if (mn_state == ESP_MN_STATE_DETECTING) {
                continue;
            }

            if (mn_state == ESP_MN_STATE_DETECTED) {
                esp_mn_results_t *mn_result = multinet->get_results(model_data);
                // for (int i = 0; i < mn_result->num; i++) {
                //     printf("TOP %d, command_id: %d, phrase_id: %d, string: %s, prob: %f\n", 
                //     i+1, mn_result->command_id[i], mn_result->phrase_id[i], mn_result->string, mn_result->prob[i]);
                // }

                // --- Gui lenh vao Queue cho Robot ---
                if (mn_result->num > 0) {
                    int cmd_id = mn_result->command_id[0];
                    robot_cmd_t robot_cmd = CMD_NONE;

                    switch (cmd_id) {
                        case 1: robot_cmd = CMD_FORWARD; break;
                        case 2: robot_cmd = CMD_BACKWARD; break;
                        case 3: robot_cmd = CMD_TURN_LEFT; break;
                        case 4: robot_cmd = CMD_TURN_RIGHT; break;
                        case 5: robot_cmd = CMD_STOP; break;
                        default: break;
                    }

                    if (robot_cmd != CMD_NONE) {
                        if (!robot_cmd_send(robot_cmd)) {
                            printf("Khong the gui lenh giong noi vao queue\n");
                        }
                    }
                }
                // ------------------------------------------

                printf("-----------listening-----------\n");


            }

            if (mn_state == ESP_MN_STATE_TIMEOUT) {
                esp_mn_results_t *mn_result = multinet->get_results(model_data);
                printf("timeout, string:%s\n", mn_result->string);
                afe_handle->enable_wakenet(afe_data);
                wakeup_flag = 0;
                printf("\n-----------awaits to be waken up-----------\n");
                continue;
            }
        }
    }
    if (model_data) {
        multinet->destroy(model_data);
        model_data = NULL;
    }
    printf("detect exit\n");
    vTaskDelete(NULL);
}

// Thông số điều khiển Vị trí
float kp_pos = -0.008f;
// float kp_pos = -0.000f;
float kd_pos = 0.002f;
float target_position = 0.0f; 

// void balance_task(void *arg) {
//     float dt = 0.01f; 
//     TickType_t xLastWakeTime = xTaskGetTickCount();
//     const TickType_t xFrequency = pdMS_TO_TICKS(10); 

//     float motor_speed = 0.0f; 
//     float filtered_motor_speed = 0.0f; 

//     while(1) {
//         float offset_angle = -0.1f; // Điểm cân bằng vật lý
//         float current_pitch = mpu6050_get_smoothed_pitch() - offset_angle;

//         if (current_pitch > 45.0f || current_pitch < -45.0f) {
//             stepper_set_speed(0, 0);
//             pid_reset(&balance_pid); 
            
//             // Xóa bộ đếm phần cứng
//             stepper_reset_step();
//             target_position = 0.0f;
//             motor_speed = 0.0f;
//             filtered_motor_speed = 0.0f;
//         } 
//         else {
//             filtered_motor_speed = (0.8f * filtered_motor_speed) + (0.2f * motor_speed);

//             // a. Lấy dữ liệu Encoder ảo từ phần cứng
//             int32_t step_l = stepper_get_left_step();
//             int32_t step_r = stepper_get_right_step();
//             float current_position = (step_l + step_r) / 2.0f;

//             // b. Tính sai số vị trí
//             float pos_error = target_position - current_position;

//             // c. Áp dụng thuật toán (Nhớ đổi dấu kp_pos thành số âm nếu robot bị đẩy trôi nhanh hơn)
//             float dynamic_target_angle = (pos_error * kp_pos) - (filtered_motor_speed * kd_pos);

//             // d. Khóa an toàn
//             if (dynamic_target_angle > 1.2f) dynamic_target_angle = 1.2f;
//             if (dynamic_target_angle < -1.2f) dynamic_target_angle = -1.2f;

//             // e. Tính toán PID Góc
//             motor_speed = pid_compute(&balance_pid, dynamic_target_angle, current_pitch, dt);
//             stepper_set_speed(motor_speed, motor_speed);
//         }

//         vTaskDelayUntil(&xLastWakeTime, xFrequency);
//     }


// 1. Khai báo các thông số cho thuật toán tự học trọng tâm
float dynamic_offset = 4.3f; // Bắt đầu bằng góc bù vật lý bạn đã đo được bằng tay
float offset_learning_rate = 0.000002f; // Tốc độ tự học (Phải là một số CỰC KỲ nhỏ)

/// @brief Task cân bằng robot 2 bánh dựa trên dữ liệu góc nghiêng từ MPU6050 và thuật toán PID, 
/// với khả năng tự học trọng tâm
/// @param arg 
void balance_task(void *arg) {
    float dt = 0.01f; 
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); 

    float motor_speed = 0.0f; 
    float filtered_motor_speed = 0.0f; 
    float commanded_angle = 0.0f;
    float commanded_turn = 0.0f;

    while(1) {
        robot_motion_setpoint_t motion;
        robot_control_get_setpoint(&motion);

        // Ramp commands so direction changes do not jerk the chassis.
        const float angle_step = 0.008f;  // 0.8 degree/s at 100 Hz
        const float turn_step = 8.0f;     // 800 steps/s^2 at 100 Hz

        if (commanded_angle < motion.target_angle_deg) {
            commanded_angle += angle_step;
            if (commanded_angle > motion.target_angle_deg) commanded_angle = motion.target_angle_deg;
        } else if (commanded_angle > motion.target_angle_deg) {
            commanded_angle -= angle_step;
            if (commanded_angle < motion.target_angle_deg) commanded_angle = motion.target_angle_deg;
        }

        if (commanded_turn < motion.turn_speed_steps_s) {
            commanded_turn += turn_step;
            if (commanded_turn > motion.turn_speed_steps_s) commanded_turn = motion.turn_speed_steps_s;
        } else if (commanded_turn > motion.turn_speed_steps_s) {
            commanded_turn -= turn_step;
            if (commanded_turn < motion.turn_speed_steps_s) commanded_turn = motion.turn_speed_steps_s;
        }

        // a. Lọc nhiễu tốc độ động cơ để làm mượt quá trình tự học
        filtered_motor_speed = (0.92f * filtered_motor_speed) + (0.2f * motor_speed);

        // b. Cập nhật góc bù tự động dựa trên tốc độ trôi
        // LƯU Ý: Nếu xe trôi tới mà càng chạy nhanh hơn, hãy lật dấu cộng (+) thành trừ (-)
        // Do not let center-of-gravity learning cancel an intentional movement.
        if (motion.target_angle_deg == 0.0f && motion.turn_speed_steps_s == 0.0f) {
            dynamic_offset += (filtered_motor_speed * offset_learning_rate);
        }

        // c. Khóa an toàn: Không cho phép robot tự bù quá +- 1 độ so với thiết kế gốc
        if (dynamic_offset > 1.0f) dynamic_offset = 1.0f;
        if (dynamic_offset < -1.0f) dynamic_offset = -1.0f;

        // d. Đọc góc nghiêng và áp dụng góc bù vừa được tự động tính toán
        float current_pitch = mpu6050_get_smoothed_pitch() - dynamic_offset;

        // e. Kiểm tra an toàn chống ngã
        if (current_pitch > 45.0f || current_pitch < -45.0f) {
            stepper_set_speed(0, 0);
            pid_reset(&balance_pid); 
            stepper_reset_step();
            motor_speed = 0.0f;
            filtered_motor_speed = 0.0f;
            commanded_angle = 0.0f;
            commanded_turn = 0.0f;
            robot_control_emergency_stop();
            
            // Tùy chọn: Bạn có thể giữ nguyên dynamic_offset để nó nhớ trọng tâm, 
            // hoặc reset về 2.0f nếu muốn nó học lại từ đầu mỗi khi ngã.
        } 
        else {
            // f. PID Góc chỉ cần cố gắng giữ xe ở mức 0 độ (vì offset đã lo phần trọng tâm)
            float target_angle = commanded_angle;

            // Tính toán PID Góc
            motor_speed = pid_compute(&balance_pid, target_angle, current_pitch, dt);

            // Truyền xuống Driver điều khiển động cơ
            stepper_set_speed(motor_speed - commanded_turn,
                              motor_speed + commanded_turn);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/// @brief Task cân bằng robot 2 bánh dựa trên dữ liệu góc nghiêng từ MPU6050 và thuật toán PID.
/// @param arg Tham số truyền vào (không sử dụng trong trường hợp này). 
// void balance_task(void *arg) {
//     float dt = 0.01f; // 10ms = 0.01s
//     TickType_t xLastWakeTime = xTaskGetTickCount();
//     const TickType_t xFrequency = pdMS_TO_TICKS(10); // Chu kỳ chuẩn xác 10ms

//     while(1) {
//         // 1. Đọc góc nghiêng mượt mà từ MPU6050
//         float current_pitch = mpu6050_get_smoothed_pitch() - (-0.1f); // Hiệu chỉnh nếu cần thiết (ví dụ: nếu robot hơi nghiêng về một bên khi đứng yên)

//         // 2. Kiểm tra an toàn: Nếu ngã quá 45 độ -> Tắt động cơ để bảo vệ
//         if (current_pitch > 45.0f || current_pitch < -45.0f) {
//             stepper_set_speed(0, 0);
//             pid_reset(&balance_pid); // Xóa nháp PID
//         } 
//         else {
//             // 3. Tính toán PID: Mục tiêu (Setpoint) là 0 độ (Đứng thẳng)
//             // Lưu ý: Nếu trọng tâm robot không chuẩn, setpoint có thể là 2.5 độ hoặc -1.0 độ
//             float target_angle = 0.0f; 
//             float motor_speed = pid_compute(&balance_pid, target_angle, current_pitch, dt);

//             // 4. Bơm tốc độ ra 2 bánh xe
//             stepper_set_speed(motor_speed, motor_speed);
//         }

//         // 5. Ngủ chính xác cho đến chu kỳ tiếp theo (Đảm bảo dt luôn là 0.01s)
//         vTaskDelayUntil(&xLastWakeTime, xFrequency);
//     }
// }


void app_main()
{

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    models = esp_srmodel_init("model"); // partition label defined in partitions.csv
    ESP_ERROR_CHECK(esp_board_init(AUDIO_HAL_16K_SAMPLES, 1, 16));
    // ESP_ERROR_CHECK(esp_sdcard_init("/sdcard", 10));

    //d:\Projects\ESP32\esp-skainet\examples\en_speech_commands_recognition\main\speech_commands_action.c
    // Khoi tao Robot Core ---
    robot_core_init();
    // xTaskCreatePinnedToCore(&command_manager_task, "Cmd_Task", 2048, NULL, 5, NULL, 1);  // Xóa: gây race condition với robot_control_task

    ble_server_init();
    stepper_driver_init();
    robot_control_task_start();
    printf(">>> ĐỘNG CƠ ĐÃ SẴN SÀNG!\n");

    mpu6050_init();

    // Khởi tạo PID: Kp, Ki, Kd, Tốc độ tối đa (ví dụ 3000 bước/s)
    pid_init(&balance_pid, 4000.0f, 0.0f, 0.08f, 30000.0f);

    // Kích hoạt Task cân bằng (Ưu tiên cao nhất để không bị Bluetooth ngắt quãng)
    xTaskCreate(balance_task, "balance_task", 4096, NULL, 10, NULL);
    // stepper_set_speed(20.0, 20.0); // Chạy tiến với tốc độ 400 steps/s
    // xTaskCreate(robot_control_task, "robot_ctrl_task", 4096, NULL, 5, NULL);
    // ---------------------------------


    esp_afe_sr_data_t *afe_data = NULL;

#if CONFIG_IDF_TARGET_ESP32
    printf("This demo only support ESP32S3\n");
    return;
#else 
    int board_feed_channel = esp_get_feed_channel();
    const char *input_format = get_afe_input_format(board_feed_channel);
    printf("AFE input_format=%s (board_feed_channel=%d)\n", input_format, board_feed_channel);

    afe_config_t *afe_config = afe_config_init(input_format, models, AFE_TYPE_SR, AFE_MODE_LOW_COST);
    if (afe_config == NULL) {
        printf("afe_config_init failed\n");
        return;
    }

    afe_handle = esp_afe_handle_from_config(afe_config);
    if (afe_handle == NULL) {
        printf("esp_afe_handle_from_config failed\n");
        afe_config_free(afe_config);
        return;
    }

    afe_data = afe_handle->create_from_config(afe_config);
    if (afe_data == NULL) {
        printf("create_from_config failed\n");
        afe_config_free(afe_config);
        return;
    }

    afe_config_free(afe_config);
#endif

    task_flag = 1;
    xTaskCreatePinnedToCore(&detect_Task, "detect", 16 * 1024, (void*)afe_data, 5, NULL, 1);
    xTaskCreatePinnedToCore(&feed_Task, "feed", 16 * 1024, (void*)afe_data, 5, NULL, 0);


    while(1) {
        // Cached read keeps BLE telemetry from disturbing the 100 Hz MPU/PID loop.
        ble_server_send_roll(mpu6050_get_cached_pitch());
        vTaskDelay(pdMS_TO_TICKS(100)); // Send telemetry at 10 Hz.
    }
}
