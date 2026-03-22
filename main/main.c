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
                        xQueueSend(robot_cmd_queue, &robot_cmd, 0);
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

void app_main()
{
    models = esp_srmodel_init("model"); // partition label defined in partitions.csv
    ESP_ERROR_CHECK(esp_board_init(AUDIO_HAL_16K_SAMPLES, 1, 16));
    // ESP_ERROR_CHECK(esp_sdcard_init("/sdcard", 10));

    //d:\Projects\ESP32\esp-skainet\examples\en_speech_commands_recognition\main\speech_commands_action.c
    // Khoi tao Robot Core ---
    robot_core_init();
    xTaskCreatePinnedToCore(&command_manager_task, "Cmd_Task", 2048, NULL, 5, NULL, 1);
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
}
