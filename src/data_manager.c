#include "data_manager.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(data_mgr, LOG_LEVEL_INF);

/* Two separate queues so slow Wi-Fi doesn't block fast BLE */
K_MSGQ_DEFINE(batch_q, sizeof(patient_batch_t), 4, 4);
K_MSGQ_DEFINE(ble_msgq, sizeof(patient_batch_t), 4, 4);

static patient_batch_t current_batch;

void data_manager_init(void)
{
    current_batch.count = 0;
}

void data_manager_push_sample(int hr, int spo2)
{
    if (current_batch.count < BATCH_SIZE) {
        current_batch.samples[current_batch.count].hr = hr;
        current_batch.samples[current_batch.count].spo2 = spo2;
        current_batch.samples[current_batch.count].timestamp = k_uptime_get_32();
        current_batch.count++;
    }

    if (current_batch.count >= BATCH_SIZE) {
        /* Push to Cloud Queue */
        k_msgq_put(&batch_q, &current_batch, K_NO_WAIT);
        
        /* Push to BLE Queue */
        k_msgq_put(&ble_msgq, &current_batch, K_NO_WAIT);

        current_batch.count = 0; 
        LOG_INF("Batch ready! Queued for Cloud & BLE.");
    }
}

int data_manager_get_batch(patient_batch_t *batch)
{
    return k_msgq_get(&batch_q, batch, K_FOREVER);
}

int data_manager_get_ble_batch(patient_batch_t *batch)
{
    return k_msgq_get(&ble_msgq, batch, K_FOREVER);
}
