#ifndef DATA_MANAGER_H
#define DATA_MANAGER_H

#include <zephyr/kernel.h>

#define BATCH_SIZE 10  /* 10 samples per batch */

/* Single measurement */
typedef struct {
    uint32_t timestamp;
    int hr;
    int spo2;
} sensor_sample_t;

/* A batch of 10 measurements */
typedef struct {
    sensor_sample_t samples[BATCH_SIZE];
    uint8_t count;
} patient_batch_t;

void data_manager_init(void);
void data_manager_push_sample(int hr, int spo2);

/* Fetch functions for different threads */
int data_manager_get_batch(patient_batch_t *batch);      /* For HTTP */
int data_manager_get_ble_batch(patient_batch_t *batch);  /* For BLE */

#endif
