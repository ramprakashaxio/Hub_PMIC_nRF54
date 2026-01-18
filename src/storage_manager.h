#ifndef STORAGE_MANAGER_H
#define STORAGE_MANAGER_H

#include <zephyr/kernel.h>
#include "data_manager.h"

/* Initialize the NVS file system */
int storage_init(void);

/* Save a batch to flash (Circular Buffer) */
int storage_save_batch(const patient_batch_t *batch);

/* Retrieve (and delete) the oldest batch */
int storage_get_next_batch(patient_batch_t *batch);

/* Check if there is pending data */
bool storage_has_data(void);

#endif
