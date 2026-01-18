#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>

#include "storage_manager.h"

LOG_MODULE_REGISTER(storage_mgr, LOG_LEVEL_INF);

static struct nvs_fs fs;

/* * We use the label 'storage' defined in the overlay.
 */
#define STORAGE_NODE DT_NODE_BY_FIXED_PARTITION_LABEL(storage)

#define MAX_STORED_BATCHES 200 
#define KEY_WRITE_IDX 1
#define KEY_READ_IDX  2
#define KEY_DATA_BASE 100 

static uint16_t write_idx = 0;
static uint16_t read_idx = 0;

int storage_init(void)
{
    int rc;
    struct flash_pages_info info;

    /* * FIX: We explicitly grab the system flash controller.
     * The macro FIXED_PARTITION_DEVICE() fails on nRF54L15 because 
     * the RRAM controller hierarchy confuses the build system.
     */
    fs.flash_device = DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));
    
    if (!device_is_ready(fs.flash_device)) {
        LOG_ERR("Flash device not ready");
        return -1;
    }

    /* * FIX: We get the address offset manually.
     * This bypasses the broken parent-lookup macros.
     */
    fs.offset = DT_REG_ADDR(STORAGE_NODE);
    
    /* Get sector info to align writes correctly */
    rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
    if (rc) {
        LOG_ERR("Unable to get page info");
        return rc;
    }

    fs.sector_size = info.size;
    fs.sector_count = 4; /* Use 4 sectors for NVS management */

    rc = nvs_mount(&fs);
    if (rc) {
        LOG_ERR("NVS Mount failed: %d", rc);
        return rc;
    }

    /* Restore Indices (Persistence Check) */
    if (nvs_read(&fs, KEY_WRITE_IDX, &write_idx, sizeof(write_idx)) <= 0) write_idx = 0;
    if (nvs_read(&fs, KEY_READ_IDX, &read_idx, sizeof(read_idx)) <= 0) read_idx = 0;

    LOG_INF("Storage Init. Pending Batches: %d", 
        (write_idx >= read_idx) ? (write_idx - read_idx) : (MAX_STORED_BATCHES - read_idx + write_idx));
    
    return 0;
}

int storage_save_batch(const patient_batch_t *batch)
{
    int rc;
    uint16_t next_write_idx = (write_idx + 1) % MAX_STORED_BATCHES;

    /* Handle Ring Buffer Overflow */
    if (next_write_idx == read_idx) {
        LOG_WRN("Storage Full! Overwriting oldest data.");
        read_idx = (read_idx + 1) % MAX_STORED_BATCHES;
        nvs_write(&fs, KEY_READ_IDX, &read_idx, sizeof(read_idx));
    }

    /* Save Data */
    rc = nvs_write(&fs, KEY_DATA_BASE + write_idx, batch, sizeof(patient_batch_t));
    if (rc < 0) {
        LOG_ERR("Flash Write Failed: %d", rc);
        return rc;
    }

    /* Update Pointer */
    write_idx = next_write_idx;
    nvs_write(&fs, KEY_WRITE_IDX, &write_idx, sizeof(write_idx));
    
    LOG_INF("Saved batch to NVM (Idx: %d)", write_idx);
    return 0;
}

int storage_get_next_batch(patient_batch_t *batch)
{
    if (read_idx == write_idx) return -1; /* Empty */

    /* Read Oldest */
    int rc = nvs_read(&fs, KEY_DATA_BASE + read_idx, batch, sizeof(patient_batch_t));
    if (rc <= 0) {
        LOG_ERR("Flash Read Failed (Idx: %d)", read_idx);
        /* Skip corrupt entry to prevent lockup */
        read_idx = (read_idx + 1) % MAX_STORED_BATCHES;
        nvs_write(&fs, KEY_READ_IDX, &read_idx, sizeof(read_idx));
        return -1;
    }

    /* Advance Pointer (Delete) */
    read_idx = (read_idx + 1) % MAX_STORED_BATCHES;
    nvs_write(&fs, KEY_READ_IDX, &read_idx, sizeof(read_idx));

    return 0;
}

bool storage_has_data(void)
{
    return (read_idx != write_idx);
}
