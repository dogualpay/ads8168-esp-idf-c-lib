#ifndef __ADS8168_H
#define __ADS8168_H

#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

typedef struct {
    spi_device_handle_t spi;
    uint8_t acquire_bus_cnt;
} ADS8168;

typedef enum {
            ADCCMD_NOP         = 0b00000,
            ADCCMD_WR_REG      = 0b00001,
            ADCCMD_RD_REG      = 0b00010,
            ADCCMD_SET_BITS    = 0b00011,
            ADCCMD_CLR_BITS    = 0b00100,
            ADCCMD_ONTHEFLY    = 0b10000
		} adc_cmd_t;

void ADS8168_init(ADS8168 *dev, spi_host_device_t host, gpio_num_t cs);

esp_err_t 	ADS8168_init_device(ADS8168 *dev);
esp_err_t 	ADS8168_acquire_bus(ADS8168 *dev);
void 		ADS8168_release_bus(ADS8168 *dev);

// Ad-Hoc mode
void        ADS8168_setChannel(ADS8168 *dev, const uint8_t channel_no);
//uint16_t    ADS8168_readChannel(ADS8168 *dev, uint8_t* channel_out = NULL);
uint16_t    ADS8168_readChannel(ADS8168 *dev, uint8_t* channel_out);

// Custom Sequence mode
//p_err_t   ADS8168_setSequence(ADS8168 *dev, const uint8_t length, const uint8_t* channels, const uint8_t repeat = 1, bool loop = false);
esp_err_t   ADS8168_setSequence(ADS8168 *dev, const uint8_t length, const uint8_t* channels, uint8_t *repeat, bool *loop);
void        ADS8168_sequenceStart(ADS8168 *dev);

// On-The-Fly Mode, Crappy 0xFFFF problems..
void        ADS8168_enableOTFMode(ADS8168 *dev);
uint16_t    ADS8168_readChannelOTF(ADS8168 *dev, const uint8_t otf_channel_next);

void ADS8168_write_cmd(ADS8168 *dev, const adc_cmd_t cmd, const uint16_t address, const uint8_t data);

//spi_device_handle_t _spi;
//uint8_t _acquire_bus_cnt = 0;

#endif // __ADS8168_H