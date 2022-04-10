#ifndef _RASPIAUDIO_PROTO_H_
#define _RASPIAUDIO_PROTO_H_

#include <stdint.h>
#include <esp_err.h>
#include "board.h"

#define RA_NMUTE_IO   CONFIG_RASPIAUDIO_NMUTE_PIN
#define RA_NENABLE_IO CONFIG_RASPIAUDIO_NENABLE_PIN

#define VP    GPIO_NUM_19     // Vol+ 
#define GAIN  GPIO_NUM_23     //
#define AUXD  GPIO_NUM_27     // AUX In detect 27
#define VM    GPIO_NUM_32     // Vol-
#define SDD   GPIO_NUM_34     // Sd detect

//Buttons https://github.com/RASPIAUDIO/museProto_speaker/blob/main/museProto_speaker/museProto_speaker.ino
// #define MU   GPIO_NUM_12    // short => mute/unmute  long => stop (deep sleep)
// #define VM   GPIO_NUM_32    // short => volume down  long => previous station
// #define VP   GPIO_NUM_19    // short => volume up   long => next station
// #define STOP GPIO_NUM_12    // for wake up

// #define I2C_MASTER_NUM            I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_NUM            0 // 16        /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE 0         /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0         /*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ        250000    /*!< I2C master clock frequency */
//#define I2C_MASTER_FREQ_HZ        100000

#define ACK_CHECK_EN              0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS             0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                   0x0              /*!< I2C ack value */
#define NACK_VAL                  0x1              /*!< I2C nack value */

#define min(a,b) (((a) < (b)) ? (a) : (b))
#define max(a,b) (((a) > (b)) ? (a) : (b))

#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */

#define ES8388_ADDR CONFIG_DAC_I2C_ADDR
#define GLED        GPIO_NUM_22

#define maxVol      64
#define M           maxVol-33



void setup_raspiaudio(void);
void ES8388_Write_Reg(uint8_t reg, uint8_t val);

esp_err_t raspiaudio_init(audio_hal_codec_config_t *codec_cfg);
esp_err_t raspiaudio_deinit(void);
esp_err_t raspiaudio_set_volume(int vol);
esp_err_t raspiaudio_get_volume(int *value);
esp_err_t raspiaudio_set_mute(bool enable);
esp_err_t raspiaudio_get_mute(bool *enabled);
esp_err_t raspiaudio_ctrl(audio_hal_codec_mode_t, audio_hal_ctrl_t);
esp_err_t raspiaudio_config_iface(audio_hal_codec_mode_t , audio_hal_codec_i2s_iface_t *);

void i2c_master_init(void);

esp_err_t ma_write_byte(
    uint8_t i2c_addr, 
    uint8_t prot, 
    uint16_t address,
    uint8_t value
);

esp_err_t ma_write(
    uint8_t i2c_addr, 
    uint8_t prot, 
    uint16_t address,
    uint8_t *wbuf, 
    uint8_t n
);

esp_err_t ma_read_byte(
    uint8_t i2c_addr, 
    uint8_t prot, 
    uint16_t address, 
    uint8_t *data
);

esp_err_t ma_read(
    uint8_t i2c_addr, 
    uint8_t prot, 
    uint16_t address,
    uint8_t *rbuf,
    uint8_t n
);

#endif /* _RASPIAUDIO_PROTO_H_  */
