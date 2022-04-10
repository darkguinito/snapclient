//
// RaspiAudio Muse Luxe
//

#include "RaspiAudio.h"

#include <stdint.h>
#include <stdio.h>

#include "driver/i2c.h"
#include "esp_log.h"
#include "board.h" 



#define RA_NMUTE_IO CONFIG_RASPIAUDIO_NMUTE_PIN
#define RA_NENABLE_IO CONFIG_RASPIAUDIO_NENABLE_PIN

// #define I2C_MASTER_NUM            I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_NUM            0 // 16        /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE 0         /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0         /*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ        250000    /*!< I2C master clock frequency */

#define WRITE_BIT                 I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                  I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN              0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS             0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                   0x0              /*!< I2C ack value */
#define NACK_VAL                  0x1              /*!< I2C nack value */

#define min(a,b) (((a) < (b)) ? (a) : (b))
#define max(a,b) (((a) > (b)) ? (a) : (b))

#define AC_ASSERT(a, format, b, ...) \
    if ((a) != 0) { \
        ESP_LOGE(TAG, format, ##__VA_ARGS__); \
        return b;\
    }
/////////////////////////////////////////////////////////////////
//*********************** NeoPixels  ***************************
////////////////////////////////////////////////////////////////
#define NUM_LEDS             1
#define LED_RMT_TX_CHANNEL   0
#define LED_RMT_TX_GPIO      22


#define BITS_PER_LED_CMD 24 
#define LED_BUFFER_ITEMS ((NUM_LEDS * BITS_PER_LED_CMD))

#define GREEN   0xFF0000
#define RED 	  0x00FF00
#define BLUE  	0x0000FF
#define WHITE   0xFFFFFF
#define YELLOW  0xE0F060


#define maxVol     64
#define M          32
//maxVol-66

struct led_state {
    uint32_t leds[NUM_LEDS];
};

void ws2812_control_init(void);
void ws2812_write_leds(struct led_state new_state);
static const char TAG[] = "es8388";	

static i2c_config_t i2c_cfg;
static int i2c_port;

#define ES8388_ADDR CONFIG_DAC_I2C_ADDR
#define GLED GPIO_NUM_22

static void battery(void *data);


audio_hal_func_t AUDIO_CODEC_RASPIAUDIO_DEFAULT_HANDLE = {
    .audio_codec_initialize = raspiaudio_init,
    .audio_codec_deinitialize = raspiaudio_deinit,
    .audio_codec_ctrl = raspiaudio_ctrl,
    .audio_codec_config_iface = raspiaudio_config_iface,
    .audio_codec_set_mute = raspiaudio_set_mute,
    .audio_codec_set_volume = raspiaudio_set_volume,
    .audio_codec_get_volume = raspiaudio_get_volume,
    .audio_hal_lock = NULL,
    .handle = NULL,
};

esp_err_t raspiaudio_deinit(void) {
  // TODO
	i2c_driver_delete(i2c_port);
  return ESP_OK;
}

esp_err_t raspiaudio_ctrl(audio_hal_codec_mode_t mode,
                       audio_hal_ctrl_t ctrl_state) {
  // TODO
  return ESP_OK;
}

esp_err_t raspiaudio_config_iface(audio_hal_codec_mode_t mode,
                               audio_hal_codec_i2s_iface_t *iface) {
  // TODO
  return ESP_OK;
}

esp_err_t raspiaudio_set_volume(int vol) {
  printf("VOLUME=============>Vgauche %u  Vdroit %u\n",vol,vol);
  uint8_t value1 = 0;
  uint8_t value2 = 0;
  float   tmp    = 0;
  ES8388_Write_Reg(25, 0x00);

  tmp = ((float)vol / 100.0) * 64;
  if (tmp > M) {
    value1 = (uint8_t)tmp - M;
  }
  else {
    value2 = (uint8_t)((M - tmp) * 3);
  }

  if (vol == 0) {
    ES8388_Write_Reg(25, 0x04);
  }

  ES8388_Write_Reg(26, value2);
  ES8388_Write_Reg(27, value2);
  ES8388_Write_Reg(46, value1);
  ES8388_Write_Reg(47, value1);

  return ESP_OK;
}

esp_err_t raspiaudio_get_volume(int *vol) {
  esp_err_t ret = ESP_OK;
 
  return ret;
}

esp_err_t raspiaudio_set_mute(bool enable) {
  esp_err_t ret = ESP_OK;
  uint8_t nmute = (enable) ? 0 : 1 ; 
  gpio_set_level(RA_NMUTE_IO, nmute);
  return ret;
}

esp_err_t ma120x0_get_mute(bool *enabled) {
  esp_err_t ret = ESP_OK;

  *enabled = false;  // TODO read from register
  return ret;
}

static i2c_config_t i2c_config = { 
    .mode             = I2C_MODE_MASTER,
    .sda_io_num       = CONFIG_DAC_I2C_SDA,
    .sda_pullup_en    = GPIO_PULLDOWN_DISABLE,
    .scl_io_num       = CONFIG_DAC_I2C_SCL,
    .scl_pullup_en    = GPIO_PULLDOWN_DISABLE,
    .master.clk_speed = I2C_MASTER_FREQ_HZ,
};

uint8_t  vauxd;
uint8_t  vsdd;

void setup(void){
  gpio_reset_pin(RA_NENABLE_IO);
  gpio_set_direction(RA_NENABLE_IO, GPIO_MODE_OUTPUT);
  gpio_set_level(RA_NENABLE_IO, 1);

  //VP
  gpio_reset_pin(VP);
  gpio_set_direction(VP, GPIO_MODE_INPUT);
  gpio_set_pull_mode(VP, GPIO_PULLUP_ONLY);
//
  ////VM
  gpio_reset_pin(VM);
  gpio_set_direction(VM, GPIO_MODE_INPUT);
  gpio_set_pull_mode(VM, GPIO_PULLUP_ONLY);

  // AUX detect
  gpio_reset_pin(AUXD);
  gpio_set_direction(AUXD, GPIO_MODE_INPUT);
  // gpio_set_pull_mode(AUXD, GPIO_PULLUP_ONLY);

  // SD detect
  gpio_reset_pin(SDD);
  gpio_set_direction(SDD, GPIO_MODE_INPUT);
  gpio_set_pull_mode(SDD, GPIO_PULLUP_ONLY);

  // pause
  gpio_reset_pin(RA_NMUTE_IO);
  gpio_set_direction(RA_NMUTE_IO, GPIO_MODE_INPUT);
  gpio_set_pull_mode(RA_NMUTE_IO, GPIO_PULLUP_ONLY);

  // Store SD detect and AUX detect initial values
  vsdd = gpio_get_level(SDD);
  vauxd = gpio_get_level(AUXD);
}

#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */


esp_err_t raspiaudio_init(audio_hal_codec_config_t *codec_cfg) {	
  esp_err_t ret = ESP_OK;

  // configure i2c
	i2c_port = I2C_MASTER_NUM;
	ret = i2c_param_config(i2c_port, &i2c_config);

	if (ret != ESP_OK) {
		ESP_LOGW(TAG, "I2C write failed %d", ret);
	}	

  ret = i2c_driver_install(
    i2c_port, 
    i2c_config.mode,
    I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
    I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 
    0
  );
  // provides MCLK
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
  WRITE_PERI_REG(PIN_CTRL, READ_PERI_REG(PIN_CTRL)& 0xFFFFFFF0);

  // reset
  ES8388_Write_Reg(0, 0x80);
  ES8388_Write_Reg(0, 0x00);
  // mute
  ES8388_Write_Reg(25, 0x04);
  ES8388_Write_Reg(1, 0x50);
  //powerup
  ES8388_Write_Reg(2, 0x00);
  // slave mode
  ES8388_Write_Reg(8, 0x00);
  // DAC powerdown
  ES8388_Write_Reg(4, 0xC0);
  // vmidsel/500k ADC/DAC idem
  ES8388_Write_Reg(0, 0x12);

  ES8388_Write_Reg(1, 0x00);
  // i2s 16 bits
  ES8388_Write_Reg(23, 0x18);
  // sample freq 256
  ES8388_Write_Reg(24, 0x02);
  // LIN2/RIN2 for mixer
  ES8388_Write_Reg(38, 0x09);
  // left DAC to left mixer
  ES8388_Write_Reg(39, 0x90);
  // right DAC to right mixer
  ES8388_Write_Reg(42, 0x90);
  // DACLRC ADCLRC idem
  ES8388_Write_Reg(43, 0x80);
  ES8388_Write_Reg(45, 0x00);
  // DAC volume max
  ES8388_Write_Reg(27, 0x00);
  ES8388_Write_Reg(26, 0x00);

  ES8388_Write_Reg(2 , 0xF0);
  ES8388_Write_Reg(2 , 0x00);
  ES8388_Write_Reg(29, 0x1C);
  // DAC power-up LOUT1/ROUT1 enabled
  ES8388_Write_Reg(4, 0x30);
  // unmute
  ES8388_Write_Reg(25, 0x00);
  // amp validation
  gpio_set_level(RA_NENABLE_IO, 1);

	if (ret != ESP_OK) {
		ESP_LOGW(TAG, "I2C write failed %d", ret);
	}	
  setup();
  printf("-------------------->>>> init ES8388\n");
	return (ret == ESP_OK);
}

esp_err_t hal_i2c_master_mem_read(i2c_port_t i2c_num, uint8_t DevAddr,uint8_t MemAddr,uint8_t* data_rd, size_t size)
{
	 if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();//a cmd list
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( DevAddr << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, MemAddr, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( DevAddr << 1 ) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t hal_i2c_master_mem_write(i2c_port_t i2c_num, uint8_t DevAddr,uint8_t MemAddr,uint8_t* data_wr, size_t size)
{
	 if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();//a cmd list
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( DevAddr << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, MemAddr, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void ES8388_Write_Reg(uint8_t reg, uint8_t val)
{
  uint8_t buf[2];
  buf[0] = reg;
  buf[1] = val;
  hal_i2c_master_mem_write((i2c_port_t)0, ES8388_ADDR, buf[0], buf + 1, 1);
}
