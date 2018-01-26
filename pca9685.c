#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#include "sdkconfig.h"

#define I2C_EXAMPLE_MASTER_SCL_IO   4    /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO   5    /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_FREQ_HZ  100000     /*!< I2C master clock frequency */
#define I2C_EXAMPLE_MASTER_NUM      I2C_NUM_0   /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */


#define I2C_ADDRESS     0x40    /*!< lave address for PCA9685 */

#define ACK_CHECK_EN    0x1     /*!< I2C master will check ack from slave */
#define ACK_CHECK_DIS   0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL         0x0     /*!< I2C ack value */
#define NACK_VAL        0x1     /*!< I2C nack value */
#define MODE1           0x00    /*!< Mode register 1 */
#define MODE2           0x01    /*!< Mode register 2 */
#define SUBADR1         0x02    /*!< I2C-bus subaddress 1 */
#define SUBADR2         0x03    /*!< I2C-bus subaddress 2 */
#define SUBADR3         0x04    /*!< I2C-bus subaddress 3 */
#define ALLCALLADR      0x05    /*!< LED All Call I2C-bus address */
#define LED0            0x6     /*!< LED0 start register */
#define LED0_ON_L       0x6     /*!< LED0 output and brightness control byte 0 */
#define LED0_ON_H       0x7     /*!< LED0 output and brightness control byte 1 */
#define LED0_OFF_L      0x8     /*!< LED0 output and brightness control byte 2 */
#define LED0_OFF_H      0x9     /*!< LED0 output and brightness control byte 3 */
#define LED_MULTIPLYER  4       /*!< For the other 15 channels */
#define ALLLED_ON_L     0xFA    /*!< load all the LEDn_ON registers, byte 0 (turn 0-7 channels on) */
#define ALLLED_ON_H     0xFB    /*!< load all the LEDn_ON registers, byte 1 (turn 8-15 channels on) */
#define ALLLED_OFF_L    0xFC    /*!< load all the LEDn_OFF registers, byte 0 (turn 0-7 channels off) */
#define ALLLED_OFF_H    0xFD    /*!< load all the LEDn_OFF registers, byte 1 (turn 8-15 channels off) */
#define PRE_SCALE       0xFE    /*!< prescaler for output frequency */
#define CLOCK_FREQ      25000000.0  /*!< 25MHz default osc clock */

static char tag[] = "PCA9685";

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);

void write_i2c_register_two_words(uint8_t regaddr, uint16_t valueOn, uint16_t valueOff);
void generic_write_i2c_register_word(uint8_t regaddr, uint16_t value);
static void i2c_example_master_init(void);
static void resetPCA9685(void);
static void setFrequencyPCA9685(uint16_t freq);
static void turnAllOff(void);
static void setPWM(uint8_t num, uint16_t on, uint16_t off);
void disp_buf(uint16_t* buf, uint8_t len);


/**
 * @brief      Sets the pwm of the pin
 *
 * @param[in]  num   The pin number
 * @param[in]  on    On time
 * @param[in]  off   Off time
 */
static void setPWM(uint8_t num, uint16_t on, uint16_t off)
{
    uint8_t pinAddress = LED0_ON_L + LED_MULTIPLYER * num;
    write_i2c_register_two_words(pinAddress & 0xff, on, off);
}

/**
 * @brief      Write two 16 bit values to the same register on an i2c device
 *
 * @param[in]  regaddr   The register address
 * @param[in]  valueOn   The value on
 * @param[in]  valueOff  The value off
 */
void write_i2c_register_two_words(uint8_t regaddr, uint16_t valueOn, uint16_t valueOff)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, valueOn & 0xff, ACK_VAL);
    i2c_master_write_byte(cmd, valueOn >> 8, NACK_VAL);
    i2c_master_write_byte(cmd, valueOff & 0xff, ACK_VAL);
    i2c_master_write_byte(cmd, valueOff >> 8, NACK_VAL);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

/**
 * @brief      Write a 16 bit value to a register on an i2c device
 *
 * @param[in]  regaddr  The register address
 * @param[in]  value    The value
 */
void generic_write_i2c_register_word(uint8_t regaddr, uint16_t value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, value & 0xff, ACK_VAL);
    i2c_master_write_byte(cmd, value >> 8, NACK_VAL);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

/**
 * @brief      test function to show buffer
 *
 * @param      buf   The buffer
 * @param[in]  len   The length
 */
void disp_buf(uint16_t* buf, uint8_t len)
{
    uint8_t i;
    for (i = 0; i < len; i++)
    {
        printf("%02x ", buf[i]);
        if (( i + 1 ) % 16 == 0)
        {
            printf("\n");
        }
    }
    printf("\n");
}

/**
 * @brief i2c master initialization
 */
static void i2c_example_master_init(void)
{
    ESP_LOGD(tag, ">> PCA9685");
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;
    
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0));
}

/**
 * @brief      Resets the PCA9685
 */
static void resetPCA9685(void)
{
    // Reset chip
    printf("Resetting chip\n");
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, MODE1, ACK_CHECK_EN);   // 0x0 = "Mode register 1"
    i2c_master_write_byte(cmd, 0x80, ACK_CHECK_EN);    // 0x80 = "Reset"
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

/**
 * @brief      Sets the frequency of PCA9685
 *
 * @param[in]  freq  The frequency
 */
static void setFrequencyPCA9685(uint16_t freq)
{
    // Send to sleep
    printf("Setting ext frequency\n");
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, MODE1, ACK_CHECK_EN);    // 0x0 = "Mode register 1"
    i2c_master_write_byte(cmd, 0x10, ACK_CHECK_EN);     // 0x10 = go to sleep
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    // Set prescaler
    printf("\tSetting prescaler\n");
    // calculation on page 25 of datasheet
    uint8_t prescale_val = round((CLOCK_FREQ / 4096 / (0.9*freq)) - 1+0.5);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, PRE_SCALE, ACK_CHECK_EN);    // 0xFE = "Prescaler"
    i2c_master_write_byte(cmd, prescale_val, ACK_CHECK_EN); //6 = 0x06 = "prescaler frequency"
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 100/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    // reset again
    printf("\tResetting again\n");
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, MODE1, ACK_CHECK_EN);    // 0x0 = "Mode register 1"
    i2c_master_write_byte(cmd, 0x80, ACK_CHECK_EN);     // 0x80 = "Reset"
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    // Send to sleep again
    printf("\tSending to sleep\n");
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, MODE1, ACK_CHECK_EN);    // 0x0 = "Mode register 1"
    i2c_master_write_byte(cmd, 0x10, ACK_CHECK_EN);     // 0x10 = go to sleep
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    printf("\tWaiting\n");
    vTaskDelay(5/portTICK_PERIOD_MS);

    printf("\tWriting 0xa0 for auto increment\n");
    // Write oldMode again
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, MODE1, ACK_CHECK_EN);    // 0x0 = "Mode register 1"
    i2c_master_write_byte(cmd, 0xa0, ACK_CHECK_EN);     // 0xa0 = auto increment
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    // // Read back set data
    // cmd = i2c_cmd_link_create();
    // ESP_ERROR_CHECK(i2c_master_start(cmd));
    // ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_READ, 1));
    // i2c_master_read_byte(cmd, data,   0);
    // i2c_master_read_byte(cmd, data+1, 0);
    // i2c_master_read_byte(cmd, data+2, 1);
    // ESP_ERROR_CHECK(i2c_master_stop(cmd));
    // ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 100/portTICK_PERIOD_MS));
    // i2c_cmd_link_delete(cmd);
}

/**
 * @brief      Turn all LEDs off
 */
static void turnAllOff(void)
{
    // Turn all LEDs off
    printf("Turning all off\n");

    uint16_t valueOn = 0;
    uint16_t valueOff = 4096;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ALLLED_ON_L, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, valueOn & 0xff, ACK_VAL);
    i2c_master_write_byte(cmd, valueOn >> 8, NACK_VAL);
    i2c_master_write_byte(cmd, valueOff & 0xff, ACK_VAL);
    i2c_master_write_byte(cmd, valueOff >> 8, NACK_VAL);

    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

void task_PCA9685(void *ignore)
{
    i2c_example_master_init();

    resetPCA9685();
    setFrequencyPCA9685(1000);
    turnAllOff();

    printf("Finished setup, entering loop now\n");

    while(1)
    {
        /*
        // led turn on and 100ms off, starting from pin 0...15
        for (uint8_t pin = 0; pin < 16; pin++)
        {
            printf("Turn LED on\n");
            setPWM(pin, 4096, 0);

            vTaskDelay(100/portTICK_PERIOD_MS);

            printf("Turn LED off\n");
            setPWM(pin, 0, 4096);
        }
        */

        // fade led up and down, starting from pin 0...15
        for (uint8_t pin = 0; pin < 7; pin++)
        {
            for (uint16_t i = 0; i < 4096; i+=8)
            {
                // fade up
                setPWM(pin, 0, i);

                vTaskDelay(5/portTICK_PERIOD_MS);
            }

            for (uint16_t i = 0; i < 4096; i+=8)
            {
                // fade down
                setPWM(pin, i, 0);

                vTaskDelay(5/portTICK_PERIOD_MS);
            }

            // ensure it's really off
            setPWM(pin, 0, 4096);

            vTaskDelay(100/portTICK_PERIOD_MS);    
        }

        // write_i2c_register_two_words(LED0_ON_L, 4096, 0);
        // vTaskDelay(1000/portTICK_PERIOD_MS);
        // write_i2c_register_two_words(LED0_ON_L, 0, 4096);   
    }

    vTaskDelete(NULL);
}

void app_main()
{
    xTaskCreate(task_PCA9685, "task_PCA9685", 1024 * 2, (void* ) 0, 10, NULL);
}

