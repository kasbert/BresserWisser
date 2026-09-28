

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "math.h"
#include "driver/gpio.h"

#include "rf69.h"

static const char *TAG = "RADIO";

#define MSG_BUF_SIZE            27

uint8_t spiRead(uint8_t reg);
uint8_t spiBurstRead(uint8_t reg, uint8_t* dest, uint8_t len);
uint8_t spiWrite(uint8_t reg, uint8_t val);
#define millis() xTaskGetTickCount()*portTICK_PERIOD_MS

bool radio_rawRead(uint8_t *buf, int max, int16_t *rssi)
{
	// Get the interrupt cause
	uint8_t irqflags2 = spiRead(RH_RF69_REG_28_IRQFLAGS2);
    if (irqflags2)
	    ESP_LOGD(TAG, "available irqflags2=%x", irqflags2);
	if (irqflags2 & RH_RF69_IRQFLAGS2_PAYLOADREADY) {
		// fifo level > thresh, so we can read the FIFO
        if (rssi) {
            *rssi = -((int8_t)(spiRead(RH_RF69_REG_24_RSSIVALUE) >> 1));
        }
		setModeIdle();
        spiBurstRead(RH_RF69_REG_00_FIFO, buf, max);
        setModeRx(); // Continue receiving
        return true;
	}
    return false;
}


#define RADIOLIB_RF69_DIV_EXPONENT                              19
#define RADIOLIB_RF69_CRYSTAL_FREQ                              32.0f


void setBitRate(float bitRate) {
    uint16_t bitRateRaw = 32000 / bitRate;
    spiWrite(RH_RF69_REG_03_BITRATEMSB, (bitRateRaw >> 8) & 0xFF);
    spiWrite(RH_RF69_REG_04_BITRATELSB, bitRateRaw & 0xFF);
}

void setFreqDev(float freqDev) {
    // calculate frequency deviation from carrier frequency
    uint32_t fdev = (freqDev * ((uint32_t)1 << RADIOLIB_RF69_DIV_EXPONENT)) / 32000;
    spiWrite(RH_RF69_REG_05_FDEVMSB, (fdev >> 8) & 0xFF);
    spiWrite(RH_RF69_REG_06_FDEVLSB, fdev & 0xFF);
}

void setRXBandwith(float rxBw, bool ookEnabled) {
    // calculate exponent and mantissa values for receiver bandwidth
    uint8_t RXBW = 0;
    for(int8_t e = 7; e >= 0; e--) {
        for(int8_t m = 2; m >= 0; m--) {
            float point = (RADIOLIB_RF69_CRYSTAL_FREQ * 1000000.0f)/(((4 * m) + 16) * ((uint32_t)1 << (e + (ookEnabled ? 3 : 2))));
            if(fabsf(rxBw - (point / 1000.0f)) <= 0.1f) {
                // set Rx bandwidth
                RXBW = (m << 3) | e;
    spiWrite(RH_RF69_REG_19_RXBW, RXBW);
                break;
            }
        }
    }
}

void setPayloadLength(uint8_t len) {
    spiWrite(RH_RF69_REG_38_PAYLOADLENGTH, len);
}

extern QueueHandle_t radioReceiveQueue;

void onPacketReceived(void *arg) {

    // TODO check if the whole 66 byte message could be read here
    // However it may not be ready yet
    int pin_number = CONFIG_DIO0_GPIO;
    BaseType_t higher_priority_task_woken = pdFALSE;

    // Send data to the back of the queue from the ISR
    xQueueSendFromISR(radioReceiveQueue, &pin_number, &higher_priority_task_woken);

    // Yield if a higher priority task was unblocked
    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}


bool radio_init() {
	float freq;

    // Initialize Radio
	if (!init()) {
		ESP_LOGE(TAG, "RFM69 radio init failed");
		return false;
	}
	ESP_LOGI(TAG, "RFM69 radio init OK!");


#if CONFIG_RF69_FREQ_315
	freq = 315.0;
#elif CONFIG_RF69_FREQ_433
	freq = 433.0;
#elif CONFIG_RF69_FREQ_868
	freq = 868.3;
#elif CONFIG_RF69_FREQ_915
	freq = 915.0;
#endif
	ESP_LOGW(TAG, "Set frequency to %.1fMHz", freq);

	// Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
	// No encryption
	if (!setFrequency(freq)) {
		ESP_LOGE(TAG, "setFrequency failed");
		return false;
	}
	ESP_LOGI(TAG, "RFM69 radio setFrequency OK!");

    uint8_t syncwords[] = { 0x2D, 0xD4 };
    setSyncWords(syncwords, sizeof(syncwords));
    setPreambleLength(16);
    setBitRate(8.22);
    setFreqDev(10.0);
    setRXBandwith(250.0, false);

    setPayloadLength(MSG_BUF_SIZE);

    // set mode to standby
    #if 0
        (RH_RF69_PACKETCONFIG1_PACKETFORMAT_VARIABLE | RH_RF69_PACKETCONFIG1_DCFREE_NONE | RH_RF69_PACKETCONFIG1_ADDRESSFILTERING_NONE) // RH_RF69_REG_37_PACKETCONFIG1
    #endif
    spiWrite(RH_RF69_REG_02_DATAMODUL, (RH_RF69_DATAMODUL_DATAMODE_PACKET | RH_RF69_DATAMODUL_MODULATIONTYPE_FSK | RH_RF69_DATAMODUL_MODULATIONSHAPING_FSK_NONE));
    //spiWrite(RH_RF69_REG_1A_AFCBW, 0xf4);
    spiWrite(RH_RF69_REG_37_PACKETCONFIG1, (RH_RF69_PACKETCONFIG1_DCFREE_NONE | RH_RF69_PACKETCONFIG1_ADDRESSFILTERING_NONE));

#if CONFIG_RF69_POWER_HIGH
	// If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
	// ishighpowermodule flag set like this:
	setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW
	ESP_LOGW(TAG, "Set TX power high");
#endif

	// The encryption key has to be the same as the one in the server
	//uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
	//	0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
	//setEncryptionKey(key);
    //setEncryptionKey(0);

    ESP_LOGI(TAG, "RFM69 MY CONFIG");

    spiWrite(RH_RF69_REG_25_DIOMAPPING1, 0x40); 
    setOpMode(RH_RF69_OPMODE_MODE_RX);

    gpio_num_t interruptNum = CONFIG_DIO0_GPIO; // GPIO number for the interrupt
    gpio_set_direction(interruptNum, GPIO_MODE_INPUT);
    gpio_set_intr_type(interruptNum, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(interruptNum, (gpio_isr_t)onPacketReceived, 0);

    return true;
}