

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "math.h"
#include "driver/gpio.h"

static const char *TAG = "RADIO";

#define MSG_BUF_SIZE            27

extern QueueHandle_t radioReceiveQueue;

void onPacketReceived(void *arg) {

    // TODO check if the whole 66 byte message could be read here
    // However it may not be ready yet
    int pin_number = CONFIG_INT_GPIO; //not really used
    BaseType_t higher_priority_task_woken = pdFALSE;

    // Send data to the back of the queue from the ISR
    xQueueSendFromISR(radioReceiveQueue, &pin_number, &higher_priority_task_woken);

    // Yield if a higher priority task was unblocked
    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

#if CONFIG_RADIO_TYPE_RFM68

#include "rf69.h"

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

    gpio_num_t interruptNum = CONFIG_INT_GPIO; // GPIO number for the interrupt
    gpio_set_direction(interruptNum, GPIO_MODE_INPUT);
    gpio_set_intr_type(interruptNum, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(interruptNum, (gpio_isr_t)onPacketReceived, 0);

    return true;
}
#endif

#if CONFIG_RADIO_TYPE_SX1262

#include "ra01s.h"

const uint8_t rxBwLut[] = {
    SX126X_GFSK_RX_BW_4_8,
    SX126X_GFSK_RX_BW_5_8,
    SX126X_GFSK_RX_BW_7_3,
    SX126X_GFSK_RX_BW_9_7,
    SX126X_GFSK_RX_BW_11_7,
    SX126X_GFSK_RX_BW_14_6,
    SX126X_GFSK_RX_BW_19_5,
    SX126X_GFSK_RX_BW_23_4,
    SX126X_GFSK_RX_BW_29_3,
    SX126X_GFSK_RX_BW_39_0,
    SX126X_GFSK_RX_BW_46_9,
    SX126X_GFSK_RX_BW_58_6,
    SX126X_GFSK_RX_BW_78_2,
    SX126X_GFSK_RX_BW_93_8,
    SX126X_GFSK_RX_BW_117_3,
    SX126X_GFSK_RX_BW_156_2,
    SX126X_GFSK_RX_BW_187_2,
    SX126X_GFSK_RX_BW_234_3,
    SX126X_GFSK_RX_BW_312_0,
    SX126X_GFSK_RX_BW_373_6,
    SX126X_GFSK_RX_BW_467_0,
  };

  int16_t findRxBw(float rxBw, const uint8_t* lut, size_t lutSize, float rxBwMax, uint8_t* val) {
  // lookup tables to avoid comparing a whole bunch of floats
  const uint16_t rxBwAvg[] = {
    53, 66, 85, 107, 132, 171, 215, 264,
    342, 430, 528, 684, 860, 1056, 1368,
    1717, 2108, 2732, 3428, 4203,
  };

  // iterate through the table and find whether the user-provided value
  // is lower than the pre-computed average of the adjacent bandwidth values
  // if it is, we consider that to be a match even though the actual value is not precise
  uint16_t rxBwInt = rxBw*10.0f;
  for(size_t i = 0; i < (lutSize - 1); i++) {
    if(rxBwInt < rxBwAvg[i]) {
      *val = lut[i];
      return(0);
    }
  }

  // if nothing matched up to here, match with the last value
  if(rxBwInt <= rxBwMax*10) {
    *val = lut[lutSize - 1];
    return(0);
  }

  return(-1);
}


#define SX126X_CRYSTAL_FREQ                            32.0f

void setModulationParamsFSK(float br, float freqDev, float rxBw, uint8_t pulseShape) {

    uint32_t brRaw = (uint32_t)((SX126X_CRYSTAL_FREQ * 1000000.0f * 32.0f) / (br * 1000.0f));

    uint8_t rxBandwidth = 0;
    findRxBw(rxBw, rxBwLut, sizeof(rxBwLut)/sizeof(rxBwLut[0]), 467.0f, &rxBandwidth);

    // set frequency deviation to lowest available setting (required for digimodes)
    // calculate raw frequency deviation value
    uint32_t freqDevRaw = (uint32_t)(((freqDev * 1000.0f) * (float)((uint32_t)(1) << 25)) / (SX126X_CRYSTAL_FREQ * 1000000.0f));

	uint8_t data[8] = {(uint8_t)((brRaw >> 16) & 0xFF), (uint8_t)((brRaw >> 8) & 0xFF), (uint8_t)(brRaw & 0xFF),
                     pulseShape, rxBandwidth,
                     (uint8_t)((freqDevRaw >> 16) & 0xFF), (uint8_t)((freqDevRaw >> 8) & 0xFF), (uint8_t)(freqDevRaw & 0xFF)};
	WriteCommand(SX126X_CMD_SET_MODULATION_PARAMS, data, sizeof(data)); // 0x8B
}

void setPacketParamsFSK(uint16_t preambleLen, uint8_t maxDetLen, uint8_t crcType, uint8_t syncWordLen, uint8_t addrCmp, uint8_t whiten, uint8_t packType, uint8_t payloadLen) {
    uint8_t preambleDetectorLen = maxDetLen >= 32 ? SX126X_GFSK_PREAMBLE_DETECT_32 :
                              maxDetLen >= 24 ? SX126X_GFSK_PREAMBLE_DETECT_24 :
                              maxDetLen >= 16 ? SX126X_GFSK_PREAMBLE_DETECT_16 :
                              maxDetLen >   0 ? SX126X_GFSK_PREAMBLE_DETECT_8 :
                              SX126X_GFSK_PREAMBLE_DETECT_OFF;

    uint8_t data[9] = {(uint8_t)((preambleLen >> 8) & 0xFF), (uint8_t)(preambleLen & 0xFF),
                     preambleDetectorLen, syncWordLen * 8, addrCmp,
                     packType, payloadLen, crcType, whiten};
	WriteCommand(SX126X_CMD_SET_PACKET_PARAMS, data, sizeof(data)); // 0x8C
}

bool radio_rawRead(uint8_t *buf, int max, int16_t *rssi_)
{
    uint8_t rxLen = LoRaReceive(buf, max);
    if ( rxLen > 0 ) { 
        int8_t rssi, snr;
        GetPacketStatus(&rssi, &snr);
        ESP_LOGD(pcTaskGetName(NULL), "rssi=%d[dBm] snr=%d[dB]", rssi, snr);
        if (rssi_) {
            *rssi_ = rssi;
        }
        return true;
    }
    return false;
}

#define CONFIG_USE_TCXO 1

bool radio_init() {
	// Initialize LoRa
	LoRaInit();
	int8_t txPowerInDbm = 22;

	uint32_t frequencyInHz = 0;
#if 0
#if CONFIG_433MHZ
	frequencyInHz = 433000000;
	ESP_LOGI(TAG, "Frequency is 433MHz");
#elif CONFIG_866MHZ
	frequencyInHz = 866000000;
	ESP_LOGI(TAG, "Frequency is 866MHz");
#elif CONFIG_915MHZ
	frequencyInHz = 915000000;
	ESP_LOGI(TAG, "Frequency is 915MHz");
#elif CONFIG_OTHER
	ESP_LOGI(TAG, "Frequency is %dMHz", CONFIG_OTHER_FREQUENCY);
	frequencyInHz = CONFIG_OTHER_FREQUENCY * 1000000;
#endif
#endif
	frequencyInHz = 868300000;
	ESP_LOGI(TAG, "Frequency is %d Hz", frequencyInHz);
#if CONFIG_USE_TCXO
	ESP_LOGW(TAG, "Enable TCXO");
	float tcxoVoltage = 3.3; // use TCXO
	bool useRegulatorLDO = true; // use DCDC + LDO
#else
	ESP_LOGW(TAG, "Disable TCXO");
	float tcxoVoltage = 0.0; // don't use TCXO
	bool useRegulatorLDO = false; // use only LDO in all modes
#endif

	LoRaDebugPrint(false);
	if (LoRaBegin(frequencyInHz, txPowerInDbm, tcxoVoltage, useRegulatorLDO) != 0) {
		ESP_LOGE(TAG, "Does not recognize the module");
		while(1) {
			vTaskDelay(1);
		}
	}

	//LoRaConfig(spreadingFactor, bandwidth, codingRate, preambleLength, payloadLen, crcOn, invertIrq);

	SetStopRxTimerOnPreambleDetect(false);

	SetDioIrqParams(SX126X_IRQ_ALL, //all interrupts enabled
		SX126X_IRQ_RX_DONE, //interrupts on DIO1
		SX126X_IRQ_NONE, //interrupts on DIO2
		SX126X_IRQ_NONE //interrupts on DIO3
	);

	SetPacketType(SX126X_PACKET_TYPE_GFSK);

    uint8_t syncwords[] = { 0x2D, 0xD4 };
	WriteRegister(SX126X_REG_SYNC_WORD_0, syncwords, 2);

    setPacketParamsFSK(24, 16, SX126X_GFSK_CRC_OFF, 
        2, SX126X_GFSK_ADDRESS_FILT_OFF, 
        SX126X_GFSK_WHITENING_OFF, SX126X_GFSK_PACKET_FIXED, 0x40);

    setModulationParamsFSK(8.22, 10.0, 250.0, SX126X_GFSK_FILTER_NONE);

	// Receive state no receive timeoout
    SetRx(0xFFFFFF);

    gpio_num_t interruptNum = CONFIG_INT_GPIO; // GPIO number for the interrupt
    gpio_set_direction(interruptNum, GPIO_MODE_INPUT);
    gpio_set_intr_type(interruptNum, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(interruptNum, (gpio_isr_t)onPacketReceived, 0);

    return true;
}

#endif