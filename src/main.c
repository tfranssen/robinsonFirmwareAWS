//#define hasTubes 1
#define hasLidar 1

#include <cJSON.h>
#include <cJSON_os.h>
#include <date_time.h>
#include <modem/lte_lc.h>
#include <modem/modem_info.h>
#include <modem/nrf_modem_lib.h>
#include <net/aws_iot.h>
#include <nrf_modem.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>

LOG_MODULE_REGISTER(robinson, CONFIG_LOG_DEFAULT_LEVEL);


char versionNr[10] = "v1.8";
bool debug = false;
int debounceTime = 500;       // in ms
int sensorEnabledTime = 5000; // in ms
int abInterval = 1500;         // in ms
int sendDelay = 1800;         // in seconds
int lowThreshold = 350;       // in cm
int highThreshold = 360;      // in cm
int lidarInterval = 500;      // in ms

char imei[32] = {
    '\0',
};

char ccid[32] = {
    '\0',
};

char plmn[32] = {
    '\0',
};

char signalVal[32] = {
    '\0',
};

void copyFirstChars(const char *source, char *destination, int n) {
  strncpy(destination, source, n);
  destination[n] = '\0'; // Ensure null-termination of destination string
}

static char topicString[75];

#define WDT_FEED_TRIES 5
#define WDT_ALLOW_CALLBACK 0
#define WDT_MAX_WINDOW 7200000U // 60*60*1000*2 = 2 hours
#define WDT_MIN_WINDOW 0U
#define WDG_FEED_INTERVAL 50U

int activeTime = 0;
int tauTime = 0;

int wdt_channel_id;
const struct device *const wdt = DEVICE_DT_GET(DT_ALIAS(watchdog0));

#ifdef hasLidar
#define I2C2_NODE DT_NODELABEL(tfluna)
#endif
#define GPIO_NODE DT_NODELABEL(gpio0)
#define PIR_NODE DT_ALIAS(pir)
#define PIR_NODE2 DT_ALIAS(pir2)
#define BUTTON_NODE DT_ALIAS(sw0)

#define SWITCH_NODE DT_ALIAS(sw1)
#ifdef hasTubes
#define PRES1_NODE DT_ALIAS(pres1)
#define PRES2_NODE DT_ALIAS(pres2)
#endif
static const struct device *gpio_dev;
#ifdef hasTubes
static struct gpio_callback gpio_cb1;
static struct gpio_callback gpio_cb2;
#endif
static struct gpio_callback gpio_cb3; //Button callback
static struct gpio_callback gpio_cb4; //Pir callback
static struct gpio_callback gpio_cb5; //Pir2 callback

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);
static const struct gpio_dt_spec pir = GPIO_DT_SPEC_GET(PIR_NODE, gpios);
static const struct gpio_dt_spec pir2 = GPIO_DT_SPEC_GET(PIR_NODE2, gpios);


#ifdef hasLidar
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C2_NODE);
#endif
static const struct gpio_dt_spec switch1 = GPIO_DT_SPEC_GET(SWITCH_NODE, gpios);

#ifdef hasTubes
static const struct gpio_dt_spec pres1Input =
    GPIO_DT_SPEC_GET(PRES2_NODE, gpios);
static const struct gpio_dt_spec pres2Input =
    GPIO_DT_SPEC_GET(PRES1_NODE, gpios);
#endif

static struct k_work_delayable toggle5V_work;
static struct k_work_delayable toggleInterrupt_work;

#ifdef hasTubes
static struct k_work_delayable pres1_work;
static struct k_work_delayable pres2_work;
#endif
static struct k_work_delayable checkLastPirEvent_work;



#ifdef hasLidar
// Lidar settings
bool lidarThreadOn = 0;
#define TFL_DIST_LO 0x00 // R Unit: cm
#define TFL_DIST_HI 0x01 // R
#define TFL_FLUX_LO 0x02 // R
#define TFL_FLUX_HI 0x03 // R
#define TFL_TEMP_LO 0x04 // R Unit: 0.01 Celsius
#define TFL_TEMP_HI 0x05 // R
static int numPeople = 0;
static int numPeopleDelta = 0;
static int lastDist = 0;
static int pir1Count = 0;
static int pir1CountDelta = 0;
static int pir2Count = 0;
static int pir2CountDelta = 0;
#endif

#define BATVOLT_R1 4.7f                 // MOhm
#define BATVOLT_R2 10.0f                // MOhm
#define INPUT_VOLT_RANGE 3.6f           // Volts
#define VALUE_RANGE_10_BIT 1.023        // (2^10 - 1) / 1000

#define ADC_NODE DT_NODELABEL(adc)
#define ADC_RESOLUTION 10
#define ADC_GAIN ADC_GAIN_1_6
#define ADC_REFERENCE ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10)
#define ADC_1ST_CHANNEL_ID 0
#define ADC_1ST_CHANNEL_INPUT SAADC_CH_PSELP_PSELP_AnalogInput0

#define BUFFER_SIZE 1
static int16_t m_sample_buffer[BUFFER_SIZE];

static const struct device *adc_dev;

static const struct adc_channel_cfg m_1st_channel_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id = ADC_1ST_CHANNEL_ID,
	.input_positive   = ADC_1ST_CHANNEL_INPUT,
};

uint32_t last_button_press_time;

#ifdef hasTubes
uint32_t last_pres1_time;
uint32_t last_pres2_time;
int pres1Count = 0;
int pres2Count = 0;
int aToBcount = 0;
int bToAcount = 0;
int pres1CountDelta = 0;
int pres2CountDelta = 0;
int aToBcountDelta = 0;
int bToAcountDelta = 0;
#endif

bool switchOn = 0;
bool interruptOn = 0;

BUILD_ASSERT(!IS_ENABLED(CONFIG_LTE_AUTO_INIT_AND_CONNECT),
             "This sample does not support LTE auto-init and connect");

#define APP_TOPICS_COUNT CONFIG_AWS_IOT_APP_SUBSCRIPTION_LIST_COUNT

static struct k_work_delayable shadow_update_work;
static struct k_work_delayable connect_work;
static struct k_work shadow_update_version_work;

static bool cloud_connected;

static K_SEM_DEFINE(lte_connected, 0, 1);
static K_SEM_DEFINE(date_time_obtained, 0, 1);

NRF_MODEM_LIB_ON_INIT(aws_iot_init_hook, on_modem_lib_init, NULL);

/* Initialized to value different than success (0) */
static int modem_lib_init_result = -1;

static int get_battery_voltage(uint16_t *battery_voltage)
{
	int err;

	const struct adc_sequence sequence = {
		.channels = BIT(ADC_1ST_CHANNEL_ID),
		.buffer = m_sample_buffer,
		.buffer_size = sizeof(m_sample_buffer), // in bytes!
		.resolution = ADC_RESOLUTION,
	};

	if (!adc_dev) {
		return -1;
	}

	err = adc_read(adc_dev, &sequence);
	if (err) {
		printk("ADC read err: %d\n", err);

		return err;
	}

	float sample_value = 0;
	for (int i = 0; i < BUFFER_SIZE; i++) {
		sample_value += (float) m_sample_buffer[i];
	}
	sample_value /= BUFFER_SIZE;

	*battery_voltage = (uint16_t)(sample_value * (INPUT_VOLT_RANGE / VALUE_RANGE_10_BIT) * ((BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2));

	return 0;
}

static void on_modem_lib_init(int ret, void *ctx) {
  modem_lib_init_result = ret;
}

static int json_add_obj(cJSON *parent, const char *str, cJSON *item) {
  cJSON_AddItemToObject(parent, str, item);

  return 0;
}

static int json_add_str(cJSON *parent, const char *str, const char *item) {
  cJSON *json_str;

  json_str = cJSON_CreateString(item);
  if (json_str == NULL) {
    return -ENOMEM;
  }

  return json_add_obj(parent, str, json_str);
}

static int json_add_number(cJSON *parent, const char *str, double item) {
  cJSON *json_num;

  json_num = cJSON_CreateNumber(item);
  if (json_num == NULL) {
    return -ENOMEM;
  }

  return json_add_obj(parent, str, json_num);
}

static int shadow_update(bool version_number_include) {
  int err;
  char *message;
  int64_t message_ts = 0;

  err = date_time_now(&message_ts);
  if (err) {
    printk("date_time_now, error: %d\n", err);
    return err;
  }

  cJSON *root_obj = cJSON_CreateObject();
  cJSON *state_obj = cJSON_CreateObject();
  cJSON *reported_obj = cJSON_CreateObject();

  if (root_obj == NULL || state_obj == NULL || reported_obj == NULL) {
    cJSON_Delete(root_obj);
    cJSON_Delete(state_obj);
    cJSON_Delete(reported_obj);
    err = -ENOMEM;
    return err;
  }

  if (version_number_include) {
    err = json_add_str(reported_obj, "app_version", versionNr);
  } else {
    err = 0;
  }

  modem_info_string_get(MODEM_INFO_OPERATOR, plmn, sizeof(plmn));
  int plmnInt = atoi(plmn);



  uint16_t batteryVoltage = 0;
  get_battery_voltage(&batteryVoltage);
  printk("Battery voltage: %u mV\n", batteryVoltage);
  err += json_add_number(reported_obj, "ts", message_ts);

  err += json_add_number(reported_obj, "batv", batteryVoltage);
  err += json_add_number(reported_obj, "5V", switchOn);
  err += json_add_number(reported_obj, "sendInterval", sendDelay);
  err += json_add_number(reported_obj, "activeTime", activeTime);
  err += json_add_number(reported_obj, "tauTime", tauTime);      
  err += json_add_number(reported_obj, "plmn", plmnInt);
  err += json_add_number(reported_obj, "signal", signalInt);        

#ifdef hasLidar
  err += json_add_number(reported_obj, "sensorEnabledTime", sensorEnabledTime);
  err += json_add_number(reported_obj, "lowLidarThreshold", lowThreshold);
  err += json_add_number(reported_obj, "highLidarThreshold", highThreshold);
  err += json_add_number(reported_obj, "lidarInterval", lidarInterval);
  err += json_add_number(reported_obj, "lastDist", lastDist);  
  err += json_add_number(reported_obj, "lidarCount", numPeople);
  err += json_add_number(reported_obj, "lidarCountDelta", numPeopleDelta);
  err += json_add_number(reported_obj, "pir1Count", pir1Count);
  err += json_add_number(reported_obj, "pir2Count", pir2Count);
  err += json_add_number(reported_obj, "pir1CountDelta", pir1CountDelta);
  err += json_add_number(reported_obj, "pir2CountDelta", pir2CountDelta);
  pir1CountDelta = 0;
  pir2CountDelta = 0;
  numPeopleDelta = 0;  
#endif
#ifdef hasTubes
  err += json_add_number(reported_obj, "debounceTime", debounceTime);
  err += json_add_number(reported_obj, "aBInterval", abInterval);  
  err += json_add_number(reported_obj, "pres1Count", pres1Count);
  err += json_add_number(reported_obj, "pres2Count", pres2Count);
  err += json_add_number(reported_obj, "aToBcount", aToBcount);
  err += json_add_number(reported_obj, "bToAcount", bToAcount);
  err += json_add_number(reported_obj, "pres1CountDelta", pres1CountDelta);
  err += json_add_number(reported_obj, "pres2CountDelta", pres2CountDelta);
  err += json_add_number(reported_obj, "aToBcountDelta", aToBcountDelta);
  err += json_add_number(reported_obj, "bToAcountDelta", bToAcountDelta);
  pres1CountDelta = 0;
  pres2CountDelta = 0;
  aToBcountDelta = 0;
  bToAcountDelta = 0;
#endif
  err += json_add_obj(state_obj, "reported", reported_obj);
  err += json_add_obj(root_obj, "state", state_obj);

  if (err) {
    printk("json_add, error: %d\n", err);
    goto cleanup;
  }

  message = cJSON_Print(root_obj);
  if (message == NULL) {
    printk("cJSON_Print, error: returned NULL\n");
    err = -ENOMEM;
    goto cleanup;
  }

  struct aws_iot_data tx_data = {.qos = MQTT_QOS_1_AT_LEAST_ONCE,
                                 .topic.type = AWS_IOT_SHADOW_TOPIC_UPDATE,
                                 .ptr = message,
                                 .len = strlen(message)};

  printk("Publishing: %s to AWS IoT broker\n", message);

  err = aws_iot_send(&tx_data);
  if (err) {
    printk("aws_iot_send, error: %d\n", err);
  }
  
  modem_info_string_get(MODEM_INFO_RSRP, signalVal, sizeof(signalVal));
  int signalInt = atoi(signalVal);  

  cJSON_FreeString(message);

cleanup:

  cJSON_Delete(root_obj);

  return err;
}

static void connect_work_fn(struct k_work *work) {
  int err;

  if (cloud_connected) {
    return;
  }

  err = aws_iot_connect(NULL);
  if (err) {
    printk("aws_iot_connect, error: %d\n", err);
  }

  printk("Next connection retry in %d seconds\n",
         CONFIG_CONNECTION_RETRY_TIMEOUT_SECONDS);

  k_work_schedule(&connect_work,
                  K_SECONDS(CONFIG_CONNECTION_RETRY_TIMEOUT_SECONDS));
}

static void shadow_update_work_fn(struct k_work *work) {
  int err;

  if (!cloud_connected) {
    return;
  }

  err = shadow_update(false);
  if (err) {
    printk("shadow_update, error: %d\n", err);
  }

  printk("Next data publication in %d seconds\n", sendDelay);

  k_work_schedule(&shadow_update_work, K_SECONDS(sendDelay));
}

static void shadow_update_version_work_fn(struct k_work *work) {
  int err;

  err = shadow_update(true);
  if (err) {
    printk("shadow_update, error: %d\n", err);
  }
}
static void print_received_data(const char *buf, const char *topic,
                                size_t topic_len) {
  char *str = NULL;
  cJSON *root_obj = NULL;

  char topicNew[topic_len + 1];

  bool changedSetting = false;

  copyFirstChars(topic, topicNew, topic_len);

  printk("Received topic is: %s\n", topicNew);
  printk("Settings topic is: %s\n", topicString);

  int result = strcmp(topicNew, topicString);

  printk("Result is %d\n", result);

  root_obj = cJSON_Parse(buf);
  if (root_obj == NULL) {
    printk("cJSON Parse failure\n");
    return;
  }

  str = cJSON_Print(root_obj);
  if (str == NULL) {
    printk("Failed to print JSON object\n");
    goto clean_exit;
  }

  printk("Data received from AWS IoT console:\nTopic: %.*s\nMessage: %s\n",
         topic_len, topic, str);

  if (result == 0) {
    cJSON *sendIntervalJSON = cJSON_GetObjectItem(root_obj, "sendInterval");
    if (cJSON_IsNumber(sendIntervalJSON) && sendIntervalJSON->valueint > 0) {
      sendDelay = sendIntervalJSON->valueint;
      printk("sendInterval changed.\n");
      changedSetting = true;
    }

    cJSON *debounceTimeJSON = cJSON_GetObjectItem(root_obj, "debounceTime");
    if (cJSON_IsNumber(debounceTimeJSON) && debounceTimeJSON->valueint > 0) {
      debounceTime = debounceTimeJSON->valueint;
      printk("debounceTime changed.\n");
      changedSetting = true;
    }

    cJSON *sensorEnabledTimeJSON =
        cJSON_GetObjectItem(root_obj, "sensorEnabledTime");
    if (cJSON_IsNumber(sensorEnabledTimeJSON) &&
        sensorEnabledTimeJSON->valueint > 0) {
      sensorEnabledTime = sensorEnabledTimeJSON->valueint;
      printk("sensorEnabledTime changed.\n");
      changedSetting = true;
    }

    cJSON *abIntervalJSON = cJSON_GetObjectItem(root_obj, "abInterval");
    if (cJSON_IsNumber(abIntervalJSON) && abIntervalJSON->valueint > 0) {
      abInterval = abIntervalJSON->valueint;
      printk("abInterval changed.\n");
      changedSetting = true;
    }

    cJSON *lowThresholdJSON = cJSON_GetObjectItem(root_obj, "lowThreshold");
    if (cJSON_IsNumber(lowThresholdJSON) && lowThresholdJSON->valueint > 0) {
      lowThreshold = lowThresholdJSON->valueint;
      printk("lowThreshold changed.\n");
      changedSetting = true;
    }

    cJSON *highThresholdJSON = cJSON_GetObjectItem(root_obj,
    "highThreshold"); if (cJSON_IsNumber(highThresholdJSON) &&
    highThresholdJSON->valueint > 0) {
      highThreshold = highThresholdJSON->valueint;
      printk("highThreshold changed.\n");
      changedSetting = true;
    }

    cJSON *lidarIntervalJSON = cJSON_GetObjectItem(root_obj,
    "lidarInterval"); if (cJSON_IsNumber(lidarIntervalJSON) &&
    lidarIntervalJSON->valueint > 0) {
      lidarInterval = lidarIntervalJSON->valueint;
      printk("lidarInterval changed.\n");
      changedSetting = true;
    }

    cJSON *debugJSON = cJSON_GetObjectItem(root_obj,
    "debug"); 
    if (cJSON_IsNumber(debugJSON)) {
      if (debugJSON->valueint == 1) {
        debug = true;
      } else {
        debug = false;
      }
      changedSetting = true;
    }
  }

  if (changedSetting) {
    k_work_cancel_delayable(&shadow_update_work);
    k_work_schedule(&shadow_update_work, K_SECONDS(2));
  }

  cJSON_FreeString(str);

clean_exit:
  cJSON_Delete(root_obj);
}

void aws_iot_event_handler(const struct aws_iot_evt *const evt) {
  switch (evt->type) {
  case AWS_IOT_EVT_CONNECTING:
    printk("AWS_IOT_EVT_CONNECTING\n");
    break;
  case AWS_IOT_EVT_CONNECTED:
    printk("AWS_IOT_EVT_CONNECTED\n");

    cloud_connected = true;
    /* This may fail if the work item is already being processed,
     * but in such case, the next time the work handler is executed,
     * it will exit after checking the above flag and the work will
     * not be scheduled again.
     */
    (void)k_work_cancel_delayable(&connect_work);

    if (evt->data.persistent_session) {
      printk("Persistent session enabled\n");
    }
    boot_write_img_confirmed();
    k_work_submit(&shadow_update_version_work);
    k_work_schedule(&shadow_update_work, K_SECONDS(sendDelay));

    int err = lte_lc_psm_req(true);
    if (err) {
      printk("Requesting PSM failed, error: %d\n", err);
    }
    break;
  case AWS_IOT_EVT_READY:
    printk("AWS_IOT_EVT_READY\n");
    break;
  case AWS_IOT_EVT_DISCONNECTED:
    printk("AWS_IOT_EVT_DISCONNECTED\n");
    cloud_connected = false;
    /* This may fail if the work item is already being processed,
     * but in such case, the next time the work handler is executed,
     * it will exit after checking the above flag and the work will
     * not be scheduled again.
     */
    (void)k_work_cancel_delayable(&shadow_update_work);
    k_work_schedule(&connect_work, K_NO_WAIT);
    break;
  case AWS_IOT_EVT_DATA_RECEIVED:
    printk("AWS_IOT_EVT_DATA_RECEIVED\n");
    print_received_data(evt->data.msg.ptr, evt->data.msg.topic.str,
                        evt->data.msg.topic.len);
    break;
  case AWS_IOT_EVT_PUBACK:
    printk("AWS_IOT_EVT_PUBACK, message ID: %d\n", evt->data.message_id);
    wdt_feed(wdt, wdt_channel_id);
    break;
  case AWS_IOT_EVT_FOTA_START:
    printk("AWS_IOT_EVT_FOTA_START\n");
    break;
  case AWS_IOT_EVT_FOTA_ERASE_PENDING:
    printk("AWS_IOT_EVT_FOTA_ERASE_PENDING\n");
    printk("Disconnect LTE link or reboot\n");
    err = lte_lc_offline();
    if (err) {
      printk("Error disconnecting from LTE\n");
    }
    break;
  case AWS_IOT_EVT_FOTA_ERASE_DONE:
    printk("AWS_FOTA_EVT_ERASE_DONE\n");
    printk("Reconnecting the LTE link");
    err = lte_lc_connect();
    if (err) {
      printk("Error connecting to LTE\n");
    }
    break;
  case AWS_IOT_EVT_FOTA_DONE:
    printk("AWS_IOT_EVT_FOTA_DONE\n");
    printk("FOTA done, rebooting device\n");
    aws_iot_disconnect();
    sys_reboot(0);
    break;
  case AWS_IOT_EVT_FOTA_DL_PROGRESS:
    printk("AWS_IOT_EVT_FOTA_DL_PROGRESS, (%d%%)", evt->data.fota_progress);
  case AWS_IOT_EVT_ERROR:
    printk("AWS_IOT_EVT_ERROR, %d\n", evt->data.err);
    break;
  case AWS_IOT_EVT_FOTA_ERROR:
    printk("AWS_IOT_EVT_FOTA_ERROR");
    break;
  default:
    printk("Unknown AWS IoT event type: %d\n", evt->type);
    break;
  }
}

#ifdef hasTubes
void pres1Work(struct k_work *work) {

  pres1Count++;
  pres1CountDelta++;
  printk("Pres 1 active. Count is %d\r\n", pres1Count);

  if (last_pres1_time - last_pres2_time < abInterval) {
    aToBcount++;
    aToBcountDelta++;
    printk("A->B: %d\r\n", aToBcount);
  }

  if (debug == true) {
    printk("Sending debug info\n");
    k_work_submit(&shadow_update_version_work);
  }
}

void pres2Work(struct k_work *work) {
  pres2Count++;
  pres2CountDelta++;
  printk("Pres 2 active. Count is %d\r\n", pres2Count);

  if (last_pres2_time - last_pres1_time < abInterval) {
    bToAcount++;
    bToAcountDelta++;
    printk("B->A: %d\r\n", bToAcount);
  }
  if (debug == true) {
    printk("Sending debug info\n");
    k_work_submit(&shadow_update_version_work);
  }
}
#endif

void turn_switch_off(void) { gpio_pin_set_dt(&switch1, 0); }

void turn_switch_on(void) { gpio_pin_set_dt(&switch1, 1); }

void init_switch(void) {
  gpio_pin_configure_dt(&switch1, GPIO_OUTPUT_INACTIVE);
}

void checkLastPirEvent(struct k_work *work) {
  uint32_t nowTime = k_uptime_get_32();
  if (nowTime - last_button_press_time >= sensorEnabledTime) {
    k_work_schedule(&toggleInterrupt_work, K_NO_WAIT);
    k_work_schedule(&toggle5V_work, K_MSEC(500));
  } else {
    // Check again if time has passed
    k_work_schedule(&checkLastPirEvent_work, K_MSEC(5000));
  }
}



#ifdef hasTubes
void pres1_callback(const struct device *gpiob, struct gpio_callback *cb,
                    gpio_port_pins_t pins) {
  uint32_t newPressTime = k_uptime_get_32();

  // debounce
  if (k_uptime_get_32() - last_pres1_time < debounceTime) {
    return;
  }

  k_work_schedule(&pres1_work, K_NO_WAIT);

  last_pres1_time = newPressTime;
}

void pres2_callback(const struct device *gpiob, struct gpio_callback *cb,
                    gpio_port_pins_t pins) {
  uint32_t newPressTime = k_uptime_get_32();

  // debounce
  if (k_uptime_get_32() - last_pres2_time < debounceTime) {
    return;
  }

  k_work_schedule(&pres2_work, K_NO_WAIT);

  last_pres2_time = newPressTime;
}
#endif

void button_callback(const struct device *gpiob, struct gpio_callback *cb,
                     gpio_port_pins_t pins) {
  uint32_t newPressTime = k_uptime_get_32();

  // debounce
  if (k_uptime_get_32() - last_button_press_time < debounceTime) {
    return;
  }
  last_button_press_time = newPressTime;

  if (!switchOn) {
    k_work_schedule(&toggle5V_work, K_NO_WAIT);
    k_work_schedule(&toggleInterrupt_work, K_MSEC(100));
    k_work_schedule(&checkLastPirEvent_work, K_MSEC(sensorEnabledTime));
  }
}

void pir_callback(const struct device *gpiob, struct gpio_callback *cb,
                  gpio_port_pins_t pins) {
  uint32_t newPressTime = k_uptime_get_32();

  // debounce
  if (k_uptime_get_32() - last_button_press_time < debounceTime) {
    return;
  }
  last_button_press_time = newPressTime;


  if (!switchOn) {
    printk("Pir 1 motion\r\n");
    pir1Count++;
    pir1CountDelta++;    
    k_work_schedule(&toggle5V_work, K_NO_WAIT);
    k_work_schedule(&toggleInterrupt_work, K_MSEC(100));
    k_work_schedule(&checkLastPirEvent_work, K_MSEC(sensorEnabledTime));
  }
}

void pir2_callback(const struct device *gpiob, struct gpio_callback *cb,
                  gpio_port_pins_t pins) {
  uint32_t newPressTime = k_uptime_get_32();

  // debounce
  if (k_uptime_get_32() - last_button_press_time < debounceTime) {
    return;
  }
  last_button_press_time = newPressTime;


  if (!switchOn) {
    printk("Pir 2 motion\r\n");
    pir2Count++;
    pir2CountDelta++;    
    k_work_schedule(&toggle5V_work, K_NO_WAIT);
    k_work_schedule(&toggleInterrupt_work, K_MSEC(100));
    k_work_schedule(&checkLastPirEvent_work, K_MSEC(sensorEnabledTime));
  }
}

bool init_sensors(void) {
  int ret;
#ifdef hasTubes
  ret = gpio_pin_configure_dt(&pres1Input, GPIO_INPUT);
  if (ret != 0) {
    printk("Error %d: failed to configure %s pin %d\n", ret,
           pres1Input.port->name, pres1Input.pin);

    return false;
  }

  ret = gpio_pin_configure_dt(&pres2Input, GPIO_INPUT);
  if (ret != 0) {
    printk("Error %d: failed to configure %s pin %d\n", ret,
           pres2Input.port->name, pres2Input.pin);

    return false;
  }
#endif

  ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
  if (ret != 0) {
    printk("Error %d: failed to configure %s pin %d\n", ret, button.port->name,
           button.pin);

    return false;
  }

  ret = gpio_pin_configure_dt(&pir, GPIO_INPUT);
  if (ret != 0) {
    printk("Error %d: failed to configure %s pin %d\n", ret, pir.port->name,
           pir.pin);

    return false;
  }

  ret = gpio_pin_configure_dt(&pir2, GPIO_INPUT);
  if (ret != 0) {
    printk("Error %d: failed to configure %s pin %d\n", ret, pir.port->name,
           pir2.pin);

    return false;
  }  

  ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
  if (ret != 0) {
    printk("Error %d: failed to configure interrupt on %s pin %d\n", ret,
           button.port->name, button.pin);

    return false;
  }

  ret = gpio_pin_interrupt_configure_dt(&pir, GPIO_INT_EDGE_TO_ACTIVE);
  if (ret != 0) {
    printk("Error %d: failed to configure interrupt on %s pin %d\n", ret,
           pir.port->name, pir.pin);
    return false;
  }

  ret = gpio_pin_interrupt_configure_dt(&pir2, GPIO_INT_EDGE_TO_ACTIVE);
  if (ret != 0) {
    printk("Error %d: failed to configure interrupt on %s pin %d\n", ret,
           pir.port->name, pir2.pin);
    return false;
  }  

#ifdef hasTubes
  gpio_init_callback(&gpio_cb1, pres1_callback, BIT(pres1Input.pin));
  gpio_add_callback(pres1Input.port, &gpio_cb1);

  gpio_init_callback(&gpio_cb2, pres2_callback, BIT(pres2Input.pin));
  gpio_add_callback(pres2Input.port, &gpio_cb2);
#endif

  gpio_init_callback(&gpio_cb3, button_callback, BIT(button.pin));
  gpio_add_callback(button.port, &gpio_cb3);

  gpio_init_callback(&gpio_cb4, pir_callback, BIT(pir.pin));
  gpio_add_callback(pir.port, &gpio_cb4);

  gpio_init_callback(&gpio_cb5, pir2_callback, BIT(pir2.pin));
  gpio_add_callback(pir2.port, &gpio_cb5);  

  return true;
}

int disableInterrupts(void) {
#ifdef hasTubes
  int ret = gpio_pin_interrupt_configure_dt(&pres1Input, GPIO_INT_DISABLE);
  if (ret != 0) {
    printk("Error %d: failed to configure interrupt on %s pin %d\n", ret,
           pres1Input.port->name, pres1Input.pin);

    return 0;
  }

  ret = gpio_pin_interrupt_configure_dt(&pres2Input, GPIO_INT_DISABLE);
  if (ret != 0) {
    printk("Error %d: failed to configure interrupt on %s pin %d\n", ret,
           pres2Input.port->name, pres2Input.pin);

    return 0;
  }
#endif

  return 0;
}

int enableInterrupts(void) {
#ifdef hasTubes
  int ret =
      gpio_pin_interrupt_configure_dt(&pres1Input, GPIO_INT_EDGE_TO_ACTIVE);
  if (ret != 0) {
    printk("Error %d: failed to configure interrupt on %s pin %d\n", ret,
           pres1Input.port->name, pres1Input.pin);

    return 0;
  }

  ret = gpio_pin_interrupt_configure_dt(&pres2Input, GPIO_INT_EDGE_TO_ACTIVE);
  if (ret != 0) {
    printk("Error %d: failed to configure interrupt on %s pin %d\n", ret,
           pres2Input.port->name, pres2Input.pin);

    return 0;
  }
#endif
  return 0;
}

void toggleInterrupts(struct k_work *work) {
  if (interruptOn) {
    disableInterrupts();
    printk("Interrupts disabled\r\n");
    interruptOn = 0;
  } else {
    enableInterrupts();
    printk("Interrupts enabled\r\n");
    interruptOn = 1;
  }
}

static void lte_handler(const struct lte_lc_evt *const evt) {
  switch (evt->type) {
  case LTE_LC_EVT_NW_REG_STATUS:
    if ((evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_HOME) &&
        (evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_ROAMING)) {
      break;
    }

    printk("Network registration status: %s\n",
           evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME
               ? "Connected - home network"
               : "Connected - roaming");

    k_sem_give(&lte_connected);
    break;
  case LTE_LC_EVT_PSM_UPDATE:
    printk("PSM parameter update: TAU: %d, Active time: %d\n", evt->psm_cfg.tau,
           evt->psm_cfg.active_time);
    activeTime = evt->psm_cfg.active_time;
    tauTime = evt->psm_cfg.tau;
    break;
  case LTE_LC_EVT_EDRX_UPDATE: {
    char log_buf[60];
    ssize_t len;

    len = snprintf(log_buf, sizeof(log_buf),
                   "eDRX parameter update: eDRX: %f, PTW: %f",
                   evt->edrx_cfg.edrx, evt->edrx_cfg.ptw);
    if (len > 0) {
      printk("%s\n", log_buf);
    }
    break;
  }
  case LTE_LC_EVT_RRC_UPDATE:
    printk("RRC mode: %s\n",
           evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED ? "Connected" : "Idle");
    break;
  case LTE_LC_EVT_CELL_UPDATE:
    printk("LTE cell changed: Cell ID: %d, Tracking area: %d\n", evt->cell.id,
           evt->cell.tac);
    break;
  default:
    break;
  }
}

static void modem_configure(void) {
  int err;

  if (IS_ENABLED(CONFIG_LTE_AUTO_INIT_AND_CONNECT)) {
    /* Do nothing, modem is already configured and LTE connected. */
  } else {
    err = lte_lc_init_and_connect_async(lte_handler);

    if (err) {
      printk("Modem could not be configured, error: %d\n", err);
      return;
    }
  }
}

static void nrf_modem_lib_dfu_handler(void) {
  int err;

  err = modem_lib_init_result;

  switch (err) {
  case NRF_MODEM_DFU_RESULT_OK:
    LOG_INF("Modem update suceeded, reboot");
    sys_reboot(SYS_REBOOT_COLD);
    break;
  case NRF_MODEM_DFU_RESULT_UUID_ERROR:
  case NRF_MODEM_DFU_RESULT_AUTH_ERROR:
    LOG_INF("Modem update failed, error: %d", err);
    LOG_INF("Modem will use old firmware");
    sys_reboot(SYS_REBOOT_COLD);
    break;
  case NRF_MODEM_DFU_RESULT_HARDWARE_ERROR:
  case NRF_MODEM_DFU_RESULT_INTERNAL_ERROR:
    LOG_INF("Modem update malfunction, error: %d, reboot", err);
    sys_reboot(SYS_REBOOT_COLD);
    break;
  case NRF_MODEM_DFU_RESULT_VOLTAGE_LOW:
    LOG_INF("Modem update cancelled due to low power, error: %d", err);
    LOG_INF("Please reboot once you have sufficient power for the DFU");
    break;
  default:
    break;
  }
}

static int app_topics_subscribe(void) {
  int err;
  static char settingsString[20] = "/settings";
  strcpy(topicString, imei);
  strcat(topicString, settingsString);

  printk("Topic %s\n", topicString);

  const struct aws_iot_topic_data topics_list[APP_TOPICS_COUNT] = {
      [0].str = topicString, [0].len = strlen(topicString)};

  err = aws_iot_subscription_topics_add(topics_list, ARRAY_SIZE(topics_list));
  if (err) {
    printk("aws_iot_subscription_topics_add, error: %d\n", err);
  }

  return err;
}

static void date_time_event_handler(const struct date_time_evt *evt) {
  switch (evt->type) {
  case DATE_TIME_OBTAINED_MODEM:
    /* Fall through */
  case DATE_TIME_OBTAINED_NTP:
    /* Fall through */
  case DATE_TIME_OBTAINED_EXT:
    printk("Date time obtained\n");
    k_sem_give(&date_time_obtained);

    /* De-register handler. At this point the sample will have
     * date time to depend on indefinitely until a reboot occurs.
     */
    date_time_register_handler(NULL);
    break;
  case DATE_TIME_NOT_OBTAINED:
    printk("DATE_TIME_NOT_OBTAINED\n");
    break;
  default:
    printk("Unknown event: %d", evt->type);
    break;
  }
}

bool init_adc() {
  int err;

  adc_dev = DEVICE_DT_GET(ADC_NODE);
  if (!adc_dev) {
    printk("Error getting adc failed\n");

    return false;
  }

  err = adc_channel_setup(adc_dev, &m_1st_channel_cfg);
  if (err) {
    printk("Error in adc setup: %d\n", err);

    return false;
  }

  return true;
}

#ifdef hasLidar
void lidarThread(void) {
  uint8_t dataArray[6] = {0};
  int16_t dist = 0;
  int16_t flux = 0;
  int16_t temp = 0;
  int ret;
  bool countActive = false;
  uint32_t lastPeopleCountTime = 0;
  k_sleep(K_MSEC(2000));

  if (!device_is_ready(dev_i2c.bus)) {
    printk("I2C bus %s is not ready!\n\r", dev_i2c.bus->name);
    return;
  }

  while (!cloud_connected) {
    k_sleep(K_MSEC(1000));
  }

  printk("Lidar thread started\n\r");
  lidarThreadOn = true;

  while (1) {
    if (switchOn) {
      ret = i2c_burst_read_dt(&dev_i2c, TFL_DIST_LO, dataArray,
                              sizeof(dataArray));
      if (ret != 0) {
        printk("Failed to read to I2C device");
      } else {
        dist = dataArray[0] + (dataArray[1] << 8);
        flux = dataArray[2] + (dataArray[3] << 8);
        temp = dataArray[4] + (dataArray[5] << 8);
        if (dist == 0) {
          dist = 800;
        }

        lastDist = dist;
        uint32_t nowTime = k_uptime_get_32();

        // printk("Distance: %d\n\r",dist);
        if (countActive == false) {
          if (dist <= lowThreshold && dist > 5) {
            if (nowTime - lastPeopleCountTime >= lidarInterval) {
              numPeople++;
              numPeopleDelta++;
              printk("Person counted. Total count: %d. Distance: %d\n",
                     numPeople, dist);
              countActive = true;
              lastPeopleCountTime = nowTime;
              if (debug == true) {
                printk("Sending debug info\n");
                k_work_submit(&shadow_update_version_work);
              }
            }
          }
        }

        if (countActive == true) {
          if (dist > highThreshold) {
            countActive = false;
          }
        }
      }
    }
    k_sleep(K_MSEC(10));
  }
}

K_THREAD_DEFINE(lidarThread_id, 1024, lidarThread, NULL, NULL, NULL, 10, 0, 0);
#endif

void toggle5V(struct k_work *work) {
  if (switchOn) {
#ifdef hasLidar
    printk("Lidar thread suspended\r\n");
    k_thread_suspend(lidarThread_id);
    k_sleep(K_MSEC(50));
#endif
    turn_switch_off();
    switchOn = 0;
    printk("5 volt disabled\r\n");
  } else {
    turn_switch_on();
    switchOn = 1;
    printk("5 volt enabled\r\n");
#ifdef hasLidar
    k_sleep(K_MSEC(500));
    printk("Lidar thread resumed\r\n");
    k_thread_resume(lidarThread_id);
#endif
  }

}

static void work_init(void) {
  k_work_init_delayable(&toggle5V_work, toggle5V);
  k_work_init_delayable(&toggleInterrupt_work, toggleInterrupts);
#ifdef hasTubes
  k_work_init_delayable(&pres1_work, pres1Work);
  k_work_init_delayable(&pres2_work, pres2Work);
#endif
  k_work_init_delayable(&checkLastPirEvent_work, checkLastPirEvent);
  k_work_init_delayable(&shadow_update_work, shadow_update_work_fn);
  k_work_init_delayable(&connect_work, connect_work_fn);
  k_work_init(&shadow_update_version_work, shadow_update_version_work_fn);
}

void main(void) {
  int err;

  if (!device_is_ready(wdt)) {
    printk("%s: device not ready.\n", wdt->name);
  }

  struct wdt_timeout_cfg wdt_config = {
      /* Reset SoC when watchdog timer expires. */
      .flags = WDT_FLAG_RESET_SOC,

      /* Expire watchdog after max window */
      .window.min = WDT_MIN_WINDOW,
      .window.max = WDT_MAX_WINDOW,
  };

  wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);
  err = wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
  if (err < 0) {
    printk("Watchdog setup error\n");
  }

  wdt_feed(wdt, wdt_channel_id);

  printk("Robinson software started, version: %s\n", versionNr);

  gpio_dev = DEVICE_DT_GET(GPIO_NODE);

  if (!gpio_dev) {
    printk("Error getting GPIO device binding\r\n");

    return;
  }

  cJSON_Init();
  nrf_modem_lib_dfu_handler();
  err = modem_info_init();
  if (err) {
    printk("Failed initializing modem info module, error: %d\n", err);
  }
  modem_configure();
  modem_info_string_get(MODEM_INFO_IMEI, imei, sizeof(imei));
  modem_info_string_get(MODEM_INFO_ICCID, ccid, sizeof(ccid));


  printk("Modem IMEI: %s \n", imei);
  printk("Modem CCID: %s \n", ccid);


  const struct aws_iot_config awsConfig = {
      .client_id = imei,
      .client_id_len = strlen(imei),
  };

  err = aws_iot_init(&awsConfig, aws_iot_event_handler);
  if (err) {
    printk("AWS IoT library could not be initialized, error: %d\n", err);
  }

  err = lte_lc_psm_req(true);
  if (err) {
    printk("Requesting PSM failed, error: %d\n", err);
  }  

  err = app_topics_subscribe();
  if (err) {
    printk("Adding application specific topics failed, error: %d\n", err);
  }

  k_sem_take(&lte_connected, K_FOREVER);
  modem_info_string_get(MODEM_INFO_OPERATOR, plmn, sizeof(plmn));
  printk("Operator: %s \n", plmn);
  date_time_update_async(date_time_event_handler);
  k_sem_take(&date_time_obtained, K_FOREVER);
  work_init();
  init_adc();
  init_switch();
  init_sensors();
  err = lte_lc_psm_req(true);
  if (err) {
    printk("Requesting PSM failed, error: %d\n", err);
  }  
  k_work_schedule(&connect_work, K_NO_WAIT);
}