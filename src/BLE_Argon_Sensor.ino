/*
 * Project BLE_Argon_Sensor
 * Description:
 * Author:
 * Date:
 */

#include "Particle.h"
#include <Wire.h>
#include <ADS1219/ADS1219.h>
// const uint8_t address = 0b01000000;
// #define rst 14
#define drdy D2

SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);
SerialLogHandler logHandler;
ADS1219 ads(drdy, 0x40);

/*
    Flow Sensor UUID
    a7ea14cf-val-43ba-ab86-1d6e136a2e9e
    Service val = 1000
    Characteristic val = 1100
*/
BleUuid flowService("a7ea14cf-1000-43ba-ab86-1d6e136a2e9e");
BleUuid flowCharacteristic("a7ea14cf-1100-43ba-ab86-1d6e136a2e9e");
// BleCharacteristicProperty::NOTIFY
// BleCharacteristicProperty::READ
// BleCharacteristicProperty::INDICATE
BleCharacteristic flowMeasurementCharacteristic("flow", BleCharacteristicProperty::READ | BleCharacteristicProperty::NOTIFY, flowCharacteristic, flowService);

// The battery level service allows the battery level to be monitored
BleUuid batteryLevelService(BLE_SIG_UUID_BATTERY_SVC);
// The battery_level characteristic shows the battery level of
BleCharacteristic batteryLevelCharacteristic("bat", BleCharacteristicProperty::NOTIFY, BleUuid(0x2A19), batteryLevelService);

/* Generic buffer */
char buf[300];

/* BLE data length */
#define DATA_LEN 4

/* Data from ADC */
float sensor_1_data[DATA_LEN] = {0.0, 0.0, 0.0, 0.0};

/* Data send over BLE */
float ble_data[DATA_LEN + 1] = {0.0, 0.0, 0.0, 0.0};

/* Counter for data */
uint8_t counter = 0;

/* battery percentage value */
uint8_t lastBattery = 100;

/* flag to transmit data */
bool data_ready = false;

/* raw data 24bits */
int raw_data_24[3];

// /* counter for raw data 24bits */
// uint8_t counter_raw_data_24 = 0;

/* flag for new data - raw data 24bits */
bool flag_raw_data = false;

/* Twos complement 24 bits modulo */
const int MODULO = 1 << 24;

/* Twos complement 24 bits max positive value */
const int MAX_VALUE = ((1 << 23) - 1);

/* flag to check for the callback in the main loop */
bool cb = false;

/* flag to decimate the data */
bool data_rate_pair = false;

/* time counter */
unsigned long lastTime_2;

/* time counter */
unsigned long lastTime_cb;

/* time counter */
unsigned long lastTime_requested;

/* enumetator for chip status */
enum chip_status
{
    idle,
    powerdown,
    requested_temperature,
    requested_sensor_1,
    requested_sensor_2,
    requested_register_0,
    requested_register_1,
    requested_register_2,
    requested_register_3,
    requested_register_4
};

/* enumetator for chip status */
chip_status ADS122_status = powerdown;

/*
 * Callback function to monitor connection from the
 * local BLE .
 */
void onConnect(const BlePeerDevice &peer, void* context)
{
    Log.info("Connected " + peer.address().toString());
}

/*
 * Callback function to monitor disconnection from the
 * local BLE.
 */
void onDisconnect(const BlePeerDevice& peer, void* context)
{
    Log.info("Disconnected " + peer.address().toString());
}

int c = 0;

/* Thread function to read the ADC value*/
void thread_ADC_Function(void *param)
{
    /* System thread control */
    system_tick_t lastThreadTime = 0;

    /* Time interval for the thread */
    int waitTime = 1;

    while(true)
    {
        if (flag_raw_data)
        {
            flag_raw_data = false;
            long val = ads.continuousRead();
            float value = (float) val * 2048.0 / (float) MAX_VALUE;
            if (data_rate_pair)
            {
                sensor_1_data[counter] = value;
                data_rate_pair = false;
                WITH_LOCK(Serial)
                {
                    Log.info("Received[%d]: %.6f", c++, sensor_1_data[counter]);
                }
                counter++;
                if (counter >= DATA_LEN)
                {
                    counter = 1;
                    data_ready = true;
                }
            }
            else
            {
                sensor_1_data[counter] = value;
                data_rate_pair = true;
            }

        }

        os_thread_delay_until(&lastThreadTime, waitTime);
        // os_thread_yield();
    }
}

/* Thread function to transmit the data over BLE */
void thread_BLE_Function(void *param)
{
    /* System thread control */
    system_tick_t lastThreadTime = 0;

    /* Time interval for the thread */
    int waitTime = 10;

    // unsigned long lastTime;

    while(true)
    {
        // Log.info("test");
        if (BLE.connected() && data_ready)
        {
            sensor_1_data[0] = sensor_1_data[0] + 1.0;
            // lastTime = millis();
            flowMeasurementCharacteristic.setValue(sensor_1_data);
            // Log.info("sent data");
            // snprintf(buf, sizeof(buf), "BLE update time: %.1f", (float) (millis() - lastTime));
            // Log.info(buf);
            data_ready = false;
        }
        os_thread_delay_until(&lastThreadTime, waitTime);
    }
}

void dataReadycb()
{
    cb = true;
    flag_raw_data = true;
    lastTime_cb = millis();
}

void adc_setup(void)
{
    ads.begin();
    ads.setGain(ONE);
    ads.setDataRate(20);
    ads.setVoltageReference(REF_INTERNAL);
    ads.readSingleEnded(2);
    ads.setConversionMode(CONTINUOUS);
}

// setup() runs once, when the device is first turned on.
void setup() {

    // Enable Wifi -- comment the line below to  disable wifi
    // Particle.connect();

    Serial.begin(115200);
    waitFor(Serial.isConnected, 1000);

    new Thread("ADCThread", thread_ADC_Function);
    new Thread("BLEThread", thread_BLE_Function);

    pinMode(D2, INPUT_PULLUP);
    pinMode(D3, OUTPUT);
    pinMode(D7, OUTPUT);
    digitalWrite(D3, HIGH);
    digitalWrite(D7, LOW);

    bool result = attachInterrupt(D2, dataReadycb, FALLING);
    if (result)
    {
        WITH_LOCK(Serial)
        {
            Log.info("Interrupt config OK");
        }
    } else
    {
        WITH_LOCK(Serial)
        {
            Log.info("Fail to config interrupt");
        }   
    }
    interrupts();   // enable interrupts

    // Enable and config BLE
    BLE.setPPCP(12, 12, 0, 200);
    BLE.setTxPower(8);
    BLE.onConnected(onConnect, NULL);
    BLE.onDisconnected(onDisconnect, NULL);
    BLE.on();
    BLE.addCharacteristic(flowMeasurementCharacteristic);
    BLE.addCharacteristic(batteryLevelCharacteristic);
    batteryLevelCharacteristic.setValue(&lastBattery, 1);
	BleAdvertisingData advData;
    advData.appendLocalName("Sensor B");
	// While we support both the flow service and the battery service, we
	// only advertise the flow sensor UUID. The battery service will be found after
	// connecting.
	advData.appendServiceUUID(flowService);

	// Continuously advertise when not connected
	BLE.advertise(&advData);
    
    adc_setup();
    // new Thread("ADCManagerThread", thread_ADC_manager_Function);
}

int last_counter = 0;

void loop() {
    if (cb)
    {
        cb=false;
        last_counter = c;
        // Log.info("cb ");
        // snprintf(buf, sizeof(buf), "loop Time: %.1f", (float) (lastTime_cb - lastTime_2));
        // WITH_LOCK(Serial)
        // {
        //     Log.info(buf);
        // }
    }
    // Serial.println("Single ended result:");
    // Serial.println(ads.readSingleEnded(2)*2.048/pow(2,23),5);
    // delay(2000);
    // Serial.println("Differential result:");
    // Serial.println(ads.readDifferential_2_3()*2.048/pow(2,23),5);
    // delay(2000);
    // Log.info("test %d", c++);
    delay(1s);
}