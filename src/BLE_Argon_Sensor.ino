/*
 * Project BLE_Argon_Sensor
 * Description:
 * Author:
 * Date:
 */

#include "Particle.h"

/*
 * All functions declaration
 *
*/

// void flowCallback(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context);

SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);
SerialLogHandler logHandler;

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
#define DATA_LEN 3

/* Data from ADC */
float sensor_1_data[DATA_LEN + 1] = {0.0, 0.0, 0.0, 0.0};

/* Data from ADC */
float sensor_temp_data[DATA_LEN + 1] = {0.0, 0.0, 0.0, 0.0};

/* Data send over BLE */
float ble_data[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

/* Counter for data */
uint8_t counter = 0;

/* battery percentage value */
uint8_t lastBattery = 100;

/* flag to transmit data */
bool data_ready = false;

/* raw data 24bits */
int raw_data_24[3];

/* counter for raw data 24bits */
uint8_t counter_raw_data_24 = 0;

/* flag for new data - raw data 24bits */
bool flag_raw_data_24 = false;

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
    // Start conversions
    Serial1.write(0b01010101);
    Serial1.write(0b00001000);
}

/*
 * Callback function to monitor disconnection from the
 * local BLE.
 */
void onDisconnect(const BlePeerDevice& peer, void* context)
{
    Log.info("Disconnected " + peer.address().toString());
    // BLEtransmit = false;
    // incomingFuncStop(buf,buf); // if it disconnects stop the test
    // // Stop conversions: POWERDOWN
    // Serial1.write(0b01010101);
    // Serial1.write(0b00000010);
}

int c = 0;

/* Thread function to read the ADC value*/
void thread_ADC_Function(void *param)
{
    /* System thread control */
    system_tick_t lastThreadTime = 0;

    /* Time interval for the thread */
    int waitTime = 1;

    unsigned long lastTime = 0;
    // Log.info("Enter thread ADC ");

    while(true)
    {
        if (Serial1.available() > 0)
        {
            WITH_LOCK(Serial)
            {
                // snprintf(buf, sizeof(buf), "From request: %.1f", (float) (millis() - lastTime_requested));
                // Log.info(buf);
                // snprintf(buf, sizeof(buf), "From last data byte: %.1f", (float) (millis() - lastTime));
                // Log.info(buf);
                // snprintf(buf, sizeof(buf), "From callback: %.1f", (float) (millis() - lastTime_cb));
                // Log.info(buf);
            }

            lastTime = millis();

            if (counter_raw_data_24 >= 3)
            {
                Log.error("Fatal Error: raw data counter bigger than 3.");
                // delay(1s);
                // counter_raw_data_24 = 0;
                digitalWrite(D7, HIGH);
                Serial1.write(0b01010101);
                Serial1.write(0b00001000);
                //  start_excited_ain3_input_ain2();
                int inByte = Serial1.read();
                Log.info("Received RAW: %d", inByte);
                continue;
            }
            else
            {
                digitalWrite(D7, LOW);
            }

            int inByte = Serial1.read();
            // Log.info("Received[%d]: %d", i++, inByte);
            raw_data_24[counter_raw_data_24++] = inByte;

            if (counter_raw_data_24 >= 3)
            {
                // Log.info("received.");
                // snprintf(buf, sizeof(buf), "From request to receive: %.1f", (float) (millis() - lastTime_requested));
                // Log.info(buf);
                // convert data to integer
                int temp_int_data = raw_data_24[2];
                temp_int_data = temp_int_data << 8;
                temp_int_data = temp_int_data | raw_data_24[1];
                temp_int_data = temp_int_data << 8;
                temp_int_data = temp_int_data | raw_data_24[0];
                if (temp_int_data > MAX_VALUE)
                {
                    temp_int_data = temp_int_data - MODULO;
                }
                float temp_data = (float) temp_int_data;
                // Log.info("Received[%d]: %.6f", c++, sensor_1_data[counter + 1]);

                if (data_rate_pair)
                {
                    sensor_1_data[counter + 1] = (sensor_1_data[counter + 1] + temp_data) / 2;
                    sensor_1_data[counter + 1] = (sensor_1_data[counter + 1] / MAX_VALUE) * 5000.0;
                    int val =  analogRead(A5);
                    sensor_temp_data[counter + 1] = ((float) val / 4096.0) * 5000.0;
                    data_rate_pair = false;
                    WITH_LOCK(Serial)
                    {
                        Log.info("Received[%d]: %.6f", ++c, sensor_1_data[counter + 1]);
                        Log.info("Temperature[%d]: %.6f", c, sensor_temp_data[counter + 1]);
                    }
                    counter++;
                    if (counter >= DATA_LEN)
                    {
                        counter = 0;
                        data_ready = true;
                    }
                }
                else
                {
                    sensor_1_data[counter + 1] = temp_data;
                    data_rate_pair = true;
                }

            }

        }
        os_thread_delay_until(&lastThreadTime, waitTime);
        // os_thread_yield();
    }
}

/* Thread function to read the ADC value*/
void thread_ADC_manager_Function(void *param)
{
    /* System thread control */
    system_tick_t lastThreadTime_m = 0;

    /* Time interval for the thread */
    int waitTime_m = 50;

    while(true)
    {
        // request_single_sensor_1();
        request_single_sensor_1_external_ref();
        lastTime_2 = millis();
        lastTime_requested = millis();
        // Log.info("requested");
        os_thread_delay_until(&lastThreadTime_m, waitTime_m);
    }
}

/* Thread function to transmit the data over BLE */
void thread_BLE_Function(void *param)
{
    /* System thread control */
    system_tick_t lastThreadTime = 0;

    /* Time interval for the thread */
    int waitTime = 10;

    unsigned long lastTime;

    while(true)
    {
        // Log.info("test");
        if (BLE.connected() && data_ready)
        {
            int i=0;
            ble_data[i] = ble_data[i] + 1.0;
            i++;
            ble_data[i] = sensor_1_data[i];
            ble_data[i+3] = sensor_temp_data[i];
            i++;
            ble_data[i] = sensor_1_data[i];
            ble_data[i+3] = sensor_temp_data[i];
            i++;
            ble_data[i] = sensor_1_data[i];
            ble_data[i+3] = sensor_temp_data[i];
            // sensor_1_data[0] = sensor_1_data[0] + 1.0;
            lastTime = millis();
            flowMeasurementCharacteristic.setValue(ble_data);
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
    flag_raw_data_24 = true;
    counter_raw_data_24 = 0;
    lastTime_cb = millis();
    // Serial1.write(0b01010101);
    // Serial1.write(0b00010000);
}

void start_single_input_ain2(void)
{
    WITH_LOCK(Serial)
    {
        // Start the ADC122U04 chip UART
        // General Reset
        Serial1.write(0b01010101);
        Serial1.write(0b00000110);  // reset
        delay(20ms);

        // Register 0
        // AINP = AIN2 and AINN = AVSS
        Serial1.write(0b01010101);
        Serial1.write(0b01000000);
        Serial1.write(0b10100000);
        delay(1ms);

        // Register 1
        // Automatic Mode 20 SPS
        Serial1.write(0b01010101);
        Serial1.write(0b01000010);
        Serial1.write(0b00001100);
        delay(1ms);

        // Register 2
        // Automatic Mode 20 SPS
        Serial1.write(0b01010101);
        Serial1.write(0b01000100);
        Serial1.write(0b00000000);
        delay(1ms);

        // Register 3
        // Automatic Transmission
        Serial1.write(0b01010101);
        Serial1.write(0b01000110);
        Serial1.write(0b00000001);
        delay(1ms);

        // Register 4
        // Enable GPIO2 as output and enable DRDY
        Serial1.write(0x55);
        Serial1.write(0x48); // write to Configuration Register 4
        Serial1.write(0x7F); // GPIO2 output mode
        delay(1ms);

        // Start conversions
        Serial1.write(0b01010101);
        Serial1.write(0b00001000);
    }
}

void start_excited_ain3_input_ain2(void)
{
    WITH_LOCK(Serial)
    {
        // Start the ADC122U04 chip UART
        // General Reset
        // Serial1.write(0b01010101);
        // Serial1.write(0b00000110);  // reset
        // delay(30ms);

        // Register 0
        // AINP = AIN2 and AINN = AVSS
        Serial1.write(0b01010101);
        Serial1.write(0b01000000);
        Serial1.write(0b10100000);
        // delay(1ms);

        // Register 1
        // Automatic Mode 20 SPS
        Serial1.write(0b01010101);
        Serial1.write(0b01000010);
        Serial1.write(0b00001000);
        // delay(1ms);

        // Register 2
        Serial1.write(0b01010101);
        Serial1.write(0b01000100);
        Serial1.write(0b00000110);
        // delay(1ms);

        // Register 3
        // Automatic Transmission
        Serial1.write(0b01010101);
        Serial1.write(0b01000110);
        Serial1.write(0b10000001);
        // delay(1ms);

        // Register 4
        // Enable GPIO2 as output and enable DRDY
        Serial1.write(0x55);
        Serial1.write(0x48); // write to Configuration Register 4
        Serial1.write(0x7F); // GPIO2 output mode
        // delay(1ms);

        // Start conversions
        Serial1.write(0b01010101);
        Serial1.write(0b00001000);
    }
}

void start_input_ain2_external_ref(void)
{
    WITH_LOCK(Serial)
    {
        // Start the ADC122U04 chip UART
        // General Reset

        // Register 0
        // AINP = AIN2 and AINN = AVSS
        Serial1.write(0b01010101);
        Serial1.write(0b01000000);
        Serial1.write(0b10100000);

        // Register 1
        // Automatic Mode 20 SPS
        // AVDD-AVSS used as reference
        Serial1.write(0b01010101);
        Serial1.write(0b01000010);
        Serial1.write(0b00001100);

        // Register 2
        Serial1.write(0b01010101);
        Serial1.write(0b01000100);
        Serial1.write(0b00000110);

        // Register 3
        // Automatic Transmission
        // IDAC disabled
        Serial1.write(0b01010101);
        Serial1.write(0b01000110);
        Serial1.write(0b00000001);

        // Register 4
        // Enable GPIO2 as output and enable DRDY
        Serial1.write(0x55);
        Serial1.write(0x48); // write to Configuration Register 4
        Serial1.write(0x7F); // GPIO2 output mode

        // Start conversions
        Serial1.write(0b01010101);
        Serial1.write(0b00001000);
    }
}

void request_single_sensor_1(void)
{
    WITH_LOCK(Serial)
    {
        // Start the ADC122U04 chip UART
        // General Reset
        // Serial1.write(0b01010101);
        // Serial1.write(0b00000110);  // reset
        // delay(30ms);

        // Register 0
        // AINP = AIN2 and AINN = AVSS
        // No Gain 
        Serial1.write(0b01010101);
        Serial1.write(0b01000000);
        Serial1.write(0b10100000);
        // delay(1ms);

        // Register 1
        // Single-shot Mode 20 SPS
        // Temperature sensor disabled
        Serial1.write(0b01010101);
        Serial1.write(0b01000010);
        Serial1.write(0b00000000);
        // delay(1ms);

        // // Register 2
        // // IDAC current 1mA
        // Serial1.write(0b01010101);
        // Serial1.write(0b01000100);
        // Serial1.write(0b00000110);
        // delay(1ms);

        // // Register 3
        // // Automatic Transmission
        // // IDAC1 connected to AIN3
        // Serial1.write(0b01010101);
        // Serial1.write(0b01000110);
        // Serial1.write(0b10000001);
        // delay(1ms);

        // // Register 4
        // // Enable GPIO2 as output and enable DRDY
        // Serial1.write(0x55);
        // Serial1.write(0x48); // write to Configuration Register 4
        // Serial1.write(0x7F); // GPIO2 output mode
        // delay(1ms);

        // Start conversion
        Serial1.write(0b01010101);
        Serial1.write(0b00001000);
    }
}

void request_single_sensor_1_external_ref(void)
{
    WITH_LOCK(Serial)
    {
        // Start the ADC122U04 chip UART
        // General Reset
        // Serial1.write(0b01010101);
        // Serial1.write(0b00000110);  // reset

        // Register 0
        // AINP = AIN2 and AINN = AVSS
        // No Gain 
        Serial1.write(0b01010101);
        Serial1.write(0b01000000);
        Serial1.write(0b10100000);

        // Register 1
        // Single-shot Mode 20 SPS
        // Temperature sensor disabled
        Serial1.write(0b01010101);
        Serial1.write(0b01000010);
        Serial1.write(0b00000110);

        // // Register 2
        // // IDAC current 1mA
        // Serial1.write(0b01010101);
        // Serial1.write(0b01000100);
        // Serial1.write(0b00000110);

        // // Register 3
        // // Automatic Transmission
        // // IDAC1 connected to AIN3
        // Serial1.write(0b01010101);
        // Serial1.write(0b01000110);
        // Serial1.write(0b10000001);

        // // Register 4
        // // Enable GPIO2 as output and enable DRDY
        // Serial1.write(0x55);
        // Serial1.write(0x48); // write to Configuration Register 4
        // Serial1.write(0x7F); // GPIO2 output mode

        // Start conversion
        Serial1.write(0b01010101);
        Serial1.write(0b00001000);
    }
}

void adc_setup(void)
{
    WITH_LOCK(Serial)
    {
        // Start the ADC122U04 chip UART
        // General Reset
        Serial1.write(0b01010101);
        Serial1.write(0b00000110);  // reset
        delay(1ms);
        // Register 2
        // IDAC current 1mA
        Serial1.write(0b01010101);
        Serial1.write(0b01000100);
        Serial1.write(0b00000000);
        // Register 3
        // Automatic Transmission
        // IDAC1 connected to AIN3
        Serial1.write(0b01010101);
        Serial1.write(0b01000110);
        Serial1.write(0b00000001);
        // Register 4
        // Enable GPIO2 as output and enable DRDY
        Serial1.write(0x55);
        Serial1.write(0x48); // write to Configuration Register 4
        Serial1.write(0x7F); // GPIO2 output mode
    }
}

// setup() runs once, when the device is first turned on.
void setup() {

    // Enable Wifi -- comment the line below to  disable wifi
    // Particle.connect();

    Serial.begin(115200);
    waitFor(Serial.isConnected, 1000);
    Serial1.begin(9600);
    // waitFor(Serial1.isConnected, 1000);
    delay(1000);

    new Thread("ADCThread", thread_ADC_Function);
    new Thread("BLEThread", thread_BLE_Function);

    pinMode(D3, INPUT_PULLUP);
    pinMode(D7, OUTPUT);
    digitalWrite(D7, LOW);

    bool result = attachInterrupt(D3, dataReadycb, FALLING);
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
    // BLE.addCharacteristic(txCharacteristic); // BLE UART Example
    // BLE.addCharacteristic(rxCharacteristic); // BLE UART Example
    batteryLevelCharacteristic.setValue(&lastBattery, 1);
	BleAdvertisingData advData;
    advData.appendLocalName("Sensor");
    // advData.appendServiceUUID(serviceUuid); // BLE UART Example
	// While we support both the flow service and the battery service, we
	// only advertise the flow sensor UUID. The battery service will be found after
	// connecting.
	advData.appendServiceUUID(flowService);

	// Continuously advertise when not connected
	BLE.advertise(&advData);
    
    // Start the ADC122U04 chip UART
    // start_excited_ain3_input_ain2();
    adc_setup();
    // start_excited_ain3_input_ain2();
    start_input_ain2_external_ref();
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
    delay(2ms);
}