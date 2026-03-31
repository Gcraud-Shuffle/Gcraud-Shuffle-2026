#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_bt.h"

#define SERVICE_UUID "f7fde41a-5dce-4cff-9dfd-58615717af41"
#define CHARACTERISTIC_UUID "3f6aeb51-860b-4a6d-aea9-e7df0bdc1f67"
#define RESPONSE_CHARACTERISTIC_UUID "ed7e297c-a497-68f1-af6d-c3dcbfee6a75"

BLEServer *pServer = nullptr;
BLECharacteristic *pCharacteristic = nullptr;
BLECharacteristic *pResponseCharacteristic = nullptr;
bool response_flag = false;
bool connected = false;
volatile bool timer_flag = false;
volatile uint8_t timer_count = 0;
volatile uint8_t tick_divider = 0;
uint8_t from_mcu_data;
uint8_t to_mcu_data;
uint8_t from_ble_data;
static uint32_t count = 0;

void IRAM_ATTR onTimer()
{
    timer_flag = true;
}

class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        connected = true;
    }

    void onDisconnect(BLEServer *pServer)
    {
        BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
        pAdvertising->start();
        connected = false;
    }
};

class MyCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        String value = pCharacteristic->getValue();
        response_flag = true;
        from_ble_data = value.toInt();
        timer_count = 0;
        tick_divider = 0;
    }
};

void setup()
{
    delay(500);
    HWCDCSerial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);

    BLEDevice::init("XIAO_ESP32S3_Server");
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P3);

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

    pResponseCharacteristic = pService->createCharacteristic(
        RESPONSE_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_WRITE);
    pResponseCharacteristic->setCallbacks(new MyCallbacks());

    pCharacteristic->addDescriptor(new BLE2902());
    pCharacteristic->setValue("Hello from Server");

    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->start();

    hw_timer_t *timer = timerBegin(1000000);
    timerAttachInterrupt(timer, &onTimer);
    timerAlarm(timer, 50000, true, 0);
    timerStart(timer);

    Serial1.begin(115200, SERIAL_8N1, 44, 43);
    digitalWrite(LED_BUILTIN, 0);
}

void loop()
{
    if (timer_flag)
    {
        timer_flag = false;
        tick_divider++;
        if (tick_divider >= 4) { // 50ms * 4 = 200ms
            timer_count += (timer_count < 3);
            tick_divider = 0;
        }
        HWCDCSerial.write(to_mcu_data);
        String message = String(from_mcu_data);
        pCharacteristic->setValue(message.c_str());
        pCharacteristic->notify();
    }

    if (response_flag)
    {
        response_flag = false;
    }

    if (Serial1.available() > 0)
    {
        from_mcu_data = Serial1.read() & 0x7f;
        to_mcu_data = ((timer_count==3) << 7) | from_ble_data;
        Serial1.write(to_mcu_data);
    }
}
