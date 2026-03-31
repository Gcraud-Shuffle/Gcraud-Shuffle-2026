#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "esp_bt.h"

#define SERVICE_UUID "f7fde41a-5dce-4cff-9dfd-58615717af41"
#define CHARACTERISTIC_UUID "3f6aeb51-860b-4a6d-aea9-e7df0bdc1f67"
#define RESPONSE_CHARACTERISTIC_UUID "ed7e297c-a497-68f1-af6d-c3dcbfee6a75"

static BLEAdvertisedDevice *myDevice;
static boolean doConnect = false;
static boolean connected = false;
static BLERemoteCharacteristic *pRemoteCharacteristic;
static BLERemoteCharacteristic *pResponseCharacteristic;
static BLEClient *pClient;
bool response_flag = false;
volatile bool timer_flag = false;
volatile uint8_t timer_count = 0;
volatile uint8_t tick_divider = 0;
uint8_t from_mcu_data;
uint8_t from_ble_data;
uint8_t to_mcu_data;
static uint32_t count = 0;

void IRAM_ATTR onTimer()
{
    timer_flag = true;
}

void notifyCallback(
    BLERemoteCharacteristic *pBLERemoteCharacteristic,
    uint8_t *pData,
    size_t length,
    bool isNotify)
{

    String value = String((char *)pData).substring(0, length);
    response_flag = true;
    from_ble_data = value.toInt();
    timer_count = 0;
    tick_divider = 0;
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice advertisedDevice)
    {
        if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(BLEUUID(SERVICE_UUID)))
        {
            myDevice = new BLEAdvertisedDevice(advertisedDevice);
            doConnect = true;
            advertisedDevice.getScan()->stop();
        }
    }
};

void setup()
{
    HWCDCSerial.begin(115200);
    timer_count = 0;

    BLEDevice::init("XIAO_ESP32S3_Client");
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P3);

    BLEScan *pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);
    pBLEScan->start(30, false);

    hw_timer_t *timer = timerBegin(1000000);
    timerAttachInterrupt(timer, &onTimer);
    timerAlarm(timer, 50000, true, 0);
    timerStart(timer);

    Serial1.begin(115200, SERIAL_8N1, 44, 43);
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
    }

    if (doConnect && !connected)
    {
        pClient = BLEDevice::createClient();
        if (pClient->connect(myDevice))
        {

            BLERemoteService *pRemoteService = pClient->getService(BLEUUID(SERVICE_UUID));
            if (pRemoteService)
            {
                pRemoteCharacteristic = pRemoteService->getCharacteristic(BLEUUID(CHARACTERISTIC_UUID));
                pResponseCharacteristic = pRemoteService->getCharacteristic(BLEUUID(RESPONSE_CHARACTERISTIC_UUID));
                if (pRemoteCharacteristic)
                {
                    pRemoteCharacteristic->registerForNotify(notifyCallback);
                }
            }
            connected = true;
        }
    }

    if (connected && !pClient->isConnected())
    {
        connected = false;

        if (pClient->isConnected())
            pClient->disconnect();

        pClient = nullptr;
        pRemoteCharacteristic = nullptr;
        pResponseCharacteristic = nullptr;

        BLEDevice::deinit();

        doConnect = true;

        BLEDevice::init("XIAO_ESP32S3_Client");
        BLEDevice::getScan()->start(5, false);
    }
    else if (connected && response_flag)
    {
        String response = String(from_mcu_data);
        if (pResponseCharacteristic)
        {
            pResponseCharacteristic->writeValue(response.c_str());
        }
        response_flag = false;
    }
    else if (!connected && !doConnect)
    {
        doConnect = true;
        BLEDevice::getScan()->start(30, false);
    }
    if (Serial1.available() > 0)
    {
        from_mcu_data = Serial1.read() & 0x7f;
        to_mcu_data = ((timer_count==3) << 7) | from_ble_data;
        Serial1.write(to_mcu_data);
    }
}
