#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/uart/uart_component.h"
#include "esphome/components/uart/uart_component_esp32_arduino.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#define Sharky775_MAX_RESPONSE_LENGTH 256
#define TIMESTRINGLEN 100

namespace esphome {
namespace sharky775ns {

enum Sharky775_State { Sharky775_NONE = 0, Sharky775_Wait, Sharky775_SendingAwake, Sharky775_WaitForAck, Sharky775_WaitForData
    , Sharky775_DataReceived, Sharky775_publishingData, Sharky775_SendTime, Sharky775_WaitForTimeAck
};

class Sharky775Component : public PollingComponent, public uart::UARTDevice {
public:
    float get_setup_priority() const override;

    void setup() override;
    void update() override;
    void loop() override;
    void dump_config() override;

    void set_temperature_in_sensor(sensor::Sensor* temperature_sensor) { temperature_in_sensor_ = temperature_sensor; }
    void set_temperature_out_sensor(sensor::Sensor* temperature_sensor) { temperature_out_sensor_ = temperature_sensor; }
    void set_energy_sensor(sensor::Sensor* energy_sensor) { energy_sensor_ = energy_sensor; }
    void set_energy_calc_sensor(sensor::Sensor* energy_calc_sensor) { energy_calc_sensor_ = energy_calc_sensor; }
    void set_power_sensor(sensor::Sensor* power_sensor) { power_sensor_ = power_sensor; }
    void set_flow_sensor(sensor::Sensor* flow_sensor) { flow_sensor_ = flow_sensor; }
    void set_volume_sensor(sensor::Sensor* volume_sensor) { volume_sensor_ = volume_sensor; }
    void set_ontime_sensor(sensor::Sensor* ontime_sensor) { ontime_sensor_ = ontime_sensor; }
    void set_counter_time(text_sensor::TextSensor* txt) { this->counter_time_ = txt; }
    void set_measure_time(text_sensor::TextSensor* txt) { this->measure_time_ = txt; }

    void set_noOfAwakeBytes(int noOfAwakeBytes) { noOfAwakeBytes_ = noOfAwakeBytes; }
    void set_mbusAdr(int mbusAdr) { mbusAdr_ = mbusAdr; }
    void set_timeout(int timeout) { timeout_ = timeout; }
    void set_time();

protected:
    int loopcounter;
    Sharky775_State state;
    bool Sharky775_write_command_(const uint8_t* command, uint16_t len, uart::UARTParityOptions parity = uart::UART_CONFIG_PARITY_EVEN);
    int availableForWrite();
    void SendAwakeBytes();
    void processData();
    bool publishData();
    bool readDataBlock();
    byte GetDataLenFromDif(byte dif);
    int Bcd2Int(int pos, byte len);
    bool GetTimeFromBytes(int i);
    bool GetTimeBytes(byte buf[4]);
    void calcEnergy();
    void sendTime();
    void ShowBytes(const uint8_t* bytes, uint16_t len);  // debugging

    byte awakeByte;         // was soll als awakeByte gesendet werden  (normal 0x55)  
    int noAwakeBytesToSend; // Anzahl der in diesem Zyklus noch zu sendenden Bytes
    int noOfAwakeBytes_;    // Anzahl der jeweils zu sendenden Awake Bytes
    int timeout_;
    int mbusAdr_;

    long nextCheckMillis;
    int response[Sharky775_MAX_RESPONSE_LENGTH];
    int responseLen;
    int datalen;
    int idx;
    bool setTimeOnNextConnection = false;
    int sendTimeCounter = 0;

    sensor::Sensor* temperature_in_sensor_{ nullptr };
    sensor::Sensor* temperature_out_sensor_{ nullptr };
    sensor::Sensor* energy_sensor_{ nullptr };
    sensor::Sensor* energy_calc_sensor_{ nullptr };
    sensor::Sensor* power_sensor_{ nullptr };
    sensor::Sensor* flow_sensor_{ nullptr };
    sensor::Sensor* volume_sensor_{ nullptr };
    sensor::Sensor* ontime_sensor_{ nullptr };
    text_sensor::TextSensor* counter_time_{ nullptr };
    text_sensor::TextSensor* measure_time_{ nullptr };

    double energy;
    double energyCalc = -1.0;
    float temp_in;
    float temp_out;
    float power;
    float flow;
    float volume;
    float onTime;
    char counter_time_val[TIMESTRINGLEN];
    char measure_time_val[TIMESTRINGLEN];
    long millisReceived;
    double lastEnergy = -1.0;
    double lastEnergyCalc = -1.0;
    double lastPower;
    long lastMillisReceived;
};

template<typename... Ts> class SetTimeAction : public Action<Ts...> {
public:
    SetTimeAction(Sharky775Component* sharky775) : sharky775_(sharky775) {}

    void play(Ts... x) override { this->sharky775_->set_time(); }

protected:
    Sharky775Component* sharky775_;
};

}  // namespace Sharky775
}  // namespace esphome
