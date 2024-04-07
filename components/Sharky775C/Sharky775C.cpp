#include "Sharky775C.h"
#include "esphome/core/log.h"
#include "esphome/core/time.h"

namespace esphome {
namespace sharky775ns {

static const char* const TAG = "Sharky775";

static const int MAX_WRITE_BUFFER = 128;

static const uint16_t Sharky775_COMMAND_START_TELE_LEN = 10;
static uint8_t Sharky775_COMMAND_START_TELE[] = { 0x68, 0x04, 0x04, 0x68, 0x53, 0x00, 0x50, 0x00, 0xA3 , 0x16 };

static const uint16_t Sharky775_COMMAND_GETDATA_TELE_LEN = 5;
static const uint8_t Sharky775_COMMAND_GETDATA_TELE[] = { 0x10, 0x5B, 0x00, 0x5B, 0x16 };

static const uint16_t Sharky775_COMMAND_SETTIME_TELE_LEN = 15;
static uint8_t Sharky775_COMMAND_SETTIME_TELE[] = { 0x68, 0x09, 0x09, 0x68, 0x53, 0x00, 0x51, 0x04, 0x6D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16 };

static const int BytesToSendAtOnce = 32;   // anzahl der Bytes die auf einmal gesendet werden
static uint8_t Sharky775_AWAKE_TELE[BytesToSendAtOnce];
static const uint16_t Sharky775_COMMAND_RESPONSE_TELE_LEN = 32;


void Sharky775Component::setup() {
    ESP_LOGI(TAG, "Start Sharky775C V1.0 from https://github.com/RrPt/esphome/tree/master/components/Sharky775C");
    loopcounter = 0;
    state = Sharky775_Wait;
    awakeByte = 0x55;
    for (size_t i = 0; i < BytesToSendAtOnce; i++)
    {
        Sharky775_AWAKE_TELE[i] = awakeByte;
    }
    Sharky775_COMMAND_START_TELE[5] = mbusAdr_;
    Sharky775_COMMAND_START_TELE[8] = 0xA3 + mbusAdr_;  // CRC
}

void Sharky775Component::update() {
    noAwakeBytesToSend = noOfAwakeBytes_; // 480;
    nextCheckMillis = 0;
    state = Sharky775_SendingAwake;
    ESP_LOGI(TAG, "%dms: Start Awake", millis());
}

void Sharky775Component::loop() {
    int anz =this->available();
    switch (state)
    {
    case esphome::sharky775ns::Sharky775_SendingAwake:
        if (noAwakeBytesToSend > 0)
        {
            SendAwakeBytes();
        }
        else  if (millis() < nextCheckMillis)
        {
            // Warten bis vollständig gesendet
            //int free = availableForWrite();

        }
        else if (availableForWrite()<128)
        { 
            // Noch nicht alle gesendet, warten bis rest gesendet + 10ms (laut Protokoll)  in bisheriger Lösung tut es mit 250ms
            nextCheckMillis = millis() + (MAX_WRITE_BUFFER - availableForWrite()) * 5 + 10+ 200;  // 5 kommt von 11 bit / 2400 Baud * 1000ms/s
            ESP_LOGV(TAG, "%dms: %d Bytes in sendbuffer wait until %dms", millis(), MAX_WRITE_BUFFER - availableForWrite(),nextCheckMillis);
        }
        else
        {
            ESP_LOGD(TAG, "%dms: Awake sent %d Bytes", millis(), noOfAwakeBytes_);
            ESP_LOGD(TAG, "%dms: Sending Sharky775_COMMAND_START_TELE", millis());
            if (!this->Sharky775_write_command_(Sharky775_COMMAND_START_TELE, Sharky775_COMMAND_START_TELE_LEN)) {
                ESP_LOGW(TAG, "Sending Sharky775_COMMAND_START_TELE failed!");
                this->status_set_warning();
                return;
            }
            ESP_LOGD(TAG, "%dms: Sent Sharky775_COMMAND_START_TELE", millis());
            nextCheckMillis = millis() + timeout_; // Timeout
            state = Sharky775_WaitForAck;
            //this->flush();  // Rx und Tx Puffer leeren
            ESP_LOGD(TAG, "%dms: wait for Ack", millis());
        }
        break;
    case esphome::sharky775ns::Sharky775_WaitForAck:
        if (anz > 0)
        {
            byte inByte = this->read();
            if ( inByte != 0xE5)  { 
                ESP_LOGW(TAG, "%dms: No ACK received after Sharky775_COMMAND_START_TELE but got Byte 0x%02X",millis(),inByte);
                this->status_set_warning();
                state = Sharky775_Wait;  // abbrechen
                ESP_LOGW(TAG, "%dms: aborting cycle", millis());
                return;
            };
            ESP_LOGD(TAG, "%dms: ACK received after Sharky775_COMMAND_START_TELE",millis());
            ESP_LOGD(TAG, "%dms: sending Sharky775_COMMAND_GETDATA_TELE", millis());
            if (!this->Sharky775_write_command_(Sharky775_COMMAND_GETDATA_TELE, Sharky775_COMMAND_GETDATA_TELE_LEN)) {
                ESP_LOGW(TAG, "%dms: Sending Sharky775_COMMAND_GETDATA_TELE failed!",millis());
                this->status_set_warning();
                ESP_LOGW(TAG, "%dms: aborting cycle", millis());
                return;
            }
            ESP_LOGD(TAG, "%dms: Sent Sharky775_COMMAND_GETDATA_TELE", millis());
            nextCheckMillis = millis() + timeout_; // 5s Timeout
            state = Sharky775_WaitForData;
            responseLen = 0;
            datalen = 0;
            this->flush();  // Rx und Tx Puffer leeren
            ESP_LOGD(TAG, "%dms: wait for Data", millis());
        }
        else if (millis() > nextCheckMillis)
        { // Kein Ack empfangen
            ESP_LOGW(TAG, "%dms: no Ack receved after %d ms",millis(), timeout_);
            this->status_set_warning();
            state = Sharky775_Wait;  // abbrechen
            ESP_LOGW(TAG, "%dms: aborting cycle", millis());
        }
/*        else
        {
            ESP_LOGD(TAG, "%dms: still waiting for Ack", millis());
        }
*/
        break;
    case esphome::sharky775ns::Sharky775_WaitForData:
        if (anz > 0)
        {
            ESP_LOGV(TAG, "%dms: %d Bytes received, data receved=%d", millis(), anz, responseLen+anz);
            while (this->available())
            {
                int data = this->read();
                response[responseLen++] = data;
                if (responseLen > Sharky775_MAX_RESPONSE_LENGTH) {
                    ESP_LOGW(TAG, "Empfangspuffer mit %d Bytes zu klein",responseLen);
                    this->status_set_warning();
                    return;
                }
                if (responseLen == 2)
                {
                    datalen = data;
                    ESP_LOGV(TAG, "%dms: datalen = %02X",millis(), data);
                }
                //ESP_LOGD(TAG, "%dms: %d. Byte empfangen: %02X",millis(),responselen, data);
             }
        }
        else if (responseLen >= datalen + 6)
        {
            ESP_LOGD(TAG, "%dms: Data received", millis());
            state = Sharky775_DataReceived; 
        }
        else if (millis() > nextCheckMillis)
        { // Keine Daten empfangen
            ESP_LOGW(TAG, "%dms: no data receved after %d ms", millis(), timeout_);
            this->status_set_warning();
            state = Sharky775_Wait;  // abbrechen
            ESP_LOGW(TAG, "%dms: aborting cycle", millis());
        }
        //else
        //{
        //    ESP_LOGD(TAG, "%dms: responselen=%d datalen=%d", millis(), responseLen,datalen);
        //}

        break;
    case esphome::sharky775ns::Sharky775_DataReceived:
        ESP_LOGD(TAG, "%dms: processing received data", millis());
        //ESP_LOGD(TAG, "%dms: Data starting: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X", millis(), response[0], response[1], response[2], response[3], response[4]);
        //ESP_LOGD(TAG, "%dms: Data Ending:  0x%02X 0x%02X 0x%02X 0x%02X 0x%02X", millis(), response[responseLen - 5], response[responseLen - 4], response[responseLen - 3], response[responseLen - 2], response[responseLen-1]);
        processData();
        idx = 0;
        state = Sharky775_publishingData;  // noch senden
        break;
    case esphome::sharky775ns::Sharky775_publishingData:
        if (publishData())
        {   // Zyklusende?
            if (setTimeOnNextConnection) state = Sharky775_SendTime;
            else  state = Sharky775_Wait;  // Zyklusende keine Zeit zu setzen
        }
        break;
    case esphome::sharky775ns::Sharky775_SendTime:
        if (sendTimeCounter > 5)
        {
            ESP_LOGE(TAG, "%dms: sentTime tried %d times, aborting trying to set time", millis(), sendTimeCounter);
            setTimeOnNextConnection = false;
            sendTimeCounter = 0;
            this->status_set_warning("Time could not set");
            state = Sharky775_Wait;  // abbrechen
            ESP_LOGW(TAG, "%dms: aborting cycle", millis());
            return;
        }
        sendTimeCounter++;
        sendTime();
        nextCheckMillis = millis() + timeout_; // 5s Timeout
        state = Sharky775_WaitForTimeAck;
        break;
    case esphome::sharky775ns::Sharky775_WaitForTimeAck:
        if (anz > 0)
        {
            byte inByte = this->read();
            if (inByte != 0xE5) {
                ESP_LOGW(TAG, "%dms: No ACK received after Sharky775_SendTime but got Byte 0x%02X", millis(), inByte);
                this->status_set_warning("No ACK received");
                state = Sharky775_Wait;  // abbrechen
                ESP_LOGW(TAG, "%dms: aborting cycle", millis());
                return;
            };
            ESP_LOGD(TAG, "%dms: ACK received after Sharky775_SendTime", millis());
            setTimeOnNextConnection = false;
            sendTimeCounter = 0;
            state = Sharky775_Wait;  // Zyklusende, zeit erfolgreich gesetzt
            this->flush();  // Rx und Tx Puffer leeren
        }
        else if (millis() > nextCheckMillis)
        { // Kein Ack empfangen
            ESP_LOGW(TAG, "%dms: no Ack receved after %d ms", millis(), timeout_);
            this->status_set_warning();
            state = Sharky775_Wait;  // abbrechen
            ESP_LOGW(TAG, "%dms: aborting cycle", millis());
        }
        break;
    case esphome::sharky775ns::Sharky775_Wait:
        break;
    case esphome::sharky775ns::Sharky775_NONE:
        ESP_LOGD(TAG, "%dms: impossible state NONE detected", millis());
        break;
    default:
        break;
    }
    loopcounter++;
}

//
// Read Data
//
void Sharky775Component::processData() {
    int t1 = millis();
    ESP_LOGV(TAG, "%dms: Start processData", millis());

    if (response[0] != 0x68 || response[3] != 0x68 || response[1] != response[2]) {
        ESP_LOGW(TAG, "%dms: Invalid preamble from Sharky775!", millis);
        this->status_set_warning();
        return;
    }

    uint8_t checksum = 0;
    for (size_t i = 4; i < responseLen - 2; i++)
    {
        checksum += response[i];
    }
    if (response[responseLen - 2] != checksum) {
        ESP_LOGW(TAG, "%dms: Checksum doesn't match: 0x%02X!=0x%02X", millis(), response[responseLen - 2], checksum);
        this->status_set_warning();
        return;
    }
    else ESP_LOGV(TAG, "%dms: Checksum ok", millis());

    millisReceived = millis();

    // Daten interpretieren
    idx = 19;
    int t2 = millis();
    while (readDataBlock());
    int t3 = millis();

    calcEnergy();


    this->status_clear_warning();

    int t4 = millis();

    ESP_LOGV(TAG, "%dms: berechenzeit= t1->t2 %d ms", millis(), t2 - t1);
    ESP_LOGV(TAG, "%dms: berechenzeit= t2->t3 %d ms", millis(), t3 - t2);
    ESP_LOGV(TAG, "%dms: berechenzeit= t3->t4 %d ms", millis(), t4 - t3);

    ESP_LOGV(TAG, "%dms: Received Power=%fW Temp_in=%f°C Temp_out=%f°C Energy=%fkWh Status=?",millis(), power, temp_in, temp_out, energy);
}

bool Sharky775Component::readDataBlock()
{
    if (idx >= 4 + response[1]) return false;  // Ende erreicht
    int i = idx;
    // DIF
    byte dif = response[i];
    byte dataLen = GetDataLenFromDif(dif);
    while (response[i] >= 0x80) i++;  // DIFE überspringen
    i++;
    // VIF
    byte vif = response[i];
    while (response[i] >= 0x80) i++;  // VIFE überspringen
    i++;
    int val = Bcd2Int(i, dataLen);

    ESP_LOGV(TAG, "%dms: IDX=: %d  val=%f", millis(), idx, val);

    switch (idx)
    {
    case 19:
        energy = val;  //* 1e-3;
        ESP_LOGV(TAG, "%dms: energy = %f kWh", millis(), energy);
        break;    // kWh
    case 39:
        volume = val * 1e-3;
        ESP_LOGV(TAG, "%dms: volume = %f m3", millis(), volume);
        break;  // L -> m3
    case 45:
        power = val;
        ESP_LOGV(TAG, "%dms: power = %f W", millis(), power);
        break;   // W 
    case 51:
        flow = val * 1e-3;
        ESP_LOGV(TAG, "%dms: flow = %f m3/h", millis(), flow);
        break; // l/h -> m3/h
    case 56:
        temp_in = val * 0.1;
        ESP_LOGV(TAG, "%dms: temp_in = %f °C", millis(), temp_in);
        break;    // d°C -> °C
    case 60:
        temp_out = val * 0.1;
        ESP_LOGV(TAG, "%dms: temp_out = %f °C", millis(), temp_out);
        break;   // d°C -> °C
    case 68:
        onTime = val;
        ESP_LOGV(TAG, "%dms: onTime = %f h", millis(), onTime);
        break;          // Stunden
    case 73:
        //timeCaptured = 
        GetTimeFromBytes(i);
        break;  // Datum + Zeit  Typ F 
    default:
        break;
    }
    idx = i + dataLen;
    return true;
}

bool Sharky775Component::Sharky775_write_command_(const uint8_t* command, uint16_t len, uart::UARTParityOptions parity) {
    //
    // parity umschalten dauert zu lange, deshalb immer mit gleicher parity (even) senden, das AWAKE kommt damit klar
    // 
    //uart::UARTParityOptions old_parity = this->parent_->get_parity();
    //if (parity != old_parity)
    //{
    //    this->parent_->set_parity(parity);
    //    this->parent_->load_settings();
    //}
    this->write_array(command, len);
    //if (parity != old_parity)
    //{
    //    this->parent_->set_parity(old_parity); // parity wieder zurücksetzen
    //    this->parent_->load_settings();
    //}
    ESP_LOGV(TAG, "%dms: Sent Tele mit %d Bytes: 0x%02X 0x%02X 0x%02X", millis(), len, command[0], command[1], command[2]);
    return true;
}

void Sharky775Component::SendAwakeBytes() {
    if (millis() < nextCheckMillis) return;  // noch nicht senden, warten bis Buffer leerer ist
    if (availableForWrite() >= BytesToSendAtOnce)
    {   // Es ist genug Platz im Schreibpuffer
        int bytesToSend = BytesToSendAtOnce;
        if (noAwakeBytesToSend < BytesToSendAtOnce) bytesToSend = noAwakeBytesToSend;  // evtl. müssen gar ncith mehr alle BytesToSendAtOnce gesendet werden

        Sharky775_write_command_(Sharky775_AWAKE_TELE, bytesToSend, uart::UART_CONFIG_PARITY_NONE);
        noAwakeBytesToSend -= bytesToSend;
        nextCheckMillis = millis() + (BytesToSendAtOnce - 4) * 10000 / 2400;
        int free = availableForWrite();

        ESP_LOGV(TAG, "%dms: %d Awakebytes sent. remaining %d nextCheckMillis=%dms free=%d", millis(), bytesToSend, noAwakeBytesToSend, nextCheckMillis, free);
    }
}

void Sharky775Component::set_time()
{
    setTimeOnNextConnection = true;
    ESP_LOGD(TAG, "set time on next connection");
}

void Sharky775Component::sendTime()
{
    ESP_LOGD(TAG, "%dms: sendTime() Start try no %d", millis(), sendTimeCounter);
    ESP_LOGD(TAG, "%dms: last CountertimeStr: %s", millis(), counter_time_val);
    ESP_LOGD(TAG, "%dms: last MeasuretimeStr: %s", millis(), measure_time_val);

    time_t currTime = time(NULL);
    char actual_time_val[TIMESTRINGLEN];
    strftime(actual_time_val, sizeof(measure_time_val), "%Y-%m-%dT%H:%M:%Sz", gmtime(&currTime));
    ESP_LOGD(TAG, "%dms: last ActualtimeStr: %s", millis(), actual_time_val);

    byte b[4];
    if (GetTimeBytes(b))
    {
        ESP_LOGD(TAG, "sendTime: actual bytes: 0x%02X 0x%02X 0x%02X 0x%02X ", b[0], b[1], b[2], b[3]);
        // create telegram
        Sharky775_COMMAND_SETTIME_TELE[5] = mbusAdr_;
        Sharky775_COMMAND_SETTIME_TELE[13] = 0x15 + mbusAdr_;  // init CRC
        for (size_t i = 0; i < 4; i++)
        {
            Sharky775_COMMAND_SETTIME_TELE[i + 9] = b[i];
            Sharky775_COMMAND_SETTIME_TELE[13] += b[i];  // CRC
        }
        //this->ShowBytes(Sharky775_COMMAND_SETTIME_TELE, Sharky775_COMMAND_SETTIME_TELE_LEN);  // nur Debug
        if (!this->Sharky775_write_command_(Sharky775_COMMAND_SETTIME_TELE, Sharky775_COMMAND_SETTIME_TELE_LEN)) {
            ESP_LOGW(TAG, "Sending Sharky775_COMMAND_SETTIME_TELE failed!");
            this->status_set_warning();
            return;
        }
    }
    else
    {
        ESP_LOGD(TAG, "time still invalid");
    }
    ESP_LOGD(TAG, "%dms: sendTime() End", millis());

}


// publish dauert lange, deshalb in jeder loop-Schleife nur eines publishen
bool Sharky775Component::publishData() {
      ESP_LOGV(TAG, "%dms: publishData Anfang  idx=%d", millis(), idx);
      if (idx == 0)      { if ((this->energy_sensor_ != nullptr) & (energy > 0.0))  this->energy_sensor_->publish_state(energy); }
      else if (idx == 1) { if ((this->energy_calc_sensor_ != nullptr)&(energyCalc>0.0)) this->energy_calc_sensor_->publish_state(energyCalc); }
      else if (idx == 2) { if (this->temperature_in_sensor_ != nullptr)  this->temperature_in_sensor_->publish_state(temp_in); }
      else if (idx == 3) { if (this->temperature_out_sensor_ != nullptr)    this->temperature_out_sensor_->publish_state(temp_out); }
      else if (idx == 4) { if (this->power_sensor_ != nullptr)    this->power_sensor_->publish_state(power); }
      else if (idx == 5) { if (this->flow_sensor_ != nullptr)     this->flow_sensor_->publish_state(flow); }
      else if (idx == 6) { if (this->volume_sensor_ != nullptr)       this->volume_sensor_->publish_state(volume); }
      else if (idx == 7) { if (this->ontime_sensor_ != nullptr)      this->ontime_sensor_->publish_state(onTime); }
      else if (idx == 8) { if (this->counter_time_ != nullptr)      this->counter_time_->publish_state(counter_time_val); }
      else if (idx == 9) { if (this->measure_time_ != nullptr)      this->measure_time_->publish_state(measure_time_val); }
      else
      {
          ESP_LOGI(TAG, "%dms: End publishData", millis());
          return true;
      }
      idx++;
      return false;  // noch nicht alles veröffentlicht
}

void Sharky775Component::calcEnergy()
{
    if (lastEnergy < 0)
    {   // noch keine lastEnergy vorhanden
        ESP_LOGW(TAG, "%dms: lastEnergy<0 %f", millis(),lastEnergy);
        lastEnergy = energy;
        return;
    }
    if (lastEnergyCalc < 0)
    {   // noch keine lastEnergyCalc vorhanden
        ESP_LOGW(TAG, "%dms: lastEnergyCalc<0 %f", millis(), lastEnergyCalc);
        lastEnergyCalc = lastEnergy;
        return;
    }

    energyCalc = lastEnergyCalc;    // auf letzten Wert setzen

    // nun berechnen
    double pow = (power + lastPower) / 2;  // in W
    double dauer = (millisReceived - lastMillisReceived)/1000.0;  // in s
    double deltaEnergy = pow * dauer / 3600000; 
    ESP_LOGV(TAG, "%dms: energy_calc vor = %f kWh ( %fkWh)", millis(), energyCalc, deltaEnergy);
    energyCalc += deltaEnergy;
    ESP_LOGV(TAG, "%dms: energy_calc nach= %f kWh (%fkWh)", millis(), energyCalc, deltaEnergy);

    // mit der gemessenen abgelichen, damit es nicht auseinanderläuft
    // der berechnete darf nicht größer sein als der nächste gemessene
    if (energyCalc > energy + 1.0) energyCalc = energy + 1.0;
    // und nicht kleiner als der letzte gemessene
    if (energyCalc < energy) energyCalc = energy;

    // wenn sich der gemessene Wert ändert, dann den berechneten darauf setzen
    if (energy > lastEnergy) energyCalc = energy;

    ESP_LOGD(TAG, "%dms: energy_calc = %f kWh (%fW * %fs -> %fkWh)", millis(), energyCalc, pow, dauer, deltaEnergy);

    lastEnergy = energy;
    lastEnergyCalc = energyCalc;
    lastPower = power;
    lastMillisReceived = millisReceived;
}

//
// Hilfsfunktionen
//
byte Sharky775Component::GetDataLenFromDif(byte dif)
{
    switch (dif & 0x7)
    {
    case 0: return 0;
    case 1: return 1;
    case 2: return 2;
    case 3: return 3;
    case 4: return 4;
    case 5: return 4;
    case 6: return 6;
    case 7: return 8;
    default: return 0;
    }
}

int Sharky775Component::Bcd2Int(int pos, byte len)
{
    int erg = 0;
    for (int i = pos + len - 1; i >= pos; i--)
    {
        erg *= 100;
        erg += (10 * (response[i] >> 4));
        erg += response[i] & 0xf;
    }
    return erg;
}

bool Sharky775Component::GetTimeFromBytes(int i)
{
    ESP_LOGD(TAG, "Measuretime bytes: 0x%02X 0x%02X 0x%02X 0x%02X ", response[i], response[i + 1], response[i + 2], response[i + 3]);
    int min = response[i] & 0x3F;
    int hour = response[i + 1] & 0x1F;
    int day = response[i + 2] & 0x1F;
    int month = response[i + 3] & 0x0F;
    int year = 2000 + ((response[i + 3] & 0xF0) >> 1) | ((response[i + 2] & 0xE0) >> 5);
    snprintf(counter_time_val,100, "%04d-%02d-%02dT%02d:%02d", year, month, day, hour, min);
    ESP_LOGV(TAG, "%dms: CountertimeStr: %s", millis(), counter_time_val);

    time_t currTime =  time(NULL);
    strftime(measure_time_val, sizeof(measure_time_val), "%Y-%m-%dT%H:%M:%Sz",gmtime(&currTime));
    ESP_LOGV(TAG, "%dms: MeasuretimeStr: %s", millis(), measure_time_val);

    return true;
}

bool Sharky775Component::GetTimeBytes(byte buf[4])
{
    //if (!time(NULL).now().is_valid()) return false;
    time_t currTime = time(NULL);
    char actual_time_val[TIMESTRINGLEN];
    strftime(actual_time_val, sizeof(measure_time_val), "%Y-%m-%dT%H:%M:%Sz", gmtime(&currTime));
    ESP_LOGD(TAG, "%dms: actualtimeStr: %s", millis(), actual_time_val);

    int y = gmtime(&currTime)->tm_year+1900-2000;
    if (gmtime(&currTime)->tm_year < 124) return false;
    int mo = gmtime(&currTime)->tm_mon+1;
    int d = gmtime(&currTime)->tm_mday;
    int h = gmtime(&currTime)->tm_hour;
    int mi = gmtime(&currTime)->tm_min;
    int s = gmtime(&currTime)->tm_sec;
    ESP_LOGD(TAG, "aktuelle Zeit: %d-%d-%d %d:%d:%dz", y,mo,d,h, mi,s);

    buf[0] = mi;
    buf[1] = h;
    buf[2] = d;
    buf[3] = mo;

    byte y2 = (byte)(y % 8);
    byte y3 = (byte)(y / 8);
    buf[2] |= (byte)(y2 << 5);
    buf[3] |= (byte)(y3 << 4);

    return true;
}

void Sharky775Component::ShowBytes(const uint8_t* bytes, uint16_t len) {
    char str[255];
    for (size_t i = 0; i < len; i++)
    {
        ESP_LOGD(TAG, "b[%d]=%02X", i, bytes[i]);
    }
}

int Sharky775Component::availableForWrite()
{
    esphome::uart::ESP32ArduinoUARTComponent* espUC = (esphome::uart::ESP32ArduinoUARTComponent*)this->parent_;
    HardwareSerial* hws = espUC->get_hw_serial();
    int free = hws->availableForWrite();
    return free;
}



float Sharky775Component::get_setup_priority() const { return setup_priority::DATA; }

void Sharky775Component::dump_config() {
  ESP_LOGCONFIG(TAG, "Sharky775:");
  LOG_SENSOR("  ", "Energy", this->energy_sensor_);
  LOG_SENSOR("  ", "Power", this->power_sensor_);
  //ESP_LOCCONFIG(TAG,"  max: %f",max);
  LOG_SENSOR("  ", "Volume", this->volume_sensor_);
  LOG_SENSOR("  ", "Flow", this->flow_sensor_);
  LOG_SENSOR("  ", "Temperature_in", this->temperature_in_sensor_);
  LOG_SENSOR("  ", "Temperature_out", this->temperature_out_sensor_);
  this->check_uart_settings(2400,1, uart::UART_CONFIG_PARITY_EVEN,8);
  ESP_LOGCONFIG(TAG, "  mbusAdr_: %d", mbusAdr_);
  ESP_LOGCONFIG(TAG, "  noOfAwakeBytes_: %d", noOfAwakeBytes_);
  if (noOfAwakeBytes_ < 0) ESP_LOGE(TAG, "noOfAwakeBytes = %d must be >= 0", noOfAwakeBytes_);
  if (noOfAwakeBytes_ > 1000) ESP_LOGE(TAG, "noOfAwakeBytes = %d must be <= 1000", noOfAwakeBytes_);
  ESP_LOGCONFIG(TAG, "  timeout_: %d", timeout_);
  if (timeout_ < 100) ESP_LOGE(TAG, "timeout = %d must be >= 100", timeout_);
  if (timeout_ > 60000) ESP_LOGE(TAG, "timeout = %d must be <= 60000", timeout_);
}

}  // namespace Sharky775
}  // namespace esphome
