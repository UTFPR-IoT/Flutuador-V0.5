
#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"
#include "Adafruit_ADS1X15.h"
#include "Adafruit_BusIO_Register.h"
#include "Adafruit_BMP280.h"
#include "RTClib.h" 
#include "SPI.h" 
#include "FS.h"
#include "SD.h"
#include "DFRobot_ESP_PH_BY_GREENPONIK-master/src/DFRobot_ESP_PH.h"
#include "atlas_gravity/do_grav.h"
#include "EEPROM.h"
#include "DallasTemperature.h"
#include "lmic.h"
#include "hal/hal.h"
#include "freertos/FreeRTOS.h"

// DEFINICAO TRANSMISSAO TTN OU SOMENTE REGISTRO?
#define TRANSMITIRDADOS
#define SOMENTEREGISTRO

// ESTADOS
enum maq_estados {AQUECIMENTO, LEITURADADOS, TRANSMISSAO};
maq_estados estado = AQUECIMENTO;
// TEMPOS DE CADA ESTADO DA MAQUINA DE ESTADOS
#define TEMPOAQUECIMENTO 5
#define TEMPOLEITURADADOS 10
#define TEMPOTRANSMISSAO 300
#define TEMPODEEPSLEEP 15
#define TEMPOACORDADO  20



TaskHandle_t xTimer1;
TaskHandle_t xTimer2;
TaskHandle_t xTimer3;
void callBackTimer(TimerHandle_t xTimer);

//SENSORES
#define ADDRTemperatura     26
#define VoltageAVGLen      100
#define ADDRTurbidez         0
#define ADDRpH               1
#define ADDRSensorOD         2
#define INTERVALOREGISTRO 1500
// TURBIDEZ
#define TensaoMaxTurbidez 2.451

// BARRAMENTO I2C
#define I2C_SDA 21
#define I2C_SCL 22

// Cartao SD pinout
#define  SD_CS      5
#define  SD_MOSI    23
#define  SD_MISO    19
#define  SD_CLK     18

// RFM95 pinout
#define  RFM95_CS      27
#define  RFM95_MOSI    23
#define  RFM95_MISO    19
#define  RFM95_CLK     18
#define  RFM95_RST     14
#define  RFM95_DIO0    2
#define  RFM95_DIO1    4
#define  RFM95_DIO2    15

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0x81, 0xB5, 0x05, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0x5A, 0x1D, 0x0B, 0x37, 0x6B, 0x7A, 0x7F, 0x31, 0xA2, 0x6E, 0x1B, 0x7C, 0x50, 0x7A, 0x3F, 0xF3 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
//static uint8_t mydata[] = "Hello, world!";
static uint8_t mydata[10];
static osjob_t sendjob;
// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;
// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = RFM95_CS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = RFM95_RST,
    .dio = {RFM95_DIO0, RFM95_DIO1, RFM95_DIO2},
};

hw_timer_t * timer0 = NULL;
Adafruit_ADS1015 ads;     
Adafruit_BMP280 bmp;
RTC_DS1307 rtc;
DFRobot_ESP_PH ph;
OneWire oneWire(ADDRTemperatura);
DallasTemperature dallasTemperature(&oneWire);
Gravity_DO DO = Gravity_DO(ADDRSensorOD);

struct parametrosIQA 
{
  float temperaturaCx, temperaturaAgua;
  float ph;
  float turbidez;
  float turbidez_tensao_max;
  float satOD;
};

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) 
{
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void setupConfigLoRaWAN() 
{
    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif
 
    //void begin(int8_t sck=-1, int8_t miso=-1, int8_t mosi=-1, int8_t ss=-1);
    SPI.begin(18,19,23,RFM95_CS);
    //SPIClass(2);
    //spi2.begin(18,19,23,RFM95_CS);

    LMIC_setClockError(MAX_CLOCK_ERROR * 20 / 100);

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

float getLeituraADS() 
{
  int16_t adc0, adc1, adc2, adc3;
  float volts0, volts1, volts2, volts3;

  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  adc2 = ads.readADC_SingleEnded(2);
  adc3 = ads.readADC_SingleEnded(3);

  volts0 = ads.computeVolts(adc0);
  volts1 = ads.computeVolts(adc1);
  volts2 = ads.computeVolts(adc2);
  volts3 = ads.computeVolts(adc3);

  Serial.println("-----------------------------------------------------------");
  Serial.print("AIN0: "); Serial.print(adc0); Serial.print("  "); Serial.print(volts0*1000); Serial.println("V");
  Serial.print("AIN1: "); Serial.print(adc1); Serial.print("  "); Serial.print(volts1*1000); Serial.println("V");
  Serial.print("AIN2: "); Serial.print(adc2); Serial.print("  "); Serial.print(volts2*1000); Serial.println("V");
  Serial.print("AIN3: "); Serial.print(adc3); Serial.print("  "); Serial.print(volts3*1000); Serial.println("V");

  return adc3;
}

float getDadosBMP280()
{
  float temperatura = bmp.readTemperature();
  //float pressao = bmp.readPressure()/101325;

  //Serial.println("-----------------------------------------------------------");
  //Serial.print("TEMPERATURA INTERNA: "); Serial.print(String(temperatura)); Serial.println(" °C");
  //Serial.print("PRESSAO INTERNA: "); Serial.print(String(pressao)); Serial.println(" atm");

  return temperatura;
}

void printDataHora()
{
  Serial.println("-----------------------------------------------------------");
  Serial.print("DATA-HORA: "); 
  Serial.print(String(rtc.now().day())); 
  Serial.print("/");
  Serial.print(String(rtc.now().month())); 
  Serial.print("/");
  Serial.print(String(rtc.now().year())); 
  Serial.print("  ");
  Serial.print(String(rtc.now().hour())); 
  Serial.print(":");
  Serial.print(String(rtc.now().minute())); 
  Serial.print(":");
  Serial.println(String(rtc.now().second())); 
}

void setupADS()
{
  Serial.println("Hello!");

  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
   ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  if (!ads.begin(0x48)) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }
}

void setupRTC()
{
  rtc.begin();                                        // Inicia o módulo RTC
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));     // Ajuste Automático da hora e data
  //rtc.adjust(DateTime(2022, 5, 1, 15, 37, 0));   // Ajuste Manual (Ano, Mês, Dia, Hora, Min, Seg)
}

void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void setupSD() 
{
  //digitalWrite(RFM95_CS, HIGH);
  //digitalWrite(SD_CS, LOW);

  //SPIClass(1);
  //spi1.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);

  //if (!SD.begin( SD_CS, spi1)) 
  if (!SD.begin( SD_CS)) 
  {
    Serial.println("Erro na leitura do arquivo não existe um cartão SD ou o módulo está conectado incorretamente...");
    return;
  }

  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE) {
    Serial.println("Nenhum cartao SD encontrado");
    return;
  }

  Serial.println("Inicializando cartao SD...");
  //if (!SD.begin(SD_CS, spi1)) 
  if (!SD.begin(SD_CS)) 
  {
    Serial.println("ERRO - SD nao inicializado!");
    return; 
  }

  // Verifica se existe o arquivo de datalogger, casao nao, entao o arquivo e criado
  File file = SD.open("/data.csv");
  if(!file) 
  {
    Serial.println("SD: arquivo data.csv nao existe");
    Serial.println("SD: Criando arquivo...");
    writeFile(SD, "/data.csv", "DATA; HORA; TEMPERATURA INTERNA; TEMPERATURA EXTERNA; TURBIDEZ; PH; OD; \r\n");
  }
  else {
    Serial.println("SD: arquivo ja existe");  
  }

  file.close();
}

void salvarDatalogger(parametrosIQA *param) 
{ 
  //digitalWrite(RFM95_CS, HIGH);
  //digitalWrite(SD_CS, LOW);

  //  DATA; HORA; TEMPERATURA INTERNA; TEMPERATURA EXTERNA; TURBIDEZ; PH; OD;
  //String leitura = String(millis()) + ";" + String("temperatura") + ";" + String("turbidez") + ";" + String(" ") + "; \r\n"; 

  // DATA
  String leitura = String(rtc.now().day());
  leitura += "/";
  leitura += String(rtc.now().month());
  leitura += "/";
  leitura += String(rtc.now().year());
  leitura += ";  ";
  //  HORA
  leitura += String(rtc.now().hour());
  leitura += ":";
  leitura += String(rtc.now().minute());
  leitura += ":";
  leitura += String(rtc.now().second());
  leitura += "; ";
  //  TEMPERATURA INTERNA DA CAIXA
  leitura += String((*param).temperaturaCx);
  leitura += "; ";
  //  TEMPERATURA EXTERNA - TEMPERATURA DA AGUA
  leitura += String((*param).temperaturaAgua);
  leitura += "; ";
  //  TURBIDEZ
  leitura += String((*param).turbidez);
  leitura += "; ";
  //  pH
  leitura += String((*param).ph);
  leitura += "; ";
  //  OXIGENIO DISSOLVIDO
  leitura += String((*param).satOD);
  leitura += "; \r\n";
  
  Serial.println(" ");
  Serial.println("DATA; HORA; TEMPERATURA INTERNA; TEMPERATURA EXTERNA; TURBIDEZ; PH; OD");
  Serial.println(leitura); 

  appendFile(SD, "/data.csv", leitura.c_str());

  //digitalWrite(SD_CS, HIGH);
  //digitalWrite(RFM95_CS, LOW);
}

float getTurbidez() 
{
  float voltage  = 0;
  //Serial.print("DEBUG VOLTAGE ADS = "); Serial.println(voltage,2);

  for(int i=0; i<VoltageAVGLen; i++)
  {
      voltage  += ads.computeVolts(ads.readADC_SingleEnded(ADDRTurbidez));
  }

  voltage = voltage/VoltageAVGLen;

  /* CORRECAO
     Divisor de tensao: 220k/(220k+100k)
  */
  voltage = voltage*1.466067416;


  //voltage += 1.665;

  //Serial.print("DEBUG VOLTAGE = "); Serial.println(voltage);
  /*
  if(voltage>4.2)
    return 0;
  else if (voltage < 2.56)
    return 3000;
  */
  /*  y = -1120.4x² + 5742.3x - 4352.9
      Parabola com raizes x1 = 0.92, x2 = 4.2 e ponto y maximo em x= 2.56
  */
  //return -(1120.4*voltage*voltage) + (5742.3*voltage) - 4352.9;
  return (voltage - 4.0769 * (TensaoMaxTurbidez / 4.0769) ) / -0.0012;
/*
  if(voltage >= 3.635 && voltage <= 3.685)
    return -(7058.333*voltage*voltage) + 50929.750*voltage - 91834.197;
  else if(voltage >= 3.42 && voltage <= 3.62)
    return -(290.221*voltage*voltage) + 1864.896*voltage - 2900.582;
  else if(voltage >= 3.05 && voltage <= 3.45)
    return +(102.251*voltage*voltage) - 849.181*voltage + 1801.435;
  else if(voltage >= 1.8 && voltage <= 3.2)
    return +(358.387*voltage*voltage) - 2117.463*voltage + 3322.84; 
  

  if(voltage >= 2 && voltage <= 3.78)
    return +(19.709*voltage*voltage) - 381.166*voltage + 1155.325;
  else if (voltage > 3.78)
    return 0;
  else
    return 500; */
}

float getpH(float temperatura)
{
  float tensao = ads.computeVolts(ads.readADC_SingleEnded(ADDRpH));
  tensao *= 1000; 
  //Serial.print("tensao ph = "); Serial.println(tensao);

  //ph.calibration(tensao,temperatura);           // Calibracao por comunicacao serial.

  return ph.readPH(tensao,temperatura); 
}

void setupSensores() 
{
  // CONVERSOR ADC
  setupADS();

  // SENSOR TEMPERATURA E PRESSAO BMP280
  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  // SENSOR DE PH
  //ph.begin() // recupera calibrcao da memoria eeprom
  ph.set_neutralVoltage(1490);
  ph.set_acidVoltage(2006);

  // SENSOR OXIGENIO DISSOLVIDO
  //DO.setFullSatVoltage(2952);
  DO.setFullSatVoltage(1254);
}

float getDOpercentage() 
{
  float voltage = 0;

  for(int i = 0; i < VoltageAVGLen; i++)
  {
    voltage += ads.computeVolts(ads.readADC_SingleEnded(DO.getPinAddr()));
  }

  voltage *= 1000;
  voltage /= VoltageAVGLen;
  //voltage = voltage - 704;

  return DO.read_do_percentage(voltage);
}

void leituraRegistroDados()
{
  //struct parametrosIQA parametrosAgua;
  parametrosIQA parametrosAgua;

  //printDataHora();
  parametrosAgua.temperaturaCx = getDadosBMP280();
  mydata[0] = (int)parametrosAgua.temperaturaCx;
  mydata[1] = (int)((parametrosAgua.temperaturaCx-mydata[0])*100);

  // Leitura da temperatura
  dallasTemperature.requestTemperatures(); 
  parametrosAgua.temperaturaAgua = dallasTemperature.getTempCByIndex(0);
  mydata[2] = (int)parametrosAgua.temperaturaAgua;
  mydata[3] = (int)((parametrosAgua.temperaturaAgua-mydata[2])*100);

  getLeituraADS(); 

  parametrosAgua.turbidez = getTurbidez();
  mydata[4] = (int)parametrosAgua.turbidez;
  mydata[5] = (int)((parametrosAgua.turbidez-mydata[4])*100);

  parametrosAgua.ph = getpH(parametrosAgua.temperaturaAgua);
  mydata[6] = (int)parametrosAgua.ph;
  mydata[7] = (int)((parametrosAgua.ph-mydata[6])*100);

  parametrosAgua.satOD = getDOpercentage();
  mydata[8] = (int)parametrosAgua.satOD;
  mydata[9] = (int)((parametrosAgua.satOD-mydata[8])*100);

  salvarDatalogger(&parametrosAgua);
}

void alteraEstado()
{
  if(estado == AQUECIMENTO)
  {
    estado = LEITURADADOS;
    // CONFIGURA TEMPO ESTADO DE LEITURA
    xTimerStart(xTimer2, 0);
  }
  else if(estado == LEITURADADOS)
  {
    #ifdef TRANSMITIRDADOS
      estado = TRANSMISSAO;
      // CONFIGURA TEMPO ESTADO DE TRANSMISSAO
      setupConfigLoRaWAN();
      xTimerStart(xTimer3, 0);
    #endif
    
    #ifdef SOMENTEREGISTRO
      estado = LEITURADADOS;
      // CONFIGURA TEMPO ESTADO DE LEITURA
      xTimerStart(xTimer2, 0);
    #endif
  }
  else if(estado == TRANSMISSAO)
  {
    estado = LEITURADADOS;
    // CONFIGURA TEMPO ESTADO DE LEITURA
    xTimerStart(xTimer2, 0);
  } 
}

void callBackTimer(TimerHandle_t xTimer)
{
  alteraEstado();
}

// Funcao de interrupcao do Timer0
void IRAM_ATTR onTimer0() {
  Serial.println("ESP entrando em deep sleep...");

  digitalWrite(25, LOW);
  esp_deep_sleep_start(); // ESP32 entra em modo SLEEP
}

void setupTimer0() 
{
  // INTERRUPCAO DO TIMER 0
  // (timer escolhido = 0, prescaler = 80 (80 MHz/80 = 1MHz), contador crescente)
  timer0 = timerBegin(0, 80, true); 
  // (A interrupcao do timer0 executa a funcao "onTimer0", funcao de interrupcao do timer, interrupcao na borda)
  timerAttachInterrupt(timer0, &onTimer0, true); 
  // Devido ao prescaler o timer0 tem periodo igual a 1us
  // Contando ate "TempoON" temos uma interrupcao do timer depois de "TempoON" segundos
  // autoreload = false
  timerAlarmWrite(timer0, TEMPOACORDADO*1000000, false);
  timerAlarmEnable(timer0);
}

void setup(void)
{
  pinMode(25, OUTPUT);
  digitalWrite(25, HIGH);

  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);

  pinMode(RFM95_CS, OUTPUT);
  digitalWrite(RFM95_CS, HIGH);

  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.begin(115200);

  dallasTemperature.begin();
  EEPROM.begin(32); 
  
  setupSensores();

  setupSD();

  setupRTC();

  xTimer1 = xTimerCreate("TIMER-M1", pdMS_TO_TICKS(TEMPOAQUECIMENTO*1000), pdFALSE, 0, callBackTimer);
  xTimer2 = xTimerCreate("TIMER-M2", pdMS_TO_TICKS(TEMPOLEITURADADOS*1000), pdFALSE, 0, callBackTimer);
  xTimer3 = xTimerCreate("TIMER-M3", pdMS_TO_TICKS(TEMPOTRANSMISSAO*1000), pdFALSE, 0, callBackTimer);

  // Tempo que ESP estara "dormindo" - deep sleep
  esp_sleep_enable_timer_wakeup( TEMPODEEPSLEEP*1000000 );
  setupTimer0();

  estado = AQUECIMENTO;
  xTimerStart(xTimer1, 0);
}

void loop(void)
{
  switch (estado)
  {
    case LEITURADADOS:

      leituraRegistroDados();

      delay(500);
      Serial.println("LEITURA DADOS");

      break;

    case TRANSMISSAO:

      os_runloop_once();

      delay(500);
      Serial.println("TRANSMISSAO");

      break;

    default:
      break;
  }
 
}

