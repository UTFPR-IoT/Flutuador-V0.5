
#include <Adafruit_ADS1X15.h>
//#include "Adafruit_BusIO_Register.h"
//#include "Adafruit_BMP280.h"
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include "Wire.h"
#include "DallasTemperature.h"
#include "RTClib.h" 
#include <lmic.h>
#include <hal/hal.h>


//SENSORES
#define temp_pin        5
//#define VoltageAVGLen   100
#define pH_pin          1
#define DO_pin          2
#define ORP_pin         3
//#define INTERVALOREGISTRO 1500

// BARRAMENTO I2C
#define I2C_SDA 21
#define I2C_SCL 22

#define I2C_SDA_2 16
#define I2C_SCL_2 17

// Cartao SD pinout
#define SD_SCK 4
#define SD_MISO 32
#define SD_MOSI 2
#define SD_CS 33


//tempo de atualização em milisegundos
#define LOG_RATE 10000 
unsigned long LAST_LOG = 0; 
//intervalo de envio
const unsigned TX_INTERVAL = 60;


 

Adafruit_ADS1015 ads;  
//Adafruit_BMP280 bmp;
RTC_DS1307 rtc;
OneWire oneWire(temp_pin);
DallasTemperature dallasTemperature(&oneWire);

SPIClass mySPI = SPIClass(HSPI);


const lmic_pinmap lmic_pins = {
    .nss = 25,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 26, 
    .dio = {27, 14, 13},
};

#define USE_OTAA
//#define USE_ABP

#ifdef USE_ABP
  static const PROGMEM u1_t NWKSKEY[16] = { 0xBB, 0xA8, 0x14, 0xAF, 0x24, 0xFF, 0x3D, 0xA6, 0xE0, 0xA7, 0x09, 0xDA, 0x45, 0x61, 0xD0, 0xF5 };
  static const u1_t PROGMEM APPSKEY[16] = { 0xC0, 0x52, 0x49, 0x2F, 0xD2, 0x82, 0x7D, 0x64, 0xBA, 0x3B, 0x7A, 0x99, 0x78, 0xD3, 0x5B, 0x7F };
  static const u4_t DEVADDR = 0x260D4F51;
  void os_getArtEui (u1_t* buf) { }
  void os_getDevEui (u1_t* buf) { }
  void os_getDevKey (u1_t* buf) { }
#endif

#ifdef USE_OTAA
  static const u1_t PROGMEM APPEUI[8] = { 0x1F, 0xBC, 0x05, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 }; //lsb format
  void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }
  static const u1_t PROGMEM DEVEUI[8]  = { 0x4C, 0xBE, 0x05, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 }; // lsb format
  void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }
  static const u1_t PROGMEM APPKEY[16] = { 0x98, 0x50, 0x5E, 0x8C, 0xA5, 0x70, 0x3D, 0x2C, 0xA8, 0xFB, 0x32, 0x43, 0x58, 0xED, 0x72, 0x2F }; //msb format
  void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }
#endif


// static uint8_t payload[] = "Hello, world!"; // Enviando uma mensagem simples - Prática 2, parte 1
byte payload[8]; // Enviando um vetor de bytes - Prática 2, parte 2
static uint16_t mydata = 0; // Variável do contador - Prática 2, parte 2
static osjob_t sendjob;
bool flag = false; // Flag para debounce


void do_send(osjob_t *j);

void printHex2(unsigned v)
{
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent(ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev)
    {
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
            for (size_t i = 0; i < sizeof(artKey); ++i)
            {
                if (i != 0)
                    Serial.print("-");
                printHex2(artKey[i]);
            }
            Serial.println("");
            Serial.print("NwkSKey: ");
            for (size_t i = 0; i < sizeof(nwkKey); ++i)
            {
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
    case EV_JOIN_FAILED:
        Serial.println(F("EV_JOIN_FAILED"));
        break;
    case EV_REJOIN_FAILED:
        Serial.println(F("EV_REJOIN_FAILED"));
        break;
    case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        // Flag de debounce, para enviar e somar o contador somente quando completar o envio anterior - Prática 2, parte 2
        flag = false;
        if (LMIC.txrxFlags & TXRX_ACK)
            Serial.println(F("Received ack"));
        if (LMIC.dataLen)
        {
            Serial.print(F("Received "));
            Serial.print(LMIC.dataLen);
            Serial.println(F(" bytes of payload"));
        }
        // Agenda a transmissão automática com intervalo de TX_INTERVAL - Prática 2, parte 1 ---------------------------------
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
        break;
    case EV_LOST_TSYNC:
        Serial.println(F("EV_LOST_TSYNC"));
        break;
    case EV_RESET:
        Serial.println(F("EV_RESET"));
        break;
    case EV_RXCOMPLETE:
        Serial.println(F("EV_RXCOMPLETE"));
        break;
    case EV_LINK_DEAD:
        Serial.println(F("EV_LINK_DEAD"));
        break;
    case EV_LINK_ALIVE:
        Serial.println(F("EV_LINK_ALIVE"));
        break;
    case EV_TXSTART:
        Serial.println(F("EV_TXSTART"));
        break;
    case EV_TXCANCELED:
        Serial.println(F("EV_TXCANCELED"));
        break;
    case EV_RXSTART:
        break;
    case EV_JOIN_TXCOMPLETE:
        Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
        break;

    default:
        Serial.print(F("Unknown event: "));
        Serial.println((unsigned)ev);
        break;
    }
}

void do_send(osjob_t *j)
{
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND)
    {
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else
    {
        // Prática 2 parte 1
        // LMIC_setTxData2(1, payload, sizeof(payload)-1, 0);
        // Serial.println("Packet queued");
        
        // // Prática 2, parte 2
        // Codificação da mensagem em bytes, dividindo um inteiro em 2 bytes
       // payload[0] = highByte(mydata);
       // payload[1] = lowByte(mydata);
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, payload, sizeof(payload), 0);
        Serial.println("Packet queued " + String(mydata));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}



void setup() {
  Serial.begin(115200); // Inicia a comunicação serial
  pinMode(25, OUTPUT);
  digitalWrite(25, HIGH);


  Wire.begin(I2C_SDA, I2C_SCL);
  Wire1.begin(I2C_SDA_2, I2C_SCL_2, 100000);

  
  

  dallasTemperature.begin();

  setupRTC();



  setupSD();

  ads.setGain(GAIN_ONE);
  if (!ads.begin(0x48, &Wire1)) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }

  //if (!bmp.begin(0x76)) {
  //  Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
  //  while (1);
  //}

  Serial.println("Starting...");
  delay(1000);
  LMIC_selectSubBand(1);
  LMIC_setLinkCheckMode(0);
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  Serial.println("Started!");

  // Envia o contador 0 automáticamente para realizar o JOIN
  do_send(&sendjob);
 

}

void loop() {
  
 

  if (LAST_LOG <= millis()){

    LAST_LOG = millis() + LOG_RATE;

    getLeituraADS();
  }

  os_runloop_once();
  //Serial.println("looping...");
  //delay(500);

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



void setupSD() {
  //digitalWrite(RFM95_CS, HIGH);
  //digitalWrite(SD_CS, LOW);

  //SPIClass(1);
  //spi1.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);

  //if (!SD.begin( SD_CS, spi1)) 
  mySPI.begin(SD_SCK, SD_MISO, SD_MOSI);
  if(!SD.begin(SD_CS, mySPI, 10000000))
  {
    Serial.println("Erro na leitura do arquivo não existe um cartão SD ou o módulo está conectado incorretamente...");
    return;
  }

  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE) {
    Serial.println("Nenhum cartao SD encontrado");
    return;
  }

  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
      Serial.println("MMC");
  } else if(cardType == CARD_SD){
      Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
      Serial.println("SDHC");
  } else {
      Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
  

  // Verifica se existe o arquivo de datalogger, casao nao, entao o arquivo e criado
  File file = SD.open("/data.csv");
  if(!file) 
  {
    Serial.println("SD: arquivo data.csv nao existe");
    Serial.println("SD: Criando arquivo...");
    writeFile(SD, "/data.csv", "date; time; pH; DO; ORP; Temp;\r\n");
  }
  else {
    Serial.println("SD: arquivo ja existe");  
  }

  file.close();
}

float read_pH(float voltage_mV) {

  float ph_amostra_4 = 2005; //Valor em tensao referente a amostra de pH 4,01
  float ph_amostra_7 = 1486;//Valor em tensao referente a amostra de pH 7,01

  //float ph_amostra_4 = 2010.00mV; //Valor em tensao referente a amostra de pH 4,01
  //float ph_amostra_7 = 1506.00mV;//Valor em tensao referente a amostra de pH 7,01
  //float ph_amostra_10 = 1042.00mV;//Valor em tensao referente a amostra de pH 10,01

  //equacao polinomial: 5e-07*xˆ2 - 0.0078*x + 17.579 
  //6.74

  //equacao linear: -0,0062x + 16,422
  //6.86



  //amostra -- 1542.00mV
  //hhann 6,55 e 6,47
  float coeficiente_angular = (7.01 - 4.01) / (ph_amostra_7 - ph_amostra_4); // m = (y2-y1)/(x2-x1)
  float coeficiente_linear = 4.01 - (coeficiente_angular * ph_amostra_4); // n = y - m*x

  
  
  float pH_value = (coeficiente_angular * voltage_mV) + coeficiente_linear; //y = m*x + n
  return pH_value;

}

float read_do(float voltage_mV) {

  float calibracao = 337.8; // 

  
  float DO_value = (voltage_mV * 100.0) / calibracao;
  return DO_value;

}

float read_orp(float voltage_mV) {

  //float calibracao = 1486; // 1726 - 240
  float calibracao = 1493; // 1718 - 225 (calibracao nova)
  
  float DO_value = (voltage_mV - calibracao) ;
  return DO_value;

}

float read_temp(){
  dallasTemperature.requestTemperatures(); 
  return dallasTemperature.getTempCByIndex(0);
 //return 23.09;
}
/*
float read_bmp280()
{
  //float temperatura = bmp.readTemperature();
  //float pressao = bmp.readPressure()/101325;

  //Serial.println("-----------------------------------------------------------");
  //Serial.print("TEMPERATURA INTERNA: "); Serial.print(String(temperatura)); Serial.println(" °C");
  //Serial.print("PRESSAO INTERNA: "); Serial.print(String(pressao)); Serial.println(" atm");

  return temperatura;
}
*/

void setupRTC()
{
  rtc.begin();                                        // Inicia o módulo RTC
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));     // Ajuste Automático da hora e data
  //rtc.adjust(DateTime(2022, 5, 1, 15, 37, 0));   // Ajuste Manual (Ano, Mês, Dia, Hora, Min, Seg)
}

void getLeituraADS() 
{
  int16_t adc0, adc1, adc2, adc3;
  float mV0, mV1, mV2, mV3;
  float pH, DO, ORP, temp;

  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  adc2 = ads.readADC_SingleEnded(2);
  adc3 = ads.readADC_SingleEnded(3);

 // volts0 = ads.computeVolts(adc0);
  mV0 = ads.computeVolts(adc0) * 1000;
  mV1 = ads.computeVolts(adc1) * 1000;
  mV2 = ads.computeVolts(adc2) * 1000;
  mV3 = ads.computeVolts(adc3) * 1000;




  pH = read_pH(mV0);
  DO = read_do(mV1);
  ORP = read_orp(mV2);
  temp = read_temp();

  Serial.print("pH: ");
  Serial.print(mV0);
  Serial.print("mV ---- DO: ");
  Serial.print(mV1);
  Serial.print("mV ---- ORP: ");
  Serial.print(mV2);
  Serial.print("mV ---- TEMP: ");
  Serial.print(temp);
  Serial.println(" C");
  

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
  // pH
  leitura += String(pH);
  leitura += "; ";
  // DO
  leitura += String(DO);
  leitura += "; ";
  // ORP
  leitura += String(ORP);
  leitura += "; ";
  // Temp
  leitura += String(temp);
  //leitura += "; ";
  // Temp IN
  //leitura += String(read_bmp280());
  //leitura += "; ";


  leitura += "; \r\n";
  
  
  
  Serial.println(" ");
  Serial.println("date; time; pH; DO; ORP; Temp; ");
  Serial.println(leitura); 

 


  payload[0] = (int)pH;
  payload[1] = (int)((pH-payload[0])*100);

  payload[2] = (int)DO;
  payload[3] = (int)((DO-payload[2])*100);

  payload[4] = (int)ORP;
  payload[5] = (int)((ORP-payload[4])*100);

  payload[6] = (int)temp;
  payload[7] = (int)((temp-payload[6])*100);

  appendFile(SD, "/data.csv", leitura.c_str());

 /// String path = "/data.csv";
 // Serial.printf("Appending to file: %s\n", path);

  //File file = SD.open(path, FILE_APPEND);
 /* if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(leitura.c_str())) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }*/
  //file.close();


  
}
