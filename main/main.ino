
#include <Adafruit_ADS1X15.h>
#include "Adafruit_BusIO_Register.h"
#include "Adafruit_BMP280.h"
#include "FS.h"
#include "SD.h"
#include "Wire.h"
#include "DallasTemperature.h"
#include "RTClib.h" 


//SENSORES
#define temp_pin        26
//#define VoltageAVGLen   100
#define pH_pin          1
#define DO_pin          2
#define ORP_pin         3
//#define INTERVALOREGISTRO 1500

// BARRAMENTO I2C
#define I2C_SDA 21
#define I2C_SCL 22

// Cartao SD pinout
#define  SD_CS      5
#define  SD_MOSI    23
#define  SD_MISO    19
#define  SD_CLK     18


//tempo de atualização em milisegundos
#define LOG_RATE 10000 
unsigned long LAST_LOG = 0; 
 

Adafruit_ADS1015 ads;  
Adafruit_BMP280 bmp;
RTC_DS1307 rtc;
OneWire oneWire(temp_pin);
DallasTemperature dallasTemperature(&oneWire);



void setup() {
  Serial.begin(115200); // Inicia a comunicação serial
  pinMode(25, OUTPUT);
  digitalWrite(25, HIGH);
  Wire.begin(I2C_SDA, I2C_SCL);
  
  dallasTemperature.begin();

  setupRTC();
  setupSD();

  ads.setGain(GAIN_ONE);
  if (!ads.begin(0x48)) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }

  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  
 

}

void loop() {
  
 

  if (LAST_LOG <= millis()){

    LAST_LOG = millis() + LOG_RATE;

    getLeituraADS();
  }

}




void getLeituraADS() 
{
  int16_t adc1, adc2, adc3;
  float mV1, mV2, mV3;

 // adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  adc2 = ads.readADC_SingleEnded(2);
  adc3 = ads.readADC_SingleEnded(3);

 // volts0 = ads.computeVolts(adc0);
  mV1 = ads.computeVolts(adc1) * 1000;
  mV2 = ads.computeVolts(adc2) * 1000;
  mV3 = ads.computeVolts(adc3) * 1000;
/*
  Serial.println("-----------------------------------------------------------");
 // Serial.print("DATA-HORA: "); 
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
  Serial.print(String(rtc.now().second())); 
  Serial.print("  ");

  Serial.print("pH: "); Serial.print(read_pH(mV1)); Serial.print("("); Serial.print(mV1); Serial.print("mV)  ");
  Serial.print("DO: "); Serial.print(read_do(mV2)); Serial.print("("); Serial.print(mV2); Serial.print("mV)  ");
  Serial.print("ORP: "); Serial.print(read_orp(mV3)); Serial.print("("); Serial.print(mV3); Serial.print("mV)  ");
  Serial.print("Temp : "); Serial.print(read_temp());Serial.print(" ");
  Serial.print("Temp IN: "); Serial.print(read_bmp280());Serial.println(" ");*/

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
  leitura += String(read_pH(mV1));
  leitura += "; ";
  // DO
  leitura += String(read_do(mV2));
  leitura += "; ";
  // ORP
  leitura += String(read_orp(mV3));
  leitura += "; ";
  // Temp
  leitura += String(read_temp());
  leitura += "; ";
  // Temp IN
  leitura += String(read_bmp280());
  leitura += "; ";
  // Battery
  leitura += String(0);
  leitura += "; \r\n";
  
  
  
  Serial.println(" ");
  Serial.println("date; time; pH; DO; ORP; Temp; Temp IN; Battery;");
  Serial.println(leitura); 

  appendFile(SD, "/data.csv", leitura.c_str());
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
    writeFile(SD, "/data.csv", "date; time; pH; DO; ORP; Temp; Temp IN; Battery;\r\n");
  }
  else {
    Serial.println("SD: arquivo ja existe");  
  }

  file.close();
}

float read_pH(float voltage_mV) {

  float ph_amostra_4 = 2005; //Valor em tensao referente a amostra de pH 4,01
  float ph_amostra_7 = 1486;//Valor em tensao referente a amostra de pH 7,01
  
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

  float calibracao = 1486; // 1726 - 240

  
  float DO_value = (voltage_mV - calibracao) ;
  return DO_value;

}

float read_temp(){
  dallasTemperature.requestTemperatures(); 
  return dallasTemperature.getTempCByIndex(0);
}

float read_bmp280()
{
  float temperatura = bmp.readTemperature();
  //float pressao = bmp.readPressure()/101325;

  //Serial.println("-----------------------------------------------------------");
  //Serial.print("TEMPERATURA INTERNA: "); Serial.print(String(temperatura)); Serial.println(" °C");
  //Serial.print("PRESSAO INTERNA: "); Serial.print(String(pressao)); Serial.println(" atm");

  return temperatura;
}

void setupRTC()
{
  rtc.begin();                                        // Inicia o módulo RTC
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));     // Ajuste Automático da hora e data
  //rtc.adjust(DateTime(2022, 5, 1, 15, 37, 0));   // Ajuste Manual (Ano, Mês, Dia, Hora, Min, Seg)
}
