#include <Wire.h> // Arduino's I2C library
#define ARDUINO_MKR_PORT
#define TX_LAPSE_MS          10000
#include "tc_lib.h"
#include <SPI.h>             
#include <LoRa.h>
#include <Arduino_PMIC.h>
#include <RTCZero.h>

action_tc4_declaration();
action_tc5_declaration();

struct ctx
{
  ctx() { onoff=false; counter=0; }

  bool onoff;
  volatile uint32_t counter;
};

ctx ctx_1;
ctx ctx_2;

RTCZero rtc;


// LCD05's command related definitions
#define COMMAND_REGISTER byte(0x00)
#define SOFTWARE_REVISION byte(0x00)
#define RANGE_HIGH_BYTE byte(2)
#define RANGE_LOW_BYTE byte(3)
#define AUTOTUNE_MINIMUM_HIGH_BYTE byte(4)
#define AUTOTUNE_MINIMUM_LOW_BYTE byte(5)

// SRF02's command codes
#define REAL_RANGING_MODE_INCHES    byte(80)
#define REAL_RANGING_MODE_CMS       byte(81)
#define REAL_RANGING_MODE_USECS     byte(82)
#define FAKE_RANGING_MODE_INCHES    byte(86)
#define FAKE_RANGING_MODE_CMS       byte(87)
#define FAKE_RANGING_MODE_USECS     byte(88)
#define TRANSMIT_8CYCLE_40KHZ_BURST byte(92)
#define FORCE_AUTOTUNE_RESTART      byte(96)
#define ADDRESS_CHANGE_1ST_SEQUENCE byte(160)
#define ADDRESS_CHANGE_3RD_SEQUENCE byte(165)
#define ADDRESS_CHANGE_2ND_SEQUENCE byte(170)

#define ok 0xEA //Codigo de ok del slave 
#define OPUS  0x06 //Codigo de Operacion del comando "us"
#define OPOFF  0x01 //Codigo de operación del comando "us <dir> off"
#define OPOS  0x03 //Codigo de operación del comando "us <dir> one-shot"
#define OPST  0x05 //Codigo de operación del comando "us <dir> status"
#define OPCM  0x10//Codigo de operación del comando "us <dir> unit cm"
#define OPMS  0x11//Codigo de operación del comando "us <dir> unit ms"
#define OPINC  0x12 //Codigo de operación del comando "us <dir> unit inc"
#define OPD  0x07 //Codigo de operación del comando "us <dir> delay ms"
#define OPON  0x02 //Codigo de operación del comando "us <dir> on ms"

byte direccion_entrada = byte((0xEE)>>1);
byte direccion_salida = byte ((0xF2)>>1);
byte fakeD1 = byte(0xEE);
byte fakeD2 = byte(0xF2);
volatile int entradaFlag = 0;
volatile int salidaFlag = 0;
volatile int entradaFlag2 = 0;
volatile int salidaFlag2 = 0;
volatile int entradaFlag3 = 0;
volatile int salidaFlag3 = 0;

volatile int valorNormalEntrada = 0;
volatile int valorNormalSalida = 0;

volatile int disparador_entrada = 0;
volatile int disparador_salida = 0;

volatile int estados[3] = {(0,0),(0,0),(0,0)};

const byte unidades[3] = {80,81,82};
const String unidades_String[3] = {"Inches","Cms","ms"};

volatile int entradas = 0;
volatile int salidas = 0;
volatile int personas = 0;
const int puertaId = 0;

//--------------LORA--------------------------


// NOTA: Ajustar estas variables 
const uint8_t localAddress = 0xB0;     // Dirección de este dispositivo
uint8_t destination = 0xFF;            // Dirección de destino, 0xFF es la dirección de broadcast
uint8_t syncWord = 0x12;              // Palabra de sincronizacion
volatile bool txDoneFlag = true;       // Flag para indicar cuando ha finalizado una transmisión
volatile bool transmitting = false;

// Estructura para almacenar la configuración de la radio
typedef struct {
  uint8_t bandwidth_index;
  uint8_t spreadingFactor;
  uint8_t codingRate;
  uint8_t txPower; 
} LoRaConfig_t;

double bandwidth_kHz[10] = {7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
                            41.7E3, 62.5E3, 125E3, 250E3, 500E3 };

LoRaConfig_t thisNodeConf   = { 6, 10, 5, 2};
LoRaConfig_t remoteNodeConf = { 0,  0, 0, 0};
int remoteRSSI = 0;
float remoteSNR = 0;


//-----Sensonres config---------------//
int delay_entrada = 100;
int delay_salida = 100;
int unidad_entrada = 1; //Todas las mediciones en cm
int unidad_salida = 1; 
volatile double periodo_entrada = 10*100000;
volatile double periodo_salida = 10*100000;
int disparador_1 = 1;
int disparador_2 = 1;
int umbralcm_entrada = 30;
int umbralcm_salida = 30;

inline void write_command(byte address,byte command)
{ 
  Wire.beginTransmission(address);
  Wire.write(COMMAND_REGISTER); 
  Wire.write(command); 
  Wire.endTransmission();
}

byte read_register(byte address,byte the_register)
{
  Wire.beginTransmission(address);
  Wire.write(the_register);
  Wire.endTransmission();
  
  // getting sure the SRF02 is not busy
  Wire.requestFrom(address,byte(1));
  while(!Wire.available()) { /* do nothing */ }
  return Wire.read();
} 


void ejecutar_dispositivo(uint8_t direccion, int unidad, int delay_ms) {
  write_command(direccion,unidades[unidad]);
  delay(delay_ms);
  
  
  byte high_byte_range=read_register(direccion,RANGE_HIGH_BYTE);
  byte low_byte_range=read_register(direccion,RANGE_LOW_BYTE);
  byte high_min=read_register(direccion,AUTOTUNE_MINIMUM_HIGH_BYTE);
  byte low_min=read_register(direccion,AUTOTUNE_MINIMUM_LOW_BYTE);


  Serial.print(int((high_byte_range<<8) | low_byte_range)); Serial.print(unidades_String[unidad] +
   ". (min=");
  Serial.print(int((high_min<<8) | low_min)); Serial.println(unidades_String[unidad] + ".)");
int resultado = int((high_byte_range<<8) | low_byte_range);
int min = int((high_min<<8) | low_min);



if(direccion == direccion_entrada){

float cambioE = resultado - valorNormalEntrada;
Serial.println(cambioE);
if(cambioE < - umbralcm_entrada){
    entradaFlag = 1;
  }else{
        entradaFlag = 0;

  }
}

if(direccion == direccion_salida){
  float cambioS = resultado - valorNormalSalida;
Serial.println(cambioS);
if(cambioS < - umbralcm_salida){
    salidaFlag = 1;
  }else{
     salidaFlag = 0;
  }
}
      
            
}

void disparar_1(void* a_ctx)
{
  ctx* the_ctx=reinterpret_cast<ctx*>(a_ctx);
  disparador_1 = 1;
  the_ctx->onoff=!(the_ctx->onoff);

  the_ctx->counter++;
}

void disparar_2(void* a_ctx)
{
  ctx* the_ctx=reinterpret_cast<ctx*>(a_ctx);

  disparador_2 = 1;
  the_ctx->onoff=!(the_ctx->onoff);

  the_ctx->counter++;
}


int calibrar_dispositivo(uint8_t direccion, int unidad, int delay_ms) {
  write_command(direccion,unidades[unidad]);
  delay(delay_ms);
  
  
  byte high_byte_range=read_register(direccion,RANGE_HIGH_BYTE);
  byte low_byte_range=read_register(direccion,RANGE_LOW_BYTE);
  byte high_min=read_register(direccion,AUTOTUNE_MINIMUM_HIGH_BYTE);
  byte low_min=read_register(direccion,AUTOTUNE_MINIMUM_LOW_BYTE);


  Serial.print(int((high_byte_range<<8) | low_byte_range)); Serial.print(unidades_String[unidad] +
   ". (min=");
  Serial.print(int((high_min<<8) | low_min)); Serial.println(unidades_String[unidad] + ".)");
int resultado = int((high_byte_range<<8) | low_byte_range);

int min = int((high_min<<8) | low_min);

return resultado;

}



void setup() {
  Serial.begin(9600);
  Wire.begin();
  //Serial1.begin(9600);

   Serial.println("LoRa Duplex with TxDone and Receive callbacks");
  Serial.println("Using binary packets");
  
  // Es posible indicar los pines para CS, reset e IRQ pins (opcional)
  // LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  
  if (!init_PMIC()) {
    Serial.println("Initilization of BQ24195L failed!");
  }
  else {
    Serial.println("Initilization of BQ24195L succeeded!");
  }

  if (!LoRa.begin(868E6)) {      // Initicializa LoRa a 868 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                
  }

  // Configuramos algunos parámetros de la radio
  LoRa.setSignalBandwidth(long(bandwidth_kHz[thisNodeConf.bandwidth_index])); 
                                  // 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3
                                  // 41.7E3, 62.5E3, 125E3, 250E3, 500E3 
                                  // Multiplicar por dos el ancho de banda
                                  // supone dividir a la mitad el tiempo de Tx
                                  
  LoRa.setSpreadingFactor(thisNodeConf.spreadingFactor);     
                                  // [6, 12] Aumentar el spreading factor incrementa 
                                  // de forma significativa el tiempo de Tx
                                  // SPF = 6 es un valor especial
                                  // Ver tabla 12 del manual del SEMTECH SX1276
  
  LoRa.setCodingRate4(thisNodeConf.codingRate);         
                                  // [5, 8] 5 da un tiempo de Tx menor
                                  
  LoRa.setTxPower(thisNodeConf.txPower, PA_OUTPUT_PA_BOOST_PIN); 
                                  // Rango [2, 20] en dBm
                                  // Importante seleccionar un valor bajo para pruebas
                                  // a corta distancia y evitar saturar al receptor
  LoRa.setSyncWord(syncWord);         // Palabra de sincronización privada por defecto para SX127X 
                                  // Usaremos la palabra de sincronización para crear diferentes
                                  // redes privadas por equipos
  LoRa.setPreambleLength(8);      // Número de símbolos a usar como preámbulo

  
  // No vamos a recibir paquetes
 // LoRa.onReceive(onReceive);
  
  // Activamos el callback que nos indicará cuando ha finalizado la 
  // transmisión de un mensaje
  LoRa.onTxDone(TxFinished);

  // No vamos a recibir paquetes
  //LoRa.receive();

  Serial.println("LoRa init succeeded.\n");
 
 //Inicializamos el reloj
  rtc.begin();
  rtc.setMinutes(0);
  rtc.enableAlarm(rtc.MATCH_MMSS);
  rtc.setAlarmMinutes(5);
  rtc.attachInterrupt(LoRaSegment);

//Calibramos los sensores para saber que distancia es la que va a ser de regular
valorNormalEntrada =  calibrar_dispositivo(direccion_entrada,unidades[1],delay_entrada);

valorNormalSalida = calibrar_dispositivo(direccion_salida,unidades[1],delay_salida);



//Iniciamos las interrupciones 
  action_tc4.start(periodo_entrada,disparar_1,&ctx_1);
  action_tc5.start(periodo_salida,disparar_2,&ctx_2);

}

void loop() {
  // put your main code here, to run repeatedly:
 if(disparador_1 == 1){
  Serial.print("Dispositivo 1: \n");
  ejecutar_dispositivo(direccion_entrada,unidad_entrada,delay_entrada);
  disparador_1 = 0;
  }

  if(disparador_2 == 1){
  Serial.print("Dispositivo 2: \n");
 ejecutar_dispositivo(direccion_salida,unidad_salida,delay_salida);
 disparador_2 = 0;
  }

//Logica de progreso para saber si estan entrando o saliendo
  if(salidaFlag == 0 && entradaFlag == 1 && salidaFlag3 == 1){
    salidas++;
    personas--;
  }

  if(salidaFlag == 1 && entradaFlag == 1 && salidaFlag2 == 1 && entradaFlag2 == 0){
  salidaFlag3 = 1;
}else{
  salidaFlag3 = 0;
}

if(salidaFlag3 == 1){
  return;
}

if(salidaFlag == 1 && entradaFlag == 0 && entradaFlag2 == 0){
  salidaFlag2 = 1;
}else{
  salidaFlag2 = 0;
}

if(salidaFlag2 == 1){
  return;
}

if(salidaFlag == 1 && entradaFlag == 0 && entradaFlag3 == 1){
  entradas++;
    personas++;
    
}



if(entradaFlag3 == 1){
  
}


if(salidaFlag == 1 && entradaFlag == 1 && entradaFlag2 == 1){
 entradaFlag3 = 1;
}else{
  entradaFlag3 = 0;
}

if(entradaFlag3 == 1){
  return;
}

if(salidaFlag == 0 && entradaFlag == 1){
  entradaFlag2 = 1;
}else{
  entradaFlag2 = 0;
}

if(entradaFlag2 == 1){
  return;
}


  Serial.println("Personas: ");
  Serial.print(personas);
  Serial.print("Salidas: ");
  Serial.println(salidas);
  Serial.print("Entradas : ");
  Serial.println(entradas);




}


void LoRaSegment(){

  action_tc4.stop();
  action_tc5.stop();

  static uint16_t msgCount = 0;


    uint8_t payload[50];
    uint8_t payloadLength = 0;

    payload[payloadLength]    = (thisNodeConf.bandwidth_index << 4);
    payload[payloadLength++] |= ((thisNodeConf.spreadingFactor - 6) << 1);
    payload[payloadLength]    = ((thisNodeConf.codingRate - 5) << 6);
    payload[payloadLength++] |= ((thisNodeConf.txPower - 2) << 1);

    // Incluimos el RSSI y el SNR del último paquete recibido
    // RSSI puede estar en un rango de [0, -127] dBm
    payload[payloadLength++] = uint8_t(-LoRa.packetRssi() * 2);
    // SNR puede estar en un rango de [20, -148] dBm
    payload[payloadLength++] = uint8_t(148 + LoRa.packetSnr());
    //Añadimos el numero de personas que ha captado el sensor
    payload[payloadLength++] = uint8_t(personas);
    //Añadimos el número de entradas que ha captado el sensor
    payload[payloadLength++] = uint8_t(entradas);
    //Añadimos el número de salidas que ha captado el sensor
    payload[payloadLength++] = uint8_t(salidas);
  
    sendMessage(payload, payloadLength, msgCount);
    Serial.print("Sending packet ");
    Serial.print(msgCount++);
    Serial.print(": ");
    printBinaryPayload(payload, payloadLength);

    rtc.setMinutes(0);

  action_tc4.start(periodo_entrada,disparar_1,&ctx_1);
  action_tc5.start(periodo_salida,disparar_2,&ctx_2);
}


void TxFinished()
{
  txDoneFlag = true;
}

void printBinaryPayload(uint8_t * payload, uint8_t payloadLength)
{
  for (int i = 0; i < payloadLength; i++) {
    Serial.print((payload[i] & 0xF0) >> 4, HEX);
    Serial.print(payload[i] & 0x0F, HEX);
    Serial.print(" ");
  }
}

// --------------------------------------------------------------------
// Sending message function
// --------------------------------------------------------------------
void sendMessage(uint8_t* payload, uint8_t payloadLength, uint16_t msgCount) 
{
  while(!LoRa.beginPacket()) {            // Comenzamos el empaquetado del mensaje
    delay(10);                            // 
  }
  LoRa.write(destination);                // Añadimos el ID del destinatario
  LoRa.write(localAddress);               // Añadimos el ID del remitente
  LoRa.write((uint8_t)(msgCount >> 7));   // Añadimos el Id del mensaje (MSB primero)
  LoRa.write((uint8_t)(msgCount & 0xFF)); 
  LoRa.write(payloadLength);              // Añadimos la longitud en bytes del mensaje
  LoRa.write(payload, (size_t)payloadLength); // Añadimos el mensaje/payload 
  LoRa.endPacket(true);                   // Finalizamos el paquete, pero no esperamos a
                                          // finalice su transmisión
}
