#include <Wire.h>  // Arduino's I2C library
#define ARDUINO_MKR_PORT
#define TX_LAPSE_MS 10000
#include "tc_lib.h"
#include <SPI.h>
#include <LoRa.h>
#include <Arduino_PMIC.h>

action_tc4_declaration();



// LCD05's command related definitions
#define COMMAND_REGISTER byte(0x00)
#define SOFTWARE_REVISION byte(0x00)
#define RANGE_HIGH_BYTE byte(2)
#define RANGE_LOW_BYTE byte(3)
#define AUTOTUNE_MINIMUM_HIGH_BYTE byte(4)
#define AUTOTUNE_MINIMUM_LOW_BYTE byte(5)

// SRF02's command codes
#define REAL_RANGING_MODE_USECS byte(82)
#define ADDRESS_CHANGE_1ST_SEQUENCE byte(160)
#define ADDRESS_CHANGE_3RD_SEQUENCE byte(165)
#define ADDRESS_CHANGE_2ND_SEQUENCE byte(170)

volatile int entradaFlag = 0;
volatile int salidaFlag = 0;
volatile int entradaFlag2 = 0;
volatile int salidaFlag2 = 0;
volatile int entradaFlag3 = 0;
volatile int salidaFlag3 = 0;

volatile int normal_val_in = 0;
volatile int normal_val_out = 0;

volatile int disparador_entrada = 0;
volatile int disparador_salida = 0;

volatile int8_t entradas = 0;
volatile int8_t salidas = 0;
volatile int8_t personas = 0;
//--------------LORA--------------------------


// NOTA: Ajustar estas variables
const uint8_t localAddress = 0xB0;  // Dirección de este dispositivo
uint8_t destination = 0x02;         // Dirección de destino, 0xFF es la dirección de broadcast
uint8_t syncWord = 0x13;            // Palabra de sincronizacion
volatile bool txDoneFlag = true;    // Flag para indicar cuando ha finalizado una transmisión
volatile bool transmitting = false;

// Estructura para almacenar la configuración de la radio
typedef struct {
  uint8_t bandwidth_index;
  uint8_t spreadingFactor;
  uint8_t codingRate;
  uint8_t txPower;
} LoRaConfig_t;

double bandwidth_kHz[10] = { 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
                             41.7E3, 62.5E3, 125E3, 250E3, 500E3 };

LoRaConfig_t thisNodeConf   = { 9, 8, 5, 5};

enum STAGE {
  BASE,
  INDISTINGUISHED,
  IN_ACTIVATED,
  IN_OUT,
  IN_THEN_OUT,
  OUT_ACTIVATED,
  OUT_IN,
  OUT_THEN_IN,
};

//-----Sensonres config---------------//
int umbralcm_entrada = 30;
int umbralcm_salida = 30;
long time = 0;
const uint8_t addrs_vec[2] = {0xE0 >> 1, 0xEA >> 1};
uint16_t measures[2] = {0, 0};
volatile bool flag_get = false;
constexpr const long period = 70 * 100000;
uint16_t threshold_in = 0;
uint16_t threshold_out = 0;
STAGE stage = BASE;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  Serial.println("LoRa Duplex with TxDone and Receive callbacks");
  Serial.println("Using binary packets");

  if (!init_PMIC()) {
    Serial.println("Initilization of BQ24195L failed!");
  } else {
    Serial.println("Initilization of BQ24195L succeeded!");
  }

  if (!LoRa.begin(868E6)) {  // Initicializa LoRa a 868 MHz
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
  LoRa.setSyncWord(syncWord);  // Palabra de sincronización privada por defecto para SX127X
                               // Usaremos la palabra de sincronización para crear diferentes
                               // redes privadas por equipos
  LoRa.setPreambleLength(8);   // Número de símbolos a usar como preámbulo

  LoRa.onTxDone(TxFinished);

  //Calibramos los sensores para saber que distancia es la que va a ser de regular
  normal_val_in = calibrar_dispositivo(0);
  normal_val_out = calibrar_dispositivo(1);
  threshold_in = normal_val_in/2;
  threshold_out = normal_val_out/2;
  Serial.println(normal_val_in);
  Serial.println(normal_val_out);
  delay(2000);
  //Iniciamos las interrupciones
  time = millis();
  action_tc4.start(period, disparar, nullptr);
}

void loop() {
  static uint32_t measure_counter = 0;
  // put your main code here, to run repeatedly:
  if (flag_get) {
    if (get_measure(0)) {
      write_command(0);

      int cambioE = normal_val_in - measures[0];
      Serial.println(cambioE);
      if (cambioE > threshold_in) {
        entradaFlag = 1;
      } else {
        entradaFlag = 0;
      }
    }
    if (get_measure(1)) {
      write_command(1);
      int cambioS = normal_val_out - measures[1];
      Serial.println(cambioS);
      if (cambioS > threshold_out) {
        salidaFlag = 1;
      } else {
        salidaFlag = 0;
      }
    }
    flag_get = false;

    uint8_t inout = (entradaFlag<<1) + salidaFlag;
    switch (stage) {
      case BASE: // 0 -- 0
      Serial.println("BASE");
        switch(inout) {
          case 0b00: break;
          case 0b01: stage = OUT_ACTIVATED; break;
          case 0b10: stage = IN_ACTIVATED; break;
          case 0b11: stage = INDISTINGUISHED; break;
        }
      break;
      case INDISTINGUISHED: // 1 -- 1
      Serial.println("INDISTINGUISHED");
        switch(inout) {
          case 0b00: stage = BASE; break;
          case 0b01: break;
          case 0b10: break;
          case 0b11: break;
        }
      break;
      case IN_ACTIVATED: // 1 -> 0
      Serial.println("IN_ACTIVATED");
        switch(inout) {
          case 0b00: stage = BASE; break;
          case 0b01: stage = IN_THEN_OUT; break;
          case 0b10: break;
          case 0b11: stage = IN_OUT; break;
        }
      break;
      case IN_OUT: // 1 -> 1
      Serial.println("IN_OUT");
        switch(inout) {
          case 0b00: stage = BASE; personas++; break;
          case 0b01: stage = IN_THEN_OUT; break;
          case 0b10: stage = IN_ACTIVATED; break;
          case 0b11: break;
        }
      break;
      case IN_THEN_OUT: // 0 -> 1
      Serial.println("IN_THEN_OUT");
        switch(inout) {
          case 0b00: stage = BASE; personas ++; break;
          case 0b01: break;
          case 0b10: stage = INDISTINGUISHED; break;
          case 0b11: stage = IN_OUT; break;
        }
      break;
      case OUT_ACTIVATED: // 0 <- 1
      Serial.println("OUT_ACTIVATED");
        switch(inout) {
          case 0b00: stage = BASE; break;
          case 0b01: break;
          case 0b10: stage = OUT_THEN_IN; break;
          case 0b11: stage = OUT_IN; break;
        }
      break;
      case OUT_IN: // 1 <- 1
      Serial.println("OUT_IN");
        switch(inout) {
          case 0b00: stage = BASE; personas--; break;
          case 0b01: stage = OUT_ACTIVATED; break;
          case 0b10: stage = OUT_THEN_IN; break;
          case 0b11: break;
        }
      break;
      case OUT_THEN_IN: // 1 <- 0
      Serial.println("OUT_THEN_IN");
        switch(inout) {
          case 0b00: stage = BASE; personas--; break;
          case 0b01: stage = INDISTINGUISHED; break;
          case 0b10: break;
          case 0b11: stage = OUT_IN; break;
        }
      break;
    }

    Serial.print("Personas: ");
    Serial.println(personas);
    measure_counter++;
  }

  if (millis() - time >= 10000) {
    LoRaSegment();
    time = millis();
    personas = 0;
  }

}


void LoRaSegment() {

  action_tc4.stop();

  static uint16_t msgCount = 0;


  uint8_t payload[5];
  uint8_t payloadLength = 0;
  //Añadimos el número de entradas que ha captado el sensor
  payload[payloadLength++] = personas;

  sendMessage(payload, payloadLength, msgCount);
  Serial.print("Sending packet ");
  Serial.print(msgCount++);
  Serial.print(": ");
  Serial.print((payload[0] & 0xF0) >> 4, HEX);
  Serial.println(payload[0] & 0x0F, HEX);

  action_tc4.start(period, disparar, nullptr);
}

void TxFinished() {
  txDoneFlag = true;
}

// --------------------------------------------------------------------
// Sending message function
// --------------------------------------------------------------------
void sendMessage(uint8_t* payload, uint8_t payloadLength, uint16_t msgCount) {
  while (!LoRa.beginPacket()) {  // Comenzamos el empaquetado del mensaje
    delay(10);                   //
  }
  LoRa.write(destination);               // Añadimos el ID del destinatario
  LoRa.write(localAddress);              // Añadimos el ID del remitente
  LoRa.write((uint8_t)(msgCount >> 7));  // Añadimos el Id del mensaje (MSB primero)
  LoRa.write((uint8_t)(msgCount & 0xFF));
  LoRa.write(payloadLength);                   // Añadimos la longitud en bytes del mensaje
  LoRa.write(payload, (size_t)payloadLength);  // Añadimos el mensaje/payload
  LoRa.endPacket(true);                        // Finalizamos el paquete, pero no esperamos a
                                               // finalice su transmisión
}

void write_command(uint8_t dev) {
  Wire.beginTransmission(addrs_vec[dev]);
  Wire.write(COMMAND_REGISTER);
  Wire.write(REAL_RANGING_MODE_USECS);
  Wire.endTransmission();
}

bool get_measure(uint8_t dev) {
  if (read_register(addrs_vec[dev], 0) != 0xFF) {
    byte hi = read_register(addrs_vec[dev], RANGE_HIGH_BYTE);
    byte lo = read_register(addrs_vec[dev], RANGE_LOW_BYTE);
    measures[dev] = get_measure(hi, lo);
    return true;
  } else return false;
}

uint16_t get_measure(uint8_t hi, uint8_t lo) {
  return uint16_t(hi << 8) + uint16_t(lo);
}

byte read_register(byte address, byte the_register) {
  Wire.beginTransmission(address);
  Wire.write(the_register);
  Wire.endTransmission();

  // getting sure the SRF02 is not busy
  Wire.requestFrom(address, byte(1));
  while (!Wire.available()) { Serial.println("WIRE ERROR"); /* do nothing */ }
  return Wire.read();
}

void disparar(void* v) {
  flag_get = true;
}

int calibrar_dispositivo(uint8_t dev) {
  write_command(dev);

  while(read_register(addrs_vec[dev], 0) == 0xFF);

  byte hi = read_register(addrs_vec[dev], RANGE_HIGH_BYTE);
  byte lo = read_register(addrs_vec[dev], RANGE_LOW_BYTE);

  return get_measure(hi, lo);
}
