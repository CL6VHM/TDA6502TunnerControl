/*
 * Controlando un sintonizador de TV i2c basado en Philips TDA6502
 * Este sketch utiliza el pinout de depuración del sintonizador presente
 * en el televisor analógico cubano/chino Haier de 21" que en este
 * proyecto se usa como RF Frontend y convertidor descendente para
 * un dongle RTL-SDR u otro receptor proporcionando preamplificación,
 * filtrado y superando la pérdida de coaxial por la salida IF de
 * frecuencia más baja. La potencia del LNA y el AGC (un tipo de
 * amplificador de ganancia variable) se pueden comandar a través
 * de la placa Arduino. Se proporciona una interfaz de tipo prompt
 * serial para interactuar con el sintonizador, enviando instrucciones
 * o consultando el estado.
 * 
 * Versión: 1.0.0
 * Fecha: 23/12/2020
 * Autor: Raydel Abreu, CM2ESP
 * MOD: 23/1/2025
 * Autor de MOD: Victor Hdez Miteff, CL6VHM
 */

#include <Wire.h>
#include <LiquidCrystal.h>
#include <Encoder.h>

// Inicializar la librería con los números de los pines de la interfaz
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Pines del encoder
const int encoderPinA = 6;
const int encoderPinB = 7;
const int encoderButtonPin = 8;

// Pines de los botones para LNA y AGC
const int lnaButtonPin = 9;
const int agcButtonPin = 10;

Encoder myEncoder(encoderPinA, encoderPinB);

const bool DEBUG_FLAG = false;
const int SerialTimeOut = 1000;
const byte TUNE_RETRIES = 5;
const float IF_FREQ = 45;
const float F_STEP = 50.0;
const byte ADB = 0x60;
const byte LNA_PIN = 10; //LNA es un interruptor ON/OFF
const byte AGC_PIN = 11;  //AGC necesita un DAC PWM para controlar la ganancia
//Estado de inicialización predeterminado es VHF Alto a 145 MHz
byte DIVA = B00001110;
byte DIVB = B11011000;
//byte CNTR = B11001110; //FSTEP = 62.5k
byte CNTR = B11001000; //FSTEP = 50k
byte BBSW = B0010;
byte gain = 127;
bool PLL_Lock = false;
bool LNA_Status = false;
unsigned int multWord = 3800;
float currentFreq = 145.0;
bool adjustAGC = false;
long lastEncoderPos = 0;

void serialRead()
{
  if (Serial.available() > 0)
  {
    float freq = 0;
    String inBuff = Serial.readStringUntil('\n');
    //Verificar si es un comando de alternancia del LNA

    if (inBuff[0] == 'a' || inBuff[0] == 'A')
    {
      LNA_Status = !LNA_Status;
      Serial.print("La potencia del LNA es: ");
      if (LNA_Status) Serial.println("ON"); else Serial.println("OFF");
      if (LNA_Status) digitalWrite(LNA_PIN, HIGH); else digitalWrite(LNA_PIN, LOW);
      updateDisplay();
    }
    else
    {
      //Buscar comando de control de ganancia del AGC
      if (inBuff[0] == 'g' || inBuff[0] == 'G')
      {
        gain = inBuff.substring(1).toInt();
        analogWrite(AGC_PIN, gain);
        Serial.print("Establecer AGC a: ");
        Serial.println(gain);
        updateDisplay();
      }
      else
      {
        //Consulta de estado
        if (inBuff[0] == '?')
        {
          //Frecuencia
          Serial.println();
          Serial.println(".= Consulta de Estado =.");
          Serial.print("Sintonizado a: ");
          Serial.println((float)((multWord * F_STEP / 1000.0) - IF_FREQ));
          //Bloqueo del PLL
          Serial.print("Bloqueo del PLL: ");
          if (PLL_Lock) Serial.println("YES"); else Serial.println("NO");
          //Estado del LNA
          Serial.print("Estado del LNA: ");
          if (LNA_Status) Serial.println("ON"); else Serial.println("OFF");
          //Ganancia del AGC
          Serial.print("Ganancia del AGC: ");
          Serial.println(gain);
          Serial.println("Sistema Listo.");
          Serial.println();
        }
        else
        {
          //Buscar entrada de frecuencia en su lugar
          freq = inBuff.toFloat();
        }
      }
    }

    if (freq > 50 && freq < 830)
    {
      freq += IF_FREQ;
      freq = freq * 1000;
      multWord = (unsigned int)(freq / F_STEP);

      //Verificar la sintonización, los pasos del multiplicador podrían no coincidir exactamente
      float real_tuned = (float)((multWord * F_STEP / 1000.0) - IF_FREQ);
      Serial.print("Sintonizando a: ");
      Serial.println(real_tuned);

      DIVB = multWord & 0xff;
      DIVA = (multWord >> 8);
      if (DEBUG_FLAG)
      {
        Serial.print("Multiplicador de sintonización: ");
        Serial.println(multWord);
        Serial.println(DIVA, BIN);
        Serial.println(DIVB, BIN);
      }
      //Ajustar el interruptor de banda
      if (real_tuned <= 130) BBSW = B0001; //VHF Bajo
      if (real_tuned > 130 && real_tuned <= 300) BBSW = B0010; //VHF Alto
      if (real_tuned > 300) BBSW = B0100; //UHF
      //Sintonizar ahora
      Tune();
    }
    Serial.print("> ");
  }
}

void AskTuner()
{
  if (DEBUG_FLAG)
  {
    Serial.print("Interrogación enviada a: ");
    Serial.println(ADB, HEX);
  }

  Wire.requestFrom(ADB, 1);    // solicita 1 byte

  while (Wire.available()) { // el dispositivo puede enviar menos de lo solicitado
    byte c = Wire.read(); // recibe un byte como carácter

    if (DEBUG_FLAG)
    {
      Serial.print("Recibido: ");
      Serial.println(c, BIN);        // imprime el carácter
    }

    if (c == 88)PLL_Lock = true; else PLL_Lock = false;

    Serial.print("BLOQUEO DEL PLL: ");
    if (PLL_Lock) Serial.println("YES"); else Serial.println("NO");
    Serial.println("");
  }
}

void SendTuner()
{
  Serial.println("Enviando solicitud de sintonización...");

  Wire.beginTransmission(ADB); // transmite al dispositivo #
  delay(10);
  Wire.write(DIVA);        // envía un byte
  delay(10);
  Wire.write(DIVB);        // envía un byte
  delay(10);
  Wire.write(CNTR);        // envía un byte
  delay(10);
  Wire.write(BBSW);        // envía un byte
  delay(10);
  Wire.endTransmission();    // deja de transmitir

  Serial.println("Listo...");
}

void Tune()
{
  for (int i = 0; i < TUNE_RETRIES; i++)
  {
    SendTuner();
    delay(100);
    AskTuner();
    if (PLL_Lock) break;
  }
  updateDisplay();
}

void setup() {
  Wire.begin(); // unirse al bus i2c (dirección opcional para el maestro)

  // Abrir comunicaciones seriales y esperar a que el puerto se abra:
  Serial.begin(9600);
  Serial.setTimeout(SerialTimeOut);
  while (!Serial) {
    ; // esperar a que el puerto serial se conecte. Necesario solo para puerto USB nativo
  }

  Serial.println("Serial Inicializado...");

  // Inicializar el LCD
  lcd.begin(16, 2);
  lcd.print("Sistema Listo");

  //Encender el LED para saber que está listo
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  //Al iniciar, apagar el LNA
  pinMode(LNA_PIN, OUTPUT);
  digitalWrite(LNA_PIN, LOW);

  //Al iniciar, establecer el voltaje del AGC a 2.5V
  analogWrite(AGC_PIN, gain);

  // Inicializar encoder y botones
  pinMode(encoderButtonPin, INPUT_PULLUP);
  pinMode(lnaButtonPin, INPUT_PULLUP);
  pinMode(agcButtonPin, INPUT_PULLUP);
  
  //Enviar estado inicial
  Tune();
}

void loop()
{
  serialRead();

  // Manejar la rotación del encoder
  long newPosition = myEncoder.read();
  if (newPosition != lastEncoderPos) {
    if (adjustAGC) {
      gain = constrain(gain + (newPosition - lastEncoderPos) / 4, 0, 255); // Ajustar ganancia del AGC
      analogWrite(AGC_PIN, gain);
      Serial.print("Establecer AGC a: ");
      Serial.println(gain);
    } else {
      currentFreq = constrain(currentFreq + (newPosition - lastEncoderPos) * 0.05, 50, 830); // Ajustar frecuencia
      float freq = currentFreq + IF_FREQ;
      freq = freq * 1000;
      multWord = (unsigned int)(freq / F_STEP);
      float real_tuned = (float)((multWord * F_STEP / 1000.0) - IF_FREQ);
      Serial.print("Sintonizando a: ");
      Serial.println(real_tuned);
      DIVB = multWord & 0xff;
      DIVA = (multWord >> 8);
      Tune();
    }
    lastEncoderPos = newPosition;
    updateDisplay();
  }

  // Manejar la pulsación del botón del encoder
  if (digitalRead(encoderButtonPin) == LOW) {
    while (digitalRead(encoderButtonPin) == LOW); // Deshacer el rebote
    adjustAGC = !adjustAGC;
    Serial.print("Ajustando ");
    Serial.println(adjustAGC ? "AGC" : "Frecuencia");
    updateDisplay();
  }

  // Manejar la pulsación del botón del LNA
  if (digitalRead(lnaButtonPin) == LOW) {
    while (digitalRead(lnaButtonPin) == LOW); // Deshacer el rebote
    LNA_Status = !LNA_Status;
    Serial.print("La potencia del LNA es: ");
    Serial.println(LNA_Status ? "ON" : "OFF");
    digitalWrite(LNA_PIN, LNA_Status ? HIGH : LOW);
    updateDisplay();
  }

  // Manejar la pulsación del botón del AGC
  if (digitalRead(agcButtonPin) == LOW) {
    while (digitalRead(agcButtonPin) == LOW); // Deshacer el rebote
    adjustAGC = !adjustAGC;
    Serial.print("Ajustando ");
    Serial.println(adjustAGC ? "AGC" : "Frecuencia");
    updateDisplay();
  }

  delay(1);
}

void updateDisplay()
{
  float tunedFreq = (float)((multWord * F_STEP / 1000.0) - IF_FREQ);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Freq: ");
  lcd.print(tunedFreq, 1);
  lcd.print(" MHz");
  
  lcd.setCursor(0, 1);
  lcd.print("LNA: ");
  lcd.print(LNA_Status ? "ON" : "OFF");
  lcd.print(" AGC: ");
  lcd.print(gain);
}
