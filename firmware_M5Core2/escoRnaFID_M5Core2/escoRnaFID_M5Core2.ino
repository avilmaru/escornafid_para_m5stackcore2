// ---------------------------------------------------------------------------
// escoRnaFID para MCore 2
// Escornabot: Control del robot mediante  M5Core2 + M5GO Battery Bottom2 + Mini RFID Reader/Writer Unit (MFRC522)

// AUTOR/LICENCIA:
//
// Creado por Angel Villanueva - @avilmaru
// Licencia Creative Commons Attribution-ShareAlike 4.0 International (CC BY-SA 4.0).
//
// LINKS:
// Blog: http://www.mecatronicalab.es
//
//
// HISTORICO:
// 19/01/2021 v1.0 - Release inicial.
//
// ---------------------------------------------------------------------------


/**************************************************************************
 * Identificadores de tarjeta
**************************************************************************/

// NOTA: Cambiar por los códigos de las tarjetas que se tengan (6 tarjetas son necesarias)

byte tarjetaGirarIzquierda[4]  = {0xF1, 0x81, 0xE1, 0x1E};   
byte tarjetaGirarDerecha[4]    = {0xB4, 0x5C, 0x5E, 0x13};
byte tarjetaAvanzar[4]         = {0xF7, 0x74, 0x7C, 0x01}; 
byte tarjetaRetroceder[4]      = {0xD3, 0xFA, 0xD3, 0x1C}; 
byte tarjetaResetear[4]        = {0x73, 0x21, 0x2F, 0x1D}; 
byte tarjetaEjecutar[4]        = {0xBC, 0x21, 0x53, 0x83};  

//////////////////////////////////////////////////////////////////////


#include <M5Core2.h>
#include <Wire.h>
#include <WiFi.h>

//////////////////////////////////////////////////////////////////////

#include "MFRC522_I2C.h"
  
MFRC522 mfrc522(0x28);  // 0x28 is i2c address on SDA.

//////////////////////////////////////////////////////////////////////

#include <Adafruit_NeoPixel.h>

const int LEDS_PIN = 25;
const int LEDS_NUM = 10;

Adafruit_NeoPixel leds = Adafruit_NeoPixel(LEDS_NUM, LEDS_PIN, NEO_GRB + NEO_KHZ800);

//////////////////////////////////////////////////////////////////////

#include <driver/i2s.h>

#include <AudioFileSourceSD.h>
#include <AudioFileSourceID3.h>
#include <AudioGeneratorMP3.h>
#include <AudioOutputI2S.h>

AudioGeneratorMP3 *mp3;
AudioFileSourceSD *file;
AudioOutputI2S *out;
AudioFileSourceID3 *id3;


/**************************************************************************/

bool instruccionesEnCurso = false;

/**************************************************************************/

extern const unsigned char bibiSig[8820];

#define Speak_I2S_NUMBER I2S_NUM_0

const int CONFIG_I2S_BCK_PIN =  12;
const int CONFIG_I2S_LRCK_PIN = 0;
const int CONFIG_I2S_DATA_PIN  = 2;
const int CONFIG_I2S_DATA_IN_PIN = 34;
    
//////////////////////////////////////////////////////////////////////

#include <BLEDevice.h>

// The remote service we wish to connect to.
static BLEUUID serviceUUID("ffe0");
// The characteristic of the remote service we are interested in.
static BLEUUID    charUUID("ffe1");


static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;


static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {

  instruccionesEnCurso = true;
     
  std::string myString((char*)pData, length);
  
  if (myString.compare("N\n") == 0)
    movimiento("avanzar",true);
  else if (myString.compare("S\n") == 0)
    movimiento("retroceder",true);
  else if (myString.compare("W\n") == 0)
    movimiento("izquierda",true);
  else if (myString.compare("E\n") == 0)
    movimiento("derecha",true);
  else if (myString.compare("Z\n") == 0)
    accion("error");
  else if (myString.compare("X\n") == 0)
    accion("fin ok");
  else if (myString.compare("Y\n") == 0)
    accion("fin ko");
            
}

class MyClientCallback : public BLEClientCallbacks {
  
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
  }
  
};

bool connectToServer() {
   
    // Create client
    BLEClient*  pClient  = BLEDevice::createClient();
   // Created client
   
    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    // Connected to server

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {  //Failed to find our service UUID
      pClient->disconnect();
      return false;
    }
    
    //Found our service

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {   //Failed to find our characteristic UUID
      pClient->disconnect();
      return false;
    }
    // Found our characteristic

    // Register for notify
    if(pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(notifyCallback);

    connected = true;
    return true;
}



/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
   
    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;
      escribeTexto(290,50,2,"OK",GREEN);

    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks


/**************************************************************************/


void setup(void) {

    // Inicializar  M5Core2
    M5.begin();
    SpeakInit(false);
    WiFi.mode(WIFI_OFF); 

    escornafid();
    delay(3000);
    reset();
  
    // Inicialización LEDS  
    leds.begin(); 
    leds.setBrightness(255);
   
    // Inicializar I2C
    Wire.begin();              

    escribeTexto(5,10,2,"VERIFICANDO",ORANGE);
  
    // Inicilializar MFRC522
    escribeTexto(5,50,2,"Unidad RFID ...",WHITE);
    mfrc522.PCD_Init(); 
    escribeTexto(290,50,2,"OK",GREEN);      

    // Verificaciones de la tarjeta SD
    delay(1000);
    escribeTexto(5,90,2,"Tarjeta SD  ...",WHITE);
    SD.begin();
    if (!checkSDCard())
    {
       escribeTexto(290,90,2,"KO",RED); 
       efectoLeds("error");     
       while(1);
    }
    escribeTexto(290,90,2,"OK",GREEN); 
    
    delay(1000);  
    ponerOK();
    beep();
    delay(2000);
    reset();
    
    // BLE
    escribeTexto(5,10,2,"Vamos a conectarnos con el escornabot.",ORANGE);   
    escribeTexto(5,70,2,"Enciende el robot y activa su bluetooth.",WHITE);  
    botonConectar();   
    reset();

    escribeTexto(5,10,2,"VINCULANDO",ORANGE);
    escanear();
    conectar();

    delay(1000);  
    ponerOK();
    beep();
    delay(2000);
    reset();

    SpeakInit(true);  

    // Imagen 
    M5.Lcd.drawJpgFile(SD, "/images/ok.jpg");
    delay(2000);
    reset();
  
    
}

void loop() {


  /* 
   * ---------------------
   * LECTURA TARJETAS RFID
   * ---------------------
   */

   if (connected)
   {
      if (!instruccionesEnCurso)
        escribeTexto(25,100,2,"ESPERANDO INSTRUCCIONES",ORANGE);

      lecturaTarjetasRFID();
   }
     
   

}


/**************************************************************************
 * 
 * FUNCIONES
 * 
**************************************************************************/

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

/* 
 *  Función que lee el código de la tarjeta RFID y la compara con los códigos de movimientos y acciones almacenados
 *  Si hay coincidencia se enviará al escornabot vía bluetooth las acción a realizar
 */


void lecturaTarjetasRFID()
{

  byte codigoLeido[4] = {0,0,0,0};        // Almacenará el código leído
   
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
  
    for (byte i = 0; i < mfrc522.uid.size; i++) {
       
            codigoLeido[i]=mfrc522.uid.uidByte[i];          
    }
    
    // Terminamos la lectura de la tarjeta tarjeta actual
    mfrc522.PICC_HaltA(); 
    mfrc522.PCD_StopCrypto1();
  
    // Comparamos el código leído para ver si coincide con los códigos de movimientos y acciones
    if (compareArray(codigoLeido,tarjetaGirarIzquierda))
    {
      if (instruccionesEnCurso)
        accion("stop");
      else
        movimiento("izquierda",false);  
    }
    else if (compareArray(codigoLeido,tarjetaGirarDerecha))
    {
      if (instruccionesEnCurso)
        accion("stop");
      else
        movimiento("derecha",false);  
    }
    else if (compareArray(codigoLeido,tarjetaAvanzar))
    {
      if (instruccionesEnCurso)
        accion("stop");
      else
        movimiento("avanzar",false);  
    }
    else if (compareArray(codigoLeido,tarjetaRetroceder))
    {
      if (instruccionesEnCurso)
        accion("stop");
      else
        movimiento("retroceder",false);  
    }
    else if (compareArray(codigoLeido,tarjetaEjecutar))
    {
      if (instruccionesEnCurso)
        accion("stop");
      else
        accion("ejecutar");  
    }    
    else if (compareArray(codigoLeido,tarjetaResetear))
    {
      if (instruccionesEnCurso)
        accion("stop");
      else
        accion("resetear"); 
    }
               
   
  }
  
}

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

void enviarInstruccion(String instruccion)
{

  String newValue = "";
  
  if (instruccion == "izquierda")
      newValue = "w\n";
  else if (instruccion == "derecha")
      newValue = "e\n";
  else if (instruccion == "avanzar")
      newValue = "n\n";
   else if (instruccion == "retroceder")
      newValue = "s\n";
   else if (instruccion =="ejecutar")
      newValue = "g\n";
   else if (instruccion == "resetear" || instruccion == "stop")
      newValue = "r\n";

  if (connected) {
    // Set the characteristic's value to be the array of bytes that is actually a string.
    pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
  }
  
}
   

// Función para comparar dos arrays

boolean compareArray(byte array1[],byte array2[])
{
  if(array1[0] != array2[0])
    return(false);
  if(array1[1] != array2[1])
    return(false);
  if(array1[2] != array2[2])
    return(false);
  if(array1[3] != array2[3])
    return(false);
    
  return(true);
}


char* strToChar(String str) 
{
  int len = str.length() + 1;
  char* buf = new char[len];
  strcpy(buf, str.c_str());
  return buf;
}


void escribeTexto(uint16_t x, uint16_t y, uint8_t ts, char *text, uint16_t color) 
{
  M5.Lcd.setCursor(x, y);
  M5.Lcd.setTextSize(ts);
  M5.Lcd.setTextColor(color);
  M5.Lcd.setTextWrap(true);
  M5.Lcd.print(text);
}


void efectoLeds(String instruccion)
{

  uint32_t color;
  uint32_t rojo = leds.Color(255, 0, 0);
  uint32_t verde = leds.Color(0, 255, 0);
  uint32_t azul = leds.Color(0, 0, 255);
  uint32_t amarillo = leds.Color(255, 255, 0);
  uint32_t negro = leds.Color(0, 0, 0);
  uint32_t blanco = leds.Color(255, 255, 255);
  uint32_t naranja = leds.Color(255, 050, 0);
  uint32_t morado = leds.Color(150, 0, 220);

  if (instruccion == "izquierda")
    color = rojo;
  else if (instruccion == "derecha")
    color = verde;
  else if (instruccion == "avanzar")
    color = azul;  
  else if (instruccion == "retroceder")
    color = amarillo;  
  else if (instruccion == "ejecutar")
   color = blanco;
  else if (instruccion == "resetear")
   color = morado; 
  else if (instruccion == "stop")
   color = rojo;  
  else if (instruccion == "error")
   color = naranja;
  else if (instruccion == "fin ok")
   color = verde;
  else if (instruccion == "fin ko")
   color = naranja;
  for(int i=0;i<LEDS_NUM;i++)
  { 
    leds.setPixelColor(i, color); 
  }
  
  leds.show(); 
  
}



void decirFrase(String instruccion, bool ejecutando)
{

  /* 
  Locuciones presentes en los archivos mp3:
  
  0001.mp3: "Izquierda"
  0002.mp3: "Derecha"
  0003.mp3: "Avanzar"
  0004.mp3: "Retroceder"
  0005.mp3: "Ejecutar"
  0006.mp3: "Resetear"
  
  0007.mp3: "Girando a la izquierda"
  0008.mp3: "Girando a la derecha"
  0009.mp3: "Avanzando"
  0010.mp3: "Retrocediendo"
  
  0011.mp3: "¡comenzados!  
  0012.mp3 :"¡Vaya!, creo que me he perdido, ¿volvemos a intentarlo?
  0013.mp3 : "¡Hemos terminado!, ¡lo has hecho genial!"
  0014.mp3 : "¡Vaya!, no he llegado al final, ¿volvemos a intentarlo?
  0015.mp3 : "Parando el robot y reseteando movimientos"

  */
   
  delete file;
  delete out;
  delete mp3;
  mp3 = NULL;
  file = NULL;
  out = NULL;
  
  String mp3_fichero = "";
 
  if (instruccion == "izquierda")
  {
    if (ejecutando)
      mp3_fichero="0001.mp3";
    else
      mp3_fichero="0001.mp3";
  }
  else if (instruccion == "derecha")
  {
    if (ejecutando)
      mp3_fichero="0002.mp3";
    else
      mp3_fichero="0002.mp3";
  }
  else if (instruccion == "avanzar")
  {
    if (ejecutando)
      mp3_fichero="0009.mp3";
    else
      mp3_fichero="0003.mp3";
  }
  else if (instruccion == "retroceder")
  {
    if (ejecutando)
      mp3_fichero="0010.mp3";
    else
      mp3_fichero="0004.mp3";
  }
  else if (instruccion == "ejecutar")
      mp3_fichero="0005.mp3";
  else if (instruccion == "resetear")
      mp3_fichero="0006.mp3";
  else if (instruccion == "comenzar")
      mp3_fichero="0011.mp3";
  else if (instruccion == "error")
      mp3_fichero="0012.mp3";    
  else if (instruccion == "fin ok")
      mp3_fichero="0013.mp3"; 
  else if (instruccion == "fin ko")
      mp3_fichero="0014.mp3"; 
  else if (instruccion == "stop")
      mp3_fichero="0015.mp3"; 
          
  if (mp3_fichero != "")
  {

    file = new AudioFileSourceSD(strToChar("/mp3/" + mp3_fichero));
    id3 = new AudioFileSourceID3(file);
    out = new AudioOutputI2S(0, 0); // Output to builtInDAC
    out->SetPinout(12, 0, 2);
    out->SetOutputModeMono(true);
    mp3 = new AudioGeneratorMP3();
    mp3->begin(id3, out);
    for (;;) 
    {
       if (mp3->isRunning()) 
       {
        if (!mp3->loop())
        {
          mp3->stop();
          break;
        }
       }else{
        delay(100);
       }
    }
     
  }
     
}



void dibujarBoton(String instruccion) 
{
 
  int i;
  
  M5.Lcd.fillScreen(BLACK);
  
  if (instruccion == "izquierda")
    M5.Lcd.fillTriangle(60, 120, 260, 20, 260, 220, RED);
  else if (instruccion == "derecha")
    M5.Lcd.fillTriangle(260, 120, 60, 20, 60, 220, GREEN); 
  else if (instruccion == "avanzar")
    M5.Lcd.fillTriangle(160, 20, 60, 220, 260, 220, BLUE);
  else if (instruccion == "retroceder")
    M5.Lcd.fillTriangle(160, 220, 60, 20, 260, 20, YELLOW);
  else if (instruccion == "ejecutar")
  {
    M5.Lcd.fillCircle(160, 120, 110, WHITE);
    escribeTexto(65,110,4,"EJECUTAR",BLACK);
  }
  else if (instruccion == "resetear")
  {
    M5.Lcd.fillCircle(160, 120, 110, MAGENTA);
    escribeTexto(65,110,4,"RESETEAR",WHITE);
  }
  else if (instruccion == "stop")
  {
    M5.Lcd.fillCircle(160, 120, 110, RED);
    escribeTexto(105,100,5,"STOP",WHITE);
  }
  else if (instruccion == "error")
  {
    M5.Lcd.fillCircle(160, 120, 110, ORANGE);
    M5.Lcd.fillRoundRect(150,50, 20, 100, 5, BLACK);
    M5.Lcd.fillCircle(160, 170, 12, BLACK);
}
  else if (instruccion == "fin ok")
  {
    M5.Lcd.fillCircle(160, 120, 110, 0x0560); //RGB565
    for (i=0;i<30;i++)
    {
      M5.Lcd.drawLine((110+i),120,160,(170-i),WHITE); 
      M5.Lcd.drawLine(160,(170-i),(220-i),70,WHITE);
    }
  }
  else if (instruccion == "fin ko")
  {
    M5.Lcd.fillCircle(160, 120, 110, ORANGE);
    M5.Lcd.fillRoundRect(150,50, 20, 100, 5, BLACK);
    M5.Lcd.fillCircle(160, 170, 12, BLACK);
  }
    
}


void ponerOK() 
{

  int i;
  
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.fillCircle(160, 120, 110, 0x0560); //RGB565
  for (i=0;i<30;i++)
  {
    M5.Lcd.drawLine((110+i),120,160,(170-i),WHITE); 
    M5.Lcd.drawLine(160,(170-i),(220-i),70,WHITE);
  }

  uint32_t verde = leds.Color(0, 255, 0);
  for(int i=0;i<LEDS_NUM;i++)
  { 
    leds.setPixelColor(i, verde); 
  }
  
  leds.show(); 
 
  
}


void movimiento(String movimiento, bool ejecutando)
{
    dibujarBoton(movimiento);
    efectoLeds(movimiento);
    
    if (ejecutando)
    {
      decirFrase(movimiento, ejecutando); 
    }
    else
    {
      enviarInstruccion(movimiento);
      decirFrase(movimiento, ejecutando); 
      delay(1500); 
      reset();
    }
   
}


void accion(String instruccion)
{
  
  if (instruccion == "ejecutar"){
  
    dibujarBoton(instruccion);
    efectoLeds(instruccion);   
    decirFrase(instruccion,false);
    delay(1000);
    reset();
    
    decirFrase("comenzar",false); 
    delay(1200);
    enviarInstruccion(instruccion);
    
    instruccionesEnCurso = true;
  
  }else if (instruccion == "resetear" || instruccion == "stop"){
  
    enviarInstruccion(instruccion);
    dibujarBoton(instruccion);
    efectoLeds(instruccion); 
    decirFrase(instruccion,false); 
    delay(1000);
    reset();

    if (instruccion == "stop")
      instruccionesEnCurso = false;
      
  
  }else if (instruccion == "error" || instruccion == "fin ok" || instruccion == "fin ko"){

    dibujarBoton(instruccion);
    efectoLeds(instruccion); 
    decirFrase(instruccion,false);
    delay(1000);
    reset();
    instruccionesEnCurso = false;
  
  }
    
}

void reset()
{
    leds.clear();
    leds.show();
    M5.Lcd.fillScreen(BLACK);
}


bool checkSDCard()
{
    sdcard_type_t Type = SD.cardType();

    if( Type == CARD_UNKNOWN || Type == CARD_NONE )
      return false;
       
    return true;
}

void botonConectar()
{

   int x,y,w,h;
   
   x=60;
   y=150;
   w=200;
   h=60;
   
   TouchButton boton(x,y,w,h);
   M5.Lcd.fillRoundRect(x,y, w, h, 20, ORANGE);
   escribeTexto(90,170,3,"CONECTAR",BLACK);

    while(1)
    {
        M5.update();
        if(boton.wasPressed())
        {
          beep();
          break;
        }
        delay(10);
    }
}

void escanear()
{
    BLEDevice::init("");
    // Escaneamos dispositivo periferico
    escribeTexto(5,50,2,"Buscando escornabot ...",WHITE);
    
    do
    {
      BLEScan* pBLEScan = BLEDevice::getScan();
      pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
      pBLEScan->setInterval(1349);
      pBLEScan->setWindow(449);
      pBLEScan->setActiveScan(true);
      pBLEScan->start(5, false);
      
    } while (doConnect == false);
 

}

void conectar()
{
    delay(1000);
    escribeTexto(5,90,2,"Conectando al robot ...",WHITE);
    
   // If the flag "doConnect" is true then we have scanned for and found the desired
   // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
   // connected we set the connected flag to be true.
   
    if (connectToServer()) {
      escribeTexto(290,90,2,"OK",GREEN);
    } else {
      escribeTexto(290,90,2,"KO",RED);
      efectoLeds("error");   
      while(1);
    }
  
 
}


void SpeakInit(bool _32bitsPerSample)
{
  M5.Axp.SetSpkEnable(true);
  InitI2SSpeaker(_32bitsPerSample);
}


void beep()
{
  size_t bytes_written = 0;
  i2s_write(Speak_I2S_NUMBER, bibiSig, 8820, &bytes_written, portMAX_DELAY);
}


bool InitI2SSpeaker(bool _32bitsPerSample)
{
    i2s_bits_per_sample_t BITS_PER_SAMPLE = I2S_BITS_PER_SAMPLE_16BIT;
    
    if (_32bitsPerSample)
      BITS_PER_SAMPLE = I2S_BITS_PER_SAMPLE_32BIT;
    
    // Deleting the Driver
    i2s_driver_uninstall(Speak_I2S_NUMBER);

    // Installing the Driver
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = 44100,
        .bits_per_sample = BITS_PER_SAMPLE,
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 2,
        .dma_buf_len = 128,
        .use_apll = false,
        .tx_desc_auto_clear = true
    };
  
    i2s_driver_install(Speak_I2S_NUMBER, &i2s_config, 0, NULL);

    // Setting Communication Pins
    // Once the driver is installed, configure physical GPIO pins to which signals will be routed.
    
    i2s_pin_config_t tx_pin_config = {
        .bck_io_num = CONFIG_I2S_BCK_PIN,
        .ws_io_num = CONFIG_I2S_LRCK_PIN,
        .data_out_num = CONFIG_I2S_DATA_PIN,
        .data_in_num = CONFIG_I2S_DATA_IN_PIN
    };
    
    i2s_set_pin(Speak_I2S_NUMBER, &tx_pin_config);
    
    i2s_set_clk(Speak_I2S_NUMBER, 44100, BITS_PER_SAMPLE, I2S_CHANNEL_MONO);

    return true;
}


void escornafid()
{
  
  M5.Lcd.drawChar(10 ,70, 'e' , WHITE, BLACK, 5);
  M5.Lcd.drawChar(40 ,70, 's' , WHITE, BLACK, 5);
  M5.Lcd.drawChar(70 ,70, 'c' , WHITE, BLACK, 5);
  M5.Lcd.drawChar(100 ,70, 'o' , WHITE, BLACK, 5);
  M5.Lcd.drawChar(130 ,70, 'R' , ORANGE, BLACK, 5);
  M5.Lcd.drawChar(160 ,70, 'n' , WHITE, BLACK, 5);
  M5.Lcd.drawChar(190 ,70, 'a' , WHITE, BLACK, 5);
  M5.Lcd.drawChar(220 ,70, 'F' , ORANGE, BLACK, 5);
  M5.Lcd.drawChar(250 ,70, 'I' , ORANGE, BLACK, 5);
  M5.Lcd.drawChar(280 ,70, 'D' , ORANGE, BLACK, 5);
 
  escribeTexto(80,120,2,"para MCORE",WHITE);
  escribeTexto(210,120,2,"2",RED);
  escribeTexto(200,210,2,"Ver 1.0",WHITE);

}
