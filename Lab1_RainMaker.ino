#include "RMaker.h" //libreria para el deesarrollo de la aplicación de ESP RainMaker
#include "WiFi.h" //libreria que permite la conectidad WiFi con la tarejta ESP32
#include "WiFiProv.h" //libreria para el aprovisamiento de WiFi
#include "DHT.h" //libreria para el manejo del sensor DHT11
#include <SimpleTimer.h> //libreria para el manejo de temporizadores
#include <wifi_provisioning/manager.h> //libreria para la gestión de credenciales

//valores por defecto de las variables
#define DEFAULT_RELAY_MODE false //valor por defecto para el relé
#define DEFAULT_Temperature 0 //valor por defecto para la temperatura
#define DEFAULT_Humidity 0 //valor por defecto para la humedad
#define DEFAULT_SET 20 //valor por defecto para el setPoint a la temperatura

float setPoint = DEFAULT_SET; //se le establece al setPonit el valor por defecto

//credenciales BLE para la conexión WiFi
const char *service_name = "ORBIN"; //se define el nombre de la red
const char *pop = "1053834012"; //se define la contraseña de la red

//definición de pines GPIO
static uint8_t gpio_reset = 0; //pin para el reset de la tarjeta
static uint8_t DHTPIN = 19; //pin para lectura de datos del sensor DHT
static uint8_t relePin = 18; //pin para el relé que activa el boombillo
static uint8_t ventilador_pin = 32; //pin para el TIP122 que activa el ventilador
static uint8_t LED1 = 22; //pin para el LED correspondiente a la temperatura
static uint8_t LED2 = 23; //pin para el LED correspondiente a la humedad
static uint8_t LED3 = 21; //pin para el LED en donde se puede editar el parametro de la temperatura

//definición de estados iniciales
bool led1_state = false; //se define el estado inicial del LED1
bool led2_state = false; //se define el estado incial del LED2
bool led3_state = false; //se define ele stado incial del LED3
bool vent_state = false; //se define el estado inicial del ventilador
bool bomb_state = false; //se define el estado inicial del bombillo
bool wifi_connected = 0; //se define el estado incial de la conexión WiFi

//definición de un objeto de tipo DHT
DHT dht(DHTPIN, DHT11);

//definición de un objeto de tipo timer
SimpleTimer Timer;

//definición de dispositivos
static TemperatureSensor temperature("Temperature"); //dispositivo de temperatura
static TemperatureSensor humidity("Humidity"); //dispositivo de humedad
static Switch my_switch("Bombillo", &relePin); //dispositivo del bombillo
static Switch my_switch1("ledTemp", &LED1); //dispositivo del LED de temperatura
static Switch my_switch2("ledHum", &LED2); //dispositivo del LED de humedad
static Switch my_switch3("ledAux", &LED3); //dispositivo del LED de setpoint
static Switch ventilador("Vent", &ventilador_pin); //dispositibo del ventilador


//evento para  la conexión WIFI
void sysProvEvent(arduino_event_t *sys_event)
{
  //switch para diferentes tipos de eventos
  switch (sys_event->event_id)
  {
  //inicio del aprovisamiento
  case ARDUINO_EVENT_PROV_START:
#if CONFIG_IDF_TARGET_ESP32
    Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on BLE\n", service_name, pop);
    printQR(service_name, pop, "ble");
#else
    Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on SoftAP\n", service_name, pop);
    printQR(service_name, pop, "softap");
#endif
    break;
    //si la conexión a la red es exitosa
  case ARDUINO_EVENT_WIFI_STA_CONNECTED:
    //se imprime un mensaje de que es exitosa
    Serial.printf("\nConnected to Wi-Fi!\n");
    wifi_connected = 1;
    delay(500);
    break;
  //caso para cuando hay recepción de credenciales  durante el aprovisamiento
  case ARDUINO_EVENT_PROV_CRED_RECV:
  {
    //impresión del mensaje que inidca que se han recibido credenciales
    Serial.println("\nReceived Wi-Fi credentials");
    Serial.print("\tSSID : ");
    Serial.println((const char *)sys_event->event_info.prov_cred_recv.ssid);
    Serial.print("\tPassword : ");
    Serial.println((char const *)sys_event->event_info.prov_cred_recv.password);
    break;
  }
  //caso para la inicialización del aprovisamiento
  case ARDUINO_EVENT_PROV_INIT:
    wifi_prov_mgr_disable_auto_stop(10000);
    break;
  //caso del aprovisamiento de credenciales
  case ARDUINO_EVENT_PROV_CRED_SUCCESS:
    Serial.println("Stopping Provisioning!!!");
    wifi_prov_mgr_stop_provisioning();
    break;
  }
}

void write_callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx){
  //obtiene el nombre del dispositivo
  const char *device_name = device->getDeviceName();
  //imprime el nombre del dispositivo en la consola
  Serial.println(device_name);
  //se obtiene el nombre del parametro
  const char *param_name = param->getParamName();
  
  //caso en que el dispositivo sea bombillo
  if (strcmp(device_name, "Bombillo") == 0)
  {
    if (strcmp(param_name, "Power") == 0)
    {
      Serial.printf("Received value = %s for %s - %s\n", val.val.b ? "true" : "false", device_name, param_name);
      bomb_state = val.val.b;
      (bomb_state == false) ? digitalWrite(relePin, HIGH) : digitalWrite(relePin, LOW);
      param->updateAndReport(val);
    }
  }

  //caso en el que el dispositivo es "ledTemp"
  else if (strcmp(device_name, "ledTemp") == 0)
  {
    //compruebas si el parametro es power
    if (strcmp(param_name, "Power") == 0)
    {
      //se actualiza el estado del LED 
      Serial.printf("Received value = %s for %s - %s\n", val.val.b ? "true" : "false", device_name, param_name);
      led1_state = val.val.b;
      (led1_state == false) ? digitalWrite(LED1, LOW) : digitalWrite(LED1, HIGH);
      //se informa sobre el cambio de parametro
      param->updateAndReport(val);
    }
  }
 
  //caso en el que el dispositivo es "ledHum"
  else if (strcmp(device_name, "ledHum") == 0)
  {
    //comprueba si el parametro es power
    if (strcmp(param_name, "Power") == 0)
    {
      Serial.printf("Received value = %s for %s - %s\n", val.val.b ? "true" : "false", device_name, param_name);
      led2_state = val.val.b;
      (led2_state == false) ? digitalWrite(LED2, LOW) : digitalWrite(LED2, HIGH);
      //se informa sobre el cambio de parametro
      param->updateAndReport(val);
    }
  }

  else if(strcmp(device_name, "ledAux") == 0) {
    //Serial.printf("Fan = %s\n", val.val.b? "true" : "false");
    if(strcmp(param_name, "Power") == 0) {
      //int fan_on_off_value = val.val.b;
      //Serial.printf("\nReceived value = %d for %s - %s\n", fan_on_off_value, device_name, param_name);
      //param->updateAndReport(val);
    }else if (strcmp(param_name, "Level") == 0) {
      int new_setpoint = val.val.i;
      Serial.printf("\nReceived value = %d for %s - %s\n", new_setpoint, device_name, param_name);
      //led2_state = val.val.b;
      //(led2_state == false) ? digitalWrite(LED2, LOW) : digitalWrite(LED2, HIGH);
      setPoint = new_setpoint; // Actualiza la variable global setPoint con el nuevo valor del setpoint
      delay(15); // waits 15ms for the servo to reach the position
      //param->updateAndReport(val);
      }
    }

  //caso en el que el dispositivo es "Vent"
  else if (strcmp(device_name, "Vent") == 0)
  {
    //comprueba si el parametros es power
    if (strcmp(param_name, "Power") == 0)
    {
      Serial.printf("Received value = %s for %s - %s\n", val.val.b ? "true" : "false", device_name, param_name);
      vent_state = val.val.b;
      (vent_state == false) ? digitalWrite(ventilador_pin, LOW) : digitalWrite(ventilador_pin, HIGH);
      //se informa sobre el cambio de parametro
      param->updateAndReport(val);
    }
  }
}

void setup()
{

  Serial.begin(115200);

  //configuracion de los pines GPIO
  pinMode(gpio_reset, INPUT); //se define el pin de reset como entrada
  pinMode(relePin,OUTPUT); //se define el pin del relé como salida
  pinMode(ventilador_pin, OUTPUT); //se define el pin del ventilador como salida
  pinMode(LED1, OUTPUT); //se define el pin del LED1 como salida
  pinMode(LED2, OUTPUT); //se define el pin del LED2 como salida
  pinMode(LED3, OUTPUT); //se define el pin del LED3 como salida


  // Estados GPIO iniciales 
  digitalWrite(relePin, bomb_state); //estado inicial del bombillo
  digitalWrite(LED1, led1_state); //estado inicial del LED1
  digitalWrite(LED2, led2_state); //estado inicial del LED2
  digitalWrite(LED3, led3_state); //estado inicial del LED3
  digitalWrite(ventilador_pin, vent_state); //estado inical del ventilador

  dht.begin(); //inicialización del sensor DHT

  Node my_node; //declaración del nodo

  //se establece el nombre del nodo a visualizar en la app 
  my_node = RMaker.initNode("Monitoreo de ambiente y control de dispositivos"); 

  //creación del parametro slide para establecer la temperatura
  Param level_param("Level", "custom.param.level", value(DEFAULT_SET), PROP_FLAG_READ | PROP_FLAG_WRITE);
  level_param.addBounds(value(17), value(25), value(1)); //se establece un rango entre 17 y 25
  level_param.addUIType(ESP_RMAKER_UI_SLIDER); //se añade el elemento de interfaz tipo slide
  my_switch3.addParam(level_param); //se añade al parametro level al dispositivo de temperatura

  my_switch.addCb(write_callback); //callback para el switch  del bombillo
  my_switch1.addCb(write_callback); //callback para el pin del LED1
  my_switch2.addCb(write_callback); //callback para el pin del LED2
  my_switch3.addCb(write_callback); //callback para el pin del LED3
  ventilador.addCb(write_callback); //callback para el pin del ventilador
  //temperature.addCb(write_callback);

  //se añaden todos los dispositivos al nodo
  my_node.addDevice(humidity); //añade dispositivo de humedad
  my_node.addDevice(my_switch); //añade el dispositivo del bombillo
  my_node.addDevice(my_switch1); //añade el dispositivo del LED1
  my_node.addDevice(my_switch2); //añade e dispositivo del LED2
  my_node.addDevice(my_switch3); //añade el dispostivo del LED3
  my_node.addDevice(ventilador); //añade el dispostivo del ventilador
  my_node.addDevice(temperature); //añade el dispositivo de temperatura

  //habilitar la actualizacion OTA para el dispositivo
  RMaker.enableOTA(OTA_USING_PARAMS);
  //habilitar el servicio de zona horaria
  RMaker.enableTZService();
  //habilita la funcionalidad del programa en ESP_Rainmaker
  RMaker.enableSchedule();

  //impreme en pantalla 
  Serial.printf("\nStarting ESP-RainMaker\n");
  RMaker.start();

  // Temporizador para el envío de datos del sensor
  Timer.setInterval(3000);
  
  //manejo de eventos WiFi
  WiFi.onEvent(sysProvEvent);

//identificar el tipo del dispositivo 
#if CONFIG_IDF_TARGET_ESP32
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM, WIFI_PROV_SECURITY_1, pop, service_name);
#else
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_SOFTAP, WIFI_PROV_SCHEME_HANDLER_NONE, WIFI_PROV_SECURITY_1, pop, service_name);
#endif
  //configura el estado del pin asociado al bombillo
  digitalWrite(relePin, bomb_state);
  //actualizar y reportar el parametro asociado a power
  my_switch1.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, bomb_state);
  //imprime el estado del bombillo en el puerto serial
  Serial.printf("El estado del bombillo es %s \n", bomb_state? "ON" : "OFF");
}

void loop()
{
  // si el temporizador está listo se envían los datos del sensor
  if (Timer.isReady() && wifi_connected)
  {
    Serial.println("Sending Sensor's Data");
    Send_Sensor(); // se llama a la función que envía los parámetros del sensor
    Timer.reset(); //se reincia el temporizador
  }

  // lectura del pin GPIO0 (botón externo para reiniciar el dispositivo)
  if (digitalRead(gpio_reset) == LOW)
  { //se identifica que se presiono el boton
    Serial.printf("Reset Button Pressed!\n");
    delay(100);
    int startTime = millis();
    while (digitalRead(gpio_reset) == LOW)
      delay(50);
    int endTime = millis();

    if ((endTime - startTime) > 10000)
    {
      //si se presiona mas de 10 segundos se resetea todo
      Serial.printf("Reset to factory.\n");
      wifi_connected = 0;
      RMakerFactoryReset(2);
    }
    else if ((endTime - startTime) > 3000)
    {
      Serial.printf("Reset Wi-Fi.\n");
      wifi_connected = 0;
      // If key pressed for more than 3secs, but less than 10, reset Wi-Fi
      RMakerWiFiReset(2);
    }
  }
  delay(100);
}

//función que envia los valores del sensor
void Send_Sensor(){
  //lectura de la humedad
  float h = dht.readHumidity();
  //lectura de la temperatura
  float t = dht.readTemperature();

  //impresión en el puerto serial de los valores establecidos
  Serial.print("Temperature - ");
  Serial.println(t);
  Serial.print("Humidity - ");
  Serial.println(h);
  Serial.print("Set Point - ");
  Serial.println(setPoint);

  //se reporta y cambia el nuevo parametro de temperatura
  temperature.updateAndReportParam("Temperature", t);
  //se reporta y cambia el nuevo parametro de humedad
  humidity.updateAndReportParam("Temperature", h);

  //establecer acciones para el control en base a la sensorica
  if (t > setPoint)
  {
    //activa led temperatura
    digitalWrite(LED1, HIGH);
    //activa ventilador
    digitalWrite(ventilador_pin, HIGH);
    //se actualizan los parametros
    my_switch1.updateAndReportParam("Power", HIGH);
    ventilador.updateAndReportParam("Power", HIGH);
  }
  else
  {
    //desactiva led temperatura
    digitalWrite(LED1, LOW);
    //desactiva el ventilador
    digitalWrite(ventilador_pin, LOW);
    //actualización del parametro
    my_switch1.updateAndReportParam("Power", LOW);
    ventilador.updateAndReportParam("Power", HIGH);
  }

  if (h > 65)
  {
    //se activa el led de humedad
    digitalWrite(LED2, HIGH); 
    my_switch2.updateAndReportParam("Power", HIGH);
  }
  else
  {
    //se desactiva el led de humedad
    digitalWrite(LED2, LOW);
    my_switch2.updateAndReportParam("Power", LOW);
  }
}