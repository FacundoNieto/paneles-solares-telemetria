/*===============================================
  Multitarea FreeRTOS & MQTT (PVLib Azimut)
  ===============================================*/

#include <WiFi.h>
#include <PubSubClient.h> // biblioteca de comunicación mqtt
#include <Wire.h>
#include <MechaQMC5883.h>
#include <PZEM004Tv30.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


// Constantes para el WiFi:
const char *ssid = "Zhone_F861";
const char *password = "znid314350433";

// Constantes para el broker MQTT
const char *mqtt_broker = "192.168.1.226";
const int   mqtt_port = 1883;
const char *mqtt_username = "";
const char *mqtt_password = "";

// Tópicos
const char *modo_topic = "control/modo";

const char *servo_topic_azimut = "control/motor/azimut";
const char *brujula_topic = "control/motor/brujula";
const char *motor_manual_topic = "control/motor/manual";

const char *corriente_cc_topic = "sensores/cc/corriente";
const char *volt_crud_corriente_topic = "sensores/cc/corriente/vcrudo";
const char *volt_medido_corriente_topic = "sensores/cc/corriente/vmedido";
const char *tension_cc_topic = "sensores/cc/tension";
const char *volt_crud_tension_topic = "sensores/cc/tension/vcrudo";
const char *volt_medido_tension_topic = "sensores/cc/tension/vmedido";
const char *potencia_cc_topic = "sensores/cc/potencia";

const char *tension_ca_topic = "sensores/ca/tension";
const char *corriente_ca_topic = "sensores/ca/corriente";
const char *potencia_ca_topic = "sensores/ca/p_activa";
const char *aparente_ca_topic = "sensores/ca/p_aparente";
const char *reactiva_ca_topic = "sensores/ca/p_reactiva";
const char *energia_ca_topic = "sensores/ca/energia";
const char *frecuencia_ca_topic = "sensores/ca/frecuencia";
const char *factor_potencia_topic = "sensores/ca/fp";

const char *will_esp32_topic = "status/esp32";
const char *will_message = "OFF";  // last will
int will_qos = 2;

// Objetos WiFi y MQTT
WiFiClient   espClient;
PubSubClient mqtt_client(espClient);

int automatico = 1; // Variable global para el control del modo de operación
int servo_posicion_manual = 0;

/*======================================================*/
/*           Señal PWM para el Servomotor               */
/*======================================================*/
const int servo = 19; //Pin de Salida PWM para el servomotores
const int canal_pwm_servo = 0; // ESP32 tiene 16 canales que pueden generar 16 señales pwm independientes
const int frec_pwm_servo = 50; //En Hz (el esp32 soporta hasta 40kHz)
const int resolucion = 10;  // En bits (el esp32 soporta hasta una resolución de 16 bits)
// el valor máximo del duty_cycle que puede dar el ESP32 está determinado por la resolución 
const int max_duty_cycle = (int)(pow(2, resolucion) - 1);
int pausa = 60; // milisegundos de pausa en cada paso de 1° del servo
int duty_actual = 30;
float Kp = 0.3; // Ganancia proporcional

// Brújula digital (magnetómetro)
MechaQMC5883 qmc;
int x, y, z;
int azimuth;
// Variables compartidas por los hilos
int azimut_brujula = 40;
int azimut_pvlib = 40; // recibido por MQTT

// Pines de UART para el sensor de CA
#define RXD2 16
#define TXD2 17
// Crear una instancia del objeto PZEM usando Serial2
PZEM004Tv30 pzem(Serial2, RXD2, TXD2);

/*=================================================================*/
/*              Sensores de tensión y corriente CC                 */
/*=================================================================*/
const int   pinCorriente = 36;
const float sensibilidadACS712 = 0.185;
float corriente_cc = 0;

const int   pinVoltaje = 39;
const float R1= 6.8, R2 = 1.0; //resisores del divisor de tensión
const float factorDivisorTension = R2/(R1+R2);
float tension_cc   = 0;

// Mutex para proteger lecturas
SemaphoreHandle_t xMutexSensores;

// Prototipos de tareas
void tareaMqttLoop(void* pvParameters);
void tareaLeerSensores(void* pvParameters);
void tareaRevisarPosicion(void* pvParameters);

// Prototipos de funciones auxiliares
void configurar_wifi();
void conexion_mqtt();
void callback(char* topic, byte* payload, unsigned int length);
void enviar(float dato, const char* topico, char* unidad);
void leerBrujula(int ciclos);
void leerCorriente(int nroDeMuestras);
void leerTension(int nroDeMuestras);
void calcularPotencia();
void leerCA();
void revisar_posicion(int az_brujula, int az_pvlib);

void setup(){
  Wire.begin(21, 22); //pines i2c magnetómetro
  Serial.begin(9600);

  /*======================================================*/
  /*            Conexión WiFi y MQTT (inicial)            */
  /*======================================================*/
  configurar_wifi();
  mqtt_client.setServer(mqtt_broker, mqtt_port);
  mqtt_client.setCallback(callback);
  conexion_mqtt();
  
  mqtt_client.subscribe(modo_topic);
  mqtt_client.subscribe(servo_topic_azimut); 
  mqtt_client.subscribe(motor_manual_topic);
 
  /*======================================================*/
  /*        Inicializar librería del magnetómetro         */
  /*======================================================*/
  qmc.init();

  /*======================================================*/
  /*                 PWM del Servomotor                   */
  /*======================================================*/  
  // Configurar los canales pwm (0-15), la frecuencia del ciclo de trabajo pwm, y la resulución (1 - 16 bits) 
  // ledcSetup(uint8_t channel, double freq, uint8_t resolution_bits);
  ledcSetup(canal_pwm_servo, frec_pwm_servo, resolucion);
  ledcAttachPin(servo, canal_pwm_servo);
  ledcWrite(canal_pwm_servo, duty_actual);

  /*======================================================*/
  /*                   Tareas                             */
  /*======================================================*/
  // Crear mutex
  xMutexSensores = xSemaphoreCreateMutex();

  // Crear tareas en Core 1
  xTaskCreatePinnedToCore(tareaMqttLoop,    // función
                          "MQTT_Loop",      // nombre
                          2048,             // stack
                          NULL,             // parámetro
                          2,                // prioridad
                          NULL,             // handle
                          1);               // core

  xTaskCreatePinnedToCore(tareaLeerSensores,
                          "Leer_Sensores",
                          4096,
                          NULL,
                          1,
                          NULL,
                          1);

  xTaskCreatePinnedToCore(tareaRevisarPosicion,
                        "Revisar_Pos",
                        4096,
                        NULL,
                        1,
                        NULL,
                        1);
}

void loop(){
  // vacío: todo corre en tareas
}


/*================================================*/
/*   Tarea 1: mantiene MQTT y procesa callback    */
/*================================================*/
void tareaMqttLoop(void* pvParameters){
  for(;;){
    if (!mqtt_client.connected()){
      conexion_mqtt();
      mqtt_client.subscribe(servo_topic_azimut);
    }
    mqtt_client.loop();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}


/*== ===============================================*/
/*  Tarea 2: cada 1 segundo lee la corriente CC,    */
/*  tensión CC, calcula potencia y publica          */
/*==================================================*/

void tareaLeerSensores(void* pvParameters){
  const int intervalo = 1000; // ms
  for(;;){
    Serial.printf("\n---------------------------\n");
    leerBrujula(1);
    leerCorriente(50);
    leerTension(50);
    calcularPotencia();
    leerCA();
    Serial.printf("\n---------------------------\n");

    vTaskDelay(intervalo / portTICK_PERIOD_MS);
  }
}

/*=================================================================*/
/*   Tarea 3: cada 0.15 segundos revisa la posición del prototipo  */
/*=================================================================*/
void tareaRevisarPosicion(void* pvParameters) {
  const int intervalo = 150;  // chequea cada 0.15 s
  for (;;) {
    if(automatico){
      int bru, pv;
      // Activación del mutex para utilizar correctamente variables que se usan en otras tareas:
      if (xSemaphoreTake(xMutexSensores, portMAX_DELAY)) {
        bru = azimut_brujula;
        pv  = azimut_pvlib;
        xSemaphoreGive(xMutexSensores);
      }
      // Llamada a tu función de control
      revisar_posicion(bru, pv);
    }
    else{
      int nuevo_duty_manual = 30 + (4.45f * servo_posicion_manual) + 0.5f;  
      moverServo(duty_actual, nuevo_duty_manual);
      duty_actual = nuevo_duty_manual;
    }

    // Asegura que no bloquee todo el CPU
    vTaskDelay(intervalo / portTICK_PERIOD_MS);
  }
}


/*======================================================*/
/*   Callback MQTT: recibe instrucciones de Node-Red    */
/*======================================================*/
void callback(char *topic, byte *payload, unsigned int length){
  char msg[length + 1];
  for(int i = 0; i < length; i++)
    msg[i] = (char)payload[i];
  msg[length] = '\0';

  Serial.printf("\n--- Mensaje recibido ---\nTópico: %s\nMensaje: %s\n", topic, msg);

  if (strcmp(topic, servo_topic_azimut) == 0){
    azimut_pvlib = atoi(msg);
    if ((azimut_pvlib < 270) && (azimut_pvlib > 90))
      azimut_pvlib = 90;
  } 
  else if(strcmp(topic, modo_topic) == 0){
    (strcmp(msg, "Automatico") == 0) ?  automatico = 1 : automatico = 0;
  }
  else if(strcmp(topic, motor_manual_topic) == 0){
    servo_posicion_manual = constrain(atoi(msg)*0.1 , 0, 18);
  }
}

/*======================================================*/
/*           Leer magnetómetro y publicar               */
/*======================================================*/
void leerBrujula(int ciclos){
  azimut_brujula = 0;
  for(int i = 1; i<= ciclos; i ++){
    qmc.read(&x, &y, &z,&azimuth);
    azimut_brujula += azimuth;
  }
  azimut_brujula = azimut_brujula / ciclos;
  enviar((float)azimut_brujula, brujula_topic, "°");
}

/*======================================================*/
/*           Leer Corriente CC y publicar               */
/*======================================================*/
void leerCorriente(int nroDeMuestras){
  int valor_crudo = 0;
  for(int i = 0; i < nroDeMuestras; i++){
    valor_crudo += analogRead(pinCorriente);
  }
  valor_crudo /= nroDeMuestras;
  float vMed = valor_crudo * 3.300f / 4095.0f;
  //2.27 es la tensión de salida en vacío que mide el CAD del esp32  
  float I = (vMed - 2.27f) / sensibilidadACS712;
  if (fabs(I) < 0.06f) I = 0;
  corriente_cc = I;

  // Publicar
  enviar(corriente_cc, corriente_cc_topic, "A");
  enviar(valor_crudo, volt_crud_corriente_topic, "z");
  enviar(vMed, volt_medido_corriente_topic, "z");
}


/*======================================================*/
/*            Leer Tensión CC y publicar                */
/*======================================================*/
void leerTension(int nroDeMuestras){
  int valor_crudo = 0;
  for(int i = 0; i < nroDeMuestras; i++){
    valor_crudo += analogRead(pinVoltaje);
  }
  valor_crudo /= nroDeMuestras;
  float vMed = valor_crudo * 3.300f / 4095.0f;
  tension_cc = (vMed / factorDivisorTension) + 1.0f;

  // Publicar
  enviar(tension_cc, tension_cc_topic, "V");
  enviar(valor_crudo, volt_crud_tension_topic, "x");
  enviar(vMed, volt_medido_tension_topic,"x");
}

/*======================================================*/
/*          Calcular y publicar potencia CC             */
/*======================================================*/
void calcularPotencia(){
  float P = tension_cc * corriente_cc;
  enviar(P, potencia_cc_topic, "W");
}

void leerCA(){
  // Leer los valores del PZEM-004T
  float tension_ca = pzem.voltage();
  enviar(tension_ca, tension_ca_topic, "v");
  float corriente_ca = pzem.current();
  enviar(corriente_ca, corriente_ca_topic, "a");
  float potencia_ca = pzem.power();
  enviar(potencia_ca, potencia_ca_topic, "w");
  float energia = pzem.energy();
  enviar(energia, energia_ca_topic, "j");
  float frecuencia = pzem.frequency();
  enviar(frecuencia, frecuencia_ca_topic, "h");
  float factor_potencia = pzem.pf();
  enviar(factor_potencia, factor_potencia_topic, "c");
  float aparente_ca = potencia_ca/factor_potencia;
  enviar(aparente_ca, aparente_ca_topic, "s");
  float reactiva_ca = potencia_ca * factor_potencia; 
  enviar(reactiva_ca, reactiva_ca_topic, "q");
}

// ---------------------------------------------------------
// Enviar string por MQTT y Serial
void enviar(float dato, const char *topico, char *unidad){
  char datoStr[16];
  snprintf(datoStr, sizeof(datoStr), "%.2f", dato);
  if(unidad == "°")
    Serial.printf("Brújula: %d °\n", (int)dato);
  else if (unidad == "A")
    Serial.printf("Corriente Generada: %.2f [A]\n", dato);
  else if (unidad == "V")
    Serial.printf("Tensión Generada: %.2f [V]\n", dato);
  else if (unidad == "W")
    Serial.printf("Potencia Generada: %.2f [W]\n", dato);
  else if(unidad == "d")
    Serial.printf("Duty cycle pw+m servo: %d\n", (int)dato);
  else if (unidad == "v")
    Serial.printf("Tensión CA: %.2f [V]\n", dato);
  else if (unidad == "a")
    Serial.printf("Corriente CA: %.2f [A]\n", dato);
  else if (unidad == "w")
    Serial.printf("Potencia Activa CA: %.2f [W]\n", dato);
  else if (unidad == "j")
    Serial.printf("Energía CA: %.2f [J]\n", dato);
  else if (unidad == "h")
    Serial.printf("Frecuencia CA: %.2f [Hz]\n", dato);
  else if (unidad == "c")
    Serial.printf("Factor de Potencia: %.2f\n", dato);
  else if (unidad == "s")
    Serial.printf("Potencia Aparente S: %.2f [VA]\n", dato);
  else if (unidad == "q")
    Serial.printf("Potencia Reactiva Q: %.2f [VAR]\n", dato);
  
  mqtt_client.publish(topico, datoStr, true);
}

void revisar_posicion(int az_brujula, int az_pvlib){ 
  char mensaje[64];
  int nuevo_duty_cycle = 26;
  float error = az_brujula - az_pvlib;
  // Corregir error cuando pvlib y la brújula apuntan a distintos cuadrantes (queda un valor entre 0 y 180)
  //(BRÚJULA = Nor-Este && PVLIB = Nor-Oeste)
  if((error >= -359 && error <= -180)){
    error = (359 - az_pvlib) + az_brujula;
  }
  //(BRÚJULA = Nor-Oeste && PVLIB = Nor-Este)
  else if((error >= 180 && error <= 359)){
    error = -((359 - az_brujula) + az_pvlib); //se fuerza a que sea negativo
  }
  snprintf(mensaje, sizeof(mensaje), "az_brujula = %d | azimut_pvlib = %d | error = %f", 
                  az_brujula, az_pvlib, error);
  Serial.printf(mensaje);
  mqtt_client.publish("debug/borrar", mensaje, true);
  int correccion = Kp * error; // 0.3 * error  
  if (error > 6){ // girar a la izq, pvlib calcula un sol más al Nor-Oeste que lo que mide la brújula
    nuevo_duty_cycle = constrain(duty_actual + abs(correccion), 30, 128); // abs() quita los decimales y lo vuelve positivo
    Serial.printf("\nduty_actual = %d, nuevo_duty_cycle = %d\n", duty_actual, nuevo_duty_cycle);
  }
  else if(error < (-6)){ // girar a la der, pvlib calcula un sol más al Nor-Este que lo que mide la brújula
    nuevo_duty_cycle = constrain(duty_actual - abs(correccion), 30, 128);
    Serial.printf("\nduty_actual = %d, nuevo_duty_cycle = %d\n", duty_actual, nuevo_duty_cycle);
  }
  else{
    nuevo_duty_cycle = duty_actual; //no hacer ningún cambio
  }
  moverServo(duty_actual, nuevo_duty_cycle);
  duty_actual = nuevo_duty_cycle;
}

void moverServo(int inicial, int final){
  if(inicial < final){
    for (int pos = inicial; pos <= final; pos++) {
      ledcWrite(canal_pwm_servo, pos);
      // Detiene 'pausa' ms, pero cediendo CPU a otras tareas:
      vTaskDelay(pausa / portTICK_PERIOD_MS);
      Serial.printf("\nvariable nuevo_duty_cycle = %d\n", pos);
    }
  }
  else{
    for(int pos = inicial; pos >= final; pos --){
      ledcWrite(canal_pwm_servo, pos);
      vTaskDelay(pausa / portTICK_PERIOD_MS);
      Serial.printf("\nvariable nuevo_duty_cycle = %d\n", pos);
    }
  }  
}

// ---------------------------------------------------------
// Configurar y reconectar WiFi
void configurar_wifi(){
  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nConectado a la red WiFi. IP: %s\n", WiFi.localIP().toString().c_str());
}

// ---------------------------------------------------------
// Conectar/Reconectar MQTT
void conexion_mqtt(){
  while (!mqtt_client.connected()){
    String client_id = "esp32-client-" + String(WiFi.macAddress());
    Serial.printf("Conectado al broker MQTT. Client ID: %s\n", client_id.c_str());
    if (mqtt_client.connect(client_id.c_str(),
                            mqtt_username, mqtt_password,
                            will_esp32_topic, will_qos,
                            true, will_message, false)){
      Serial.println("Conectado al broker MQTT");
      mqtt_client.publish(will_esp32_topic, "ON", true);
    } else {
      Serial.printf("Error conectando al broker MQTT: rc=%d, reintentando en 2s\n", mqtt_client.state());
      delay(2000);
    }
  }
}
