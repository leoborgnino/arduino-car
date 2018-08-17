#include <TimerThree.h>
#include <TimerOne.h>
#include <Wire.h>
#include "MPU6050.h"
#include "Interrupciones.h"

/********************************
 *        Constantes            *
/********************************/

const int   INTE0              = 2;
const int   INTE1              = 3;
const long  BAUD_RATE         = 115200; 
const int   CONTROL_PERIOD     = 20;
const long  TIME_SAMPLE        = 25000;
const long  TIME_SAMPLE2       = 200;
const float DELTA_T            = (TIME_SAMPLE * CONTROL_PERIOD)/(1000.0*1000.0);
const float GIRO_MAX           = ROTACION_VOLANTE/2;
const float GIRO_MIN           = 5.0;
const float GIRO_MIN_ITER      = 10.0;
const float RUIDO_ROTACION     = 0.05;
const int   MICRO_ADDR         = 10;
const float PPV                = 36.0;
const float LONG_ARC           = 2.0 * 3.14 * 10.0; //longitud de arco de la rueda en cm
const float DIST_PULS          = LONG_ARC / PPV;
const int   LIMITE_DESVIO      = 2;
const int   PWM1               = 10;//Motor Principal
const int   PWM2               = 9;//Motor Volante
const float LIMITE_REVERSA     = 20;
const float LIMITE_GIRO        = 10;
const float ROTACION_VOLANTE   = 52.0 ;
const int   PULSOS_VOLANTE     = 11;
const float PASO_VOLANTE       = ROTACION_VOLANTE / PULSOS_VOLANTE;
const float GIRO_MOV           = PASO_VOLANTE * 4.0;
const float GIRO_LIMITE        = PASO_VOLANTE * 4.0;
const int   VOLANTE_CENTRADO   = 4;
const float GIRO_LEVE          = PASO_VOLANTE * 3.0;
const float REVERSA_MIN        = 45.0;
const float REVERSA_MAX        = 40.0;
const float ULTR_LIMITE        = 40.0;
const int   N_ULTR_SENSOR      =    3;
const int   ULTR_PERIOD        =    1;
const int   ULTRA_TRIGER       =   A1;
const int   ULTRA_ECHO         =   A3;
const int   ULTRB_TRIGER       =   A4;
const int   ULTRB_ECHO         =   17;
const int   ULTRC_TRIGER       =   A2;//Check
const int   ULTRC_ECHO         =   16;//Check
const int   MODO_ULTRASONIDO   =    1;
const int   MUESTRAS_DETECCION =    3;
const int   LIMITE_MUESTRAS    =    2;
const int   PIN_ALARM          =   A8;
const int   ALARM_PERIOD       =   15;
const int   N_BEEPS            =    1;
const float SATURACION_INTEGRADOR = 30.0;


/********************************
 *    Variables Globales        *
/********************************/

// Banderas
int flag_uart                = 0;
int flag_accion              = 0;
int flag_centrar_vehiculo    = 0;
int flag_cntrl_vel           = 0;
int flag_back                = 0;
int flag_timer               = 0;
int flag_girar_volante       = 0;
int flag_mover               = 0;
int flag_rotacion            = 0;
int flag_linea_recta         = 0;
int flag_giro_leve           = 0;
int flag_reversa_corta       = 0;
int flag_ultrasonido         = 0;
int flag_ultr_request        = 0;
int ultr_flag_start          = 0;

// Variables de Comunicacion Serie
unsigned char  SerRx;
int   data_len_rx         = 0;
unsigned char  data_rec[10];
char  mystring[100];
char  buff[11][7];
float datos[7];

int  respuestaid_mpu      =  0;

// Variables Acelerometro
float giro_z_instantaneo     = 0;
float offset[6]          = {0, 0, 0, 0, 0, 0};

// Variables Interrupciones
int contador_pid        = 0;
int respuestaid_plan    = 0;

float grados_por_rotar  = 0;
float centrar_vehiculo  = 0;

//Variables Movimiento
int  enderezar_volante    = 0;
int  esperar_volante      = 0;
int  completar_movimiento = 0;
int  posicion_volante     = 0;
int  sentido_giro         = 0;
int  contador_movimiento  = 0;
int  sentido_temp         = 0;
int  grados_volante_max   = 0;
int  objeto_detectado     = 0;
unsigned int distancia_max        = 0;
unsigned int distancia_shifted = 0;

float  distancia_temp[2]      = {0.0, 0.0};
float  velocidad_ref          = 3.0; // Velocidad crucero en m/s
float  movimiento[2]          = {0.0, 0.0};
float  vel_inicial            = 55.0;
float  distancia_temp_d[2]    = {0.0, 0.0};
float  p_controller[2]        = {0.0, 0.0};
float  d_controller[2]        = {0.0, 0.0};
float  i_controller[2]        = {0.0, 0.0};
float  prev_error[2]          = {0.0, 0.0};
float  valor_giro_instantaneo = 0;
float  valor_giro_total       = 0;
float  grados_objetivo        = 0;
float  grados_a_rotar         = 0;
double velocidad_temp         = 0.0;
double velocidad_abs          = 0.0;
double distancia_abs          = 0.0;
double distancia_abs_d        = 0.0;
double kd                     = 1.0;
double ki                     = 0.20;
double kp                     = 10.0;

// Variables Ultrasonido

double ultr_distance[N_ULTR_SENSOR];
double distancia_objeto[N_ULTR_SENSOR];
long   ultr_start_time[N_ULTR_SENSOR];
int    contador_obstaculo[N_ULTR_SENSOR][MUESTRAS_DETECCION];
int    contador_libre[N_ULTR_SENSOR][MUESTRAS_DETECCION];
int    contador_deteccion[N_ULTR_SENSOR];
int    contador_no_deteccion[N_ULTR_SENSOR];
int    flag_no_objeto_detectado[N_ULTR_SENSOR];
int    flag_objeto_detectado[N_ULTR_SENSOR];

void setup()
{
  // Configuracion Timer 1
  Timer1.initialize(double(TIME_SAMPLE)); // WARNING: Depende de la versión del compilador  // Dispara cada TIME_SAMPLE ms
  Timer1.attachInterrupt(ISR_Timer);     // Activa la interrupcion y la asocia a ISR_Timer

  // Configuracion Timer 1
  Timer3.initialize(double(TIME_SAMPLE2));
  Timer3.attachInterrupt(ISR_Timer3);

  // TRISX
  pinMode(ULTRA_TRIGER,OUTPUT);
  pinMode(ULTRB_TRIGER,OUTPUT);
  pinMode(ULTRC_TRIGER,OUTPUT);
  pinMode(PIN_ALARM,OUTPUT);
  digitalWrite(ULTRA_TRIGER,LOW);
  digitalWrite(ULTRB_TRIGER,LOW);
  digitalWrite(ULTRC_TRIGER,LOW);

  // Serial Comunication
  Serial1.begin(BAUD_RATE);               // Inicio la transmision serie
  Serial.begin(9600);               // Inicio la transmision serie

  // PWM Configuration
  TCCR2B = TCCR2B & 0b11111000 | 0x01;   // Establece frecuencia pwm pines 9,10 a 31K
  analogWrite(PWM1, 127);                // PWM1 50% duty
  analogWrite(PWM2, 127);                // PWM2 50% duty


  // Interrupciones externas (Pines 18,19,2,3)
  attachInterrupt(digitalPinToInterrupt(INTE0), ISR_INTE0,    CHANGE); // Interrupcion externa en pin 2 por cambio de nivel
  attachInterrupt(digitalPinToInterrupt(INTE1), ISR_INTE1,    CHANGE); // Interrupción externa en pin 3 por cambio de nivel
  // Acelerometro Inicializacion y calibracion
  init_MPU6050();
  calibrar_MPU6050(offset);
  obtener_datos(datos, offset);

  for(int i = 0; i < N_ULTR_SENSOR; i++)
  {
    for(int j = 0; j < MUESTRAS_DETECCION; j++)
    {
      contador_obstaculo[i][j] =   0;
    }
  }
  // Reset Sensor ultrasonido variables
  for (int i = 0; i < N_ULTR_SENSOR; i++)
  {
    ultr_distance           [i] = 100.0;
    contador_deteccion      [i] =   0;
    flag_objeto_detectado   [i] =   0;
  }
  // Centrar volante
  centrar_volante();
  delay(5000);
  
  Serial.println("START");  

  //mover(100,0.1,0);
 
}

/**                                                                                                                                                                  
 * @name setup
 * @brief Main Loop. Cuando recibe datos desde la CPU realiza acciones.
 * @param None
 */
void loop()
{
  if ((data_rec[1] == 'g') && (flag_accion))
  {
    respuestaid_mpu = data_rec[0];
    obtener_datos(datos, offset);
    //for(int i=0;i<6;i++)
    for(int i=0;i<3;i++)
      dtostrf(datos[i],6,2,buff[i]);
    dtostrf(valor_giro_total,6,2,buff[3]);
    dtostrf((velocidad_abs)  ,6,2,buff[4]);
    dtostrf((ultr_distance[0])  ,6,2,buff[5]);
    dtostrf((ultr_distance[1])  ,6,2,buff[6]);
    //dtostrf((ultr_distance[2])  ,6,2,buff[10]);

    //sprintf(mystring, "%s %s %s %s %s %s %s %s %s %s %s !", buff[0], buff[1], buff[2], buff[3], buff[4], buff[5],buff[6],buff[7],buff[8],buff[9],buff[10]);
    sprintf(mystring, "%s %s %s %s %s %s %s!", buff[0], buff[1], buff[2],buff[3],buff[4],buff[5],buff[6]);
    send_uart(mystring, respuestaid_mpu);
    flag_accion = 0;
  }

  if ((data_rec[1] == 'h') && (flag_accion))
  {
    respuestaid_mpu = data_rec[0];
    calibrar_MPU6050(offset);
    flag_accion = 0;
  }

  if ((data_rec[1] == 'k') && (flag_accion))
  {
    respuestaid_plan = data_rec[0];
    
    if(data_rec[7])
      grados_objetivo = data_rec[6] * (-1.0);
    else
      grados_objetivo = data_rec[6] * 1.0;

    distancia_shifted = data_rec[2] + data_rec[3]*256;
    girar();
    mover(distancia_shifted,data_rec[4]/100.0, data_rec[5]);
    flag_accion = 0;
  }

  if(objeto_detectado == 3)
  {
    objeto_detectado = 0;
    Serial.println("Ya me acomode. Informando objeto");
    Serial.println(flag_rotacion);
    dtostrf(valor_giro_total,6,2,buff[0]);
    dtostrf((distancia_objeto[0])  ,6,2,buff[1]);
    dtostrf((distancia_objeto[1])  ,6,2,buff[2]);

    sprintf(mystring, "1 %s %s %s !", buff[0], buff[1], buff[2]);
    send_uart(mystring, respuestaid_plan);
  }
  
  if(completar_movimiento == 3)
  { Serial.print("Completar Movimiento3: ");
    if(objeto_detectado == 2)
    {
      Serial.print("Entre por Objeto detectado");
      Serial.print(flag_rotacion);
      //mover(int(distancia_temp[0]), 0.3, 0);
      mover(40, 0.3, 0);
    }
    else
    {
      if(flag_reversa_corta)
        mover(REVERSA_MIN, 0.3, 0);
      else
        mover(REVERSA_MAX, 0.3, 0);
    }
  
    completar_movimiento = 4;
  }

  if(completar_movimiento == 5)
  {
    girar();
    if(flag_reversa_corta)
    {
      mover(REVERSA_MIN, 0.3, 1);
      flag_reversa_corta = 0;
    }
    else
      mover(REVERSA_MAX, 0.3, 1);
    completar_movimiento = 0;
  }
  
  // En caso que el vehículo este rotando se obtiene cada 50ms el valor del gyróscopo en el eje Z y lo va acumulando
  //Serial.print(flag_timer);Serial.print("  ");Serial.println(flag_rotacion);
  if (flag_timer)
  {
    giro_z_instantaneo = obtener_z_gyro(datos, offset) * (TIME_SAMPLE/(1000.0*1000.0));
    if(fabs(giro_z_instantaneo) > RUIDO_ROTACION)
    {
      valor_giro_total = valor_giro_total + giro_z_instantaneo;
      while (fabs(valor_giro_total) > 180)
        if(valor_giro_total > 0)
          valor_giro_total = valor_giro_total - 360;
        else
          valor_giro_total = valor_giro_total + 360;
    }
    flag_timer = 0;
  }

  if (flag_cntrl_vel)
  {
    velocidad_abs = ((distancia_abs - distancia_abs_d))/DELTA_T;
    distancia_abs_d = distancia_abs;
    if (flag_mover)
    {
      if (!(sentido_temp))
      {
         flag_cntrl_vel = 0;
         analogWrite(PWM1, 127 + vel_inicial + pid_controller(0));
      }
      else
      {
        flag_cntrl_vel = 0;
        analogWrite(PWM1, 127 - vel_inicial - pid_controller(0));
      }
    }
     flag_cntrl_vel = 0;
  }

  if(flag_ultrasonido == 1)
  {
    if (flag_ultr_request == 0)
    {      
      //Serial.println("TRIGGER 0");
      digitalWrite(ULTRA_TRIGER,HIGH); //se envia el pulso ultrasonico
      delayMicroseconds(10);//El pulso debe tener una duracion minima de 20 microsegundos // FIX ME (Estaba en 20)
      digitalWrite(ULTRA_TRIGER,LOW); //Ambas lineas son por estabilizacion del sensor
      flag_ultr_request = 1;
      flag_ultrasonido = 0;
    }
    else if (flag_ultr_request == 1)
    {
     //Serial.println("TRIGGER 1");
     digitalWrite(ULTRB_TRIGER,HIGH); //se envia el pulso ultrasonico
     delayMicroseconds(10);//El pulso debe tener una duracion minima de 20 microsegundos // FIX ME (Estaba en 20)
     digitalWrite(ULTRB_TRIGER,LOW); //Ambas lineas son por estabilizacion del sensor      
     flag_ultr_request = 2;
     flag_ultrasonido = 0;
    }
    else
    {
     digitalWrite(ULTRC_TRIGER,HIGH); //se envia el pulso ultrasonico
     delayMicroseconds(10);//El pulso debe tener una duracion minima de 20 microsegundos // FIX ME (Estaba en 20)
     digitalWrite(ULTRC_TRIGER,LOW); //Ambas lineas son por estabilizacion del sensor      
     flag_ultr_request = 0;
     flag_ultrasonido = 0;
    }
  }

  if (Serial1.available() > 0)
    receive_uart();
}


