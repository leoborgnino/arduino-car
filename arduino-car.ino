#include "TimerOne.h"
#include <Wire.h>

#include "MPU6050.h"
#include "Interrupciones.h"

/********************************
 *        Constantes            *
/********************************/

const int   INTE0            = 2;
const int   INTE1            = 3;
const int   BAUD_RATE        = 9600; 
const float GIRO_MAX         = ROTACION_VOLANTE/2;
const float GIRO_MIN         = 2.0;
const float RUIDO_ROTACION   = 0.06;
const int   MICRO_ADDR       = 10;
const float PPV              = 36.0;
const float LONG_ARC         = 2.0 * 3.14 * 10.0; //longitud de arco de la rueda en cm
const float DIST_PULS        = LONG_ARC / PPV;
const int   LIMITE_DESVIO    = 2;
const int   PWM1             = 10;//Motor Principal
const int   PWM2             = 9;//Motor Volante
const float GIRO_LIMITE      = 40;
const float LIMITE_GIRO      = 5;
const float ROTACION_VOLANTE = 52.0 ;
const int   PULSOS_VOLANTE   = 9;
const float PASO_VOLANTE     = ROTACION_VOLANTE / PULSOS_VOLANTE;
const float GIRO_MOV         = PASO_VOLANTE * 5.0;
const int   VOLANTE_CENTRADO = 4;
const float GIRO_LEVE        = PASO_VOLANTE * 3.0;
const int   CONTROL_PERIOD   = 10;
const long  TIME_SAMPLE      = 50000;
const float DELTA_T          = (TIME_SAMPLE * CONTROL_PERIOD)/(1000.0*1000.0);




/********************************
 *    Variables Globales        *
/********************************/

// Banderas
int flag_uart                = 0;
int flag_accion              = 0;
int flag_centrar_vehiculo    = 0;
int flag_cntrl_vel           = 0;
int flag_finished            = 0;
int flag_back                = 0;
int flag_terminar_movimiento = 0;
int flag_terminar_giro       = 0;
int flag_timer               = 0;
int flag_girar               = 0;
int flag_mover               = 0;
int flag_rotacion            = 0;
int flag_linea_recta         = 0;
int flag_giro_leve           = 0;

// Variables de Comunicacion Serie
char  SerRx;
int   data_len_rx         = 0;
char  data_rec[10];
char  mystring[100];
char  buff[8][7];
float datos[7];

int  respuestaid_mpu      =  0;

// Variables Acelerometro
float giro_z_instantaneo     = 0;
float offset[6]          = {0, 0, 0, 0, 0, 0};

// Variables Interrupciones
int contador_pid        = 0;
int rotacion_incompleta = 0;
int respuestaid_plan    = 0;

float grados_por_rotar  = 0;
float centrar_vehiculo  = 0;

//Variables Movimiento
int  posicion_volante    = 0;
int  sentido_giro        = 0;
int  grados_max          = 0;
int  contador_movimiento = 0;
int  sentido_temp        = 0;
int  grados_volante_max  = 0;
long distancia_max       = 0;

float  distancia_temp[2]      = {0.0, 0.0};
float  velocidad_ref          = 3.0; // Velocidad crucero en m/s
float  movimiento[2]          = {0.0, 0.0};
float  vel_inicial            = 35.0;
float  distancia_temp_d[2]    = {0.0, 0.0};
float  p_controller[2]        = {0.0, 0.0};
float  d_controller[2]        = {0.0, 0.0};
float  i_controller[2]        = {0.0, 0.0};
float  prev_error[2]          = {0.0, 0.0};
float  valor_giro_instantaneo = 0;
float  valor_giro_total       = 0;
float  valor_giro_temp        = 0;
float  valor_giro_offset      = 0;
double kd                     = 1.0;
double ki                     = 0.20;
double kp                     = 20.0;


void setup()
{
  TCCR2B = TCCR2B & 0b11111000 | 0x01;   // Establece frecuencia pwm pines 9,10 a 31K
  Timer1.initialize(double(TIME_SAMPLE)); // WARNING: Depende de la versión del compilador  // Dispara cada TIME_SAMPLE ms
  Timer1.attachInterrupt(ISR_Timer);     // Activa la interrupcion y la asocia a ISR_Timer
  Serial.begin(BAUD_RATE);               // Inicio la transmision serie
  
  analogWrite(PWM1, 127);                // PWM1 50% duty
  analogWrite(PWM2, 127);                // PWM2 50% duty
  
  attachInterrupt(digitalPinToInterrupt(INTE0), ISR_INTE0,    CHANGE); // Interrupcion externa en pin 2 por cambio de nivel
  attachInterrupt(digitalPinToInterrupt(INTE1), ISR_INTE1,    CHANGE); // Interrupción externa en pin 3 por cambio de nivel
  
  init_MPU6050();
  calibrar_MPU6050(offset);
  
  obtener_datos(datos, offset);

  centrar_volante();
  Serial.println("START");
}

/**                                                                                                                                                                  
 * @name setup
 * @brief Main Loop. Cuando recibe datos desde la CPU realiza acciones.
 * @param None
 */
void loop()
{
  if ((data_rec[1] == 'd') && (flag_accion))
  {
    respuestaid_plan = data_rec[0];
    mover(data_rec[2], data_rec[3], data_rec[4]);
    flag_accion = 0;
  }

  if ((data_rec[1] == 'f') && (flag_accion))
  {
    respuestaid_plan = data_rec[0];
    girar(data_rec[2], data_rec[3]);
    flag_accion = 0;
  }

  if ((data_rec[1] == 'g') && (flag_accion))
  {
    respuestaid_mpu = data_rec[0];
    obtener_datos(datos, offset);
    for(int i=0;i<7;i++)
    {
      dtostrf(datos[i],6,2,buff[i]);
    }
    sprintf(mystring, "%s %s %s %s %s %s !", buff[0], buff[1], buff[2], buff[3], buff[4], buff[5]);
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
    girar(data_rec[5], data_rec[6]);
    mover(data_rec[2],data_rec[3]/100.0, data_rec[4]);
    flag_accion = 0;
  }

  if(flag_terminar_giro)
  {
    mover(30, 0.3, 0);
    flag_terminar_giro = 0;
    flag_back = 1;
    valor_giro_temp = 0;
  }

  if (flag_terminar_movimiento)
  {
    Serial.print("Grados por rotar:");Serial.println(grados_por_rotar - fabs(valor_giro_temp));
    girar(int(grados_por_rotar - fabs(valor_giro_temp)), sentido_giro);
    mover(40, 0.3, 1);
    flag_terminar_movimiento = 0;
  }

  if(flag_centrar_vehiculo)
  {
//    if(centrar_vehiculo > 0)
//      girar(int(fabs(centrar_vehiculo)), 0);
//    else
//      girar(int(fabs(centrar_vehiculo)), 1);
    flag_centrar_vehiculo = 0;
  }
  // En caso que el vehículo este rotando se obtiene cada 50ms el valor del gyróscopo en el eje Z y lo va acumulando
  //Serial.print(flag_timer);Serial.print("  ");Serial.println(flag_rotacion);
  if (flag_timer)
  {
    giro_z_instantaneo = obtener_z_gyro(datos, offset) * (TIME_SAMPLE/(1000.0*1000.0));
    if((flag_rotacion || flag_back) && (fabs(giro_z_instantaneo) > RUIDO_ROTACION))
      valor_giro_temp = valor_giro_temp + giro_z_instantaneo;
    if(fabs(giro_z_instantaneo) > RUIDO_ROTACION)
      valor_giro_total = valor_giro_total + giro_z_instantaneo;
    
    flag_timer = 0;
  }

  if (flag_cntrl_vel && flag_mover)
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

  if (Serial.available() > 0)
    receive_uart();
}


