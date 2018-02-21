#include "TimerOne.h"
#include <Wire.h>

#include "MPU6050.h"

/********************************
 *        Constantes            *
/********************************/

const int   PWM2           =    9;//Motor Volante
const int   PWM1           =   10;//Motor Principal
const int   INTE0          =    2;
const int   INTE1          =    3;
const float PPV            = 36.0;
const long  TIME_SAMPLE    =   50000;
const int   CONTROL_PERIOD =     10;
const float DELTA_T        = (TIME_SAMPLE * CONTROL_PERIOD)/(1000.0*1000.0);

const int   BAUD_RATE        = 9600;
const int   MICRO_ADDR       =   10; 
const float LONG_ARC         =  2.0 * 3.14 * 10.0; //longitud de arco de la rueda en cm
const float DIST_PULS        = LONG_ARC / PPV;
const float ROTACION_VOLANTE =  52.0 ;
const float GIRO_MAX         = ROTACION_VOLANTE/2;
const int   PULSOS_VOLANTE   =   9;
const float PASO_VOLANTE     = ROTACION_VOLANTE / PULSOS_VOLANTE;
const float GIRO_MOV         = PASO_VOLANTE * 4;
const float GIRO_LIMITE      = 40;
const float GIRO_LEVE        = PASO_VOLANTE * 2;
const float LIMITE_GIRO      = 10;
const int   LIMITE_DESVIO    = 2;
const int   VOLANTE_CENTRADO = 4;
const float RUIDO_ROTACION   = 1;

/********************************
 *    Variables Globales        *
/********************************/

// Banderas

boolean flag_rotacion            = false;
boolean flag_girar               = false;
boolean flag_timer               = false;
boolean flag_mover               = false;
boolean flag_accion              = false;
boolean flag_cntrl_vel           = false;
boolean flag_finished            = false;
boolean rotacion_incompleta      = false;
boolean flag_terminar_movimiento = false;
boolean flag_back                = false;
boolean flag_terminar_giro       = false;
boolean flag_giro_leve           = false;
boolean flag_linea_recta         = false;
boolean flag_centrar_vehiculo    = false;
boolean toggle                   = true;

// Variables de Comunicacion Serie

int  flag_uart          =  0;
int  data_len_rx        =  0;
char SerRx;
char formlist[150];
char data_rec[10];
char mystring[100];
char buff[8][7];
float datos[7];

int  respuestaid_mpu    =  0;
int  respuestaid_plan   =  0;

// Variables de movimiento

float distancia_temp[2]   = {0.0, 0.0};
float distancia_temp_d[2] = {0.0, 0.0};
float movimiento[2]       = {0.0, 0.0};
float velocidad_ref       = 3.0; // Velocidad crucero en m/s
float vel_inicial         = 35.0;
float p_controller[2]     = {0.0, 0.0};
float d_controller[2]     = {0.0, 0.0};
float i_controller[2]     = {0.0, 0.0};
float prev_error[2]       = {0.0, 0.0};
double kd                 = 1.0;
double ki                 = 0.20;
double kp                 = 20.0;

float valor_giro_temp        = 0;
float valor_giro_total       = 0;
float valor_giro_instantaneo = 0;
float centrar_vehiculo       = 0;
float giro_z_instantaneo     = 0;
float grados_por_rotar       = 0;
float aceleracion_old        = 0;
float velocidad              = 0;
float velocidad_old          = 0;
long distancia_max           = 0;
int sentido_temp             = 0;
int sentido_giro             = 0;
int contador_movimiento      = 0;
int contador_pid             = 0;
int grados_max               = 0;
int grados_volante_max       = 0;
int posicion_volante         = 0;

// Variables Acelerometro

float offset[6]         = {0, 0, 0, 0, 0, 0};
float Tmp; 

/********************************
 * Declaración de funciones     *
/********************************/

/**                                                                                                                                                                  
 * @name send_uart
 * @brief Armado de trama + envio de datos a traves del modulo UART
 * @param Cadena de caracteres a enviar. Debe finalizar con el caracter "!" 
 * @param Id del objeto que va a recibir los datos
 */
void send_uart(char*, int);

/**                                                                                                                                                                  
 * @name receive_uart
 * @brief Extraccion de los datos entramados para posterior procesamiento
 * @param None
 */
void receive_uart();

/**                                                                                                                                                                  
 * @name centrar_volante
 * @brief Coloca el volante en la posicion central.
 * @param None
 */
void centrar_volante();

/**                                                                                                                                                                  
 * @name doblar_volante
 * @brief Rotacion del volante un determinado angulo.
 * @param type: int. Angulo a rotar en grados.
 * @param type: int. Sentido de rotacion. 0: Horario 1: Antihorario.
 */
void doblar_volante(int, int);

/**                                                                                                                                                                  
 * @name mover Warning: Podríamos cambiarle el nombre
 * @brief Traslacion del vehiculo a cierta distancia, velocidad y sentido.
 * @param type: int. Distancia a recorrer en centimetros.
 * @param type: double. Velocidad en metros por segundo.
 * @param type: int. Sentido en el que se va a desplazar. 1: Hacia adelante 0: Hacia atras.
 */
void mover(int, double, int);

/**                                                                                                                                                                  
 * @name pid_controller
 * @brief Controlador PID para la velocidad de los motores
 * @param type: int. Motor a controlar (0 o 1).
 */
double pid_controller(int);

/**                                                                                                                                                                  
 * @name girar Warning: Podriamos cambiarle el nombre
 * @brief Rotacion del vehiculo cierto angulo y sentido.
 * @param type: int. Angulo a rotar en grados.
 * @param type: int. Sentido de rotacion. 0: Horario 1: Antihorario.
 */
void girar(int, int);     

/**                                                                                                                                                                  
 * @name setup
 * @brief Inicializacion de variables, interrupciones, comunicacion.
 * @param None
 */
void setup()
{
  TCCR2B = TCCR2B & 0b11111000 | 0x01;   // Establece frecuencia pwm pines 9,10 a 31K
  Timer1.initialize(double(TIME_SAMPLE)); // WARNING: Depende de la versión del compilador  // Dispara cada TIME_SAMPLE ms
  Timer1.attachInterrupt(ISR_Timer);     // Activa la interrupcion y la asocia a ISR_Timer
  Serial.begin(BAUD_RATE);               // Inicio la transmision serie
  //Serial2.begin(BAUD_RATE, SERIAL_8N1);
  
  analogWrite(PWM1, 127);                // PWM1 50% duty
  analogWrite(PWM2, 127);                // PWM2 50% duty
  
  attachInterrupt(digitalPinToInterrupt(INTE0), ISR_INTE0,    CHANGE); // Interrupcion externa en pin 2 por cambio de nivel
  attachInterrupt(digitalPinToInterrupt(INTE1), ISR_INTE1,    CHANGE); // Interrupción externa en pin 3 por cambio de nivel
  
  init_MPU6050();
  calibrar_MPU6050(offset);
  
  obtener_datos(datos, offset);

  centrar_volante();
  //delay (5000);
  //girar(60, 1);
  //mover(80, 0.3, toggle);
}

/**                                                                                                                                                                  
 * @name setup
 * @brief Main Loop. Cuando recibe datos desde la CPU realiza acciones.
 * @param None
 */
void loop()
{
 //if (flag_finished)
 // {
 //   toggle = !toggle;
 //   girar(20, 1);
 //   mover(80, 0.3, toggle);
 //   flag_finished = false;
 // }

  if ((data_rec[1] == 'd') && (flag_accion))
  {
    respuestaid_plan = data_rec[0];
    mover(data_rec[2], data_rec[3], data_rec[4]);
    flag_accion = false;
  }

  if ((data_rec[1] == 'f') && (flag_accion))
  {
    respuestaid_plan = data_rec[0];
    girar(data_rec[2], data_rec[3]);
    flag_accion = false;
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
    flag_accion = false;
  }

  if ((data_rec[0] == 'h') && (flag_accion))
  {
    respuestaid_mpu = data_rec[0];
    calibrar_MPU6050(offset);
    flag_accion = false;
  }

  if ((data_rec[1] == 'k') && (flag_accion))
  {
    if (data_rec[5] < 1)
      data_rec[5] = 0;
    respuestaid_plan = data_rec[0];
    girar(data_rec[5], data_rec[6]);
    mover(data_rec[2],data_rec[3]/100.0, data_rec[4]);
    flag_accion = false;
  }

  if(flag_terminar_giro)
  {
    mover(40, 0.3, 0);
    flag_terminar_giro = false;
    flag_back = true;
    valor_giro_temp = 0;
  }

  if (flag_terminar_movimiento)
  {
    Serial.print("Grados por rotar:");Serial.println(grados_por_rotar - fabs(valor_giro_temp));
    girar(int(grados_por_rotar - fabs(valor_giro_temp)), sentido_giro);
    mover(40, 0.3, 1);
    flag_terminar_movimiento = false;
  }

  if(flag_centrar_vehiculo)
  {
    if(centrar_vehiculo > 0)
      girar(int(fabs(centrar_vehiculo)), 0);
    else
      girar(int(fabs(centrar_vehiculo)), 1);

    flag_centrar_vehiculo = false;
  }
  // En caso que el vehículo este rotando se obtiene cada 50ms el valor del gyróscopo en el eje Z y lo va acumulando
  //Serial.print(flag_timer);Serial.print("  ");Serial.println(flag_rotacion);
  if (flag_timer)
  {
    giro_z_instantaneo = obtener_z_gyro(datos, offset) * (TIME_SAMPLE/(1000.0*1000.0));
    if(flag_rotacion || flag_back)
      valor_giro_temp = valor_giro_temp + giro_z_instantaneo;
    if(giro_z_instantaneo > RUIDO_ROTACION)
      valor_giro_total = valor_giro_total + giro_z_instantaneo;
    
    flag_timer = false;
  }

  if (flag_cntrl_vel && flag_mover)
    if (!(sentido_temp))
      {
         flag_cntrl_vel = false;
         analogWrite(PWM1, 127 + vel_inicial + pid_controller(0));
      }
    else
      {
        flag_cntrl_vel = false;
        analogWrite(PWM1, 127 - vel_inicial - pid_controller(0));
      }

  if (Serial.available() > 0)
    receive_uart();
}

/************************************************************************************************************************************
 * Función para mover el vehículo. Se deben pasar como parámetros la distancia a recorrer, la velocidad de los motores y el sentido *
/************************************************************************************************************************************/
void centrar_volante()
{
  analogWrite(PWM2, 30);
  delay(4000);
  analogWrite(PWM2, 127);
  doblar_volante(GIRO_MOV, 0);
  posicion_volante = VOLANTE_CENTRADO;
}

void mover(int distancia, double vlc, int sentido)
{
  if (!(sentido))
  {
    analogWrite(PWM1, 127 + vel_inicial);
  }
  else
  {
    analogWrite(PWM1, 127 - vel_inicial);
  }
  contador_movimiento = 0;
  distancia_max = distancia;
  velocidad_ref = vlc;
  sentido_temp = sentido;
  movimiento[0] = vel_inicial;
  distancia_temp[0] = 0;
  distancia_temp_d[0] = 0;
  p_controller[0]  = 0;
  d_controller[0]  = 0;
  i_controller[0]  = 0;
  prev_error[0]    = 0;
  flag_mover = true;
}

/******************************************************************************************
 * Función para girar el vehículo. Se debe pasar el sentido de giro y los grados deseados *
/******************************************************************************************/
void girar(int grados, int sentido)
{
  grados_max = grados;
  sentido_giro = sentido;
  while (grados_max > 180)
  {
    grados_max = grados_max - 180;
    if (sentido)
      sentido = 0;
    else
      sentido = 1;
  }

  if(grados_max < 0)
  {
    grados_max = fabs(grados_max);
    sentido_giro = !sentido;
  }
  
  if(grados_max != 0)
  {
    if(grados_max >= LIMITE_GIRO)
    {
      doblar_volante(GIRO_LIMITE, sentido);
      valor_giro_temp = 0;
      flag_rotacion = true;
    }
    else
    {
      doblar_volante(GIRO_LIMITE, sentido);
      valor_giro_temp = 0;
      flag_rotacion = true;
      flag_giro_leve = true;
    }
  }
  else
  {
    flag_linea_recta = true;
    valor_giro_instantaneo = valor_giro_total;
  }

}

void doblar_volante(int grados, int sentido)
{    
  if (!(sentido))
  {
    analogWrite(PWM2, 230);
  }
  else
  {
    analogWrite(PWM2, 30);
  }
  distancia_temp[1] = 0;
  grados_volante_max = grados;
  flag_girar = true;
  
}

/*******************************************************************************************************************************************************
 * Arma la trama con el dato a enviar y lo envía por la UART. La trama contiene un encabezado, la longitud del dato high y low, el identificador del ***
 * objeto que requirió el dato, la dirección desde la cual se envía el dato, el DATO y un fin de trama *************************************************
/*******************************************************************************************************************************************************/
void send_uart(char* data_send, int respuestaid)
{
  int data_len    = 0;
  char caracter   = 0;
  int header      = 0;
  int iheader     = 0;
  int PL          = 0;
  int mod_header  = 0;
  int mod_iheader = 0;
  int size_l      = 0;
  int size_h      = 0;

  header = 160;
  iheader = 64;
  while (caracter != '!')
  {
    caracter = data_send[data_len];
    data_len++;
  }
  data_len--;
  PL = 16;
  mod_header = header + PL + ((data_len & 0xf0000) >> 16);
  mod_iheader = iheader;
  size_l = data_len & 0xff;
  size_h = (data_len & 0xff00) >> 8;
  formlist[0] = mod_header;
  formlist[1] = size_h;
  formlist[2] = size_l;
  formlist[3] = respuestaid;
  formlist[4] = MICRO_ADDR;

  for ( int i = 5 ; i < 5 + data_len ; i++)
    formlist[i] = data_send[i - 5];
  formlist[5 + data_len] = mod_iheader;

  for (int i = 0 ; i < data_len + 6; i++)
    Serial.write(formlist[i]);
}

void receive_uart()
{
  SerRx = Serial.read();
  if ( ( flag_uart == 0 ) && ( (SerRx & 0xE0) == 160) ) //Encontramos el MIT
  {
    flag_uart++;
    data_len_rx = (SerRx & 0x0F);
  }
  else if ( flag_uart == 1 )
  {
    data_len_rx = data_len_rx + (SerRx << 8);
    flag_uart++;
  }
  else if ( flag_uart == 2 )
  {
    data_len_rx = data_len_rx + SerRx;
    flag_uart++;
  }
  else if ( ( SerRx == MICRO_ADDR ) && ( flag_uart == 3 ) )
  {
    flag_uart++;
  }
  else if ( flag_uart == 4 )
  {
    data_rec[0] = SerRx;
    flag_uart++;
  }
  else if ( (flag_uart > 4) && (flag_uart < 5 + data_len_rx) )
  {
    data_rec[flag_uart - 4] = SerRx;
    flag_uart++;
  }
  else if ( (SerRx == 64) && ( flag_uart == (5 + data_len_rx) ) ) //termina la recepción de la trama
  {
    flag_uart = 0;
    flag_accion = true; //Ejecutamos la accion en el loop principal
  }
  else flag_uart = 0;

}

void ISR_Timer()
{
  if((distancia_temp[1] >= grados_volante_max) && (flag_girar == true))
  {
    analogWrite(PWM2, 127);
    flag_girar = false;
    distancia_temp[1] = 0;
  }

  if ((fabs(valor_giro_temp) > grados_max) && (flag_rotacion == true))
    {
      if(flag_giro_leve)
      {
        doblar_volante(GIRO_LEVE, !sentido_giro);
        flag_rotacion = false;
        flag_giro_leve = false;
        valor_giro_temp = 0;
      }
      else
      {
        doblar_volante(GIRO_MOV, !sentido_giro);
        flag_rotacion = false;
        valor_giro_temp = 0;
      }
    }

  if(flag_linea_recta)
  {
     centrar_vehiculo = valor_giro_instantaneo - valor_giro_total;
     if(fabs(centrar_vehiculo) > LIMITE_DESVIO)
       flag_centrar_vehiculo = true;
  }
  
  contador_pid = (contador_pid + 1) % CONTROL_PERIOD;
  if((contador_pid == 1) && flag_mover)
    flag_cntrl_vel = true;

  if ( (( distancia_temp[0] > distancia_max) ) && (flag_mover == true) )
    {
      flag_finished = true;
      analogWrite(PWM1, 127);
      flag_mover = false;
      flag_linea_recta = false;
      flag_cntrl_vel = false;  
    
      if (flag_back == true)
      {
        flag_terminar_movimiento = true;
        flag_back = false;
      }
    
      if(flag_rotacion == true)
        rotacion_incompleta = true;  
      else
        send_uart("0 !", respuestaid_plan);
      
      distancia_temp[0] = 0;

    }

  if(rotacion_incompleta == true)
  {
    if(flag_giro_leve)
    {
      doblar_volante(GIRO_LEVE, !sentido_giro);
      flag_giro_leve = false;
    }
    else
    {
      doblar_volante(GIRO_MOV, !sentido_giro);
    }
    grados_por_rotar = grados_max - fabs(valor_giro_temp);
    flag_terminar_giro = true;
    flag_rotacion = false;
    valor_giro_temp = 0;
    rotacion_incompleta = false;
  }
  flag_timer = true;
}

//Esta es la distancia instantanea pero se suman las dos juntas. Para poder controlar las velocidades de los motores individualmente hay que hacerlo por separado.
void ISR_INTE0()//PIN2 Motor Principal
{
  if (flag_mover)
    distancia_temp[0] = distancia_temp[0] + DIST_PULS;
}

void ISR_INTE1()//PIN3 Motor Volante
{
  if (flag_girar)
  {
    distancia_temp[1] = distancia_temp[1] + PASO_VOLANTE;
    if(sentido_giro)
      posicion_volante++;
    else
      posicion_volante--;
  }
  Serial.println(distancia_temp[1]);
}

//Control PID para la velocidad de los motores, es necesario calibrar los parametros.
//Hay que ver de controlar la velocidad de ambos motores por separado como modifico variables locales
// voy a tener que tener todas las variables duplicadas para ambos controles

double pid_controller(int motor)
{
  double error = 0;
  double valor_instantaneo = 0;
  double pid_contr = 0;
  double velocidad_temp = 0; 

  velocidad_temp = ((distancia_temp[motor] - distancia_temp_d[motor])*0.01)/DELTA_T;
  error = velocidad_ref - velocidad_temp;
  distancia_temp_d[motor] = distancia_temp[motor];

  p_controller[motor]  = kp * error;
  i_controller[motor] += ki * error * DELTA_T;
  d_controller[motor]  = (kd * (error - prev_error[motor])) / DELTA_T;
  prev_error[motor]    = error;
  pid_contr      = p_controller[motor] + i_controller[motor] + d_controller[motor];
  //pid_contr      = p_controller[motor];
  movimiento[motor]    += pid_contr;
  if(movimiento[motor] > 110.0)
      movimiento[motor] = 110.0;
   
  return movimiento[motor];
}



