#include <Servo.h>
#include <TimerOne.h>
#include <Wire.h>

#include </home/lborgnino/CarritoSoftware/arduino-car/Librerias/MPU6050.h>

/********************************
 *        Constantes            *
/********************************/
const int   PWM2        =    9;
const int   PWM1        =   10;
const int   INTE0       =    2;
const int   INTE1       =    3;
const float PPV         =   24.0;
const int   TIME_SAMPLE =   50;


const int   ULTR_PERIOD    =  250;
const int   ULTR_N_STEPS   =    8;
const int   ULTR_TRIGER    =   35;
const int   ULTR_ECHO      =   19;
const int   ULTR_INT       =    0;
const int   ULTR_SERVO     =   44;
const float ALARM_DISTANCE = 30.0; 

const int   BAUD_RATE   = 9600;
const int  MICRO_ADDR   =   10; //Verificar
const float LONG_ARC    =  2.0 * 3.14 * 10.0; //longitud de arco de la rueda en cm
const float DIST_PULS   = LONG_ARC / PPV;

boolean flag_rotacion    = false;
boolean flag_timer       = false;
boolean flag_mover       = false;
boolean flag_transmision = false;
boolean flag_accion      = false;
boolean flag_alarm       = false;
boolean flag_back        = false;

char formlist[150];
char data_rec[10];
char mystring[100];
char SerRx;
int  flag_uart          =  0;
int  contador_backup    =  0;
int  data_len_rx        =  0;

int  respuestaid_mpu    =  0;
int  respuestaid_plan   =  0;
int  respuestaid_ultr   =  0;

float valor_giro_temp   = 0;
float distancia_temp    = 0;
float aceleracion_old   = 0;
float velocidad         = 0;
float velocidad_old     = 0;

float offset[6]         = {0, 0, 0, 0, 0, 0};
float datos[7];

float ultr_data[ULTR_N_STEPS];
int   ultr_index         =  0;
long  ultr_start_time    =  0;
int contador_ultrasonido =  0;
 
float Tmp;
char buff[8][7];

long distancia_max      = 0;
int contador_movimiento = 0;
int encoder1            = 0; //contadores encoders ruedas
int encoder2            = 0;
int grados_max          = 0;

Servo ultr_servo;

/********************************
 * Declaración de funciones     *
/********************************/

void send_uart(char*,int);
void receive_uart();
void print_datos();
void mover(double, int, int); //moverse hacia adelante x cm a y velocidad
void girar(int, int);      //girar (grados, sentido) 0 horario, 1 antihorario
void reset_transmision();
double ultr_distance();

void setup()
{
  TCCR2B = TCCR2B & 0b11111000 | 0x01;   // Establece frecuencia pwm pines 9,10 a 31K
  Timer1.initialize(500.0);   // Dispara cada TIME_SAMPLE ms
  Timer1.attachInterrupt(ISR_Timer);     // Activa la interrupcion y la asocia a ISR_Blink
  
  Serial.begin(BAUD_RATE);               // Inicio la transmision serie
  Serial2.begin(BAUD_RATE, SERIAL_8N1);
  
  analogWrite(PWM1, 127);                // PWM1 50% duty
  analogWrite(PWM2, 127);                // PWM2 50% duty
  
  attachInterrupt(digitalPinToInterrupt(INTE0), ISR_INTE0,    CHANGE); // Interrupcion externa en pin 2 por cambio de nivel
  attachInterrupt(digitalPinToInterrupt(INTE1), ISR_INTE1,    CHANGE); // Interrupción externa en pin 3 por cambio de nivel
  attachInterrupt(digitalPinToInterrupt(ULTR_ECHO),ISR_ECHO_INT, CHANGE); 

  pinMode(ULTR_TRIGER,OUTPUT);
  digitalWrite(ULTR_TRIGER,LOW);
  
  ultr_servo.attach(ULTR_SERVO);         // Aviso al programa que el servo estara en el pin 9
  ultr_servo.write(10);                  // Set servo to mid-point

  for(int i = 0; i < ULTR_N_STEPS; i++)
    ultr_data[i] = 0.0;
  
  init_MPU6050();
  calibrar_MPU6050(offset);
  obtener_datos(datos, offset);
  

}

/*************************************************************************************************************
 * Bucle infinito donde se llama a las diferentes funciones según el caracter que recibe el microcontrolador *
/*************************************************************************************************************/

void loop()
{


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

       //
  if ((data_rec[1] == 'i') && (flag_accion))
  {
    respuestaid_ultr = data_rec[0];
    for(int i = 0; i < ULTR_N_STEPS; i++)
    {
      if(ultr_data[i] > 300)
        ultr_data[i] = 300.0;
      dtostrf(ultr_data[i],6,2,buff[i]);
    }
    sprintf(mystring, "%s %s %s %s %s %s %s %s !", buff[0], buff[1], buff[2], buff[3], buff[4], buff[5], buff[6], buff[7]);
    send_uart(mystring, respuestaid_ultr);
    flag_accion = false;
  }

  /***************************************************************
 *      Función para calibrar el MPU6050 en un momento deseado   *
/*****************************************************************/

  if ((data_rec[0] == 'h') && (flag_accion))
  {
    respuestaid_mpu = data_rec[0];
    calibrar_MPU6050(offset);
    flag_accion = false;
  }

  /***********************************************************************************************************************
 *    En caso que el vehículo este rotando se obtiene cada 50ms el valor del gyróscopo en el eje Z y lo va acumulando    *
/*************************************************************************************************************************/

  if ((flag_timer) && (flag_rotacion))
  {
    valor_giro_temp = valor_giro_temp + obtener_z_gyro(datos, offset) * (TIME_SAMPLE/1000);
    flag_timer = false;
  }

  if (Serial2.available() > 0)
    receive_uart();

}

/************************************************************************************************************************************
 * Función para mover el vehículo. Se deben pasar como parámetros la distancia a recorrer, la velocidad de los motores y el sentido *
/************************************************************************************************************************************/
void mover(double distancia, int vlc, int sentido)
{
  if (!(sentido))
  {
    analogWrite(PWM1, 127 + vlc);
    analogWrite(PWM2, 127 + vlc);
  }
  else
  {
    analogWrite(PWM1, 127 - vlc);
    analogWrite(PWM2, 127 - vlc);
  }
  contador_movimiento = 0;
  distancia_max = distancia;
  distancia_temp = 0;
  flag_back = sentido;
  flag_mover = true;

}

/******************************************************************************************
 * Función para girar el vehículo. Se debe pasar el sentido de giro y los grados deseados *
/******************************************************************************************/
void girar(int grados, int sentido) //0 antihorario 1 horario
{
  grados_max = grados;
  while (grados_max > 180)
  {
    grados_max = grados_max - 180;
    if (sentido)
      sentido = 0;
    else
      sentido = 1;
  }
  if (sentido)
  {
    analogWrite(PWM2, 210);
    analogWrite(PWM1, 30);
    valor_giro_temp = 0;
    flag_rotacion = 1;
  }
  else
  {
    analogWrite(PWM1, 210);
    analogWrite(PWM2, 40);
    valor_giro_temp = 0;
    flag_rotacion = 1;
  }

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
    Serial2.write(formlist[i]);
}

void receive_uart()
{
  SerRx = Serial2.read();
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
    flag_accion = 1; //Ejecutamos la accion en el loop principal
  }
  else flag_uart = 0;

}

void ISR_Timer()
{
  if ( ( fabs(valor_giro_temp) < grados_max) && (flag_rotacion == true) )
    {
      analogWrite(PWM1, 127);
      analogWrite(PWM2, 127);
      flag_rotacion = false;
      send_uart("0 !", respuestaid_plan);
      valor_giro_temp = 0;
    }

  if ( (( distancia_temp > distancia_max) || flag_alarm) && (flag_mover == true) )
    {
      analogWrite(PWM1, 127);
      analogWrite(PWM2, 127);
      flag_mover = false;

    if(!(flag_back))
      if(flag_alarm)
      {
        flag_alarm = false;
        mover(distancia_temp,70,1);
      }
      else
      {
        send_uart("0 !", respuestaid_plan);
      }
      else
        send_uart("1 !", respuestaid_plan);            
  
      distancia_temp = 0;

    }

    contador_ultrasonido = (contador_ultrasonido+1) % ULTR_PERIOD;
    if ( contador_ultrasonido == 1)
    {
      //Serial.print(distancia_temp); Serial.print(" "); Serial.print(distancia_max);Serial.print(" ");Serial.println(DIST_PULS);
       //Serial.print("Servo move: ");Serial.println(ultr_index*15+37.5);
       ultr_servo.write(ultr_index*15+37.5);
       digitalWrite(ULTR_TRIGER,HIGH); //se envia el pulso ultrasonico
       delayMicroseconds(20);//El pulso debe tener una duracion minima de 10 microsegundos
       digitalWrite(ULTR_TRIGER,LOW); //Ambas lineas son por estabilizacion del sensor
       
    }

  flag_timer = true;
}

void ISR_INTE0()
{
  //if (flag_mover)
  //  distancia_temp = distancia_temp + DIST_PULS;
}

void ISR_INTE1()
{
  if (flag_mover)
    distancia_temp = distancia_temp + DIST_PULS;
}

void ISR_ECHO_INT()
{
 if(digitalRead(ULTR_ECHO))
 {
  ultr_start_time = micros();
 }
 else
 {
    ultr_data[ultr_index] = (micros()-ultr_start_time)/58;
    if ((flag_mover) && (!flag_back) && (ultr_data[ultr_index] < ALARM_DISTANCE))
      flag_alarm = true;
    ultr_index = (ultr_index+1)%ULTR_N_STEPS;
    Serial.print(ultr_index);Serial.print(" ");Serial.println(ultr_data[ultr_index]);
 }
}


