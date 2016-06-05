#include <TimerOne.h>
#include <Wire.h>
#include <MPU6050.h>

/********************************
 *        Constantes            *
/********************************/
const int   PWM2        =    9;
const int   PWM1        =   10;
const int   INTE0       =    2;
const int   INTE1       =    3;
const int   PPV         =   24;
const int   TIME_SAMPLE =   50;
const int   BAUD_RATE   = 115200;
const float LONG_ARC    =  2 * 3.14 * 10; //longitud de arco de la rueda en cm
const float DIST_PULS   = LONG_ARC / PPV;

boolean flag_rotacion    = false;
boolean flag_timer       = false;
boolean flag_mover       = false;
boolean flag_transmision = false;
boolean flag_accion      = false;

char formlist[100];
char data_rec[100];
char mystring[30];
char SerRx;
int  respuestaid        =  0;
int  MICRO_ADDR         = 10; //Verificar
int  flag_uart          =  0;
int  contador_backup    =  0;
int  data_len_rx        =  0;

float valor_giro_temp   = 0;
float distancia_temp    = 0;
float aceleracion_old   = 0;
float velocidad         = 0;
float velocidad_old     = 0;
float offset[6]         = {0, 0, 0, 0, 0, 0};
float datos[7];
float Tmp;
char buff[6][6];

long distancia_max      = 0;
int contador_movimiento = 0;
int encoder1            = 0; //contadores encoders ruedas
int encoder2            = 0;
int grados_max          = 0;

/********************************
 * Declaración de variables     *
/********************************/

void send_uart(char*);
void receive_uart();
void print_datos();
void mover(int, int, int); //moverse hacia adelante x cm a y velocidad
void girar(int, int);      //girar (grados, sentido) 0 horario, 1 antihorario
void reset_transmision();

void setup()
{
  TCCR2B = TCCR2B & 0b11111000 | 0x01;   // Establece frecuencia pwm pines 9,10 a 31K
  Timer1.initialize(TIME_SAMPLE*1000);   // Dispara cada TIME_SAMPLE ms
  Timer1.attachInterrupt(ISR_Timer);     // Activa la interrupcion y la asocia a ISR_Blink
  Serial.begin(BAUD_RATE);               // Inicio la transmision serie
  Serial1.begin(BAUD_RATE, SERIAL_8N1);     
  analogWrite(PWM1, 127);                // PWM1 50% duty
  analogWrite(PWM2, 127);                // PWM2 50% duty
  attachInterrupt(0, ISR_INTE0, CHANGE); // Interrupcion externa en pin 2 por cambio de nivel
  attachInterrupt(1, ISR_INTE1, CHANGE); // Interrupción externa en pin 3 por cambio de nivel
  init_MPU6050();
  calibrar_MPU6050(offset);
  obtener_datos(datos, offset);


}

/*************************************************************************************************************
 * Bucle infinito donde se llama a las diferentes funciones según el caracter que recibe el microcontrolador *
/*************************************************************************************************************/

void loop()
{

  if ((data_rec[0] == 'd') && (flag_accion))
  {
    mover(data_rec[1], data_rec[2], data_rec[3]);
    flag_accion = false;
  }


  if ((data_rec[0] == 'f') && (flag_accion))
  {
    girar(data_rec[1], data_rec[2]);
    flag_accion = false;
  }


  if ((data_rec[0] == 'g') && (flag_accion))
  {
    obtener_datos(datos, offset);
    for(int i=0;i<6;i++)
      dtostrf(datos[i],5,2,buff[i]);
    sprintf(mystring, "%s %s %s %s %s %s !", buff[0], buff[1], buff[2], buff[3], buff[4], buff[5]);
    send_uart(mystring);
    flag_accion = false;
  }

  /***************************************************************
 *      Función para calibrar el MPU6050 en un momento deseado   *
/*****************************************************************/

  if ((data_rec[0] == 'h') && (flag_accion))
  {
    calibrar_MPU6050(offset);
    flag_accion = false;
  }

  /***********************************************************************************************************************
 *    En caso que el vehículo este rotando se obtiene cada 50ms el valor del gyróscopo en el eje Z y lo va acumulando    *
/*************************************************************************************************************************/

  if ((flag_timer) && (flag_rotacion))
  {    
    valor_giro_temp = valor_giro_temp + obtener_z_gyro(datos, offset) * (TIME_SAMPLE/1000);
    Serial.println(valor_giro_temp);
    flag_timer = false;
  }

  if (Serial1.available() > 0)
    receive_uart();

}

/************************************************************************************************************************************
 * Función para mover el vehículo. Se deben pasar como parámetros la distancia a recorrer, la velocidad de los motores y el sentido *
/************************************************************************************************************************************/
void mover(int distancia, int vlc, int sentido)
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
void send_uart(char* data_send)
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
    Serial1.write(formlist[i]);
}

void receive_uart()
{
  SerRx = Serial1.read();
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
    respuestaid = SerRx;
    flag_uart++;
  }
  else if ( (flag_uart > 4) && (flag_uart < 5 + data_len_rx) )
  {
    data_rec[flag_uart - 5] = SerRx;
    flag_uart++;
  }
  else if ( (SerRx == 64) && ( flag_uart == (5 + data_len_rx) ) ) //termina la recepción de la trama
  {
    flag_uart = 0;
    flag_accion = 1; //Ejecutamos la accion en el loop principal
  }
  else flag_uart = 0;

}

void print_datos()
{
  Serial.print("AcX = "); Serial.print(datos[0]);
  Serial.print(" | AcY = "); Serial.print(datos[1]);
  Serial.print(" | AcZ = "); Serial.print(datos[2]);
  Serial.print(" | Tmp = "); Serial.print(Tmp / 340.00 + 36.53); //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(datos[4]);
  Serial.print(" | GyY = "); Serial.print(datos[5]);
  Serial.print(" | GyZ = "); Serial.println(datos[6]);
}

void ISR_Timer()
{
  if ( ( fabs(valor_giro_temp) > grados_max) && (flag_rotacion == true) )
    {
      analogWrite(PWM1, 127);
      analogWrite(PWM2, 127);
      flag_rotacion = false;
      send_uart("ok!");
      valor_giro_temp = 0;
    }
      
  if ( ( distancia_temp > distancia_max) && (flag_mover == true) )
    {
      analogWrite(PWM1, 127);
      analogWrite(PWM2, 127);
      flag_mover = false;
      send_uart("ok!");
      distancia_temp = 0;
    }
  
  flag_timer = true;
}

void ISR_INTE0()
{
  if (flag_mover)
    distancia_temp = distancia_temp + DIST_PULS;
}

void ISR_INTE1()
{
  if (flag_mover)
    distancia_temp = distancia_temp + DIST_PULS;
}





