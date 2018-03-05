#include <Arduino.h>

extern const int   PWM1;//Motor Principal
extern const int   PWM2;//Motor Volante
extern const float GIRO_LIMITE;
extern const float GIRO_MIN;
extern const float LIMITE_GIRO;
extern const float ROTACION_VOLANTE;
extern const int   PULSOS_VOLANTE;
extern const float PASO_VOLANTE;
extern const float GIRO_MOV;
extern const int   VOLANTE_CENTRADO;
extern const float GIRO_LEVE;
extern const int   CONTROL_PERIOD;
extern const long  TIME_SAMPLE;
extern const float DELTA_T;

extern int         flag_girar;
extern int         flag_mover;
extern int         flag_rotacion;
extern int         flag_linea_recta;
extern int         flag_giro_leve;

extern int         posicion_volante;
extern int         sentido_giro;
extern int         grados_max;
extern int         contador_movimiento;
extern int         sentido_temp;
extern int         grados_volante_max;
extern long        distancia_max;

extern float       distancia_temp[2];
extern float       velocidad_ref; // Velocidad crucero en m/s
extern float       movimiento[2];
extern float       vel_inicial;
extern float       distancia_temp_d[2];
extern float       p_controller[2];
extern float       d_controller[2];
extern float       i_controller[2];
extern float       prev_error[2];
extern float       valor_giro_instantaneo;
extern float       valor_giro_total;
extern float       valor_giro_temp;
extern float       valor_giro_offset;
extern double      kd;
extern double      ki;
extern double      kp;


/************************************************************************************************************************************
 * Función para mover el vehículo. Se deben pasar como parámetros la distancia a recorrer, la velocidad de los motores y el sentido *
/************************************************************************************************************************************/
void doblar_volante(int grados, int sentido)
{    
  if (!(sentido))
  {
    analogWrite(PWM2, 215);
  }
  else
  {
    analogWrite(PWM2, 45);
  }
  distancia_temp[1] = 0;
  grados_volante_max = grados;
  flag_girar = 1;
  
}

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
  flag_mover = 1;
}

/******************************************************************************************
 * Función para girar el vehículo. Se debe pasar el sentido de giro y los grados deseados *
/******************************************************************************************/
void girar(int grados, int sentido)
{
  grados_max = grados + valor_giro_offset;
  sentido_giro = sentido;
  while (grados_max > 180)
  {
    grados_max = grados_max - 180;
    if (sentido)
      sentido = 0;
    else
      sentido = 1;
  }

//  if(grados_max < 0)
//  {
//    grados_max = fabs(grados_max);
//    sentido_giro = !sentido;
//  }
  
  if(fabs(grados_max) > GIRO_MIN)
  {
    if(fabs(grados_max) >= LIMITE_GIRO)
    {
      doblar_volante(GIRO_LIMITE, sentido);
      valor_giro_temp = 0;
      flag_rotacion = 1;
    }
    else
    {
      doblar_volante(GIRO_LEVE, sentido);
      valor_giro_temp = 0;
      flag_rotacion = 1;
      flag_giro_leve = 1;
    }
    valor_giro_offset = 0.0;
  }
  else
  {
    valor_giro_offset = valor_giro_offset + grados_max;
    flag_linea_recta = 0;
    valor_giro_instantaneo = valor_giro_total;
  }

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

