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
extern const float ULTR_LIMITE;
extern const int   MUESTRAS_DETECCION;
extern const int   LIMITE_MUESTRAS;
extern const int   N_ULTR_SENSOR;
extern const float   SATURACION_INTEGRADOR;

extern int         flag_girar_volante;
extern int         flag_mover;
extern int         flag_rotacion;
extern int         flag_linea_recta;
extern int         flag_giro_leve;
extern int         flag_back;

extern int          posicion_volante;
extern int          sentido_giro;
extern int          contador_movimiento;
extern int          sentido_temp;
extern int          grados_volante_max;
extern unsigned int distancia_max;

extern float       distancia_temp[2];
extern float       velocidad_ref; // Velocidad crucero en m/s
extern float       movimiento[2];
extern float       vel_inicial;
extern float       grados_objetivo;
extern float       grados_a_rotar;
extern float       distancia_temp_d[2];
extern float       p_controller[2];
extern float       d_controller[2];
extern float       i_controller[2];
extern float       prev_error[2];
extern float       valor_giro_instantaneo;
extern float       valor_giro_total;
extern double      velocidad_temp;
extern double      kd;
extern double      ki;
extern double      kp;

extern float ultr_distance[3];
extern double distancia_objeto[3];
extern int    contador_obstaculo[3][3];
extern int    contador_libre[3][3];
extern int    contador_deteccion[3];
extern int    contador_no_deteccion[3];
extern int    flag_objeto_detectado[3];
extern int    flag_no_objeto_detectado[3];


/************************************************************************************************************************************
 * Función para mover el vehículo. Se deben pasar como parámetros la distancia a recorrer, la velocidad de los motores y el sentido *
/************************************************************************************************************************************/
void doblar_volante(int grados, int sentido)
{    
  if (!(sentido))
  {
    analogWrite(PWM2, 210);
  }
  else
  {
    analogWrite(PWM2, 45);
  }
  distancia_temp[1] = 0;
  grados_volante_max = grados;
  flag_girar_volante = 1;
  
}

void centrar_volante()
{
  analogWrite(PWM2,50);
  delay(4000);
  analogWrite(PWM2, 127);
  doblar_volante(GIRO_MOV+1, 0);
  posicion_volante = VOLANTE_CENTRADO;
}

void mover(unsigned int distancia, double vlc, int sentido)
{
  if (!(sentido))
  {
    analogWrite(PWM1, 127 + vel_inicial);
    flag_back = 1;
    flag_rotacion = 0;
  }
  else
  {
    analogWrite(PWM1, 127 - vel_inicial);
  }
  contador_movimiento = 0;
  distancia_max       = distancia;
  velocidad_ref       = vlc;
  sentido_temp        = sentido;
  movimiento[0]       = vel_inicial;
  distancia_temp[0]   = 0;
  distancia_temp_d[0] = 0;
  p_controller[0]     = 0;
  d_controller[0]     = 0;
  i_controller[0]     = 0;
  prev_error[0]       = 0;
  flag_mover          = 1;
}

/******************************************************************************************
 * Función para girar el vehículo. Se debe pasar el sentido de giro y los grados deseados *
/******************************************************************************************/
void girar()
{
  grados_a_rotar = grados_objetivo - valor_giro_total;

  while (fabs(grados_a_rotar) > 180)
  {
    if(grados_a_rotar > 0)
      grados_a_rotar = grados_a_rotar - 360;
    else
      grados_a_rotar = grados_a_rotar + 360;
  }

  if(grados_a_rotar <= 0)
    sentido_giro = 1;  
  else
    sentido_giro = 0;
  
  if(fabs(grados_a_rotar) > GIRO_MIN)
  {
    if(fabs(grados_a_rotar) >= LIMITE_GIRO)
    {
      doblar_volante(GIRO_LIMITE, sentido_giro);
      flag_rotacion = 1;
    }
    else
    {
      doblar_volante(GIRO_LEVE, sentido_giro);
      flag_rotacion = 1;
      flag_giro_leve = 1;
    }
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

  velocidad_temp = ((distancia_temp[motor] - distancia_temp_d[motor])*0.01)/DELTA_T;
  error = velocidad_ref - velocidad_temp;
  distancia_temp_d[motor] = distancia_temp[motor];

  p_controller[motor]  = kp * error;
  i_controller[motor] += ki * error * DELTA_T;

  if (i_controller[motor] >= SATURACION_INTEGRADOR)
    i_controller[motor] = SATURACION_INTEGRADOR;
  
  d_controller[motor]  = (kd * (error - prev_error[motor])) / DELTA_T;
  prev_error[motor]    = error;
  pid_contr      = p_controller[motor] + i_controller[motor] + d_controller[motor];
  //pid_contr      = p_controller[motor];
  movimiento[motor]    += pid_contr;
  if(movimiento[motor] > 60.0)
      movimiento[motor] = 60.0;
   
  return movimiento[motor];
}

// Filtro de moda para la deteccion de objetos del sensor de ultrasonido.
// Solo se considera objeto detectado si mas de la mitad de los datos lo confirman.
void filtrar_datos_ultrasonido(int indice)
{
  for(int i; i < MUESTRAS_DETECCION; i++)
  {
     if(i == MUESTRAS_DETECCION -1 )
     {
      if(ultr_distance[indice] < ULTR_LIMITE)
        contador_obstaculo[indice][(MUESTRAS_DETECCION-1) - i] = 1;
      else
        contador_obstaculo[indice][(MUESTRAS_DETECCION-1) - i] = 0;
     }
     else
      contador_obstaculo[indice][(MUESTRAS_DETECCION-1) - i] = contador_obstaculo[indice][(MUESTRAS_DETECCION-1) - i - 1];  
  }
  for(int i; i < MUESTRAS_DETECCION; i++)
  {
     if(i == MUESTRAS_DETECCION -1 )
     {
      if(ultr_distance[indice] > ULTR_LIMITE)
        contador_libre[indice][(MUESTRAS_DETECCION-1) - i] = 1;
      else
        contador_libre[indice][(MUESTRAS_DETECCION-1) - i] = 0;
     }
     else
      contador_libre[indice][(MUESTRAS_DETECCION-1) - i] = contador_libre[indice][(MUESTRAS_DETECCION-1) - i - 1];  
  }
  
  contador_deteccion[indice] = 0;
  contador_no_deteccion[indice] = 0;

  for(int j = 0; j < MUESTRAS_DETECCION; j++)
  {
    if(contador_obstaculo[indice][j])
      contador_deteccion[indice]++;
    if(contador_libre[indice][j])
      contador_no_deteccion[indice]++;
  }

  if(contador_deteccion[indice] >= LIMITE_MUESTRAS)
  {
    for(int i; i < MUESTRAS_DETECCION; i++)
      contador_obstaculo[indice][i] = 0; 
    flag_no_objeto_detectado[indice] = 0;
    flag_objeto_detectado[indice] = 1;
    distancia_objeto[indice] = ultr_distance[indice];   
  }

  if(contador_no_deteccion[indice] >= LIMITE_MUESTRAS)
  {
    for(int i; i < MUESTRAS_DETECCION; i++)
      contador_libre[indice][i] = 0; 
    flag_no_objeto_detectado[indice] = 1;
    flag_objeto_detectado[indice] = 0;  
  }
}

