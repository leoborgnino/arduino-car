#include <Arduino.h>
#include "ComunicacionUart.h"
#include "Movimientos.h"


extern const float PPV;
extern const float LONG_ARC; //longitud de arco de la rueda en cm
extern const float DIST_PULS;
extern const int   LIMITE_DESVIO;

extern int         flag_centrar_vehiculo;
extern int         flag_cntrl_vel;
extern int         flag_finished;
extern int         flag_back;
extern int         flag_terminar_movimiento;
extern int         flag_terminar_giro;
extern int         flag_timer;

extern int         contador_pid;
extern int         rotacion_incompleta;
extern int         respuestaid_plan;

extern float       grados_por_rotar;
extern float       centrar_vehiculo;


void ISR_Timer()
{
  if((distancia_temp[1] >= grados_volante_max) && (flag_girar == 1))
  {
    analogWrite(PWM2, 127);
    flag_girar = 0;
    distancia_temp[1] = 0;
  }

  if ((fabs(valor_giro_temp) > grados_max) && (flag_rotacion == 1))
    {
      if(flag_giro_leve)
      {
        doblar_volante(GIRO_LEVE, !sentido_giro);
        flag_rotacion = 0;
        flag_giro_leve = 0;
        valor_giro_temp = 0;
      }
      else
      {
        doblar_volante(GIRO_MOV, !sentido_giro);
        flag_rotacion = 0;
        valor_giro_temp = 0;
      }
    }

  if(flag_linea_recta)
  {
     centrar_vehiculo = 0;//valor_giro_instantaneo - valor_giro_total;
     if(fabs(centrar_vehiculo) > LIMITE_DESVIO)
       flag_centrar_vehiculo = 1;
  }
  
  contador_pid = (contador_pid + 1) % CONTROL_PERIOD;
  if((contador_pid == 1) && flag_mover)
    flag_cntrl_vel = 1;

  if ( (( distancia_temp[0] > distancia_max) ) && (flag_mover == 1) )
    {
      flag_finished = 1;
      analogWrite(PWM1, 127);
      flag_mover = 0;
      flag_linea_recta = 0;
      flag_cntrl_vel = 0;  
    
      if (flag_back == 1)
      {
        flag_terminar_movimiento = 1;
        flag_back = 0;
      }
    
      if(flag_rotacion == 1 && ((grados_max - fabs(valor_giro_temp)) > GIRO_MIN) )
        rotacion_incompleta = 1;  
      else
        send_uart("0 !", respuestaid_plan);
      
      distancia_temp[0] = 0;

    }

  if(rotacion_incompleta == 1)
  {
    if(flag_giro_leve)
    {
      doblar_volante(GIRO_LEVE, !sentido_giro);
      flag_giro_leve = 0;
    }
    else
    {
      doblar_volante(GIRO_MOV, !sentido_giro);
    }
    grados_por_rotar = grados_max - fabs(valor_giro_temp);
    flag_terminar_giro = 1;
    flag_rotacion = 0;
    valor_giro_temp = 0;
    rotacion_incompleta = 0;
  }
  flag_timer = 1;
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

