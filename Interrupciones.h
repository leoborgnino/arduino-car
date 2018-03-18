#include <Arduino.h>
#include "ComunicacionUart.h"
#include "Movimientos.h"


extern const float PPV;
extern const float LONG_ARC; //longitud de arco de la rueda en cm
extern const float DIST_PULS;
extern const float GIRO_MIN_ITER;
extern const int   LIMITE_DESVIO;
extern const float LIMITE_REVERSA;

extern int         flag_centrar_vehiculo;
extern int         flag_cntrl_vel;
extern int         flag_back;
extern int         flag_terminar_movimiento;
extern int         flag_terminar_giro;
extern int         flag_timer;
extern int         flag_reversa_corta;

extern int         contador_pid;
extern int         respuestaid_plan;
extern int         completar_movimiento;
extern int         enderezar_volante;
extern int         esperar_volante;

extern float       grados_por_rotar;
extern float       centrar_vehiculo;


void ISR_Timer()
{
  if((distancia_temp[1] >= grados_volante_max) && (flag_girar_volante == 1))
  {
    analogWrite(PWM2, 127);
    flag_girar_volante = 0;
    distancia_temp[1] = 0;
    
    if(completar_movimiento == 2)
      completar_movimiento = 3;
    
    if(enderezar_volante == 2 || esperar_volante == 1)
      {
        enderezar_volante = 0;
        esperar_volante = 0;
        send_uart("0 !", respuestaid_plan);
      }
  }

  grados_por_rotar = grados_objetivo - valor_giro_total;

  if (((fabs(grados_por_rotar) < GIRO_MIN) && (flag_rotacion == 1)) || completar_movimiento == 1 || enderezar_volante == 1)
    {
      if(completar_movimiento == 1)
        completar_movimiento = 2;

      if(enderezar_volante == 1)
        enderezar_volante = 2;
               
      if(flag_giro_leve)
      {
        doblar_volante(GIRO_LEVE, !sentido_giro);
        flag_rotacion = 0;
        flag_giro_leve = 0;
      }
      else
      {
        doblar_volante(GIRO_MOV, !sentido_giro);
        flag_rotacion = 0;
      }
    }

  contador_pid = (contador_pid + 1) % CONTROL_PERIOD;
  if((contador_pid == 1) && flag_mover)
    flag_cntrl_vel = 1;

  if ( (( distancia_temp[0] > distancia_max) ) && (flag_mover == 1) )
    {
      analogWrite(PWM1, 127);
      
      if(flag_back)
        flag_back = 0;

      if(completar_movimiento == 4)
        completar_movimiento = 5;
        
      flag_mover = 0;
      flag_cntrl_vel = 0;  
      
      if(flag_rotacion == 1 && (fabs(grados_objetivo - valor_giro_total) > GIRO_MIN_ITER))
      {
        if(fabs(grados_objetivo - valor_giro_total) <= LIMITE_REVERSA)
          flag_reversa_corta = 1;
        else
          flag_reversa_corta = 0;

        completar_movimiento = 1;
      }
      else if(flag_rotacion == 1)
      {
        enderezar_volante = 1;  
      }
      else if(completar_movimiento == 0)
      { 
        if(flag_girar_volante == 1)
          esperar_volante = 1;
        else
          send_uart("0 !", respuestaid_plan);
      }
      distancia_temp[0] = 0;

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
  if (flag_girar_volante)
  {
    distancia_temp[1] = distancia_temp[1] + PASO_VOLANTE;
    if(sentido_giro)
      posicion_volante++;
    else
      posicion_volante--;
  }
  Serial.println(distancia_temp[1]);
}

