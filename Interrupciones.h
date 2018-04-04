#include <Arduino.h>
#include "ComunicacionUart.h"
#include "Movimientos.h"


extern const float PPV;
extern const float LONG_ARC; //longitud de arco de la rueda en cm
extern const float DIST_PULS;
extern const float GIRO_MIN_ITER;
extern const int   LIMITE_DESVIO;
extern const float LIMITE_REVERSA;
extern const float ULTR_LIMITE;
extern const int   N_ULTR_SENSOR;
extern const int   ULTR_PERIOD;
extern const int   ULTRA_TRIGER;
extern const int   ULTRA_ECHO;
extern const int   ULTRB_TRIGER;
extern const int   ULTRB_ECHO;

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
extern int         objeto_detectado;

extern float       grados_por_rotar;
extern float       centrar_vehiculo;
extern double      distancia_abs;

extern double      ultr_distance[2];
extern double      distancia_objeto[2];
extern long        ultr_start_time[2];

int contador_ultrasonido = 0;

void ISR_Timer()
{
  // Se evalua si hay un objeto cercano
  if(((ultr_distance[0] < ULTR_LIMITE)||(ultr_distance[1] < ULTR_LIMITE)) && (objeto_detectado = 0))
  {
    objeto_detectado = 1;
    distancia_objeto[0] = ultr_distance[0];
    distancia_objeto[1] = ultr_distance[1];
  }

  // Condiciones de corte volante
  
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

  // Condiciones de corte de grados a rotar
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

  // Condiciones para control PID
  contador_pid = (contador_pid + 1) % CONTROL_PERIOD;
  if(contador_pid == 1)
    flag_cntrl_vel = 1;

  // Condiciones de distancia de desplazamiento
  if ( ((distancia_temp[0] > distancia_max) && (flag_mover == 1)) || (objeto_detectado == 1 && (flag_mover == 1)) )
    {
      analogWrite(PWM1, 127);
      
      if(flag_back)
        flag_back = 0;
        
      flag_mover = 0;
      flag_cntrl_vel = 0;  
      
      if(((flag_rotacion == 1) && (fabs(grados_objetivo - valor_giro_total) > GIRO_MIN_ITER)) || objeto_detectado == 1)
      {
        if(objeto_detectado == 1)
        {
          if(flag_rotacion == 1)
          {
            completar_movimiento = 1;
            objeto_detectado = 2;
            flag_reversa_corta = 0;
          }
          else
          {
            completar_movimiento = 3;
            objeto_detectado = 2;
            flag_reversa_corta = 0;
          }  
        }
        else
        {
          if(fabs(grados_objetivo - valor_giro_total) <= LIMITE_REVERSA)
            flag_reversa_corta = 1;
          else
            flag_reversa_corta = 0;

          completar_movimiento = 1;
        }
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

      if(completar_movimiento == 4)
      {
        if(objeto_detectado == 2)
        {
          completar_movimiento = 0;
          objeto_detectado = 3;
        }
        else
        {
          completar_movimiento = 5;
        }
      }
        
      distancia_temp[0] = 0;

    }
    
  // Condiciones de Trigger del ultrasonido
  contador_ultrasonido = (contador_ultrasonido + 1) % ULTR_PERIOD;
  if(contador_ultrasonido == 1)
  {
    digitalWrite(ULTRA_TRIGER,HIGH); //se envia el pulso ultrasonico
    digitalWrite(ULTRB_TRIGER,HIGH); //se envia el pulso ultrasonico
    delayMicroseconds(10);//El pulso debe tener una duracion minima de 10 microsegundos // FIX ME (Estaba en 20)
    digitalWrite(ULTRA_TRIGER,LOW); //Ambas lineas son por estabilizacion del sensor
    digitalWrite(ULTRB_TRIGER,LOW); //Ambas lineas son por estabilizacion del sensor
  }


  flag_timer = 1;
}


void ISR_INTE0()//PIN2 Motor Principal
{
  if (flag_mover)
    distancia_temp[0] = distancia_temp[0] + DIST_PULS;
  distancia_abs = distancia_abs + DIST_PULS;
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
  //Serial.println(distancia_temp[1]);
}

void ISR_ECHOA_INT()
{
 if(digitalRead(ULTRA_ECHO))
  ultr_start_time[0] = micros();
 else
  ultr_distance[0] = (micros()-ultr_start_time[0])/58.0;
 }

 void ISR_ECHOB_INT()
{
 if(digitalRead(ULTRB_ECHO))
  ultr_start_time[1] = micros();
 else
  ultr_distance[1] = (micros()-ultr_start_time[1])/58.0;
 }


