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
extern const int   ULTRC_TRIGER;
extern const int   ULTRC_ECHO;
extern const int   MODO_ULTRASONIDO;


extern int         flag_centrar_vehiculo;
extern int         flag_cntrl_vel;
extern int         flag_back;
extern int         flag_terminar_movimiento;
extern int         flag_terminar_giro;
extern int         flag_timer;
extern int         flag_reversa_corta;
extern int         flag_ultrasonido;
extern int         flag_ultr_request;
extern int         ultr_flag_start;

extern int         contador_pid;
extern int         respuestaid_plan;
extern int         completar_movimiento;
extern int         enderezar_volante;
extern int         esperar_volante;
extern int         objeto_detectado;

extern float       grados_por_rotar;
extern float       centrar_vehiculo;
extern double      distancia_abs;

extern double      ultr_distance[3];
extern double      distancia_objeto[3];
extern long        ultr_start_time[3];

int contador_ultrasonido = 0;

void ISR_Timer3()
{
 if((digitalRead(ULTRA_ECHO)) && (ultr_flag_start == 0) && (flag_ultr_request == 1))
 {
  ultr_start_time[0] = micros();
  ultr_flag_start = 1;
 }
 else if ( (ultr_flag_start == 1) && (flag_ultr_request == 1) && (digitalRead(ULTRA_ECHO) == 0))
 {
  ultr_distance[0] = (micros()-ultr_start_time[0])/58.0;
  if(objeto_detectado == 0)
    filtrar_datos_ultrasonido(0);
  ultr_flag_start = 0;
 }
 
  if((digitalRead(ULTRB_ECHO)) && (ultr_flag_start == 0) && (flag_ultr_request == 2))
 {
  ultr_start_time[1] = micros();
  ultr_flag_start = 1;
 }
 else if ( (ultr_flag_start == 1) && (flag_ultr_request == 2) && (digitalRead(ULTRB_ECHO) == 0))
 {
  ultr_distance[1] = (micros()-ultr_start_time[1])/58.0;
  if(objeto_detectado == 0)
    filtrar_datos_ultrasonido(1);
  ultr_flag_start = 0;
 }

if((digitalRead(ULTRC_ECHO)) && (ultr_flag_start == 0) && (flag_ultr_request == 0))
 {
  ultr_start_time[2] = micros();
  ultr_flag_start = 1;
 }
 else if ( (ultr_flag_start == 1) && (flag_ultr_request == 0) && (digitalRead(ULTRC_ECHO) == 0))
 {
  ultr_distance[2] = (micros()-ultr_start_time[2])/58.0;
  ultr_flag_start = 0;
 }  
   
}
void ISR_Timer()
{
  // Se evalua si hay un objeto cercano adelante
  if(((flag_objeto_detectado[0])||(flag_objeto_detectado[1])) && (objeto_detectado == 0) && (flag_mover == 1) && (flag_back == 0))
  {
    if(flag_objeto_detectado[0] == 0)
      Serial.println("Sensor A");
    else
      Serial.println("Sensor B");
      
    flag_objeto_detectado[0] = 0;
    flag_objeto_detectado[1] = 0;
    objeto_detectado = MODO_ULTRASONIDO;
    distancia_objeto[0] = distancia_objeto[0] + distancia_temp[0];
    distancia_objeto[1] = distancia_objeto[1] + distancia_temp[0];
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
  {
    flag_cntrl_vel = 1;
    Serial.print("SENSOR A: "); Serial.println(flag_objeto_detectado[0]);
    Serial.print("SENSOR B: "); Serial.println(flag_objeto_detectado[1]);
  } 

  // Condiciones de distancia de desplazamiento
  if ( ((distancia_temp[0] > distancia_max) && (flag_mover == 1)) || ((objeto_detectado == 1) && (flag_mover == 1)) )
    {
      analogWrite(PWM1, 127);
        
      flag_mover = 0;
      flag_cntrl_vel = 0;  
      
      if(((flag_rotacion == 1) && (fabs(grados_objetivo - valor_giro_total) > GIRO_MIN_ITER)) || (objeto_detectado == 1))
      {
        Serial.print("ITERACION");
        if(objeto_detectado == 1)
        {
          Serial.print(" OBJ DETECTADO");
          if(flag_rotacion == 1)
          {
            Serial.println(" ROTANDO");
            completar_movimiento = 1;
            objeto_detectado = 2;
            flag_reversa_corta = 0;
          }
          else
          {
            Serial.println(" RECTO");
            completar_movimiento = 3;
            objeto_detectado = 2;
            flag_reversa_corta = 0;
          }  
        }
        else
        {
          Serial.println(" COMUN");
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

      if(flag_back)
        flag_back = 0;
        
      distancia_temp[0] = 0;

    }
    
  // Condiciones de Trigger del ultrasonido
  contador_ultrasonido = (contador_ultrasonido + 1) % ULTR_PERIOD;
  flag_ultrasonido = (contador_ultrasonido == 0);


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

