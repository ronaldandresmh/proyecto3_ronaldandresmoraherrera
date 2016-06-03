#include <QTRSensors.h>
QTRSensorsRC qtrrc((unsigned char[]) {7,6,5,4,3,2},6,2500,A3);  

#define led1          A5 
#define led2          9  
#define mot_i         12
#define mot_d         13
#define boton_1       0  //pin para boton
#define pin_pwm_i     10
#define pin_pwm_d     11
#define trimer_1      A0   //trimer proporcional
#define trimer_2      A1   //trimer integral
#define trimer_3      A2   //trimer derivativo

//variables para almacenar valores de sensores y posicion
unsigned int sensorValues[6];
unsigned int pos=0;

/// variables para el pid
int  derivativo=0, proporcional=0, integral=0;
int  salida_pwm=0, proporcional_pasado=0;

//AQUI CAMBIEREMOS LOS PARAMETROS DE NUESTRO ROBOT
int velocidad=120; //maximo es 255
float Kp=0, Kd=0, Ki=0;  //constantes PID

void setup()
{
 delay(800);
 pinMode(mot_i, OUTPUT);//pin de direccion motor izquierdo
 pinMode(mot_d, OUTPUT);//pin de direccion motor derecho
 pinMode(led1, OUTPUT); //led1
 pinMode(led2, OUTPUT); //led2
 pinMode(0, INPUT); //boton 1 como pull up, de iniciacion de procesos
 Kp=(analogRead(trimer_1))*(3.0/1023.0);
 Ki=(analogRead(trimer_2))*(1.0/1023.0);
 Kd=(analogRead(trimer_3))*(30.0/1023.0);

  
//CALIBRACIÓN
      
  for (int i = 0; i < 40; i++){ 
        digitalWrite(led2, HIGH);
        delay(20);
        qtrrc.calibrate();    //funcion para calibrar sensores 
        digitalWrite(led2, LOW);  
        delay(20);
  } 
        digitalWrite(led2, LOW); //apagar sensores para indicar fin de calibración
        delay(400); 
        digitalWrite(led2,HIGH); //encender led 2 para indicar la espera de pulsación del boton
                  
        while(digitalRead(0)); //sale al precionar el botón                    
        digitalWrite(led2,LOW); //indicamos que se presiono boton
        digitalWrite(led1,HIGH);//encendiendo led 1
        delay(2000);
}
               


void loop()
{
  pid(velocidad,Kp,Ki,Kd); //funcion para algoritmo pid pid(0, 120, 0.18, 4, 0.001); 
  frenos_contorno(700); //funcion para frenado en curvas tipo flanco de comparación va desde 0 hasta 1000 , esto para ver si esta en negro o blanco
}


////////funciones para el control del robot////

 void pid(int velocidad, float Kp, float Ki, float Kd)
{
  pos = qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, 0); //0 para linea negra, 1 para linea blanca
  proporcional = (pos) - 2500; // set point es 2500, asi obtenemos el error
  integral=integral + proporcional_pasado; //obteniendo integral
  derivativo = (proporcional - proporcional_pasado); //obteniedo el derivativo
  if (integral>1000) integral=1000; //limitamos la integral para no causar problemas
  if (integral<-1000) integral=-1000;
  
  salida_pwm =(proporcional*Kp)+(derivativo*Kd )+(integral*Ki); //ecuación del cotrolador
  if (salida_pwm > velocidad)  salida_pwm = velocidad; //limitamos la salida de pwm
  if (salida_pwm < -velocidad)  salida_pwm = -velocidad;
  
  if (salida_pwm < 0) motores(velocidad+salida_pwm, velocidad);
  if (salida_pwm > 0) motores(velocidad, velocidad-salida_pwm);
 proporcional_pasado = proporcional;  
}

void motores(int motor_izq, int motor_der)
{  
  if ( motor_izq >= 0 ){
  digitalWrite(mot_i,HIGH);             // con high avanza
  analogWrite(pin_pwm_i,255-motor_izq); //se controla de manera inversa para mayor control
 }
 else {
  digitalWrite(mot_i,LOW);    //con low retrocede
  motor_izq = motor_izq*(-1); //cambio de signo, para volver el PWM un valor positivo
  analogWrite(pin_pwm_i,motor_izq); 
  }

  if ( motor_der >= 0 ){
  digitalWrite(mot_d,HIGH);
  analogWrite(pin_pwm_d,255-motor_der);
 }
 else
 {
  digitalWrite(mot_d,LOW);
  motor_der= motor_der*(-1);
  analogWrite(pin_pwm_d,motor_der);
 }  
}

void frenos_contorno(int flanco_comparacion){
  
//si se salio por la parte derecha de la linea
if (pos<=500) {
    motores(10,-80); //el negativo es para contrarrestar la inercia 
    while(true) {
    qtrrc.read(sensorValues); //lectura en bruto de sensor   
      if ( sensorValues[0]>flanco_comparacion || sensorValues[1]>flanco_comparacion ){
      break; //cuando detecta la linea en los sensores de la parte derecha de nuevo
      } 
    }
}

//si se salió por la izquierda
if (pos>=4500){ 
    motores(-80,10);
    while(true){
      qtrrc.read(sensorValues);
      if (sensorValues[7]>flanco_comparacion || sensorValues[6]>flanco_comparacion ){
      break;
      }  
    }
}
}
