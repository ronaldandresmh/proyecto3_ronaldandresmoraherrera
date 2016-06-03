#include <QTRSensors.h>
#include <Wire.h>
QTRSensorsRC qtrrc((unsigned char[]) {8,7,6,5,4,3},8,2500,A3);  
QTRSensorsRC side((unsigned char[]) {9,2},2,2500,A3);  

//Direcciones de la IMU
#define MPU 0x68
#define A_R 16384.0
#define G_R 131.0
#define RAD_A_DEG = 57.295779
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
float Acc[2];
float Gy[2];
float Angle[2];

//direcciones de orientación cardinal
#define N             8
#define S             0x4
#define E             2
#define O             1
#define flanco        700
#define giro          1000 //tiempo de giro para 90 grados en milisegundos a una velocidad angular proporcional a 2*velocidad/radio 


#define trigPin       A2 //emisor de ultrasonido, PWM
#define echoPin       A1 //detector ultrasonoro
#define led2          A0  
#define mot_i         12
#define mot_d         13
#define boton_1       0  //pin para boton
#define pin_pwm_i     10
#define pin_pwm_d     11

//del algoritmo Flood fill
#define tam           5    //tamaño de la matriz que representa el laberinto
unsigned int M[tam][tam];//matrices de ubicacion
boolean MN[tam][tam];     //matriz de rastro

//variables para almacenar valores de sensores y posicion
unsigned int sensorValues[6];
unsigned int sensorValues_side[2];
unsigned int pos=0;
float F;

/// variables para el pid
int  derivativo=0, proporcional=0, integral=0;
int  salida_pwm=0, proporcional_pasado=0;
int velocidad=120; //maximo es 255
float Kp=0, Kd=0, Ki=0;  //constantes PID

void setup(){
Serial.begin(9600);

//matrice de ubicación
     for (int i = 0; i <= tam/2; i++) {
      for (int j = 0; j <= tam/2; j++) {
        M[i][j] =  tam-j-i-1;
        M[tam-i-1][j] =  tam-j-i-1;
        M[i][tam-j-1] =  tam-j-i-1;
        M[tam-i-1][tam-j-1] =  tam-j-i-1;
      }
     }

//Parámetros cardinales
Wire.begin();
Wire.beginTransmission(MPU);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);
float cardinalreference;
 //parámetros generales
 delay(800);
 pinMode(mot_i, OUTPUT);//pin de direccion motor izquierdo
 pinMode(mot_d, OUTPUT);//pin de direccion motor derecho
 pinMode(led2, OUTPUT); //led2
 pinMode(0, INPUT); //boton 1 como pull up, de iniciacion de procesos
 pinMode(trigPin,OUTPUT);
 pinMode(echoPin,INPUT);

  
//CALIBRACIÓN
      
  for (int i = 0; i < 40; i++){
        digitalWrite(led2, HIGH);
        delay(20);
        qtrrc.calibrate();    //funcion para calibrar sensores 
        side.calibrate();    //funcion para calibrar sensores 
        digitalWrite(led2, LOW);  
        delay(20); 
  } 
        digitalWrite(led2, LOW); //apagar sensores para indicar fin de calibración
        delay(400); 
        digitalWrite(led2,HIGH); //encender led 2 para indicar la espera de pulsación del boton
                  
        while(digitalRead(0)); //sale al precionar el botón                    
        digitalWrite(led2,LOW); //indicamos que se presiono boton
        delay(2000);
//calibración de la referencia angular
  cardinalreference=angulo();
}
               


void loop(){
   
if((1)&&(1)){
  uint8_t T=0;
  qtrrc.read(sensorValues);
  side.read(sensorValues_side);
  F=ultrasonido();
  //Establecimiento y envío de Trama
  if(F<=20)
    T=T|(1<<N);
  if(!(sensorValues[6]>flanco))
    T=T|(1<<E);
  if(!(sensorValues[1]>flanco))
    T=T|(1<<O);
  if(!(sensorValues[3]>flanco)&&!(sensorValues[4]>flanco)&&!(sensorValues[5]>flanco))
    T=T|(1<<S);
  
  Serial.println(T);  
    //Toma de deciciones
    if(T&4){
      Stop();
      }
      
    else if(T&2)//derecha
    {
      menos(giro);
      }
    else if(T&8)//sigue derecho
    {
      control();
      }
    else if(T&1)
    //izquierda{
      mas(giro);
      }
    else if(T&11)//da vuelta
    {
      mas(giro);
      mas(giro);
      }
  }
}

//funciónes de girado
void menos (int grad) //controla giro a la derecha
{
  while( angulo(); > 90*){
  digitalWrite(mot_i,HIGH);             
  analogWrite(pin_pwm_i,velocidad); 
  digitalWrite(mot_d,LOW);
  analogWrite(pin_pwm_d,velocidad); 

 
  }
void mas (int grad) //controla un giro en la izquierda
{
  digitalWrite(mot_i,LOW);             
  analogWrite(pin_pwm_i,velocidad); 
  digitalWrite(mot_d,HIGH);
  analogWrite(pin_pwm_d,velocidad); 
  angulo();
  delay(grad);
  }

void Stop ()
{
  //oprimir botón para renaudar
  }

void control()
{
      pid(velocidad,Kp,Ki,Kd); //funcion para algoritmo pid pid(0, 120, 0.18, 4, 0.001); 
      frenos_contorno(flanco); //funcion para frenado en curvas tipo flanco de comparación va desde 0 hasta 1000 , esto para ver si esta en negro o blanco  
  }
float ultrasonido()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return (0.017*pulseIn(echoPin, HIGH)); //convierte el tiempo a centimetros y envía
  }

float angulo(){
 //Leer los valores del Acelerometro de la IMU
   Wire.beginTransmission(MPU);
   Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,6,true); //A partir del 0x3B, se piden 6 registros
   AcX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
   AcY=Wire.read()<<8|Wire.read();
   AcZ=Wire.read()<<8|Wire.read();
 
    //A partir de los valores del acelerometro, se calculan los angulos Y, X
    //respectivamente, con la formula de la tangente.
   Acc[1] = atan(-1*(AcX/A_R)/sqrt(pow((AcY/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
 
   //Leer los valores del Giroscopio
   Wire.beginTransmission(MPU);
   Wire.write(0x43);
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,4,true); //A diferencia del Acelerometro, solo se piden 4 registros
   GyX=Wire.read()<<8|Wire.read();
 
   //Calculo del angulo del Giroscopio
   Gy[0] = GyX/G_R;
 
   //Aplicar el Filtro Complementario
  return (0.98 *(Angle[0]+Gy[0]*0.010) + 0.02*Acc[0]);
}

////////funciones para el control del robot////

 void pid(int velocidad, float Kp, float Ki, float Kd){
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
