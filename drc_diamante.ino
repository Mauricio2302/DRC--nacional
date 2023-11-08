





/// Equipe diamante: Fira 2023 ///
/// Membros da equipe: Huan Lin Fui
///                    Maurício Calvet
///                    Sabino Pinheiro


// Definindo Bibliotecas para o controle dos motores
#include <TwoMotors.h>
#include <DC_motor_controller.h>
#include <Servo.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_TCS34725.h>
#include <VL53L0X.h>


//Nomeando os motores
DC_motor_controller motorR;
DC_motor_controller motorL;

//Variável para o controle paralelo dos motores
TwoMotors both(&motorR, &motorL);
///// sensor de distancia a laser
VL53L0X sensor;



// Constantes do PID
int kp = 1.8; // Valor da proporcional
int ki = 1.5; // Valor da integral
int kd = 0.06; // Valor da derivada

// Variáveis ultrassônico
int trigF = 26;
int echoF = 24;
int distanciaF;
int tempoF;

//ULTRASSONICO SERVO :

int trigS = 37;
int echoS = 36;
int distanciaS;
int tempoS;

// ULTRASSONICO LADO :

int trigL = 14;
int echoL = 15;
int distanciaL;
int tempoL;

// ULTRASSONICO TRÁS :

int trigT = 14;
int echoT = 15;
int distanciaT;
int tempoT;

// Servos :
//////////// servos braços  garra:
Servo garraBL;
Servo garraBD;
///////////// servos abrir e fecha a garra :
Servo garraL;
Servo garraD;
 /////////// servos ultra:
 Servo garraU;

/////////// servos sensor d cor :

Servo garraSC;

// Funções para controle da interrupção do motor
void interruptR () {
  motorR.isr();
}
void interruptL () {
  motorL.isr();
}

/* Initialise with default values (int time = 2.4ms, gain = 1x) */
// Adafruit_TCS34725 tcs = Adafruit_TCS34725();

/* Initialise with specific in t time and gain values */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

//funções para movimento dos motores
void esquerda() {
  motorR.walk(170);
  motorL.walk(-140);
}
void direita() {
  motorR.walk(-140);
  motorL.walk(170);
}
void frente() {
  motorR.walk(130);
  motorL.walk(130);
}
void meiavolta() {
  both.together(180, 2, -160, -2);
}

// Função para leitura do ultrassônico
void ultraleituraL() {
  digitalWrite(trigL, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigL, LOW);
  tempoL = pulseIn(echoL, HIGH);
  distanciaL = tempoL / 29.4 / 2;
}





/*void pegar (int vel2, int rotations) {

baixargarra();
opengarra();
both.together(vel2,rotations);
closegarra();
armgarra();

  
}

void baixargarra () {
garraBD.write(0);
garraBL.write(0);

}

void armgarra() {
garraBL.write(0);
}

void soltar () {
baixargarra();
opengarra();

}

void opengarra () {
garraL.write(0);
garraD.write(0);
}

void closegarra () {
garraL.write(180);
garraD.write(180);
}

void verificar() {


}

void ultramoveS (int angle) {

garraU.write(angle);
  
}*/


void setup() {

  // Declaração de servos : 

 /* garraBL.attach(28);
  garraBD.attach(26);
  garraL.attach(32);
  garraD.attach(30);
  garraU.attach(34);
  garraSC.attach(36);*/ 
  
 motorR.hBridge (10, 9, 11 );       // Pinos da ponte H
  motorR.setEncoderPin(3, 5); // Pinos do encoder
  motorR.setRR(21.3);    // Razão da caixa de redução é 30
  motorR.setPIDconstants(kp,ki,kd);
  motorR.setPins();
  
  attachInterrupt(digitalPinToInterrupt(3), interruptR, FALLING);

  motorL.hBridge(7, 8, 6 );       // Pinos da ponte H
  motorL.setEncoderPin(2, 4); // Pinos do encoder
  motorL.setRR(21.3);    // Razão da caixa de redução é 30
  //motorL.setPPR(11); // 11 pulsos que o encoder envia para cada volta dada em   // torno de seu eixo
    motorL.setPIDconstants(kp,ki,kd);
  motorL.setPins();
  motorL.invertDirection();
  attachInterrupt(digitalPinToInterrupt(2), interruptL, FALLING);

  both.setGyreDegreesRatio(0.8,90);

 

  // Definiindo se os pinos do ultrassônico sao OUTPUT ou INPUT
  pinMode(trigS, OUTPUT);
  pinMode(echoS, INPUT);
  
  pinMode(trigF, OUTPUT);
  pinMode(echoF, INPUT);
  
  pinMode(trigL, OUTPUT);
  pinMode(echoL, INPUT);
  
  pinMode(trigT, OUTPUT);
  pinMode(echoT, INPUT);



  // Inicializa a comunicação serial
  Serial.begin(9600);
  // Inicializa a comunicação I2C
  Wire.begin();

  // Inicializa o sensor
  sensor.init();
  // Define um timeout de 500mS para a leitura do sensor
  // Em caso de erro, este será o tempo máximo de espera da resposta do sensor
  sensor.setTimeout(500);
  
//Inicia o sensor e diz se ele conectou ou não
 if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
  }

  // Now we're ready to get readings!
 

}
void loop() {

  // Faz a medição da distância e retorna um valor em milímetros
  int dist = sensor.readRangeSingleMillimeters();
 
  // Imprime no monitor serial
  Serial.println(dist);
  
/*uint16_t r, g, b, c, colorTemp, lux;

  tcs.getRawData(&r, &g, &b, &c);
  // colorTemp = tcs.calculateColorTemperature(r, g, b);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);

  Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  Serial.println(" ");

  
  ultraleituraS();
  ultraleituraF();
 armgarra();
 
 
*/
if (dist <= 155 ) {
       motorR.walk(90);
 motorL.walk(20);


 
}

if ( dist >= 155) {
motorR.walk(20);
 motorL.walk(90);
   
 
}
//if ( c >= 750) {
  
  // both.together(-60,-0.4 );
 // both.turnDegree(-100,-90);
//;}


}
