

/// Código destinado a Olimpíada Brasileira de Robótica
/// OBR Modalidade prátical virtual 2023 - Categoria Programação
/// Equipe diamante: OBR 2023 ///
/// Membros da equipe: Huan Lin Fui
///                    Maurício Calvet
///                    Sabino Pinheiro
///                    Maria Clara Palhano


// Definindo Bibliotecas para o controle dos motores
#include <TwoMotors.h>
#include <DC_motor_controller.h>
#include <Servo.h>

//Nomeando os motores
DC_motor_controller motorR;
DC_motor_controller motorL;

//Variável para o controle paralelo dos motores
TwoMotors both(&motorR, &motorL);


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
//////////// servo braço  garra:
Servo garraB;
///////////// servos abriri e fecha a garra :
Servo garraL;
Servo garraD;
 /////////// servos ultra:
 Servo garraU;

// Funções para controle da interrupção do motor
void interruptR () {
  motorR.isr();
}
void interruptL () {
  motorL.isr();
}


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

void ultraleituraF() {
  digitalWrite(trigF, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigF, LOW);
  tempoF = pulseIn(echoF, HIGH);
  distanciaF = tempoF / 29.4 / 2;
}

void ultraleituraT() {
  digitalWrite(trigT, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigT, LOW);
  tempoT = pulseIn(echoT, HIGH);
  distanciaT = tempoT / 29.4 / 2;
}

void ultraleituraS() {
  digitalWrite(trigS, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigS, LOW);
  tempoS = pulseIn(echoS, HIGH);
  distanciaS = tempoS/ 29.4 / 2;
}



void pegar (int vel2, int rotations) {

baixargarra();
opengarra();
both.together(vel2,rotations);
closegarra();
armgarra();

  
}

void baixargarra () {
garraB.write(0);

}

void armgarra() {
garraB.write(180);
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

void ultramoveS (int angle) {

garraU.write(angle);
  
}


void setup() {
  motorR.hBridge(10, 9, 11);       // Pinos da ponte H
  motorR.setEncoderPin(3, 5); // Pinos do encoder
  motorR.setRR(21.3);    // Razão da caixa de redução é 30
  motorR.setPIDconstants(kp,ki,kd);
  motorR.setPins();
  motorR.invertDirection();
  attachInterrupt(digitalPinToInterrupt(3), interruptR, FALLING);

  motorL.hBridge(7, 8, 6);       // Pinos da ponte H
  motorL.setEncoderPin(2, 4); // Pinos do encoder
  motorL.setRR(21.3);    // Razão da caixa de redução é 30
  //motorL.setPPR(11); // 11 pulsos que o encoder envia para cada volta dada em   // torno de seu eixo
    motorL.setPIDconstants(kp,ki,kd);
  motorL.setPins();
  
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

  // Declaração de servos : 

  garraB.attach(0);
  garraL.attach(0);
  garraD.attach(0);
  garraU.attach(34);

  // Iniciando monitor serial
  Serial.begin(9600);

 

}
void loop() {
  ultraleituraS();
  ultraleituraF();
  garraU.write(0);
 garraU.write(8);
 
 Serial.println(distanciaF);

if (distanciaS <= 15 ) {
  motorR.walk(20);
  motorL.walk(90);
}

if ( distanciaS >= 15) {
   
   motorR.walk(90);
  motorL.walk(20);
}

if ( distanciaF <= 10 && distanciaF >= 1 ) {
   
   both.turnDegree(100,90);
}

}
