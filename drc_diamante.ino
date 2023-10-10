

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
int trigF = 14;
int echoF = 15;
int distanciaF;
int tempoF;

//ULTRASSONICO SERVO :

int trigS = 14;
int echoS = 15;
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
void ultraleitura() {
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  tempo = pulseIn(echo, HIGH);
  distancia = tempo / 29.4 / 2;
}



void pegar (int vel2 int rotations) {

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
  //Definindo motores
  motorR.hBridge(7, 8, 6 );      // Pinos da ponte H
  motorR.setEncoderPin(3, 5); // Pinos do encoder
  motorR.setRR(21.3);    // Razão da caixa de redução
  motorR.setPIDconstants(kp, ki, kd); // Definindo constantes do PID
  motorR.setPins(); // Definindo pinos do motor
  attachInterrupt(digitalPinToInterrupt(3), interruptR, FALLING);

  motorL.hBridge(10, 9, 11);       // Pinos da ponte H
  motorL.setEncoderPin(2, 4); // Pinos do encoder
  motorL.setRR(21.3);    // Razão da caixa de redução
  motorL.setPIDconstants(kp, ki, kd); // Definindo constantes do PID
  motorL.setPins(); // Definindo pinos do motor
  motorL.invertDirection(); // Invertendo a direção dos motores
  attachInterrupt(digitalPinToInterrupt(2), interruptL, FALLING);

  pinMode(LED_BUILTIN, OUTPUT); // Dizendo que o led vai enviar um sinal, OUTPUT

  // Interrupção externa

  digitalWrite(LED_BUILTIN , LOW); // Deixando o Led desligado, assim evitando falso-positivo

  // Definiindo se os pinos do ultrassônico sao OUTPUT ou INPUT
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  // Declaração de servos : 

  garraB.attach(0);
  garraL.attach(0);
  garraR.attach(0);
  garraU.attach(0);

  // Iniciando monitor serial
  Serial.begin(9600);

}
void loop() {

  // Verificando se está havendo comunicação serial
  if (Serial.available()) {
    // Declarando variável para fazer a leitura das informações no monitor serial
    char lido = char(Serial.read());

    // Ligando o led, para indicar que o código está sendo executado
    digitalWrite(LED_BUILTIN , HIGH);
    ultraleitura();

    if (lido == '0') { // Linha a esquerda
      esquerda();
    }
    if (lido == '1') { // Linha a direita
      direita();
    }
    if (lido == '2') { // Linha ao centro
      frente();
    }

    if (lido == '3') { // Verde a esquerda
      esquerda();
    }

    if (lido == '4') { // Verde a direita
      direita();
    }
    if (lido == '6') { // Dois verdes identificados, meia volta
      meiavolta();

    }
  
  }

}
