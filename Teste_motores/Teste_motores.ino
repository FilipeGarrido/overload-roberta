const int motor_vd = 7; //Velocidade do motor direito
const int motor_ve = 2; //Velocidade do motor esquerdo
const int motor_IN1 = 6; //Rotação do motor direito
const int motor_IN2 = 5;
const int motor_IN3 = 4; //Rotação do motor esquerdo
const int motor_IN4 = 3;

void setup() {
   pinMode(motor_vd, OUTPUT);
   pinMode(motor_ve, OUTPUT);
   pinMode(motor_IN1, OUTPUT);
   pinMode(motor_IN2, OUTPUT);
   pinMode(motor_IN3, OUTPUT);
   pinMode(motor_IN4, OUTPUT);

}

void loop() {
  analogWrite(motor_vd,150);
  digitalWrite(motor_IN3, HIGH);
  digitalWrite(motor_IN4, LOW);
  analogWrite(motor_ve, 200);
  digitalWrite(motor_IN1,LOW);
  digitalWrite(motor_IN2,HIGH);

}
