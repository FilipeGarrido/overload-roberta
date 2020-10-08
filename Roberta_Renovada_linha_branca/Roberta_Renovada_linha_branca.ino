int motor_vd = 11; //Velocidade do motor direito
int motor_ve = 10; //Velocidade do motor esquerdo
int motor_IN1 = 13; //Rotação do motor direito
int motor_IN2 = 12;
int motor_IN3 = 9; //Rotação do motor esquerdo
int motor_IN4 = 8;

int error_c=0;

//velocidade inicial dos motores
int v=150;

//contador de parada
int i =0;

//Declaração dos sensores de linha
const int sensor_linha1 = A0;
const int sensor_linha2 = A1;
const int sensor_linha3 = A2;
const int sensor_linha4 = A3;
//Declaração dos sensores de curva e de começo/fim de curso
//const int sensor_curva = A5;
const int sensor_curso = A5;

//Variáveis para o controle pwm dos motores
int controle_pwm_direito=0;
int controle_pwm_esquerdo=0;

//Variável responsável por definir a condição de curva e de começo e fim de curso
int condicao=0;
//int condicao_pista=0;
int aux_1=0;//variável auxiliar para o controle de começo e fim de curso
//int aux_2=0;//variável auxiliar para o controle de curva

/*=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
/*=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/

//Aplicação do PID
class PID{
public:

double error;
double sample;
double lastSample;
double kP, kI, kD, _kP, _kI, _kD;      
double P, I, D;
double pid;

  
double setPoint;
long lastProcess;

PID(double _kP, double _kI, double _kD){
    kP = _kP;
    kI = _kI;
    kD = _kD;
  }

  double erro (){
    //teste linha branca
    /*erro0*/
    /*--------------------------------------------------------------------------------------------------------------------------------*/
    if((digitalRead(sensor_linha1)==1)&&(digitalRead(sensor_linha2)==0)&&(digitalRead(sensor_linha3)==0)&&(digitalRead(sensor_linha4)==1))
    {
      return 0;
    }
     if((digitalRead(sensor_linha1)==0)&&(digitalRead(sensor_linha2)==0)&&(digitalRead(sensor_linha3)==0)&&(digitalRead(sensor_linha4)==0))
    {
      return 0;
    }
    /*erro1*/
    /*--------------------------------------------------------------------------------------------------------------------------------*/
    if((digitalRead(sensor_linha1)==1)&&(digitalRead(sensor_linha2)==1)&&(digitalRead(sensor_linha3)==0)&&(digitalRead(sensor_linha4)==1))
    {
      return 1;
    }
     /*erro2*/
    /*--------------------------------------------------------------------------------------------------------------------------------*/
    if((digitalRead(sensor_linha1)==1)&&(digitalRead(sensor_linha2)==1)&&(digitalRead(sensor_linha3)==0)&&(digitalRead(sensor_linha4)==0))
    {
      return 2;
    }
     /*erro3*/
    /*--------------------------------------------------------------------------------------------------------------------------------*/
      if((digitalRead(sensor_linha1)==1)&&(digitalRead(sensor_linha2)==1)&&(digitalRead(sensor_linha3)==1)&&(digitalRead(sensor_linha4)==0))
    {
      return 3;
    }
     /*erro-1*/
    /*--------------------------------------------------------------------------------------------------------------------------------*/
       if((digitalRead(sensor_linha1)==1)&&(digitalRead(sensor_linha2)==0)&&(digitalRead(sensor_linha3)==1)&&(digitalRead(sensor_linha4)==1))
    {
      return -1;
    }

    /*erro-2*/
    /*--------------------------------------------------------------------------------------------------------------------------------*/
    if((digitalRead(sensor_linha1)==0)&&(digitalRead(sensor_linha2)==0)&&(digitalRead(sensor_linha3)==1)&&(digitalRead(sensor_linha4)==1))
    {
      return -2;
    }
        /*erro-3*/
    /*--------------------------------------------------------------------------------------------------------------------------------*/
    if((digitalRead(sensor_linha1)==0)&&(digitalRead(sensor_linha2)==1)&&(digitalRead(sensor_linha3)==1)&&(digitalRead(sensor_linha4)==1))
    {
      return -3;
    }
    if((digitalRead(sensor_linha1)==1)&&(digitalRead(sensor_linha2)==1)&&(digitalRead(sensor_linha3)==1)&&(digitalRead(sensor_linha4)==1))
    {
      if(error_c<0){
        return -4;
      }
      if(error_c>0){
        return 4;
      }
    }
    

  }


  void addNewSample( int erro(), int Velocidade)
  {
    sample = erro(); // sample é erro pra cada condição dos sensores
  }
  
  void setSetPoint()
  {
    setPoint = 0; //valor do erro que queremos atingir
  }
  
  double process()
  {
    
    error = setPoint-sample;
    
    float deltaTime = (millis() - lastProcess) / 1000.0; // serve para agilizar o processo de analise do erro, ele recebe o tempo atual do loop menos o tempo do ultimo loop e divide por mil pra termos o tempo em segundos
    
    lastProcess = millis();
    
    
    P = error * kP;
    
    I = I + (error * kI) * deltaTime;
    
    D = (lastSample - sample) * kD / deltaTime;
    
    lastSample = sample;
    
    pid = P + I + D;
    
    return pid;
  }
};

PID meuPid(0.8 , 0.0 , 0.0);

void setup() {
   pinMode(motor_vd, OUTPUT);
   pinMode(motor_ve, OUTPUT);
   pinMode(motor_IN1, OUTPUT);
   pinMode(motor_IN2, OUTPUT);
   pinMode(motor_IN3, OUTPUT);
   pinMode(motor_IN4, OUTPUT);


   pinMode(sensor_linha1,INPUT);
   pinMode(sensor_linha2,INPUT);
   pinMode(sensor_linha3,INPUT);
   pinMode(sensor_linha4,INPUT);
   pinMode(sensor_curso,INPUT);

   digitalWrite(5,HIGH);
   digitalWrite(4,HIGH);
   
   Serial.begin(9600);

}

void loop() {
   error_c = meuPid.erro();
   int pid=meuPid.process();
   meuPid.process();
   meuPid.erro();
/*
   Serial.print(digitalRead(sensor_linha1));
   Serial.print(';');
   Serial.print(digitalRead(sensor_linha2));
   Serial.print(';');
   Serial.print(digitalRead(sensor_linha3));
   Serial.print(';');
   Serial.print(digitalRead(sensor_linha4));
   Serial.print(';');
   Serial.print(pid);
   Serial.print(';');
   Serial.println(error);
   delay(1000);
*/  

/* O sensor_curso é responsável por receber o sinal do sensor de comço/fim de percurso*/

   if((digitalRead(sensor_curso)==0)){
      aux_1++;
      if(aux_1==8){
        condicao=1;
        delay(100);
      }
      else{
        condicao=0;
      }
      delay(100);
   }
   
/* O primeiro switch é responsável pela condição de começo/fim do percurso. O caso 0 será
responsável pelo controle do percurso. Dentro dele há um segundo switch responsável pelo
controle de velocidade dentro e fora das curvas. O caso 1 é responsável pela parada dos 
motores ao finalizar o percurso*/
   
   switch(condicao){
    case 0:
          //mantem o valor de saida dos motores entre 0 e 255;
          constrain(controle_pwm_direito,0,255);
          constrain(controle_pwm_esquerdo,0,255);
          
          controle_pwm_esquerdo=(v+meuPid.process());
          controle_pwm_direito=(v-meuPid.process());

           if(error_c==0){
            analogWrite(motor_vd,v);
            digitalWrite(motor_IN3, LOW);
            digitalWrite(motor_IN4, HIGH);
            analogWrite(motor_ve, v);
            digitalWrite(motor_IN1,HIGH);
            digitalWrite(motor_IN2,LOW);  
          }

          if(error_c>0){
            analogWrite(motor_vd,controle_pwm_direito);
            digitalWrite(motor_IN3, LOW);
            digitalWrite(motor_IN4, HIGH);
            analogWrite(motor_ve, v);
            digitalWrite(motor_IN1,HIGH);
            digitalWrite(motor_IN2,LOW);  
          }
          
           if(error_c<0){
            analogWrite(motor_vd,v);
            digitalWrite(motor_IN3, LOW);
            digitalWrite(motor_IN4, HIGH);
            analogWrite(motor_ve, controle_pwm_esquerdo);
            digitalWrite(motor_IN1,HIGH);
            digitalWrite(motor_IN2,LOW);  
          }
          
    break;
    case 1:

          while(i<=2000){
          constrain(controle_pwm_direito,0,255);
          constrain(controle_pwm_esquerdo,0,255);
          
          controle_pwm_esquerdo=(v+meuPid.process());
          controle_pwm_direito=(v-meuPid.process());

           if(error_c==0){
            analogWrite(motor_vd,v);
            digitalWrite(motor_IN3, LOW);
            digitalWrite(motor_IN4, HIGH);
            analogWrite(motor_ve, v);
            digitalWrite(motor_IN1,HIGH);
            digitalWrite(motor_IN2,LOW);  
          }

          if(error_c>0){
            analogWrite(motor_vd,controle_pwm_direito);
            digitalWrite(motor_IN3, LOW);
            digitalWrite(motor_IN4, HIGH);
            analogWrite(motor_ve,v);
            digitalWrite(motor_IN1,HIGH);
            digitalWrite(motor_IN2,LOW);  
          }
          
           if(error_c<0){
            analogWrite(motor_vd,v);
            digitalWrite(motor_IN3, LOW);
            digitalWrite(motor_IN4, HIGH);
            analogWrite(motor_ve, controle_pwm_esquerdo);
            digitalWrite(motor_IN1,HIGH);
            digitalWrite(motor_IN2,LOW);  
          }
          i++;
        }
          analogWrite(motor_vd,0);
          digitalWrite(motor_IN3, HIGH);
          digitalWrite(motor_IN4, HIGH);
          analogWrite(motor_ve,0);
          digitalWrite(motor_IN1,HIGH);
          digitalWrite(motor_IN2,HIGH);
    break;
   }
}
