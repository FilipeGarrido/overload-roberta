const int sensor_linha1 = A0 ;
const int sensor_linha2 = A1;
const int sensor_linha3 = A2;
const int sensor_linha4 = A3;
const int sensor_curso = A5;

int i=0;

void setup ()
{
  digitalWrite(5,HIGH);
  digitalWrite(4,HIGH);
  Serial.begin(9600);

}

void loop()
{
  i=0;
  while(i<300){
  Serial.print(digitalRead(sensor_linha1));
  Serial.print(digitalRead(sensor_linha2));
  Serial.print(digitalRead(sensor_linha3));
  Serial.print(digitalRead(sensor_linha4));
  Serial.println(digitalRead(sensor_curso));
  delay(1000);
  i++;
  } 
}
