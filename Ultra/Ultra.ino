
#define echo 7
#define trig 8
#define delay_samp 12.5

long duration;
float distance;
int val_read;

void setup()
{
  pinMode(echo,INPUT);
  pinMode(A0,INPUT);
  pinMode(trig,OUTPUT);
  Serial.begin(19200);
}

void loop()
{
//  Serial.println("Entering LOOP NOW**********");
  int count1 = 0;
  int read_adc;
  float ret_val;
  digitalWrite(trig,LOW);
  delayMicroseconds(2);
  
  digitalWrite(trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);
  
  duration = pulseIn(echo,HIGH);
//  val_read = analogRead(A0);
//  Serial.println("ADC Value: ");
//  Serial.println(val_read);
  distance = duration * 0.034/2;
//  Serial.print("s");
//  Serial.print(distance);
//  Serial.print("e");

  for(int i = 0; i < 2000; i++){
    count1++;
    read_adc = analogRead(A0);
    if(read_adc > 60){
      ret_val = (duration + (count1*delay_samp*0.001)) * (0.034/2);
 //     Serial.write("Condition true: ");
      Serial.print("s");
      Serial.print(ret_val);
      Serial.print("e");
    }
    delayMicroseconds(delay_samp);
  }
  delay(1);
}
