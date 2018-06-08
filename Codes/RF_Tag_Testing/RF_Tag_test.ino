byte message[] = {0x02, 0x00}; //request packet
byte recv[18];
int count =0;

void setup() {
  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);
  pinMode(5,INPUT);
  Serial.begin(115200);
  Serial.write(message, sizeof(message));
 }

// the loop function runs over and over again forever
void loop() 
{ 
  if(digitalRead(5)==HIGH) //if button is pressed
  {
    Serial.write(message, sizeof(message)); //send request
    count=0;
    digitalWrite(13,HIGH); //indicate using led
    delay(400);
    digitalWrite(13,LOW);//indicate end of transmission using led
    }
  if(Serial.available()) //read when buffer full
  {
    recv[count]=Serial.read(); //store in array
    count++;
    if(count==17)
    {
      Serial.write(recv, sizeof(recv)); //send complete response
      digitalWrite(13,HIGH);
      delay(4000);
      digitalWrite(13,LOW);
    }
  }
  
  
}
