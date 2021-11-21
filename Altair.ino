#include <SoftwareSerial.h>
void setup()  
{
  Serial.begin(9600);
}

void loop() 
{
  if (Serial.available())
  {
    char dato=Serial.read();
    Serial.print("Dato recibido: ");
    Serial.println(dato);
  }
}
  
