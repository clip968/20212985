int red = 7;

void setup()
{
    Serial.begin(115200);
	pinMode(red, OUTPUT);
    for (int i = 0;i <= 5;i++){
        digitalWrite(red, 1);
        delay(100);
        digitalWrite(red, 0);
        delay(100);
    }
}

void loop()
{
    while(1) {
        digitalWrite(red, 1);
    }    
}
