// Test for Arduino to G2 Motor Controller Serial Interface
#define VOLTAGE 23
#define TEMPERATURE 24

#define RXD2 16
#define TXD2 17

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Serial2.begin(19200, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Serial Txd is on pin: " + String(TX));
  Serial.println("Serial Rxd is on pin: " + String(RX));
}

void loop() {
  // put your main code here, to run repeatedly:digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second

  Serial.println("Reading temp...");
  int temp = getVariable(TEMPERATURE);
  float tempC = ((float)temp) / 10;
  float tempF = (tempC * 1.8) + 32;
  Serial.print("Temperature: ");
  Serial.print(tempC, 1);
  Serial.println("° C");
  Serial.print("Temperature: ");
  Serial.print(tempF, 1);
  Serial.println("° F");

  Serial.println("Reading Volts...");
  int volts = getVariable(VOLTAGE);
  Serial.println(volts);
  float voltsfloat = ((float)volts) / 1000;
  Serial.print("Input Voltage: ");
  Serial.print(voltsfloat, 3);
  Serial.println("V");

}

unsigned int getVariable(unsigned char variableID)
{
  Serial2.write(0xA1);
  Serial2.write(variableID);
  byte byte1 = readByte();
  byte byte2 = readByte();
  return byte1 + 256 * byte2;
}

int readByte()
{
  char c;
  int availablebytes = Serial2.readBytes(&c, 1);
  if (availablebytes == 0) {
    return -1;
  }
  return (byte)c;
}
