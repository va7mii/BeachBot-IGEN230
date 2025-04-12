#include <WiFi.h>
#include <ESP32Servo.h>
#include "esp_task_wdt.h"




const char *ssid = "hydrogen-laptop";
const char *password = "baymax-lovers";



// Static IP configuration
IPAddress local_IP(192, 168, 137, 73);     // desired static IP
IPAddress gateway(192, 168, 1, 1);       // usually your router
IPAddress subnet(255, 255, 255, 0);      // typical for home networks
IPAddress primaryDNS(8, 8, 8, 8);        // optional
IPAddress secondaryDNS(8, 8, 4, 4); 

WiFiServer server(23);  // Port 23 (like Telnet)
WiFiClient client;

String msg = "";
unsigned long lastCharTime = 0;
const unsigned long msgTimeout = 1000;



// Time stuff

// MOTOR STUFF
#define MIN 180
#define MAX 200
//motor stuff
int motor1In1 = 33;
int motor1In2 = 25;
int motor1EnA = 32;
int motor1In3 = 26; // Right motor Direction 1
int motor1In4 = 27; // Right motor Direction 2
int motor1EnB = 14; // Right motor PWM speed control


int motor2In1 = 16; // Second L298N IN1
int motor2In2 = 4; // Second L298N IN2
int motor2EnA = 22;
int motor2In3 = 18; // Second Right motor Direction 1
int motor2In4 = 19; // Second Right motor Direction 2
int motor2EnB = 23; // Second Right motor PWM speed control


// Setting PWM properties
const int freq = 30000;
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int pwmChannel3 = 2;
const int pwmChannel4 = 3;
const int resolution = 8;
int dutyCycle = MIN;


//servo stuff
Servo sweeper; // servo name


int servoPin = 13; // connect the pwm pin on esp 32
int shut = 200; // degree to rotate to
int wide = 0; // intial position = open
int sweep = 140;
int speedPerDegree = 160/80;

void setup() {
  
  
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Configure static IP
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("Failed to configure static IP");
  }

  WiFi.begin(ssid, password);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");
  Serial.println(WiFi.localIP());

  server.begin();
  Serial.println("Server started");

  
  // Configure motor pins
  pinMode(motor1In1, OUTPUT);
  pinMode(motor1In2, OUTPUT);
  pinMode(motor1EnA, OUTPUT);
  pinMode(motor1In3, OUTPUT);
  pinMode(motor1In4, OUTPUT);
  pinMode(motor1EnB, OUTPUT);
 
  pinMode(motor2In1, OUTPUT);
  pinMode(motor2In2, OUTPUT);
  pinMode(motor2EnA, OUTPUT);
  pinMode(motor2In3, OUTPUT);
  pinMode(motor2In4, OUTPUT);
  pinMode(motor2EnB, OUTPUT);

  // configure LEDC PWM
  ledcAttachChannel(motor1EnA, freq, resolution, pwmChannel1);
  ledcAttachChannel(motor1EnB, freq, resolution, pwmChannel2);
  ledcAttachChannel(motor2EnA, freq, resolution, pwmChannel3);
  ledcAttachChannel(motor2EnB, freq, resolution, pwmChannel4);
  


  //sweeper.write(100);

  //delay(2000);

  // //Close
  // for (int a = 100; a >= 50; a--) {
  //   sweeper.write(a);
  //   delay(80);
  // }

  // Move servo to desired position
  Serial.println("Let the servo begin moving!");
  sweeper.attach(servoPin); // attach servo to pwm pin
  // //sweeper.write(wide); // let servo move to initial closed position (0 degrees);
  // delay(90*speedPerDegree); // allow servo to move

  


}

int counter = 0;
int object_sum = 0;
unsigned int startTime;




void loop() { 

  
  
  
  if (!client || !client.connected()) {
    client = server.available();
    if (client) {
      Serial.println("Client connected.");
      msg = "";
    }
  }
  

  if (client && client.connected()) {
    while (client.available()) {

      char c = client.read();
      lastCharTime = millis();

      if (c == '\n') {
        msg.trim();
        double num_objects = msg.toDouble();

        Serial.print("Parsed: ");
        Serial.println(msg);

        if (num_objects >= 0.5) {
          // sweeper.write(100);
          // delay(speedPerDegree*100);
        
          Serial.println("warn!");
          controlMotor(0,0,0,0);
          sweeping();

        } else {
          // sweeper.write(0);
          // delay(speedPerDegree*100);
          Serial.println("nothing detected!");
          controlMotor(-170,-170,-170,-170);
          delay(1500);
          controlMotor(0,0,0,0);
          delay(1500);
        }

        

        msg = "";
      } else {
        msg += c;
        if (msg.length() > 200) {
          Serial.println("Too long, clearing");
          msg = "";
        }
      }

      sweeping();

    }

    // Drop stuck message if timeout
    if (msg.length() > 0 && millis() - lastCharTime > msgTimeout) {
      Serial.println("Timeout - clearing");
      msg = "";
    }


  }

  delay(1);  // Feed watchdog
}


void sweeping(){
  for (int pos=0;pos <= 120; pos += 1) { // goes from 90 degrees to 180 degrees
      // in steps of 1 degree
      sweeper.write(pos);              // tell servo to go to position in variable 'pos'
      delay(4*speedPerDegree);                       // waits 15ms for the servo to reach the position
    }

    delay(1000);
    
    for (int pos = 120; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
      sweeper.write(pos);              // tell servo to go to position in variable 'pos'
      delay(4*speedPerDegree);                       // waits 15ms for the servo to reach the position
    }


    Serial.println("May the sweep begin anew");
    delay(2000); // only for testing
}
// Each parameter controls speed of each motor (i.e. duty cycle)
void controlMotor(int motor_UL, int motor_UR, int motor_DL, int motor_DR) {
  
  //Motor_UL
  if (motor_UL < 0) { //Check if backwards (negative)
    ledcWrite(motor1EnA, -1*motor_UL);
    digitalWrite(motor1In1, HIGH);
    digitalWrite(motor1In2, LOW);
  } else if (motor_UL == 0) {  //Check if stopped (0)
    ledcWrite(motor1EnA, 0);
    digitalWrite(motor1In1, LOW);
    digitalWrite(motor1In2, LOW);
  } else { // Then, motor is moving forward (postive)
    ledcWrite(motor1EnA, motor_UL);
    digitalWrite(motor1In1, LOW);
    digitalWrite(motor1In2, HIGH);
  }

  //Motor_UR
  if (motor_UR < 0) { //Check if backwards (negative)
    ledcWrite(motor1EnB, -1*motor_UR);
    digitalWrite(motor1In3, HIGH);
    digitalWrite(motor1In4, LOW);
  } else if (motor_UR == 0) {  //Check if stopped (0)
    ledcWrite(motor1EnB, 0);
    digitalWrite(motor1In3, LOW);
    digitalWrite(motor1In4, LOW);
  } else { // Then, motor is moving forward (postive)
    ledcWrite(motor1EnB, motor_UR);
    digitalWrite(motor1In3, LOW);
    digitalWrite(motor1In4, HIGH);
  }

  //Motor_DL
  if (motor_DL < 0) { //Check if backwards (negative)
    ledcWrite(motor2EnA, -1*motor_DL);
    digitalWrite(motor2In1, HIGH);
    digitalWrite(motor2In2, LOW);
  } else if (motor_DL == 0) {  //Check if stopped (0)
    ledcWrite(motor2EnA, 0);
    digitalWrite(motor2In1, LOW);
    digitalWrite(motor2In2, LOW);
  } else { // Then, motor is moving forward (postive)
    ledcWrite(motor2EnA, motor_DL);
    digitalWrite(motor2In1, LOW);
    digitalWrite(motor2In2, HIGH);
  }

  //Motor_DR
  if (motor_DR < 0) { //Check if backwards (negative)
    ledcWrite(motor2EnB, -1*motor_DR);
    digitalWrite(motor2In3, HIGH);
    digitalWrite(motor2In4, LOW);
  } else if (motor_DR == 0) {  //Check if stopped (0)
    ledcWrite(motor2EnB, 0);
    digitalWrite(motor2In3, LOW);
    digitalWrite(motor2In4, LOW);
  } else { // Then, motor is moving forward (postive)
    ledcWrite(motor2EnB, motor_DR);
    digitalWrite(motor2In3, LOW);
    digitalWrite(motor2In4, HIGH);
  }


}


