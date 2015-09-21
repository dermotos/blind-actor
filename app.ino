
#define IDENTIFIER "bedroom-blinds"

SYSTEM_MODE(SEMI_AUTOMATIC);

// This #include statement was automatically added by the Spark IDE.
#include "OneButton.h"
#include "DynamicCommandParser.h"

/*
Blinds connected to D4 and D5
Button zero connected to D1 (to GND)
Button one connected to D2 (to GND)


*/

//Constants
const unsigned long OPEN_TIME = 70000;
const unsigned long CLOSE_TIME = 60000;

const int BLIND_PIN_A = D4;
const int BLIND_PIN_B = D5;

const int BUTTON_0_PIN = D1;
const int BUTTON_1_PIN = D2;

const int INDICATOR_LED = D7; // D7 pin is connected to a small LED on the board. We will use it to indicate the blind motors are active

const int OPENING = 1;
const int CLOSING = -1;
const int STOPPED = 0;



//State variables
bool blindsOpen = true; // Stores the final state of the blinds when the motors stop.
int motorState = STOPPED; //Stores the current state of the blind motors.
unsigned long motorStopTime = 0; //Used to schedule a time in the future when the blind motors turn off.



//Used by heartbeat()
unsigned long deviceHeartbeatInterval = 1000 * 10; //Checkin with central control every 10 seconds
unsigned long nextDeviceHeartbeat = 0;

unsigned long centralHeartbeatInterval = 60 * 1000; //Expect a heartbeat from central control every 60 seconds (usually less).
unsigned long centralHeartbeatLastReceived = 0; //Keep track of the last time we received a heartbeat.

int connectionRetries = 0; //Keeps track of the number of reconnection tries. If it exceeds connectionRetryLimit, the system is reset.
const int connectionRetryLimit = 5;
//Two physical buttons are connected to the device, configure them here:

OneButton button0(BUTTON_0_PIN,INPUT_PULLUP);
OneButton button1(BUTTON_1_PIN,INPUT_PULLUP);


DynamicCommandParser commandParser('^','$',',');

// Network connectivity
TCPClient client;
byte server[] = { 10,0,0,100 };
int port = 9998;



void setup() {
    // Setup the pins connected to the blind motors as outputs
    pinMode(BLIND_PIN_A,OUTPUT);
    pinMode(BLIND_PIN_B,OUTPUT);
    pinMode(INDICATOR_LED,OUTPUT);

    // Connect the event handlers for the buttons
    button0.attachClick(button0Pressed);
    button1.attachClick(button1Pressed);


    commandParser.addParser("heartbeat",centralHeartbeat);
    commandParser.addParser("reset",reset);
    commandParser.addParser("connect-to-cloud",connectToCloud);
    commandParser.addParser("control-blinds",controlBlinds);

    if(WiFi.ready() == false){
        WiFi.connect();
    }

    while(WiFi.ready() == false){
        delay(50);
    }
    Serial.begin(9600);
    delay(2000);
    heartbeat();

    RGB.control(true);
    RGB.brightness(0);
}


void sendMessage(String message){
    /*Serial.print("Sending message '");
    Serial.print(message);
    Serial.println("'");*/

    //Ensure we don't have a half open socket. Check if we've received a heartbeat from central recently
    if(centralHeartbeatLastReceived + centralHeartbeatInterval < millis()){
      //We haven't received a heartbeat. Close the connection here, so it's restablished below.
      client.stop();
      centralHeartbeatLastReceived = millis(); //Prevents an infinite loop of disconnects.
    }

     while(!client.connected()){
        client.stop();
        delay(700);
        Serial.println("Reconnecting...");
        client.connect(server,port);

        if(!client.connected()){
            //Connection attempt timed out
            connectionRetries++;
            if(connectionRetries >= connectionRetryLimit){
                //Hmmm, either central control is down, or the spark core is acting up, try restarting the CC3000 (WiFi chip)
                RGB.control(false);

                WiFi.disconnect();
                WiFi.off();
                delay(500);
                WiFi.on();
                delay(500);
                WiFi.connect();
                while(!WiFi.ready()){
                  delay(100);
                }

                RGB.control(true);
                RGB.brightness(0);

            }
        }
        else{
            //Connection successful
            connectionRetries = 0;
        }
    }

    client.print(IDENTIFIER);
    client.print(":");
    client.println(message);
}

//Checks if data is available from network receives it into a buffer.
void receiveData(){
    if(client.connected() && client.available()){
        char c = client.read();
        Serial.print(c);
        commandParser.appendChar(c);
    }
}





void heartbeat(){

    if(nextDeviceHeartbeat < millis()){
        Serial.println("Sending heartbeat in...");
        sendMessage("heartbeat");
        nextDeviceHeartbeat = millis() + deviceHeartbeatInterval;
    }

}




// Sets the blind motors to a new state (opening, closing or stopped)
void setMotorState(int newState)
{
    switch(newState)
    {
        case OPENING:
        {
            // Set blinds to opening. Set a timer for them to stop after enough time has passed for them to open
            digitalWrite(BLIND_PIN_A, HIGH);
            digitalWrite(BLIND_PIN_B, LOW);
            motorStopTime = millis() + OPEN_TIME;
            motorState = OPENING;
            blindsOpen = true; //Optimistically set the blind state.
            digitalWrite(INDICATOR_LED,HIGH); //Turn on the indicator led
        }
        break;

        case CLOSING:
        {
            // Set the blinds to closing. Set a timer for them to stop after enough time has passed for them to close
            digitalWrite(BLIND_PIN_A, LOW);
            digitalWrite(BLIND_PIN_B, HIGH);
            motorStopTime = millis() + CLOSE_TIME;
            motorState = CLOSING;
            blindsOpen = false; //Optimistically set the blind state.
            digitalWrite(INDICATOR_LED,HIGH); //Turn on the indicator led
        }
        break;

        case STOPPED:
        {
            // Stop the blinds where they currently are. Disable the timer, in case its set.
            digitalWrite(BLIND_PIN_A, LOW);
            digitalWrite(BLIND_PIN_B, LOW);
            motorStopTime = 0;
            motorState = STOPPED;
            digitalWrite(INDICATOR_LED,LOW); // turn off the indicator led
            //blinds open will remain at its current value.
        }
        break;
    }
}

// Called regularly to check if the blind motors stop time has arrived (if it is set)
void checkMotorStopTime(){
    if(motorStopTime > 0 && motorStopTime < millis())
    {
        // The stop time has been set, and the time is in the past, so stop the motors
        setMotorState(STOPPED);
    }
}

// Button 0 pressed. Toggle between blinds opening, closing or stopped.
void button0Pressed(){
    Serial.println("Button pressed");
    digitalWrite(D7,HIGH);
    if(motorState != STOPPED)
    {
        //Blinds are currently opening or closing, pressing the button should halt them at their current position.
        setMotorState(STOPPED);
    }
    else
    {
        setMotorState(CLOSING);
    }
}


void button1Pressed(){
  if(motorState != STOPPED)
  {
      //Blinds are currently opening or closing, pressing the button should halt them at their current position.
      setMotorState(STOPPED);
  }
  else
  {
      setMotorState(OPENING);
  }
}





void reset(char **values, int valueCount){
  System.reset();
}

void connectToCloud(char **values, int valueCount){
  Spark.connect();
}

void controlBlinds(char **values, int valueCount){
  /*
  Index 0 is the name of the command (in this case, 'control-blinds')
  Index 1 is the command, eg: open, close, stop
  */
  Serial.print("Processing command:");
  Serial.println(values[1]);



  if(strcmp(values[1], "open")  == 0){
    Serial.print("Opening blinds...");
    setMotorState(OPENING);
  }
  else if(strcmp(values[1], "close")  == 0){
    Serial.print("Closing blinds...");
    setMotorState(CLOSING);
  }
  else if(strcmp(values[1], "stop")  == 0){
    Serial.print("Stopping blinds...");
    setMotorState(STOPPED);
  }
  else if(strcmp(values[1], "toggle")  == 0){
    Serial.print("Toggling blind state...");
    //Similar to original controls. Pressing toggles between opening, stopped and closing
    if(motorState != STOPPED)
    {
        //Blinds are currently opening or closing, pressing the button should halt them at their current position.
        setMotorState(STOPPED);
    }
    else
    {
        if(blindsOpen)
        {
            // Blinds are currently open, set them to close
            setMotorState(CLOSING);
        }
        else
        {
            // Blinds are currently closed, set them to open
            setMotorState(OPENING);
        }
    }
  }
  else if(values[1] == "report-state"){
    //Report the state of the blinds back to central control
    if(blindsOpen){
        sendMessage("state-blinds-open");
    }
    else{
        sendMessage("state-blinds-closed");
    }
  }

}

void centralHeartbeat(char **values, int valueCount){
  // A heartbeat command from central was received. If one hasn't been received within 60 seconds, the device attempts to re-establish
  // TCP connection on next outbound communication attempt.
  centralHeartbeatLastReceived = millis();
}

void loop() {
    // This method loops forever...
    checkMotorStopTime();
    button0.tick();
    button1.tick();

    heartbeat();
    receiveData();

    delay(10); //Buttons get all weird if this delay isn't here


}
