
#define IDENTIFIER "bedroom-blinds"

SYSTEM_MODE(SEMI_AUTOMATIC);

// This #include statement was automatically added by the Spark IDE.
#include "OneButton.h"
#include "DynamicCommandParser.h"
#include "HX711.h"
/*
Blinds connected to D4 and D5
Button zero connected to D1 (to GND)
Button one connected to D2 (to GND)


*/

//Constants


const unsigned long OPEN_TIME = 70000;
const unsigned long CLOSE_TIME = 60000;

const int BLIND_0_PIN_0 = D3;
const int BLIND_0_PIN_1 = D4;

const int BLIND_1_PIN_0 = D5;
const int BLIND_1_PIN_1 = D6;

const int LEFT_OCCUPANCY_CLOCK_PIN = A0;
const int LEFT_OCCUPANCY_DATA_PIN = A1;

const int RIGHT_OCCUPANCY_CLOCK_PIN = A2;
const int RIGHT_OCCUPANCY_DATA_PIN = A3;

#define calibration_factor_left -23100.0 //This value is obtained using the SparkFun_HX711_Calibration sketch
#define calibration_factor_right -18400.0 //This value is obtained using the SparkFun_HX711_Calibration sketch

HX711 leftScale(LEFT_OCCUPANCY_DATA_PIN, LEFT_OCCUPANCY_CLOCK_PIN);
HX711 rightScale(RIGHT_OCCUPANCY_DATA_PIN, RIGHT_OCCUPANCY_CLOCK_PIN);


const int OPENING = 1;
const int CLOSING = -1;
const int STOPPED = 0;

// State variables
bool blindOpen[] = {true,true};                 // Stores the final state of the blinds when the motors stop.
int motorState[] = {STOPPED,STOPPED};   // Stores the current state of the blind motors.
unsigned long motorStopTime[] = {0,0};  // Used to schedule a time in the near future when the blind motors turn off.


//Used by heartbeat()
unsigned long deviceHeartbeatInterval = 1000 * 10; //Checkin with central control every 10 seconds
unsigned long nextDeviceHeartbeat = 0;

unsigned long centralHeartbeatInterval = 60 * 1000; //Expect a heartbeat from central control every 60 seconds (usually less).
unsigned long centralHeartbeatLastReceived = 0; //Keep track of the last time we received a heartbeat.

unsigned long scaleReadingInterval = 1000 * 2;  // Send weight reading every 2 seconds
unsigned long scaleReadingLastSent = 0;         // Keep track of the last time we sent the reading.

int connectionRetries = 0; //Keeps track of the number of reconnection tries. If it exceeds connectionRetryLimit, the system is reset.
const int connectionRetryLimit = 5;


DynamicCommandParser commandParser('^','$',',');

// Network connectivity
TCPClient client;
byte server[] = { 10,0,0,100 };
int port = 9998;



void setup() {
    // Setup the pins connected to the blind motors as outputs
    pinMode(BLIND_0_PIN_0,OUTPUT);
    pinMode(BLIND_0_PIN_1,OUTPUT);
    pinMode(BLIND_1_PIN_0,OUTPUT);
    pinMode(BLIND_1_PIN_1,OUTPUT);

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

    //Setup the scales
    leftScale.set_scale(calibration_factor_left);
    rightScale.set_scale(calibration_factor_right);

    // This could cause issues if the devices resets while there's weight on the bed
    // If errors occur, remove weight from the bed and reset the device
    // If it becomes an issue, get a hard value for the offset and hard code it.
    leftScale.tare();
    rightScale.tare();

    RGB.control(true);
    RGB.brightness(0);
}

void checkForHalfOpenConnection(){
  //Ensure we don't have a half open socket. Check if we've received a heartbeat from central recently
  if(centralHeartbeatLastReceived + centralHeartbeatInterval < millis()){
    //We haven't received a heartbeat. Close the connection here, so it's restablished below.
    client.stop();
    centralHeartbeatLastReceived = millis(); //Prevents an infinite loop of disconnects.
  }
}

void ensureConnectivity(){
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
}

void sendMessage(String message){

    checkForHalfOpenConnection();
    ensureConnectivity();

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

// Sends raw weight readings to central control
void sendScaleReadings(){
  Serial.println("Sending readings");
  if(scaleReadingLastSent + scaleReadingInterval < millis()){

    float leftReading = leftScale.get_units();
    float rightReading = rightScale.get_units();

    checkForHalfOpenConnection();
    ensureConnectivity();

    client.print(IDENTIFIER);
    client.print(":");
    client.print("scales-");
    client.print(leftReading);
    client.print("|");
    client.println(rightReading);

    scaleReadingLastSent = millis(); //Prevents an infinite loop of disconnects.
  }


}

// Sets a blind motor to a new state (opening, closing or stopped)
void setMotorState(int blindIndex, int newState)
{
    switch(newState)
    {
        case OPENING:
        {
            // Set blind to opening. Set a timer for them to stop after enough time has passed for them to open
            if(blindIndex == 0){
              digitalWrite(BLIND_0_PIN_0, LOW);
              digitalWrite(BLIND_0_PIN_1, HIGH);
            }else{
              digitalWrite(BLIND_1_PIN_0, LOW);
              digitalWrite(BLIND_1_PIN_1, HIGH);
            }

            motorStopTime[blindIndex] = millis() + OPEN_TIME;
            motorState[blindIndex] = OPENING;
            blindOpen[blindIndex] = true; //Optimistically set the blind state.
        }
        break;

        case CLOSING:
        {
            // Set the blinds to closing. Set a timer for them to stop after enough time has passed for them to close
            if(blindIndex == 0){
              digitalWrite(BLIND_0_PIN_0, HIGH);
              digitalWrite(BLIND_0_PIN_1, LOW);
            }else{
              digitalWrite(BLIND_1_PIN_0, HIGH);
              digitalWrite(BLIND_1_PIN_1, LOW);
            }
            motorStopTime[blindIndex] = millis() + CLOSE_TIME;
            motorState[blindIndex] = CLOSING;
            blindOpen[blindIndex] = false; //Optimistically set the blind state.
        }
        break;

        case STOPPED:
        {
            // Stop the blinds where they currently are. Disable the timer, in case its set.
            if(blindIndex == 0){
              digitalWrite(BLIND_0_PIN_0, LOW);
              digitalWrite(BLIND_0_PIN_1, LOW);
            }else{
              digitalWrite(BLIND_1_PIN_0, LOW);
              digitalWrite(BLIND_1_PIN_1, LOW);
            }
            motorStopTime[blindIndex] = 0;
            motorState[blindIndex] = STOPPED;

            //blinds open will remain at its current value.
        }
        break;
    }
}

// Called regularly to check if the blind motors stop time has arrived (if it is set)
void checkMotorStopTime(){
    if(motorStopTime[0] > 0 && motorStopTime[0] < millis())
    {
        setMotorState(0,STOPPED); // The stop time has been set, and the time is in the past, so stop the motors
    }

    if(motorStopTime[1] > 0 && motorStopTime[1] < millis())
    {
        setMotorState(1,STOPPED); // The stop time has been set, and the time is in the past, so stop the motors
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
  Index 1 is the blind index (0 or 1)
  Index 2 is the command, eg: open, close, stop
  */
  Serial.print("Processing command:");
  Serial.println(values[1]);
  int targetBlindIndex = (strcmp(values[1],"0") == 0) ? 0 : 1;

  if(strcmp(values[2], "open")  == 0){
    Serial.print("Opening blinds...");
    setMotorState(targetBlindIndex,OPENING);
  }
  else if(strcmp(values[2], "close")  == 0){
    Serial.print("Closing blinds...");
    setMotorState(targetBlindIndex,CLOSING);
  }
  else if(strcmp(values[2], "stop")  == 0){
    Serial.print("Stopping blinds...");
    setMotorState(targetBlindIndex,STOPPED);
  }
  else if(strcmp(values[2], "toggle")  == 0){
    Serial.print("Toggling blind state...");
    //Similar to original controls. Pressing toggles between opening, stopped and closing.
    if(motorState[targetBlindIndex] != STOPPED)
    {
        //Blinds are currently opening or closing, pressing the button should halt them at their current position.
        setMotorState(targetBlindIndex, STOPPED);
    }
    else
    {
        if(blindOpen[targetBlindIndex])
        {
            // Blind is currently open, set it to close
            setMotorState(targetBlindIndex, CLOSING);
        }
        else
        {
            // Blinds are currently closed, set them to open
            setMotorState(targetBlindIndex, OPENING);
        }
    }
  }
  else if(strcmp(values[2], "toggle-sync")  == 0){
    Serial.print("Toggling blind state...");
    // Similar to original controls. Pressing toggles between opening, stopped and closing.
    // Toggle sync keeps both blinds in sync. The state is based on motor 0 (blind on the biggest window)
    if(motorState[0] != STOPPED)
    {
        //Blinds are currently opening or closing, pressing the button should halt them at their current position.
        setMotorState(0, STOPPED);
        setMotorState(1, STOPPED);
    }
    else
    {
        if(blindOpen[0])
        {
            // Blind is currently open, set it to close
            setMotorState(0, CLOSING);
            setMotorState(1, CLOSING);
        }
        else
        {
            // Blinds are currently closed, set them to open
            setMotorState(0, OPENING);
            setMotorState(1, OPENING);
        }
    }
  }
  else if(values[2] == "report-state"){
    //Report the state of the blind back to central control.
    if(blindOpen[0] || blindOpen[1]){
        sendMessage("state-blinds-open");   // At least one blind is open
    }
    else{
        sendMessage("state-blinds-closed"); // Both blinds are closed
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
    heartbeat();
    receiveData();
    sendScaleReadings();
    delay(5);
}
