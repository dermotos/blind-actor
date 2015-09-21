#define OneButton_h

#include "application.h"


// ----- Callback function types -----

extern "C" {
  typedef void (*callbackFunction)(void);
}


class OneButton
{
public:
  // ----- Constructor -----
  OneButton(int pin, PinMode inputType);

  // ----- Set runtime parameters -----

  // set # millisec after single click is assumed.
  void setClickTicks(int ticks);

  // set # millisec after press is assumed.
  void setPressTicks(int ticks);

  // attach functions that will be called when button was pressed in th especified way.
  void attachClick(callbackFunction newFunction);
  void attachDoubleClick(callbackFunction newFunction);
  void attachPress(callbackFunction newFunction);

  // ----- State machine functions -----

  // call this function every some milliseconds for handling button events.
  void tick(void);

private:
  int _pin;        // hardware pin number.
  int _clickTicks; // number of ticks that have to pass by before a click is detected
  int _pressTicks; // number of ticks that have to pass by before a lonn button press is detected

  int _buttonReleased;
  int _buttonPressed;


  // These variables will hold functions acting as event source.
  callbackFunction _clickFunc;
  callbackFunction _doubleClickFunc;
  callbackFunction _pressFunc;

  // These variables that hold information across the upcomming tick calls.
  // They are initialized once on program start and are updated every time the tick function is called.
  int _state;
  unsigned long _startTime; // will be set in state 1
};
