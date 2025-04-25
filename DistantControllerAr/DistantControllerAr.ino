//www.elegoo.com
//2016.12.08
#include "SR04.h" //ultrasound sensor
#include "LedControl.h" //matrix led module
//ultrasould module
  #define TRIG_PIN 12
  #define ECHO_PIN 11
  SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);

//rotory encoder:
  const int PinCLK=2;   // Generating interrupts using CLK signal
  const int PinDT=3;    // Reading DT signal
  const int PinSW=4;    // Reading Push Button switch
  //for rotory encoder:
  volatile boolean TurnDetected;  // need volatile for Interrupts
  volatile boolean rotationdirection;  // CW or CCW rotation
  int PrevPosition;
  int RotaryPosition = 0;

//led matrix:
  LedControl lc=LedControl(10,8,9,1);
  /* we always wait a bit between updates of the display */
  unsigned long delaytime1=500;
  unsigned long delaytime2=50;

  const uint64_t NUMBERS[] = { //all numbers for the led matrix
  0x1824424242422418,//zero
  0x2020202024283020,//one
  0x7c04081020404438,//two
  0x1c2020201c20201c,//three
  0x2020207c24283020,//four
  0x182420203c04043c,//five 
  0x182422261a020438,//six
  0x040404080810203e,//seven
  0x1824242418242418,//eight
  0x1820202038242438//nine
  };
  const int IMAGES_LEN = sizeof(NUMBERS)/8;

long a;

// Interrupt routine runs if CLK goes from HIGH to LOW for rotory encoder
void isr ()  {
  delay(4);  // delay for Debouncing
  if (digitalRead(PinCLK))
    rotationdirection= digitalRead(PinDT);
  else
    rotationdirection= !digitalRead(PinDT);
  TurnDetected = true;
}

void setup() {
  //rotory encoder:
    pinMode(PinCLK,INPUT);
    pinMode(PinDT,INPUT);  
    pinMode(PinSW,INPUT);
    digitalWrite(PinSW, HIGH); // Pull-Up resistor for switch
    attachInterrupt (0,isr,FALLING); // interrupt 0 always connected to pin 2 on Arduino UNO

  //matrix module:
      /*
    The MAX72XX is in power-saving mode on startup,
    we have to do a wakeup call
    */
    lc.shutdown(0,false);
    /* Set the brightness to a medium values */
    lc.setIntensity(0,8);
    /* and clear the display */
    lc.clearDisplay(0);

  Serial.begin(9600);
  delay(1000);
}

//number display on matrix leg
void displaymatrix(uint64_t number) {
  for (int i = 0; i < 8; i++) {
    byte row = (number >> i * 8) & 0xFF;
    for (int j = 0; j < 8; j++) {
      lc.setLed(0, i, j, bitRead(row, j));
    }
  }
}

void loop() {
  //rotory encoder:
  if (!(digitalRead(PinSW))) {   // check if button is pressed
    if (RotaryPosition == 0) {  // check if button was already pressed
    } else {
        RotaryPosition=0; // Reset position to ZERO
      }
  }

  if (TurnDetected)  {
    PrevPosition = RotaryPosition; // Save previous position in variable
    if(PrevPosition < 0){
      PrevPosition = 0;
      RotaryPosition = 0;
    }
    if(PrevPosition > 9){
      PrevPosition = 9;
      RotaryPosition = 9;
    }
    if (rotationdirection) {
      RotaryPosition=RotaryPosition-1;} // decrase Position by 1
    else {
      RotaryPosition=RotaryPosition+1;} // increase Position by 1

    TurnDetected = false;  // do NOT repeat IF loop until new rotation detected
  }
  displaymatrix(NUMBERS[PrevPosition]); //update matrixled
  a=sr04.Distance();
  Serial.print(a);
  Serial.println("cm");
  //matrixled:
  

}
