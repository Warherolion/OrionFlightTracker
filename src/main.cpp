#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);

  /*Basic startup, just init all sensors, radio and GPS, stager Radio till gps lock 
    main loop, gps and output to radio 
    Launch detection starts saving data to sd card from imu baro 
    Check apogee detection 

    append to sd card at that point
    
  
  */

}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}