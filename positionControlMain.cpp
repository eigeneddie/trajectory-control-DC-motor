/* Position control Program
@author: Edgar Sutawika


Notes: 20180805
1. Trajectory program successful!
2. Need to test for heel strike period update. 
Notes: 20180810
1. Research sensors: masukin recommendation if constrained by time
2. 
*/

// ==LIBRARY==
#include "mbed.h"
#include "motor.h"
#include "encoderMotor.h" //@adopted from QEI authored by: Aaron Berk.
#include "PID.h"


Serial pc(USBTX,USBRX, 230400);
DigitalOut led(PA_4);
      //====================================//
      //===========Program Setup============//
      //====================================//

// 1. ===============Global Variable===================
const double samplingTime = 0.005; //[s]
const double leadOfLeadscrew = 8; //[mm/rev]
    
// 2. ================Define pin======================
//  NOTE: 
//  motor (pin PWM right, pin PWM left, pin enable);
//  encoder (pin channel A, pin channel B, total PPR, encoding type (choose X4));
//  PPR Total = PPR Encoder * Mode Encoding * Gear Reduction 103.6

// a. Motor pins
motor motor1(PA_5, PA_6, PC_8);

// b. Encoder pins
encoderMotor enc1(PB_2, PB_12, 1036 , encoderMotor::X4_ENCODING); //103.6 (internal encoder)
//encoderMotor enc1(PB_14, PB_15, 14400 , encoderMotor::X4_ENCODING); //103.6 (external encoder)

// c. Heel strike pine
//attaching external interrupt to pin
//InterruptIn heelStrike(PB_13); 

// 3. =============Obligatory function================//
void positionControlPID(void);                                  // Closed-loop Program
double nutPosition (float seconds1, float T1 );                 // Trajectory function, updated by the attached-to-interrupt function. 
double coeffMultiply(double polyOrder, double period);          // Coefficient multiplier (for pacing)
double powerFunction(double basis, double power);               // to calcualte exponents
    

// 4. =============PID Parameter Assignment===========//
PID pid1( 0.5926365365738419,  19.352250803617786,  0.0124121991504719082, 300.571735406622, samplingTime); // Coba coba

// 5. ===========Local Variables ===========//
    
//<INITIAL STATE OF THE PROSTHETIC CONTROLS>
// a. Control purposes
double targetRate = 0;               // [from -1 to 1]
double currentPosition = 0;      // [mm] measured from encoder.
double setpointPosition;         // [mm] set from targetNutPosition();
double currentRevs = 0;          // [revolution] measured to calculate currentPosition.
double previousRevs = 0; 

// b. Mischellaneous
float heelPeriod = 1.25;            // [ms] time in between heel strikes. Initially at rest


//c. TAMBAHAN BULAN DESEMBER
int afterTesting = 1;
const int arraySize = 2250; // 1500 2400; 4800;
int arrayCount = 0;
double savedReading[arraySize];
double setpointReading[arraySize];
double timeCount[arraySize];
void samplingReading(void);

//</INITIAL STATE OF THE PROSTHETIC CONTROLS>

// 6. =====Time related utilities======//
    
Ticker posConPID;       // position control PID attach to ticker
Ticker sampleData;
Timer t;                // set timer
int oldTime;            
int counterInt = 0;
int countCycle =0;



//====================================//
//===========Main program=============//
//====================================//

int main() {
    led = 0;

    wait(3); 
          
                      
    //  heelStrike.fall(&updateHeelToHeelPeriod); 
    // serialPrint.attach(&shortSerialPrint, 0.20); //samplingTime
    sampleData.attach(&samplingReading, 0.00833);
    posConPID.attach(&positionControlPID, samplingTime);
    // </SETUP>
    t.start();
    led = 1;
    while(afterTesting == 1) {
          if (t.read()>heelPeriod){
          t.reset();
          countCycle++;
          }

          // </LOOPING>
          if (countCycle >= 15){
              
              motor1.setpwm(0);
              led = 0;
              posConPID.detach();
              sampleData.detach();
              afterTesting = 0;
          }
    }
      
    for (int u = 0 ; u < arraySize; u++){
        pc.printf("%.5f, %.5f, %.5f", -setpointReading[u], -savedReading[u], timeCount[u]); 
        pc.printf("\n");
        }
    // End of program
}
  //====================================//
  //============Functions===============//
  //====================================//

// I. <CONTROL PROGRAM>
void positionControlPID (void){
 //   if (heelPeriod <= maximumPeriod && heelPeriod >= minimumPeriod) {
    // 1. Measuring current position
        //enc1.disableInterrupts();  // Disable interrupts
        currentRevs = enc1.getRevolutions() + previousRevs; // in revolutions
        //enc1.enableInterrupts();  // Enable interrupts
        
        previousRevs = currentRevs;
        // Correct encoder unit
        currentPosition = currentRevs * leadOfLeadscrew ; //  [mm]
        // Calculate motor input pwm
        setpointPosition = nutPosition( t.read(), heelPeriod); //input float
        targetRate = pid1.createpwm(setpointPosition, currentPosition);
      
        // Set motor pwm
        motor1.setpwm(targetRate);
}

// II. <IMPORTANT FUNCTIONS>
// 1. Trajectory // ok
double nutPosition (float seconds1, float T1 ) { // millisecond = current time, T = period in between heel strike
        
        double seconds = seconds1;
        double T = T1; // Operating polynomial equation: Default T = 1.25 seconds (Default frequency 0.8 Hz)
       
        double y1;
        double y2;
        double y3;
        double y4;
        double y5, y6, y7;
   
    
        y1 = 188447.160408*coeffMultiply(12, T)*powerFunction(seconds, 12) - 1379605.690202*coeffMultiply(11, T)*powerFunction(seconds, 11) ;
        y2 = 4339908.790252*coeffMultiply(10, T)*powerFunction(seconds, 10) - 7675036.282053*coeffMultiply(9, T)*powerFunction(seconds, 9);
        y3 = 8393025.147604*coeffMultiply(8, T)*powerFunction(seconds, 8) - 5893232.388210*coeffMultiply(7, T)*powerFunction(seconds, 7);  
        y4 = 2677405.855402*coeffMultiply(6, T)*powerFunction(seconds, 6) - 772109.569471*coeffMultiply(5, T)*powerFunction(seconds, 5);
        y5 = 131706.621239*coeffMultiply(4, T)*powerFunction(seconds, 4) - 10647.949391*coeffMultiply(3, T)*powerFunction(seconds, 3);
        y6 = 147.615860*coeffMultiply(2, T)*powerFunction(seconds, 2) - 6.385159*coeffMultiply(1, T)*powerFunction(seconds, 1);
        y7 = -0.271777;
   
           /* if (seconds<T/2){
                y1 = 1666*coeffMultiply(5, T)*powerFunction(seconds, 5) - 3834*coeffMultiply(4, T)*powerFunction(seconds, 4);
                y2 = 2308*coeffMultiply(3, T)*powerFunction(seconds, 3) - 373*coeffMultiply(2, T)*powerFunction(seconds, 2);
                y3 = -5.43*coeffMultiply(1, T)*powerFunction(seconds, 1) - 0.1263*coeffMultiply(0, T) ;
                y4 = 0;
            } 
            else if (seconds>T/2){
                y1 = -133357.140*coeffMultiply(7, T)*powerFunction(seconds, 7) + 829202.250*coeffMultiply(6, T)*powerFunction(seconds, 6) ;
                y2 = -2163094.444*coeffMultiply(5, T)*powerFunction(seconds, 5) + 3062751.685*coeffMultiply(4, T)*powerFunction(seconds, 4);
                y3 = -2537401.339*coeffMultiply(3, T)*powerFunction(seconds, 3) + 1228131.978*coeffMultiply(2, T)*powerFunction(seconds, 2);  
                y4 = -321215.690*coeffMultiply(1, T)*powerFunction(seconds, 1) + 34985.378;
            }*/
            double y = (y1 + y2 + y3 + y4 + y5 + y6 + y7);
         
         /*  if (seconds>T){
                y = 0;
            }*/
            
            if (y> 10){  //<Kodingan jaga2>
                y = 10;
            }
            else if (y <-35){
                y = -35;
            }            //</Kodingan jaga2>
    
       // y = y*0.68;
        return -y; // [mm]
}

// 3. Function Coefficient Multiplier //ok
double coeffMultiply(double polyOrder, double period) {
    double B = 1/powerFunction(0.8*period, polyOrder);
    return B;
}
// 4. Calculating Exponents //ok
double powerFunction (double basis, double power) {
    double result = 1;
        for (int i = 0; i < power; i++  ){
            result = result*basis;
        }
    return result;
}

// 5. sampling reading routine
void samplingReading(void){
    
    setpointReading[arrayCount] = setpointPosition;
    savedReading[arrayCount] = currentPosition;
    timeCount[arrayCount] = t.read();
    arrayCount = arrayCount+1; 
}
