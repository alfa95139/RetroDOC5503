////////////////////////////////////////////////////////////////////
// RetroShield 6809 for Teensy 3.5
// RetroDOC 5503
//
// Copyright (C) 2021 Erturk Kocalar, Alessandro Fasan
//  


////////////////////////////////////////////////////////////////////
// Options
//   outputDEBUG: Print memory access debugging messages.
////////////////////////////////////////////////////////////////////
#define outputDEBUG     0


byte PROGRAM[][5] = {
   {'W', 0x10, 0xAA},         // Write register 0x10 with value 0xAA
   {'D', 0x08},               // NOP for 8 cycles
   {'R', 0x10, 0xA1},         // READ register 0x10, expect value 0xAA -> should flag that expected data is different (show received data)
   {'d'},                     // NOP
   {'r', 0x10},               // READ register 0x10, no expected value  
   {'E'}          // END of program
};

/*
D is delay in terms of cpu cycles.

In loop(), you can read a program row and set global flags so cpu_tick
can execute them.
For read/write, cpu_tick() will assert CS and drive/read the databus.
Nop is do nothing for cpu_tick(), i.e. CS not asserted, while the delay
decrements at each iteration of loop().
*/

////////////////////////////////////////////////////////////////////
// Arduino Port Mapping to Teensy GPIO Numbers.
////////////////////////////////////////////////////////////////////
#define MEGA_PD7  (24)  // DOC RES_
#define MEGA_PG2  (17)  // DOC BS
#define MEGA_PG1  (16)  // DOC RW_
#define MEGA_PG0  (13)  // DOC CS_
#define MEGA_PB3  (30)  // DOC IRQ
#define MEGA_PB2  (29)  // DOC  Q
#define MEGA_PB1  (39)  // DOC  CLK
#define MEGA_PB0  (28)  // DOC  E

#define MEGA_PC7  (27)  // DOC RAS_
#define MEGA_PC6  (26)  // DOC CAS_
#define MEGA_PC5  (4)   // DOC CSTRB_
#define MEGA_PC4  (3)   // DOC CA0
#define MEGA_PC3  (38)  // DOC CA1
#define MEGA_PC2  (37)  // DOC CA2
#define MEGA_PC1  (36)  // DOC CA3
#define MEGA_PC0  (35)  // GND

#define MEGA_PL7  (5)   // DOC A15/D7
#define MEGA_PL6  (21)  // DOC A14/D6
#define MEGA_PL5  (20)  // DOC A13/D5
#define MEGA_PL4  (6)   // DOC A12/D4
#define MEGA_PL3  (8)   // DOC A11/D3
#define MEGA_PL2  (7)   // DOC A10/D2
#define MEGA_PL1  (14)  // DOC  A9/D1
#define MEGA_PL0  (2)   // DOC  A8/D0

#define MEGA_PA7  (12)  // DOC A0
#define MEGA_PA6  (11)  // DOC A1
#define MEGA_PA5  (25)  // DOC A2
#define MEGA_PA4  (10)  // DOC A3
#define MEGA_PA3  (9)   // DOC A4
#define MEGA_PA2  (23)  // DOC A5
#define MEGA_PA1  (22)  // DOC A6
#define MEGA_PA0  (15)  // DOC A7

/* Digital Pin Assignments 

You can later optimize this by accessing the GPIOX_PDDR register directly.
this is what we do for data bus.0=input, 1=output.

#define xDATA_DIR_IN()    (GPIOD_PDDR = (GPIOD_PDDR & 0xFFFFFF00))       // same for RetroDOC
#define xDATA_DIR_OUT()   (GPIOD_PDDR = (GPIOD_PDDR | 0x000000FF))       // same for RetroDOC

So, modify above for xADDR_DIR_IN() and xADDR_DIR_OUT() using GPIOA_PDDR and the bitfields here.

//#define ADDR_H_RAW      ((word) (GPIOA_PDIR & 0b1111000000100000))    // how do I change direction?
#define ADDR_L_RAW        ((word) (GPIOC_PDIR & 0b0000111111011111))    // how to I change direction?

// read bits raw
#define xDATA_DIR_IN()    (GPIOD_PDDR = (GPIOD_PDDR & 0xFFFFFF00))       // same for RetroDOC
#define xDATA_DIR_OUT()   (GPIOD_PDDR = (GPIOD_PDDR | 0x000000FF))       // same for RetroDOC
#define SET_DATA_OUT(D)   (GPIOD_PDOR = (GPIOD_PDOR & 0xFFFFFF00) | (D)) // same for RetroDOC
#define xDATA_IN          ((byte) (GPIOD_PDIR & 0xFF))                   // same for RetroDOC

// Teensy has an LED on its digital pin13 (PTC5). which interferes w/
// level shifters.  So we instead pick-up A5 from PTA5 port and use
// PTC5 for PG0 purposes.
//
//#define ADDR_H_RAW        ((word) (GPIOA_PDIR & 0b1111000000100000))    // how do I change direction?
#define ADDR_L_RAW        ((word) (GPIOC_PDIR & 0b0000111111011111))    // how to I change direction?
// build ADDR, ADDR_H, ADDR_L
//#define ADDR              ((word) (ADDR_H_RAW | ADDR_L_RAW))
#define ADDR_in           ((word) (xDATA_DIR_IN() << 8 | ADDR_L_RAW))
//#define ADDR_H            ((byte) ((ADDR & 0xFF00) >> 8))
#define ADDR_L            ((byte) (ADDR & 0x00FF))

*/

#define SET_DATA_OUT(D)   (GPIOD_PDOR = (GPIOD_PDOR & 0xFFFFFF00) | (D)) // same for RetroDOC
#define xDATA_IN          ((byte) (GPIOD_PDIR & 0xFF))    
// same for RetroDOC
//#define ADDR_L_RAW        ((word) (GPIOC_PDIR & 0b0000111111011111))    // how to I change direction?
//#define ADDR_L            ((byte) (ADDR & 0x00FF))

#define DOC_RESET_N  MEGA_PD7    // RST_ output to RetroDOC
#define DOC_RW_N     MEGA_PG1    // RW_  output to RetroDOC
#define DOC_CS_N     MEGA_PG0    // CS_  output to RetroDOC
#define DOC_BS       MEGA_PG2    // BS   input, Bank Select
#define DOC_IRQ_N    MEGA_PB3    // IRQ_ input 
#define DOC_Q        MEGA_PB2    // Q    input
#define DOC_CLK      MEGA_PB1    // OUTPUT this is CLK for RetroDOC
#define DOC_E        MEGA_PB0    // E    input

#define DOC_RAS_N    MEGA_PC7  
#define DOC_CAS_N    MEGA_PC6 
#define DOC_CSTRB_N  MEGA_PC5  
#define DOC_CA0      MEGA_PC4  
#define DOC_CA1      MEGA_PC3  
#define DOC_CA2      MEGA_PC2  
#define DOC_CA3      MEGA_PC1  
#define DOC_GND      MEGA_PC0 
// Fast routines to drive signals high/low; faster than digitalWrite
//
#define CLK_HIGH          (GPIOA_PSOR = 0x20000)
#define CLK_LOW           (GPIOA_PCOR = 0x20000)
#define STATE_RW_N          ((byte) (GPIOB_PDIR & 0x01) ) // same for RetroDOC

unsigned long clock_cycle_count;

word DOC_ADDR;
byte DOC_DATA;



//  Set direction for CPU BUS, READ operation
void xCPU_DIR_READ()
{
byte pinTable[16] = {
    5,21,20,6,8,7,14,2,      // D7..D0
    12,11,25,10,9,23,22,15}; // A7..A0

for (int i=0; i<8; i++)
    {
      pinMode(pinTable[i],INPUT);
      pinMode(pinTable[i+8],OUTPUT);
    }
}

// Set direction for CPU BUS, WRITE operation
void xCPU_DIR_WRITE()
{
byte pinTable[16] = {
    5,21,20,6,8,7,14,2,      // D7..D0
    12,11,25,10,9,23,22,15}; // A7..A0

for (int i=0; i<8; i++)
    {
      pinMode(pinTable[i],OUTPUT);
      pinMode(pinTable[i+8],OUTPUT);
    }
}

// Set direction for DOC BUS
void xDOC_BUS()
{
byte pinTable[16] = {
    5,21,20,6,8,7,14,2,      // D7..D0
    12,11,25,10,9,23,22,15}; // A7..A0

for (int i=0; i<8; i++)
    {
      pinMode(pinTable[i],INPUT);
      pinMode(pinTable[i+8],INPUT);
    }
}


void DOC_init()
{
// Set directions for ADDR & DATA Bus.
xCPU_DIR_WRITE();

// The following are fixed

// Input to CPU (Teensy) - Output from DOC
pinMode(DOC_RAS_N,INPUT);
pinMode(DOC_CAS_N,INPUT);
pinMode(DOC_CSTRB_N,INPUT);
pinMode(DOC_CA0,INPUT);
pinMode(DOC_CA1,INPUT);
pinMode(DOC_CA2,INPUT);
pinMode(DOC_CA3,INPUT);
pinMode(DOC_GND,INPUT);
pinMode(DOC_IRQ_N, INPUT);
pinMode(DOC_Q, INPUT);
pinMode(DOC_BS, INPUT);
pinMode(DOC_E, INPUT);

// Outputs of CPU (Teensy) - Input to DOC
pinMode(DOC_CLK, OUTPUT);
pinMode(DOC_RESET_N, OUTPUT);
pinMode(DOC_RW_N, OUTPUT);
pinMode(DOC_CS_N, OUTPUT);

DOC_assert_reset();

clock_cycle_count = 0;
}

///
void DOC_assert_reset()
{
  // Drive RESET conditions
  digitalWrite(DOC_RESET_N, LOW);

}

///
void DOC_release_reset()
{
  // Drive RESET conditions
  digitalWrite(DOC_RESET_N, HIGH);
}

////////////////////////////////////////////////////////////////////
// Processor Control Loop
////////////////////////////////////////////////////////////////////

volatile byte DATA_OUT;
volatile byte DATA_IN;

volatile byte ADDRESS_OUT;
volatile byte ADDRESS_IN;

int clk;

// doc_tick()
inline __attribute__((always_inline))
void doc_tick()
{
if (clk) {
       CLK_HIGH;
       clk = 0;
        }
   else {
       CLK_LOW;
       clk = 1;        
        }
delay(100);
}

////////////////////////////////////////////////////////////////////
// Setup
////////////////////////////////////////////////////////////////////

void setup() 
{
  Serial.begin(115200);
  while (!Serial);


  Serial.println("\n\n");
  Serial.println("***********************************");
  Serial.println("* RetroShield: DOC 5503 (ENSONIQ) *");
  Serial.println("*       (c) 2021 ALFASoniQ        *");
  Serial.println("***********************************");


  DOC_init();
  
  // Reset DOC
  Serial.println("Retroshield: Resetting DOC 5503...  ");
  DOC_assert_reset();
  for(int i=0;i<25;i++) doc_tick();  
  DOC_release_reset();
  Serial.println("Reset DOC 5503 completed.\n");
}

////////////////////////////////////////////////////////////////////
// Loop
////////////////////////////////////////////////////////////////////
int E, m_E = 0; 
int PC = 0;
bool cpu_halt = false;

void loop()
{
word j = 0;

// Loop forever
//  

while(1)
        {
        doc_tick();

        E = digitalRead(DOC_E);
        if (!cpu_halt)
                if( (m_E ==0) & (E==1) ) {
                        Serial.printf("Posedege E detected: Time for the CPU to work here...\n");
                        digitalWrite(DOC_CS_N, 0); // assert CS_
                        switch(PROGRAM[0][PC]) {
                                case 'd':       // ***  NOP  ***
                                        Serial.printf("CPU (cycle %d): NOP\n", PC);
                                break;
                                case 'D': {      // *** NOP x CYCLES ***
                                        Serial.printf("CPU (cycle %d): NOP\n", PC);
                                }
                                case 'W': {     // *** WRITE ***
                                        Serial.printf("CPU (cycle %d): WRITE DOC register=%x, data=%x\n", PC, PROGRAM[1][PC], PROGRAM[2][PC] );
                                        xCPU_DIR_WRITE();
                                        SET_DATA_OUT(PROGRAM[2][PC]); 
                                        digitalWrite(DOC_RW_N, 0); // WRITE to DOC Reg
                                        delay(2);
                                }
                                break;
                                case 'R': {     // *** READ VERIFY ***
                                        Serial.printf("CPU (cycle %d):   READ DOC register=%x, EXPECTED data=%x, ", PC, PROGRAM[1][PC], PROGRAM[2][PC] );
                                        xCPU_DIR_READ();
                                        digitalWrite(DOC_RW_N, 1); // READ from DOC
                                        // set address here   <<< HOW
                                        delay(2);
                                        while(DOC_RAS_N == 1);  // wait until DOC asserts RAS (active low)
                                       
                                        DATA_IN= xDATA_IN;
                                        Serial.printf("READ %x\n", DATA_IN);
                                        if (PROGRAM[2][PC] != DATA_IN) {        // Check expected data
                                                Serial.printf("CPU ERROR ON CPU READ - CPU WILL HALT\n");
                                                cpu_halt = true;        // Stop CPU on error
                                                }
                                }
                                break;
                                case 'e': {     // *** READ  ***
                                        Serial.printf("CPU (cycle %d):   READ DOC register=%x, ", PC, PROGRAM[1][PC] );
                                        xCPU_DIR_READ();
                                        digitalWrite(DOC_RW_N, 1); // READ from DOC
                                        // set address here    <<< HOW
                                        delay(2);
                                        while(DOC_RAS_N == 1);  // wait until DOC asserts RAS (active low)
                                        DATA_IN= xDATA_IN;
                                        Serial.printf("READ %x\n", DATA_IN);
                                }
                                case 'E':       // *** END PROGRAM ***
                                        cpu_halt = true; // Stop CPU
                                break;
                                default: {      // *** INVALID COMMAND ****
                                        Serial.printf("COMMAND NOT RECOGNIZED: CPU HALT\n");
                                        cpu_halt = true; // Stop CPU
                                }
                        } // switch

                PC++;                           // update Program Counter
                delay(2);                       // short delay
                digitalWrite(DOC_CS_N, 1);      // de-assert CS_
                
                } // posedge E

    if( (m_E==1) & (E==0) ) {
        xDOC_BUS();
        Serial.printf("Negedge E detected: time for the DOC to work...\n");
        // negedege E - here the DOC drives
        // Which means: A15 to A0 are input (output from DOC)
        // check for ADDRESS, BS, CA3,CA2,CA1,CA0, CSTRB,  and IRQ here
    } // negedge E

    m_E = E; // Do this here

    if (j-- == 0)
    {
      Serial.flush();
      j = 500;
    }
  }
}
