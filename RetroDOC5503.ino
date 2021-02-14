////////////////////////////////////////////////////////////////////
// RetroShield 6809 for Teensy 3.5
// RetroDOC 5503
//
// Copyright (C) 2021 Erturk Kocalar, 8Bitforce.com


////////////////////////////////////////////////////////////////////
// Options
//   outputDEBUG: Print memory access debugging messages.
////////////////////////////////////////////////////////////////////
#define outputDEBUG     0


byte PROGRAM[][3] = {
   {'W', 0x10, 0xAA},         // Write register 0x10 with value 0xAA
   {'D', 0x08, 0x00},        // wait 08 clock cycles
   {'R', 0x10, 0xAA},        // READ register 0x10, expect value 0xAA
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

/* Digital Pin Assignments */

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

void DOC_init()
{
  // Set directions for ADDR & DATA Bus.
  
  byte pinTable[16] = {
    5,21,20,6,8,7,14,2,      // D7..D0
    12,11,25,10,9,23,22,15}; //  // A7..A0

  
  for (int i=0; i<16; i++)
  {
    pinMode(pinTable[i],INPUT);
  } 

  /*   THIS WILL FLIP THE DIRECTION OF THE BUS
  for (int i=0; i<16; i++)
  {
    pinMode(pinTable[i],OUTPUT);
  } 
  */

pinMode(DOC_RAS_N,INPUT);
pinMode(DOC_CAS_N,INPUT);
pinMode(DOC_CSTRB_N,INPUT);
pinMode(DOC_CA0,INPUT);
pinMode(DOC_CA1,INPUT);
pinMode(DOC_CA2,INPUT);
pinMode(DOC_CA3,INPUT);
pinMode(DOC_GND,INPUT);
pinMode(DOC_RESET_N, OUTPUT);
pinMode(DOC_RW_N, OUTPUT);
pinMode(DOC_CS_N, OUTPUT);
pinMode(DOC_BS, INPUT);
pinMode(DOC_IRQ_N, INPUT);
pinMode(DOC_Q, INPUT);
pinMode(DOC_CLK, OUTPUT);
pinMode(DOC_E, INPUT);

  DOC_assert_reset();

  clock_cycle_count = 0;
}

////

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

void loop()
{
  word j = 0;

  // Loop forever
  //  
  DOC_ADDR = 0;
  while(1)
  {
    doc_tick();

    E = digitalRead(DOC_E);
    if( (m_E ==0) & (E==1) ) {
        Serial.printf("Posedege E detected: Time for the CPU to work here...\n");
        //posedge E - here the CPU drives
        // put WRITE REGS/READ REGS here

        // Update address
        DOC_ADDR = DOC_ADDR++; // low 8 bits only
        xDATA_DIR_IN();
        // assert CS_
        digitalWrite(DOC_CS_N, 0);
        // *** READ *** 
        digitalWrite(DOC_RW_N, 1); // READ

        while(DOC_RAS_N == 1);
        DATA_IN= xDATA_IN;
        Serial.printf("Reg %x, reading %x\n", DATA_IN);
        // *** WRITE ***
        // xDATA_DIR_OUT();
        // digitalWrite(DOC_RW_N, 0); // WRITE to DOC Reg
        // de-assert CS_
        // digitalWrite(DOC_CS_N, 1);

    }

    if( (m_E==1) & (E==0) ) {
        Serial.printf("Negedge E detected: time for the DOC to work...\n");
        //
        // negedege E - here the DOC drives
        // check for ADDRESS and IRQ here
    }

    m_E = E; // Do this here

    if (j-- == 0)
    {
      Serial.flush();
      j = 500;
    }
  }
}
