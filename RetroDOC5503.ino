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

#define rFreqLow        0x00    // to 0x1F
#define rFreqHigh       0x20    // to 0x3F
#define rVolume         0x40    // to 0x5F
#define rDatasample     0x60    // to 0x7F
#define rAddressPtr     0x80    // to 0x9F
#define rControl        0xA0    // to 0xBF       [CA3, CA2, CA1, CA0, IE, M2, M1,  H]
#define rResTableSize   0xC0    // to 0xDF       [ NU,  BS,  T2,  T1, T0, R2, R1, R0]
#define rOIR            0xE0    // Osc Interrupt [ IR,   1,  o4,  o3, o2, o1, o0,  1]
#define rOER            0xE1    // Osc Enable    [  1,   1,  e4,  e3, e2, e1, e0,  1]
#define rADC            0xE2    // A/D Converter

byte PROGRAM[][100] = {
   {'W', rFreqLow        ,      0xF0},
   {'W', rFreqLow  + 0x01,      0xF0},
   {'W', rFreqHigh,             0x0F},
   {'W', rFreqHigh + 0x01,      0x0F},
   {'R', rFreqLow,              0xF0},  
   {'R', rFreqLow  + 0x01,       0xF0},  
   {'R', rFreqHigh,             0x0F},  
   {'R', rFreqHigh  + 0x01,      0x0F}, 
   {'D', 0x08},                         // NOP for 8 cycles
   {'d'},                               // NOP
   {'r', rOIR},                         //  
   {'r', rOER},                         // 
   {'r', rADC},                         //
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

// THIS IS GPIOD
#define MEGA_PL7  (5)   // DOC A15/D7
#define MEGA_PL6  (21)  // DOC A14/D6
#define MEGA_PL5  (20)  // DOC A13/D5
#define MEGA_PL4  (6)   // DOC A12/D4
#define MEGA_PL3  (8)   // DOC A11/D3
#define MEGA_PL2  (7)   // DOC A10/D2
#define MEGA_PL1  (14)  // DOC  A9/D1
#define MEGA_PL0  (2)   // DOC  A8/D0

// This is GPIOA - I swapped it when I connected it. NEED TO REVERSE IN SW
#define MEGA_PA7  (12)  // DOC A0 GPIOC, Teensy Pin 12 - bit7
#define MEGA_PA6  (11)  // DOC A1 ..
#define MEGA_PA5  (25)  // DOC A2
#define MEGA_PA4  (10)  // DOC A3
#define MEGA_PA3  (9)   // DOC A4
#define MEGA_PA2  (23)  // DOC A5
#define MEGA_PA1  (22)  // DOC A6 ..
#define MEGA_PA0  (15)  // DOC A7 GPIOC, Teensy Pin 15 - bit 0

// Digital Pin Assignments 

// Set ADDR_L as input
#define ADDR_L_dir_in1()        ((byte) (GPIOA_PDDR & ~0b0000000000100000))
#define ADDR_L_dir_in2()        ((byte) (GPIOC_PDDR & ~0b0000000011011111))
// Set ADDR_L as output
#define ADDR_L_dir_out1()       ((byte) (GPIOA_PDDR | 0b0000000000100000))
#define ADDR_L_dir_out2()       ((byte) (GPIOC_PDDR | 0b0000000011011111))

#define ADDR_L_dir_in()        { ADDR_L_dir_in1();  ADDR_L_dir_in2()}
#define ADDR_L_dir_out()       { ADDR_L_dir_out1(); ADDR_L_dir_out2()}

// READ A0..A7 - DOC is driving 
#define inADDR_L_RAW1        ((byte) (GPIOA_PDIR & 0b0000000000100000))
#define inADDR_L_RAW2        ((byte) (GPIOC_PDIR & 0b0000000011011111))
// build inADDR_L
#define inADDR_L             (inADDR_L_RAW1 | inADDR_L_RAW2)

// WRITE A0..A7 - CPU is driving
#define outADDR_L_RAW1(ADDR_low)     ((byte) ( (GPIOA_PDOR & ~(0b0000000000100000)) | (0b0000000000100000 & ADDR_low) ))
#define outADDR_L_RAW2(ADDR_low)     ((byte) ( (GPIOC_PDOR & ~(0b0000000011011111)) | (0b0000000011011111 & ADDR_low) ))
// build outADD_L
#define outADDR_L(ADDR_low)          { outADDR_L_RAW1(ADDR_low); outADDR_L_RAW2(ADDR_low);}


#define SET_DATA_OUT(D)   (GPIOD_PDOR = (GPIOD_PDOR & 0xFFFFFF00) | (D)) // same for RetroDOC
#define outADDR_H(D)      SET_DATA_OUT(D)                                 // A15..A8 - normally 0 for WRITE
#define xDATA_IN          ((byte) (GPIOD_PDIR & 0xFF))                    // for RetroDOC, these are also A15..A8
#define inADDR_H          xDATA_IN
//#define inADDR            (word)(inADDR_H << 8) | inADDR_L) // ONLY IF YOU FIGURE OUT A WAY TO REVERSE inADDR_L with #defines)

/* OLD VERSION - REMOVE
#define xADDR_H_IN        xDATA_IN
// same for RetroDOC
#define ADDR_L_RAW        ((word) (GPIOC_PDIR & 0b0000111111011111))    // how to I change direction?
#define ADDR_L            ((byte) (ADDR_L_RAW & 0x00FF))
#define xADDR_L_IN     
*/   

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

unsigned long clock_cycle_count=0;

word DOC_ADDR;
byte DOC_DATA;

// To be used with ADD LOW
byte reverse(byte b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

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
volatile byte ADDRESS_IN_LOW;
volatile byte ADDRESS_IN_HIGH;

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
int nop_counter = 0;

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
                        clock_cycle_count++;
                        if(nop_counter > 0) {
                                Serial.printf("CPU (cycle %d): NOP\n", clock_cycle_count);
                                nop_counter--;
                                }
                                else
                        switch(PROGRAM[0][PC]) {
                                case 'd':       // ***  NOP  ***
                                        Serial.printf("CPU (cycle %d): NOP\n", clock_cycle_count);
                                break;
                                case 'D': {      // *** NOP x CYCLES ***
                                        Serial.printf("CPU (cycle %d): NOP\n", clock_cycle_count);
                                        nop_counter = (uint8_t)PROGRAM[1][PC];
                                        if(nop_counter == 0) nop_counter=0;
                                }
                                case 'W': {     // *** WRITE ***
                                        Serial.printf("CPU (cycle %d): WRITE DOC register=%x, data=%x\n", clock_cycle_count, PROGRAM[1][PC], PROGRAM[2][PC] );
                                        xCPU_DIR_WRITE();
                                        outADDR_L(PROGRAM[1][PC]);       //ADDR_L = reverse(PROGRAM[1][PC];
                                        delay(2);
                                        digitalWrite(DOC_RW_N, 0);       // WRITE to DOC Reg
                                        digitalWrite(DOC_CS_N, 0);       // assert CS_
                                        SET_DATA_OUT(PROGRAM[2][PC]); 
                                }
                                break;
                                case 'R': {     // *** READ VERIFY ***
                                        Serial.printf("CPU (cycle %d):   READ DOC register=%x, EXPECTED data=%x, ", clock_cycle_count, PROGRAM[1][PC], PROGRAM[2][PC] );
                                        xCPU_DIR_READ();
                                        digitalWrite(DOC_RW_N, 1); // READ from DOC
                                        outADDR_L(PROGRAM[1][PC]);       //ADDR_L = reverse(PROGRAM[1][PC];
                                        delay(2);
                                        digitalWrite(DOC_CS_N, 0);       // assert CS_
                                }
                                break;
                                case 'r': {     // *** READ  ***
                                        Serial.printf("CPU (cycle %d):   READ DOC register=%x, ", clock_cycle_count, PROGRAM[1][PC] );
                                        xCPU_DIR_READ();
                                        digitalWrite(DOC_RW_N, 1); // READ  DOC register
                                        outADDR_L(PROGRAM[1][PC]);       //ADDR_L = reverse(PROGRAM[1][PC];
                                        delay(2);
                                        digitalWrite(DOC_CS_N, 0);       // assert CS_
                                }
                                case 'E':       // *** END PROGRAM ***
                                        cpu_halt = true; // Stop CPU
                                break;
                                default: {      // *** INVALID COMMAND ****
                                        Serial.printf("COMMAND NOT RECOGNIZED: CPU HALT\n");
                                        cpu_halt = true; // Stop CPU
                                }
                        } // switch
                } // posedge E

    if( (m_E==1) & (E==0) ) {
        // finish managing reads
        switch(PROGRAM[0][PC]) {
                case 'R': {     // *** complete READ VERIFY ***
                        DATA_IN= xDATA_IN;
                       Serial.printf("READ %x\n", DATA_IN);
                       if (PROGRAM[2][PC] != DATA_IN) {        // Check expected data
                                    Serial.printf("CPU ERROR ON CPU READ - CPU WILL HALT\n");
                                    cpu_halt = true;        // Stop CPU on error
                                    }
                }
                case 'r': {     // *** complete READ  ***
                        DATA_IN= xDATA_IN;
                        Serial.printf("READ %x\n", DATA_IN);
                }
        }
        PC++;                           // update Program Counter
        digitalWrite(DOC_CS_N, 1);      // de-assert CS_
        digitalWrite(DOC_RW_N, 1);       // de-assert RW_ if was write

        xDOC_BUS();
        Serial.printf("Negedge E detected: time for the DOC to work...\n");
        while(digitalRead(DOC_RAS_N) == 1);  // at the negedge of RAS_ we read the address lines
        delay(2);       // let signals settle
        ADDRESS_IN_LOW  = inADDR_L;
        ADDRESS_IN_HIGH = inADDR_H;
        byte cstrb_n = digitalRead(DOC_CSTRB_N);
        byte CA      = digitalRead(DOC_CA0);
        CA           = (digitalRead(DOC_CA1) << 1) | CA;
        CA           = (digitalRead(DOC_CA2) << 2) | CA;
        CA           = (digitalRead(DOC_CA3) << 3) | CA;
        byte irq_n   = digitalRead(DOC_IRQ_N);
        byte bankSel = digitalRead(DOC_BS);
        Serial.printf("DOC: Bank %d: Address %x%x\n", bankSel, ADDRESS_IN_HIGH, ADDRESS_IN_LOW);
        Serial.printf("DOC: {CA3, CA2, CA1, CA0 } = {%x} CSTRB = %x\n", CA, cstrb_n);
        Serial.printf("DOC: IRQ_ is %s\n", irq_n ? "HIGH" : "LOW");
    } // negedge E

    m_E = E; // Do this here

    if (j-- == 0)
    {
      Serial.flush();
      j = 500;
    }
  }
}
