#include "mbed.h"
#include <iostream>

//------------------------------------
// Hyperterminal configuration
// 9600 bauds, 8-bit data, no parity
//------------------------------------

Timer enc_t,dec_t;

Serial pc(SERIAL_TX, SERIAL_RX);

#define DWT_CONTROL ((volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT  ((volatile uint32_t *)0xE0001004)
#define SCB_DEMCR   ((volatile uint32_t *)0xE000EDFC)

static inline uint32_t getDWT_CYCCNT(void)
{
    return *DWT_CYCCNT;
}

static inline void resetDWT_CYCCNT(void)
{
    *DWT_CYCCNT = 0; // reset the counter
}

static inline void enableDWT_CYCCNT(void)
{
    *SCB_DEMCR = *SCB_DEMCR | 0x01000000; // TRCENA = 1 Enable DWT
    *DWT_CONTROL = *DWT_CONTROL | 1 ; // enable the counter (CYCCNTENA = 1)
}

static const uint8_t SBox[16] = {
    0x0c, 0x00, 0x0f, 0x0a,
    0x02, 0x0b, 0x09, 0x05,
    0x08, 0x03, 0x0d, 0x07,
    0x01, 0x0e, 0x06, 0x04
};
                    
static const uint8_t PI[16] = {
    0x05, 0x00, 0x01, 0x04,
    0x07, 0x0c, 0x03, 0x08,
    0x0d, 0x06, 0x09, 0x02,
    0x0f, 0x0a, 0x0b, 0x0e
};                    

static const uint8_t Constant[35] = {
    0x01, 0x02, 0x04, 0x08,
    0x10, 0x20, 0x03, 0x06,
    0x0c, 0x18, 0x30, 0x23,
    0x05, 0x0a, 0x14, 0x28,
    0x13, 0x26, 0x0f, 0x1e,
    0x3c, 0x3b, 0x35, 0x29,
    0x11, 0x22, 0x07, 0x0e,
    0x1c, 0x38, 0x33, 0x25,
    0x09, 0x12, 0x24
};
void KeySchedule(const uint16_t *key, uint8_t output[36][8])
{
    int i;
    uint8_t RoundKey[80/4],temp, temp1, temp2, temp3;
    for(i=0;i<(80/4);i++)
        RoundKey[i] = (key[(i/4)]>>(4*(i&0x03))) & 0x0F;

    for(i=0;i<35;i++)
    {
        output[i][0] = RoundKey[1];output[i][1] = RoundKey[3];
        output[i][2] = RoundKey[4];output[i][3] = RoundKey[6];
        output[i][4] = RoundKey[13];output[i][5] = RoundKey[14];
        output[i][6] = RoundKey[15];output[i][7] = RoundKey[16];
           
        RoundKey[1]=RoundKey[1] ^SBox[RoundKey[0]];
        RoundKey[4]=RoundKey[4] ^SBox[RoundKey[16]];
        RoundKey[7]=RoundKey[7] ^(Constant[i]>>3);
        RoundKey[19]=RoundKey[19] ^(Constant[i]&0x07);         
        temp=RoundKey[0];
        RoundKey[0]=RoundKey[1];RoundKey[1]=RoundKey[2];      
        RoundKey[2]=RoundKey[3];RoundKey[3]=temp;
                              
        temp=RoundKey[0];temp1=RoundKey[1];
        temp2=RoundKey[2];temp3=RoundKey[3];
          
        RoundKey[0]=RoundKey[4];RoundKey[1]=RoundKey[5];
        RoundKey[2]=RoundKey[6];RoundKey[3]=RoundKey[7];
        RoundKey[4]=RoundKey[8];RoundKey[5]=RoundKey[9];
        RoundKey[6]=RoundKey[10];RoundKey[7]=RoundKey[11];
        RoundKey[8]=RoundKey[12];RoundKey[9]=RoundKey[13];
        RoundKey[10]=RoundKey[14];RoundKey[11]=RoundKey[15];
        RoundKey[12]=RoundKey[16];RoundKey[13]=RoundKey[17];
        RoundKey[14]=RoundKey[18];RoundKey[15]=RoundKey[19];
        RoundKey[16]=temp;RoundKey[17]=temp1;
        RoundKey[18]=temp2;RoundKey[19]=temp3;
    }
    output[35][0] = RoundKey[1];output[35][1] = RoundKey[3];
    output[35][2] = RoundKey[4];output[35][3] = RoundKey[6];
    output[35][4] = RoundKey[13];output[35][5] = RoundKey[14];
    output[35][6] = RoundKey[15];output[35][7] = RoundKey[16];
}     

void OneRound(uint8_t x[16], uint8_t k[8])
{
    uint8_t t[16];
    uint8_t i;
    for(i=0;i<8;i++)
        x[2*i+1]=SBox[x[2*i]^k[i]]^x[2*i+1] & 0x0F;

    for(i=0;i<16;i++)
        t[PI[i]]=x[i];
  
    for(i=0;i<16;i++)
        x[i]=t[i];
}

void Encrypt(uint8_t x[16], uint8_t SubKey[36][8])
{
    uint8_t i;
    // 35 rounds
    for(i=0;i<35;i++)
        OneRound(x,SubKey[i]);
    // last round
    for(i=0;i<8;i++)
        x[2*i+1]=SBox[x[2*i]^SubKey[35][i]]^x[2*i+1] & 0x0F;
}

int main()
{
    uint8_t Plain[16], SubKey[36][8];
    uint16_t KEY[(80/4)];
    uint8_t i;
    KEY[0] = 0x1100;KEY[1] = 0x3322;KEY[2] = 0x5544;KEY[3] = 0x7766;KEY[4] = 0x9988;
    for(i=0;i<16;i++)
        Plain[i] = i;

    //Print out the MCU Processor Speed
    pc.printf("CPU SystemCoreClock is %.2f MHz\r\n", (float)SystemCoreClock/1000.0f/1000.0f);
    //Print out the Plain Text
    pc.printf("PLAIN TEXT: ");
    for(i=0;i<16;i++) 
        pc.printf("%x",Plain[i]); 
    pc.printf("\n"); 
    
    //Encryption Process
    //To store the Total Clock Cycle
    uint32_t count1,count2;   
    enableDWT_CYCCNT();
    resetDWT_CYCCNT();
    enc_t.start();
    count1 = getDWT_CYCCNT();
    
    //for (int temp=1;temp<=64;temp++)
    {
        KeySchedule(KEY, SubKey);
        Encrypt(Plain,SubKey);
    }
    
    count2 = getDWT_CYCCNT();
    enc_t.stop();
    
    int BusyClockCycle = count2-count1;
    pc.printf("Time taken to Encrypt was: %.3f msec\n", enc_t.read()*1000);
    pc.printf("Total number of clock cylces used: %d\n",BusyClockCycle);
    pc.printf("CIPHER TEXT:\t");
    for(i=0;i<16;i=i++)  
        printf("%x",Plain[i]); 

    wait_ms(2000); // 1 second
    return 0;  
} 
