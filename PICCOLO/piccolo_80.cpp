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

#define GF_POLY       0x13 //irred. polynomial F_function = x^4+x+1 for F_16 = F_2[x]/(F_function)
int BLOCK_SIZE = 64,DEG_GF_POLY = 4;
int N80=10; //key size length in bytes
const int RN80=25; //number of rounds
int KEYSIZE80=80; //key size

typedef unsigned char BYTE; //8 bit
typedef unsigned long WORD; //32 bit

//key
typedef struct {
  WORD WhiteningKey[2]; // WhiteningKey: WhiteningKey[0] = w_0 | w_1, WhiteningKey[1] = w_2 | w_3
  WORD RoundKey[RN80]; // RoundKey: 1 RoundKey are encoded in a 32-bit word
} KEY;

//64-bit state
typedef struct {
  WORD b[2]; 
} STATE;

//Piccolo SBox
BYTE SBox[16] = 
{ 
    0xE, 0x4, 0xB, 0x2, 0x3, 0x8, 0x0, 0x9,
    0x1, 0xA, 0x7, 0xF, 0x6, 0xC, 0x5, 0xD
};        
//Diffusion matrix
const BYTE DiffusionMatrix[4][4] = 
{
    {0x2,0x3,0x1,0x1},
    {0x1,0x2,0x3,0x1},
    {0x1,0x1,0x2,0x3},
    {0x3,0x1,0x1,0x2},
};
        
//Constants for the KeySchedule
WORD Constant80[RN80] = 
{
    0x071c293d, 0x1f1a253e, 0x1718213f, 0x2f163d38, 0x27143939,
    0x3f12353a, 0x3710313b, 0x4f0e0d34, 0x470c0935, 0x5f0a0536,
    0x57080137, 0x6f061d30, 0x67041931, 0x7f021532, 0x77001133,
    0x8f3e6d2c, 0x873c692d, 0x9f3a652e, 0x9738612f, 0xaf367d28,
    0xa7347929, 0xbf32752a, 0xb730712b, 0xcf2e4d24, 0xc72c4925
};

//galois multiplication in F_16
BYTE gm(BYTE a, BYTE b) 
{
    BYTE res = 0;
    for (int i = 0; i < DEG_GF_POLY; i++) 
    {
        if ( (b & 0x1) == 1 ) 
            res ^= a;
        BYTE hbs = (a & 0x8);
        a <<= 0x1;
        if ( hbs == 0x8) 
            a ^= GF_POLY;
        b >>= 0x1;
    }
    return res;
}

void F_function(BYTE b[]) 
{
    //InitializeBuffers
    BYTE x[4] = {0,0,0,0};
    BYTE y[4] = {0,0,0,0};
    //Apply SBox
    x[0] = SBox[(b[0] >> 4) & 0xf];
    x[1] = SBox[(b[0] >> 0) & 0xf];
    x[2] = SBox[(b[1] >> 4) & 0xf];
    x[3] = SBox[(b[1] >> 0) & 0xf];
    //Multiply Matrix and Column */
    y[0] = (gm(x[0],DiffusionMatrix[0][0]) ^ gm(x[1],DiffusionMatrix[0][1]) ^ gm(x[2],DiffusionMatrix[0][2]) ^ gm(x[3],DiffusionMatrix[0][3]));
    y[1] = (gm(x[0],DiffusionMatrix[1][0]) ^ gm(x[1],DiffusionMatrix[1][1]) ^ gm(x[2],DiffusionMatrix[1][2]) ^ gm(x[3],DiffusionMatrix[1][3]));
    y[2] = (gm(x[0],DiffusionMatrix[2][0]) ^ gm(x[1],DiffusionMatrix[2][1]) ^ gm(x[2],DiffusionMatrix[2][2]) ^ gm(x[3],DiffusionMatrix[2][3]));
    y[3] = (gm(x[0],DiffusionMatrix[3][0]) ^ gm(x[1],DiffusionMatrix[3][1]) ^ gm(x[2],DiffusionMatrix[3][2]) ^ gm(x[3],DiffusionMatrix[3][3]));
    //Apply SBox again
    x[0] = SBox[y[0]];
    x[1] = SBox[y[1]];
    x[2] = SBox[y[2]];
    x[3] = SBox[y[3]];
    b[0] = (x[0] << 4) ^ x[1];
    b[1] = (x[2] << 4) ^ x[3];
}

void RoundPermutation(STATE *s) 
{
    WORD t[2] = {0,0}; //Store result of the RoundPermutation

  //split, permute, reassemble 64-bit state
    t[1] ^= ((s->b[0] >> 24) & 0xff) <<  8; // x_0 -> x_6
    t[0] ^= ((s->b[0] >> 16) & 0xff) <<  0; // x_1 -> x_3
    t[0] ^= ((s->b[0] >>  8) & 0xff) << 24; // x_2 -> x_0
    t[1] ^= ((s->b[0] >>  0) & 0xff) << 16; // x_3 -> x_5
    t[0] ^= ((s->b[1] >> 24) & 0xff) <<  8; // x_4 -> x_2
    t[1] ^= ((s->b[1] >> 16) & 0xff) <<  0; // x_5 -> x_7
    t[1] ^= ((s->b[1] >>  8) & 0xff) << 24; // x_6 -> x_4
    t[0] ^= ((s->b[1] >>  0) & 0xff) << 16; // x_7 -> x_1

    s->b[0] = t[0]; s->b[1] = t[1];
}

void KeySchedule(BYTE InitialKey[], KEY *k, WORD C[], int N, int NoRound, int KEYSIZE) 
{
    //InitialKey is the UserKey: 80-bit: k_0 = InitialKey[0] | InitialKey[1],...,k_4 = InitialKey[ 8] | InitialKey[ 9]
    
    //InitializeWhiteningKey
    k->WhiteningKey[0] = 0x0;
    k->WhiteningKey[1] = 0x0;
    //Initialize RoundKeys
    for (int i = 0; i < NoRound; i++) 
        k->RoundKey[i] = 0x0;

    //Expand WhiteningKey and RoundKey from the UserKey
    //WhiteningKey
    k->WhiteningKey[0] ^= (InitialKey[0] << 24);
    k->WhiteningKey[0] ^= (InitialKey[3] << 16);
    k->WhiteningKey[0] ^= (InitialKey[2] <<  8);
    k->WhiteningKey[0] ^= (InitialKey[1] <<  0);
   
    k->WhiteningKey[1] ^= (InitialKey[8] << 24);
    k->WhiteningKey[1] ^= (InitialKey[7] << 16);
    k->WhiteningKey[1] ^= (InitialKey[6] <<  8);
    k->WhiteningKey[1] ^= (InitialKey[9] <<  0);

    //RoundKeys
    for (int r = 0; r < NoRound; r++) 
    {
        k->RoundKey[r] ^= C[r];
        if ( r%5 == 0 || r%5 == 2 ) 
            k->RoundKey[r] ^= (InitialKey[4] << 24) ^ (InitialKey[5] << 16) ^ (InitialKey[6] << 8) ^ (InitialKey[7] << 0);
        else if ( r%5 == 1 || r%5 == 4 )
            k->RoundKey[r] ^= (InitialKey[0] << 24) ^ (InitialKey[1] << 16) ^ (InitialKey[2] << 8) ^ (InitialKey[3] << 0);
        else if (r%5 == 3)
            k->RoundKey[r] ^= (InitialKey[8] << 24) ^ (InitialKey[9] << 16) ^ (InitialKey[8] << 8) ^ (InitialKey[9] << 0);
      
    } 
}

void Encrypt(STATE *s, KEY *k, int NoRound) 
{
    //InputWhitening
    s->b[0] ^= (k->WhiteningKey[0] & 0xffff0000);       // X_0 ^= wk_0
    s->b[1] ^= (k->WhiteningKey[0] & 0x0000ffff) << 16; // X_2 ^= wk_1
    BYTE x[2] = {0,0};
    for (int r = 0; r < NoRound; r++) 
    {
        //XOR RoundKey rk_2i to X_1
        s->b[0] ^= (k->RoundKey[r] >> 16) & 0xffff;
        //Extract X_0, apply F function and XOR result to X_1
        x[0] = (s->b[0] >> 24) & 0xff;
        x[1] = (s->b[0] >> 16) & 0xff;
        F_function(x);
        s->b[0] ^= x[0] << 8;
        s->b[0] ^= x[1] << 0;
        //XOR RoundKey rk_2i to X_3
        s->b[1] ^= (k->RoundKey[r] >>  0) & 0xffff;
        //Extract X_2, apply F function and XOR result to X_3
        x[0] = (s->b[1] >> 24) & 0xff;
        x[1] = (s->b[1] >> 16) & 0xff;
        F_function(x);
        s->b[1] ^= x[0] << 8;
        s->b[1] ^= x[1] << 0;
        //Apply RoundPermutation
        if (r != NoRound-1) 
            RoundPermutation(s);
    }
    //OutputWhitening
    s->b[0] ^= (k->WhiteningKey[1] & 0xffff0000);       // X_0 ^= wk_2
    s->b[1] ^= (k->WhiteningKey[1] & 0x0000ffff) << 16; // X_2 ^= wk_3
}

int main () 
{
    //PlainText
    STATE PlainText;
    //InitialKey
    BYTE InitialKey[N80];
       
    WORD w80 = (WORD) strtoul("00112233",NULL,16); //"00112233"  
    InitialKey[0] = (w80 >> 24) & 0xff;
    InitialKey[1] = (w80 >> 16) & 0xff;
    InitialKey[2] = (w80 >>  8) & 0xff;
    InitialKey[3] = (w80 >>  0) & 0xff;
    w80 = (WORD) strtoul("44556677",NULL,16); //"44556677"  
    InitialKey[4] = (w80 >> 24) & 0xff;
    InitialKey[5] = (w80 >> 16) & 0xff;
    InitialKey[6] = (w80 >>  8) & 0xff;
    InitialKey[7] = (w80 >>  0) & 0xff;
    w80 = (WORD) strtoul("8899",NULL,16); //"8899"  
    InitialKey[8] = (w80 >>  8) & 0xff;
    InitialKey[9] = (w80 >>  0) & 0xff;
    
    //Print out the MCU Processor Speed
    pc.printf("CPU SystemCoreClock is %.2f MHz\r\n", (float)SystemCoreClock/1000.0f/1000.0f);
    //Print out the Plain Text
    PlainText.b[0] = (WORD) strtoul("01234567",NULL,16); //"01234567"; 
    PlainText.b[1] = (WORD) strtoul("89abcdef",NULL,16); //"89abcdef";
    pc.printf("PLAIN TEXT: %08x %08x\n", PlainText.b[0], PlainText.b[1]);
    //Print out the Key Used
    pc.printf("SECRET KEY: ");
    for (int i=0; i<N80; i++) 
        pc.printf("%02x",InitialKey[i]);
    pc.printf("\n");

    //Initialize key
    KEY k80;
    
    //Encryption Process
    //To store the Total Clock Cycle
    uint32_t count1,count2;   
    enableDWT_CYCCNT();
    resetDWT_CYCCNT();
    enc_t.start();
    count1 = getDWT_CYCCNT();
    
    //for (int temp=1;temp<=192;temp++)
    {
        KeySchedule(InitialKey,&k80, Constant80, N80, RN80, KEYSIZE80);
        Encrypt(&PlainText,&k80,RN80);
    }
    
    count2 = getDWT_CYCCNT();
    enc_t.stop();
    int BusyClockCycle = count2-count1;
    pc.printf("Time taken to Encrypt was: %.3f msec\n", enc_t.read()*1000);
    pc.printf("Total number of clock cylces used: %d\n",BusyClockCycle);
    pc.printf("CIPHER TEXT:\t");
    pc.printf("%08x %08x\n", PlainText.b[0],PlainText.b[1]);
        
    wait_ms(2000); // 1 second
    return 0;
}