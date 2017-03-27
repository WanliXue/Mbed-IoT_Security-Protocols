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
int N128=16; //key size length in bytes
const int RN128=31; //number of rounds
int KEYSIZE128=128; //key size

typedef unsigned char BYTE; //8 bit
typedef unsigned long WORD; //32 bit

//key
typedef struct {
  WORD WhiteningKey[2]; // WhiteningKey: WhiteningKey[0] = w_0 | w_1, WhiteningKey[1] = w_2 | w_3
  WORD RoundKey[RN128]; // RoundKey: 1 RoundKey are encoded in a 32-bit word
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
WORD Constant128[RN128] = 
{
    0x6d45ad8a, 0x7543a189, 0x7d41a588, 0x454fb98f, 0x4d4dbd8e,
    0x554bb18d, 0x5d49b58c, 0x25578983, 0x2d558d82, 0x35538181,
    0x3d518580, 0x055f9987, 0x0d5d9d86, 0x155b9185, 0x1d599584,
    0xe567e99b, 0xed65ed9a, 0xf563e199, 0xfd61e598, 0xc56ff99f,
    0xcd6dfd9e, 0xd56bf19d, 0xdd69f59c, 0xa577c993, 0xad75cd92,
    0xb573c191, 0xbd71c590, 0x857fd997, 0x8d7ddd96, 0x957bd195,
    0x9d79d594
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
    //InitialKey is the UserKey: 128-bit: k_0 = InitialKey[0] | InitialKey[1],...,k_8 = InitialKey[14] | InitialKey[15]
    
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
    k->WhiteningKey[1] ^= (InitialKey[15] << 16);
    k->WhiteningKey[1] ^= (InitialKey[14] <<  8);
    k->WhiteningKey[1] ^= (InitialKey[9] <<  0);

    //initialize buffer
    int r = 0;
    BYTE y[N]; 
    for (r = 0; r < N; r++) 
        y[r] = InitialKey[r]; 
    //RoundKeys
    for (r = 0; r < 2*NoRound; r++) 
    {
        int c = (r+2)%8; 
        if (c == 0) 
        {
            y[ 0] = InitialKey[ 4]; y[ 1] = InitialKey[ 5]; // k_0 = k_2
            y[ 2] = InitialKey[ 2]; y[ 3] = InitialKey[ 3]; // k_1 = k_1
            y[ 4] = InitialKey[12]; y[ 5] = InitialKey[13]; // k_2 = k_6
            y[ 6] = InitialKey[14]; y[ 7] = InitialKey[15]; // k_3 = k_7
            y[ 8] = InitialKey[ 0]; y[ 9] = InitialKey[ 1]; // k_4 = k_0
            y[10] = InitialKey[ 6]; y[11] = InitialKey[ 7]; // k_5 = k_3
            y[12] = InitialKey[ 8]; y[13] = InitialKey[ 9]; // k_6 = k_4
            y[14] = InitialKey[10]; y[15] = InitialKey[11]; // k_7 = k_5
            
            //update the InitialKey
            for (int i = 0; i < N; i++)
                InitialKey[i] = y[i]; 
        }
        if (r%2 == 0) 
            k->RoundKey[r/2] ^= (C[r/2] & 0xffff0000) ^ (y[2*c] << 24) ^ (y[2*c+1] << 16);
        else
            k->RoundKey[r/2] ^= (C[r/2] & 0x0000ffff) ^ (y[2*c] << 8) ^ (y[2*c+1] << 0);
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
    BYTE InitialKey[N128];
       
    WORD w128 = (WORD) strtoul("00112233",NULL,16); //"00112233"  
    InitialKey[0] = (w128 >> 24) & 0xff;
    InitialKey[1] = (w128 >> 16) & 0xff;
    InitialKey[2] = (w128 >>  8) & 0xff;
    InitialKey[3] = (w128 >>  0) & 0xff;
    w128 = (WORD) strtoul("44556677",NULL,16); //"44556677"  
    InitialKey[4] = (w128 >> 24) & 0xff;
    InitialKey[5] = (w128 >> 16) & 0xff;
    InitialKey[6] = (w128 >>  8) & 0xff;
    InitialKey[7] = (w128 >>  0) & 0xff;
    w128 = (WORD) strtoul("8899aabb",NULL,16); //"8899aabb"  
    InitialKey[8] = (w128 >>  24) & 0xff;
    InitialKey[9] = (w128 >>  16) & 0xff;
    InitialKey[10] = (w128 >>  8) & 0xff;
    InitialKey[11] = (w128 >>  0) & 0xff;
    w128 = (WORD) strtoul("ccddeeff",NULL,16); //"ccddeeff"  
    InitialKey[12] = (w128 >> 24) & 0xff;
    InitialKey[13] = (w128 >> 16) & 0xff;
    InitialKey[14] = (w128 >>  8) & 0xff;
    InitialKey[15] = (w128 >>  0) & 0xff;
            
    
    //Print out the MCU Processor Speed
    pc.printf("CPU SystemCoreClock is %.2f MHz\r\n", (float)SystemCoreClock/1000.0f/1000.0f);
    //Print out the Plain Text
    PlainText.b[0] = (WORD) strtoul("01234567",NULL,16); //"01234567"; 
    PlainText.b[1] = (WORD) strtoul("89abcdef",NULL,16); //"89abcdef";
    pc.printf("PLAIN TEXT: %08x %08x\n", PlainText.b[0], PlainText.b[1]);
    //Print out the Key Used
    pc.printf("SECRET KEY: ");
    for (int i=0; i<N128; i++) 
        pc.printf("%02x",InitialKey[i]);
    pc.printf("\n");

    //Initialize key
    KEY k128;
    
    //Encryption Process
    //To store the Total Clock Cycle
    uint32_t count1,count2;   
    enableDWT_CYCCNT();
    resetDWT_CYCCNT();
    enc_t.start();
    count1 = getDWT_CYCCNT();
    
    //for (int temp=1;temp<=192;temp++)
    {
        KeySchedule(InitialKey,&k128, Constant128, N128, RN128, KEYSIZE128);
        Encrypt(&PlainText,&k128,RN128);
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