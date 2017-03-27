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

#define Multiply4(_x) (Multiply2(Multiply2((_x)))) 
#define Multiply6(_x) (Multiply2((_x)) ^ Multiply4((_x)))
#define Multiply8(_x) (Multiply2(Multiply4((_x))))
#define MultiplyA(_x) (Multiply2((_x)) ^ Multiply8((_x)))
//Bitwise XOR (Exclusive OR operation)
void XOR(unsigned char *destination, const unsigned char *a, const unsigned char *b, int byte_length)
{
  while(byte_length-- > 0){
    *destination++ = *a++ ^ *b++;
  }
}

unsigned char Multiply2(unsigned char x)
{
    // multiplication over GF(2^8) 
    if(x & 0x80U)
        x ^= 0x0eU;
    return ((x << 1) | (x >> 7)); //bitwise inclusive OR
}
//Copy the entire bytes from source to destination
void copy(unsigned char *destination, const unsigned char *source, int byte_length)
{
    while(byte_length-- > 0)
        *destination++ = *source++;
}

//------------------------------2 SBox used-------------------------------------
// first variant: S0 (8-bit based on four 4-bit S-boxes) 
const unsigned char SBox0[256] = {
  0x57U, 0x49U, 0xd1U, 0xc6U, 0x2fU, 0x33U, 0x74U, 0xfbU,
  0x95U, 0x6dU, 0x82U, 0xeaU, 0x0eU, 0xb0U, 0xa8U, 0x1cU,
  0x28U, 0xd0U, 0x4bU, 0x92U, 0x5cU, 0xeeU, 0x85U, 0xb1U,
  0xc4U, 0x0aU, 0x76U, 0x3dU, 0x63U, 0xf9U, 0x17U, 0xafU,
  0xbfU, 0xa1U, 0x19U, 0x65U, 0xf7U, 0x7aU, 0x32U, 0x20U,
  0x06U, 0xceU, 0xe4U, 0x83U, 0x9dU, 0x5bU, 0x4cU, 0xd8U,
  0x42U, 0x5dU, 0x2eU, 0xe8U, 0xd4U, 0x9bU, 0x0fU, 0x13U,
  0x3cU, 0x89U, 0x67U, 0xc0U, 0x71U, 0xaaU, 0xb6U, 0xf5U,
  0xa4U, 0xbeU, 0xfdU, 0x8cU, 0x12U, 0x00U, 0x97U, 0xdaU,
  0x78U, 0xe1U, 0xcfU, 0x6bU, 0x39U, 0x43U, 0x55U, 0x26U,
  0x30U, 0x98U, 0xccU, 0xddU, 0xebU, 0x54U, 0xb3U, 0x8fU,
  0x4eU, 0x16U, 0xfaU, 0x22U, 0xa5U, 0x77U, 0x09U, 0x61U,
  0xd6U, 0x2aU, 0x53U, 0x37U, 0x45U, 0xc1U, 0x6cU, 0xaeU,
  0xefU, 0x70U, 0x08U, 0x99U, 0x8bU, 0x1dU, 0xf2U, 0xb4U,
  0xe9U, 0xc7U, 0x9fU, 0x4aU, 0x31U, 0x25U, 0xfeU, 0x7cU,
  0xd3U, 0xa2U, 0xbdU, 0x56U, 0x14U, 0x88U, 0x60U, 0x0bU,
  0xcdU, 0xe2U, 0x34U, 0x50U, 0x9eU, 0xdcU, 0x11U, 0x05U,
  0x2bU, 0xb7U, 0xa9U, 0x48U, 0xffU, 0x66U, 0x8aU, 0x73U,
  0x03U, 0x75U, 0x86U, 0xf1U, 0x6aU, 0xa7U, 0x40U, 0xc2U,
  0xb9U, 0x2cU, 0xdbU, 0x1fU, 0x58U, 0x94U, 0x3eU, 0xedU,
  0xfcU, 0x1bU, 0xa0U, 0x04U, 0xb8U, 0x8dU, 0xe6U, 0x59U,
  0x62U, 0x93U, 0x35U, 0x7eU, 0xcaU, 0x21U, 0xdfU, 0x47U,
  0x15U, 0xf3U, 0xbaU, 0x7fU, 0xa6U, 0x69U, 0xc8U, 0x4dU,
  0x87U, 0x3bU, 0x9cU, 0x01U, 0xe0U, 0xdeU, 0x24U, 0x52U,
  0x7bU, 0x0cU, 0x68U, 0x1eU, 0x80U, 0xb2U, 0x5aU, 0xe7U,
  0xadU, 0xd5U, 0x23U, 0xf4U, 0x46U, 0x3fU, 0x91U, 0xc9U,
  0x6eU, 0x84U, 0x72U, 0xbbU, 0x0dU, 0x18U, 0xd9U, 0x96U,
  0xf0U, 0x5fU, 0x41U, 0xacU, 0x27U, 0xc5U, 0xe3U, 0x3aU,
  0x81U, 0x6fU, 0x07U, 0xa3U, 0x79U, 0xf6U, 0x2dU, 0x38U,
  0x1aU, 0x44U, 0x5eU, 0xb5U, 0xd2U, 0xecU, 0xcbU, 0x90U,
  0x9aU, 0x36U, 0xe5U, 0x29U, 0xc3U, 0x4fU, 0xabU, 0x64U,
  0x51U, 0xf8U, 0x10U, 0xd7U, 0xbcU, 0x02U, 0x7dU, 0x8eU
};

//second variant: S1 (8-bit based on inverse function) 
const unsigned char SBox1[256] = {
  0x6cU, 0xdaU, 0xc3U, 0xe9U, 0x4eU, 0x9dU, 0x0aU, 0x3dU,
  0xb8U, 0x36U, 0xb4U, 0x38U, 0x13U, 0x34U, 0x0cU, 0xd9U,
  0xbfU, 0x74U, 0x94U, 0x8fU, 0xb7U, 0x9cU, 0xe5U, 0xdcU,
  0x9eU, 0x07U, 0x49U, 0x4fU, 0x98U, 0x2cU, 0xb0U, 0x93U,
  0x12U, 0xebU, 0xcdU, 0xb3U, 0x92U, 0xe7U, 0x41U, 0x60U,
  0xe3U, 0x21U, 0x27U, 0x3bU, 0xe6U, 0x19U, 0xd2U, 0x0eU,
  0x91U, 0x11U, 0xc7U, 0x3fU, 0x2aU, 0x8eU, 0xa1U, 0xbcU,
  0x2bU, 0xc8U, 0xc5U, 0x0fU, 0x5bU, 0xf3U, 0x87U, 0x8bU,
  0xfbU, 0xf5U, 0xdeU, 0x20U, 0xc6U, 0xa7U, 0x84U, 0xceU,
  0xd8U, 0x65U, 0x51U, 0xc9U, 0xa4U, 0xefU, 0x43U, 0x53U,
  0x25U, 0x5dU, 0x9bU, 0x31U, 0xe8U, 0x3eU, 0x0dU, 0xd7U,
  0x80U, 0xffU, 0x69U, 0x8aU, 0xbaU, 0x0bU, 0x73U, 0x5cU,
  0x6eU, 0x54U, 0x15U, 0x62U, 0xf6U, 0x35U, 0x30U, 0x52U,
  0xa3U, 0x16U, 0xd3U, 0x28U, 0x32U, 0xfaU, 0xaaU, 0x5eU,
  0xcfU, 0xeaU, 0xedU, 0x78U, 0x33U, 0x58U, 0x09U, 0x7bU,
  0x63U, 0xc0U, 0xc1U, 0x46U, 0x1eU, 0xdfU, 0xa9U, 0x99U,
  0x55U, 0x04U, 0xc4U, 0x86U, 0x39U, 0x77U, 0x82U, 0xecU,
  0x40U, 0x18U, 0x90U, 0x97U, 0x59U, 0xddU, 0x83U, 0x1fU,
  0x9aU, 0x37U, 0x06U, 0x24U, 0x64U, 0x7cU, 0xa5U, 0x56U,
  0x48U, 0x08U, 0x85U, 0xd0U, 0x61U, 0x26U, 0xcaU, 0x6fU,
  0x7eU, 0x6aU, 0xb6U, 0x71U, 0xa0U, 0x70U, 0x05U, 0xd1U,
  0x45U, 0x8cU, 0x23U, 0x1cU, 0xf0U, 0xeeU, 0x89U, 0xadU,
  0x7aU, 0x4bU, 0xc2U, 0x2fU, 0xdbU, 0x5aU, 0x4dU, 0x76U,
  0x67U, 0x17U, 0x2dU, 0xf4U, 0xcbU, 0xb1U, 0x4aU, 0xa8U,
  0xb5U, 0x22U, 0x47U, 0x3aU, 0xd5U, 0x10U, 0x4cU, 0x72U,
  0xccU, 0x00U, 0xf9U, 0xe0U, 0xfdU, 0xe2U, 0xfeU, 0xaeU,
  0xf8U, 0x5fU, 0xabU, 0xf1U, 0x1bU, 0x42U, 0x81U, 0xd6U,
  0xbeU, 0x44U, 0x29U, 0xa6U, 0x57U, 0xb9U, 0xafU, 0xf2U,
  0xd4U, 0x75U, 0x66U, 0xbbU, 0x68U, 0x9fU, 0x50U, 0x02U,
  0x01U, 0x3cU, 0x7fU, 0x8dU, 0x1aU, 0x88U, 0xbdU, 0xacU,
  0xf7U, 0xe4U, 0x79U, 0x96U, 0xa2U, 0xfcU, 0x6dU, 0xb2U,
  0x6bU, 0x03U, 0xe1U, 0x2eU, 0x7dU, 0x14U, 0x95U, 0x1dU
};

//------------------------------2 Ffunctions used-------------------------------
//F-function f0
void F0_XOR(unsigned char *destination, const unsigned char *source, const unsigned char *RoundKey)
{
    unsigned char x[4], y[4], z[4];
    // XOR input, round key and store it into x
    XOR(x, source, RoundKey, 4);
    // Substitution layer 
    z[0] = SBox0[x[0]];
    z[1] = SBox1[x[1]];
    z[2] = SBox0[x[2]];
    z[3] = SBox1[x[3]];
    // Diffusion layer (M0) 
    y[0] =            z[0]  ^ Multiply2(z[1]) ^ Multiply4(z[2]) ^ Multiply6(z[3]);
    y[1] = Multiply2(z[0]) ^            z[1]  ^ Multiply6(z[2]) ^ Multiply4(z[3]);
    y[2] = Multiply4(z[0]) ^ Multiply6(z[1]) ^            z[2]  ^ Multiply2(z[3]);
    y[3] = Multiply6(z[0]) ^ Multiply4(z[1]) ^ Multiply2(z[2]) ^            z[3] ;
    // XOR to produce intermediate output
    copy(destination + 0, source + 0, 4);
    XOR(destination + 4, source + 4, y, 4);
}
//F-function f1
void F1_XOR(unsigned char *destination, const unsigned char *source, const unsigned char *RoundKey)
{
    unsigned char x[4], y[4], z[4];
    // XOR input, round key and store it into x
    XOR(x, source, RoundKey, 4);
    //Substitution layer 
    z[0] = SBox1[x[0]];
    z[1] = SBox0[x[1]];
    z[2] = SBox1[x[2]];
    z[3] = SBox0[x[3]];
    //Diffusion layer (M1) 
    y[0] =            z[0]  ^ Multiply8(z[1]) ^ Multiply2(z[2]) ^ MultiplyA(z[3]);
    y[1] = Multiply8(z[0]) ^            z[1]  ^ MultiplyA(z[2]) ^ Multiply2(z[3]);
    y[2] = Multiply2(z[0]) ^ MultiplyA(z[1]) ^            z[2]  ^ Multiply8(z[3]);
    y[3] = MultiplyA(z[0]) ^ Multiply2(z[1]) ^ Multiply8(z[2]) ^            z[3] ;
    // XOR to produce intermediate output
    copy(destination + 0, source + 0, 4);
    XOR(destination + 4, source + 4, y, 4);
}

// GFN(4,18):Basic Structure for Key size 128bit (4 data line and 18 rounds) 
void GFN_4(unsigned char *y, const unsigned char *x, const unsigned char *RoundKey, int NoRounds)
{
    // Input and Output of intermediate Round Function
    unsigned char fin[16], fout[16];
    copy(fin, x, 16);
    while(NoRounds-- > 0)
    {
        F0_XOR(fout + 0, fin + 0, RoundKey + 0);
        F1_XOR(fout + 8, fin + 8, RoundKey + 4);
        RoundKey += 8;
        if(NoRounds)
        { 
            // swapping for encryption 
            copy(fin + 0,  fout + 4, 12);
            copy(fin + 12, fout + 0, 4);
        }
    }
    copy(y, fout, 16);
}
// GFN(4,18):Basic Structure for Key size 128bit (4 data line and 18 rounds) 
void GFN_8(unsigned char *y, const unsigned char *x, const unsigned char *RoundKey, int NoRounds)
{
    // Input and Output of intermediate Round Function
    unsigned char fin[32], fout[32];
    copy(fin, x, 32);
    while(NoRounds-- > 0)
    {
        F0_XOR(fout + 0,  fin + 0,  RoundKey + 0);
        F1_XOR(fout + 8,  fin + 8,  RoundKey + 4);
        F0_XOR(fout + 16, fin + 16, RoundKey + 8);
        F1_XOR(fout + 24, fin + 24, RoundKey + 12);
        RoundKey += 16;
        if(NoRounds)
        { 
            // swapping for encryption 
            copy(fin + 0,  fout + 4, 28);
            copy(fin + 28, fout + 0, 4);
        }
    }
    copy(y, fout, 32);
}

// Double Swap function to update intermediate values every 2 rounds
void DoubleSwap(unsigned char *IntermediateKey)
{
    // To store the new intermediate value to genearte RoundKey and WhiteningKey
    unsigned char t[16];
    t[0]  = (IntermediateKey[0] << 7) | (IntermediateKey[1]  >> 1);
    t[1]  = (IntermediateKey[1] << 7) | (IntermediateKey[2]  >> 1);
    t[2]  = (IntermediateKey[2] << 7) | (IntermediateKey[3]  >> 1);
    t[3]  = (IntermediateKey[3] << 7) | (IntermediateKey[4]  >> 1);
    t[4]  = (IntermediateKey[4] << 7) | (IntermediateKey[5]  >> 1);
    t[5]  = (IntermediateKey[5] << 7) | (IntermediateKey[6]  >> 1);
    t[6]  = (IntermediateKey[6] << 7) | (IntermediateKey[7]  >> 1);
    t[7]  = (IntermediateKey[7] << 7) | (IntermediateKey[15] & 0x7fU);

    t[8]  = (IntermediateKey[8]  >> 7) | (IntermediateKey[0]  & 0xfeU);
    t[9]  = (IntermediateKey[9]  >> 7) | (IntermediateKey[8]  << 1);
    t[10] = (IntermediateKey[10] >> 7) | (IntermediateKey[9]  << 1);
    t[11] = (IntermediateKey[11] >> 7) | (IntermediateKey[10] << 1);
    t[12] = (IntermediateKey[12] >> 7) | (IntermediateKey[11] << 1);
    t[13] = (IntermediateKey[13] >> 7) | (IntermediateKey[12] << 1);
    t[14] = (IntermediateKey[14] >> 7) | (IntermediateKey[13] << 1);
    t[15] = (IntermediateKey[15] >> 7) | (IntermediateKey[14] << 1);

    copy(IntermediateKey, t, 16);
}

// Generate Constants(32bit) 128bit key size takes 60 constants
// 192bit key size takes 84, and 256bit key value takes 92 constants
void ConstantGenerate(unsigned char *ConstantValue, const unsigned char *InitialConstant, int IntermediateKey)
{
    unsigned char t[2];
    unsigned char tmp;
    copy(t, InitialConstant, 2);
    while(IntermediateKey-- > 0)
    {
        ConstantValue[0] = t[0] ^ 0xb7U; // P_16 = 0xb7e1 (natural logarithm) 
        ConstantValue[1] = t[1] ^ 0xe1U;
        ConstantValue[2] = ~((t[0] << 1) | (t[1] >> 7));
        ConstantValue[3] = ~((t[1] << 1) | (t[0] >> 7));
        ConstantValue[4] = ~t[0] ^ 0x24U; // Q_16 = 0x243f (circle ratio) 
        ConstantValue[5] = ~t[1] ^ 0x3fU;
        ConstantValue[6] = t[1];
        ConstantValue[7] = t[0];
        ConstantValue += 8;
        // Updating T 
        if(t[1] & 0x01U)
        {
            t[0] ^= 0xa8U;
            t[1] ^= 0x30U;
        }
        tmp = t[0] << 7;
        t[0] = (t[0] >> 1) | (t[1] << 7);
        t[1] = (t[1] >> 1) | tmp;
    }    
}

void KeySchedule128(unsigned char *RoundKey, const unsigned char *InitialKey)
{
    const unsigned char InitialConstant[2] = {0x42U, 0x8aU}; // cubic root of 2 
    unsigned char IntermediateKey[16];
    unsigned char con128[4 * 60];
    int i;
    // Generate CONi^(128) (0 <= i < 60, IntermediateKey = 30) 
    ConstantGenerate(con128, InitialConstant, 30);
    // GFN(4,12) (generating IntermediateKey from InitialKey) 
    GFN_4(IntermediateKey, InitialKey, con128, 12);
    // Expanding InitalKey and IntermediateKey to generate RoundKey and WhiteningKey
    copy(RoundKey, InitialKey, 8); // InitialWhiteningKey (WK0, WK1) 
    RoundKey += 8;
    for(i = 0; i < 9; i++)
    { 
        // RoundKey (RKi (0 <= i < 36)) 
        XOR(RoundKey, IntermediateKey, con128 + i * 16 + (4 * 24), 16);
        if(i % 2)
        {
            XOR(RoundKey, RoundKey, InitialKey, 16); 
        }
        DoubleSwap(IntermediateKey); // Updating L (DoubleSwap function) 
        RoundKey += 16;
    }
    copy(RoundKey, InitialKey + 8, 8); // FinalWhiteningKey (WK2, WK3)
}

void Encrypt(unsigned char *CipherText, const unsigned char *PlainText, const unsigned char *RoundKey, const int NoRounds)
{
    // Input and Output for each Round 
    unsigned char rin[16], rout[16];
    copy(rin,  PlainText,  16);
    XOR(rin + 4,  rin + 4,  RoundKey + 0, 4); //InitialKeyWhitening
    XOR(rin + 12, rin + 12, RoundKey + 4, 4);
    RoundKey += 8;
    GFN_4(rout, rin, RoundKey, NoRounds); //GFN{4,18}
    copy(CipherText, rout, 16);
    XOR(CipherText + 4,  CipherText + 4,  RoundKey + NoRounds * 8 + 0, 4); //FinalKeyWhitening
    XOR(CipherText + 12, CipherText + 12, RoundKey + NoRounds * 8 + 4, 4);
}

void Output(const unsigned char *Data, int ByteLen)
{
    while(ByteLen-- > 0)
        pc.printf("%02x", *Data++);
  pc.printf("\n");
}

int main(void)
{
    //Initial Key value: ffeeddccbbaa99887766554433221100f0e0d0c0b0a0908070605040302010
    const unsigned char InitialKey[32] = {
        0xffU,0xeeU,0xddU,0xccU,0xbbU,0xaaU,0x99U,0x88U,
        0x77U,0x66U,0x55U,0x44U,0x33U,0x22U,0x11U,0x00U,
        0xf0U,0xe0U,0xd0U,0xc0U,0xb0U,0xa0U,0x90U,0x80U,
        0x70U,0x60U,0x50U,0x40U,0x30U,0x20U,0x10U,0x00U
    };
    //Plain Text: 000102030405060708090a0b0c0d0e0f
    const unsigned char PlainText[16] = {
        0x00U,0x01U,0x02U,0x03U,0x04U,0x05U,0x06U,0x07U,
        0x08U,0x09U,0x0aU,0x0bU,0x0cU,0x0dU,0x0eU,0x0fU
    };
    unsigned char dst_enc[16]; //to store the Cipher Text
    int NoRounds = 18; //Number of rounds for 128Bit key
    unsigned char RoundKey[8 * NoRounds + 16]; // 8 bytes x 18 rounds(max) + whitening keys 
    //Print out the MCU Processor Speed
    pc.printf("CPU SystemCoreClock is %.2f MHz\r\n", (float)SystemCoreClock/1000.0f/1000.0f);
    //Print out the Plain Text
    pc.printf("PLAIN TEXT:  "); Output(PlainText, 16);
    //Print out the Key Used
    pc.printf("SECRET KEY:  "); Output(InitialKey, 32);
    pc.printf("\nCLEFIA: 128Bit KEY SIZE\n");
    
    //Encryption Process
    //To store the Total Clock Cycle
    uint32_t count1,count2;   
    enableDWT_CYCCNT();
    resetDWT_CYCCNT();
    enc_t.start();
    count1 = getDWT_CYCCNT();
    
    for (int temp=1;temp<=192;temp++)
    {
        KeySchedule128(RoundKey, InitialKey);
        Encrypt(dst_enc, PlainText, RoundKey, NoRounds);
    }
  
    count2 = getDWT_CYCCNT();
    enc_t.stop();
    int BusyClockCycle = count2-count1;
    pc.printf("Time taken to Encrypt was: %.3f msec\n", enc_t.read()*1000);
    pc.printf("Total number of clock cylces used: %d\n",BusyClockCycle);
    pc.printf("CIPHER TEXT:\t");
    Output(dst_enc, 16);
  
    wait_ms(2000); // 1 second
    return 0;
}


