
//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red Backlight LED:
//   PB5 drives an NPN transistor that powers the red LED
// Green Backlight LED:
//   PE4 drives an NPN transistor that powers the green LED
// Blue Backlight LED:
//   PE5 drives an NPN transistor that powers the blue LED
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// Pushbutton:
//   SW1 pulls pin PF4 low (internal pull-up is used)
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "tm4c123gh6pm.h"

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define BLUE_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))
#define DEN            (*((volatile uint32_t *)(0x42000000 +(0x400063FC-0x40000000)*32 + 6*4 )))
int count=0;
char c[31];
char check[31];
char input;
char output[31];
int j;
int charcount=0;
char type1[31];
int fieldposition[20];
int no_of_fields=0;
int command();
int cnt1=0;
int newstringposition[20];
char name[20];
char num[20];
int getnumber();
int address[20];
int  superaddress;
int validation();
uint8_t address_tx;
int error=0;
int index=0;
uint8_t p,q,r,s,t,u,v;
int rs485(p,q,r,s,t,u,v);
uint8_t seq_id_tx=0;
uint8_t channel_field_tx;
uint8_t value_field_tx;
uint8_t cmd_tx;
uint8_t size_tx;
uint8_t checksum;
uint8_t checksum_cal;
uint8_t address_r;
int checksum_r;
uint8_t seq_id_r=0;
uint8_t channel_field_r;
uint8_t value_field_r;
uint8_t cmd_r;
uint8_t size_r;
int interrupt_ip;
int a,b;
int address_check=0;
int recieve_phase=0;
int MY_ADDRESS=7;
int redtimeout=0;
int greentimeout=0;
int re_trans=0;
int re_seq_id;
int re_channel;
int re_cmd;
int re_size;
int re_value;
int re_checksum;
int counter_re=0;

///reciever side declarations/////



//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------




// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);
    
    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;
    
    // Enable GPIO port A and F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF |SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOD;
    
    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R = 0x0A;  // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x0A; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x1A;  // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R = 0x10;  // enable internal pull-up for push button
    
    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R  |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX ;
    
    // Configure UART1 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTC_DEN_R |= 0x70;                           // default, added for clarity
    GPIO_PORTC_DIR_R = 0x60;
    GPIO_PORTC_AFSEL_R |= 0x30;                         // default, added for clarity
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_U1TX | GPIO_PCTL_PC4_U1RX;
    
    
    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
    
    // Configure UART1 to 38400 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART1_CTL_R = 0;                                 // turn-off UART1 to allow safe programming
    UART1_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART1_IBRD_R = 65;                               // r = 40 MHz / (Nx38.4kHz), set floor(r)=65, where N=16
    UART1_FBRD_R = 7;                               // round(fract(r)*64)=6
    UART1_LCRH_R = UART_LCRH_WLEN_8; //| UART_LCRH_PEN |UART_LCRH_SPS; // configure for 8-1-1 w/o 16-level FIFO
    NVIC_EN0_R |= 1 << (INT_UART1-16);
    UART1_IM_R |= UART_IM_RXIM ;
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN | UART_CTL_LBE; // enable TX, RX, and module
    
    //UART1 RX interupt handling
    
    
    /* // Configure Timer 1 for keyboard service
     SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
     TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
     TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
     TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
     TIMER1_TAILR_R = 0x61A80;                        // set load value to 1e5 for 100 Hz interrupt rate
     TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
     
     TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer*/
    
    
    
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    
    
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

void putiUart1(uint8_t n)
{
    
    while (UART1_FR_R & UART_FR_TXFF);
    UART1_DR_R = n;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    int i;
    for (i = 0; i < strlen(str); i++)
    {
        putcUart0(str[i]);
        //delay(1000);
    }
    
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);
    return UART0_DR_R & 0xFF;
}
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------
bool iscommand (char fieldname[20],int minarg)
{
    bool answer;
    
    answer= (strcmp (fieldname,&check[fieldposition[0]])==0) && (no_of_fields >minarg);
    
    return answer ;
    
}

int command()
{
    if (iscommand("reset",1))
    {
        putsUart0("Reset entered\r\n");
    }
    else if (iscommand("set",3))
    {
        putsUart0("Set entered\r\n");
    }
    else if (iscommand("get",2))
    {
        putsUart0("get entered\r\n");
    }
    else if (iscommand("poll",0))
    {
        putsUart0("Poll entered \r\n");
    }
    else if (iscommand("sa",2))
    {
        putsUart0("set a new address  \r\n");
    }
    else if (iscommand("ack",0))
    {
        if (strcmp('on',&check[fieldposition[1]]))
        {
            putsUart0("Acknoledgement entered\r\n");
        }
    }
    else putsUart0("invalid command");
}
int delay(int a)
{
    
    //no of cycles
    //__asm("        MOV R0,#7 ");         //1
    __asm("LINE2: SUB R0,#1 ");       //7
    __asm("        CBZ R0,LOOP2 ");  //6*2+1*3=15
    __asm("          B LINE2 ");        //12
    __asm("LOOP2: NOP ");            //1
    __asm("          NOP ");            //1
    __asm("          NOP ");            //1
    __asm("          NOP ");            //1
    __asm("          NOP ");            //1
    //40 cycles
}
int reset_of_variables()
{
    
    charcount=0;
    cnt1=0;j=0;
    no_of_fields=0;
    index=0;
    error=0;
    int l=0;
    for(l=0;l<33;l++){
        check[l]=0;
    }
}

bool isalpha (int index)
{
    
    int k,len,co=0;
    
    strcpy(name,&check[fieldposition[index]]);
    len=strlen(name);
    for(k=index;k<(index+len);k++)
    {
        if(type1[k]=='a')
        {
            co++;
        }
        else
            putsUart0("\r\nsomething is wrong!!\r\n");
    }
    if(co == len)
    {
        putsUart0("\r\nalphabets entered\r\n");
        superaddress=getalpha();
    }
    
}
bool isnumber (index)
{
    
    int k,len,co=0;
    
    strcpy(num,&check[fieldposition[index]]);
    len=strlen(num);
    //if (index=fieldposition[1])
    
    
    int q=0;
    for(k=index;k<(index+len);k++)
    {
        if(type1[fieldposition[index]+ q] =='n')
        {
            co++;
            q++;
        }
        
    }
    if(co == len)
    {
        putsUart0("\r\nnumber is  entered\r\n");
        return 1;
        
        
    }else
        putsUart0("\r\nNo number is entered\r\n");
    error++;
    return 0;
}

int getnumber(int index)
{
    
    strcpy (address,&check[fieldposition[index]]);
    //sprintf("the address enetred is %s",address);
    int num2;
    num2 = atoi(address);
    return num2;
    
}

int getalpha()
{
    
    int *palpha=&fieldposition[0];
    return palpha;
    
    
}
int validation()
{
    char string1[30];
    
    
    if(iscommand("reset",1))
    {
        if(isnumber(1)&& error==0)
        {
            
            address_tx=getnumber(1);
            
            sprintf(string1,"%u",address_tx);
            putsUart0("RESETTING THE ADDRESS at  \t");
            putsUart0(string1);
            putsUart0("\r\nVALID COMMAND WAS ENTERED\r\n");
            seq_id_tx++;
            cmd_tx=127;
            size_tx=strlen(value_field_tx);
            checksum= ~(address_tx|seq_id_tx|channel_field_tx|cmd_tx|size_tx|value_field_tx);
            rs485(address_tx,seq_id_tx,cmd_tx,channel_field_tx,size_tx,value_field_tx,checksum);
            
            
        }
        else putsUart0("INVALID COMMAND WAS ENTERED\r\n");
        
    }
    if (iscommand("set",3))
    {
        if(isnumber(1))
        {
            
            address_tx=getnumber(1);
            
            sprintf(string1,"%u",address_tx);
            putsUart0("ADDRESS is  ");
            putsUart0(string1);
            
        }
        if(isnumber(2) && error==0)
        {
            
            channel_field_tx=getnumber(2);
            sprintf(string1,"%u",channel_field_tx);
            putsUart0("CHANNEL is\t");
            putsUart0(string1);
            
        }
        if(isnumber(3)&& error==0)
        {
            
            value_field_tx=getnumber(3);
            sprintf(string1,"%u",value_field_tx);
            putsUart0("VALUE is  ");
            putsUart0(string1);
            char strtemp[20] = "\r\nVALID COMMAND\r\n";
            //putsUart0("\r\nVALID COMMAND\r\n");
            putsUart0(strtemp);
            seq_id_tx++;
            cmd_tx=0;
            size_tx=1;
            checksum= ~(address_tx|seq_id_tx|channel_field_tx|cmd_tx|size_tx|value_field_tx);
            rs485(address_tx,seq_id_tx,cmd_tx,channel_field_tx,size_tx,value_field_tx,checksum);
            
        }
        else putsUart0("INVALID COMMAND WAS ENTERED\r\n");
    }
    
    if(iscommand("get",2))
    {
        if(isnumber(1))
        {
            address_tx=getnumber(1);
            sprintf(string1,"%u",address_tx);
            putsUart0("ADDRESS is  ");
            putsUart0(string1);
        }
        if(isnumber(2) && error==0)
        {
            
            channel_field_tx=getnumber(2);
            sprintf(string1,"%u",channel_field_tx);
            putsUart0("CHANNEL is\t");
            putsUart0(string1);
            putsUart0("\r\nVALID COMMAND WAS ENTERED\r\n");
            seq_id_tx++;
            cmd_tx= 48;
            value_field_tx=0;
            size_tx=strlen(value_field_tx);
            checksum= ~(address_tx|seq_id_tx|channel_field_tx|cmd_tx|size_tx|value_field_tx);
            rs485(address_tx,seq_id_tx,cmd_tx,channel_field_tx,size_tx,value_field_tx,checksum);
            
        }
    }
    
    
    if(iscommand("poll",0))
    {
        
        putsUart0("polling is requested \t");
        putsUart0("\r\nVALID COMMAND WAS ENTERED\r\n");
        seq_id_tx++;
        cmd_tx= 120;
        size_tx=strlen(value_field_tx);
        address_tx=255;
        checksum= ~(address_tx|seq_id_tx|channel_field_tx|cmd_tx|size_tx|value_field_tx);
        rs485(address_tx,seq_id_tx,cmd_tx,channel_field_tx,size_tx,value_field_tx,checksum);
    }
    
    
    if(iscommand("sa",2))
    {
        if(isnumber(1))
        {
            
            address_tx=getnumber(1);
            sprintf(string1,"%u",address_tx);
            putsUart0("CURRENT ADDRESS is  ");
            putsUart0(string1);
        }
        if(isnumber(2) && error==0)
        {
            value_field_tx=getnumber(2);
            sprintf(string1,"%u",channel_field_tx);
            putsUart0("NEW ADDRESS is\t");
            putsUart0(string1);
            putsUart0("\r\nVALID COMMAND WAS ENTERED\r\n");
            seq_id_tx++;
            cmd_tx= 121;
            channel_field_tx=0;
            size_tx=1;
            checksum= ~(address_tx|seq_id_tx|channel_field_tx|cmd_tx|size_tx|value_field_tx);
            rs485(address_tx,seq_id_tx,cmd_tx,channel_field_tx,size_tx,value_field_tx,checksum);
        }
    }
    
    if(iscommand("ack",0))
    {
        if (!strcmp("on",&check[fieldposition[1]]))
        {
            putsUart0("Acknoledgement of command is required \r\n");
            putsUart0("\r\nVALID COMMAND WAS ENTERED\r\n");
            seq_id_tx++;
            cmd_tx= 112;
            size_tx=0;
            address_tx=255;
            checksum= ~(address_tx|seq_id_tx|channel_field_tx|cmd_tx|size_tx|value_field_tx);
            rs485(address_tx,seq_id_tx,cmd_tx,channel_field_tx,size_tx,value_field_tx,checksum);
            
            
            
            
            if (!strcmp("off",&check[fieldposition[1]]))
            {
                putsUart0("Acknoledgement of command is not required \r\n");
                putsUart0("\r\nVALID COMMAND WAS ENTERED\r\n");
            }
        }
        else putsUart0("INVALID COMMAND WAS ENTERED\r\n");
        
        
        putsUart0("\r\n");
        
        
        
        
    }
}

int rs485(p,q,r,s,t,u,v)
{
    
    char string1[30];
    //if(re_trans=='1')
    //counter_re++;
    /*p= address field ,q= seq id,r= cmd, s=channel field,t=size,u=value field,v=checksum */
    UART1_CTL_R = 0;
    UART1_LCRH_R = 0xE2 ;
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN | UART_CTL_LBE;
    DEN=1;
    //while(1)
    putiUart1(p);
    delay(100000);
    sprintf(string1,"%d",p);
    putsUart0("\r\naddress is\r\n");
    putsUart0(string1);
    UART1_CTL_R = 0;
    UART1_LCRH_R = 0xE6;
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN | UART_CTL_LBE;
    putiUart1(q);
    delay(100000);
    //sprintf(string1,"%d",q);
    //putsUart0("\r\nseq id is\r\n");
    //putsUart0(string1);
    putiUart1(r);
    delay(100000);
    putiUart1(s);
    delay(100000);
    putiUart1(t);
    delay(100000);
    putiUart1(u);
    delay(100000);
    //while(1)
    putiUart1(v);
    delay(100000);
    //while(!(UART1_FR_R & UART_FR_BUSY))
    DEN=0;
    //TIMER1_isr();
    
    //return ;
    
    
}
/*int getaddress()
 {
 
 bool check2;
 check2= UART1_DR_R ^ 0x10;
 if(check!=0)
 {
 putsUart0("\r\naddress recieved\r\n");
 address_check++;
 }
 return check2;
 }*/
U1recieverISR()
{
    
    char string1[30];
    //recieve_phase=0;
    //GREEN_LED=1;
    interrupt_ip= UART1_DR_R;
    //int f;
    switch(recieve_phase)
    {
        case 0:if(UART1_DR_R ^ 00000200 !=0)
        {
            address_r=interrupt_ip & 0xFF;
            if(address_r==MY_ADDRESS | address_r==255)
            {
                GREEN_LED=1;
                greentimeout=25;
                recieve_phase=1;
                sprintf(string1,"%d",address_r);
                putsUart0("\r\n Adress is\r\n");
                putsUart0(string1);
                //GREEN_LED=0;
                UART1_ICR_R|=0x00;
                break;
            }
            else putsUart0("\r\nNot my address\r\n");
            break;
            
            
        case 1: seq_id_r=interrupt_ip & 0xFF;
            //seq_id=interrupt_ip;
            recieve_phase=2;
            sprintf(string1,"%d",seq_id_r);
            putsUart0("\r\n seq_id is\r\n");
            putsUart0(string1);
            //GREEN_LED=0;
            UART1_ICR_R|=0x00;
            break;
        case 2:
            cmd_r=interrupt_ip & 0xFF;
            recieve_phase=3;
            sprintf(string1,"%d",cmd_r);
            putsUart0("\r\ncmd\r\n");
            putsUart0(string1);
            //GREEN_LED=0;
            UART1_ICR_R|=0x00;
            break;
        case 3:
            channel_field_r=interrupt_ip & 0xFF;
            recieve_phase=4;
            sprintf(string1,"%d",channel_field_r);
            putsUart0("\r\nchannel_field\r\n");
            putsUart0(string1);
            //GREEN_LED=1;
            UART1_ICR_R|=0x00;
            break;
            
        case 4:
            size_r=interrupt_ip & 0xFF;
            recieve_phase=5;
            sprintf(string1,"%d",size_r);
            putsUart0("\r\nsize\r\n");
            putsUart0(string1);
            //GREEN_LED=0;
            UART1_ICR_R|=0x00;
            break;
        case 5:
            value_field_r=interrupt_ip & 0xFF;
            recieve_phase=6;
            sprintf(string1,"%d",value_field_r);
            putsUart0("\r\n value_field\r\n");
            putsUart0(string1);
            //GREEN_LED=1;
            UART1_ICR_R|=0x00;
            break;
            
            
        case 6:
            
            checksum_r=interrupt_ip & 0xFF;
            checksum_cal=~(address_r|seq_id_r|channel_field_r|cmd_r|size_r|value_field_r);
            recieve_phase=0;
            if(checksum_r==checksum_cal)
            {
                sprintf(string1,"%d",checksum_r);
                putsUart0("\r\nchecksum\r\n");
                putsUart0(string1);
                if(cmd_r==112|cmd_r==120)
                    retransmit();
                //retans_fun();
                //GREEN_LED=0;
                counter_re++;
                if((re_trans==1) && (counter_re==2))
                    retans_fun();
                UART1_ICR_R|=0x00;
                GREEN_LED=0;
                break;
            }
            else putsUart0("\r\nchecksum error\r\n");
            break;
            
            
            
        }
            
            
            
            
            
    }
}



int retransmit()
{
    re_trans=1;
    return re_trans;
    
}
int retans_fun()
{
    putsUart0("\r\nRETRANSMISSSION BABBBBAYYYYY!!!!!!\r\n");
    re_seq_id=seq_id_r;
    re_channel=channel_field_r;
    re_cmd=cmd_r;
    re_size=size_r;
    re_value=value_field_r;
    re_checksum=checksum_cal;
    rs485(MY_ADDRESS,re_seq_id,re_channel,re_cmd,re_size,re_value,re_checksum);
    counter_re=0;
    re_trans=0;
    
    
}
//int re_trans_fun()
//{
//rs485(MY_ADDRESS,re_seq_id,re_channel,re_cmd,re_size,re_value,re_checksum);
//counter_re=1;
//}






/*TIMER1_isr()
 {
 while(redtimeout!=0)
 redtimeout--;
 //    if (redtimeout=='0')
 RED_LED=0;
 while(greentimeout!=0)
 greentimeout--;
 //if(greentimeout=='0')
 //GREEN_LED=0;
 }*/


int main(void)
{
    
    
    reset_of_variables();
    // Initialize hardware
    initHw();
    RED_LED = 1;
    //redtimeout=25;
    delay(2000000);
    RED_LED = 0;
    //NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
    
    int i=0;
    //UART1_DR_R=0x55;
    
    //putsUart0("\n\n\Plss enter the string\r\n");
    
    while(1)
    {
        i= 0;charcount=0;cnt1=0;j=0;no_of_fields=0;index=0;
        reset_of_variables();
        //if(re_trans=='1')
        //rs485(MY_ADDRESS,re_seq_id,re_channel,re_cmd,re_size,re_value,re_checksum);
        //if(UART1_DR_R!=0)
        //U1recieverISR();
        putsUart0("\n\n\Plss enter the string\r\n");
        while(i<32)
        {
            //putsUart0("\n\n\Plss enter the string\r\n");
            input = getcUart0();
            
            
            if((input>96 && input<123) | (input>47 && input<58)|(input==32)|(input>32 && input<48)|(input>57 && input<65)|(input>90 && input<97) )
            {
                i++;
                check[i-1]=input;
                charcount++;
                output[i-1]=input-32;
                
            }
            
            if((input==8) & (i>0))
                i--;
            
            if((input>96 && input<123))
                type1[i-1]='a';//type1=1 implies it is an alphabet
            
            if((input>47 && input<58))
                type1[i-1]='n'; //type1=0 implies it is a number
            
            
            if((input>32 && input<48)|(input>57 && input<65)|(input>90 && input<97)|(input==32))
            {
                type1[i-1]='s';//type1=2 implies it is a special charachter
                check[i-1]=0;
            }
            
            
            if (input==13)
            {
                break;
            }
            //i++;
        }
        
        
        while(cnt1<32)
        {
            
            if((type1[cnt1] == 's' && type1[cnt1 + 1] == 'a') || (type1[cnt1] == 's' && type1[cnt1 + 1] == 'n') )
            {
                fieldposition[j]=cnt1+1;
                j++;
                no_of_fields++;
            }
            if(type1[cnt1]=='a'&& cnt1==0)
            {
                fieldposition[j]=cnt1;
                no_of_fields++;
                j++;
            }
            
            
            cnt1++;
        }
        
        
        command();
        isalpha(0);
        isnumber(1);
        validation();
        
    }
    
}

