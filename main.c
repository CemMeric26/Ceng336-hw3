/* =================================================================
 *  File : main.c ? Parking?lot controller (serialio?style, quick?fix) 
 * =================================================================
 *  Structure cloned from sample serialio.c (ring buffers, ISR, tasks)
 *  Now with correct tick?flag logic and no spurious messages.
 *  Remaining TODOs: slot management, queue, fee, 7?segment.
 * =================================================================*/

#include "pragmas.h"
#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define _XTAL_FREQ   40000000UL
#define TMR0_RELOAD  64554          /* 1?ms tick (1:8 prescale) */

/* ---------- UART ring?buffer (unchanged from serialio.c) ---------- */
#define BUFSIZE 128
static uint8_t inbuf[BUFSIZE], outbuf[BUFSIZE];
static uint8_t head[2] = {0,0}, tail[2] = {0,0};
typedef enum {INBUF = 0, OUTBUF = 1} buf_t;

static inline void disable_rxtx(void){ PIE1bits.RC1IE = 0; PIE1bits.TX1IE = 0; }
static inline void enable_rxtx (void){ PIE1bits.RC1IE = 1; PIE1bits.TX1IE = 1; }
static uint8_t buf_isempty(buf_t b){ return head[b]==tail[b]; }
static void buf_push(uint8_t v, buf_t b){ if(b==INBUF) inbuf[head[b]]=v; else outbuf[head[b]]=v; if(++head[b]==BUFSIZE) head[b]=0; }
static uint8_t buf_pop(buf_t b){ uint8_t v=(b==INBUF)?inbuf[tail[b]]:outbuf[tail[b]]; if(++tail[b]==BUFSIZE) tail[b]=0; return v; }

/* ------------------- High?priority UART ISR ---------------------- */
static void uart_isr(void){
    if(PIR1bits.RC1IF){ buf_push(RCREG1,INBUF); PIR1bits.RC1IF=0; }
    if(PIR1bits.TX1IF){ if(!buf_isempty(OUTBUF)) TXREG1=buf_pop(OUTBUF); else PIE1bits.TX1IE=0; PIR1bits.TX1IF=0; }
}
void __interrupt(high_priority) isr_high(void){ uart_isr(); }

/* ---------------- Packet framing task ($ ? #) -------------------- */
#define PKT_HEADER '$'
#define PKT_END    '#'
#define PKT_MAX_SIZE 12
static enum {PKT_WAIT_HDR, PKT_GET_BODY, PKT_WAIT_ACK} pkt_state = PKT_WAIT_HDR;
static uint8_t pkt_body[PKT_MAX_SIZE];
static uint8_t pkt_bodysize=0, pkt_valid=0;
static void packet_task(void){
    disable_rxtx();
    if(!buf_isempty(INBUF)){
        uint8_t v;
        switch(pkt_state){
        case PKT_WAIT_HDR:
            v=buf_pop(INBUF);
            if(v==PKT_HEADER){ pkt_state=PKT_GET_BODY; pkt_bodysize=0; }
            break;
        case PKT_GET_BODY:
            v=buf_pop(INBUF);
            if(v==PKT_END){ pkt_state=PKT_WAIT_ACK; pkt_valid=1; }
            else if(v==PKT_HEADER){ pkt_bodysize=0; }
            else if(pkt_bodysize<PKT_MAX_SIZE) pkt_body[pkt_bodysize++]=v;
            break;
        case PKT_WAIT_ACK:
            if(!pkt_valid){ pkt_state=PKT_WAIT_HDR; }
            break;
        }
    }
    enable_rxtx();
}

/* ------------------- Parking?lot data & helpers ------------------ */
#define LEVELS 4
#define SLOTS_PER 10
static uint8_t is_running=0;
static volatile uint8_t tick_100ms=0, tick_500ms=0;
static volatile uint16_t adc_value=0;

/* simple one?slot outgoing message buffer */
static char pending_msg[16];
static uint8_t pending_len=0;
static void queue_msg(const char *s){ if(pending_len==0){ pending_len=strlen(s); memcpy(pending_msg,s,pending_len+1);} }

/* ------------------- Parking_task: parses commands --------------- */
static void parking_task(void){
    if(!pkt_valid) return;
    char cmd[PKT_MAX_SIZE+1]; memcpy(cmd,pkt_body,pkt_bodysize); cmd[pkt_bodysize]=0;
    if(strcmp(cmd,"GO")==0){ is_running=1; }
    else if(strcmp(cmd,"END")==0){ is_running=0; pending_len=0; }
    else if(strncmp(cmd,"PRK",3)==0){ /* TODO */ }
    else if(strncmp(cmd,"EXT",3)==0){ /* TODO */ }
    else if(strncmp(cmd,"SUB",3)==0){ /* TODO */ }
    pkt_valid=0;
}

/* ------------------- Output_task every 100?ms -------------------- */
static uint8_t emp_counter=0;
static void output_task(void){
    if(!tick_100ms || !is_running) return;      /* only once per slot */
    tick_100ms=0;
    if(pending_len){ for(uint8_t i=0;i<pending_len;i++) buf_push(pending_msg[i],OUTBUF); pending_len=0; }
    else{ char emp[8]; sprintf(emp,"$EMP%02u#",emp_counter++); for(char *p=emp;*p;p++) buf_push(*p,OUTBUF);}    
    if(!PIE1bits.TX1IE){ PIE1bits.TX1IE=1; TXREG1=buf_pop(OUTBUF);} /* kick TX */
}

/* ------------------- Low?priority ISR (Timer0+ADC) --------------- */
void __interrupt(low_priority) isr_low(void){
    if(INTCONbits.TMR0IF)
    { 
        INTCONbits.TMR0IF=0; TMR0=TMR0_RELOAD;
        static uint8_t ms100=0, ms500=0;
        if(++ms100==100){ ms100=0; tick_100ms=1;}
        if(++ms500==100){ if(ms500==5){ ms500=0; tick_500ms=1; } } /* keeps 500?ms flag */
    }
    if(PIR1bits.ADIF){ PIR1bits.ADIF=0; adc_value=((uint16_t)ADRESH<<8)|ADRESL; }
}

/* ------------------- Hardware init ------------------------------- */
static void hw_init(void){
    /* Timer0 */ T0CON=0b00001000; TMR0=TMR0_RELOAD; INTCON2bits.TMR0IP=0; INTCONbits.TMR0IE=1; T0CONbits.TMR0ON=1;
    /* UART 115200 */ TXSTA1bits.SYNC=0; TXSTA1bits.BRGH=1; BAUDCON1bits.BRG16=0; SPBRG1=21; RCSTA1bits.CREN=1; RCSTA1bits.SPEN=1;
    /* ADC AN12 */ TRISHbits.TRISH4=1; ADCON0=0b00110001; ADCON1=0x0E; ADCON2=0b10111110; PIE1bits.ADIE=1; IPR1bits.ADIP=1;
    enable_rxtx(); INTCONbits.PEIE=1; INTCONbits.GIE=1; RCONbits.IPEN=1;
}

/* ------------------- MAIN ---------------------------------------- */
void main(void){
    hw_init();
    while(1){
        packet_task();       /* Frame incoming stream */
        parking_task();      /* Process commands & queue messages */
        output_task();       /* Send exactly one every 100?ms */
        if(is_running && tick_500ms){ tick_500ms=0; ADCON0bits.GO=1; }
    }
}