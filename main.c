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
    if(PIR1bits.RC1IF){ 
        buf_push(RCREG1,INBUF); 
        PIR1bits.RC1IF=0; 
    }
    if(PIR1bits.TX1IF){
        if(!buf_isempty(OUTBUF)){
            TXREG1=buf_pop(OUTBUF);
            PIR1bits.TX1IF=0;
        } else {
            PIE1bits.TX1IE=0;
            PIR1bits.TX1IF=0;
        } }
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
#define TOTAL_SLOTS (LEVELS * SLOTS_PER)  // 40 total slots

// Parking slot states
typedef enum {
    SLOT_EMPTY = 0,
    SLOT_OCCUPIED = 1,
    SLOT_RESERVED = 2
} slot_state_t;

// Parking slot structure
typedef struct {
    slot_state_t state;
    char license_plate[4];  // 3 digits + null terminator
} parking_slot_t;

// Global parking lot state
static parking_slot_t parking_lot[LEVELS][SLOTS_PER];
static uint8_t empty_spaces = TOTAL_SLOTS;  // Initially all 40 spaces are empty

static uint8_t is_running=0;
static volatile uint8_t tick_100ms=0, tick_500ms=0;
static volatile uint16_t adc_value=0;

/* simple one?slot outgoing message buffer */
static char pending_msg[16];
static uint8_t pending_len=0;
static void queue_msg(const char *s){ if(pending_len==0){ pending_len=strlen(s); memcpy(pending_msg,s,pending_len+1);} }

// Helper functions for parking lot management
static void init_parking_lot(void) {
    for(uint8_t level = 0; level < LEVELS; level++) {
        for(uint8_t slot = 0; slot < SLOTS_PER; slot++) {
            parking_lot[level][slot].state = SLOT_EMPTY;
            parking_lot[level][slot].license_plate[0] = '\0';
        }
    }
    empty_spaces = TOTAL_SLOTS;
}

// Convert level index (0-3) to level character (A-D)
static char level_to_char(uint8_t level) {
    return 'A' + level;
}

// Convert level character (A-D) to level index (0-3)
static uint8_t char_to_level(char level_char) {
    return level_char - 'A';
}

// Find first available non-reserved slot
static uint8_t find_available_slot(uint8_t *level, uint8_t *slot) {
    for(uint8_t l = 0; l < LEVELS; l++) {
        for(uint8_t s = 0; s < SLOTS_PER; s++) {
            if(parking_lot[l][s].state == SLOT_EMPTY) {
                *level = l;
                *slot = s;
                return 1;  // Found
            }
        }
    }
    
    return 0;  // No available slot
}

/* ------------------- Reservation --------------- */

typedef struct {
    char license_plate[4];  // 3 digits + null terminator
    uint8_t level;
    uint8_t slot;
} reserved_slot_t;

#define MAX_RESERVATIONS 40  // Maximum number of reservations
static reserved_slot_t reserved_slots[MAX_RESERVATIONS];
static uint8_t reserved_count = 0;

// Add a reservation
static void add_reservation(const char *license_plate, uint8_t level, uint8_t slot) {
    if (reserved_count < MAX_RESERVATIONS) {
        strcpy(reserved_slots[reserved_count].license_plate, license_plate);
        reserved_slots[reserved_count].level = level;
        reserved_slots[reserved_count].slot = slot;
        reserved_count++;
    }
}

// Find a reservation
static int find_reservation(const char *license_plate, uint8_t *level, uint8_t *slot) {
    for (uint8_t i = 0; i < reserved_count; i++) {
        if (strcmp(reserved_slots[i].license_plate, license_plate) == 0) {
            *level = reserved_slots[i].level;
            *slot = reserved_slots[i].slot;
            return 1;  // Found
        }
    }
    return 0;  // Not found
}


/* ------------------- Parking_task: parses commands --------------- */
static void parking_task(void){
    if(!pkt_valid) return;
    char cmd[PKT_MAX_SIZE+1]; 
    memcpy(cmd, pkt_body, pkt_bodysize); 
    cmd[pkt_bodysize] = 0;
    
    if(strcmp(cmd, "GO") == 0){ 
        is_running = 1; 
        init_parking_lot();  // Initialize parking lot state when starting
    }
    else if(strcmp(cmd, "END") == 0){ 
        is_running = 0; 
        pending_len = 0; 
    }
    else if(strncmp(cmd, "PRK", 3) == 0) {
        // Extract license plate number
        char license_plate[4];
        strncpy(license_plate, cmd + 3, 3);
        license_plate[3] = '\0';

        uint8_t level, slot;
        level=0; slot=0;
        if (find_reservation(license_plate, &level, &slot)) {
            // Park the car in the reserved spot
            parking_lot[level][slot].state = SLOT_OCCUPIED;
            strcpy(parking_lot[level][slot].license_plate, license_plate);
            
            // Send Parking Space Message
            char parking_message[12];
            sprintf(parking_message, "$SPC%s%c%02u#", license_plate, level_to_char(level), slot + 1);
            queue_msg(parking_message);
            empty_spaces--;  // Decrement empty spaces
            
        }else {
            level=0; slot=0;
            if (find_available_slot(&level, &slot)){
                // Park the car in the first available spot if no reservation
                parking_lot[level][slot].state = SLOT_OCCUPIED;
                strcpy(parking_lot[level][slot].license_plate, license_plate);
                char parking_message[12];
                sprintf(parking_message, "$SPC%s%c%02u#", license_plate, level_to_char(level), slot + 1);
                queue_msg(parking_message);
                empty_spaces--;  // Decrement empty spaces
            }
        }

    }
    else if(strncmp(cmd, "EXT", 3) == 0) {
        // Extract license plate number
        char license_plate[4];
        strncpy(license_plate, cmd + 3, 3);
        license_plate[3] = '\0';

        // Find the car in the parking lot
        uint8_t found = 0;
        for(uint8_t level = 0; level < LEVELS; level++) {
            for(uint8_t slot = 0; slot < SLOTS_PER; slot++) {
                if(parking_lot[level][slot].state == SLOT_OCCUPIED &&
                   strcmp(parking_lot[level][slot].license_plate, license_plate) == 0) {
                    // Calculate parking fee (example calculation)
                    uint16_t fee = 100;

                    // Update parking lot state
                    parking_lot[level][slot].state = SLOT_EMPTY;
                    empty_spaces++;  // Increment empty spaces

                    // If it was reserved, keep it reserved
                    if (find_reservation(license_plate, &level, &slot)) {
                        parking_lot[level][slot].state = SLOT_RESERVED;
                    } else {
                        parking_lot[level][slot].license_plate[0] = '\0';
                    }

                    // Send Parking Fee Message
                    char fee_message[12];
                    sprintf(fee_message, "$FEE%s%03u#", license_plate, fee);
                    queue_msg(fee_message);

                    found = 1;
                    break;
                }
            }
            if(found) break;
        }
    }
    else if(strncmp(cmd, "SUB", 3) == 0) {
        // Extract license plate number and parking space
        char license_plate[4];
        strncpy(license_plate, cmd + 3, 3);
        license_plate[3] = '\0';

        char level_char = cmd[6];
        uint8_t level = char_to_level(level_char);
        uint8_t slot = (cmd[7] - '0') * 10 + (cmd[8] - '0') - 1;  // Convert "01" to 0, "10" to 9

        // Check if the slot is available
        uint16_t fee = 0;
        if(parking_lot[level][slot].state == SLOT_EMPTY) {
            // Reserve the slot
            parking_lot[level][slot].state = SLOT_RESERVED;
            strcpy(parking_lot[level][slot].license_plate, license_plate);
            fee = 50;  // Subscription fee
        }
        
        add_reservation(license_plate, level, slot);

        // Send Reserved Message
        char reserved_message[11];
        sprintf(reserved_message, "$RES%s%02u#", license_plate, fee);
        queue_msg(reserved_message);
    }
    pkt_valid = 0;
}

/* ------------------- Output_task every 100?ms -------------------- */
static void output_task(void){
    if(!tick_100ms || !is_running) return;      /* only once per slot */
    tick_100ms=0;
    

    if(pending_len){ 
        // Send pending message
        disable_rxtx(); // Disable interrupts to safely access the buffer
        for(uint8_t i=0; i<pending_len; i++) {
            buf_push(pending_msg[i], OUTBUF);
        }
        enable_rxtx(); // Re-enable interrupts

        pending_len = 0; 
    } else { 
        // Send EMP message with current empty space count
        char emp_message[8];
        sprintf(emp_message, "$EMP%02u#", empty_spaces); // Format the message with the current empty spaces count
        
        disable_rxtx(); // Disable interrupts to safely access the buffer
        for (char *p = emp_message; *p; p++) {
            buf_push(*p, OUTBUF); // Push each character of the message into the buffer
        }
        enable_rxtx(); // Re-enable interrupts

    }    

    // Enable transmission if there is data in the buffer
    if(!PIE1bits.TX1IE && !buf_isempty(OUTBUF)){ 
        PIE1bits.TX1IE = 1; 
        TXREG1 = buf_pop(OUTBUF);
    }

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
    TXSTA1bits.TXEN=1; // Enable transmitter
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
