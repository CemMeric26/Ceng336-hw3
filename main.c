#include "pragmas.h"
#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define _XTAL_FREQ   40000000UL
#define TMR0_RELOAD  64286  // value for 1ms tick with 1:8 prescaler

/* ---------- UART buffer---------- */
#define BUFSIZE 128
static uint8_t inbuf[BUFSIZE], outbuf[BUFSIZE];
static uint8_t head[2] = {0,0}, tail[2] = {0,0};
typedef enum {INBUF = 0, OUTBUF = 1} buf_t;

uint8_t is_running=0;
uint8_t tick_100ms=0;
uint8_t tick_500ms=0;
uint8_t ms100=0;
uint16_t ms500=0;

// Add global variable for current time in milliseconds
uint32_t now_ms = 0;


static unsigned char digits[4];
uint8_t display_mode = 0; // 0 for total money, 1 for empty spaces

uint8_t current_level = 0;

uint16_t adc_value = 0;

static inline void disable_rxtx(void){ PIE1bits.RC1IE = 0; PIE1bits.TX1IE = 0; }
static inline void enable_rxtx (void){ PIE1bits.RC1IE = 1; PIE1bits.TX1IE = 1; }
static uint8_t buf_isempty(buf_t b){ return head[b]==tail[b]; }
static void buf_push(uint8_t v, buf_t b){ if(b==INBUF) inbuf[head[b]]=v; else outbuf[head[b]]=v; if(++head[b]==BUFSIZE) head[b]=0;}
static uint8_t buf_pop(buf_t b){ if (buf_isempty(b)) {return 0; } else { uint8_t v=(b==INBUF)?inbuf[tail[b]]:outbuf[tail[b]]; if(++tail[b]==BUFSIZE) tail[b]=0; return v; } }


static void uart_isr(void){
    if(PIR1bits.RC1IF){ 
        buf_push(RCREG1,INBUF); 
        PIR1bits.RC1IF=0;  // Clear interrupt flag
    }
    if(PIR1bits.TX1IF){
        if(!buf_isempty(OUTBUF)){
            TXREG1=buf_pop(OUTBUF);
            PIR1bits.TX1IF=0; // Clear interrupt flag
        } else {
            PIE1bits.TX1IE=0;
            PIR1bits.TX1IF=0;
        } 
    }
}

static void adc_isr(void){
    if(PIR1bits.ADIF){  
        PIR1bits.ADIF=0;  // Clear interrupt flag
        adc_value = (ADRESH <<8 )| ADRESL;
    }
}

void __interrupt(high_priority) isr_high(void){
    if (INTCONbits.RBIF) { // Check if PORTB change interrupt
        if(PORTBbits.RB4 == 1){
            display_mode ^= 1; // Toggle display mode
        }
        INTCONbits.RBIF = 0; // Clear interrupt flag
    }
    
    if(INTCONbits.TMR0IF)
    { 
        INTCONbits.TMR0IF=0; TMR0=TMR0_RELOAD;
        now_ms++;  // Increment the millisecond counter
        if(++ms100==100){ ms100=0; tick_100ms=1;}
        if(++ms500==500){ ms500=0; tick_500ms=1;}// Start ADC conversion every 500ms 
    }
    
    uart_isr(); 
    adc_isr();
}

/* ---------------- Packet framing task-------------------- */
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

/* ------------------- Parking lot data------------------ */
#define LEVELS 4
#define SLOTS_PER 10
#define TOTAL_SLOTS 40 // 40 total slots
// Add global variable for total money
uint32_t total_money = 0;

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
    uint32_t in_ms;
    uint8_t subscribed;
} parking_slot_t;

// Global parking lot state
static parking_slot_t parking_lot[LEVELS][SLOTS_PER];
static uint16_t empty_spaces = TOTAL_SLOTS;  // Initially all 40 spaces are empty

#define MAX_MESSAGES 40
#define MAX_MESSAGE_LENGTH 16

static char message_queue[MAX_MESSAGES][MAX_MESSAGE_LENGTH];
static uint8_t message_queue_head = 0, message_queue_tail = 0;

static void queue_msg(const char *s) {
    uint8_t next_tail = (message_queue_tail + 1) % MAX_MESSAGES;
    if (next_tail != message_queue_head) { // Check if the queue is not full
        strncpy(message_queue[message_queue_tail], s, MAX_MESSAGE_LENGTH - 1);
        message_queue[message_queue_tail][MAX_MESSAGE_LENGTH - 1] = '\0'; // Ensure null termination
        message_queue_tail = next_tail;
    }
}

// Helper functions for parking lot management
static void init_parking_lot(void) {
    for(uint8_t level = 0; level < LEVELS; level++) {
        for(uint8_t slot = 0; slot < SLOTS_PER; slot++) {
            parking_lot[level][slot].state = SLOT_EMPTY;
            parking_lot[level][slot].license_plate[0] = '\0';
            parking_lot[level][slot].in_ms = 0;
            parking_lot[level][slot].subscribed = 0;
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

// Find first available slot
static uint8_t find_empty_slot(uint8_t *level, uint8_t *slot) {
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

// Find first available reserved slot
static uint8_t find_reserved_slot(uint8_t *level, uint8_t *slot, char *license_plate) {
    for(uint8_t l = 0; l < LEVELS; l++) {
        for(uint8_t s = 0; s < SLOTS_PER; s++) {
            if(parking_lot[l][s].state == SLOT_RESERVED && (strcmp(parking_lot[l][s].license_plate, license_plate) == 0)) {
                *level = l;
                *slot = s;
                return 1;  // Found
            }
        }
    }
    
    return 0;  // No available slot
}

// Find first available slot
static uint8_t find_available_slot(uint8_t *level, uint8_t *slot, char *license_plate) {
    return find_reserved_slot(&level, &slot, license_plate) || find_empty_slot(&level, &slot);  // No available slot
}

// Helper function to calculate empty spaces per level
static uint8_t empty_spaces_per_level(uint8_t level) {
    uint8_t empty_count = 0;
    for (uint8_t slot = 0; slot < SLOTS_PER; slot++) {
        if (parking_lot[level][slot].state != SLOT_OCCUPIED) {
            empty_count++;
        }
    }
    return empty_count;
}

typedef struct {
    char license_plate[4];  // 3 digits + null terminator
    uint8_t level;
    uint8_t slot;
} reserved_slot_t;

// Define a simple queue for cars
typedef struct {
    char license_plate[4];
} car_t;

#define QUEUE_SIZE 16
static car_t car_queue[QUEUE_SIZE];
static uint8_t queue_head = 0, queue_tail = 0;
static uint8_t queue_count= 0;

// Enqueue a car
static void enqueue_car(const char *license_plate) {
    if (queue_count < 16) { // Check if queue is not full
        strcpy(car_queue[queue_tail].license_plate, license_plate);
        queue_tail = (queue_tail + 1) % QUEUE_SIZE;
        queue_count++;
    }
}

// Dequeue a car
static int dequeue_car() {
    if (queue_count != 0) { // Check if queue is not empty
        queue_head = (queue_head + 1) % QUEUE_SIZE;
        queue_count--;
        return 1; // Success
    }
    return 0; // Queue is empty
}

// Show the front car
static int front_car(char *license_plate) {
    if (queue_count != 0) { // Check if queue is not empty
        strcpy(license_plate, car_queue[queue_head].license_plate);
        return 1; // Success
    }
    return 0; // Queue is empty
}

// Attempt to park a car from the queue
static void try_park_from_queue(void) {
    char license_plate[4];
    uint8_t level, slot;
    level=0; slot=0;
    if (front_car(license_plate) && find_empty_slot(&level, &slot)) {
        // Park the car in the first available spot
        dequeue_car();
        parking_lot[level][slot].state = SLOT_OCCUPIED;
        strcpy(parking_lot[level][slot].license_plate, license_plate);
        parking_lot[level][slot].in_ms = now_ms;
                
        char parking_message[12];
        sprintf(parking_message, "$SPC%s%c%02u#", license_plate, level_to_char(level), slot + 1);
        queue_msg(parking_message);
        empty_spaces--;
    }
}

//parses commands
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
        // check what should we do here
    }
    else if(strncmp(cmd, "PRK", 3) == 0) {
        // Extract license plate number
        char license_plate[4];
        strncpy(license_plate, cmd + 3, 3);
        license_plate[3] = '\0';
        uint8_t level, slot;
        level=0; slot=0;
        if(find_reserved_slot(&level, &slot, license_plate)){
            // Park the car in the reserved spot
            parking_lot[level][slot].state = SLOT_OCCUPIED;                
            char parking_message[12];
            sprintf(parking_message, "$SPC%s%c%02u#", license_plate, level_to_char(level), slot + 1);
            parking_lot[level][slot].in_ms = now_ms;
            queue_msg(parking_message);
            empty_spaces--;
        }
        else{
            enqueue_car(license_plate);
            try_park_from_queue();
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

                    // Calculate parking fee
                    uint32_t dt = now_ms - parking_lot[level][slot].in_ms;
                    uint16_t fee = parking_lot[level][slot].subscribed ? 0 : (uint16_t)((dt) / 250 + 1);

                    // Queue Parking Fee Message
                    char fee_message[12];
                    total_money += fee;
                    sprintf(fee_message, "$FEE%s%03u#", license_plate, fee);
                    queue_msg(fee_message);
                    empty_spaces++;


                    if (parking_lot[level][slot].subscribed) {
                        parking_lot[level][slot].state = SLOT_RESERVED;
                    } else {
                        parking_lot[level][slot].license_plate[0] = '\0';
                        parking_lot[level][slot].state = SLOT_EMPTY;
                    }

                    found = 1;
                    break;
                }
            }
            if(found) break;
        }

        if(queue_count>0) try_park_from_queue();
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
        uint8_t fee = 0;
        if(parking_lot[level][slot].state == SLOT_EMPTY) {
            // Reserve the slot
            parking_lot[level][slot].state = SLOT_RESERVED;
            strcpy(parking_lot[level][slot].license_plate, license_plate);
            parking_lot[level][slot].subscribed = 1;
            fee = 50;
            total_money += fee;  // Subscription fee
        }
        
        // Send Reserved Message
        char reserved_message[11];
        sprintf(reserved_message, "$RES%s%02u#", license_plate, fee);
        queue_msg(reserved_message);
    }
    pkt_valid = 0;
}

/* ------------------- Output_task every 100 ms -------------------- */
static void output_task(void){
    if(!tick_100ms || !is_running) return;
    tick_100ms=0;
    
    if (message_queue_head != message_queue_tail) { // Check if there are messages in the queue
        // Send the message at the head of the queue
        disable_rxtx(); // Disable interrupts to safely access the buffer
        for (char *p = message_queue[message_queue_head]; *p; p++) {
            buf_push(*p, OUTBUF);
        }
        enable_rxtx(); // Re-enable interrupts

        // Move to the next message
        message_queue_head = (message_queue_head + 1) % MAX_MESSAGES;
    } else {
        // Send EMP message with current empty space count if no other message is pending
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

// Update the 7-segment display
void find_digits(unsigned int score) {
    digits[0] = score % 10; 
    digits[1] = (score / 10) % 10; 
    digits[2] = (score / 100) % 10;
    digits[3] = (score / 1000) % 10; 
}

static void hw_init(void){
    TRISB = 0x10; // Set RB4 as input, others as output
    LATB = 0x00;
    
    //TRISA = 0x00; // FOR DEBUG
    //LATA = 0x00;

    TRISH = 0x00;
    TRISJ = 0x00;
    LATH = 0x00;
    LATJ = 0x00;
    
    /* Timer0 */ T0CON = 0b00000010; TMR0=TMR0_RELOAD; INTCONbits.TMR0IE=1; T0CONbits.TMR0ON=1;
    INTCONbits.RBIE = 1; // Enable PORTB change interrupt
    INTCONbits.RBIF = 0;
    
    
    
    TRISHbits.TRISH4 = 1;            // AN12 pin input
    ADCON1bits.PCFG  = 0b0011;       // AN0-12 analog
    ADCON0bits.CHS   = 0b1100;       // Channel = AN12
    ADCON2bits.ADFM  = 1;            // Right-justify
    ADCON2bits.ACQT  = 0b100;        // 8 \D7 TAD acquisition
    ADCON2bits.ADCS  = 0b010;        // TAD = 4 \D7 TOSC

    PIE1bits.ADIE    = 1;            // enable interrupt
    PIR1bits.ADIF = 0;

    ADCON0bits.ADON  = 1;



    /* UART 115200 */ TXSTA1bits.SYNC=0; TXSTA1bits.BRGH=1; BAUDCON1bits.BRG16=0; SPBRG1=21; RCSTA1bits.CREN=1; RCSTA1bits.SPEN=1;
    TXSTA1bits.TXEN=1; // Enable transmitter
    enable_rxtx(); INTCONbits.PEIE=1; INTCONbits.GIE=1;

}


// Segment mapping for digits 0-9
const unsigned char segment_map[10] = {
    0b00111111, // 0
    0b00000110, // 1
    0b01011011, // 2
    0b01001111, // 3
    0b01100110, // 4
    0b01101101, // 5
    0b01111101, // 6
    0b00000111, // 7
    0b01111111, // 8
    0b01101111  // 9
};

// Segment mapping for digits 0-9
unsigned char map_adc() {
    if(adc_value<256)return 0;
    else if (adc_value<512)return 1;
    else if (adc_value<768)return 2;
    else return 3;
}

void display_digits(){
    for (int i = 0; i < 4; i++) {
        LATH = 0b00001000 >> i; // determine digit
        LATJ = segment_map[digits[i]]; // Set the digit visual
        __delay_us(500); // no flicker please
    }
}

// Function to update the display based on the current mode
void update_display() {
    if (display_mode == 0) {
        find_digits(total_money);
    } else {
        current_level = map_adc();
        find_digits(empty_spaces_per_level(current_level));
    }
    display_digits();
}



/* ------------------- MAIN ---------------------------------------- */
void main(void){
    hw_init();
    while(1){
        packet_task();       /* Frame incoming stream */
        parking_task();      /* Process commands & queue messages */
        output_task();       /* Send exactly one every 100ms */
        if(is_running) update_display();    /* Update the 7-segment display */

        if(tick_500ms){
            tick_500ms = 0;
            if(ADCON0bits.GO==0) ADCON0bits.GO=1;
        }
    }
}