#include <stdint.h>
#include <Arduino.h>
/*****************************************************************************
 * Headers for type : POT
 *****************************************************************************/

// Definition of the instance struct:
struct POT_Instance {

// Instances of different sessions
bool active;
// Variables for the ID of the ports of the instance
uint16_t id_clock;
uint16_t id_potValue;
// Variables for the current instance state
int POT_POT_State;
// Variables for the properties of the instance
uint8_t POT_PIN_var;

};
// Declaration of prototypes outgoing messages :
void POT_POT_OnEntry(int state, struct POT_Instance *_instance);
void POT_handle_clock_ms25_tic(struct POT_Instance *_instance);
// Declaration of callbacks for incoming messages:
void register_POT_send_potValue_read_value_listener(void (*_listener)(struct POT_Instance *, uint32_t));
void register_external_POT_send_potValue_read_value_listener(void (*_listener)(struct POT_Instance *, uint32_t));

// Definition of the states:
#define POT_POT_READY_STATE 0
#define POT_POT_STATE 1


/*****************************************************************************
 * Headers for type : Base
 *****************************************************************************/

// Definition of the instance struct:
struct Base_Instance {

// Instances of different sessions
bool active;
// Variables for the ID of the ports of the instance
uint16_t id_timer;
uint16_t id_ledRed;
uint16_t id_pot;
// Variables for the current instance state
int Base_TrafficLights_State;
// Variables for the properties of the instance

};
// Declaration of prototypes outgoing messages :
void Base_TrafficLights_OnEntry(int state, struct Base_Instance *_instance);
void Base_handle_pot_read_value(struct Base_Instance *_instance, uint32_t val);
// Declaration of callbacks for incoming messages:
void register_Base_send_timer_timer_start_listener(void (*_listener)(struct Base_Instance *, uint8_t, uint32_t));
void register_external_Base_send_timer_timer_start_listener(void (*_listener)(struct Base_Instance *, uint8_t, uint32_t));
void register_Base_send_timer_timer_cancel_listener(void (*_listener)(struct Base_Instance *, uint8_t));
void register_external_Base_send_timer_timer_cancel_listener(void (*_listener)(struct Base_Instance *, uint8_t));
void register_Base_send_ledRed_led_ON_listener(void (*_listener)(struct Base_Instance *));
void register_external_Base_send_ledRed_led_ON_listener(void (*_listener)(struct Base_Instance *));
void register_Base_send_ledRed_led_OFF_listener(void (*_listener)(struct Base_Instance *));
void register_external_Base_send_ledRed_led_OFF_listener(void (*_listener)(struct Base_Instance *));

// Definition of the states:
#define BASE_TRAFFICLIGHTS_READY_STATE 0
#define BASE_TRAFFICLIGHTS_STATE 1


/*****************************************************************************
 * Headers for type : LED
 *****************************************************************************/

// Definition of the instance struct:
struct LED_Instance {

// Instances of different sessions
bool active;
// Variables for the ID of the ports of the instance
uint16_t id_ctrl;
// Variables for the current instance state
int LED_LED_State;
// Variables for the properties of the instance
uint8_t LED_PIN_var;

};
// Declaration of prototypes outgoing messages :
void LED_LED_OnEntry(int state, struct LED_Instance *_instance);
void LED_handle_ctrl_led_ON(struct LED_Instance *_instance);
void LED_handle_ctrl_led_OFF(struct LED_Instance *_instance);
// Declaration of callbacks for incoming messages:

// Definition of the states:
#define LED_LED_STATE 0
#define LED_LED_READY_STATE 1



struct timer2_instance_type {
    uint16_t listener_id;
    /*INSTANCE_INFORMATION*/
};
extern struct timer2_instance_type timer2_instance;

void timer2_setup();
void timer2_read();

//void forward_timer2_SoftButton_send_Timer_timer_start(struct SoftButton_Instance *_instance, uint8_t id, uint32_t time);
//void forward_timer2_SoftButton_send_Timer_timer_cancel(struct SoftButton_Instance *_instance, uint8_t id);
void forward_timer2_Base_send_timer_timer_start(struct Base_Instance *_instance, uint8_t id, uint32_t time);
void forward_timer2_Base_send_timer_timer_cancel(struct Base_Instance *_instance, uint8_t id);
// Definition of Enumeration  DigitalState
#define DIGITALSTATE_LOW 0
#define DIGITALSTATE_HIGH 1

// Definition of Enumeration  DigitalState
#define DIGITALSTATE_LOW 0
#define DIGITALSTATE_HIGH 1


/* Adds and instance to the runtime and returns its id */
uint16_t add_instance(void * instance_struct);
/* Returns the instance with id */
void * instance_by_id(uint16_t id);

/* Returns the number of byte currently in the fifo */
int fifo_byte_length();
/* Returns the number of bytes currently available in the fifo */
int fifo_byte_available();
/* Returns true if the fifo is empty */
int fifo_empty();
/* Return true if the fifo is full */
int fifo_full();
/* Enqueue 1 byte in the fifo if there is space
   returns 1 for sucess and 0 if the fifo was full */
int fifo_enqueue(byte b);
/* Enqueue 1 byte in the fifo without checking for available space
   The caller should have checked that there is enough space */
int _fifo_enqueue(byte b);
/* Dequeue 1 byte in the fifo.
   The caller should check that the fifo is not empty */
byte fifo_dequeue();

#define MAX_INSTANCES 8
#define FIFO_SIZE 256

/*********************************
 * Instance IDs and lookup
 *********************************/

void * instances[MAX_INSTANCES];
uint16_t instances_count = 0;

void * instance_by_id(uint16_t id) {
  return instances[id];
}

uint16_t add_instance(void * instance_struct) {
  instances[instances_count] = instance_struct;
  return instances_count++;
}

/******************************************
 * Simple byte FIFO implementation
 ******************************************/

byte fifo[FIFO_SIZE];
int fifo_head = 0;
int fifo_tail = 0;

// Returns the number of byte currently in the fifo
int fifo_byte_length() {
  if (fifo_tail >= fifo_head)
    return fifo_tail - fifo_head;
  return fifo_tail + FIFO_SIZE - fifo_head;
}

// Returns the number of bytes currently available in the fifo
int fifo_byte_available() {
  return FIFO_SIZE - 1 - fifo_byte_length();
}

// Returns true if the fifo is empty
int fifo_empty() {
  return fifo_head == fifo_tail;
}

// Return true if the fifo is full
int fifo_full() {
  return fifo_head == ((fifo_tail + 1) % FIFO_SIZE);
}

// Enqueue 1 byte in the fifo if there is space
// returns 1 for sucess and 0 if the fifo was full
int fifo_enqueue(byte b) {
  int new_tail = (fifo_tail + 1) % FIFO_SIZE;
  if (new_tail == fifo_head) return 0; // the fifo is full
  fifo[fifo_tail] = b;
  fifo_tail = new_tail;
  return 1;
}

// Enqueue 1 byte in the fifo without checking for available space
// The caller should have checked that there is enough space
int _fifo_enqueue(byte b) {
  fifo[fifo_tail] = b;
  fifo_tail = (fifo_tail + 1) % FIFO_SIZE;
  return 0; // Dummy added by steffend
}

// Dequeue 1 byte in the fifo.
// The caller should check that the fifo is not empty
byte fifo_dequeue() {
  if (!fifo_empty()) {
    byte result = fifo[fifo_head];
    fifo_head = (fifo_head + 1) % FIFO_SIZE;
    return result;
  }
  return 0;
}

/*****************************************************************************
 * Implementation for type : POT
 *****************************************************************************/

// Declaration of prototypes:
//Prototypes: State Machine
void POT_POT_OnExit(int state, struct POT_Instance *_instance);
//Prototypes: Message Sending
void POT_send_potValue_read_value(struct POT_Instance *_instance, uint32_t val);
//Prototypes: Function
void f_POT_getValue(struct POT_Instance *_instance, uint8_t pin);
// Declaration of functions:
// Definition of function getValue
void f_POT_getValue(struct POT_Instance *_instance, uint8_t pin) {
analogRead(pin);
}

// Sessions functionss:


// On Entry Actions:
void POT_POT_OnEntry(int state, struct POT_Instance *_instance) {
switch(state) {
case POT_POT_STATE:{
_instance->POT_POT_State = POT_POT_READY_STATE;
POT_POT_OnEntry(_instance->POT_POT_State, _instance);
break;
}
case POT_POT_READY_STATE:{
break;
}
default: break;
}
}

// On Exit Actions:
void POT_POT_OnExit(int state, struct POT_Instance *_instance) {
switch(state) {
case POT_POT_STATE:{
POT_POT_OnExit(_instance->POT_POT_State, _instance);
break;}
case POT_POT_READY_STATE:{
break;}
default: break;
}
}

// Event Handlers for incoming messages:
void POT_handle_clock_ms25_tic(struct POT_Instance *_instance) {
if(!(_instance->active)) return;
//Region POT
uint8_t POT_POT_State_event_consumed = 0;
if (_instance->POT_POT_State == POT_POT_READY_STATE) {
if (POT_POT_State_event_consumed == 0 && 1) {
POT_send_potValue_read_value(_instance, f_POT_getValue(_instance, _instance->POT_PIN_var));
POT_POT_State_event_consumed = 1;
}
}
//End Region POT
//End dsregion POT
//Session list: 
}

// Observers for outgoing messages:
void (*external_POT_send_potValue_read_value_listener)(struct POT_Instance *, uint32_t)= 0x0;
void (*POT_send_potValue_read_value_listener)(struct POT_Instance *, uint32_t)= 0x0;
void register_external_POT_send_potValue_read_value_listener(void (*_listener)(struct POT_Instance *, uint32_t)){
external_POT_send_potValue_read_value_listener = _listener;
}
void register_POT_send_potValue_read_value_listener(void (*_listener)(struct POT_Instance *, uint32_t)){
POT_send_potValue_read_value_listener = _listener;
}
void POT_send_potValue_read_value(struct POT_Instance *_instance, uint32_t val){
if (POT_send_potValue_read_value_listener != 0x0) POT_send_potValue_read_value_listener(_instance, val);
if (external_POT_send_potValue_read_value_listener != 0x0) external_POT_send_potValue_read_value_listener(_instance, val);
;
}



#define timer2_NB_SOFT_TIMER 4
uint32_t timer2_timer[timer2_NB_SOFT_TIMER];
uint32_t  timer2_prev_1sec = 0;

uint8_t timer2_tic_flags = 0;

void externalMessageEnqueue(uint8_t * msg, uint8_t msgSize, uint16_t listener_id);

uint8_t timer2_interrupt_counter = 0;
SIGNAL(TIMER2_OVF_vect) {
TCNT2 = 5;
timer2_interrupt_counter++;
if((timer2_interrupt_counter % 25) == 0) {
timer2_tic_flags |= 0b00000001;
}
if(timer2_interrupt_counter >= 25) {
timer2_interrupt_counter = 0;
}
}



//struct timer2_instance_type {
//    uint16_t listener_id;
//    /*INSTANCE_INFORMATION*/
//} timer2_instance;

struct timer2_instance_type timer2_instance;


void timer2_setup() {
	// Run timer2 interrupt up counting at 250kHz 
		 TCCR2A = 0;
		 TCCR2B = 1<<CS22 | 0<<CS21 | 0<<CS20;
		
		 //Timer2 Overflow Interrupt Enable
		 TIMSK2 |= 1<<TOIE2;


	timer2_prev_1sec = millis() + 1000;
}

void timer2_set_listener_id(uint16_t id) {
	timer2_instance.listener_id = id;
}

void timer2_timer_start(uint8_t id, uint32_t ms) {
if(id <timer2_NB_SOFT_TIMER) {
timer2_timer[id] = ms + millis();
}
}

void timer2_timer_cancel(uint8_t id) {
if(id <timer2_NB_SOFT_TIMER) {
timer2_timer[id] = 0;
}
}

void timer2_timeout(uint8_t id) {
uint8_t enqueue_buf[3];
enqueue_buf[0] = (1 >> 8) & 0xFF;
enqueue_buf[1] = 1 & 0xFF;
enqueue_buf[2] = id;
externalMessageEnqueue(enqueue_buf, 3, timer2_instance.listener_id);
}

void timer2_25ms_tic() {
{
uint8_t enqueue_buf[2];
enqueue_buf[0] = (2 >> 8) & 0xFF;
enqueue_buf[1] = 2 & 0xFF;
externalMessageEnqueue(enqueue_buf, 2, timer2_instance.listener_id);
}
}





void timer2_read() {
    uint32_t tms = millis();
    uint8_t t;
for(t = 0; t < 4; t++) {
if((timer2_timer[t] > 0) && (timer2_timer[t] < tms)) {
timer2_timer[t] = 0;
timer2_timeout(t);
}
}

    if (timer2_prev_1sec < tms) {
        timer2_prev_1sec += 1000;
    }
    if((timer2_tic_flags & 0b00000001) >> 0) {
timer2_25ms_tic();
timer2_tic_flags &= 0b11111110;
}

}
// Forwarding of messages timer2::Base::timer::timer_start
void forward_timer2_Base_send_timer_timer_start(struct Base_Instance *_instance, uint8_t id, uint32_t time){
timer2_timer_start(id, time);}

// Forwarding of messages timer2::Base::timer::timer_cancel
void forward_timer2_Base_send_timer_timer_cancel(struct Base_Instance *_instance, uint8_t id){
timer2_timer_cancel(id);}

/*****************************************************************************
 * Implementation for type : Base
 *****************************************************************************/

// Declaration of prototypes:
//Prototypes: State Machine
void Base_TrafficLights_OnExit(int state, struct Base_Instance *_instance);
//Prototypes: Message Sending
void Base_send_timer_timer_start(struct Base_Instance *_instance, uint8_t id, uint32_t time);
void Base_send_timer_timer_cancel(struct Base_Instance *_instance, uint8_t id);
void Base_send_ledRed_led_ON(struct Base_Instance *_instance);
void Base_send_ledRed_led_OFF(struct Base_Instance *_instance);
//Prototypes: Function
// Declaration of functions:

// Sessions functionss:


// On Entry Actions:
void Base_TrafficLights_OnEntry(int state, struct Base_Instance *_instance) {
switch(state) {
case BASE_TRAFFICLIGHTS_STATE:{
_instance->Base_TrafficLights_State = BASE_TRAFFICLIGHTS_READY_STATE;
Base_TrafficLights_OnEntry(_instance->Base_TrafficLights_State, _instance);
break;
}
case BASE_TRAFFICLIGHTS_READY_STATE:{
Base_send_ledRed_led_OFF(_instance);
break;
}
default: break;
}
}

// On Exit Actions:
void Base_TrafficLights_OnExit(int state, struct Base_Instance *_instance) {
switch(state) {
case BASE_TRAFFICLIGHTS_STATE:{
Base_TrafficLights_OnExit(_instance->Base_TrafficLights_State, _instance);
break;}
case BASE_TRAFFICLIGHTS_READY_STATE:{
break;}
default: break;
}
}

// Event Handlers for incoming messages:
void Base_handle_pot_read_value(struct Base_Instance *_instance, uint32_t val) {
if(!(_instance->active)) return;
//Region TrafficLights
uint8_t Base_TrafficLights_State_event_consumed = 0;
if (_instance->Base_TrafficLights_State == BASE_TRAFFICLIGHTS_READY_STATE) {
if (Base_TrafficLights_State_event_consumed == 0 && 1) {
if(val > 100) {
Base_send_ledRed_led_ON(_instance);

}
Base_TrafficLights_State_event_consumed = 1;
}
}
//End Region TrafficLights
//End dsregion TrafficLights
//Session list: 
}

// Observers for outgoing messages:
void (*external_Base_send_timer_timer_start_listener)(struct Base_Instance *, uint8_t, uint32_t)= 0x0;
void (*Base_send_timer_timer_start_listener)(struct Base_Instance *, uint8_t, uint32_t)= 0x0;
void register_external_Base_send_timer_timer_start_listener(void (*_listener)(struct Base_Instance *, uint8_t, uint32_t)){
external_Base_send_timer_timer_start_listener = _listener;
}
void register_Base_send_timer_timer_start_listener(void (*_listener)(struct Base_Instance *, uint8_t, uint32_t)){
Base_send_timer_timer_start_listener = _listener;
}
void Base_send_timer_timer_start(struct Base_Instance *_instance, uint8_t id, uint32_t time){
if (Base_send_timer_timer_start_listener != 0x0) Base_send_timer_timer_start_listener(_instance, id, time);
if (external_Base_send_timer_timer_start_listener != 0x0) external_Base_send_timer_timer_start_listener(_instance, id, time);
;
}
void (*external_Base_send_timer_timer_cancel_listener)(struct Base_Instance *, uint8_t)= 0x0;
void (*Base_send_timer_timer_cancel_listener)(struct Base_Instance *, uint8_t)= 0x0;
void register_external_Base_send_timer_timer_cancel_listener(void (*_listener)(struct Base_Instance *, uint8_t)){
external_Base_send_timer_timer_cancel_listener = _listener;
}
void register_Base_send_timer_timer_cancel_listener(void (*_listener)(struct Base_Instance *, uint8_t)){
Base_send_timer_timer_cancel_listener = _listener;
}
void Base_send_timer_timer_cancel(struct Base_Instance *_instance, uint8_t id){
if (Base_send_timer_timer_cancel_listener != 0x0) Base_send_timer_timer_cancel_listener(_instance, id);
if (external_Base_send_timer_timer_cancel_listener != 0x0) external_Base_send_timer_timer_cancel_listener(_instance, id);
;
}
void (*external_Base_send_ledRed_led_ON_listener)(struct Base_Instance *)= 0x0;
void (*Base_send_ledRed_led_ON_listener)(struct Base_Instance *)= 0x0;
void register_external_Base_send_ledRed_led_ON_listener(void (*_listener)(struct Base_Instance *)){
external_Base_send_ledRed_led_ON_listener = _listener;
}
void register_Base_send_ledRed_led_ON_listener(void (*_listener)(struct Base_Instance *)){
Base_send_ledRed_led_ON_listener = _listener;
}
void Base_send_ledRed_led_ON(struct Base_Instance *_instance){
if (Base_send_ledRed_led_ON_listener != 0x0) Base_send_ledRed_led_ON_listener(_instance);
if (external_Base_send_ledRed_led_ON_listener != 0x0) external_Base_send_ledRed_led_ON_listener(_instance);
;
}
void (*external_Base_send_ledRed_led_OFF_listener)(struct Base_Instance *)= 0x0;
void (*Base_send_ledRed_led_OFF_listener)(struct Base_Instance *)= 0x0;
void register_external_Base_send_ledRed_led_OFF_listener(void (*_listener)(struct Base_Instance *)){
external_Base_send_ledRed_led_OFF_listener = _listener;
}
void register_Base_send_ledRed_led_OFF_listener(void (*_listener)(struct Base_Instance *)){
Base_send_ledRed_led_OFF_listener = _listener;
}
void Base_send_ledRed_led_OFF(struct Base_Instance *_instance){
if (Base_send_ledRed_led_OFF_listener != 0x0) Base_send_ledRed_led_OFF_listener(_instance);
if (external_Base_send_ledRed_led_OFF_listener != 0x0) external_Base_send_ledRed_led_OFF_listener(_instance);
;
}



/*****************************************************************************
 * Implementation for type : LED
 *****************************************************************************/

// Declaration of prototypes:
//Prototypes: State Machine
void LED_LED_OnExit(int state, struct LED_Instance *_instance);
//Prototypes: Message Sending
//Prototypes: Function
void f_LED_setDigitalOutput(struct LED_Instance *_instance, uint8_t pin);
void f_LED_digitalWrite(struct LED_Instance *_instance, uint8_t pin, uint8_t value);
// Declaration of functions:
// Definition of function setDigitalOutput
void f_LED_setDigitalOutput(struct LED_Instance *_instance, uint8_t pin) {
pinMode(pin, OUTPUT);
}
// Definition of function digitalWrite
void f_LED_digitalWrite(struct LED_Instance *_instance, uint8_t pin, uint8_t value) {
digitalWrite(pin, value);
}

// Sessions functionss:


// On Entry Actions:
void LED_LED_OnEntry(int state, struct LED_Instance *_instance) {
switch(state) {
case LED_LED_STATE:{
_instance->LED_LED_State = LED_LED_READY_STATE;
f_LED_setDigitalOutput(_instance, _instance->LED_PIN_var);
LED_LED_OnEntry(_instance->LED_LED_State, _instance);
break;
}
case LED_LED_READY_STATE:{
break;
}
default: break;
}
}

// On Exit Actions:
void LED_LED_OnExit(int state, struct LED_Instance *_instance) {
switch(state) {
case LED_LED_STATE:{
LED_LED_OnExit(_instance->LED_LED_State, _instance);
break;}
case LED_LED_READY_STATE:{
break;}
default: break;
}
}

// Event Handlers for incoming messages:
void LED_handle_ctrl_led_ON(struct LED_Instance *_instance) {
if(!(_instance->active)) return;
//Region LED
uint8_t LED_LED_State_event_consumed = 0;
if (_instance->LED_LED_State == LED_LED_READY_STATE) {
if (LED_LED_State_event_consumed == 0 && 1) {
f_LED_digitalWrite(_instance, _instance->LED_PIN_var, DIGITALSTATE_HIGH);
LED_LED_State_event_consumed = 1;
}
}
//End Region LED
//End dsregion LED
//Session list: 
}
void LED_handle_ctrl_led_OFF(struct LED_Instance *_instance) {
if(!(_instance->active)) return;
//Region LED
uint8_t LED_LED_State_event_consumed = 0;
if (_instance->LED_LED_State == LED_LED_READY_STATE) {
if (LED_LED_State_event_consumed == 0 && 1) {
f_LED_digitalWrite(_instance, _instance->LED_PIN_var, DIGITALSTATE_LOW);
LED_LED_State_event_consumed = 1;
}
}
//End Region LED
//End dsregion LED
//Session list: 
}

// Observers for outgoing messages:






/*****************************************************************************
 * Definitions for configuration : Base
 *****************************************************************************/

//Declaration of instance variables
//Instance Base
// Variables for the properties of the instance
struct Base_Instance Base_var;
// Variables for the sessions of the instance
//Instance pot
// Variables for the properties of the instance
struct POT_Instance pot_var;
// Variables for the sessions of the instance
//Instance ledRed
// Variables for the properties of the instance
struct LED_Instance ledRed_var;
// Variables for the sessions of the instance


// Enqueue of messages Base::ledRed::led_ON
void enqueue_Base_send_ledRed_led_ON(struct Base_Instance *_instance){
if ( fifo_byte_available() > 4 ) {

_fifo_enqueue( (3 >> 8) & 0xFF );
_fifo_enqueue( 3 & 0xFF );

// ID of the source port of the instance
_fifo_enqueue( (_instance->id_ledRed >> 8) & 0xFF );
_fifo_enqueue( _instance->id_ledRed & 0xFF );
}
}
// Enqueue of messages Base::ledRed::led_OFF
void enqueue_Base_send_ledRed_led_OFF(struct Base_Instance *_instance){
if ( fifo_byte_available() > 4 ) {

_fifo_enqueue( (4 >> 8) & 0xFF );
_fifo_enqueue( 4 & 0xFF );

// ID of the source port of the instance
_fifo_enqueue( (_instance->id_ledRed >> 8) & 0xFF );
_fifo_enqueue( _instance->id_ledRed & 0xFF );
}
}


//New dispatcher for messages
void dispatch_timer_timeout(uint16_t sender, uint8_t param_id) {
if (sender == timer2_instance.listener_id) {

}

}


//New dispatcher for messages
void dispatch_led_ON(uint16_t sender) {
if (sender == Base_var.id_ledRed) {
LED_handle_ctrl_led_ON(&ledRed_var);

}

}


//New dispatcher for messages
void dispatch_read_value(uint16_t sender, uint32_t param_val) {
if (sender == pot_var.id_potValue) {
Base_handle_pot_read_value(&Base_var, param_val);

}

}

void sync_dispatch_POT_send_potValue_read_value(struct POT_Instance *_instance, uint32_t val){
dispatch_read_value(_instance->id_potValue, val);
}

//New dispatcher for messages
void dispatch_led_OFF(uint16_t sender) {
if (sender == Base_var.id_ledRed) {
LED_handle_ctrl_led_OFF(&ledRed_var);

}

}


//New dispatcher for messages
void dispatch_ms25_tic(uint16_t sender) {
if (sender == timer2_instance.listener_id) {
POT_handle_clock_ms25_tic(&pot_var);

}

}


int processMessageQueue() {
if (fifo_empty()) return 0; // return 0 if there is nothing to do

uint8_t mbufi = 0;

// Read the code of the next port/message in the queue
uint16_t code = fifo_dequeue() << 8;

code += fifo_dequeue();

// Switch to call the appropriate handler
switch(code) {
case 3:{
byte mbuf[4 - 2];
while (mbufi < (4 - 2)) mbuf[mbufi++] = fifo_dequeue();
uint8_t mbufi_led_ON = 2;
dispatch_led_ON((mbuf[0] << 8) + mbuf[1] /* instance port*/);
break;
}
case 1:{
byte mbuf[5 - 2];
while (mbufi < (5 - 2)) mbuf[mbufi++] = fifo_dequeue();
uint8_t mbufi_timer_timeout = 2;
union u_timer_timeout_id_t {
uint8_t p;
byte bytebuffer[1];
} u_timer_timeout_id;
u_timer_timeout_id.bytebuffer[0] = mbuf[mbufi_timer_timeout + 0];
mbufi_timer_timeout += 1;
dispatch_timer_timeout((mbuf[0] << 8) + mbuf[1] /* instance port*/,
 u_timer_timeout_id.p /* id */ );
break;
}
case 4:{
byte mbuf[4 - 2];
while (mbufi < (4 - 2)) mbuf[mbufi++] = fifo_dequeue();
uint8_t mbufi_led_OFF = 2;
dispatch_led_OFF((mbuf[0] << 8) + mbuf[1] /* instance port*/);
break;
}
case 2:{
byte mbuf[4 - 2];
while (mbufi < (4 - 2)) mbuf[mbufi++] = fifo_dequeue();
uint8_t mbufi_ms25_tic = 2;
dispatch_ms25_tic((mbuf[0] << 8) + mbuf[1] /* instance port*/);
break;
}
}
return 1;
}

void forward_Base_send_timer_timer_cancel(struct Base_Instance *_instance, uint8_t id){
if(_instance->id_timer == Base_var.id_timer) {
forward_timer2_Base_send_timer_timer_cancel(_instance, id);
}
}
void forward_Base_send_timer_timer_start(struct Base_Instance *_instance, uint8_t id, uint32_t time){
if(_instance->id_timer == Base_var.id_timer) {
forward_timer2_Base_send_timer_timer_start(_instance, id, time);
}
}

//external Message enqueue
void externalMessageEnqueue(uint8_t * msg, uint8_t msgSize, uint16_t listener_id) {
if ((msgSize >= 2) && (msg != NULL)) {
uint8_t msgSizeOK = 0;
switch(msg[0] * 256 + msg[1]) {
case 1:
if(msgSize == 3) {
msgSizeOK = 1;}
break;
case 2:
if(msgSize == 2) {
msgSizeOK = 1;}
break;
}

if(msgSizeOK == 1) {
if ( fifo_byte_available() > (msgSize + 2) ) {
	uint8_t i;
	for (i = 0; i < 2; i++) {
		_fifo_enqueue(msg[i]);
	}
	_fifo_enqueue((listener_id >> 8) & 0xFF);
	_fifo_enqueue(listener_id & 0xFF);
	for (i = 2; i < msgSize; i++) {
		_fifo_enqueue(msg[i]);
	}
}
}
}
}

void initialize_configuration_Base() {
// Initialize connectors
register_external_Base_send_timer_timer_start_listener(&forward_Base_send_timer_timer_start);
register_external_Base_send_timer_timer_cancel_listener(&forward_Base_send_timer_timer_cancel);
register_Base_send_ledRed_led_ON_listener(&enqueue_Base_send_ledRed_led_ON);
register_Base_send_ledRed_led_OFF_listener(&enqueue_Base_send_ledRed_led_OFF);
register_POT_send_potValue_read_value_listener(&sync_dispatch_POT_send_potValue_read_value);

// Init the ID, state variables and properties for external connector timer2
// Init the ID, state variables and properties for external connector timer2

// Network Initialization

timer2_instance.listener_id = add_instance(&timer2_instance);

timer2_setup();

// End Network Initialization

// Init the ID, state variables and properties for instance pot
pot_var.active = true;
pot_var.id_clock = add_instance( (void*) &pot_var);
pot_var.id_potValue = add_instance( (void*) &pot_var);
pot_var.POT_POT_State = POT_POT_READY_STATE;
pot_var.POT_PIN_var = 16;

POT_POT_OnEntry(POT_POT_STATE, &pot_var);
// Init the ID, state variables and properties for instance ledRed
ledRed_var.active = true;
ledRed_var.id_ctrl = add_instance( (void*) &ledRed_var);
ledRed_var.LED_LED_State = LED_LED_READY_STATE;
ledRed_var.LED_PIN_var = 8;

LED_LED_OnEntry(LED_LED_STATE, &ledRed_var);
// Init the ID, state variables and properties for instance Base
Base_var.active = true;
Base_var.id_timer = add_instance( (void*) &Base_var);
Base_var.id_ledRed = add_instance( (void*) &Base_var);
Base_var.id_pot = add_instance( (void*) &Base_var);
Base_var.Base_TrafficLights_State = BASE_TRAFFICLIGHTS_READY_STATE;

Base_TrafficLights_OnEntry(BASE_TRAFFICLIGHTS_STATE, &Base_var);
}




void setup() {
initialize_configuration_Base();

}

void loop() {

// Network Listener
timer2_read();
// End Network Listener

int emptyEventConsumed = 1;
while (emptyEventConsumed != 0) {
emptyEventConsumed = 0;
}

    processMessageQueue();
}
