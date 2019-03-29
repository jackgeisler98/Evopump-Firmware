#include "app_timer.h"
#include "boards.h"
#include "bsp_btn_ble.h"

///extern bool bolus_delivery;
extern uint32_t  delivery_rate_ct;


//for time, appendix _ct is the counter version.
//try to minimize float number use, convert to int
#define PUMP_NUMBER 4                                                   //number of pumps
#define PUMP_VOLUME 250                                                 //Usable volume each pump
#define TOTAL_VOLUME PUMP_NUMBER*PUMP_VOLUME                            //Total usable volume
#define MIN_BOLUS_RATE    1                                             //convert to counts later, unit uL or 0.1 U of U100
#define MAX_BOLUS_RATE    300                                           //convert to counts later, unit uL or 30 U of U100
#define BOLUS_INCREMENT   0.5f                                          //convert to counts later, unit uL or 0.05 U of U100, could be three tiers later
#define MIN_BASAL_RATE    0.25f                                         //convert to counts later, unit uL/hr or 0.025 U/hr of U100, --higher std than OmniPod, 0.5
#define MAX_BASAL_RATE    250                                           //convert to counts later, unit uL/hr or 25 U/hr of U100
#define BASAL_INCREMENT   0.25f                                         //convert to counts later, unit uL/hr or 0.025 U/hr of U100, --higher std than OmniPod, 0.5
#define MIN_SINGLE_DELIVERY 0.125f                                      //minimum single delivery used for ct unit.
#define MIN_TEMP          3                                             //convert to counts later, unit degree
#define MAX_TEMP          40                                            //convert to counts later, unit degree
#define MIN_PRESSURE      70000                                         //convert to counts later, unit Pa
#define MAX_PRESSURE      120000                                        //convert to counts later, unit Pa
#define MAX_BASAL_INTERVAL  1800                                        //unit s, maximum time gap between two basal runs.
static float SHDN_INTERVAL = 50;                                        //define pumping on off frequency, unit ms, here 50 ms on 50 ms off
#define SHDN_PIN 			6                               //define GPIO ports for SHDN pin control
#define BUZZER_PIN                      8                               //define GPIO port for buzzer control, check every adv_counter time 30 s
#define SIXMA_PIN                       34                              //define GPIO port for 6 mA level current P1.02
#define TWOMA_PIN                       35                              //define GPIO port for 2 mA level current P1.03
#define MULTIPLEXER_A0                  36                              //define GPIO port for ADG708 Multiplexer A0, P1.04
#define MULTIPLEXER_A1                  37                              //define GPIO port for ADG708 Multiplexer A1, P1.05
#define MULTIPLEXER_A2                  38                              //define GPIO port for ADG708 Multiplexer A2, P1.06
#define START_PUMP                      0                               //define the start pump number, from 0 to END_PUMP
#define END_PUMP                        7                               //END pump number, total pump number is end-start+1. //current simplified version for same test continuously only
#define GAS_COMP_RATE                   0.03f                           //unit uL/min for now, float

// Timer interval
#define SENSOR_INTERVAL         APP_TIMER_TICKS(5000)                   //Update sensor information every 5 s, including basal info
#define DELIVERY_INTERVAL          APP_TIMER_TICKS(60000)               //Update delivery info every min
#define TARGET 250 //define target total dosage
#define SHDN_PIN 			6         //define GPIO ports for SHDN pin control
#define BUZZER_PIN                      8         //define GPIO port for buzzer control, check every adv_counter time 30 s
#define SIXMA_PIN                       34        //define GPIO port for 6 mA level current P1.02
#define TWOMA_PIN                       35        //define GPIO port for 2 mA level current P1.03
#define MULTIPLEXER_A0                  36        //define GPIO port for ADG708 Multiplexer A0, P1.04
#define MULTIPLEXER_A1                  37        //define GPIO port for ADG708 Multiplexer A1, P1.05
#define MULTIPLEXER_A2                  38        //define GPIO port for ADG708 Multiplexer A2, P1.06

static uint16_t basal_time_interval_ct_update(uint16_t basal_rate_ct);
static float pumped_volumes[PUMP_NUMBER]={0.0};                                                            //initilize
static uint8_t current_pump = START_PUMP;                                                                  //For now, pump in series, pump all out of one pump and move to the next
static uint32_t shdn_counter=0;                                                                            //counter to record on off toggle switching numbers
static uint8_t pump_gpio_control[8][3]={{0,0,0},{1,0,0},{0,1,0},{1,1,0},{0,0,1},{1,0,1},{0,1,1},{1,1,1}};  //Preset the control conditions with row number related to pump number

static double command_delivery_rate_ct=0;
static bool command_bolus_delivery=false;                                                                  //received command for bolus delivery or basal delivery
static uint16_t current_pump_timing_ct;                                                                    //compensated count needed for single volume delivery of 0.125 uL, need to double for 50% duty cycle.
static uint16_t basal_time_interval_ct=0;                                                                  //basal time interval between two basal deliveries
static bool on_off_flag=false;                                                                             //To flag if the timeout spike is to turn on or off shdn for pumping

static double shdn_delivery=0.0;                                                                           //each pulsed compensated delivery, this is where no feedback and the largest risk. fit for both bolus and basal situation
static bool bolus_delivery=false;                                                                          //Flag for bolus delivery

static bool basal_idle_status=true;                                                                        //monitor if currently in basal idle phase, to save many repetitive calculations
static bool start_test=false;                                                                              //Flag for button controlled start
static volatile bool new_command_received=false;                                                           //flag for updating received command

//pump status
static float basal_rate=0;                      //rate for current or previous used rate
static float bolus_rate=0;                      //rate for current or previous used rate
static uint16_t basal_rate_ct=0;                //changed to one for bolus only delivery 
static uint16_t bolus_rate_ct=0;    
static float shdn_remaining_volume=0;           //monitor the volumes left for single delivery, initially bolus_rate for bolus and 0.125 for basal 

static uint16_t i=0;                            //monitor timing position --used or not, change later to be more informative. 
static uint16_t real_time_timing[250];
static uint16_t *pump_timing=real_time_timing;  //pointer to the timing array, initialized as 2 uL
static uint16_t dosage=5;                       //define each dosage, initialized at 2
static uint16_t delivery_counter=0;             //counter to record volume units delivered but not timestamped, reset every time recorded
static uint16_t total_delivery=0;               //counter to record volume units delivered but not timestamped, reset every time recorded

//Globals for environmental compensation alg
static uint16_t current_total_delivered_b=0;
static uint16_t new_target_delivery_a=0;
static uint16_t update_target_delivery;
static uint16_t over_delivery_e=0;
static uint16_t gas_amt_mol_s=0;

static uint16_t bsp_temp=294.15;
static uint32_t bsp_pressure=101325;
static uint16_t total_current_vol_c=0;
static uint16_t mol_pumped=0;
static uint16_t vol_pumped=0;
static bool new_delivery=false;
static uint16_t over_delivery_threshold=10;
static uint16_t threshold=0;

static uint32_t temp = 294.15;
static uint32_t pressure = 101325;
static uint16_t k = 0;
static uint32_t counter=0;                                                            //counter to turn on off toggle switching. could use just 8bit.

//time to ct conversion, might be processed at the controller side in the future. time unit s
//two bytes count up to 65535 or 54 min. It should be enough if the maximum gap between pulses are 30 min
static uint16_t time_to_ct(float time);
static uint16_t bolus_to_ct(float bolus_rate);
static uint16_t basal_to_ct(float basal_rate);

//Calculate next value based on delivered volume and algorithm
static float comp_value(float * comp_algorithm, float current_volume);

//basal_delivery 
static void basal_delivery(uint16_t basal_rate_ct);

//pump_selection
static void pump_selection();

////current level selection, so far leave as it is. Bolus use 6 mA, Basal use 2 mA. Worst situation is to deliver maximum bolus of 30U, it will take 10 min.
////Trade off is to use 15 mA for large bolus, with reduced battery life.

//initilize function
void basal_delivery_init(); 
void bolus_delivery_init(); 

//ble command interpretation, pumping mode selection
static void ble_cmd_interpretation(uint32_t command);

//multiplexer selection
static void pump_gpio_control_set(uint8_t * p_pump_gpio_control);

//Report end of use for the patch
static void report_end_of_use();

//Main shutdown algorithm
void shdn_timer_timeout_handler(void *p_context);

//Compensation curves
static double moles_pumped(double dosage, uint16_t i);

static uint16_t basal_time_interval_ct_update(uint16_t basal_rate_ct);

void application_timers_stop(void);



