//Now use 6 mA for bolus, 2 mA for basal delivery

#include <math.h>
#include "delivery_control.h"
#include "nrf_gpio.h"

//Be careful about initial pumping, special treatment before using the function
//Calculate next value based on delivered volume and algorithm
static float comp_value(float * comp_algorithm, float current_volume)
{
float comp_temp=0;
for (int i=0;i<4;i++)
  comp_temp=current_volume*comp_temp+comp_algorithm[i];
return comp_temp;
}

//time to ct conversion, might be processed at the controller side in the future.
//two bytes count up to 65535 or 54 min. It should be enough if the maximum gap between pulses are 30 min
static uint16_t time_to_ct(float time)  //time unit s
{
return (uint16_t) 1000*time/SHDN_INTERVAL;
}

static uint16_t bolus_to_ct(float bolus_rate)
{
return (uint16_t) bolus_rate/BOLUS_INCREMENT;
}
static uint16_t basal_to_ct(float basal_rate)
{
return (uint16_t) basal_rate/BASAL_INCREMENT;
}

//basal_delivery 
static void basal_delivery(uint16_t basal_rate_ct)
{
}
//bolus_delivery, need to check availability before this and set mode, nothing in this function, just use pump_selection...also has the same name as important variable 
/*static void bolus_delivery(uint16_t bolus_rate_ct)
{
    pump_selection();

}*/

//pump_selection, so far pump in an order, 
//later might reserve volume for bolus if delivery is not good at the end part, more complicated algorithms
static void pump_selection()
{
    if (pumped_volumes[current_pump]>=PUMP_VOLUME)
    {
        current_pump++;
        //multiplexer function will be here as well
        pump_gpio_control_set(pump_gpio_control[current_pump]);
    }
    if (current_pump==PUMP_NUMBER)
        report_end_of_use();
}

//Assume maximum delivery interval 30 min
//basal_rate_ct is 0.25 uL/count
static uint16_t basal_time_interval_ct_update(uint16_t basal_rate_ct)
{
    return (MAX_BASAL_INTERVAL/SHDN_INTERVAL*1000/basal_rate_ct);
}
////bolus amount update
//static void bolus_volume_update();
//
//
////current level selection, based on pumping requirement. No implementation for now.

//initilize function?
void basal_delivery_init()
{
    nrf_gpio_pin_clear(SIXMA_PIN);
    nrf_gpio_pin_set(TWOMA_PIN);
    shdn_counter=0; 
}

void bolus_delivery_init()
{
    nrf_gpio_pin_clear(TWOMA_PIN);
    nrf_gpio_pin_set(SIXMA_PIN);
}

//ble command interpretation, pumping mode selection, 0xABCD if A is 0, means basal, 1 beans bolus.
//need to compare with current delivery status 1 and 0.
static void ble_cmd_interpretation(uint32_t command)
{
    command_delivery_rate_ct=command&0x0FFF;
    command_bolus_delivery=command>>24;
    //bolus_delivery=(command>>24==0?true:false);
}

static void report_end_of_use()
{
//ble signal sent
//alarm signal sent
}

static void pump_gpio_control_set(uint8_t * p_pump_gpio_control) //set GPIO control based on array
{
  *p_pump_gpio_control==1? nrf_gpio_pin_set(MULTIPLEXER_A0):nrf_gpio_pin_clear(MULTIPLEXER_A0) ;
  *(p_pump_gpio_control+1)==1? nrf_gpio_pin_set(MULTIPLEXER_A1):nrf_gpio_pin_clear(MULTIPLEXER_A1) ;
  *(p_pump_gpio_control+2)==1? nrf_gpio_pin_set(MULTIPLEXER_A2):nrf_gpio_pin_clear(MULTIPLEXER_A2) ;
}

//Function for real time generation for 5uL--will later be used for other amt too
//static void num_ticks(uint16_t* p_dosage, uint16_t* p_i) 
static void num_ticks(uint16_t dosage, uint16_t i)
{
  if(i==k)
    {
    uint32_t local_dosage = dosage;
    uint32_t tick_num=0, tick_num_sum=0, dosage_sum=0;
      while(local_dosage>0)
      {
        if(local_dosage>=5)
        { 
          if(pumped_volumes[current_pump]<4)
          {
            tick_num = round(500.00/(0.5046*pumped_volumes[current_pump]+3.2477))*2;
          }
          else if(pumped_volumes[current_pump]>=4 && pumped_volumes[current_pump]<220)
          {
            tick_num = round(500.00/(-0.0025492*pumped_volumes[current_pump] + 5.1218))*2;
          }
          else if (pumped_volumes[current_pump] >=220)
          {
            tick_num = round(500.00/(-0.078511*pumped_volumes[current_pump] + 22.136))*2;
          }
          tick_num_sum = tick_num_sum + tick_num;   
          local_dosage = local_dosage - 5;
          dosage_sum = dosage_sum + 5;
          pumped_volumes[current_pump] = pumped_volumes[current_pump] + 5;
        }
        if(local_dosage >= 2 && local_dosage < 5)
        {
          if(pumped_volumes[current_pump]==0)
          {
            tick_num = round(500.00/(1.28507102)/2)*2;
          }
          else if(pumped_volumes[current_pump] > 0 && pumped_volumes[current_pump] <= 218)
          {
            tick_num = round(500.00/(-0.00053538*pumped_volumes[current_pump] + 2.0165)/2)*2;
          }
          else if(pumped_volumes[current_pump] > 218)
          {
            tick_num = round(500.00/(-0.0199*pumped_volumes[current_pump]+6.227)/2)*2;
          }
          tick_num_sum = tick_num_sum + tick_num;
          local_dosage = local_dosage - 2;
          dosage_sum = dosage_sum + 2;
          pumped_volumes[current_pump] = pumped_volumes[current_pump] + 2;
        } 
        if (local_dosage > 0 && local_dosage < 2) 
        {
          if(pumped_volumes[current_pump]==0)
          {
            tick_num = round(500.00/(1.28507102)/4)*2;
          }
          else if(pumped_volumes[current_pump] > 0 && pumped_volumes[current_pump] <= 218)
          {
            tick_num = round(500.00/(-0.00053538*pumped_volumes[current_pump] + 2.0165)/4)*2;
          }
          else if(pumped_volumes[current_pump] > 218)
          {
            tick_num = round(500.00/(-0.0199*pumped_volumes[current_pump]+6.227)/4)*2;
          }
          tick_num_sum = tick_num_sum + tick_num;
          local_dosage = 0;
          dosage_sum = dosage_sum;
          pumped_volumes[current_pump] = pumped_volumes[current_pump] +1;
        }
    }   
 // delivery_log[i] = dosage_sum; //should just be decreasing at
  if (tick_num_sum%2 != 0)
  {
    tick_num_sum = tick_num_sum + 1;
  }
  real_time_timing[i] = tick_num_sum;
  tick_num_sum=0; 
  tick_num = 0;
  k++;
  }
}

//SHDN control for pulsed pumping Revise for different pumping frequency and compensation
void shdn_timer_timeout_handler(void *p_context)
{   
    num_ticks(dosage, i);
    uint32_t err_code;
    UNUSED_PARAMETER(p_context);
    if (counter<*(pump_timing+i)) //pumping stage
    {
    nrf_gpio_pin_toggle(SHDN_PIN);
    //LEDS_INVERT(BSP_LED_2_MASK);
    counter++;
    }
    else if (counter==*(pump_timing+i))
    {
    nrf_gpio_pin_clear(SHDN_PIN);
    //LEDS_OFF(BSP_LED_2_MASK);
    counter++;
    delivery_counter+=5;  //after run, record volume unit, here for 5 uLs
    total_delivery+=dosage;
    }
    else if (counter>=599)  //reset counter every 30 sec ...2399 for 2 min
    {
    counter=0;
    i++; //move to next # of ticks...possibly use to allocate another position in delivery_log & i is global
    //Add function to stop everything when done with 300 uL pumping
        if (total_delivery>TARGET)  
        {
            current_pump++;
            if (current_pump>END_PUMP)
            {       
        //        total_delivery=0;
                nrf_gpio_pin_clear(SHDN_PIN); //Make sure it is shut down
                //stop all timers
                application_timers_stop(); //error 2006 to stop everything, cannot go to sleep.
        //        err_code = app_timer_stop(shdn_timer);  //Every 1 s flip SHDN
                APP_ERROR_CHECK(err_code);
                //set system to sleep.
            // Prepare wakeup buttons.
                err_code = bsp_btn_ble_sleep_mode_prepare();
                APP_ERROR_CHECK(err_code);
                // Go to system-off mode (this function will not return; wakeup will cause a reset).
                err_code = sd_power_system_off();
                APP_ERROR_CHECK(err_code);
            }
            else //switch to next gpio control
            {
                total_delivery=0;
                pumped_volumes[current_pump]= 0;
                i=0;  //timing array go back to 0
                pump_gpio_control_set(pump_gpio_control[current_pump]);
            }
        }
    }
    else
    counter++;
}



