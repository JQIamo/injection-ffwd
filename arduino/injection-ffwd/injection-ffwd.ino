#include "DueTimer/DueTimer.h"
#include "injection-init.h"

/*

// states
r[0]: Reset state; DAC -> midscale; piezo scan is on but no FB
l[1]: lock state; implements PID
a[2]: acquire; implements re-acquire routine

// parameters
p -> interpret as parameter
command structure: 'p 0 15' -> set param 0 (pzt_min) to 15

0 pzt_min
1 pzt_max
2 scans_to_average
3 reaq_stepsize
4 fb_stepsize
5 unlocked_threshold
6 locked_threshold
7 pzt_offset_delta
*/

//DueTimer timer(0);

volatile bool ramp_dir = 1; // 1 = rising; 0 = falling
volatile bool end_scan_flag = 0;    // flag for end of single period of pzt scan

uint16_t fb_o = 2048;   // current feedback output; set to mid-scale.


int unlocked_threshold = 1250;
int locked_threshold = 1450;
int fb_stepsize = 30;
int reaq_stepsize = 250;

int pzt_offset_delta = 300;


// state for state machine
int state = 0;
bool railed = 0;


int scan_counter = 0;
float average_height = 0.0;
float global_average = 0.0;


//lockmode_t fb_mode;

int fb_flag = 0;


uint16_t df_fb = 300;

int lock_sm = 0;
float lock_hist[3];
uint16_t lock_val[3];






// generate triangle ramp 
void advance_waveform(){
  pzt_o += (ramp_dir ? PZT_RRR : PZT_FRR);

  // check bounds, correct if necessary and reverse ramp direction.
  if (pzt_o >= pzt_max){
    pzt_o = pzt_max;
    ramp_dir = 0;
  } else if (pzt_o < pzt_min){
    pzt_o = pzt_min;
    ramp_dir = 1;

    Timer1.stop();
    NVIC_DisableIRQ(ADC_IRQn);
    end_scan_flag = 1;
  }

  analogWrite(DAC0, pzt_o);
}

void reset_waveform(){
  pzt_o = pzt_min;
  ramp_dir = 1;
  end_scan_flag = 0;
  pk_count = 0;
}



void query_cavity(){
  reset_waveform(); // reset counters, buffers, etc.
  NVIC_EnableIRQ(ADC_IRQn); // turn on ADC interrupts
  Timer1.start();   // timer is attached to advance_waveform; enable to start PZT waveform generation
}


/* State machine:

Sc

*/




void setup() {

  Serial.begin(115200);
  
  adc_dac_setup();  // configure ADC & DAC
  
  // output voltages on PZT and DAC
  analogWrite(DAC0, pzt_min);
  analogWrite(DAC1, fb_o);
  
 // timer = DueTimer(0);

  
  // attach timer interrupt for PZT waveform
  Timer1.attachInterrupt(advance_waveform).setPeriod(PZT_TIMESTEP);
  
  // initialize state machine...?
  state = 0;
  // start
  query_cavity();

}





void update_fb(){

    // make sure we loop back to 4095
    if (fb_o > 4095){
        fb_o = 4095;
    }


    // if in lock mode and fb_o close to rail, throw rail flag
    // else, loop at 12 bits
    if (state == 1 &&  (((4095 - fb_o) < RAILED_OVERHEAD) || fb_o < RAILED_OVERHEAD)){
        railed = 1;
    } else {
        railed = 0;
    }
    
    analogWrite(DAC1, fb_o);    
}


void processSM(){

   
    switch (state){
    case 0:
        // state 0: reset
        fb_o = 2048;
        update_fb();
        query_cavity();
        break;
    
    case 1:
        // state 1: lock mode
        
        
        break;
        
    case 2:
    
        break;
        
    default:
        fb_o = 2048;
        update_fb();
        query_cavity();
        break;
    }


}

void processSerial(){
 // check for serial
    if (Serial.available() > 0){
        char c = Serial.read();
        switch (c){
        case 'r':
            // reset state
            state = 0;
           //  Serial.print("state: ");
//             Serial.println(state);
            break;
        case 'l':
            // lock state
            state = 1;
//             Serial.print("state: ");
//             Serial.println(state);            
            break;
        case 'a':
            // lock state
            state = 2;
           //  Serial.print("state: ");
//             Serial.println(state);           
            break;
        
        case 'p':
            // 0 pzt_min
            // 1 pzt_max
            // 2 scans_to_average
            // 3 reaq_stepsize
            // 4 fb_stepsize
            // 5 unlocked_threshold
            // 6 locked_threshold
            // 7 pzt_offset_delta
            int pnum = Serial.parseInt();
            int val = Serial.parseInt();
            switch (pnum){
            case 0:
                pzt_min = val;
                break;
            case 1:
                pzt_max = val;
                break;
            case 2:
                scans_to_average = val;
                break;
            case 3:
                reaq_stepsize = val;
                break;
            case 4:
                fb_stepsize = val;
                break;
            case 5:
                unlocked_threshold = val;
                break;
            case 6:
                locked_threshold = val;
                break;
            case 7:
                pzt_offset_delta = val;
                break;
                    
            
            
            }
        }
        
        // not sure why flush() is broken...
        // but then again so is this :/
        while (Serial.available() > 0){
            Serial.read();
        }
        processSM();
    }
}


float process_scan(){

// increment scan count
      scan_counter++;
      
      uint16_t peak_heights[MAX_PEAK_CT];
      uint8_t pk_height_idx = 0;
      
      uint16_t current_max;
      
      // stash peak maxima
      for (int i = 0; i < pk_count; i++){
        current_max = 0;
        for (int j = 0; j < pt_count[i]; j++){
            current_max = (buf[i][j] > current_max) ? buf[i][j] : current_max;
        }
        peak_heights[pk_height_idx++] = current_max;
      }
      
      // calculate average
      uint32_t sum = 0;
      for (int i = 0; i < pk_height_idx; i++){
        sum += peak_heights[i];
      }
      
      // adjust pzt_min in case too close to turnaround.
      // want to servo first peak at pzt_min + 300??
      
//       if (pk_count > 1){
//           int pzt_offset = pk_position[0] - pzt_offset_delta;
//           if (pzt_offset < 0){
//             pzt_min = pk_position[0] + pzt_offset_delta;
//           } else {
//             pzt_min = pzt_offset;
//           }
//       }
      return ((float) sum / pk_count);
    
}


float old_average = 0.0;
int polarity = 1;

void loop() {

    processSerial();
   
    
    // if finished a scan, calculate average peak height
    if (end_scan_flag){
        // reset scan flag
        end_scan_flag = 0;
       // reject scans with bad peak count?
        average_height +=  process_scan();
        
        if (scan_counter % scans_to_average == 0){
            global_average = average_height/scans_to_average;
            
            if (isnan(global_average) && state != 0){
                state = 2;        
            }
            
            Serial.print(global_average);
            Serial.print("\t");
            Serial.print(fb_o);
            Serial.print("\t");
            Serial.print(state);
            Serial.print("\t");
            Serial.println(polarity);
            
            switch (state){
            case 0:
                // reset mode
                break;
            case 1:
                // lock mode
                if (unlocked_threshold < global_average && global_average <= locked_threshold){
                    // if "locked" but marginal, step current to see which side
                    old_average = global_average;
                    fb_o += fb_stepsize*polarity;
                    state = 11;
                
                    update_fb();
                   
                } else if(global_average < unlocked_threshold){
                    state = 2;  // put in re-acquire mode
                }
                //otherwise, do nothing!
                break;
            
            case 11:
                // entered here from lock mode; sampling which side of lock region
                if (global_average < old_average){
                    // moving in wrong direction, flip polarity
                    polarity = -1*polarity;
                }
                fb_o += fb_stepsize*polarity;
                old_average = global_average;
            
            
                if (global_average > locked_threshold){
                    fb_o += fb_stepsize*polarity;
                    state = 1;
                } else if (global_average < unlocked_threshold){
                    // messed up, try to reacquire lock
                    state = 2;
                }
                update_fb();
                break;
            
            case 2:
                // need to re-acquire
                if (global_average > unlocked_threshold){
                    // lock has caught, so go into locking mode!
                    state = 1;
                } else {
                    fb_o -= reaq_stepsize;
                    update_fb();
                }
                break;
            
            
        
        
            }
        
            // reset average height
            average_height = 0.0;
        }
        
        delay(1);
        query_cavity();
    } 
}  


       //  if (fb_flag == 1){
//             if (average_height/scans_to_average >= 1450.0){ 
//                 fb_flag = 2; // move to locked mode
//                 Serial.print("Lock mode\t");
//                 Serial.println(fb_o);
//             } else {
//                   Serial.print(fb_o);
//             Serial.print("\t");
//             Serial.println(global_ave);
//                 fb_o -= df_fb;
//                 analogWrite(DAC1, fb_o);
//             }
//         } else if (fb_flag == 2){
//             // if in locked mode...
//             switch (lock_sm){
//             
//                 case 0:
// 
//                 lock_hist[0] = global_ave;
//                 lock_val[0] = fb_o;
//                 
//                 // only do something if at edge of range
//                 if(lock_hist[0] < 1450 || isnan(lock_hist[0])){
//                                   
//                 
//                 // sample up next time around
//                 fb_o += 30;
//                 analogWrite(DAC1, fb_o);
//                 lock_sm = 1;
//                 }
//                 break;
//                 
//                 case 1:
//        
// 
//                 lock_hist[1] = global_ave;
//                 lock_val[1] = fb_o;
//                 // sample down next time around
//                 fb_o = lock_val[0] - 30;
//                 analogWrite(DAC1, fb_o);
//                 
//                 lock_sm = 2;
//                 break;
//                 
//                 case 2:
//       
//                 lock_hist[2] = global_ave;
//                 lock_val[2] = fb_o;
//       
//                 Serial.print("lock\t");
//                 Serial.print(fb_o+30);
//                 Serial.print("\t");
//                 Serial.print(lock_hist[2]);
//                 Serial.print("\t");
//                 Serial.print(lock_hist[0]);
//                 Serial.print("\t");
//                 Serial.println(lock_hist[1]);
//               
//                 
//       
//                 if(lock_hist[2] > (lock_hist[0]+5)){
//                     Serial.println("decreasing");
//                     //fb_o = lock_val[2];
//                     fb_o = lock_val[0] - 60;
//                 //fb_o -= 2*30;
//                 analogWrite(DAC1, fb_o);
//                 } else if (lock_hist[1] > (lock_hist[0] + 5)){
//                     //fb_o = lock_val[1]; 
//                     fb_o = lock_val[0] + 60;
//                     analogWrite(DAC1, fb_o);
//                     
//                     Serial.println("increasing");
//                     }else if (isnan(lock_hist[0])){
//                         lock_sm = 0;
//                
//                         fb_flag = 1;
//                         df_fb = 300;
//                         fb_o = 4000;
//                         Serial.println("relocking");
//                     }
//                     else {
//                     fb_o = lock_val[0];
//                     analogWrite(DAC1, fb_o);
//                     //Serial.println("same");
//                     } 
//                 lock_sm = 0;
//                 break;
//                 
//                 
//             }
//         }
//         average_height = 0.0;
//       }
