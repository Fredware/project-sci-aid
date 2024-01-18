/*
    Copyright 2021-2023 Picovoice Inc.

    You may not use this file except in compliance with the license. A copy of the license is located in the "LICENSE"
    file accompanying this source.

    Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on
    an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the
    specific language governing permissions and limitations under the License.
*/

#include <Picovoice_EN.h>

#include "params.h"
#include "nRF52_MBED_PWM.h"

uint32_t faultPin = D12;
uint32_t greenPin =  LEDG;

uint32_t myPinClose  = D3;
uint32_t myPinOpen  = D5;
float dutyCycleClose = 0;
float freq      = 500.0f;
float dutyCycleOpen = 0;
//float freq      = 50.0f;

float curDutyCycle  = dutyCycleClose;
float curFreq       = freq;

mbed::PwmOut* pwm   = NULL;
mbed::PwmOut* pwmOpen   = NULL;
          
#define MEMORY_BUFFER_SIZE (70 * 1024)

static const char *ACCESS_KEY = "Uk0xI2KTv04bnYFj0T8fEhxVcqcNuhFvv1LYh/nkiiFXLqUIQh0OsQ=="; // AccessKey string obtained from Picovoice Console (https://picovoice.ai/console/)

static pv_picovoice_t *handle = NULL;

static int8_t memory_buffer[MEMORY_BUFFER_SIZE] __attribute__((aligned(16)));

static const float PORCUPINE_SENSITIVITY = 0.5f;
static const float RHINO_SENSITIVITY = 1.0;
static const float RHINO_ENDPOINT_DURATION_SEC = .5f;
static const bool RHINO_REQUIRE_ENDPOINT = false;


static void wake_word_callback(void) {
    //Serial.println("Wake word detected!");
    digitalWrite(greenPin, HIGH);

}

static void inference_callback(pv_inference_t *inference) {
///////////////////////////
//uncomment //Serial lines for debugging
// //Serial.println("{");
//     //Serial.print("    is_understood : ");
//     //Serial.println(inference->is_understood ? "true" : "false");
//     if (inference->is_understood) {
//         //Serial.print("    intent : ");
//         //Serial.println(inference->intent);
//         if (inference->num_slots > 0) {
//             //Serial.println("    slots : {");
//             for (int32_t i = 0; i < inference->num_slots; i++) {
//                 //Serial.print("        ");
//                 //Serial.print(inference->slots[i]);
//                 //Serial.print(" : ");
//                 //Serial.println(inference->values[i]);
//             }
//             //Serial.println("    }");
//         }
//     }
//     //Serial.println("}\n");

// parse intent and set speed and duration accordingly
    if(inference->is_understood){
    
      if(String(inference->intent)== "close"){
        if(inference->num_slots > 0){
          if((String(inference->slots[0]) == "fast") || 
          (String(inference->slots[0]) == "quick") || 
          (String(inference->slots[0]) == "quickly"))
          { 
          dutyCycleClose = 35;
          setPWM(pwm, myPinClose, freq, dutyCycleClose);
          delay(500);
          dutyCycleClose = 0;
          setPWM(pwm, myPinClose, freq, dutyCycleClose);

          }else{
          dutyCycleClose = 15;
          setPWM(pwm, myPinClose, freq, dutyCycleClose);
          delay(500);
          dutyCycleClose = 0;
          setPWM(pwm, myPinClose, freq, dutyCycleClose);
          }
        }else{

            dutyCycleClose = 20;
            setPWM(pwm, myPinClose, freq, dutyCycleClose);
            delay(1000);
            dutyCycleClose = 0;
            setPWM(pwm, myPinClose, freq, dutyCycleClose);
        }
        }
        else if(String(inference->intent) == "open"){
           if(inference->num_slots > 0){
              if((String(inference->slots[0]) == "fast") || 
              (String(inference->slots[0]) == "quick") || 
              (String(inference->slots[0]) == "quickly"))
                { 
                dutyCycleOpen = 35;
                setPWM(pwmOpen, myPinOpen, freq, dutyCycleOpen);
                delay(500);
                dutyCycleOpen = 0;
                setPWM(pwmOpen, myPinOpen, freq, dutyCycleOpen);

                }
              else{
                dutyCycleOpen = 15;
                setPWM(pwmOpen, myPinOpen, freq, dutyCycleOpen);
                delay(500);
                dutyCycleOpen = 0;
                setPWM(pwmOpen, myPinOpen, freq, dutyCycleOpen);
                }
            }
          else{
            dutyCycleOpen = 20;
            setPWM(pwmOpen, myPinOpen, freq, dutyCycleOpen);
            delay(1000);
            dutyCycleOpen = 0;
            setPWM(pwmOpen, myPinOpen, freq, dutyCycleOpen);
          }
          
        }
        else{
          dutyCycleClose = 0;
          setPWM(pwm, myPinClose, freq, dutyCycleClose);
          delay(100);
          dutyCycleOpen = 0;
          setPWM(pwmOpen, myPinOpen, freq, dutyCycleOpen);

         }
    }else
        {

        dutyCycleClose = 0;
        dutyCycleOpen = 0;
        setPWM(pwm, myPinClose, freq, dutyCycleClose);
        delay(100);
        setPWM(pwmOpen, myPinOpen, freq, dutyCycleOpen);

      }
  
    digitalWrite(greenPin, LOW);
    pv_inference_delete(inference);
}

static void print_error_message(char **message_stack, int32_t message_stack_depth) {
    for (int32_t i = 0; i < message_stack_depth; i++) {
        //Serial.println(message_stack[i]);
    }
}

void setup() {
    //Serial.begin(9600);
    //while (!//Serial);

    pv_status_t status = pv_audio_rec_init();
    if (status != PV_STATUS_SUCCESS) {
        //Serial.print("Audio init failed with ");
        //Serial.println(pv_status_to_string(status));
        while (1);
    }

    char **message_stack = NULL;
    int32_t message_stack_depth = 0;
    pv_status_t error_status;

    status = pv_picovoice_init(
        ACCESS_KEY,
        MEMORY_BUFFER_SIZE,
        memory_buffer,
        sizeof(KEYWORD_ARRAY),
        KEYWORD_ARRAY,
        PORCUPINE_SENSITIVITY,
        wake_word_callback,
        sizeof(CONTEXT_ARRAY),
        CONTEXT_ARRAY,
        RHINO_SENSITIVITY,
        RHINO_ENDPOINT_DURATION_SEC,
        RHINO_REQUIRE_ENDPOINT,
        inference_callback,
        &handle);
    if (status != PV_STATUS_SUCCESS) {
        //Serial.print("Picovoice init failed with ");
        //Serial.println(pv_status_to_string(status));

        error_status = pv_get_error_stack(&message_stack, &message_stack_depth);
        if (error_status != PV_STATUS_SUCCESS) {
            //Serial.println("Unable to get Porcupine error state");
            while (1);
        }
        print_error_message(message_stack, message_stack_depth);
        pv_free_error_stack(message_stack);

        while (1);
    }

    const char *rhino_context = NULL;
    status = pv_picovoice_context_info(handle, &rhino_context);
    if (status != PV_STATUS_SUCCESS) {
        //Serial.print("retrieving context info failed with");
        //Serial.println(pv_status_to_string(status));
        while (1);
    }
    //Serial.println("Wake word: 'Fillauer'");
    //Serial.println(rhino_context);

    //set up pins for debugging and hand control
    
    pinMode(myPinOpen, OUTPUT);
    digitalWrite(myPinOpen, LOW);
    pinMode(greenPin, OUTPUT);
    digitalWrite(greenPin, LOW);
    pinMode(myPinClose, OUTPUT);
    digitalWrite(myPinClose, LOW);
    delay(100);
    PWM_LOGERROR7("Freq = ", freq, ", DutyCycle % = ", dutyCycleClose, ", DutyCycle = ", dutyCycleClose / 100, ", Pin = ", myPinClose);
    delay(100);


}

void loop() {



    const int16_t *buffer = pv_audio_rec_get_new_buffer();
    if (buffer) {
        const pv_status_t status = pv_picovoice_process(handle, buffer);
        if (status != PV_STATUS_SUCCESS) {
            //Serial.print("Picovoice process failed with ");
            //Serial.println(pv_status_to_string(status));
            char **message_stack = NULL;
            int32_t message_stack_depth = 0;
            pv_get_error_stack(
                &message_stack,
                &message_stack_depth);
            for (int32_t i = 0; i < message_stack_depth; i++) {
                //Serial.println(message_stack[i]);
            }
            pv_free_error_stack(message_stack);
            while (1);
        }
    }
}
