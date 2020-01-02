/**
******************************************************************************
* @file    main.cpp
* @author  Davide Aliprandi, STMicroelectronics
* @version V1.0.0
* @date    October 14th, 2015
* @brief   mbed test application for the STMicroelectronics X-NUCLEO-IHM01A1
*          Motor Control Expansion Board: control of 2 motors.
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/


/* Includes ------------------------------------------------------------------*/

/* mbed specific header files. */
#include "mbed.h"

/* Helper header files. */
#include "DevSPI.h"

/* Component specific header files. */
#include "L6474.h"
#include "L6474_def.h"

//#include "steppermotor.h"


/* Definitions ---------------------------------------------------------------*/

/* Number of steps. */
#define STEPS 3200

/* Delay in milliseconds. */
#define DELAY_1 2000
#define DELAY_2 6000
#define DELAY_3 8000

/* Speed in pps (Pulses Per Second).
In Full Step mod  queue.dispatch();e: 1 pps = 1 step/s).
In 1/N Step Mode:  N pps = 1 step/s). */
#define SPEED_1 2400
#define SPEED_2 1200


#include "Tokenizer.h"

/* Variables -----------------------------------------------------------------*/

/* Motor Control Component. */
L6474 *motor1;
L6474 *motor2;
L6474 *motor3;

int step_mode=0;


#define M3
#define M2




InterruptIn InterA4(A4);
InterruptIn InterA3(A3);

DigitalIn PinA4 (A4);
DigitalIn PinA3 (A3);

DigitalOut POWER(A5);

DigitalOut myled(LED1);




RawSerial Console(USBTX, USBRX, 19200);

static EventQueue queue(64*100*sizeof(char)); // 8 words of storage



void alarm_pinA4_on(){
  printf("ON A4\n");
  myled=1;
}

void alarm_pinA4_off(){
  printf("OFF A4\n");
  myled=0;
}

void alarm_pinA3_on(){
  printf("ON A3\n");
}

void alarm_pinA3_off(){
  printf("OFF A3\n");
}




PlatformMutex stdio_mutex;

void sendString(char * line){

	char buffer[64];
  //stdio_mutex.lock();
  sprintf(buffer,"%s\n",line);
  Console.puts(buffer);  
  //stdio_mutex.unlock();

}


void SendStatus(){
  unsigned int statusRegister = motor2->get_status();
  char buffer[100] ;
  sprintf(buffer,"%s %d\n","STATUS",statusRegister);
  sendString(buffer);
}





int RangeX=0;
char bufferCommand[100] ;

char * test;

const char * test_line;


void processLine(string line){



  char bufferCommand[100] ;

  Tokenizer parser(line," ,");

  string command = parser.next();

  if (command=="POWER"){

    string SW = parser.next();
    int iSW=atoi(SW.c_str());
    if (iSW==0){
      POWER=1;

    }
    else{
      POWER=0;
      wait_ms(5000);
      //motor2->enable();
      //motor3->enable();
    }


  sprintf(bufferCommand,"INFO ECHO %s\n",line.c_str());
  Console.puts(bufferCommand);



  }


  if (command=="INC"){

    string s_idxMotor = parser.next();
    string s_step = parser.next();
    int i_idxMotor=atoi(s_idxMotor.c_str());
    int i_step=atoi(s_step.c_str());
    L6474 *motor;
    switch (i_idxMotor)
    {
    case 0:
      motor=motor1;
      break;
    case 1:
      motor=motor2;
      break;
    case 2:
      motor=motor3;
      break;
    }
    int CurrX=motor->get_position();
    motor->set_direction(FORWARD);
    for (int i=0;i<i_step;i++){
        if (PinA4==0) {
          motor->Step();
          wait_us(1000);
        }
    }
    int actualX=motor->get_position();
    int CurrEle= 0;
    sprintf(bufferCommand,"%s %d,%d\n","POSX",actualX,CurrEle);
    sendString(bufferCommand);


  }

  if (command=="DEC"){

    string s_idxMotor = parser.next();
    string s_step = parser.next();
    int i_idxMotor=atoi(s_idxMotor.c_str());
    int i_step=atoi(s_step.c_str());
    L6474 *motor;
    switch (i_idxMotor)
    {
    case 0:
      motor=motor1;
      break;
    case 1:
      motor=motor2;
      break;
    case 2:
      motor=motor3;
      break;
    }
    int CurrX=motor->get_position();
    //if (PinA4==0)
    {
      motor->set_direction(BACKWARD);
      for (int i=0;i<i_step;i++){
      motor->Step();
      wait_us(1000);
      }
    }

    int actualX=motor->get_position();
    int CurrEle= 0;
    sprintf(bufferCommand,"%s %d,%d\n","POSX",actualX,CurrEle);
    sendString(bufferCommand);
  }

  if (command=="HOME"){

    string s_idxMotor = parser.next();
    int i_idxMotor=atoi(s_idxMotor.c_str());
    L6474 *motor;
    switch (i_idxMotor)
    {
    case 0:
      motor=motor1;
      break;
    case 1:
      motor=motor2;
      break;
    case 2:
      motor=motor3;
      break;
    }
    int CurrX=motor->get_position();
    motor->set_direction(FORWARD);
    while (true)
    {
        if (PinA4==0)
        {
          motor->Step();
          wait_us(1000);
        }
        else
          break;

    }
    int actualX=motor->get_position();
    int CurrEle= 0;
    sprintf(bufferCommand,"%s %d,%d\n","POSX",actualX,CurrEle);
    sendString(bufferCommand);


  }

  if (command=="STEP_MODE"){

    string s_step = parser.next();

    StepperMotor::step_mode_t mode=StepperMotor::STEP_MODE_UNKNOWN ;

    if (s_step.compare("Full")==0){
       mode=StepperMotor::STEP_MODE_FULL;
    }
    else if (s_step.compare("Half")==0){
      mode=StepperMotor::STEP_MODE_HALF;
    }
    else if (s_step.compare("1/4")==0){
      mode=StepperMotor::STEP_MODE_1_4;
    }
    else if (s_step.compare("1/8")==0){
      mode=StepperMotor::STEP_MODE_1_8;
    }
    else if (s_step.compare("1/16")==0){
      mode=StepperMotor::STEP_MODE_1_16;
    }
    if (mode!=StepperMotor::STEP_MODE_UNKNOWN)
    {
      motor1->set_step_mode(mode);
      motor2->set_step_mode(mode);
      motor3->set_step_mode(mode);
      motor2->enable();
      motor3->enable();
      sprintf(bufferCommand,"%s %s","OK",line.c_str());
      sendString(bufferCommand);
    }
    else
    {
      sprintf(bufferCommand,"%s %s","KO",line.c_str());
      sendString(bufferCommand);
    }

  }

  if (command=="GOTOA"){

    int CurrX=motor2->get_position();
    string TO = parser.next();
    int iTo=atoi(TO.c_str());
    int diff=abs(iTo-CurrX);
    if (CurrX>iTo)
    {
      motor2->set_direction(BACKWARD);
      while (true)
      {
        if(diff==0) break;
        if (PinA3==0) break;
        motor2->Step();
        diff=diff-1;
      }
    }
    else if (iTo>CurrX)
    {
      motor2->set_direction(FORWARD);
      while (true)
      {
        if(diff==0) break;
        if (PinA4==0) break;
        motor2->Step();
        diff=diff-1;
      }

    }

    int actualX=motor2->get_position();
    unsigned int CurrEle= 0;
    sprintf(bufferCommand,"%s=%d,%d\n","POSX",actualX,CurrEle);
    sendString(bufferCommand);

  }






}



/**
* @brief  This is an example of error handler.
* @param[in] error Number of the error
* @retval None
* @note   If needed, implement it, and then attach it:
*           + motor->attach_error_handler(&my_error_handler);
*/
void my_error_handler(uint16_t error)
{
  /* Printing to the console. */
  printf("Error %d detected\r\n\n", error);

  /* Infinite loop */

}




Thread      ISRthread(osPriorityAboveNormal);
osThreadId  ISRthreadId;
Thread t;

typedef struct {
    char  message [64];
    uint32_t counter;   /* A counter value               */
} message_t;


MemoryPool<message_t, 16> mpool;

Queue<message_t, 16> queueMessage;



//RawSerial   pc(USBTX, USBRX);
Mutex       serial_mutex;

void        newInput();
void        ISR_thread();





void newInput()
{
    Console.attach(NULL);    //detach the ISR to prevent recursive calls
    osSignalSet(ISRthreadId, 0x01);
}



int serial_count=0;



void ISR_thread()
{

    ISRthreadId = osThreadGetId();


    string line="";

    for (;;) {
        osSignalWait(0x01, osWaitForever);

        while (Console.readable()) {
                //test_line=line.c_str();
                char serial_char = Console.getc();
                serial_count++;
                if (serial_char == '\n' )
                {
                //test_line=line.c_str();
                //sprintf(bufferCommand,"INFO %s\n",test_line);
                //Console.puts(bufferCommand);
                //char buffer[64] ;
							  //void * mem=malloc(64);	
                //sprintf(bufferCommand,"%s",mem);
                //queue.call(processLine,(char *)mem);
								message_t  *Message = mpool.alloc();
								test_line=line.c_str();
								sprintf(Message->message,"%s",test_line);
								queueMessage.put(Message); 
                //queue.dispatch();
                //processLine(line);
                line="";
                serial_count=0;

                }
                else
                {
                    line.push_back(serial_char);
                }
        }
//        serial_mutex.unlock();
        Console.attach(newInput);    // re-attach the ISR
    }
}


int main_th(void)
{
    t.start(callback(&queue, &EventQueue::dispatch_forever));
    ISRthread.start(callback(ISR_thread));
	  Console.attach(newInput);   //interrupt to catch input
	  while (true) {
			   myled = !myled;
        osEvent evt = queueMessage.get();
        if (evt.status == osEventMessage) {
            message_t *Message = (message_t*)evt.value.p;          
					char * test=Message->message		;			
						processLine(Message->message);
            mpool.free(Message);
        }
    }

}

/* Main ----------------------------------------------------------------------*/


char * debug_r;

Ticker alive;

void send_alive() {
	
	   alive.detach();
     sendString("INFO ALIVE");
	   alive.attach(&send_alive, 5.0);
}

int main()
{

  alive.attach(&send_alive, 5.0);
	
  sendString("INFO START WITH POWER OFF");
	
	
  POWER=1;
  
  wait_ms(4000);

  main_th();

  /*----- Initialization. -----*/

  /* Initializing SPI bus. */
  DevSPI dev_spi(D11, D12, D13);

  /* Initializing Motor Control Components. */
  motor1 = new L6474(D2, D8, D7, D9, D10, dev_spi);
#ifdef M2
  motor2 = new L6474(D2, D8, D4, D3, D10, dev_spi);
#ifdef M3
  motor3 = new L6474(D2, D8, D5, D6, D10, dev_spi);
#endif
#endif

  motor1->attach_error_handler(my_error_handler);

  //motor1 = new L6474(D2, D8, D7, D9, D10, dev_spi);
  //    motor2 = new L6474(D2, D8, D4, D3, D10, dev_spi);
  //    motor3 = new L6474(D2, D8, D5, D6, D10, dev_spi);


  if (motor1->init() != COMPONENT_OK) {
    exit(EXIT_FAILURE);
  }
#ifdef M2
  if (motor2->init() != COMPONENT_OK) {
    exit(EXIT_FAILURE);
  }
#ifdef M3
  if (motor3->init() != COMPONENT_OK) {
    exit(EXIT_FAILURE);
  }
#endif
#endif


  motorStepMode_t stepm;
  step_mode=motor1->set_step_mode(StepperMotor::STEP_MODE_1_4);
#ifdef M2
  step_mode=motor2->set_step_mode(StepperMotor::STEP_MODE_1_8);
#ifdef M3
  step_mode=motor3->set_step_mode(StepperMotor::STEP_MODE_1_16);
#endif
#endif

  step_mode=motor1->get_parameter(L6474_STEP_MODE);
#ifdef M2
  step_mode=motor2->get_parameter(L6474_STEP_MODE);
#ifdef M3
  step_mode=motor3->get_parameter(L6474_STEP_MODE);
#endif
#endif

  /* Printing to the console. */


  InterA4.rise(&alarm_pinA4_on);
  InterA4.fall(&alarm_pinA4_off);
  InterA4.enable_irq();
	
	


  motor2->enable();
  motor3->enable();




  while(false) {
    motor2->set_direction(FORWARD);
    motor3->set_direction(FORWARD);
    for (int i=0;i<200*16;i++){
      motor2->Step();
      motor3->Step();
      wait_us(300);
    }

    POWER=0;
    wait_ms(5000);
    POWER=1;
    wait_ms(5000);

    motor2->enable();
    motor3->enable();

    motor2->set_direction(BACKWARD);
    motor3->set_direction(BACKWARD);

    for (int i=0;i<200*16;i++){
      motor2->Step();
      motor3->Step();
      wait_us(300);
    }

    POWER=0;
    wait_ms(5000);
    POWER=1;
    wait_ms(5000);

    motor2->enable();
    motor3->enable();

  }

  /* Infinite Loop. */
  while(false) {

    step_mode=motor1->set_step_mode(StepperMotor::STEP_MODE_1_4);
#ifdef M2
    step_mode=motor2->set_step_mode(StepperMotor::STEP_MODE_1_4);
#ifdef M3
    step_mode=motor3->set_step_mode(StepperMotor::STEP_MODE_1_4);
#endif
#endif

    /* Requesting to go to a specified position. */
    motor1->go_to(- (STEPS >> 1));
#ifdef M2
    motor2->go_to(- (STEPS >> 1));
#ifdef M3
    motor3->go_to(- (STEPS >> 1));
#endif
#endif

    /* Waiting while the motor is active. */
    motor1->wait_while_active();
#ifdef M2
    motor2->wait_while_active();
#ifdef M3
    motor3->wait_while_active();
#endif
#endif

    wait_ms(1000);

    step_mode=motor1->set_step_mode(StepperMotor::STEP_MODE_1_8);
#ifdef M2
    step_mode=motor2->set_step_mode(StepperMotor::STEP_MODE_1_16);
#ifdef M3
    step_mode=motor3->set_step_mode(StepperMotor::STEP_MODE_1_4);
#endif
#endif
    /* Requesting to go to a specified position. */
    motor1->go_to( (STEPS >> 1));
#ifdef M2
    motor2->go_to( (STEPS >> 1));
#ifdef M3
    motor3->go_to( (STEPS >> 1));
#endif
#endif

    /* Waiting while the motor is active. */
    motor1->wait_while_active();
#ifdef M2
    motor2->wait_while_active();
#ifdef M3
    motor3->wait_while_active();
#endif
#endif


  }

  string line;


 //stage

  while(true){

    if ( Console.readable() )
    {
      debug_r= (char*) line.c_str();

      char serial_char = Console.getc();
      serial_count++;
      if (serial_char == '\n' )
      {
        // pc.puts(line.c_str());
        processLine(line);
        line="";
        serial_count=0;

      }
      else
      {
        line.push_back(serial_char);
      }

    }

  }
}

