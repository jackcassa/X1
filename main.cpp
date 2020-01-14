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

#include "Tokenizer.h"

/* Definitions ---------------------------------------------------------------*/


#define M3
#define M2



/* Variables -----------------------------------------------------------------*/

/* Motor Control Component. */
L6474 *motor1;
L6474 *motor2;
L6474 *motor3;

int step_mode=0;

float debug_f;
float mode;


InterruptIn InterA4(A4);
InterruptIn InterA3(A3);
InterruptIn InterA2(A2);

DigitalIn PinA4 (A4);
DigitalIn PinA3 (A3);
DigitalIn PinA2 (A2);


DigitalOut POWER(A5,1);
DigitalOut myled(LED2);

RawSerial Console(USBTX, USBRX, 115200);

typedef struct {
    char  message [64];
    uint32_t counter;   /* A counter value               */
} message_t;

static EventQueue queue(3*sizeof(int)); // 8 words of storage

int RangeX=0;
char bufferCommand[100] ;
char * test;
const char * test_line;
int serial_count=0;
Thread      ISRthread(osPriorityAboveNormal);
osThreadId  ISRthreadId;
Thread t;

Queue<message_t, 16> queueMessage;
Queue<uint32_t, 5> queueEvent;

MemoryPool<message_t, 16> mpool;
char * debug_r;
//Ticker alive;
DevSPI dev_spi(D11, D12, D13);
Mutex       serial_mutex;

uint32_t message;
PlatformMutex stdio_mutex;

void        newInput();
void        ISR_thread();


void sendString(char * line){

	char buffer[128];
  //stdio_mutex.lock();
  sprintf(buffer,"%s\n",line);
  Console.puts(buffer);  
  //stdio_mutex.unlock();

}


void SendStatus(){
  unsigned int statusRegister1 = motor1->get_status();
	unsigned int statusRegister2 = motor2->get_status();
	unsigned int statusRegister3 = motor3->get_status();
  char buffer[100] ;
  sprintf(buffer,"%s %d %d %d\n","STATUS",statusRegister1,statusRegister2,statusRegister3);
  sendString(buffer);
}

void SendError(int v1,int v2,int v3 ){
  char buffer[100] ;
  sprintf(buffer,"%s %d %d %d\n","STATUS",v1,v2,v3);
  sendString(buffer);
}

void SendLog(char * message ){
	char buffer[100] ;
  sprintf(buffer,"%s %s \n","INFO ",message);
  sendString(buffer);
}


void SendPos(){
      int x1,x2;
	    int y1,y2;
			int z1,z2;
      char bufferCommand[128] ;
      z1=motor3->get_parameter(L6474_ABS_POS);      
      z2=motor3->get_parameter(L6474_EL_POS);
	    y1=motor2->get_parameter(L6474_ABS_POS);      
      y2=motor2->get_parameter(L6474_EL_POS);
			x1=motor1->get_parameter(L6474_ABS_POS);      
      x2=motor1->get_parameter(L6474_EL_POS);
      
      sprintf(bufferCommand,"%s %d,%d %d %d %d %d\n","POSX",x1,x2,y1,y2,z1,z2);      
      sendString(bufferCommand);
  
}
void SendConfig(unsigned int isw){
      unsigned int x1,x2;
	    unsigned int y1,y2;
		  unsigned int z1,z2;
		  float tvx,tvy,tvz;
		  unsigned int stpx,stpy,stpz;
		
	    unsigned int ochx,ochy,ochz;
	
      char bufferCommand[128] ;
      z1=motor3->get_parameter(L6474_ABS_POS);      
      z2=motor3->get_parameter(L6474_EL_POS);
	
	    y1=motor2->get_parameter(L6474_ABS_POS);      
      y2=motor2->get_parameter(L6474_EL_POS);
	
	    x1=motor1->get_parameter(L6474_ABS_POS);      
      x2=motor1->get_parameter(L6474_EL_POS);
	
      tvx=motor1->get_parameter(L6474_TVAL);
		  tvy=motor2->get_parameter(L6474_TVAL);
		  tvz=motor3->get_parameter(L6474_TVAL);
	
	    ochx=motor1->get_parameter(L6474_OCD_TH);
		  ochy=motor2->get_parameter(L6474_OCD_TH);
		  ochz=motor3->get_parameter(L6474_OCD_TH);
			
			stpx=motor1->get_parameter(L6474_STEP_MODE);
		  stpy=motor2->get_parameter(L6474_STEP_MODE);
		  stpz=motor3->get_parameter(L6474_STEP_MODE);
	
	
      sprintf(bufferCommand,"%s %d %d,%d %d %d %d %d %f %f %f %d %d %d %d %d %d\n","POWER",isw,x1,x2,y1,y2,z1,z2,tvx,tvy,tvz,ochx,ochy,ochz,stpx,stpy,stpz);      
			
      sendString(bufferCommand);
  
}
void SendConfig2(){
      unsigned int x1,x2;
	    unsigned int y1,y2;
		  unsigned int z1,z2;
		  float tvx,tvy,tvz;
	    unsigned int ochx,ochy,ochz;
	    unsigned int stpx,stpy,stpz;
	
      char bufferCommand[128] ;
      z1=motor3->get_parameter(L6474_ABS_POS);      
      z2=motor3->get_parameter(L6474_EL_POS);
	
	    y1=motor2->get_parameter(L6474_ABS_POS);      
      y2=motor2->get_parameter(L6474_EL_POS);
	
	    x1=motor1->get_parameter(L6474_ABS_POS);      
      x2=motor1->get_parameter(L6474_EL_POS);
	
      tvx=motor1->get_parameter(L6474_TVAL);
		  tvy=motor2->get_parameter(L6474_TVAL);
		  tvz=motor3->get_parameter(L6474_TVAL);
	
	    ochx=motor1->get_parameter(L6474_OCD_TH);
		  ochy=motor2->get_parameter(L6474_OCD_TH);
		  ochz=motor3->get_parameter(L6474_OCD_TH);
	
			stpx=motor1->get_parameter(L6474_STEP_MODE);
		  stpy=motor2->get_parameter(L6474_STEP_MODE);
		  stpz=motor3->get_parameter(L6474_STEP_MODE);
	
	
      sprintf(bufferCommand,"%s %d,%d %d %d %d %d %f %f %f %d %d %d %d %d %d\n","CONFIG",x1,x2,y1,y2,z1,z2,tvx,tvy,tvz,ochx,ochy,ochz,stpx,stpy,stpz);      
      sendString(bufferCommand);
  
}

//TO DO COME MESSAGE
void alarm_pinA4_on(){
sendString((char*)"ON A4\n");
  myled=1;
}

void alarm_pinA4_off(){
  sendString((char*)"OFF A4\n");
  myled=0;
}

void alarm_pinA3_on(){
  sendString((char*)"ON A3\n");
}

void alarm_pinA3_off(){
  sendString((char*)"OFF A3\n");
}
void alarm_pinA2_on(){
  sendString((char*)"ON A2\n");
}

void alarm_pinA2_off(){
  sendString((char*)"OFF A2\n");
}



  



	L6474 *getSelMotor(string s_idxMotor){

    //string s_idxMotor = parser.next();    
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
		return motor;
	}

	DigitalIn *getSelSwitch(string s_idxMotor){

    //string s_idxMotor = parser.next();    
    int i_idxMotor=atoi(s_idxMotor.c_str());    
    DigitalIn * pinSW;
    switch (i_idxMotor)
    {
    case 0:
			pinSW=&PinA2;
      break;
    case 1:
      pinSW=&PinA3;
      break;
    case 2:
      pinSW=&PinA4;
      break;
		case 3:
      break;
    }
		return pinSW;
	}

	
	int getSelSwitchState(string s_idxMotor){

    //string s_idxMotor = parser.next();    
    int i_idxMotor=atoi(s_idxMotor.c_str());    
    int state=0;
    switch (i_idxMotor)
    {
    case 0:
			state=0;
      break;
    case 1:
      state=0;
      break;
    case 2:
      state=1;
      break;
		case 3:
      break;
    }
		return state;
	}

	

	void processLine(string line){
  
  test=(char *)line.c_str();
  Tokenizer parser(line," ,");
  string command = parser.next();
		
	
	
	if (command=="STATUS"){
		SendStatus();    
  }
  
	if (command=="ERROR"){
		//string s1 = parser.next();    
		//string s2 = parser.next();    
		//string s3 = parser.next();        
		SendStatus(); 
  }
  
  if (command=="POWER"){
    
		
    string SW = parser.next();    
    int iSW=atoi(SW.c_str());    
    if (iSW==0){
			motor1->disable();
			motor2->disable();
      motor3->disable();
      POWER=1;
    }
    else
			{
		
			motor1->disable();
			motor2->disable();
      motor3->disable();
    				
      POWER=0;
      Thread::wait(5000);
				
  		motor3->set_parameter(L6474_OCD_TH,L6474_OCD_TH_1500mA);       
      motor3->set_parameter(L6474_TVAL,1200);

	  	motor2->set_parameter(L6474_OCD_TH,L6474_OCD_TH_2625mA);       
      motor2->set_parameter(L6474_TVAL,2000);
      				
			motor1->set_parameter(L6474_OCD_TH,L6474_OCD_TH_2625mA);       
      motor1->set_parameter(L6474_TVAL,2000);
      
			
      motor3->set_step_mode(StepperMotor::STEP_MODE_1_16);      
			motor3->set_parameter(L6474_EL_POS,0);
			
			motor2->set_step_mode(StepperMotor::STEP_MODE_1_16);      
			motor2->set_parameter(L6474_EL_POS,0);
      
			motor1->set_step_mode(StepperMotor::STEP_MODE_1_16);      
			motor1->set_parameter(L6474_EL_POS,0);
      
      motor1->enable();
			motor2->enable();
      motor3->enable();
      
			
			motor1->set_direction(FORWARD); 
			motor2->set_direction(FORWARD); 
      motor3->set_direction(FORWARD); 
			
			mode=motor2->get_parameter(L6474_ALARM_EN);
      
    }
		SendConfig(iSW);
		SendStatus();
    
    
    
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
          wait_us(500);
        }
    } 
    SendPos();
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
      wait_us(500);
      }
    }
    
    SendPos();
  }
  
  if (command=="HOME"){
    
    string s_idxMotor = parser.next();    
    int i_idxMotor=atoi(s_idxMotor.c_str());    
    L6474 *motor;
		DigitalIn * pinSW;
		int State=0;
		
    switch (i_idxMotor)
    {
    case 0:
      motor=motor1;
		  pinSW=&PinA2;
			State=0;
      break;
    case 1:
      motor=motor2;
		  pinSW=&PinA3;
			State=0;
      break;
    case 2:
      motor=motor3;
			pinSW=&PinA4;
			State=1;
      break;
    }
		
		if (i_idxMotor==3){
		motor1->set_direction(FORWARD); 
		motor2->set_direction(FORWARD); 
		motor3->set_direction(FORWARD); 
    while (true)
    {
			  osEvent evt = queueEvent.get(1);
			  if (evt.status == osEventMessage){
					 break;
				 }
				if (PinA4==1 && PinA3==1  && PinA2==1)
          break;
				
				if (PinA2==0) 
        {         
					motor1->Step();
        }
        if (PinA4==0) 
        {         
					motor3->Step();
        }
				if (PinA3==0) 
        {         
					motor2->Step();
        }
				SendPos();
				
    }
    
		}
		else
		{
    motor->set_direction(BACKWARD); 
    while (true)
    {
			  osEvent evt = queueEvent.get(1);
			   if (evt.status == osEventMessage){
					 break;
				 }
        if (*pinSW==State) 
        {
          motor->Step();
         SendPos();
        }
        else
          break;
      
    }
		motor->set_parameter(L6474_ABS_POS ,0);      
		
		
    SendPos();
	}
    
    
  }

	if (command=="ENABLE"){
		
		  L6474 *motor=getSelMotor(parser.next());
      string s_val = parser.next(); 
		  int state=atoi(s_val.c_str());    
      if (state==0)
				motor->disable();
		  if (state==1)
				motor->enable();
  		sprintf(bufferCommand,"%s %s %d ","INFO ENABLE OK ",line.c_str(),state);         
      sendString(bufferCommand);
			SendStatus();
  }
  
  if (command=="TVAL"){
		
		  L6474 *motor=getSelMotor(parser.next());
      string s_val = parser.next(); 
		  int fTo=atof(s_val.c_str());    
      if (fTo<=2800)
				{
				motor->set_parameter(L6474_TVAL,fTo);      
				SendConfig2();
				sprintf(bufferCommand,"%s %s %d ","INFO TVAL OK ",line.c_str(),fTo);    
      }
      else
        sprintf(bufferCommand,"%s %s","INFO TVAL KO",line.c_str());    
      
      sendString(bufferCommand);
  }
  
  
  if (command=="STEP_MODE"){
    
		L6474 *motor=getSelMotor(parser.next());
		
    string s_step  = parser.next();    
    
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
      motor->set_step_mode(mode);
			SendConfig2();
      sprintf(bufferCommand,"%s %s","OK",line.c_str());    
      sendString(bufferCommand);
    }
    else
    {
      sprintf(bufferCommand,"%s %s","KO",line.c_str());    
      sendString(bufferCommand);
    }
    
    
    
    
    
  }
  
  if (command=="GOTO"){
    
		string s_motor=parser.next();
		
		L6474     *motor =getSelMotor (s_motor);
		DigitalIn *sw    =getSelSwitch(s_motor);
		int State        =getSelSwitchState(s_motor);
    int CurrX=motor->get_position();         
		
    string TO = parser.next();    
		
    int iTo=atoi(TO.c_str());    
		
    int diff=abs(iTo-CurrX);    
    if (CurrX>iTo)
    {
      motor->set_direction(BACKWARD);  
      while (true)
      {
        if(diff==0) break;
        if (*sw==State) {  
        motor->Step();
				Thread::wait(1);
        diff=diff-1;
				}
				else
					break;
      }   
    }
    else if (iTo>CurrX)
    {
      motor->set_direction(FORWARD);  
      while (true)
      {
        if(diff==0) break;
     
        motor->Step();
				Thread::wait(1);
        diff=diff-1;
      }         
    }    
    
    SendPos();
    
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
//  printf("Error %d detected\r\n\n", error);	
	sendString((char*)"INFO ERROR\n");
  /* Infinite loop */

}




//LAST
#ifdef ALIVE
void send_alive() {
	
	   alive.detach();
     sendString("INFO ALIVE");
	   alive.attach(&send_alive, 5.0);
}
#endif

void int_error_handler()
{
	unsigned int statusRegister1 = motor1->get_status();
	unsigned int statusRegister2 = motor2->get_status();
	unsigned int statusRegister3 = motor3->get_status();
	//message_t  *Message = mpool.alloc();		
	//sprintf(Message->message,"ERROR %d %d %d",statusRegister1,statusRegister2,statusRegister3);
	//sprintf(Message->message,"ERROR");
	//queueMessage.put(Message); 
	SendError(statusRegister1,statusRegister2,statusRegister3);
}


void newInput()
{
    Console.attach(NULL);    //detach the ISR to prevent recursive calls
    osSignalSet(ISRthreadId, 0x01);
}


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
							   if (strcmp(test_line,"SHUTDOWN")==0)  
										queueEvent.put(&message);
								 else
									 
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
			  sendString((char*)"INFO ALIVE");
        osEvent evt = queueMessage.get(5000);
        if (evt.status == osEventMessage) {
            message_t *Message = (message_t*)evt.value.p;          
					char * test=Message->message		;			
						processLine(Message->message);
            mpool.free(Message);
        }
    }

}



/* Main ----------------------------------------------------------------------*/

int main()
{

  //alive.attach(&send_alive, 5.0);
	
	EventQueue *queue = mbed_event_queue();
	
	POWER=1;
	
  sendString((char*)"INFO START WITH POWER OFF");
	
  wait_ms(4000);

  motor1 = new L6474(D2, D8, D7, D9, D10, dev_spi);
#ifdef M2
  motor2 = new L6474(D2, D8, D4, D3, D10, dev_spi);
#ifdef M3
  motor3 = new L6474(D2, D8, D5, D6, D10, dev_spi);
#endif
#endif



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
	
  
  InterA4.rise(&alarm_pinA4_on);

  InterA4.fall(&alarm_pinA4_off);

  InterA3.rise(&alarm_pinA3_on);

  InterA3.fall(&alarm_pinA3_off);

	InterA2.rise(&alarm_pinA2_on);

  InterA2.fall(&alarm_pinA2_off);

	
  InterA4.enable_irq();
	
	motor1->attach_error_handler(my_error_handler);
	motor2->attach_error_handler(my_error_handler);
	motor3->attach_error_handler(my_error_handler);

	motor3->flag_irq.fall(queue->event(int_error_handler));
	
	
  
	
	main_th();
	
	
}

