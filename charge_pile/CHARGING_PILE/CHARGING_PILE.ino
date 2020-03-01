#include <avr/io.h>
#include <avr/interrupt.h>

unsigned char IR1[5]= {0xA1,0xF1,0x11,0x11,0x11};
unsigned char IR2[5]= {0xA1,0xF1,0x22,0x22,0x22};
unsigned char IR3[5]= {0xA1,0xF1,0x33,0x33,0x33};
unsigned char IR4[5]= {0xA1,0xF1,0x44,0x44,0x44};
unsigned char IR5[5]= {0xA1,0xF1,0x55,0x55,0x55};
unsigned char IR6[5]= {0xA1,0xF1,0x66,0x66,0x66};


#define  Red_PIN 10               //全彩LED灯Red端口接PWM针脚8；
#define  Green_PIN 12             //全彩LED灯Green端口接PWM针脚12；
#define  Blue_PIN 8               //全彩LED灯Blue端口接PWM针脚10；
#define  Motor_PIN 7              //风扇电机输出端口接PWM针脚7；
#define  Switch1_PIN  5           //光电开关1 OUT端口接数字针脚5
#define  Switch2_PIN  6           //光电开关2 OUT端口接数字针脚6
#define  Automatic_PIN  11        //自动充电信号输出端口接数字针脚11；
#define  EMERGENCY_PIN  18        //定义中断（手动充电信号）输入端口接数字针脚18；

int INDEX=0;
unsigned char RECIVE_BUFFER[3]={0x00,0x00,0x00};


int STATE;

#define EMERGENCY 0x01
#define FREE 0x02
#define RECIEVED_IR1 0x03
#define SENSOR_TOUCHED 0x04
#define RECIEVED_IR4 0x05

bool IS_FIRST_FREE;
bool IS_FIRST_EMERGENCY;
bool IS_FIRST_RECIEVED_IR1;
bool IS_FIRST_RECIEVED_IR4;
bool IS_FIRST_SENSOR_TOUCHED;



#define TIME_STOP 10			//Define the time for the robot stopping. At the last two seconds the ATMOSPHERE_LED will blink green.
//bool IS_STOPPING_BEGINNING;
int Timer;						//Timer is zero every time the sensor is touched
//Giving Timer5 1Hz frequency. Period is 16*10^6/prescaler*(OCR5AINIT-TCNT5INIT).
//The Prescaler is set 1024 with CS52 and CS50 set 1.
#define TCNT5INIT 49910
#define OCR5AINIT 0xFFFF

void TIMER5_INIT()  //1Hz
{
	TCCR5A=0;
	TCCR5B=0;
	//TCNT5=TCNT5INIT;
	OCR5A=OCR5AINIT;
	TCCR5B|=_BV(WGM52);
	TCCR5B|=_BV(CS52)|_BV(CS50);
}

void USART3_INIT()
{
	//USART2 setup
	UBRR3 = 103; // for configuring baud rate of 9600bps    
	UCSR3C |= (1 << UCSZ31) | (1 << UCSZ30); // Use 8-bit character sizes     
	UCSR3B |= (1 << RXEN3) | (1 << TXEN3) | (1 << RXCIE3);  // Turn on the transmission, reception, and Receive interrupt          
}

void USART0_INIT()
{
	//USART0 setup
	UBRR0 = 103; // for configuring baud rate of 9600bps    
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);// Use 8-bit character sizes     
	UCSR0B |= (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0); // Turn on the transmission, reception, and Receive interrupt          
	//UCSR0B |= (1 << RXEN0) | (1 << RXCIE0); // Turn on the transmission, reception, and Receive interrupt  
}


void LED_SWITCH_MOTOR_CHARGING_INIT()
{
	pinMode(Switch1_PIN, INPUT);         //将光电开关1引脚设置为输入模式（硬件下拉）
	pinMode(Switch2_PIN, INPUT);         //将光电开关2引脚设置为输入模式（硬件下拉）
	//pinMode(Interrupt_PIN, INPUT);       //定义中断引脚18为输入模式
	pinMode(Automatic_PIN, OUTPUT);      //将自动充电信号引脚设置为输出模式
	digitalWrite(Automatic_PIN,HIGH);
	pinMode(Motor_PIN, OUTPUT);          //将风扇引脚设置为输出模式      
	digitalWrite(Automatic_PIN,LOW);
	pinMode(Red_PIN, OUTPUT); 
	digitalWrite(Red_PIN,LOW);
	pinMode(Green_PIN, OUTPUT);
	digitalWrite(Green_PIN,LOW);	
	pinMode(Blue_PIN, OUTPUT);
	digitalWrite(Blue_PIN,LOW);
	pinMode(EMERGENCY_PIN,INPUT);
	attachInterrupt(5,Emergency_Or_Not,CHANGE);
	if(digitalRead(EMERGENCY_PIN)==HIGH)
	{
		STATE=EMERGENCY;
		IS_FIRST_EMERGENCY=true;
	}
	else
	{
		STATE=FREE;
		IS_FIRST_FREE=true;
	}
}
	
ISR(USART3_RX_vect)
{
	cli();
	if(INDEX<2)
	{
		RECIVE_BUFFER[INDEX]=UDR3;
		INDEX++;
	}
	else
	{
		RECIVE_BUFFER[INDEX]=UDR3;
		if((RECIVE_BUFFER[0]==0x11)&&(RECIVE_BUFFER[2]==0x11)&&(RECIVE_BUFFER[2]==0x11))
		{
			UCSR3B&=~_BV(RXCIE3);
			//USART3_TX(IR2,5);
			STATE=RECIEVED_IR1;
			IS_FIRST_RECIEVED_IR1=true;
		
		}
		else if((RECIVE_BUFFER[0]==0x44)&&(RECIVE_BUFFER[2]==0x44)&&(RECIVE_BUFFER[2]==0x44))
		{
			//USART3_TX(IR2,5);
			UCSR3B&=~_BV(RXCIE3);
			STATE=RECIEVED_IR4;
			IS_FIRST_RECIEVED_IR4=true;
		}
		else if((RECIVE_BUFFER[0]==0x66)&&(RECIVE_BUFFER[2]==0x66)&&(RECIVE_BUFFER[2]==0x66))
		{
			//USART3_TX(IR2,5);
			UCSR3B&=~_BV(RXCIE3);
			STATE=FREE;
			IS_FIRST_FREE=true;
		}
		INDEX=0;
	}
	sei();
}

ISR(TIMER5_OVF_vect)
{
	Timer++;
	if(Timer==TIME_STOP)
	{
		STATE=FREE;
		IS_FIRST_FREE=true;
		TIMSK5&=~_BV(TOIE5);
	}
	TCNT5=TCNT5INIT;
}

void Sensor_touching()
{
	delay(20);
	if((digitalRead(20)==LOW)&&(digitalRead(21)==LOW))
	{
		STATE=SENSOR_TOUCHED;
		IS_FIRST_SENSOR_TOUCHED=true;
		detachInterrupt(2);
		detachInterrupt(3);
	}
}
void Emergency_Or_Not()
{
	if(digitalRead(EMERGENCY_PIN)==HIGH)
	{
		digitalWrite(Automatic_PIN,HIGH);
		STATE=EMERGENCY;
		IS_FIRST_EMERGENCY=true;
		cli();
		UCSR3B&=~_BV(RXCIE3);
		sei();
	}
	else
	{
		STATE=FREE;
		IS_FIRST_FREE=true;
	}
}


void Not_touched()
{
	delay(5);
	if((digitalRead(20)==HIGH)||(digitalRead(20)==HIGH))
	{
		detachInterrupt(2);
		detachInterrupt(3);
		STATE=FREE;
		IS_FIRST_FREE=true;
	}
}

void USART0_TX(uint8_t * Data, uint16_t Length)
{
	uint16_t i;
	for(i=0;i<Length;i++)
	{
		while(!(UCSR0A&(1<<UDRE0)))
		{
			;
		}
		UDR0=Data[i];
	}
}

void USART3_TX(uint8_t * Data, uint16_t Length)
{
	uint16_t i;
	for(i=0;i<Length;i++)
	{
		while(!(UCSR3A&(1<<UDRE3)))
		{
			;
		}
		UDR3=Data[i];
	}
}

void Emergency()
{
	if(IS_FIRST_EMERGENCY==true)
	{
		digitalWrite(Automatic_PIN,HIGH);
		cli();
		UCSR3B&=~_BV(RXCIE3);
		sei();
		digitalWrite(Red_PIN,HIGH);
		digitalWrite(Green_PIN,LOW);
		digitalWrite(Blue_PIN,LOW);
		IS_FIRST_EMERGENCY=false;
	}
}

void Free()
{
	if(IS_FIRST_FREE==true)
	{
		digitalWrite(Automatic_PIN,HIGH);
		IS_FIRST_FREE=false;
		digitalWrite(Red_PIN,LOW);
		digitalWrite(Green_PIN,HIGH);
		digitalWrite(Blue_PIN,HIGH);
		cli();
		UCSR3B|=_BV(RXCIE3);
		sei();
	}
}

void Recieved_IR1()
{

	USART3_TX(IR2,5);
	
	if(IS_FIRST_RECIEVED_IR1==true)
	{
		//cli();
		//PCICR=B00000001;
		//PCMSK0=(1<<PCINT6)|(1 << PCINT7);
		//sei();
		IS_FIRST_RECIEVED_IR1=false;
		IS_FIRST_RECIEVED_IR1=false;
		digitalWrite(Red_PIN,LOW);
		digitalWrite(Green_PIN,LOW);
		digitalWrite(Blue_PIN,HIGH);
		attachInterrupt(2,Sensor_touching,FALLING);
		attachInterrupt(3,Sensor_touching,FALLING);
		
	}
}



void Sensor_touched()
{
	if(IS_FIRST_SENSOR_TOUCHED==true)
	{
		IS_FIRST_SENSOR_TOUCHED=false;
		digitalWrite(Red_PIN,LOW);
		analogWrite(Green_PIN,200);
		analogWrite(Blue_PIN,200);
		cli();
		UCSR3B|=(1<<RXCIE3); 
		Timer=0;
		TCNT5=TCNT5INIT;
		TIMSK5|=_BV(TOIE5);
		sei();
	}
	USART3_TX(IR3,5);
	
}

void Recieved_IR4()
{
	if(IS_FIRST_RECIEVED_IR4==true)
	{
		IS_FIRST_RECIEVED_IR4=false;
		digitalWrite(Green_PIN,LOW);
		digitalWrite(Blue_PIN,LOW);
		digitalWrite(Red_PIN,HIGH);
		digitalWrite(Automatic_PIN,LOW);
		cli();
		UCSR3B|=(1<<RXCIE3);
		attachInterrupt(2,Not_touched,RISING);
		attachInterrupt(3,Not_touched,RISING);
		sei();
	}
	USART0_TX(IR5,5);
}


void setup(){
	
	cli();
    USART0_INIT();
	USART3_INIT();
	LED_SWITCH_MOTOR_CHARGING_INIT();
	TIMER5_INIT();
	sei();
}


void loop()
{ 
	switch(STATE)
	{
		case EMERGENCY:
			Emergency();
			break;
		case FREE:
			Free();
			break;
		case RECIEVED_IR1:
			Recieved_IR1();
			break;
		case SENSOR_TOUCHED:
			Sensor_touched();
			break;
		case RECIEVED_IR4:
			Recieved_IR4();
			break;
		
		
	}
}
