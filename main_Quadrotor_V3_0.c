/*
 *Version: 0.0: the first version can flight
 *Work time: July - august, 2015
 *Author: Pham Nhat Tan
 *Description: This version can flight only by remote control, yaw angle's still not stable.
 */

#include "config.h"

void controller();
void communication();
void display_com();
//void task_200Hz();
void task_100Hz();
void task_50Hz();
void task_20Hz();
void task_IMU();
void task_RF();
void mode_selection();
void Stable_Controller(float sampling_time);
void height_controller(float sampling_time);
void postion_controller(float sampling_time);
void task_sonar();
void task_camera();
void display_system_infor();

/*
 *  autonomous fly
 */
#define DELTA_ERROR_POSITION_ 	45
#define DELTA_ERROR_Z1_			30
#define AUTONOMOUS_ATTITUDE_ 	420
#define POINT_COLOR_NUMBER_ 	21
#define NUMBER_OF_TASK_			25
#define DELTA_TAKE_OFF_Z1_		5	//0.05(20Hz)*350(attitude)/3(s)
#define DELTA_LANDING_Z1_		3//0.05(20Hz)*350(attitude)/3(s)
//#define TASK_NUMBER_END_		10
#define TASK_NUMBER_END_		5
#define TASK_NUMBER_MAIN_END_		23

unsigned char task_number=0;
//unsigned char task_color[POINT_COLOR_NUMBER_]={CAPTURE_BLUE_,3,CAPTURE_GREEN_,CAPTURE_RED_,CAPTURE_BLUE_,CAPTURE_GREEN_,CAPTURE_RED_,CAPTURE_BLUE_,CAPTURE_GREEN_,CAPTURE_RED_,CAPTURE_BLUE_};//,CAPTURE_RED_,CAPTURE_BLUE_};
//unsigned char task_color[POINT_COLOR_NUMBER_]={CAPTURE_BLUE_,CAPTURE_RED_,CAPTURE_GREEN_,3,CAPTURE_BLUE_};//,CAPTURE_RED_,CAPTURE_BLUE_};
//unsigned char task_color[POINT_COLOR_NUMBER_]={CAPTURE_RED_};//,CAPTURE_RED_,CAPTURE_BLUE_};

unsigned char task_color[NUMBER_OF_TASK_]={CAPTURE_RED_,CAPTURE_RED_,CAPTURE_GREEN_,CAPTURE_BLUE_};

unsigned char flag_task_finish[ NUMBER_OF_TASK_];
unsigned char flag_task_init[ NUMBER_OF_TASK_];

unsigned char flag_takeoff_20Hz=0;
unsigned char flag_landing_20Hz=0;

void set_color_capture(unsigned char color_tag);
void autonomous_task();
void autonomous_takeoff();
void autonomous_takeoff_20Hz();

void autonomous_go_into_battlefield();
void autonomous_switchingPoint(unsigned char i);
void autonomous_go_out_battlefield();
void autonomous_landing();
void autonomous_end();
void autonomous_reset();
void autonomous_task_1();
void autonomous_task_2();
void autonomous_task_main();

//void autonomous_takeoff_20Hz(){
//	if( flag_takeoff_20Hz==1)
//	{
//		if(PID_z.set_point<AUTONOMOUS_ATTITUDE_)
//			PID_z.set_point+=DELTA_TAKE_OFF_Z1_;
//	}
//
//}
void autonomous_go_into_battlefield(){

	float delta_x=abs(PID_x.set_point-Camera.X);
	float delta_y=abs(PID_y.set_point-Camera.Y);
	if((delta_x+delta_y)<DELTA_ERROR_POSITION_)
	{
		flag_task_finish[task_number]=1;
		//		flag_takeoff_20Hz=0;
	}
	else
	{

	}
}

void autonomous_go_out_battlefield(){

}
void set_color_capture(unsigned char color_tag){
	if(color_tag==CAPTURE_BLUE_)
	{
		Serial_Sendcommand_Camera("b");
		color_capture=CAPTURE_BLUE_;
	}
	else if(color_tag==CAPTURE_GREEN_)
	{
		Serial_Sendcommand_Camera("g");
		color_capture=CAPTURE_GREEN_;
	}
	else if(color_tag==CAPTURE_RED_)
	{
		Serial_Sendcommand_Camera("r");
		color_capture=CAPTURE_RED_;
	}
}

void autonomous_reset()
{
	task_number=0;
	unsigned char i=0;
	for (i=0;i<POINT_COLOR_NUMBER_;i++)
	{
		flag_task_finish[i]=0;
		flag_task_init[i]=0;
	}
	PID_x.set_point=0;
	PID_y.set_point=0;
	PID_z.set_point=0;
	Socket.throttle_offset=THROTTLE_OFFSET;
	flag_landing_20Hz=0;
	flag_takeoff_20Hz=0;
	SerialPutStrLn(UART_COM_2_CONTROLLER_,"auto reset");
	set_color_capture(task_color[0]);
}

void autonomous_takeoff(){

	float delta_z1=abs(Sonar_module.attitude-AUTONOMOUS_ATTITUDE_);
	float delta_x=abs(PID_x.set_point-Camera.X);
	float delta_y=abs(PID_y.set_point-Camera.Y);

	if((delta_z1<DELTA_ERROR_Z1_)&&((delta_x+delta_y)<DELTA_ERROR_POSITION_))
	{
		flag_task_finish[0]=1;
		flag_takeoff_20Hz=0;
	}
	else
	{


	}
}



void autonomous_switchingPoint(unsigned char i){
	float delta_x=abs(PID_x.set_point-Camera.X);
	float delta_y=abs(PID_y.set_point-Camera.Y);
	if((delta_x+delta_y)<DELTA_ERROR_POSITION_)
	{
		flag_task_finish[i]=1;
		//		flag_takeoff_20Hz=0;
	}
	else
	{
	}
}

void autonomous_landing(){

}
void autonomous_end(){

}

void autonomous_task_2(){
	if(Flag.cameraUpdate_error==0)
	{
		if(task_number==0)
		{
			if(flag_task_init[task_number]==0)
			{
				flag_task_init[task_number]=1;
				/*
				 * your code begin from here
				 */
				set_color_capture(task_color[task_number]);
				PID_x.set_point=0;
				PID_y.set_point=0;
				PID_z.set_point=0;
				Socket.throttle_offset=55;//30;//50;
				SerialPutStrLn(UART_COM_2_CONTROLLER_,"task 0 init");
				flag_takeoff_20Hz=1;
			}
			else if(flag_color_capture==task_color[task_number])
			{
				if(flag_task_finish[task_number])
				{
					task_number++;
					/*
					 * your code  begin from heare
					 */
					SerialPutStrLn(UART_COM_2_CONTROLLER_,"task 0 finish");
				}
				else
				{
					autonomous_takeoff();
				}
			}
		}
		//		else if(task_number==1)
		//		{
		//			if(flag_task_init[task_number]==0)
		//			{
		//				flag_task_init[task_number]=1;
		//				/*
		//				 * your code begin from here
		//				 */
		//				//			set_color_capture(task_color[task_number]);
		//				PID_x.set_point=200;
		//				//			PID_y.set_point=0;
		//				SerialPutStrLn(UART_COM_2_CONTROLLER_,"task 1 int");
		//			}
		//			else if(flag_task_finish[task_number])
		//			{
		//				task_number=2;
		//				/*
		//				 * your code  begin from heare
		//				 */
		//				SerialPutStrLn(UART_COM_2_CONTROLLER_,"task 1 finish");
		//			}
		//			else
		//				autonomous_go_into_battlefield();
		//		}
		else if(task_number<(TASK_NUMBER_END_-2))
		{
			if(flag_task_init[task_number]==0)
			{
				flag_task_init[task_number]=1;
				/*
				 * your code begin from here
				 */
				PID_x.set_point=0;
				PID_y.set_point=0;

				set_color_capture(task_color[task_number]);

				char buffer[10];
				int2num(task_number,buffer);
				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"task ");
				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
				SerialPutStrLn(UART_COM_2_CONTROLLER_," init");
			}
			else if(flag_color_capture==task_color[task_number])
			{
				if(flag_task_finish[task_number])
				{
					/*
					 * your code begin from here
					 */
					char buffer[10];
					int2num(task_number,buffer);
					SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"task ");
					SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
					SerialPutStrLn(UART_COM_2_CONTROLLER_," finish");
					task_number++;
				}
				else
					autonomous_switchingPoint(task_number);
			}
		}
		else if(task_number==(TASK_NUMBER_END_-2))
		{
			if(flag_task_init[task_number]==0)
			{
				flag_task_init[task_number]=1;
				/*
				 * your code begin from here
				 */
				//			set_color_capture(task_color[task_number]);
				PID_x.set_point=200;
				//			PID_y.set_point=0;
				SerialPutStrLn(UART_COM_2_CONTROLLER_,"task go out int");
			}
			else if(flag_task_finish[task_number])
			{
				task_number++;
				/*
				 * your code  begin from heare
				 */
				SerialPutStrLn(UART_COM_2_CONTROLLER_,"task go out finish");
			}
			else
				autonomous_go_into_battlefield();
		}
		else if(task_number==(TASK_NUMBER_END_-1))
		{
			if(flag_task_init[task_number]==0)
			{
				flag_task_init[task_number]=1;
				/*
				 * your code begin from here
				 */
				PID_x.set_point=0;
				PID_y.set_point=0;
				set_color_capture(task_color[task_number]);

				char buffer[10];
				int2num(task_number,buffer);
				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"task ");
				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
				SerialPutStrLn(UART_COM_2_CONTROLLER_," init");
			}
			else if(flag_color_capture==task_color[task_number])
			{
				if(flag_task_finish[task_number])
				{
					/*
					 * your code begin from here
					 */
					char buffer[10];
					int2num(task_number,buffer);
					SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"task ");
					SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
					SerialPutStrLn(UART_COM_2_CONTROLLER_," finish");
					task_number++;
				}
				else
					autonomous_switchingPoint(task_number);
			}
		}
		else if(task_number==TASK_NUMBER_END_)
		{
			if(flag_task_init[TASK_NUMBER_END_]==0)
			{
				flag_task_init[TASK_NUMBER_END_]=1;
				/*
				 * your code begin from here
				 */
				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"landing..");
				set_color_capture(task_color[task_number]);
				flag_landing_20Hz=1;

				//				Flag.Autonomous=0;
			}
			else if(flag_color_capture==task_color[task_number])
			{
				if(flag_task_finish[TASK_NUMBER_END_])
				{
					task_number++;
					/*
					 * your code begin from here
					 */
					//			autonomous_end();
				}
				else
					autonomous_landing();
			}
		}

	}
}

void autonomous_task_main(){
	if(Flag.cameraUpdate_error==0)
	{
		if(task_number==TASK_TAKEUP_)
		{
			if(flag_task_init[task_number]==0)
			{
				flag_task_init[task_number]=1;
				/*
				 * your code begin from here
				 */
				set_color_capture(task_color[task_number]);
				PID_x.set_point=0;
				PID_y.set_point=0;
				PID_z.set_point=0;
				Socket.throttle_offset=55;//30;//50;
				SerialPutStrLn(UART_COM_2_CONTROLLER_,"task 0 init take up");
				flag_takeoff_20Hz=1;
			}
			else if(flag_color_capture==task_color[task_number])
			{
				if(flag_task_finish[task_number])
				{
					task_number=TASK_IDLE_;
					/*
					 * your code  begin from heare
					 */
					SerialPutStrLn(UART_COM_2_CONTROLLER_,"task 0 finish");
				}
				else
				{
					autonomous_takeoff();
				}
			}
		}
		else if(task_number==TASK_RED_)
		{
			if(flag_task_init[task_number]==0)
			{
				flag_task_init[task_number]=1;
				/*
				 * your code begin from here
				 */
				PID_x.set_point=0;
				PID_y.set_point=0;
				set_color_capture(task_color[task_number]);

				SerialPutStrLn(UART_COM_2_CONTROLLER_,"task red init");

			}
			else if(flag_color_capture==task_color[task_number])
			{
				if(flag_task_finish[task_number])
				{
					/*
					 * your code begin from here
					 */
					SerialPutStrLn(UART_COM_2_CONTROLLER_,"task red finish");
					task_number=TASK_IDLE_;
				}
				else
					autonomous_switchingPoint(task_number);
			}
		}
		else if(task_number==TASK_GREEN_)
			{
				if(flag_task_init[task_number]==0)
				{
					flag_task_init[task_number]=1;
					/*
					 * your code begin from here
					 */
					PID_x.set_point=0;
					PID_y.set_point=0;
					set_color_capture(task_color[task_number]);

					SerialPutStrLn(UART_COM_2_CONTROLLER_,"task green init");

				}
				else if(flag_color_capture==task_color[task_number])
				{
					if(flag_task_finish[task_number])
					{
						/*
						 * your code begin from here
						 */
						SerialPutStrLn(UART_COM_2_CONTROLLER_,"task green finish");
						task_number=TASK_IDLE_;
					}
					else
						autonomous_switchingPoint(task_number);
				}
			}
		else if(task_number==TASK_BLUE_)
			{
				if(flag_task_init[task_number]==0)
				{
					flag_task_init[task_number]=1;
					/*
					 * your code begin from here
					 */
					PID_x.set_point=0;
					PID_y.set_point=0;
					set_color_capture(task_color[task_number]);

					SerialPutStrLn(UART_COM_2_CONTROLLER_,"task blue init");

				}
				else if(flag_color_capture==task_color[task_number])
				{
					if(flag_task_finish[task_number])
					{
						/*
						 * your code begin from here
						 */
						SerialPutStrLn(UART_COM_2_CONTROLLER_,"task blue finish");
						task_number=TASK_IDLE_;
					}
					else
						autonomous_switchingPoint(task_number);
				}
			}
//		else if(task_number==(TASK_NUMBER_MAIN_END_-2))
//		{
//			if(flag_task_init[task_number]==0)
//			{
//				flag_task_init[task_number]=1;
//				/*
//				 * your code begin from here
//				 */
//				//			set_color_capture(task_color[task_number]);
//				PID_x.set_point=200;
//				//			PID_y.set_point=0;
//				SerialPutStrLn(UART_COM_2_CONTROLLER_,"task go out init");
//			}
//			else if(flag_task_finish[task_number])
//			{
//				task_number++;
//				/*
//				 * your code  begin from heare
//				 */
//				SerialPutStrLn(UART_COM_2_CONTROLLER_,"task go out finish");
//			}
//			else
//				autonomous_go_into_battlefield();
//		}
//		else if(task_number==(TASK_NUMBER_MAIN_END_-1))
//		{
//			if(flag_task_init[task_number]==0)
//			{
//				flag_task_init[task_number]=1;
//				/*
//				 * your code begin from here
//				 */
//				PID_x.set_point=0;
//				PID_y.set_point=0;
//				set_color_capture(CAPTURE_BLUE_);
//
//
//				SerialPutStrLn(UART_COM_2_CONTROLLER_,"final point init");
//			}
//			else if(flag_color_capture==CAPTURE_BLUE_)
//			{
//				if(flag_task_finish[task_number])
//				{
//					/*
//					 * your code begin from here
//					 */
//					SerialPutStrLn(UART_COM_2_CONTROLLER_,"final point finish");
//					task_number++;
//				}
//				else
//					autonomous_switchingPoint(task_number);
//			}
//		}
		else if(task_number==TASK_LANDING_)
		{
			if(flag_task_init[task_number]==0)
			{
				flag_task_init[task_number]=1;
				/*
				 * your code begin from here
				 */
				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"landing..");
//				set_color_capture(CAPTURE_BLUE_);
				flag_landing_20Hz=1;

				//				Flag.Autonomous=0;
			}
			else if(flag_color_capture==task_color[task_number])
			{
				if(flag_task_finish[task_number])
				{
					task_number++;
					/*
					 * your code begin from here
					 */
					//			autonomous_end();
				}
				else
					autonomous_landing();
			}
		}
	}
}
void autonomous_task_1(){
	if(Flag.cameraUpdate_error==0)
	{
		if(task_number==0)
		{
			if(flag_task_init[task_number]==0)
			{
				flag_task_init[task_number]=1;
				/*
				 * your code begin from here
				 */
				set_color_capture(task_color[task_number]);
				PID_x.set_point=0;
				PID_y.set_point=0;
				PID_z.set_point=0;
				Socket.throttle_offset=55;//30;//50;
				SerialPutStrLn(UART_COM_2_CONTROLLER_,"task 0 init");
				flag_takeoff_20Hz=1;
			}
			else if(flag_color_capture==task_color[task_number])
			{
				if(flag_task_finish[task_number])
				{
					task_number=1;
					/*
					 * your code  begin from heare
					 */
					SerialPutStrLn(UART_COM_2_CONTROLLER_,"task 0 finish");
				}
				else
				{
					autonomous_takeoff();
				}
			}
		}
	}
}
void autonomous_task(){

	if(Flag.cameraUpdate_error==0)
	{
		if(task_number==0)
		{
			if(flag_task_init[task_number]==0)
			{
				flag_task_init[task_number]=1;
				/*
				 * your code begin from here
				 */
				set_color_capture(task_color[task_number]);
				PID_x.set_point=0;
				PID_y.set_point=0;
				PID_z.set_point=0;
				Socket.throttle_offset=55;//30;//50;
				SerialPutStrLn(UART_COM_2_CONTROLLER_,"task 0 init");
				flag_takeoff_20Hz=1;
			}
			else if(flag_color_capture==task_color[task_number])
			{
				if(flag_task_finish[task_number])
				{
					task_number=1;
					/*
					 * your code  begin from heare
					 */
					SerialPutStrLn(UART_COM_2_CONTROLLER_,"task 0 finish");
				}
				else
				{
					autonomous_takeoff();
				}
			}
		}
		else if(task_number==1)
		{
			if(flag_task_init[task_number]==0)
			{
				flag_task_init[task_number]=1;
				/*
				 * your code begin from here
				 */
				//			set_color_capture(task_color[task_number]);
				PID_x.set_point=200;
				//			PID_y.set_point=0;
				SerialPutStrLn(UART_COM_2_CONTROLLER_,"task 1 int");
			}
			else if(flag_task_finish[task_number])
			{
				task_number=2;
				/*
				 * your code  begin from heare
				 */
				SerialPutStrLn(UART_COM_2_CONTROLLER_,"task 1 finish");
			}
			else
				autonomous_go_into_battlefield();
		}

		else if(task_number<TASK_NUMBER_END_)
		{
			if(flag_task_init[task_number]==0)
			{
				flag_task_init[task_number]=1;
				/*
				 * your code begin from here
				 */
				PID_x.set_point=0;
				PID_y.set_point=0;

				set_color_capture(task_color[task_number]);

				char buffer[10];
				int2num(task_number,buffer);
				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"task ");
				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
				SerialPutStrLn(UART_COM_2_CONTROLLER_," init");

			}
			else if(flag_color_capture==task_color[task_number])
			{
				if(flag_task_finish[task_number])
				{
					/*
					 * your code begin from here
					 */
					char buffer[10];
					int2num(task_number,buffer);
					SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"task ");
					SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
					SerialPutStrLn(UART_COM_2_CONTROLLER_," finish");
					task_number++;
				}
				else
					autonomous_switchingPoint(task_number);
			}
		}
		else if(task_number==TASK_NUMBER_END_)
		{
			if(flag_task_init[TASK_NUMBER_END_]==0)
			{
				flag_task_init[TASK_NUMBER_END_]=1;
				/*
				 * your code begin from here
				 */
				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"landing..");
				//			set_color_capture(task_color[task_number]);
				flag_landing_20Hz=1;
				//				Flag.Autonomous=0;
			}
			else if(flag_color_capture==task_color[task_number])
			{
				if(flag_task_finish[TASK_NUMBER_END_])
				{
					task_number++;
					/*
					 * your code begin from here
					 */
					//			autonomous_end();
				}
				else
					autonomous_landing();
			}
		}

	}
}

void main(void) {

	/*
	 * ================== Hardware's initialization ==========================================
	 */
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
	myIO_init();
	led(LED_RED,0);
	led(LED_BLUE,1);

	//	UART6_Init();

	//		UART0_Init();


	PPM_init();
	Timer0_init();
	SysTick_Init();
	UART1_Init();
	I2C1_Init();
	while(MPU6050_Init());
	//
	RF_init();
	Sonar_module_init();


	// timer - sensor - communication
	//		IntPrioritySet(INT_TIMER0A,0x00);// configurated in Timer0_init()
	//		IntPrioritySet(INT_GPIOE,0xA0);// configurated in MPU6050_INTpin_Init() and MaxSonar_init()
	//		IntPrioritySet(INT_GPIOD,0xC0);//configurated in RF_init()
	//		IntPrioritySet(INT_GPIOC,0xC0);//configurated in RF_init()
	//		IntPrioritySet(INT_UART1,0xE0);//configurated in UART1_init()

	/*
	 *================  software's initialization ====================================
	 */
	//
	SerialPutStrLn(UART_COM_2_CONTROLLER_,"calib gyroscope offset ...");
	while(Calib_Gyro());
	SerialPutStrLn(UART_COM_2_CONTROLLER_,"calib accelerometer offset ...");
	while(Calib_Accelerometer_Amplitude());
	SerialPutStrLn(UART_COM_2_CONTROLLER_," leave sonar module under ground for calib!!!...");
	while(Calib_Sonar_module());
	led(LED_BLUE,0);

	UART5_Init();

	SerialPutStrLn(UART_COM_2_CONTROLLER_,"config done!");
	PID_init();
	GUI_init();
	IMU_init();
	model_init();
	//	quad_model.g=MPU6050.acc_amplitude_offset;
	SysCtlDelay(6666666);
	set_color_capture(task_color[0]);
	safe_check();
#ifndef POSITION_CONTROLLER_
	Flag.test_motor=1;
#endif
	while(1)
	{
		communication();
		mode_selection();
		/*
		 * your code begin from here
		 */



		//		if(Flag.Autonomous)
		//		{
		//			if(Flag_SonarHoldAttitudeMode)
		//			{
		//				autonomous_task();
		//			}
		//			else
		//			{
		//				SerialPutStrLn(UART_COM_2_CONTROLLER_,"landing");
		//				Flag.Autonomous=0;
		//				flag_landing_20Hz=1;
		//			}
		//		}

		//
		//		if(Flag.Autonomous)
		//			autonomous_task();

		task_IMU();
		if(Flag.SonarHoldAttitudeMode)
		{
			task_sonar();
		}

		//		if(Flag.Autonomous)
		task_camera();

		task_RF();

		//		height_controller();

		/* code for position control
		 *
		 */
		task_20Hz();
		task_50Hz();
		task_100Hz();
		//		task_200Hz();


	}//end of while(1)
}//end of main

void postion_controller(float sampling_time){
	PID_y.sampling_time=sampling_time;
	PID_x.sampling_time=sampling_time;

	if(Flag.Autonomous)
	{
		PID_type_4(Camera.Y,&PID_y,0.5);
		PID_type_4(Camera.X,&PID_x,0.5);
		//		SerialPutStrLn(UART_COM_2_CONTROLLER_,"p1");
	}
	else
	{
		//		SerialPutStrLn(UART_COM_2_CONTROLLER_,"p2");
		if(RF_module.Channel_3<1100)
		{
			if(Flag.PID_p_controller==1)
			{
				Flag.PID_p_controller=0;
				PID_y.I_term=0;
				PID_x.I_term=0;
			}
			Flag.PD_p_controller=1;
			PD_type_4(Camera.Y,&PID_y,0.5);
			PD_type_4(Camera.X,&PID_x,0.5);
		}

		else{
			if(Flag.PD_p_controller==1)
			{
				Flag.PD_p_controller=0;
				PID_y.I_term=0;
				PID_x.I_term=0;
			}
			Flag.PID_p_controller=1;
			PID_type_4(Camera.Y,&PID_y,0.5);
			PID_type_4(Camera.X,&PID_x,0.5);
		}
	}


	//	if(PID_y.output>PID_Y_RANGE)
	//		Socket.input_aileron = PID_Y_RANGE;
	//	else if(PID_y.output <- PID_Y_RANGE)
	//		Socket.input_aileron = -PID_Y_RANGE;
	//	else
	//		Socket.input_aileron=PID_y.output;
	//
	//	if(PID_x.output>PID_X_RANGE)
	//		Socket.input_elevator = PID_X_RANGE;
	//	else if(PID_x.output <- PID_X_RANGE)
	//		Socket.input_elevator = -PID_X_RANGE;
	//	else
	//		Socket.input_elevator=PID_x.output;

	if(PID_y.output>pid_y_range)
		Socket.input_aileron = pid_y_range;
	else if(PID_y.output <- pid_y_range)
		Socket.input_aileron = -pid_y_range;
	else
		Socket.input_aileron=PID_y.output;

	if(PID_x.output>pid_x_range)
		Socket.input_elevator = pid_x_range;
	else if(PID_x.output <- pid_x_range)
		Socket.input_elevator = -pid_x_range;
	else
		Socket.input_elevator=PID_x.output;

}
void task_camera(){
	//	if(Flag.cameraUpdate_blue)
	//	{
	//		Flag.cameraUpdate_blue=0;
	//		/*
	//		 * your code begin from here
	//		 */
	//
	//	}
	//	if(Flag.cameraUpdate_green)
	//	{
	//		Flag.cameraUpdate_green=0;
	//		/*
	//		 * your code begin from here
	//		 */
	//
	//	}
	//
	//	if(Flag.cameraUpdate_red)
	//	{
	//		Flag.cameraUpdate_red=0;
	//		/*
	//		 * your code begin from here
	//		 */
	//	}

	if(Flag.cameraUpdate)
	{
		Flag.cameraUpdate=0;
		/*
		 * your code begin from here;
		 */


		/*
		 * update data
		 */

		Camera.OO2_Y= (Sonar_module.z1_filter+100)/IMU.cos_roll*IMU.sin_roll;
		Camera.O2I_Y=(Sonar_module.z1_filter+100)/IMU.cos_roll/IMU.cos_roll*Position.axis.Y_pixel/MAX_PIXEL_Y_*TANG_ALPHA_Y_;
		Camera.Y=Camera.OO2_Y+Camera.O2I_Y;

		Camera.OO2_X= (Sonar_module.z1_filter+100)/IMU.cos_pitch*IMU.sin_pitch;
		Camera.O2I_X=(Sonar_module.z1_filter+100)/IMU.cos_pitch/IMU.cos_pitch*Position.axis.X_pixel/MAX_PIXEL_X_*TANG_ALPHA_X_;
		Camera.X=Camera.OO2_X+Camera.O2I_X;

		/*
		 * check system
		 */

		if(Flag_SonarHoldAttitudeMode)
		{
			if(Flag.Autonomous)
				autonomous_task_main();
		}
		else
		{
			//			{
			//				SerialPutStrLn(UART_COM_2_CONTROLLER_,"landing 1");
			//				//				Flag.Autonomous=0;
			//				flag_landing_20Hz=1;
			//			}
			if(Flag.SonarHoldAttitudeMode)
			{
				flag_landing_20Hz=1;
				//				SerialPutStrLn(UART_COM_2_CONTROLLER_,"landing 2");
			}
		}


#ifdef POSITION_CONTROLLER_
		if(Flag.run_controller==1)
		{
			//			uint32_t microSecond=0;
			//			microSecond=getMicroSecond();
			//			sampling_time_second=(float)(microSecond- preMicroSecond_camera)/1000000.0;
			//			preMicroSecond_camera=microSecond;
			//			data_buffer_cn[1]=sampling_time_second;
			sampling_time_second=0.02;
			postion_controller(sampling_time_second);
			//			PID_y.sampling_time=sampling_time_second;
			//
			//			//			if(Position.blue.Y_pixel>MAX_Y_)
			//			//				Socket.input_aileron_offset=-5;
			//			//			else if(Position.blue.Y_pixel<-MAX_Y_)
			//			//				Socket.input_aileron_offset=5;
			//			//			else
			//			//				Socket.input_aileron_offset=0;
			//
			//			PID_type_4(PositionFilted.Y,&PID_y,0.6);
			//
			//			if(PID_y.output>PID_Y_RANGE)
			//				Socket.input_aileron = PID_Y_RANGE;
			//			else if(PID_y.output <- PID_Y_RANGE)
			//				Socket.input_aileron = -PID_Y_RANGE;
			//			else
			//				Socket.input_aileron=PID_y.output;


			//			PID_x.sampling_time=sampling_time_second;
			//			PID_type_4(Position.blue.X_pixel,&PID_x,0.8);
			//
			//			if(PID_x.output>PID_X_RANGE)
			//				Socket.input_elevator = PID_X_RANGE;
			//			else if(PID_x.output<-PID_X_RANGE)
			//				Socket.input_elevator = -PID_X_RANGE;
			//			else
			//				Socket.input_elevator=PID_x.output;
		}
#endif
	}
}
void task_sonar(){
	if(Sonar_module.flag_update==1)
	{
		Sonar_module.flag_update=0;
		/*
		 * your code begin from here;
		 */
		/*
		 * check first time running
		 */



		//		if(Flag.firstTimeRunning_taskSonar==0)
		//		{
		//			Flag.firstTimeRunning_taskSonar=1;
		//			preMicroSecond_sonar=getMicroSecond();
		//		}

		//		uint32_t microSecond=getMicroSecond();
		//		sampling_time_second=(float)(microSecond- preMicroSecond_sonar)/1000000.0;
		//		preMicroSecond_sonar=microSecond;
		sampling_time_second=0.05;

		/*
		 * term for estimator
		 */
		float acc=MPU6050.z1_dot_dot_filter;
		//		acc=constrain(acc,-5,5);
		float acc_term=0.5*acc*sampling_time_second*sampling_time_second*1000;
		//		acc_term=constrain(acc_term,-5,5);
		float velo_term=Sonar_module.z1_dot_filter*sampling_time_second;
		//		velo_term=velo_term=constrain(velo_term,-40,40);
		float pre_z1=Sonar_module.z1;

		data_buffer_cn[0]=acc_term;
		data_buffer_cn[1]=velo_term;
		data_buffer_cn[3]=acc;

		Sonar_module.distance_raw=(Sonar_module.pulse_width-Sonar_module.pulse_width_offset)*SONAR_SCALE_FACTOR;
		Sonar_module.z1_dot_measure=(Sonar_module.distance_raw-Sonar_module.pre_distance_raw)/sampling_time_second;
		Sonar_module.pre_distance_raw=Sonar_module.distance_raw;

		if((abs(Sonar_module.d_pulse_width)<SONAR_MAX_D_PULSE_WIDTH)&&(Sonar_module.pulse_width>SONAR_MIN_PULSE_WIDTH_)&&(Sonar_module.pulse_width<SONAR_MAX_PULSE_WIDTH_))
		{

			Sonar_module.z1=(pre_z1+velo_term+acc_term)*0.9+0.1*Sonar_module.distance_raw;
			Sonar_module.z1_error=abs(Sonar_module.z1-Sonar_module.distance_raw);

			if((Sonar_module.z1_error>100)&&(Sonar_module.divergence_error_count<1))  //4
			{
				//				flag_error_sonal_signal=1;
				Sonar_module.divergence_error_count++;
				//				Sonar_module.z1=(pre_z1+velo_term+acc_term)*0.95+0.05*Sonar_module.distance_raw;
				//				Sonar_module.z1_dot_filter=(Sonar_module.z1_dot_filter+acc*sampling_time_second*1000)*0.95+0.05*Sonar_module.z1_dot_measure;

				Sonar_module.z1=(pre_z1+velo_term+acc_term);
				Sonar_module.z1_dot_filter=(Sonar_module.z1_dot_filter+acc*sampling_time_second*1000);
			}
			else
			{
				Sonar_module.divergence_error_count=0;
				Sonar_module.z1_dot_filter=(Sonar_module.z1_dot_filter+acc*sampling_time_second*1000)*0.9+0.1*Sonar_module.z1_dot_measure;
				//				flag_error_sonal_signal=0;
			}
			data_buffer_cn[2]=Sonar_module.divergence_error_count;
		}
		else
		{
			Sonar_module.z1=(pre_z1+velo_term+acc_term);
			Sonar_module.z1_dot_filter=(Sonar_module.z1_dot_filter+acc*sampling_time_second*1000);
		}
		Sonar_module.z1_filter=Sonar_module.z1_filter*0.95+0.05*Sonar_module.z1;

		if(Flag.run_controller)
		{
			height_controller(sampling_time_second);
		}

		//		if((Sonar_module.pulse_width>SONAR_MIN_PULSE_WIDTH_)&&(Sonar_module.pulse_width<SONAR_MAX_PULSE_WIDTH_))
		//		{
		//			uint32_t microSecond=0;
		//			microSecond=getMicroSecond();
		//			sampling_time_second=(float)(microSecond- preMicroSecond_sonar)/1000000.0;
		//			preMicroSecond_sonar=microSecond;
		//
		//			Sonar_module.distance_raw=(Sonar_module.pulse_width-Sonar_module.pulse_width_offset)*SONAR_SCALE_FACTOR;
		//			Sonar_module.z1_dot_measure=(Sonar_module.distance_raw-Sonar_module.pre_distance_raw)/sampling_time_second;
		//			Sonar_module.pre_distance_raw=Sonar_module.distance_raw;
		//			/*
		//			 * term for estimator
		//			 */
		//			float acc=MPU6050.z1_dot_dot_filter;
		//			float acc_term=0.5*acc*sampling_time_second*sampling_time_second*1000;
		//			float velo_term=Sonar_module.z1_dot_filter*sampling_time_second;
		//			float pre_z1=Sonar_module.z1;
		//			//		Sonar_module.z1_dot_estimate=Sonar_module.z1_dot_measure;
		//			data_buffer_cn[0]=acc_term;
		//			//			data_buffer_cn[1]=Sonar_module.z1_dot_estimate;
		//			//			data_buffer_cn[2]=sampling_time_second;
		//			//		Sonar_module.z1_dot_predict=data_buffer_cn[0]+data_buffer_cn[1];
		//			//			Sonar_module.z1_dot_predict=Sonar_module.z1_dot_predict+MPU6050.z1_dot_dot_filter*sampling_time_second*1000;
		//
		//			//				Sonar_module.z1=Sonar_module.pre_distance_raw;
		//
		//
		//			Sonar_module.z1=(pre_z1+velo_term+acc_term)*0.9+0.1*Sonar_module.distance_raw;
		//			Sonar_module.z1_error=abs(Sonar_module.z1-Sonar_module.distance_raw);
		//
		//			if((Sonar_module.z1_error)>100)
		//				Sonar_module.z1=(pre_z1+velo_term+acc_term);
		//
		//			Sonar_module.z1_dot_filter=(Sonar_module.z1_dot_filter+acc*sampling_time_second*1000)*0.95+0.05*Sonar_module.z1_dot_measure;
		//			//			if(abs(Sonar_module.z1_dot_measure)<2000)Sonar_module.z1_dot_filter=(Sonar_module.z1_dot_filter+acc*sampling_time_second*1000)*0.95+0.05*Sonar_module.z1_dot_measure;
		//			//				Sonar_module.z1_dot_filter=(Sonar_module.z1_dot_filter+acc*sampling_time_second*1000)*0.95+0.05*Sonar_module.z1_dot_measure;
		//			//			else
		//			//				Sonar_module.z1_dot_filter=(Sonar_module.z1_dot_filter+acc*sampling_time_second*1000);
		//
		//
		//			if(Flag.run_controller)
		//			{
		//				height_controller(sampling_time_second);
		//			}
		//			//			preMicroSecond_sonar=buffer_timer;
		//
		//		}
	}
}

void display_com(){
	char buffer[20];

	if(display_mode==COM2CTL_DISPLAY_MODE_0_)
	{

	}
	else if(display_mode==COM2CTL_DISPLAY_MODE_1_)
	{

		/*
		 * khao sat PID_z
		 */

		float2num(Sonar_module.attitude,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_1_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		float2num(PID_z.set_point,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_2_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		float2num(Socket.attitude_hold_throttle,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_3_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//Dte
		float2num(PID_z.P_term,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_4_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//
		float2num(PID_z.I_term,buffer); //acc_term
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_5_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		float2num(PID_z.D_term,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_6_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//
		float2num(PID_z.KP*100.0,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_7_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		float2num(PID_z.KI*100.0,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_8_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//
		float2num(PID_z.KD*100.0,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_9_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		//		float2num(-12343.23,buffer);
		//		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_10_);
		//		SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);
	}
	else if(display_mode==COM2CTL_DISPLAY_MODE_2_)
	{
		/*
		 * khao sat do cao
		 */
		float2num(Sonar_module.z1,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_1_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		float2num(Sonar_module.distance_raw,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_2_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		float2num(Sonar_module.z1_dot_filter,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_3_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//
		float2num(data_buffer_cn[0],buffer);//acc term
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_4_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//
		float2num(data_buffer_cn[1],buffer);//vello term
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_5_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//		//
		int2num(data_buffer_cn[2],buffer);//flag error
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_6_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//	//
		float2num(Sonar_module.z1_dot_measure,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_7_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		int2num(Sonar_module.pulse_width,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_8_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//
		float2num(data_buffer_cn[3],buffer);//acc
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_9_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//
		//		float2num(PID_roll.set_point,buffer);
		//		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_10_);
		//		SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);


	}
	else if(display_mode==COM2CTL_DISPLAY_MODE_3_)
	{
		/*
		 * khao sat PIDx
		 */
		float2num(PID_x.set_point,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_1_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		float2num(Camera.X,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_2_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		float2num(PID_x.P_term,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_3_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//Dte
		float2num(PID_y.I_term,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_4_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//
		float2num(PID_x.D_term,buffer); //acc_term
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_5_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		float2num(Socket.input_elevator,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_6_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//
		float2num(PID_pitch.set_point,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_7_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		float2num(IMU.pitch,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_8_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//
		float2num(PID_x.fb_filter,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_9_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//
		//	float2num(-12343.23,buffer);
		//	SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_10_);
		//	SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);
	}
	else if(display_mode==COM2CTL_DISPLAY_MODE_4_)
	{
		/*
		 *  khao sat input RF
		 */
		int2num(RF_module.Channel_1,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_1_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		int2num(RF_module.Channel_2,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_2_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		int2num(RF_module.Channel_3,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_3_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//Dte
		int2num(RF_module.Channel_4,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_4_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//
		int2num(RF_module.Channel_5,buffer); //acc_term
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_5_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		int2num(RF_module.Channel_6,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_6_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//
		int2num(RF_module.Channel_7,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_7_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		int2num(RF_module.Channel_8,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_8_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		//		float2num(Socket.throttle_offset,buffer);
		//		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_9_);
		//		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
	}
	else if(display_mode==COM2CTL_DISPLAY_MODE_5_)
	{
		/*
		 * khao sat IMU
		 */
		float2num(IMU.roll,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_1_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		float2num(IMU.pitch,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_2_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		float2num(IMU.yaw_gyro,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_3_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//Dte
		float2num(PID_roll.set_point,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_4_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//
		float2num(PID_pitch.set_point,buffer); //acc_term
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_5_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		float2num(PID_yaw.set_point,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_6_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//
		float2num(PID_x.KP*1000,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_7_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		float2num(PID_x.KI*1000,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_8_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//
		float2num(PID_x.KD*1000,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_9_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//
		//	float2num(-12343.23,buffer);
		//	SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_10_);
		//	SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);
	}

	else if(display_mode==COM2CTL_DISPLAY_MODE_6_)
	{
		/*
		 * khao sat PIDx -y
		 */
		float2num(Camera.X,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_1_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		float2num(Camera.Y,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_2_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		//		float2num(PID_x.fb_filter,buffer);
		//		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_3_);
		//		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//		//Dte
		//		float2num(PID_y.fb_filter,buffer);
		//		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_4_);
		//		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//
		int2num(Position.axis.X_pixel,buffer); //acc_term
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_5_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		int2num(Position.axis.Y_pixel,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_6_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//
		float2num(Sonar_module.z1,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_7_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		//		int2num((char)flag_landing_20Hz,buffer);
		//		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_8_);
		//		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//		//
		//		int2num((char)flag_takeoff_20Hz,buffer);
		//		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_9_);
		//		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//
		//	float2num(-12343.23,buffer);
		//	SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_10_);
		//	SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);
	}
	else if(display_mode==COM2CTL_DISPLAY_MODE_7_)
	{
		/*
		 * khao sat socket
		 */
		float2num(IMU.roll,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_1_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		float2num(IMU.pitch,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_2_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		float2num(Socket.input_elevator_midpoint,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_3_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//
		float2num(Socket.input_aileron_midpoint,buffer);//acc term
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_4_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//
		float2num(Socket.input_elevator,buffer);//vello term
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_5_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//		//
		float2num(Socket.input_aileron,buffer);//flag error
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_6_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//	//
		float2num(Socket.throttle,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_7_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		float2num(PID_roll.set_point,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_8_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//
		float2num(PID_pitch.set_point,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_9_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//
		//		float2num(PID_roll.set_point,buffer);
		//		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_10_);
		//		SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);


	}

	else
	{
		/*
		 * khao sat PIDy
		 */
		float2num(Socket.input_aileron,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_1_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		float2num(Camera.Y,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_2_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		float2num(PID_y.P_term,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_3_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//Dte
		float2num(PID_y.I_term,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_4_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//
		float2num(PID_y.D_term,buffer); //acc_term
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_5_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		float2num(Socket.input_aileron_midpoint,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_6_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//
		float2num(PID_roll.set_point,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_7_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

		float2num(IMU.roll,buffer);
		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_8_);
		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//
		//		float2num(PID_y.KD*1000,buffer);
		//		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_9_);
		//		SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
		//
		//		float2num(-12343.23,buffer);
		//		SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_10_);
		//		SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);
	}


	SerialTerminator(UART_COM_2_CONTROLLER_);
}



void height_controller(float sampling_time){
	//#ifdef SONAR_HOLD_ATTITUDE
	//	if(Flag.SonarHoldAttitudeMode){
	//		if((Sonar_module.flag_update==1)&&(Flag.run_controller==1))
	//		{

	//			Sonar_module.flag_update=0;
	Sonar_module.attitude=Sonar_module.z1*IMU.cos_roll*IMU.cos_pitch;
	/*
	 * Your code start from here
	 */
	//			uint32_t microSecond=0;
	//			microSecond=getMicroSecond();
	//			sampling_time_second=(float)(microSecond- preMicroSecond_sonar)/1000000.0;
	//			preMicroSecond_sonar=microSecond;

	/*
	 * Y_Controller
	 */
	PID_z.sampling_time=sampling_time;

	if(PID_z.set_point<30)
	{
		if(Flag.PID_z_controller==1)
		{
			Flag.PID_z_controller=0;
			PID_z.I_term=0;
		}
		Flag.PD_z_controller=1;
		PD_type_4(Sonar_module.attitude,&PID_z,0);
	}

	else{
		if(Flag.PD_z_controller==1)
		{
			Flag.PD_z_controller=0;
			PID_z.I_term=0;
		}
		Flag.PID_z_controller=1;
		PID_type_4(Sonar_module.attitude,&PID_z,0);
	}
}

void task_IMU()
{
	if(flag_MPU6050_INTpin==1)
	{
		flag_MPU6050_INTpin=0;
		/*
		 * Your code begin from here
		 */
		uint32_t microSecond=0;
		microSecond=getMicroSecond();
		sampling_time_second=(float)(microSecond- preMicroSecond_angle)/1000000.0;
		preMicroSecond_angle=microSecond;
		/*
		 * update sensor's datas
		 */


		while(MPU6050DataGetRaw(&MPU6050.accX_raw))
		{}
		//		{led(LED_BLUE,1);}// loop untill data is read
		//		led(LED_BLUE,0);

		/*
		 * sensor processing
		 */
		angle(sampling_time_second);
		update_accelerometer();

		/*
		 *  running controller
		 */
		if(Flag.run_controller)
			Stable_Controller(sampling_time_second);
	}// end of (flag_MPU6050_INTpin==1)
}

void Stable_Controller(float sampling_time){
	PID_roll.sampling_time=sampling_time;
	PID_pitch.sampling_time=sampling_time;
	PID_yaw.sampling_time=sampling_time;

	PID_pitch.set_point = Socket.input_elevator+Socket.input_elevator_midpoint+Socket.input_elevator_offset;
	PID_roll.set_point = Socket.input_aileron+Socket.input_aileron_midpoint+Socket.input_aileron_offset;

	//#ifdef SONAR_HOLD_ATTITUDE
	//	//if(Flag.SonarHoldAttitudeMode)
	//	if(PID_y.set_point<10)
	//		//	else
	//#else
	//		if(Socket.input_throttle<20)
	//#endif

	//	if(RF_module.Channel_3<1400)
	if(Flag.Autonomous)
	{
		PID_type_3(IMU.roll,&PID_roll);
		PID_type_3(IMU.pitch,&PID_pitch);
		PID_type_3(IMU.yaw_gyro,&PID_yaw);
	}
	else
	{
		if(RF_module.Channel_3<1100)
		{
			if(Flag.PID_o_controller==1)
			{
				Flag.PID_o_controller=0;
				PID_pitch.I_term=0;
				PID_roll.I_term=0;
				PID_yaw.I_term=0;

				PID_yaw.set_point=0;
				IMU.yaw_gyro=0;
			}
			Flag.PD_o_controller=1;

			PD_type_3(IMU.roll,&PID_roll);
			PD_type_3(IMU.pitch,&PID_pitch);
			PD_type_3(IMU.yaw_gyro,&PID_yaw);

		}
		else{
			if(Flag.PD_o_controller==1)
			{
				Flag.PD_o_controller=0;
				PID_pitch.I_term=0;
				PID_roll.I_term=0;
				PID_yaw.I_term=0;
			}
			Flag.PID_o_controller=1;

			PID_type_3(IMU.roll,&PID_roll);
			PID_type_3(IMU.pitch,&PID_pitch);
			PID_type_3(IMU.yaw_gyro,&PID_yaw);
		}
	}
	//	cos_roll=cos(IMU.roll*deg_to_rad);
	//	cos_pitch=cos(IMU.pitch*deg_to_rad);
	/*
	 * height control
	 */
	//#ifdef SONAR_HOLD_ATTITUDE
	if(Flag.SonarHoldAttitudeMode){
		Socket.throttle=Socket.throttle_offset+PID_z.output;
		if ((IMU.cos_roll>0)&&(IMU.cos_pitch>0)&&(Socket.throttle>=0))
		{
			Socket.attitude_hold_throttle=sqrt(Socket.throttle/IMU.cos_roll/IMU.cos_pitch);
			Socket.attitude_hold_throttle=Map_y(Socket.attitude_hold_throttle,0,10,0,1000);
		}

		if (Socket.attitude_hold_throttle<MIN_THROTTLE)
			Socket.attitude_hold_throttle=MIN_THROTTLE;
		else if(Socket.attitude_hold_throttle>MAX_THROTTLE)
			Socket.attitude_hold_throttle=MAX_THROTTLE;
	}
	else{
		//#else
		if ((IMU.cos_roll>0)&&(IMU.cos_pitch>0)&&(Socket.input_throttle>=0))
		{
			Socket.attitude_hold_throttle=sqrt(Socket.input_throttle/IMU.cos_roll/IMU.cos_pitch);
			Socket.attitude_hold_throttle=Map_y(Socket.attitude_hold_throttle,0,10,0,1000);
		}
	}
	//#endif

	Socket.output_1=Socket.attitude_hold_throttle;
	Socket.output_2=Socket.attitude_hold_throttle;
	Socket.output_3=Socket.attitude_hold_throttle;
	Socket.output_4=Socket.attitude_hold_throttle;

	/*
	 * pitch control
	 */


	Socket.output_1 -= PID_pitch.output;
	Socket.output_4 -= PID_pitch.output;
	Socket.output_2 += PID_pitch.output;
	Socket.output_3 += PID_pitch.output;

	/*
	 * roll control
	 */
	Socket.output_1 += PID_roll.output;
	Socket.output_2 += PID_roll.output;
	Socket.output_3 -= PID_roll.output;
	Socket.output_4 -= PID_roll.output;

	/*
	 * yaw control
	 */
	Socket.output_1 -= PID_yaw.output;
	Socket.output_3 -= PID_yaw.output;
	Socket.output_2 += PID_yaw.output;
	Socket.output_4 += PID_yaw.output;

	/*
	 * Map system's output (controller + remote) signal to ESC's input  signal
	 */
	update_omega(Socket.output_1,Socket.output_2,Socket.output_3,Socket.output_4);

	ESC.ppm_1=(int32_t)Map_y(Socket.output_1,0,1000,ESC_1_MIN,ESC_1_MAX);
	ESC.ppm_2=(int32_t)Map_y(Socket.output_2,0,1000,ESC_2_MIN,ESC_2_MAX);
	ESC.ppm_3=(int32_t)Map_y(Socket.output_3,0,1000,ESC_3_MIN,ESC_3_MAX);
	ESC.ppm_4=(int32_t)Map_y(Socket.output_4,0,1000,ESC_4_MIN,ESC_4_MAX);


	if(Flag.test_motor==1)
	{
		ESC_ppm(1, ESC.ppm_1);
		ESC_ppm(2, ESC.ppm_2);
		ESC_ppm(3, ESC.ppm_3);
		ESC_ppm(4, ESC.ppm_4);
	}
	else
	{
		ESC_ppm(1,ESC_1_MIN);
		ESC_ppm(2,ESC_2_MIN);
		ESC_ppm(3,ESC_3_MIN);
		ESC_ppm(4,ESC_4_MIN);
	}
}
void task_RF()
{
	if(RF_module.flag_update==1)
	{
		RF_module.flag_update=0;
		/*
		 * Your code begin from here
		 */


		/*
		 * Map RF signals to user control(remote) signal
		 */
		//#ifdef SONAR_HOLD_ATTITUDE
		if(Flag.SonarHoldAttitudeMode){
			if((Flag.Autonomous==0)&&(flag_landing_20Hz==0)){
				Socket.input_throttle=Map_y((float)RF_module.Channel_3,C3_MIN,C3_MAX,0,1000);
				if(Socket.input_throttle<0)
					Socket.input_throttle=0;
				else if(Socket.input_throttle>1000)
					Socket.input_throttle=1000;
				PID_z.set_point=Socket.input_throttle;
			}
		}
		else{
			//#else
			Socket.input_throttle=Map_y((float)RF_module.Channel_3,C3_MIN,C3_MAX,0,81);
			if(Socket.input_throttle<0)
				Socket.input_throttle=0;
			else if(Socket.input_throttle>81)
				Socket.input_throttle=81;
		}
		//#endif

#ifndef POSITION_CONTROLLER_
		Socket.input_elevator=Map_y((float)RF_module.Channel_2,C2_ZERO,C2_MAX,0,ELEVATOR_RANGE);
		Socket.input_aileron=Map_y((float)RF_module.Channel_1,C1_ZERO,C1_MAX,0,AILERON_RANGE);
#endif

		Socket.input_rudder=Map_y((float)RF_module.Channel_4,C4_ZERO,C4_MAX,0,RUDDER_MAX_VELO);
		if((Socket.input_rudder>5)|(Socket.input_rudder<-5))
			PID_yaw.set_point-=Socket.input_rudder*0.02;//50hZ

		if(Flag.OnlinePIDTuneMode)
		{
			Socket.input_elevator_midpoint=PITCH_SET_ANGLE;
			Socket.input_aileron_midpoint=ROLL_SET_ANGLE;
			Socket.throttle_offset=THROTTLE_OFFSET;//24

			PID_y.KP=Map_y((float)RF_module.Channel_8,C8_ZERO,C8_MAX,PID_POSITION_KP_,PID_POSITION_KP_+PID_POSITION_KP_RANGE_);
			PID_y.KI=Map_y((float)RF_module.Channel_6,C6_ZERO,C6_MAX,PID_POSITION_KI_,PID_POSITION_KI_+PID_POSITION_KI_RANGE_);
			PID_y.KD=Map_y((float)RF_module.Channel_5,C5_ZERO,C5_MAX,PID_POSITION_KD_,PID_POSITION_KD_+PID_POSITION_KD_RANGE_);

			PID_x.KP=PID_y.KP;
			PID_x.KI=PID_y.KI;
			PID_x.KD=PID_y.KD;

			//			PID_z.KP=Map_y((float)RF_module.Channel_8,C8_ZERO,C8_MAX,PID_Z_KP_,PID_Z_KP_+PID_Z_KP_RANGE_);
			//			PID_z.KI=Map_y((float)RF_module.Channel_6,C6_ZERO,C6_MAX,PID_Z_KI_,PID_Z_KI_+PID_Z_KI_RANGE_);
			//			PID_z.KD=Map_y((float)RF_module.Channel_5,C5_ZERO,C5_MAX,PID_Z_KD_,PID_Z_KD_+PID_Z_KD_RANGE_);
		}
		else
		{
			//pitch==elevator=C6; roll==aileron==C5
			//			Socket.input_aileron_midpoint=ROLL_SET_ANGLE;
			//#ifdef SONAR_HOLD_ATTITUDE

			if(Flag.SonarHoldAttitudeMode)
			{
				if(Flag.Autonomous==0)
					Socket.throttle_offset=Map_y((float)RF_module.Channel_8,C8_MIN,C8_MAX,0,49);

				Socket.input_elevator_midpoint=PITCH_SET_ANGLE;
				Socket.input_aileron_midpoint=ROLL_SET_ANGLE;
			}
			else
			{
				Socket.input_elevator_midpoint=Map_y((float)RF_module.Channel_6,C6_ZERO,C6_MAX,PITCH_SET_ANGLE,PITCH_SET_ANGLE+SET_ANGLE_RANGE);
				Socket.input_aileron_midpoint=Map_y((float)RF_module.Channel_5,C5_ZERO,C5_MAX,ROLL_SET_ANGLE,ROLL_SET_ANGLE+SET_ANGLE_RANGE);

			}

			//#endif

			//		PID_pitch.set_point = Socket.input_elevator+Socket.input_elevator_midpoint;
			//		PID_roll.set_point = Socket.input_aileron+Socket.input_aileron_midpoint;
		}
	}

}


void mode_selection()
{
	if(Flag_Safe.RF_module==0)
	{
		safe_check();
		//		Flag.test_motor=0;
		//		Flag_Safe.System=0;

		if(Flag_SonarHoldAttitudeMode==1)
		{
			Flag.SonarHoldAttitudeMode=1;
			SerialPutStrLn(UART_COM_2_CONTROLLER_,"attitude_hold mode!");
			//			if(Flag.Autonomous)
			//				SerialPutStrLn(UART_COM_2_CONTROLLER_,"autonomous_on!");
			//			else
			//				SerialPutStrLn(UART_COM_2_CONTROLLER_,"autonomous_off!");
			Flag.Autonomous=1;
			autonomous_reset();
			system_reset();
			//			Flag.test_motor=1;
			//			Flag_Safe.System=1;

			SerialPutStrLn(UART_COM_2_CONTROLLER_,"autonomous mode on");
			led(LED_RED,1);
		}
		else
		{
			Flag.SonarHoldAttitudeMode=0;
			SerialPutStrLn(UART_COM_2_CONTROLLER_,"manual_mode!");
			led(LED_RED,0);
		}

		Flag.run_controller=1;
		system_reset();

		if(Flag.Autonomous)
			autonomous_reset();
	}
}

void task_20Hz(){
	if(FlagTimer.Hz_20)
	{
		FlagTimer.Hz_20=0;
		/*
		 * Your code begin from here
		 */
		if(flag_landing_20Hz)
		{
			flag_takeoff_20Hz=0;
			if(PID_z.set_point>0)
				PID_z.set_point-=DELTA_LANDING_Z1_;
			else
			{
				PID_z.set_point=0;

			}
			if(Sonar_module.z1<100)
			{
				Flag_Safe.System=0;
				flag_landing_20Hz=0;
			}
			//			{
			//				ESC_ppm(1,ESC_1_MIN);
			//				ESC_ppm(2,ESC_2_MIN);
			//				ESC_ppm(3,ESC_3_MIN);
			//				ESC_ppm(4,ESC_4_MIN);
			//				flag_landing_20Hz=0;
			//
			//			}
		}
		else if(Flag.Autonomous){
			if(flag_takeoff_20Hz)
			{
				if(PID_z.set_point<AUTONOMOUS_ATTITUDE_)
					PID_z.set_point+=DELTA_TAKE_OFF_Z1_;
				else
				{
					PID_z.set_point=AUTONOMOUS_ATTITUDE_;
					flag_takeoff_20Hz=0;
				}
			}
		}
		//		Socket.throttle_offset=THROTTLE_OFFSET;// bien dat luc day ban dau cho take off
		//		PID_z.set_point; // bien giam do cao tu tu

	}
}




void task_100Hz(){
	if(FlagTimer.Hz_100)
	{
		FlagTimer.Hz_100=0;
		/*
		 * Your code begin from here
		 */
		//		if(Flag.display)
		//			display_com();

	}
}

void task_50Hz(){
	if(FlagTimer.Hz_50)
	{
		FlagTimer.Hz_50=0;
		/*
		 * Your code begin from here
		 */
		if(Flag.display)
			display_com();

	}
}




//volatile uint16_t PositionSensor_count=0;
void communication()
{
	if(UartCamera.Flag_receive)
	{
		UartCamera.Flag_receive=0;
		toggle_led[0]^=1;
		//		SerialPutStrLn(UART_COM_2_CONTROLLER_,UartCamera.Command_Data);
		switch(UartCamera.Command_Data[0])
		{
		case SENSOR2CTL_BLUE_:
			set_position(&(UartCamera.Command_Data[1]),Position.data);
			Flag.cameraUpdate=1;
			Flag.cameraUpdate_error=0;
			led(LED_BLUE,toggle_led[0]);
			led(LED_RED,0);
			led(LED_GREEN,0);

			if(color_capture==CAPTURE_RED_)
				Serial_Sendcommand_Camera("r");
			else if(color_capture==CAPTURE_GREEN_)
				Serial_Sendcommand_Camera("g");

			//			SerialPutStrLn(UART_COM_2_CONTROLLER_,"b");
			flag_color_capture=CAPTURE_BLUE_;
			break;
		case SENSOR2CTL_RED_:
			set_position(&(UartCamera.Command_Data[1]),Position.data);
			Flag.cameraUpdate=1;
			Flag.cameraUpdate_error=0;
			//			SerialPutStrLn(UART_COM_2_CONTROLLER_,"r");
			//			Flag.cameraUpdate_red=1;
			led(LED_RED,toggle_led[0]);
			led(LED_BLUE,0);
			led(LED_GREEN,0);

			if(color_capture==CAPTURE_GREEN_)
				Serial_Sendcommand_Camera("g");
			else if(color_capture==CAPTURE_BLUE_)
				Serial_Sendcommand_Camera("b");

			flag_color_capture=CAPTURE_RED_;
			break;
		case SENSOR2CTL_GREEN_:
			set_position(&(UartCamera.Command_Data[1]),Position.data);
			Flag.cameraUpdate=1;
			Flag.cameraUpdate_error=0;
			//			SerialPutStrLn(UART_COM_2_CONTROLLER_,"g");
			//			Flag.cameraUpdate_green=1;
			flag_color_capture=CAPTURE_GREEN_;
			led(LED_GREEN,toggle_led[0]);
			led(LED_RED,0);
			led(LED_BLUE,0);

			if(color_capture==CAPTURE_RED_)
				Serial_Sendcommand_Camera("r");
			else if(color_capture==CAPTURE_BLUE_)
				Serial_Sendcommand_Camera("b");

			//			Flag.cameraUpdate_red=0;
			break;
		case SENSOR2CTL_ERROR_:
			Flag.cameraUpdate_error=1;
			Flag.cameraUpdate=1;
			//			SerialPutStrLn(UART_COM_2_CONTROLLER_,"error color");
			led(LED_BLUE,toggle_led[0]);
			led(LED_RED,toggle_led[0]);
			led(LED_GREEN,toggle_led[0]);

			if(color_capture==CAPTURE_RED_)
				Serial_Sendcommand_Camera("r");
			else if(color_capture==CAPTURE_GREEN_)
				Serial_Sendcommand_Camera("g");
			else if(color_capture==CAPTURE_BLUE_)
				Serial_Sendcommand_Camera("b");
			flag_color_capture=CAPTURE_ERROR_;
			break;
		default:
			break;

		}
	}

	if(Uart.Flag_receive)
	{
		Uart.Flag_receive=0;
		switch(Uart.Command_Data[0])
		{

		case COM2CTL_DISPLAY_ON_:
			Flag.display=1;
			display_mode=Uart.Command_Data[1];
			break;
		case COM2CTL_DISPLAY_OFF_:
			Flag.display=0;
			break;

		case COM2CTL_AUTONOMOUS_ON_:
			Flag.Autonomous=1;
			autonomous_reset();
			system_reset();
			SerialPutStrLn(UART_COM_2_CONTROLLER_,"autonomous mode on");
			break;
		case COM2CTL_AUTONOMOUS_OFF_:
			Flag.Autonomous=0;

			SerialPutStrLn(UART_COM_2_CONTROLLER_,"autonomous mode off");
			break;

		case COM2CTL_SET_COLOR_RED_:
			Serial_Sendcommand_Camera("r");
			color_capture=CAPTURE_RED_;
			SerialPutStrLn(UART_COM_2_CONTROLLER_,"set red");

			break;
		case COM2CTL_SET_COLOR_GREEN_:
			Serial_Sendcommand_Camera("g");
			color_capture=CAPTURE_GREEN_;
			SerialPutStrLn(UART_COM_2_CONTROLLER_,"set green");

			break;
		case COM2CTL_SET_COLOR_BLUE_:
			Serial_Sendcommand_Camera("b");
			color_capture=CAPTURE_BLUE_;
			SerialPutStrLn(UART_COM_2_CONTROLLER_,"set blue");
			break;

		case COM2CTL_ONLINE_PID_TUNE_ON_:
			Flag.OnlinePIDTuneMode=1;
			SerialPutStrLn(UART_COM_2_CONTROLLER_,"pid tunning on");
			break;
		case COM2CTL_ONLINE_PID_TUNE_OFF_:
			Flag.OnlinePIDTuneMode=0;
			SerialPutStrLn(UART_COM_2_CONTROLLER_,"pid tunning off");
			break;
		case  COM2CTL_MOTOR_ON_:
			Flag.test_motor=1;
			Flag_Safe.System=1;
			system_reset();
			if(Flag.Autonomous)
				autonomous_reset();
			SerialPutStrLn(UART_COM_2_CONTROLLER_,"motor on");
			//			switch (Uart.Command_Data[1])
			//			{
			//			case COM2CTL_MODE_MANUAL_FLY_:
			//				SerialPutStrLn(UART_COM_2_CONTROLLER_,"mode: manual fly");
			//				operation_mode=COM2CTL_MODE_MANUAL_FLY_;
			//				break;
			//			case COM2CTL_MODE_ATTITUDE_HOLD_ON_:
			//				SerialPutStrLn(UART_COM_2_CONTROLLER_,"mode: manual fly");
			//				operation_mode=COM2CTL_MODE_MANUAL_FLY_;
			//				break;
			//			case
			//			case
			//			}
			break;
		case  COM2CTL_MOTOR_OFF_:
			Flag.test_motor=0;
			SerialPutStrLn(UART_COM_2_CONTROLLER_,"motor off");
			break;
		case  COM2CTL_SET_KP_:
		{
			switch(Uart.Command_Data[1])
			{
			char buffer[10];
			case  COM2CTL_PID_ROLL_:
				PID_roll.KP=atof(&Uart.Command_Data[2]);
				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"set KP roll: ");
				float2num(PID_roll.KP,buffer);
				SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);
				break;
			case COM2CTL_PID_PITCH_:
				PID_pitch.KP=atof(&Uart.Command_Data[2]);
				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"set KP pitch: ");
				float2num(PID_pitch.KP,buffer);
				SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);
				break;
			case COM2CTL_PID_YAW_:
				PID_yaw.KP=atof(&Uart.Command_Data[2]);
				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"set KP yaw: ");
				float2num(PID_yaw.KP,buffer);
				SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);
				break;
			case COM2CTL_PID_X_:
				PID_x.KP=atof(&Uart.Command_Data[2]);
				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"set KP X: ");
				float2num(PID_x.KP,buffer);
				SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);
				break;
			case COM2CTL_PID_Y_:
				PID_y.KP=atof(&Uart.Command_Data[2]);
				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"set KP Y: ");
				float2num(PID_y.KP,buffer);
				SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);
				break;
			case COM2CTL_PID_Z_:
				PID_z.KP=atof(&Uart.Command_Data[2]);
				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"set KP Z: ");
				float2num(PID_z.KP,buffer);
				SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);
				break;
			default:
				break;
			}

		}
		break;
		case  COM2CTL_SET_KI_:
		{
			switch(Uart.Command_Data[1])
			{
			char buffer[10];
			case  COM2CTL_PID_ROLL_:
				PID_roll.KI=atof(&Uart.Command_Data[2]);
				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"set KI roll: ");
				float2num(PID_roll.KI,buffer);
				SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);
				break;
			case COM2CTL_PID_PITCH_:
				PID_pitch.KI=atof(&Uart.Command_Data[2]);
				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"set KI pitch: ");
				float2num(PID_pitch.KI,buffer);
				SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);
				break;
			case COM2CTL_PID_YAW_:
				PID_yaw.KI=atof(&Uart.Command_Data[2]);
				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"set KI yaw: ");
				float2num(PID_yaw.KI,buffer);
				SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);
				break;
			case COM2CTL_PID_X_:
				PID_x.KI=atof(&Uart.Command_Data[2]);
				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"set KI X: ");
				float2num(PID_x.KI,buffer);
				SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);
				break;
			case COM2CTL_PID_Y_:
				PID_y.KI=atof(&Uart.Command_Data[2]);
				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"set KI Y: ");
				float2num(PID_y.KI,buffer);
				SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);
				break;
			case COM2CTL_PID_Z_:
				PID_z.KI=atof(&Uart.Command_Data[2]);
				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"set KI Z: ");
				float2num(PID_z.KI,buffer);
				SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);
				break;
			default:
				break;
			}
		}
		break;
		case  COM2CTL_SET_KD_:
		{
			switch(Uart.Command_Data[1])
			{
			char buffer[10];
			case  COM2CTL_PID_ROLL_:
				PID_roll.KD=atof(&Uart.Command_Data[2]);
				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"set KD roll: ");
				float2num(PID_roll.KD,buffer);
				SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);
				break;
			case COM2CTL_PID_PITCH_:
				PID_pitch.KD=atof(&Uart.Command_Data[2]);
				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"set KD pitch: ");
				float2num(PID_pitch.KD,buffer);
				SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);
				break;
			case COM2CTL_PID_YAW_:
				PID_yaw.KD=atof(&Uart.Command_Data[2]);
				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"set KD yaw: ");
				float2num(PID_yaw.KD,buffer);
				SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);
				break;
			case COM2CTL_PID_X_:
				PID_x.KD=atof(&Uart.Command_Data[2]);
				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"set KD X: ");
				float2num(PID_x.KD,buffer);
				SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);
				break;
			case COM2CTL_PID_Y_:
				PID_y.KD=atof(&Uart.Command_Data[2]);
				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"set KD Y: ");
				float2num(PID_y.KD,buffer);
				SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);
				break;
			case COM2CTL_PID_Z_:
				PID_z.KD=atof(&Uart.Command_Data[2]);
				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"set KD Z: ");
				float2num(PID_z.KD,buffer);
				SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);
				break;
			default:
				break;
			}

		}
		break;


		case COM2CTL_MOTOR_:
		{
			switch(Uart.Command_Data[1])
			{
			char buffer[10];
			case TEST_MOTOR_1_:
				ESC.ppm_1=atoi(&Uart.Command_Data[2]);

				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"motor_1: ");
				int2num(ESC.ppm_1,buffer);
				SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);

				ESC_ppm(1,ESC.ppm_1);
				break;
			case TEST_MOTOR_2_:
				ESC.ppm_2=atoi(&Uart.Command_Data[2]);

				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"motor_2: ");
				int2num(ESC.ppm_2,buffer);
				SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);

				ESC_ppm(2,ESC.ppm_2);
				break;
			case TEST_MOTOR_3_:
				ESC.ppm_3=atoi(&Uart.Command_Data[2]);

				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"motor_3: ");
				int2num(ESC.ppm_1,buffer);
				SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);

				ESC_ppm(3,ESC.ppm_3);
				break;
			case TEST_MOTOR_4_:
				ESC.ppm_4=atoi(&Uart.Command_Data[2]);

				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"motor_4: ");
				int2num(ESC.ppm_4,buffer);
				SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);

				ESC_ppm(4,ESC.ppm_4);
				break;
			case MOTOR_STOP_:
				Motor_stop();
				break;
			default:
				break;
			}

		}
		break;
		case COM2CTL_SYSTEM_INFOR_:
		{
			switch(Uart.Command_Data[1])
			{
			case '1':
				display_system_infor();
				break;
			default:
				break;
			}

		}
		break;
		default:
			break;
		}//end of switch case

		//		printf(Uart.Command_Data);
		SerialPutStrLn(UART_COM_2_CONTROLLER_,Uart.Command_Data);
	} //end of if(Uart.Flag_receive)
}

void display_system_infor(){
	char buffer[10];
	/*
	 * for roll
	 */
	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"PID_roll: ");
	float2num(PID_roll.KP,buffer);
	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_," ");

	float2num(PID_roll.KI,buffer);
	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_," ");

	float2num(PID_roll.KD,buffer);
	SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);

	/*
	 * for pitch
	 */

	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"PID_pitch: ");
	float2num(PID_pitch.KP,buffer);
	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_," ");

	float2num(PID_pitch.KI,buffer);
	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_," ");

	float2num(PID_pitch.KD,buffer);
	SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);

	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"PID_yaw: ");
	float2num(PID_yaw.KP,buffer);
	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_," ");

	float2num(PID_yaw.KI,buffer);
	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_," ");

	float2num(PID_yaw.KD,buffer);
	SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);

	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"PID_x*100: ");
	float2num(PID_x.KP*100,buffer);
	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_," ");

	float2num(PID_x.KI*100,buffer);
	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_," ");

	float2num(PID_x.KD*100,buffer);
	SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);

	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"PID_y*100: ");
	float2num(PID_y.KP*100,buffer);
	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_," ");

	float2num(PID_y.KI*100,buffer);
	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_," ");

	float2num(PID_y.KD*100,buffer);
	SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);

	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"PID_z: ");
	float2num(PID_z.KP,buffer);
	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_," ");

	float2num(PID_z.KI,buffer);
	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_," ");

	float2num(PID_z.KD,buffer);
	SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);
}
