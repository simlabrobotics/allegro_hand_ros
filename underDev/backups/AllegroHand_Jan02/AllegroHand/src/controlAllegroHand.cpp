/*
 * control_AllegroHand.cpp
 *
 *  Created on: Nov 15, 2012
 *      Author: Seungsu KIM
 */

#include "controlAllegroHand.h"
#include <iostream>
#include <math.h>
#include <stdio.h>
using namespace std;

void PRINT_INFO(const char *msg)
{
	cout << msg << endl;
}

controlAllegroHand::controlAllegroHand()
{
	mEmergencyStop = false;

	mPWM_MAX[eJOINTNAME_INDEX_0] = PWM_LIMIT_ROLL;
	mPWM_MAX[eJOINTNAME_INDEX_1] = PWM_LIMIT_NEAR;
	mPWM_MAX[eJOINTNAME_INDEX_2] = PWM_LIMIT_MIDDLE;
	mPWM_MAX[eJOINTNAME_INDEX_3] = PWM_LIMIT_FAR;

	mPWM_MAX[eJOINTNAME_MIDDLE_0] = PWM_LIMIT_ROLL;
	mPWM_MAX[eJOINTNAME_MIDDLE_1] = PWM_LIMIT_NEAR;
	mPWM_MAX[eJOINTNAME_MIDDLE_2] = PWM_LIMIT_MIDDLE;
	mPWM_MAX[eJOINTNAME_MIDDLE_3] = PWM_LIMIT_FAR;

	mPWM_MAX[eJOINTNAME_PINKY_0] = PWM_LIMIT_ROLL;
	mPWM_MAX[eJOINTNAME_PINKY_1] = PWM_LIMIT_NEAR;
	mPWM_MAX[eJOINTNAME_PINKY_2] = PWM_LIMIT_MIDDLE;
	mPWM_MAX[eJOINTNAME_PINKY_3] = PWM_LIMIT_FAR;

	mPWM_MAX[eJOINTNAME_THUMB_0] = PWM_LIMIT_THUMB_ROLL;
	mPWM_MAX[eJOINTNAME_THUMB_1] = PWM_LIMIT_THUMB_NEAR;
	mPWM_MAX[eJOINTNAME_THUMB_2] = PWM_LIMIT_THUMB_MIDDLE;
	mPWM_MAX[eJOINTNAME_THUMB_3] = PWM_LIMIT_THUMB_FAR;

//SAH01020003 Left
mEncoderOffset[eJOINTNAME_INDEX_0] = -199;
mEncoderOffset[eJOINTNAME_INDEX_1] = -66044;
mEncoderOffset[eJOINTNAME_INDEX_2] = -632;
mEncoderOffset[eJOINTNAME_INDEX_3] = 1517;
mEncoderOffset[eJOINTNAME_MIDDLE_0] = 497;
mEncoderOffset[eJOINTNAME_MIDDLE_1] = -66232;
mEncoderOffset[eJOINTNAME_MIDDLE_2] = -190;
mEncoderOffset[eJOINTNAME_MIDDLE_3] = 1031;
mEncoderOffset[eJOINTNAME_PINKY_0] = -777;
mEncoderOffset[eJOINTNAME_PINKY_1] = -66787;
mEncoderOffset[eJOINTNAME_PINKY_2] = 548;
mEncoderOffset[eJOINTNAME_PINKY_3] = 1358;
mEncoderOffset[eJOINTNAME_THUMB_0] = -64803;
mEncoderOffset[eJOINTNAME_THUMB_1] = -66346;
mEncoderOffset[eJOINTNAME_THUMB_2] = 682;
mEncoderOffset[eJOINTNAME_THUMB_3] = 574;

mEncoderDirection[eJOINTNAME_INDEX_0] = 1;
mEncoderDirection[eJOINTNAME_INDEX_1] = -1;
mEncoderDirection[eJOINTNAME_INDEX_2] = 1;
mEncoderDirection[eJOINTNAME_INDEX_3] = 1;
mEncoderDirection[eJOINTNAME_MIDDLE_0] = 1;
mEncoderDirection[eJOINTNAME_MIDDLE_1] = -1;
mEncoderDirection[eJOINTNAME_MIDDLE_2] = 1;
mEncoderDirection[eJOINTNAME_MIDDLE_3] = 1;
mEncoderDirection[eJOINTNAME_PINKY_0] = 1;
mEncoderDirection[eJOINTNAME_PINKY_1] = -1;
mEncoderDirection[eJOINTNAME_PINKY_2] = 1;
mEncoderDirection[eJOINTNAME_PINKY_3] = 1;
mEncoderDirection[eJOINTNAME_THUMB_0] = -1;
mEncoderDirection[eJOINTNAME_THUMB_1] = -1;
mEncoderDirection[eJOINTNAME_THUMB_2] = 1;
mEncoderDirection[eJOINTNAME_THUMB_3] = 1;

mMotorDirection[eJOINTNAME_INDEX_0] = 1;
mMotorDirection[eJOINTNAME_INDEX_1] = -1;
mMotorDirection[eJOINTNAME_INDEX_2] = 1;
mMotorDirection[eJOINTNAME_INDEX_3] = 1;
mMotorDirection[eJOINTNAME_MIDDLE_0] = 1;
mMotorDirection[eJOINTNAME_MIDDLE_1] = -1;
mMotorDirection[eJOINTNAME_MIDDLE_2] = 1;
mMotorDirection[eJOINTNAME_MIDDLE_3] = 1;
mMotorDirection[eJOINTNAME_PINKY_0] = 1;
mMotorDirection[eJOINTNAME_PINKY_1] = -1;
mMotorDirection[eJOINTNAME_PINKY_2] = 1;
mMotorDirection[eJOINTNAME_PINKY_3] = 1;
mMotorDirection[eJOINTNAME_THUMB_0] = -1;
mMotorDirection[eJOINTNAME_THUMB_1] = -1;
mMotorDirection[eJOINTNAME_THUMB_2] = 1;
mMotorDirection[eJOINTNAME_THUMB_3] = 1;
// End SAH01020003 Left



/* // SAH01010002 Right (Seungsu's Hand_ Orig)
	mEncoderOffset[eJOINTNAME_INDEX_0]  =  -1105;
	mEncoderOffset[eJOINTNAME_INDEX_1]  = -65301;
	mEncoderOffset[eJOINTNAME_INDEX_2]  =   -412;
	mEncoderOffset[eJOINTNAME_INDEX_3]  =    832;
	mEncoderOffset[eJOINTNAME_MIDDLE_0] =    327;
	mEncoderOffset[eJOINTNAME_MIDDLE_1] = -66828;
	mEncoderOffset[eJOINTNAME_MIDDLE_2] =     44;
	mEncoderOffset[eJOINTNAME_MIDDLE_3] =   -436;
	mEncoderOffset[eJOINTNAME_PINKY_0]  =    758;
	mEncoderOffset[eJOINTNAME_PINKY_1]  = -65920;
	mEncoderOffset[eJOINTNAME_PINKY_2]  =    -42;
	mEncoderOffset[eJOINTNAME_PINKY_3]  =   -180;
	mEncoderOffset[eJOINTNAME_THUMB_0]  =    412;
	mEncoderOffset[eJOINTNAME_THUMB_1]  =   -266;
	mEncoderOffset[eJOINTNAME_THUMB_2]  = -64487;
	mEncoderOffset[eJOINTNAME_THUMB_3]  = -65882;

	mEncoderDirection[eJOINTNAME_INDEX_0]  =  1;
	mEncoderDirection[eJOINTNAME_INDEX_1]  = -1;
	mEncoderDirection[eJOINTNAME_INDEX_2]  =  1;
	mEncoderDirection[eJOINTNAME_INDEX_3]  =  1;
	mEncoderDirection[eJOINTNAME_MIDDLE_0] =  1;
	mEncoderDirection[eJOINTNAME_MIDDLE_1] = -1;
	mEncoderDirection[eJOINTNAME_MIDDLE_2] =  1;
	mEncoderDirection[eJOINTNAME_MIDDLE_3] =  1;
	mEncoderDirection[eJOINTNAME_PINKY_0]  =  1;
	mEncoderDirection[eJOINTNAME_PINKY_1]  = -1;
	mEncoderDirection[eJOINTNAME_PINKY_2]  =  1;
	mEncoderDirection[eJOINTNAME_PINKY_3]  =  1;
	mEncoderDirection[eJOINTNAME_THUMB_0]  =  1;
	mEncoderDirection[eJOINTNAME_THUMB_1]  =  1;
	mEncoderDirection[eJOINTNAME_THUMB_2]  = -1;
	mEncoderDirection[eJOINTNAME_THUMB_3]  = -1;

	mMotorDirection[eJOINTNAME_INDEX_0]  = -1;
	mMotorDirection[eJOINTNAME_INDEX_1]  =  1;
	mMotorDirection[eJOINTNAME_INDEX_2]  = -1;
	mMotorDirection[eJOINTNAME_INDEX_3]  = -1;
	mMotorDirection[eJOINTNAME_MIDDLE_0] = -1;
	mMotorDirection[eJOINTNAME_MIDDLE_1] =  1;
	mMotorDirection[eJOINTNAME_MIDDLE_2] = -1;
	mMotorDirection[eJOINTNAME_MIDDLE_3] = -1;
	mMotorDirection[eJOINTNAME_PINKY_0]  = -1;
	mMotorDirection[eJOINTNAME_PINKY_1]  =  1;
	mMotorDirection[eJOINTNAME_PINKY_2]  = -1;
	mMotorDirection[eJOINTNAME_PINKY_3]  = -1;
	mMotorDirection[eJOINTNAME_THUMB_0]  = -1;
	mMotorDirection[eJOINTNAME_THUMB_1]  = -1;
	mMotorDirection[eJOINTNAME_THUMB_2]  =  1;
	mMotorDirection[eJOINTNAME_THUMB_3]  =  1; 
*/
// end SAH01010002 Right 	
	

/* // SAH01020006 Right
	mEncoderOffset[eJOINTNAME_INDEX_0]  =  1465;
	mEncoderOffset[eJOINTNAME_INDEX_1]  = -65230;
	mEncoderOffset[eJOINTNAME_INDEX_2]  =   13;
	mEncoderOffset[eJOINTNAME_INDEX_3]  =    697;
	
	mEncoderOffset[eJOINTNAME_MIDDLE_0] =    1251;
	mEncoderOffset[eJOINTNAME_MIDDLE_1] = -66019;
	mEncoderOffset[eJOINTNAME_MIDDLE_2] =     420;
	mEncoderOffset[eJOINTNAME_MIDDLE_3] =   -413;
	
	mEncoderOffset[eJOINTNAME_PINKY_0]  =    377;
	mEncoderOffset[eJOINTNAME_PINKY_1]  = -63587;
	mEncoderOffset[eJOINTNAME_PINKY_2]  =    1017;
	mEncoderOffset[eJOINTNAME_PINKY_3]  =   -50;
	
	mEncoderOffset[eJOINTNAME_THUMB_0]  =    13;
	mEncoderOffset[eJOINTNAME_THUMB_1]  =   1294;
	mEncoderOffset[eJOINTNAME_THUMB_2]  = -65652;
	mEncoderOffset[eJOINTNAME_THUMB_3]  = -66046;
//
	mEncoderDirection[eJOINTNAME_INDEX_0]  =  1;
	mEncoderDirection[eJOINTNAME_INDEX_1]  = -1;
	mEncoderDirection[eJOINTNAME_INDEX_2]  =  1;
	mEncoderDirection[eJOINTNAME_INDEX_3]  =  1;
	
	mEncoderDirection[eJOINTNAME_MIDDLE_0] =  1;
	mEncoderDirection[eJOINTNAME_MIDDLE_1] = -1;
	mEncoderDirection[eJOINTNAME_MIDDLE_2] =  1;
	mEncoderDirection[eJOINTNAME_MIDDLE_3] =  1;
	
	mEncoderDirection[eJOINTNAME_PINKY_0]  =  1;
	mEncoderDirection[eJOINTNAME_PINKY_1]  = -1;
	mEncoderDirection[eJOINTNAME_PINKY_2]  =  1;
	mEncoderDirection[eJOINTNAME_PINKY_3]  =  1;
	
	mEncoderDirection[eJOINTNAME_THUMB_0]  =  1;
	mEncoderDirection[eJOINTNAME_THUMB_1]  =  1;
	mEncoderDirection[eJOINTNAME_THUMB_2]  = -1;
	mEncoderDirection[eJOINTNAME_THUMB_3]  = -1;
//
	mMotorDirection[eJOINTNAME_INDEX_0]  = 1;
	mMotorDirection[eJOINTNAME_INDEX_1]  =  -1;
	mMotorDirection[eJOINTNAME_INDEX_2]  = 1;
	mMotorDirection[eJOINTNAME_INDEX_3]  = 1;
	
	mMotorDirection[eJOINTNAME_MIDDLE_0] = 1;
	mMotorDirection[eJOINTNAME_MIDDLE_1] =  -1;
	mMotorDirection[eJOINTNAME_MIDDLE_2] = 1;
	mMotorDirection[eJOINTNAME_MIDDLE_3] = 1;
	
	mMotorDirection[eJOINTNAME_PINKY_0]  = 1;
	mMotorDirection[eJOINTNAME_PINKY_1]  =  -1;
	mMotorDirection[eJOINTNAME_PINKY_2]  = 1;
	mMotorDirection[eJOINTNAME_PINKY_3]  = 1;
	
	mMotorDirection[eJOINTNAME_THUMB_0]  = 1;
	mMotorDirection[eJOINTNAME_THUMB_1]  = 1;
	mMotorDirection[eJOINTNAME_THUMB_2]  =  -1;
	mMotorDirection[eJOINTNAME_THUMB_3]  =  -1; 

// end SAH01020006 Right 
*/

}

controlAllegroHand::~controlAllegroHand()
{
	PRINT_INFO("Setting System OFF");
	_writeDeviceMsg(ID_CMD_SET_SYSTEM_OFF, ID_DEVICE_MAIN, ID_COMMON);
	usleep(10000);

	if(CAN_Close(CanHandle))
	{
		PRINT_INFO("Error in CAN_Close()");
	}
}

void controlAllegroHand::init(int mode)
{

	unsigned char data[8];
	int ret;

	PRINT_INFO("Opening CAN device");

	CanHandle = LINUX_CAN_Open("/dev/pcan32", O_RDWR);
	if (!CanHandle)
	{
		PRINT_INFO("Error in CAN_Open()");
	}

	char txt[VERSIONSTRING_LEN];
	ret = CAN_VersionInfo(CanHandle, txt);
	if (!ret)
	{
		PRINT_INFO(txt);
	}
	else {
		PRINT_INFO("Error getting CAN_VersionInfo()");
	}

	PRINT_INFO("Initializing CAN device");
	// init to an user defined bit rate
	ret = CAN_Init(CanHandle, CAN_BAUD_1M, CAN_INIT_TYPE_ST);
	if (ret)
	{
		PRINT_INFO("Error in CAN_Init()");
	}

	// system off
	_writeDeviceMsg(ID_CMD_SET_SYSTEM_OFF, ID_DEVICE_MAIN, ID_COMMON);
	usleep(10000);

	PRINT_INFO("Setting loop period = 3 ms");
	data[0] = (char)(ALLEGRO_CONTROL_TIME_INTERVAL * 1000.);
	_writeDeviceMsg(ID_CMD_SET_PERIOD, ID_DEVICE_MAIN, ID_COMMON, 1, data );
	usleep(100000);

	PRINT_INFO("Setting task mode");
	_writeDeviceMsg(ID_CMD_SET_MODE_TASK, ID_DEVICE_MAIN, ID_COMMON);
	usleep(100000);

	PRINT_INFO("Setting joint query command");
	_writeDeviceMsg(ID_CMD_QUERY_STATE_DATA, ID_DEVICE_MAIN, ID_COMMON);
	usleep(100000);

	PRINT_INFO("Setting System ON");
	_writeDeviceMsg(ID_CMD_SET_SYSTEM_ON, ID_DEVICE_MAIN, ID_COMMON);
	usleep(100000);

	PRINT_INFO("Clear the CAN buffer");

	while( LINUX_CAN_Read_Timeout(CanHandle, &read_msg, 0) != 0 )
	{
		usleep(1000);
	}

	// write initial read command
	_writeDeviceMsg(ID_CMD_QUERY_STATE_DATA, ID_DEVICE_MAIN, ID_COMMON);
}

int controlAllegroHand::update(void)
{
	_readDevices();
	_writeDevices();

	if(mEmergencyStop == true)
	{
		return -1;
	}
	else
	{
		return 0;
	}
}

int  controlAllegroHand::command(const short& cmd, const int& arg)
{
	return 0;
}

void controlAllegroHand::setTorque(double *torque)
{
	for(int i=0; i<DOF_JOINTS; i++)
	{
		desired_torque[i] = torque[i];
	}
}

void controlAllegroHand::getJointInfo(double *position, double *torque)
{
	for(int i=0; i<DOF_JOINTS; i++)
	{
		position[i] = curr_position[i];
		torque[i] = curr_torque[i];
	}
}


void controlAllegroHand::_readDevices()
{
	double q[4];
	char lID;
	int ret = 0;
	int itr = 0;
	static int errorcnt = 0;

	while( true )
	{
		//ret=LINUX_CAN_Read_Timeout(CanHandle, &read_msg, 2900); // timeout in microsecond
		ret=LINUX_CAN_Read_Timeout(CanHandle, &read_msg, 0); // 0 : polling

		if (ret)
		{
			//PRINT_INFO("CAN communication error (reading timeout)");
			break;
		}
		else
		{
			lID  = _parseCANMsg(read_msg.Msg, q);
			if( (lID >= ID_DEVICE_SUB_01) && (lID <= ID_DEVICE_SUB_04) )
			{
				for(int i=0; i<4; i++)
				{
					curr_position[i+4*(lID-ID_DEVICE_SUB_01)] = q[i];
				}
				itr++;
			}
			if( lID < 0 )
			{
				mEmergencyStop = true;
			}
		}
	}
	if( itr < 4)
	{
		errorcnt++;
		if( errorcnt > 5 ){
			cout << "read error" << endl;
			mEmergencyStop = true;
		}
	}
	else{
		errorcnt = 0;
	}


}

void controlAllegroHand::_writeDevices()
{
	double pwmDouble[DOF_JOINTS];
	short pwm[DOF_JOINTS];
	unsigned char data[8];

	// convert to torque to pwm
	for(int i=0; i<DOF_JOINTS; i++ ){
		pwmDouble[i] =  desired_torque[i] *1.0 * (double)mMotorDirection[i] *800.0;

		mPWM_MAX[i] = 800.0;

		// limitation should be less than 800
		if     ( pwmDouble[i] >  mPWM_MAX[i] )
		{
			pwmDouble[i] =  mPWM_MAX[i];
			cout <<i << " max"<< endl;
		}
		else if( pwmDouble[i] < -mPWM_MAX[i] ) {
			pwmDouble[i] = -mPWM_MAX[i];
			cout <<i<< " min"<< endl;
		}

		pwm[i] = (short)pwmDouble[i];
	}


	for(int findex=0; findex<4; findex++ ){
		data[0] = (unsigned char)( (pwm[0+findex*4] >> 8) & 0x00ff);
		data[1] = (unsigned char)(  pwm[0+findex*4]       & 0x00ff);
		data[2] = (unsigned char)( (pwm[1+findex*4] >> 8) & 0x00ff);
		data[3] = (unsigned char)(  pwm[1+findex*4]       & 0x00ff);
		data[4] = (unsigned char)( (pwm[2+findex*4] >> 8) & 0x00ff);
		data[5] = (unsigned char)(  pwm[2+findex*4]       & 0x00ff);
		data[6] = (unsigned char)( (pwm[3+findex*4] >> 8) & 0x00ff);
		data[7] = (unsigned char)(  pwm[3+findex*4]       & 0x00ff);

		_writeDeviceMsg( (unsigned long)(ID_CMD_SET_TORQUE_1 + findex), ID_DEVICE_MAIN, ID_COMMON, 8, data);
	}

	// send message to call joint position and torque query
	_writeDeviceMsg(ID_CMD_QUERY_STATE_DATA, ID_DEVICE_MAIN, ID_COMMON);

}

void controlAllegroHand::_writeDeviceMsg(unsigned long command, unsigned long from, unsigned long to, BYTE len, unsigned char *data)
{
	TPCANMsg msg1;
	long Txid;

	Txid = ((unsigned long)command<<6) | ((unsigned long)to <<3) | ((unsigned long)from);
	msg1.ID  = Txid;
	msg1.MSGTYPE  = MSGTYPE_STANDARD;
	msg1.LEN  = len;

	for(BYTE i=0; i<len; i++)
	{
		msg1.DATA[i] = data[i];
	}

	if(CAN_Write(CanHandle, &msg1))
	{
		cout << "CAN communication error (write)" << endl;
	}

}

void controlAllegroHand::_writeDeviceMsg(unsigned long command, unsigned long from, unsigned long to)
{
	_writeDeviceMsg(command, from, to, 0, NULL);
}



char controlAllegroHand::_parseCANMsg(TPCANMsg &read_msg,  double *values)
{
	char cmd, src, to;
	int len;
	unsigned char tmpdata[8];
	int tmppos[4];
	int lIndexBase;

	cmd = (char)( (read_msg.ID >> 6) & 0x1f );
	to  = (char)( (read_msg.ID >> 3) & 0x07 );
	src = (char)( read_msg.ID & 0x07 );
	len = (int)( read_msg.LEN );
	for(int nd=0; nd<len; nd++)
		tmpdata[nd] = read_msg.DATA[nd];

	switch (cmd)
	{
	case ID_CMD_QUERY_STATE_DATA:
		if (src >= ID_DEVICE_SUB_01 && src <= ID_DEVICE_SUB_04)
		{

			tmppos[0] = (int)(tmpdata[0] | (tmpdata[1] << 8));
			tmppos[1] = (int)(tmpdata[2] | (tmpdata[3] << 8));
			tmppos[2] = (int)(tmpdata[4] | (tmpdata[5] << 8));
			tmppos[3] = (int)(tmpdata[6] | (tmpdata[7] << 8));

			lIndexBase = 4*(src-ID_DEVICE_SUB_01);

			//values[0] = (double)mEncoderDirection[lIndexBase+0] * (double)(tmppos[0] - 32768 - mEncoderOffset[lIndexBase+0]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);
			//values[1] = (double)mEncoderDirection[lIndexBase+1] * (double)(tmppos[1] - 32768 - mEncoderOffset[lIndexBase+1]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);
			//values[2] = (double)mEncoderDirection[lIndexBase+2] * (double)(tmppos[2] - 32768 - mEncoderOffset[lIndexBase+2]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);
			//values[3] = (double)mEncoderDirection[lIndexBase+3] * (double)(tmppos[3] - 32768 - mEncoderOffset[lIndexBase+3]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);
			values[0] = (double)(mEncoderDirection[lIndexBase+0] *tmppos[0] - 32768 - mEncoderOffset[lIndexBase+0]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);
			values[1] = (double)(mEncoderDirection[lIndexBase+1] *tmppos[1] - 32768 - mEncoderOffset[lIndexBase+1]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);
			values[2] = (double)(mEncoderDirection[lIndexBase+2] *tmppos[2] - 32768 - mEncoderOffset[lIndexBase+2]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);
			values[3] = (double)(mEncoderDirection[lIndexBase+3] *tmppos[3] - 32768 - mEncoderOffset[lIndexBase+3]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);

			return src;

		}
		else
		{
			cout << "No subdevice match!" << endl;
			return -1;
		}

		break;
		//TODO: Implement this
	case ID_CMD_QUERY_CONTROL_DATA:
		return 0;
		break;
	default:
		printf("unknown command %d, src %d, to %d, len %d \n", cmd, src, to, len);
		/*
		for(int nd=0; nd<len; nd++)
		{
			printf("%d \n ", tmpdata[nd]);
		}
		*/
		return -1;
		break;
	}

}
