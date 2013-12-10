/*
 * 	controlAllegroHand.cpp
 *
 *  Created on: 		Nov 15, 2012
 *  Added to Project: 	Jan 17, 2013
 *  Author: 			Seungsu Kim & Alex Alspach
 */

#include "controlAllegroHand.h"
#include <iostream>
#include <math.h>
#include <stdio.h>
#include "ros/ros.h"
#include <string>

using namespace std;




void PRINT_INFO(const char *msg)
{
	cout << msg << endl;
}

controlAllegroHand::controlAllegroHand()
{

	if (ros::param::has("~zero"))
	{
		mEmergencyStop = false;	
		ROS_INFO("\n\nCAN: Joint zeros and directions loaded from parameter server.\n");	
	}
	else
	{
		ROS_ERROR("\n\nEncoder/Motor offsets and directions not loaded.\nCheck launch file is loading /parameters/zero.yaml\nShutting down...\n");
		mEmergencyStop = true;
	}
	
	// This version number is used to in setting the finger motor CAN channels
	// the channels used differ from versions 1.0 to 2.0
	
	ros::param::get("~hand_info/version",hand_version);


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

	ros::param::get("~zero/encoder_offset/j00",mEncoderOffset[eJOINTNAME_INDEX_0]);
	ros::param::get("~zero/encoder_offset/j01",mEncoderOffset[eJOINTNAME_INDEX_1]);
	ros::param::get("~zero/encoder_offset/j02",mEncoderOffset[eJOINTNAME_INDEX_2]);
	ros::param::get("~zero/encoder_offset/j03",mEncoderOffset[eJOINTNAME_INDEX_3]);
	ros::param::get("~zero/encoder_offset/j10",mEncoderOffset[eJOINTNAME_MIDDLE_0]);
	ros::param::get("~zero/encoder_offset/j11",mEncoderOffset[eJOINTNAME_MIDDLE_1]);
	ros::param::get("~zero/encoder_offset/j12",mEncoderOffset[eJOINTNAME_MIDDLE_2]);
	ros::param::get("~zero/encoder_offset/j13",mEncoderOffset[eJOINTNAME_MIDDLE_3]);
	ros::param::get("~zero/encoder_offset/j20",mEncoderOffset[eJOINTNAME_PINKY_0]);
	ros::param::get("~zero/encoder_offset/j21",mEncoderOffset[eJOINTNAME_PINKY_1]);
	ros::param::get("~zero/encoder_offset/j22",mEncoderOffset[eJOINTNAME_PINKY_2]);
	ros::param::get("~zero/encoder_offset/j23",mEncoderOffset[eJOINTNAME_PINKY_3]);
	ros::param::get("~zero/encoder_offset/j30",mEncoderOffset[eJOINTNAME_THUMB_0]);
	ros::param::get("~zero/encoder_offset/j31",mEncoderOffset[eJOINTNAME_THUMB_1]);
	ros::param::get("~zero/encoder_offset/j32",mEncoderOffset[eJOINTNAME_THUMB_2]);
	ros::param::get("~zero/encoder_offset/j33",mEncoderOffset[eJOINTNAME_THUMB_3]);
	
	ros::param::get("~zero/encoder_direction/j00",mEncoderDirection[eJOINTNAME_INDEX_0]);
	ros::param::get("~zero/encoder_direction/j01",mEncoderDirection[eJOINTNAME_INDEX_1]);
	ros::param::get("~zero/encoder_direction/j02",mEncoderDirection[eJOINTNAME_INDEX_2]);
	ros::param::get("~zero/encoder_direction/j03",mEncoderDirection[eJOINTNAME_INDEX_3]);
	ros::param::get("~zero/encoder_direction/j10",mEncoderDirection[eJOINTNAME_MIDDLE_0]);
	ros::param::get("~zero/encoder_direction/j11",mEncoderDirection[eJOINTNAME_MIDDLE_1]);
	ros::param::get("~zero/encoder_direction/j12",mEncoderDirection[eJOINTNAME_MIDDLE_2]);
	ros::param::get("~zero/encoder_direction/j13",mEncoderDirection[eJOINTNAME_MIDDLE_3]);
	ros::param::get("~zero/encoder_direction/j20",mEncoderDirection[eJOINTNAME_PINKY_0]);
	ros::param::get("~zero/encoder_direction/j21",mEncoderDirection[eJOINTNAME_PINKY_1]);
	ros::param::get("~zero/encoder_direction/j22",mEncoderDirection[eJOINTNAME_PINKY_2]);
	ros::param::get("~zero/encoder_direction/j23",mEncoderDirection[eJOINTNAME_PINKY_3]);
	ros::param::get("~zero/encoder_direction/j30",mEncoderDirection[eJOINTNAME_THUMB_0]);
	ros::param::get("~zero/encoder_direction/j31",mEncoderDirection[eJOINTNAME_THUMB_1]);
	ros::param::get("~zero/encoder_direction/j32",mEncoderDirection[eJOINTNAME_THUMB_2]);
	ros::param::get("~zero/encoder_direction/j33",mEncoderDirection[eJOINTNAME_THUMB_3]);
	
	ros::param::get("~zero/motor_direction/j00",mMotorDirection[eJOINTNAME_INDEX_0]);
	ros::param::get("~zero/motor_direction/j01",mMotorDirection[eJOINTNAME_INDEX_1]);
	ros::param::get("~zero/motor_direction/j02",mMotorDirection[eJOINTNAME_INDEX_2]);
	ros::param::get("~zero/motor_direction/j03",mMotorDirection[eJOINTNAME_INDEX_3]);
	ros::param::get("~zero/motor_direction/j10",mMotorDirection[eJOINTNAME_MIDDLE_0]);
	ros::param::get("~zero/motor_direction/j11",mMotorDirection[eJOINTNAME_MIDDLE_1]);
	ros::param::get("~zero/motor_direction/j12",mMotorDirection[eJOINTNAME_MIDDLE_2]);
	ros::param::get("~zero/motor_direction/j13",mMotorDirection[eJOINTNAME_MIDDLE_3]);
	ros::param::get("~zero/motor_direction/j20",mMotorDirection[eJOINTNAME_PINKY_0]);
	ros::param::get("~zero/motor_direction/j21",mMotorDirection[eJOINTNAME_PINKY_1]);
	ros::param::get("~zero/motor_direction/j22",mMotorDirection[eJOINTNAME_PINKY_2]);
	ros::param::get("~zero/motor_direction/j23",mMotorDirection[eJOINTNAME_PINKY_3]);
	ros::param::get("~zero/motor_direction/j30",mMotorDirection[eJOINTNAME_THUMB_0]);
	ros::param::get("~zero/motor_direction/j31",mMotorDirection[eJOINTNAME_THUMB_1]);
	ros::param::get("~zero/motor_direction/j32",mMotorDirection[eJOINTNAME_THUMB_2]);
	ros::param::get("~zero/motor_direction/j33",mMotorDirection[eJOINTNAME_THUMB_3]);	
	
}


controlAllegroHand::~controlAllegroHand()
{
	//PRINT_INFO("Setting System OFF");
	ROS_INFO("Setting System OFF");
	_writeDeviceMsg(ID_CMD_SET_SYSTEM_OFF, ID_DEVICE_MAIN, ID_COMMON);
	usleep(10000);

	if(CAN_Close(CanHandle))
	{
		//PRINT_INFO("Error in CAN_Close()");
		ROS_ERROR("Error in CAN_Close()");
	}
}

void controlAllegroHand::init(int mode)
{

	unsigned char data[8];
	int ret;
	TPCANRdMsg lmsg;

	//PRINT_INFO("Opening CAN device");
	ROS_INFO("CAN: Opening device");
	

	string CAN_CH;
	ros::param::get("~comm/CAN_CH",CAN_CH);
	const char * CAN_CH_c = CAN_CH.c_str();
	
	//ROS_WARN("[arm] Failed to find %s segment in the KDL chain with a tip at %s.", name.c_str(), chain_tip_name_.c_str());
	
	CanHandle = LINUX_CAN_Open(CAN_CH_c, O_RDWR);
	if (!CanHandle)
	{
		//PRINT_INFO("Error in CAN_Open()");
		ROS_ERROR("CAN: Error in CAN_Open() on Channel %s", CAN_CH_c );
	}
	else
	{
		ROS_WARN("CAN: Success Opening Channel %s", CAN_CH_c );
	}

	char txt[VERSIONSTRING_LEN];
	ret = CAN_VersionInfo(CanHandle, txt);
	if (!ret)
	{
		//PRINT_INFO(txt);
		ROS_INFO("CAN: %s", txt);
	}
	else {
		//PRINT_INFO("Error getting CAN_VersionInfo()");
		ROS_ERROR("CAN: Error in CAN_VersionInfo()");
	}

	//PRINT_INFO("Initializing CAN device");
	ROS_INFO("CAN: Initializing device");
	// init to an user defined bit rate
	ret = CAN_Init(CanHandle, CAN_BAUD_1M, CAN_INIT_TYPE_ST);
	if (ret)
	{
		//PRINT_INFO("Error in CAN_Init()");
		ROS_ERROR("CAN: Error in CAN_Init()");
	}

	//PRINT_INFO("Clear the can buffer");
	ROS_INFO("CAN: Clearing the CAN buffer");
	for(int i=0; i<100; i++){
		LINUX_CAN_Read_Timeout(CanHandle, &lmsg, 1000); // polding
	}

	//PRINT_INFO("System off");
	ROS_INFO("CAN: System off");
	_writeDeviceMsg(ID_CMD_SET_SYSTEM_OFF, ID_DEVICE_MAIN, ID_COMMON);
	usleep(100);

	//PRINT_INFO("Setting loop period = 3 ms");
	ROS_INFO("CAN: Setting loop period = 3 ms");
	//data[0] = (char)(ALLEGRO_CONTROL_TIME_INTERVAL * 1000.);
	data[0] = 3;
	_writeDeviceMsg(ID_CMD_SET_PERIOD, ID_DEVICE_MAIN, ID_COMMON, 1, data );
	usleep(100);

	//PRINT_INFO("Setting task mode");
	ROS_INFO("CAN: Setting task mode");
	_writeDeviceMsg(ID_CMD_SET_MODE_TASK, ID_DEVICE_MAIN, ID_COMMON);
	usleep(100);

	//PRINT_INFO("Setting System ON");
	ROS_INFO("CAN: Setting System ON");
	_writeDeviceMsg(ID_CMD_SET_SYSTEM_ON, ID_DEVICE_MAIN, ID_COMMON);
	usleep(100);

	for(int i=0; i<100; i++) ret=LINUX_CAN_Read_Timeout(CanHandle, &lmsg, 0);

	//PRINT_INFO("Setting joint query command");
	ROS_INFO("CAN: Setting joint query command");
	_writeDeviceMsg(ID_CMD_QUERY_STATE_DATA, ID_DEVICE_MAIN, ID_COMMON);
	usleep(100);

	// wait for the first command
	//PRINT_INFO("Wait for first joint command");
	int cnt = 0;
	int itr = 0;
	double q[4];
	char lID;
	while(true)
	{
		ret=LINUX_CAN_Read_Timeout(CanHandle, &lmsg, 1000000);
		lID  = _parseCANMsg( lmsg.Msg, q);
		if( (lID >= ID_DEVICE_SUB_01) && (lID <= ID_DEVICE_SUB_04) )
		{
			cnt++;
			if(cnt == 8) break;
		}
		else{
			itr++;
		}

		if(itr > 4){
			mEmergencyStop = true;
			break;
		}
	}

	//cout << "started" << endl;
	ROS_INFO("CAN: Communicating");
}

int controlAllegroHand::update(void)
{
	_readDevices();
	usleep(10);
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

//void controlAllegroHand::getJointInfo(double *position, double *torque)
void controlAllegroHand::getJointInfo(double *position)
{
	for(int i=0; i<DOF_JOINTS; i++)
	{
		position[i] = curr_position[i];
		//torque[i] = curr_torque[i];
	}
}

void controlAllegroHand::_readDevices()
{
	double q[4];
	char lID;
	int ret = 0;
	int itr = 0;
	static int errorcnt = 0;
	TPCANRdMsg lmsg;

	while( itr<4 )
	//while( true)
	{
		ret=LINUX_CAN_Read_Timeout(CanHandle, &lmsg, 3000); // timeout in micro second
		//ret=LINUX_CAN_Read_Timeout(CanHandle, &lmsg, 0); // 0 : polling

		if (ret)
		{
			break;
		}
		else
		{
			lID  = _parseCANMsg( lmsg.Msg, q);
			if( (lID >= ID_DEVICE_SUB_01) && (lID <= ID_DEVICE_SUB_04) )
			{
				for(int i=0; i<4; i++)
				{
					curr_position[i+4*(lID-ID_DEVICE_SUB_01)] = q[i];
				}
				itr++;
				//printf("%d, ", lID );
			}
			else if( lID == 0)
			{
				errorcnt = 0;
				//printf("(%d), ", lID );
			}
			else if( lID < 0 )
			{
				mEmergencyStop = true;
			}
		}
	}


	if( itr < 4)
	{
		//printf(": %d  \n", itr );
		errorcnt++;
		if( errorcnt > 3 ){
			mEmergencyStop = true;
		}
	}
	else{
		//printf(": %d  \n", itr );
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
			//cout <<i << " max"<< endl;
		}
		else if( pwmDouble[i] < -mPWM_MAX[i] ) {
			pwmDouble[i] = -mPWM_MAX[i];
			//cout <<i<< " min"<< endl;
		}

		pwm[i] = (short)pwmDouble[i];
	}
	
	
	// ZEROS FOR TESTING
        /*
	pwm[eJOINTNAME_INDEX_0] = 0;
	pwm[eJOINTNAME_INDEX_1] = 0;
	pwm[eJOINTNAME_INDEX_2] = 0;
	pwm[eJOINTNAME_INDEX_3] = 0;
	pwm[eJOINTNAME_MIDDLE_0] = 0;
	pwm[eJOINTNAME_MIDDLE_1] = 0;
	pwm[eJOINTNAME_MIDDLE_2] = 0;
	pwm[eJOINTNAME_MIDDLE_3] = 0;
	pwm[eJOINTNAME_PINKY_0] = 0;
	pwm[eJOINTNAME_PINKY_1] = 0;
	pwm[eJOINTNAME_PINKY_2] = 0;
	pwm[eJOINTNAME_PINKY_3] = 0;
	pwm[eJOINTNAME_THUMB_0] = 0;
	pwm[eJOINTNAME_THUMB_1] = 0;
	pwm[eJOINTNAME_THUMB_2] = 0;
	pwm[eJOINTNAME_THUMB_3] = 0;
        */
	//pwm[eJOINTNAME_THUMB_1] = 0;


if (hand_version == 1.0 )
{
	// for Allegro Hand v1.0
	for(int findex=0; findex<4; findex++ ){
		data[0] = (unsigned char)( (pwm[0+findex*4] >> 8) & 0x00ff);
		data[1] = (unsigned char)(  pwm[0+findex*4]       & 0x00ff);
		data[2] = (unsigned char)( (pwm[1+findex*4] >> 8) & 0x00ff);
		data[3] = (unsigned char)(  pwm[1+findex*4]       & 0x00ff);
		data[4] = (unsigned char)( (pwm[2+findex*4] >> 8) & 0x00ff);
		data[5] = (unsigned char)(  pwm[2+findex*4]       & 0x00ff);
		data[6] = (unsigned char)( (pwm[3+findex*4] >> 8) & 0x00ff);
		data[7] = (unsigned char)(  pwm[3+findex*4]       & 0x00ff);

		_writeDeviceMsg( (DWORD)(ID_CMD_SET_TORQUE_1 + findex), ID_DEVICE_MAIN, ID_COMMON, 8, data);
		usleep(10);
	}

}
else if (hand_version >= 2.0 )
{
	// for Allegro Hand v2.0
	for(int findex=0; findex<4; findex++ ){
		data[0] = (unsigned char)( (pwm[3+findex*4] >> 8) & 0x00ff);
		data[1] = (unsigned char)(  pwm[3+findex*4]       & 0x00ff);
		data[2] = (unsigned char)( (pwm[2+findex*4] >> 8) & 0x00ff);
		data[3] = (unsigned char)(  pwm[2+findex*4]       & 0x00ff);
		data[4] = (unsigned char)( (pwm[1+findex*4] >> 8) & 0x00ff);
		data[5] = (unsigned char)(  pwm[1+findex*4]       & 0x00ff);
		data[6] = (unsigned char)( (pwm[0+findex*4] >> 8) & 0x00ff);
		data[7] = (unsigned char)(  pwm[0+findex*4]       & 0x00ff);

		_writeDeviceMsg( (DWORD)(ID_CMD_SET_TORQUE_1 + findex), ID_DEVICE_MAIN, ID_COMMON, 8, data);
		usleep(10);
	}
}	
else
{
		ROS_ERROR("CAN: Can not determine proper finger CAN channels. Check the Allegro Hand version number in 'zero.yaml'");
}	

	// send message to call joint position and torque query
	// _writeDeviceMsg(ID_CMD_QUERY_STATE_DATA, ID_DEVICE_MAIN, ID_COMMON);
	// usleep(10);
}

void controlAllegroHand::_writeDeviceMsg(DWORD command, DWORD from, DWORD to, BYTE len, unsigned char *data)
{
	TPCANMsg msg1;
	DWORD Txid;

	Txid = (command<<6) | (to <<3) | (from);
	msg1.ID  = Txid;
	msg1.MSGTYPE  = MSGTYPE_STANDARD;
	msg1.LEN  = len;
	for(BYTE i=0; i<8; i++) msg1.DATA[i] = 0;

	for(BYTE i=0; i<len; i++)
	{
		msg1.DATA[i] = data[i];
	}

	//if(LINUX_CAN_Write_Timeout(CanHandle, &msg1, 0.0))
	if(CAN_Write(CanHandle, &msg1))
	{
		cout << "CAN communication error (write)" << endl;
		ROS_ERROR("CAN: Write error");
		mEmergencyStop = true;
	}

}

void controlAllegroHand::_writeDeviceMsg(DWORD command, DWORD from,DWORD to)
{
	_writeDeviceMsg(command, from, to, 0, NULL);
}



char controlAllegroHand::_parseCANMsg(TPCANMsg read_msg,  double *values)
{
	unsigned char cmd, src, to;
	unsigned char len;
	unsigned char tmpdata[8];
	int tmppos[4];
	int lIndexBase;

	cmd = (unsigned char)( (read_msg.ID >> 6) & 0x1f );
	to  = (unsigned char)( (read_msg.ID >> 3) & 0x07 );
	src = (unsigned char)( read_msg.ID & 0x07 );
	len = (unsigned char)( read_msg.LEN );
	for(unsigned int nd=0; nd<len; nd++)
		tmpdata[nd] = read_msg.DATA[nd];

	switch (cmd)
	{
	case ID_CMD_QUERY_CONTROL_DATA:
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
			values[0] = ((double)mEncoderDirection[lIndexBase+0] *(double)tmppos[0] - 32768.0 - (double)mEncoderOffset[lIndexBase+0]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);
			values[1] = ((double)mEncoderDirection[lIndexBase+1] *(double)tmppos[1] - 32768.0 - (double)mEncoderOffset[lIndexBase+1]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);
			values[2] = ((double)mEncoderDirection[lIndexBase+2] *(double)tmppos[2] - 32768.0 - (double)mEncoderOffset[lIndexBase+2]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);
			values[3] = ((double)mEncoderDirection[lIndexBase+3] *(double)tmppos[3] - 32768.0 - (double)mEncoderOffset[lIndexBase+3]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);

			return src;

		}
		else
		{
			cout << "No subdevice match!" << endl;
			return -1;
		}

		break;
		//TODO: Implement this
	case ID_CMD_QUERY_STATE_DATA:
		return 0;
		break;
	default:
		//printf("unknown command %d, src %d, to %d, len %d \n", cmd, src, to, len);
		ROS_WARN("unknown command %d, src %d, to %d, len %d", cmd, src, to, len);
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
