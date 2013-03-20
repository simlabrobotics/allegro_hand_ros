/*
 * 	controlAllegroHand.h
 *
 *  Created on: 		Nov 15, 2012
 *  Added to Project: 	Jan 17, 2013
 *  Authors: 			Seungsu Kim & Ashwini Schukla
 */

#ifndef CONTROLALLEGROHAND_H_
#define CONTROLALLEGROHAND_H_

#include <list>
#include "BHand/BHand.h"
#include "allegroCANProtocol.h"
#include <libpcan.h>
#include <fcntl.h>
#include <string>

#define ALLEGRO_CONTROL_TIME_INTERVAL 0.003

#define PWM_LIMIT_ROLL 250.0*1.5
#define PWM_LIMIT_NEAR 450.0*1.5
#define PWM_LIMIT_MIDDLE 300.0*1.5
#define PWM_LIMIT_FAR 190.0*1.5

#define PWM_LIMIT_THUMB_ROLL 350.0*1.5
#define PWM_LIMIT_THUMB_NEAR 270.0*1.5
#define PWM_LIMIT_THUMB_MIDDLE 180.0*1.5
#define PWM_LIMIT_THUMB_FAR 180.0*1.5

class controlAllegroHand
{
public:
	controlAllegroHand();
	~controlAllegroHand();

	void init(int mode = 0);
	int update(void);
	int  command(const short& cmd, const int& arg = 0);

	void setTorque(double *torque);
//	void getJointInfo(double *position, double *torque);
	void getJointInfo(double *position);

private:
	HANDLE CanHandle;
	//TPCANMsg read_msg;
	TPCANRdMsg read_msg;

	double curr_position[DOF_JOINTS];
	double curr_torque[DOF_JOINTS];
	double desired_position[DOF_JOINTS];
	double desired_torque[DOF_JOINTS];
	
	double hand_version;

	double mPWM_MAX[DOF_JOINTS];
	int    mEncoderOffset[DOF_JOINTS];
	int    mEncoderDirection[DOF_JOINTS];
	int    mMotorDirection[DOF_JOINTS];
	
	//std::string hand_version;

	volatile bool mEmergencyStop;

	void _readDevices();
	DWORD _readDevicesCAN(unsigned char& id, double *position );
	void _writeDevices();
	void _parseCANMsg();

	void _writeDeviceMsg(DWORD command, DWORD from, DWORD to, BYTE len, unsigned char *data);
	void _writeDeviceMsg(DWORD command, DWORD from, DWORD to);
	//void _writeDeviceMsg(unsigned long command, unsigned long from, unsigned long to, BYTE len, unsigned char *data);
	//void _writeDeviceMsg(unsigned long command, unsigned long from, unsigned long to);
	char _parseCANMsg(TPCANMsg read_msg,  double *values);
	//char _parseCANMsg(TPCANMsg &read_msg,  double *values);

};


#endif /* CONTROLALLEGROHAND_H_ */
