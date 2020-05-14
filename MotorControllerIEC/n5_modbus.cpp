#include "n5_modbus.h"
#include <modbus/modbus.h>
#include <stdexcept>
#include <iostream>
#include <ctime>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>

//using namespace N5PosUnit;

N5Motor::N5Motor(std::string controller_IP) : controller_IP(controller_IP) {
	usePDO = false;
	statusword = std::bitset<16>(0);
	mb = modbus_new_tcp(controller_IP.c_str(), 502);
	modbus_set_slave(mb, 0);
	if (modbus_connect(mb)<0)
		throw std::runtime_error("Connection to Modbus Slave (MotorController) at " + controller_IP + " failed!");
	resetOperation();
	setupEndSwitches();
	//configurePDO();
	uint32_t val = 0x1;
	writeObject(ObjectIndexes().Motor_Drive_Submode_Select, (uint8_t)0x00, val);
}

N5Motor::~N5Motor() {
	modbus_close(mb);
	modbus_free(mb);
}

void N5Motor::setupEndSwitches()
{
	uint32_t val = 0x3;
	writeObject(ObjectIndexes().Digital_Inputs_Control, 0x01, val);
	usleep(SLEEPRW);
	writeObject(ObjectIndexes().Digital_Inputs_Control, 0x02, val);
	usleep(SLEEPRW);
	val = 0x0;
	writeObject(ObjectIndexes().Digital_Inputs_Control, 0x06, val);
	usleep(SLEEPRW);
	writeObject(ObjectIndexes().Limit_SwitTolerance_Band, 0x0, val);
	usleep(SLEEPRW);
}

int N5Motor::setOperationMode()
{
	return writeObject(ObjectIndexes().Modes_Of_Operation, 0, operationMode);
}

int N5Motor::setOperationMode(uint8_t newOperationMode)
{
	operationMode = newOperationMode;
	return setOperationMode();
}

int N5Motor::getOperationMode()
{
	return operationMode;
}

int N5Motor::setControlword()
{
	uint16_t rawControlword = static_cast<uint16_t>(controlword.to_ulong());
	auto ret = writeObject(ObjectIndexes().Controlword, 0, rawControlword);
	return ret;
}

int N5Motor::setControlword(uint16_t newControlword)
{
	controlword = std::bitset<16>(newControlword);
	auto ret = setControlword();
	return ret;
}

int N5Motor::setControlword(std::bitset<16> newControlword)
{
	controlword = newControlword;
	return setControlword();
}

int N5Motor::setTargetPosition(int pos) {
	targetPosition = pos;
	return 0;
}

std::bitset<16> N5Motor::getStatusword()
{
	uint16_t rawStatusword;
	readObject(ObjectIndexes().Statusword,0,rawStatusword);
	statusword = std::bitset<16>(rawStatusword);
	return statusword;
}

bool N5Motor::getStatusbit(int i) {
	std::bitset<16> statusword = getStatusword();
	bool statusbit = statusword[i];
	return statusbit;
}

int N5Motor::getActualPosition()
{
	int32_t actPosraw;
	readObject(ObjectIndexes().Position_Actual_Value, 0, actPosraw);
	actualPosition = actPosraw;
	return actualPosition;
}

int N5Motor::getFollowingError()
{
	int32_t followingerror;
	readObject(ObjectIndexes().Following_Error_Actual_Value, 0, followingerror);
	return followingerror;
}

int N5Motor::setUsePDO()
{
	usePDO = true;
	return 0;
}

int N5Motor::setDontUsePDO()
{
	usePDO = false;
	return 0;
}

bool N5Motor::isOperational()
{
	if (operational == true)
		return true;
	return false;
}

int N5Motor::quickStop()
{
	controlword[2] = 1;
	auto ret = setControlword();
	usleep(SLEEPRW);
	return ret;
}

int N5Motor::emergencyShutdown()
{
	controlword[0] = 0;
	controlword[1] = 0;
	controlword[2] = 1;
	auto ret = setControlword();
	usleep(SLEEPRW);
	return ret;
}

void N5Motor::setupProfilePositionMode()
{
	setOperationMode(0x1);
	usleep(SLEEPRW);
	startMode();
	usleep(SLEEPRW);
	writeObject(ObjectIndexes().Profile_Velocity, 0x0, maxvel);

	controlword[4] = 0;
	controlword[5] = 1;
	controlword[6] = 0;
	controlword[8] = 0;
	controlword[9] = 0;
	setControlword();
	startMode();
}

void N5Motor::setupProfileVelocityMode()
{
	setOperationMode(0x2);
	startMode();
}

void N5Motor::setupSynchronousPositionMode(int8_t msTimestep)
{
	msSyncTime = msTimestep;
	writeObject(ObjectIndexes().Interpolation_Time_Period, 0x01, msSyncTime);
	usleep(SLEEPRW);
	setOperationMode(8);
	usleep(SLEEPRW);
	startMode();

	//readObject(ObjectIndexes().Following_Error_Time_Out, 0x0, writePDO.followingerrortimeout);
	//readObject(ObjectIndexes().Following_Error_Window, 0x0, writePDO.followingerrorwindow);

	////Setup PDO
	////					Controlword	  Target Pos  Foll Err Window  Foll Err Timeout
	//int rxvals[] = { 0 , 0x60400010 , 0x607A0020 , 0x60650020 , 0x60660010 };
	//rxvals[0] = sizeof(rxvals) / sizeof(rxvals[0]) - 1;
	//for (int i; i < sizeof(rxvals) / sizeof(rxvals[0]); i++) {
	//	writeObject(ObjectIndexes().Modbus_Tx_PDO_Mapping, i, rxvals[i]);
	//}

	////					Statusword	  Pos Act Value	Pos Demand	Foll Err Act Value
	//int txvals[] = { 0 , 0x60410010 , 0x60640020 , 0x60620020 , 0x60F40020 };
	//txvals[0] = sizeof(txvals) / sizeof(txvals[0]) - 1;
	//for (int i; i < sizeof(txvals) / sizeof(txvals[0]); i++) {
	//	writeObject(ObjectIndexes().Modbus_Tx_PDO_Mapping, i, txvals[i]);
	//}
}

void N5Motor::setupSynchronousVelocityMode(int8_t msTimestep)
{
	msSyncTime = msTimestep;
	writeObject(ObjectIndexes().Interpolation_Time_Period, 0x01, msSyncTime);
	usleep(SLEEPRW);
	int32_t vel = 0;
	writeObject(ObjectIndexes().Target_Velocity, 0x01, vel);
	setOperationMode(9);
	usleep(SLEEPRW);
	startMode();
}

int N5Motor::resetOperation()
{
	std::clock_t starttime;
	starttime = std::clock();

	//Off
	setControlword(0x0);
	usleep(SLEEPRW);
	//Fault Reset
	setControlword(0x80);
	usleep(SLEEPRW);
	//Shutdown
	while (!getStatusbit(0) && (std::clock() - starttime) / ((double)CLOCKS_PER_SEC) < 1)
	{
		setControlword(0x06);
		usleep(SLEEPRW);
	}
	if ((std::clock() - starttime) / CLOCKS_PER_SEC > 0.5)
		return 1;
	return 0;
}

int N5Motor::turnOnEnableOperation()
{
	std::clock_t starttime;
	starttime = std::clock();
	while (getStatusbit(2) || !getStatusbit(1) && (std::clock() - starttime) / ((double)CLOCKS_PER_SEC) < 1)
	{
		setControlword(0x07);
		usleep(SLEEPRW);
	}
	//Enable Operation
	while (!getStatusbit(2) && (std::clock() - starttime) / ((double)CLOCKS_PER_SEC) < 1)
	{
		setControlword(0x0F);
		usleep(SLEEPRW);
	}
	if ((std::clock() - starttime) / CLOCKS_PER_SEC > 0.5)
		return 1;
	return 0;
}

int N5Motor::startMode()
{
	if (!getStatusbit(2) || getStatusbit(3) || !getStatusbit(5))
		resetOperation();
	return turnOnEnableOperation();
}

int N5Motor::calcSyncVelocity()
{
	int vel = (targetPosition - actualPosition) * 10 / (msSyncTime * 12);
	if (abs(vel) > maxvel)
	{
		vel = maxvel*vel / abs(vel);
	}
	return vel;
}

int N5Motor::writeTargetPosition()
{
	if (usePDO)
	{
		controlword = 0x0F;
		return writeProcessData();
	}

	if (operationMode == MODE_SYNC_VELOCITY && msSyncTime > 0)
	{
		targetVelocity = calcSyncVelocity();
		return writeTargetVelocity();
	}

	uint32_t rawPos = targetPosition;
	return writeObject(ObjectIndexes().Target_Position, 0, rawPos);
}

int N5Motor::writeTargetVelocity()
{
	uint32_t rawVel = targetVelocity;
	return writeObject(ObjectIndexes().Target_Velocity, 0, rawVel);
}

int N5Motor::startMoveToPosition() {
	controlword[4] = 0;
	setControlword(0x0F);
	usleep(SLEEPRW);
	controlword[4] = 1;
	int ret = 0;
	ret = setControlword(0x1F);
	return ret;
}

bool N5Motor::targetReached()
{
	getStatusword();
	if (getStatusbit(10) || !getStatusbit(5) || getStatusbit(3))
		return true;
	return false;
}

int N5Motor::shutdown() {
	return setControlword(0x0);
}

int N5Motor::readProcessData() {
	uint16_t rawPDO[MODBUS_MAX_ADU_LENGTH];
	int ret = modbus_read_registers(mb, 5000, 7, rawPDO);
	readPDO.statusword = std::bitset<16>(rawPDO[0]);
	readPDO.posactualvalue = (rawPDO[2] << 16) | rawPDO[3];
	readPDO.velocityactualvalue = rawPDO[4];
	readPDO.endswitchstatus = (rawPDO[5] << 16) | rawPDO[6];
	return ret;
}

//void N5Motor::configurePDO() {
//	uint8_t nTxmaps = 4;
//	writeObject(ObjectIndexes().Modbus_Tx_PDO_Mapping, 0x0, nTxmaps);
//
//	uint32_t mapvalsTx[nTxmaps];
//	mapvalsTx[0] = ObjectIndexes().Statusword						<< 16 | 0x10;
//	mapvalsTx[1] = ObjectIndexes().Position_Actual_Value << 16 | 0x20;
//	mapvalsTx[2] = ObjectIndexes().Position_Demand_Value << 16 | 0x20;
//	mapvalsTx[3] = ObjectIndexes().Following_Error_Actual_Value << 16 | 0x20;
//
//	for (uint8_t i = 0; i < nTxmaps; i++)
//	{
//		writeObject(ObjectIndexes().Modbus_Tx_PDO_Mapping, i+1, mapvalsTx[i]);
//	}
//
//	uint8_t nRxmaps = 4;
//	writeObject(ObjectIndexes().Modbus_Rx_PDO_Mapping, 0x0, nRxmaps);
//
//	uint32_t mapvalsRx[nRxmaps];
//	mapvalsRx[0] = ObjectIndexes().Controlword << 16 | 0x10;
//	mapvalsRx[1] = ObjectIndexes().Target_Position << 16 | 0x20;
//	mapvalsRx[2] = ObjectIndexes().Following_Error_Window << 16 | 0x20;
//	mapvalsRx[3] = ObjectIndexes().Following_Error_Time_Out << 16 | 0x20;
//
//	for (uint8_t i = 0; i < nRxmaps; i++)
//	{
//		writeObject(ObjectIndexes().Modbus_Rx_PDO_Mapping, i + 1, mapvalsRx[i]);
//	}
//
//	//usePDO = true;
//}

struct readPDO N5Motor::getPDO() {
	return readPDO;
}

struct readPDO N5Motor::getReadPDO() {
	readProcessData();
	return getPDO();
}

int N5Motor::writeProcessData() {
	uint16_t rawPDO[MODBUS_MAX_ADU_LENGTH];
	rawPDO[0] = (uint16_t) controlword.to_ulong();
	rawPDO[1] = (uint16_t) operationMode;
	rawPDO[2] = 0; //Motor Drive Submode 1
	rawPDO[3] = 1; //Closed Loop operation
	rawPDO[4] = (uint16_t) (targetPosition >> 16);
	rawPDO[5] = (uint16_t) (targetPosition & 0xFFFF);
	rawPDO[6] = (uint16_t) (maxvel >> 16); //Profile Vel
	rawPDO[7] = (uint16_t) (maxvel & 0xFFFF); //Profile Vel
	rawPDO[8] = (uint16_t) (maxvel >> 16); //VI Vel
	rawPDO[9] = (uint16_t) (maxvel & 0xFFFF); //VI Vel
	rawPDO[10] = 0; //Digital output
	rawPDO[11] = 0; //Digital output
	return modbus_write_registers(mb, 6000, 12,rawPDO);
}

int N5Motor::setFollowingErrorTimeout(uint16_t timeout) {
	writeObject(ObjectIndexes().Following_Error_Time_Out,0x00,timeout);
	return 0;
}

void N5Motor::setOffset(int32_t offsetenc)
{
	int32_t negoffset = -1*offsetenc;
	writeObject(ObjectIndexes().Home_Offset, 0x0, negoffset);
	usleep(SLEEPRW);
}

void N5Motor::startHoming(int32_t offset)
{
	setOffset(offset); // Standard from zero. Avoiding issues with tolerance band

	//Method 1: Move to negative endswitch and set offset. Go to new zero. (?)
	int8_t homing_method = 17;
	writeObject(ObjectIndexes().Homing_Method, 0x0, homing_method);
	usleep(SLEEPRW);

	//Define Maxvel before homing
	uint32_t homing_vel = 50;
	writeObject(ObjectIndexes().Homing_Speed, 0x01, homing_vel);
	usleep(SLEEPRW);
	writeObject(ObjectIndexes().Homing_Speed, 0x02, homing_vel);
	usleep(SLEEPRW);

	//Leave at default value
	/*uint32_t homing_acc = ???;
	writeObject(ObjectIndexes().Homing_Acceleration, 0x00, homing_acc);*/

	//Need to set nonzero tolerance band to use end switches for homing, use Block detection instead???
	uint32_t temp_tolerance_band = 200; //TODO: Find good value
	writeObject(ObjectIndexes().Limit_SwitTolerance_Band, 0x0, temp_tolerance_band);
	usleep(SLEEPRW);

	setOperationMode(0x6);
	usleep(SLEEPRW);
	startMode();
	usleep(SLEEPRW);

	controlword[4] = 1;
	controlword[5] = 1;
	controlword[6] = 0;
	controlword[8] = 0;
	controlword[9] = 0;
	setControlword();
}

bool N5Motor::checkHomingFinished()
{
	std::bitset<16> status;
	status = getStatusword();
	usleep(SLEEPRW);
	if (status[13])
		throw std::runtime_error("Error while performing Homing");
	bool finished = status[12] && status[10];
	//if (finished)
	//{
	//	uint32_t zero_tolerance_band = 0;
	//	writeObject(ObjectIndexes().Limit_SwitTolerance_Band, 0x0, zero_tolerance_band);
	//	usleep(SLEEPRW);
	//}
	return finished;
}

void N5Motor::performHoming(int32_t offsetenc)
{
	startHoming(offsetenc);

	while (!checkHomingFinished())
	{
	}
	//setOffset(offsetenc);
}

bool N5Motor::checkFinished()
{
	switch (operationMode)
	{
	case 1:
		return targetReached();
	case 6:
		return checkHomingFinished();
	default:
		return true;
	}
}
