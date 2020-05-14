#include "PositioningUnit.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>

//using namespace N5PosUnit;

PositioningUnit::PositioningUnit() : PositioningUnit("motorqueue.txt")
{
}

PositioningUnit::PositioningUnit(std::string f) : PositioningUnit(3, f)
{
}

PositioningUnit::PositioningUnit(std::string controllerPosX, std::string controllerPosY, std::string controllerRotZ, std::string f) : PositioningUnit(controllerPosX, controllerPosY, controllerRotZ, 3, f)
{
}


PositioningUnit::PositioningUnit(int syncTime, std::string f) : PositioningUnit("192.168.137.80", "192.168.137.81", "192.168.137.82", syncTime, f)
{
}

PositioningUnit::PositioningUnit(std::string controllerPosX, std::string controllerPosY, std::string controllerRotZ, int syncTime, std::string f) : motorPosX(controllerPosX), motorPosY(controllerPosY), motorRotZ(controllerRotZ)
{
	msSyncTime = syncTime;
	writeVecPos = 0;
	allMotors.reserve(3);
	allMotors[0] = &motorPosX;
	allMotors[1] = &motorPosY;
	allMotors[2] = &motorRotZ;
	repeat_once = false;
	readQueueFile(f);
}

PositioningUnit::~PositioningUnit()
{
	motorPosX.shutdown();
	motorPosY.shutdown();
	motorRotZ.shutdown();
}

void PositioningUnit::startHoming(int32_t offsetX, int32_t offsetY, int32_t offsetRotZ)
{
	motorPosX.startHoming(offsetX);
	motorPosY.startHoming(offsetY);
	motorRotZ.startHoming(offsetRotZ);
}

bool PositioningUnit::checkHomingFinished()
{
	return motorPosX.checkHomingFinished() && motorPosX.checkHomingFinished() && motorPosX.checkHomingFinished();
}

void PositioningUnit::setOffsets(int32_t offsetx, int32_t offsety, int32_t offsetrotz)
{
	int xmoveto = 0;
	if (offsetx < 10)
		xmoveto = -offsetx;
	std::vector<int> newpos = { offsetx, 0, 0 };
	motorPosX.setOffset(offsetx);
	motorPosY.setOffset(offsety);
	motorRotZ.setOffset(offsetrotz);
	newpos[0] = xmoveto;
	startMoveToPosition(newpos);
}

//void PositioningUnit::performHoming(int32_t offsetx, int32_t offsety, int32_t offsetrotz)
//{
//	motorPosX.performHoming(offsetx);
//	motorPosY.performHoming(offsety);
//	motorRotZ.performHoming(offsetrotz);
//	int xmoveto = 0;
//	if (offsetx < 10)
//		xmoveto = -offsetx;
//	std::vector<int> newpos = { offsetx, 0, 0 };
//	moveToPosition(newpos);
//}

void PositioningUnit::readQueueFile(std::string &f)
{
	std::ifstream input(f);
	std::string lineData;

	while (getline(input, lineData))
	{
		uint64_t time;
		posXdof rowPos;
		int posX;
		int posY;
		int posZ;
		int rotZ;

		std::vector<double> row;
		std::stringstream lineStream(lineData);

		lineStream >> time;
		//rowPos.msTime = time;
		lineStream >> posX;
		rowPos.posX = posX;
		lineStream >> posY;
		rowPos.posY = posY;
		lineStream >> posZ;
		//rowPos.posZ = posZ;
		lineStream >> rotZ;
		rowPos.rotZ = rotZ;

		inputPos.push(rowPos);
	}
}

void PositioningUnit::writePosFile(std::string &f)
{
	std::ofstream outstream(f,std::ios_base::app);
	if (writeVecPos == 0)
		outstream = std::ofstream(f);
	std::string lineData;

	for (auto pos = actualPos.begin()+writeVecPos; pos != actualPos.end(); ++pos)
	{
		outstream << " " << pos->posX << " " << pos->posY << " " << "0" << " " << pos->rotZ << std::endl;
	}
	writeVecPos = actualPos.size();
}

void PositioningUnit::setInputPos(std::queue<struct posXdof> input)
{
	inputPos = input;
}

void PositioningUnit::setupSynchronousPosition() {
	motorPosX.setupSynchronousPositionMode(msSyncTime);
	motorPosY.setupSynchronousPositionMode(msSyncTime);
	motorRotZ.setupSynchronousPositionMode(msSyncTime);
}

void PositioningUnit::setupSynchronousVelocity() {
	motorPosX.setupSynchronousVelocityMode(msSyncTime);
	motorPosY.setupSynchronousVelocityMode(msSyncTime);
	motorRotZ.setupSynchronousVelocityMode(msSyncTime);
}

void PositioningUnit::startMoveToPosition(std::vector<int> &vecpos) {
	struct posXdof pos;
	switch (vecpos.size()) {
	case 4:
		pos.posX = vecpos[0];
		pos.posY = vecpos[1];
		pos.rotZ = vecpos[3];
		break;
	case 3:
		pos.posX = vecpos[0];
		pos.posY = vecpos[1];
		pos.rotZ = vecpos[2];
		break;
	default:
		throw std::runtime_error("Wrong position format in moveToPosition");
	};
	startMoveToPosition(pos);
}

void PositioningUnit::startMoveToPosition(struct posXdof &pos) {
	motorPosX.setupProfilePositionMode();
	motorPosY.setupProfilePositionMode();
	motorRotZ.setupProfilePositionMode();

	setWritePos(pos);

	motorPosX.startMoveToPosition();
	motorPosY.startMoveToPosition();
	motorRotZ.startMoveToPosition();
}

void PositioningUnit::moveToPosition(struct posXdof &pos) {
	startMoveToPosition(pos);

	while (!targetReached())
	{
	}
}

void PositioningUnit::repeatSyncPos() {
	repeat_once = true;
}

bool PositioningUnit::targetReached()
{
	return motorPosX.targetReached() && motorPosY.targetReached() && motorRotZ.targetReached();
}

void PositioningUnit::startMoveToStart() {
	startMoveToPosition(inputPos.front());
}

void PositioningUnit::moveToStart() {
	moveToPosition(inputPos.front());
}

void PositioningUnit::readActualPosition()
{
	struct posXdof actPos;
	actPos.posX = motorPosX.getActualPosition();
	actPos.posY = motorPosY.getActualPosition();
	actPos.rotZ = motorRotZ.getActualPosition();
	actualPos.push_back(actPos);
}

std::vector<int32_t> PositioningUnit::getCurrPos()
{
	readActualPosition();
	struct posXdof actPos = actualPos.back();
	std::vector<int32_t> pos(3);
	pos[0] = actPos.posX;
	pos[1] = actPos.posY;
	pos[2] = actPos.rotZ;
	return pos;
}

void PositioningUnit::flushBuffer()
{
	inputPos = {};
}

posXdof PositioningUnit::nextPos() 
{
	posXdof ret = inputPos.front();
	setPos(inputPos.front());
	//Enables Pause mode 
	if (!repeat_once)
	{
		inputPos.pop();
		repeat_once = false;
	}
	writePos();
	return ret;
}

void PositioningUnit::readStatus() {
	struct readPDO statusMotorPosX = motorPosX.getReadPDO();
	struct readPDO statusMotorPosY = motorPosY.getReadPDO();
	struct readPDO statusMotorRotZ = motorRotZ.getReadPDO();

	struct posXdof actpos;
	actpos.posX = statusMotorPosX.posactualvalue;
	actpos.posY = statusMotorPosY.posactualvalue;
	actpos.rotZ = statusMotorRotZ.posactualvalue;
	actualPos.push_back(actpos);

	struct statuswords3 actstatus;
	actstatus.posX = statusMotorPosX.statusword;
	actstatus.posY = statusMotorPosY.statusword;
	actstatus.rotZ = statusMotorRotZ.statusword;
	statuswords.push_back(actstatus);
}



bool PositioningUnit::checkEnd()
{
	return inputPos.empty();
}

void PositioningUnit::printActPos()
{
	struct posXdof actPos = actualPos.back();
	std::cout << "X:\t" << actPos.posX << "\tmm\n";
	std::cout << "Y:\t" << actPos.posY << "\tmm\n";
	std::cout << "phi:\t" << actPos.rotZ << "\tdeg\n";
}

void PositioningUnit::setPos(struct posXdof &tarpos)
{
	motorPosX.setTargetPosition(tarpos.posX);
	motorPosY.setTargetPosition(tarpos.posY);
	motorRotZ.setTargetPosition(tarpos.rotZ);
}

void PositioningUnit::writePos() {
	motorPosX.writeTargetPosition();
	motorPosY.writeTargetPosition();
	motorRotZ.writeTargetPosition();
}

void PositioningUnit::setWritePos(struct posXdof &tarpos)
{
	setPos(tarpos);
	writePos();
}

void PositioningUnit::setUsePDO()
{
	motorPosX.setUsePDO();
	motorPosY.setUsePDO();
	motorRotZ.setUsePDO();
}

void PositioningUnit::quickStop()
{
	motorPosX.quickStop();
	motorPosY.quickStop();
	motorRotZ.quickStop();
}

int PositioningUnit::getRemainingSteps()
{
	return inputPos.size();
}


bool PositioningUnit::checkFinished()
{
	return motorPosX.checkFinished() && motorPosY.checkFinished() && motorRotZ.checkFinished();
}
//void PositioningUnit::printResultsToFile() {
//
//}
