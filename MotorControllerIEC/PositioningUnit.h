#pragma once
#include "n5_modbus.h"
#include <queue>
#include <string>


namespace N5PosUnit
{
}

struct posXdof {
	int32_t posX;
	int32_t posY;
	int32_t rotZ;
};

struct statuswords3 {
	std::bitset<16> posX;
	std::bitset<16> posY;
	std::bitset<16> rotZ;
};


class PositioningUnit
{
private:
	int8_t msSyncTime;
	bool enableMove;
	bool repeat_once;

	std::vector<N5Motor *> allMotors;

	N5Motor motorPosX;
	N5Motor motorPosY;
	N5Motor motorRotZ;

	struct readPDO statusPDOPosX;
	struct readPDO statusPDOPosY;
	struct readPDO statusPDORotZ;

	std::queue<struct posXdof> inputPos;
	std::vector<struct posXdof> actualPos;
	std::vector<struct statuswords3> statuswords;

	int writeVecPos;

public:
	PositioningUnit();
	PositioningUnit(std::string f);
	PositioningUnit(std::string controllerPosX, std::string controllerPosY, std::string controllerRotZ, int syncTime, std::string f);
	PositioningUnit(std::string controllerPosX, std::string controllerPosY, std::string controllerRotZ, std::string f);
	PositioningUnit(int syncTime, std::string f);
	//PositioningUnit(std::string controllerPosX, std::string controllerPosY, std::string controllerRotZ, int syncTime, std::string f);
	~PositioningUnit();

	void startHoming(int32_t offsetX, int32_t offsetY, int32_t offsetRotZ);

	bool checkHomingFinished();

	void setOffsets(int32_t offsetx, int32_t offsety, int32_t offsetrotz);

	//void performHoming(int32_t offsetx, int32_t offsety, int32_t offsetrotz);

	void readQueueFile(std::string & f);

	void writePosFile(std::string & f);

	//void writePosFile();

	void setInputPos(std::queue<struct posXdof> input);
	//std::queue<struct posXdof> getInputPositions();

	void setupSynchronousPosition();

	void setupSynchronousVelocity();

	void startMoveToPosition(std::vector<int> &vecpos);
	void startMoveToPosition(posXdof &pos);

	void startMoveToStart();

	void moveToStart();
	void moveToPosition(struct posXdof &pos);

	void repeatSyncPos();

	bool targetReached();

	void readActualPosition();

	std::vector<int32_t> getCurrPos();

	void flushBuffer();

	posXdof nextPos();

	void readStatus();

	bool checkEnd();

	void printActPos();

	void setPos(struct posXdof &actpos);
	void writePos();
	void setWritePos(struct posXdof &actpos);
	void setUsePDO();
	void quickStop();
	int getRemainingSteps();
	bool checkFinished();
};

#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

class CSVRow
{
public:
	std::string const& operator[](std::size_t index) const
	{
		return m_data[index];
	}
	std::size_t size() const
	{
		return m_data.size();
	}
	void readNextRow(std::istream& str)
	{
		std::string         line;
		std::getline(str, line);

		std::stringstream   lineStream(line);
		std::string         cell;

		m_data.clear();
		while (std::getline(lineStream, cell, ','))
		{
			m_data.push_back(cell);
		}
		// This checks for a trailing comma with no data after it.
		if (!lineStream && cell.empty())
		{
			// If there was a trailing comma then add an empty element.
			m_data.push_back("");
		}
	}
private:
	std::vector<std::string>    m_data;
};

inline std::istream& operator>>(std::istream& str, CSVRow& data)
{
	data.readNextRow(str);
	return str;
}

//}

//int main()
//{
//	std::ifstream       file("plop.csv");
//
//	CSVRow              row;
//	while (file >> row)
//	{
//		std::cout << "4th Element(" << row[3] << ")\n";
//	}
//}