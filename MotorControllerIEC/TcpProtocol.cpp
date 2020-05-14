#include "TcpProtocol.h"
#include "TcpSocket.h"
#include <stdexcept>

//using namespace N5TcpSocket;

TcpProtocol::TcpProtocol(DataSocket& socket)
	: socket(socket)
{
	disconnect = false;
}

TcpProtocol::~TcpProtocol()
{
	socket.putMessageClose();
}

//void TcpProtocol::sendMessage(uint8_t *function_code, uint8_t *error_code)
//{
//	//Ugly hard-coded stuff. Real time capability relies on no dynamic allocations taking place. Find out how to make this flexible with "allocation" at compile time
//	int32_t a = 0;
//	std::vector<char> rawmessage(message_size);
//	memcpy(rawmessage.data(), function_code, 1);
//	memcpy(rawmessage.data() + 1, error_code, 1);
//	memcpy(rawmessage.data() + 2, &a, 4);
//	memcpy(rawmessage.data() + 6, &a, 4);
//	memcpy(rawmessage.data() + 10, &a, 4);
//
//	writeMessage(rawmessage);
//}

void TcpProtocol::sendMessage(uint8_t *function_code, uint8_t *error_code, std::vector<int> pos)
{
	//Ugly hard-coded stuff. Real time capability relies on no dynamic allocations taking place. Find out how to make this flexible with "allocation" at compile time
	std::vector<char> rawmessage(message_size);
	memcpy(rawmessage.data(), function_code, 1);
	memcpy(rawmessage.data() + 1, error_code, 1);
	memcpy(rawmessage.data() + 2, &(pos[0]), 4);
	memcpy(rawmessage.data() + 6, &pos[1], 4);
	memcpy(rawmessage.data() + 10, &pos[2], 4);

	writeMessage(rawmessage);
}

void TcpProtocol::sendMessage(uint8_t *function_code, uint8_t *error_code, struct posXdof *pos)
{
	//Ugly hard-coded stuff. Real time capability relies on no dynamic allocations taking place. Find out how to make this flexible with "allocation" at compile time
	std::vector<char> rawmessage(message_size);
	memcpy(rawmessage.data(), function_code, 1);
	memcpy(rawmessage.data()+1, error_code, 1);
	memcpy(rawmessage.data()+2, &pos->posX, 4);
	memcpy(rawmessage.data()+6, &pos->posY, 4);
	memcpy(rawmessage.data()+10, &pos->rotZ, 4);

    writeMessage(rawmessage);
}

//void TcpProtocol::recvMessage(uint8_t *function_code, uint8_t *error_code)
//{
//	std::vector<char> rawmessage = readMessage();
//	memcpy(function_code, rawmessage.data(), 1);
//	memcpy(error_code, rawmessage.data() + 1, 1);
//}

void TcpProtocol::recvMessage(uint8_t *function_code, uint8_t *error_code, std::vector<int> &pos)
{
	std::vector<char> rawmessage = readMessage();
	memcpy(function_code, rawmessage.data(), 1);
	memcpy(error_code, rawmessage.data() + 1, 1);
	memcpy(&pos[0], rawmessage.data() + 2, 4);
	memcpy(&pos[1], rawmessage.data() + 6, 4);
	memcpy(&pos[2], rawmessage.data() + 10, 4);
}

void TcpProtocol::recvMessage(uint8_t *function_code, uint8_t *error_code, struct posXdof *pos)
{
	std::vector<char> rawmessage = readMessage();
	memcpy(function_code, rawmessage.data(), 1);
	memcpy(error_code, rawmessage.data() + 1, 1);
	memcpy(&pos->posX, rawmessage.data() + 2, 4);
	memcpy(&pos->posY, rawmessage.data() + 6, 4);
	memcpy(&pos->rotZ, rawmessage.data() + 10, 4);
}

bool TcpProtocol::isDisconnected()
{
	return disconnect;
}

void TcpProtocol::writeMessage(std::vector<char> &rawmessage)
{
	if (disconnect)
		return;
	std::vector<char> fullmessage = {0,0,0,(char)message_size};
	fullmessage.insert(fullmessage.end(), rawmessage.begin(), rawmessage.end());
	socket.putMessageData(fullmessage.data(), fullmessage.size());
}

std::vector<char> TcpProtocol::readMessage()
{
	std::vector<char> fullmessage(message_size + header_size);
	socket.getMessageData(fullmessage.data(), fullmessage.size(), [](std::size_t) {return false; });

	//Connection close message
	if (fullmessage[3] == 0)
	{
		disconnect = true;
		std::vector<char> nullvect = { 0x01,0,0,0,0,0,0,0,0,0,0,0,0,0 };
		return nullvect;
	}

	if (fullmessage[3] != message_size)
		throw std::runtime_error("Error in receiving message. Wrong message size! Required message size is 14 while message received has size" + std::to_string((int) fullmessage[3]));

	return std::vector<char>(fullmessage.begin()+4, fullmessage.end());
}