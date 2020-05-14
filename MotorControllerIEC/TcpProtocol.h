#pragma once

#include "PositioningUnit.h"
#include "TcpSocket.h"
#include <vector>

namespace N5TcpSocket
{
}

class TcpProtocol
{
protected:
	DataSocket& socket;
	bool disconnect;
	const int message_size = 14;
	const int header_size = 4;
	void writeMessage(std::vector<char>& rawmessage);
	std::vector<char> readMessage();
public:
	TcpProtocol(DataSocket& socket);
	~TcpProtocol();

	//void sendMessage(uint8_t * function_code, uint8_t * error_code);
	void sendMessage(uint8_t * function_code, uint8_t * error_code, std::vector<int> pos);
    void sendMessage(uint8_t *function_code, uint8_t *error_code, struct posXdof *pos);
	//void recvMessage(uint8_t * function_code, uint8_t * error_code);
	void recvMessage(uint8_t * function_code, uint8_t * error_code, std::vector<int>& pos);
    void recvMessage(uint8_t *function_code, uint8_t *error_code, struct posXdof *pos);
	bool isDisconnected();
};

//}

