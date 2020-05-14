#include "n5_modbus.h"
#include "PositioningUnit.h"
#include "TcpProtocol.h"
#include "TcpSocket.h"
#include <sched.h>
#include <stdio.h>
#include <unistd.h>
#include <limits.h>
#include <pthread.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <modbus/modbus.h>
#include <stdio.h>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <iostream>
#include <vector>
#include <string>

const bool debug = false;

//Basic Socket Protocol
//0x00 Emergency Shutdown
//0x01 Operation Enabled / Quick Stop
//0x02 Run
//0x03 Request Position
//0x04 Quick Move
//0x05 Homing

//using namespace N5TcpSocket;
//using namespace N5PosUnit;

std::string IP_X = "192.168.12.101";
std::string IP_Y = "192.168.12.102";
std::string IP_ROTZ = "192.168.12.103";
std::string queuefile = "/home/pi/motorcommand/motorqueue.txt";
std::string outfile = "/home/pi/motorcommand/outpos.txt";
std::string configfile = "/home/pi/motorcommand/config.txt";
int CycleTimeMs = 5;
int Port = 8081;
std::vector<int> offsets = { 10000, 10000, 2000 };

//Status
//0 Shutdown/Abort
//1 Pause
//2 Run
int status;
//Modes 
//0x01 Run File
//0x02 Quick Move
//0x03 Homing
int mode;

struct posXdof quick_move_pos;
std::vector<int32_t> curr_pos;

pthread_mutex_t status_mutex;
pthread_mutex_t mode_mutex;

struct period_info {
	struct timespec next_period;
	long period_ns;
};

static void inc_period(struct period_info *pinfo)
{
	pinfo->next_period.tv_nsec += pinfo->period_ns;

	while (pinfo->next_period.tv_nsec >= 1000000000) {
		/* timespec nsec overflow */
		pinfo->next_period.tv_sec++;
		pinfo->next_period.tv_nsec -= 1000000000;
	}
}

void *tcp_server_func(void *)
{
	bool exit = false;
	ServerSocket server(Port);
	while (!exit)
	{
		DataSocket  accept = server.accept();
		TcpProtocol protocol(accept);
		uint8_t function_code = 0;
		uint8_t error_code = 0;
		struct posXdof pos;
		std::vector<int> one_pos = { 1,2,3 };
		pos.posX = 0;
		pos.posY = 0;
		pos.rotZ = 0;
		while (!exit && !protocol.isDisconnected())
		{
			protocol.recvMessage(&function_code, &error_code, &pos);
			switch (error_code)
			{
			case 0:
				break;
			default:
				function_code = 0;
				std::cout << "Labview reported an error to the raspberry. Reason unknown. Shutting down" << std::endl;
			};
			if (debug)
				std::cout << "Starting function code " << (int32_t)function_code << std::endl;
			std::vector<int32_t> loc_pos(3);
			switch (function_code)
			{
			case 0: //Emergency Shutdown
				pthread_mutex_lock(&status_mutex);
				status = 0;
				loc_pos = curr_pos;
				pthread_mutex_unlock(&status_mutex);
				exit = true;
				protocol.sendMessage(&function_code, &error_code, loc_pos);
				break;
			case 1: //Quick Stop
				pthread_mutex_lock(&status_mutex);
				status = 1;
				loc_pos = curr_pos;
				pthread_mutex_unlock(&status_mutex);
				protocol.sendMessage(&function_code, &error_code, loc_pos);
				break;
			case 2: //Run Sync
				pthread_mutex_lock(&mode_mutex);
				mode = 1;
				pthread_mutex_unlock(&mode_mutex);
				pthread_mutex_lock(&status_mutex);
				status = 2;
				loc_pos = curr_pos;
				pthread_mutex_unlock(&status_mutex);
				protocol.sendMessage(&function_code, &error_code, loc_pos);
				break;
			case 3: //Request Pos
				pthread_mutex_lock(&status_mutex);
				loc_pos = curr_pos;
				pthread_mutex_unlock(&status_mutex);
				protocol.sendMessage(&function_code, &error_code, loc_pos);
				break;
			case 4: // Quick Move
				quick_move_pos = pos;
				pthread_mutex_lock(&mode_mutex);
				mode = 2;
				pthread_mutex_unlock(&mode_mutex);
				pthread_mutex_lock(&status_mutex);
				status = 2;
				loc_pos = curr_pos;
				pthread_mutex_unlock(&status_mutex);
				protocol.sendMessage(&function_code, &error_code, loc_pos);
				break;
			case 5: //Homing
				pthread_mutex_lock(&mode_mutex);
				mode = 3;
				pthread_mutex_unlock(&mode_mutex);
				pthread_mutex_lock(&status_mutex);
				status = 2;
				loc_pos = curr_pos;
				pthread_mutex_unlock(&status_mutex);
				protocol.sendMessage(&function_code, &error_code, loc_pos);
				break;
			case 99: //Test
				std::cout << "Communication test successful!" << std::endl;
				protocol.sendMessage(&function_code, &error_code, one_pos);
				break;
			default:
				error_code = 1;
				loc_pos = curr_pos;
				protocol.sendMessage(&function_code, &error_code, loc_pos);
			};
		}
		pthread_mutex_lock(&mode_mutex);
		mode = 2;
		pthread_mutex_unlock(&mode_mutex);
		pthread_mutex_lock(&status_mutex);
		status = 1;
		pthread_mutex_unlock(&status_mutex);
	}
	return NULL;
}

void wait_for_finish(PositioningUnit &posUnit)
{
	pthread_mutex_lock(&status_mutex);
	int localstatus = status;
	pthread_mutex_unlock(&status_mutex);

	while ((localstatus == 2) && (!posUnit.checkFinished()))
	{
		pthread_mutex_lock(&status_mutex);
		curr_pos = posUnit.getCurrPos();
		localstatus = status;
		pthread_mutex_unlock(&status_mutex);
		if (debug)
			std::cout << "Waiting for Finish. Status: " << localstatus << " Mode: " << mode << std::endl;
	}
	if (localstatus < 2)
		posUnit.quickStop();
}

void run_sync_mode(PositioningUnit &posUnit)
{
	int retnanosleep = 0;
	int write = 0;

	std::vector<int> sleeptimes;
	sleeptimes.reserve(posUnit.getRemainingSteps());

	struct period_info pinfo;
	pinfo.period_ns = CycleTimeMs * 1000000;
	clock_gettime(CLOCK_MONOTONIC, &(pinfo.next_period));
	long start = pinfo.next_period.tv_sec * 1000 + std::lround(pinfo.next_period.tv_nsec / 1.0e3);

	pthread_mutex_lock(&status_mutex);
	int localstatus = status;
	pthread_mutex_unlock(&status_mutex);
	pthread_mutex_lock(&mode_mutex);
	int localmode = mode;
	pthread_mutex_unlock(&mode_mutex);
	
	while (!posUnit.checkEnd()) {
		posUnit.readStatus();
		posXdof pos = posUnit.nextPos();
		pthread_mutex_lock(&status_mutex);
		curr_pos = { pos.posX, pos.posY, pos.rotZ };
		localstatus = status;
		pthread_mutex_unlock(&status_mutex);
		pthread_mutex_lock(&mode_mutex);
		localmode = mode;
		pthread_mutex_unlock(&mode_mutex);

		if (write++ > 10)
		{
			posUnit.writePosFile(outfile);
			write = 0;
		}

		if (localmode != 0x01 || localstatus == 0)
			posUnit.flushBuffer();
		else if(localstatus == 1)
			posUnit.repeatSyncPos();

		if (debug && localstatus < 2)
			std::cout << "Status in Sync Pos < 2. Status=" << localstatus << std::endl;
		
		

		long t = pinfo.next_period.tv_sec * 1000 + std::lround(pinfo.next_period.tv_nsec / 1.0e3) - start;

		inc_period(&pinfo);
		/* for simplicity, ignoring possibilities of signal wakes */
		retnanosleep = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo.next_period, NULL);

		if (debug)
		{
			sleeptimes.push_back(retnanosleep);
		}
	}
	pthread_mutex_lock(&status_mutex);
	status = 1;
	pthread_mutex_unlock(&status_mutex);
	if (debug)
	{
		std::cout << "Run complete" << std::endl;
		int st0 = std::count_if(sleeptimes.begin(), sleeptimes.end(), [](int i) {return i == EINVAL; });
		std::cout << "Number of sleeptimes < 0: " << st0 << std::endl;
	}
}

void *n5_motors_func(void *)
{
	PositioningUnit posUnit(IP_X, IP_Y, IP_ROTZ, CycleTimeMs, queuefile);
	posUnit.startHoming(offsets[0], offsets[1], offsets[2]);
	wait_for_finish(posUnit);
	//posUnit.setOffsets(offsets[0], offsets[1], offsets[2]);
	std::vector<int> loc_pos = { 0,0,0 };
	posUnit.startMoveToPosition(loc_pos);
	wait_for_finish(posUnit);

	pthread_mutex_lock(&status_mutex);
	status = 1;
	int localstatus = status;
	pthread_mutex_unlock(&status_mutex);
	while (localstatus > 0)
	{
		pthread_mutex_lock(&mode_mutex);
		int localmode = mode;
		pthread_mutex_unlock(&mode_mutex);
		if (localstatus > 1)
		{
			if (debug)
				std::cout << "Mode: " << localmode << std::endl;
			switch (localmode) {
			case 1:
				posUnit.readQueueFile(queuefile);
				posUnit.startMoveToStart();
				wait_for_finish(posUnit);
				posUnit.setupSynchronousPosition();
				run_sync_mode(posUnit);
				localstatus = 1;
				break;
			case 2:
				posUnit.startMoveToPosition(quick_move_pos);
				wait_for_finish(posUnit);
				break;
			case 3:
				posUnit.startHoming(offsets[0], offsets[1], offsets[2]);
				wait_for_finish(posUnit);
				status = 1;
				//posUnit.setOffsets(offsets[0], offsets[1], offsets[2]);
				break;
			}
		}

		loc_pos = posUnit.getCurrPos();
		pthread_mutex_lock(&status_mutex);
		localstatus = status;
		curr_pos = loc_pos;
		pthread_mutex_unlock(&status_mutex);

		if (debug)
			std::cout << "Waiting for command. Status: " << localstatus << std::endl;
	}


	////Print positions at times
	//for (int i = 0; i < sleeptimes.size(); i++)
	//{
	//	std::cout << utime[i] / 1000 << ":\t" << actpos[i] << "\t" << tarpos[i] << "\t" << sleeptimes[i] << "\t" << statuswords[i].to_string() << "\t" << follerr[i] << std::endl;
	//}
	//for (int i = 0; i < 100; i++)
	//{
	//	std::cout << utime[i] / 1000 << ":\t" << actpos[i] << "\t" << tarpos[i] << "\t" << sleeptimes[i] << "\t" << statuswords[i].to_string() << "\t" << follerr[i] << std::endl;
	//}
	//
	return NULL;
}

std::string remove_windows_line_ending(std::string s)
{
	if (!s.empty() && s[s.size() - 1] == '\r')
		s.erase(s.size() - 1);
	return s;
}

void config_reader() {
	std::ifstream input(configfile, std::ifstream::binary);
	std::string lineData;

	while (getline(input, lineData))
	{
		std::istringstream istr_line(lineData);
		std::string key;
		if (std::getline(istr_line, key, '='))
		{
			std::string value;
			if (std::getline(istr_line, value))
			{
				if (key == "IP_X")
					IP_X = remove_windows_line_ending(value);
				else if (key == "IP_Y")
					IP_Y = remove_windows_line_ending(value);
				else if (key == "IP_ROTZ")
					IP_ROTZ = remove_windows_line_ending(value);
				else if (key == "INPUT_FILE")
					queuefile = remove_windows_line_ending(value);
				else if (key == "OUTPUT_FILE")
					outfile = remove_windows_line_ending(value);
				else if (key == "CYCLETIME")
					CycleTimeMs = std::stoi(value);
				else if (key == "CYCLEFREQ")
					CycleTimeMs = 1000 / std::stoi(value);
				else if (key == "OFFSET_X")
					offsets[0] = std::stoi(value);
				else if (key == "OFFSET_Y")
					offsets[1] = std::stoi(value);
				else if (key == "OFFSET_ROTZ")
					offsets[2] = std::stoi(value);
				else if (key == "PORT")
					Port = std::stoi(value);
				else
					std::cerr << "Unknown key " << key << " in config.txt ignored." << std::endl;
			}
		}
	}
	if (debug)
		std::cout << "IP_X: " << IP_X << "\nIP_Y: " << IP_Y << "\nIP_ROTZ: " << IP_ROTZ << "\nINPUT_FILE: " << queuefile << "\nOUTPUT_FILE: " << outfile << "\nCYCLETIME: " << CycleTimeMs << "\nCYCLEFREQ: " << 1000 / CycleTimeMs << "\nOFFSETX: " << offsets[0] << "\nOFFSETY: " << offsets[1] << "\nOFFSETROTZ: " << offsets[2] << "\nPORT: " << Port << std::endl;
}

int main(int argc, char* argv[])
{
	curr_pos = { 0, 0, 0 };
	status = 2;
	mode = 3;
	quick_move_pos = {0, 0, 0 };
	config_reader();

	struct sched_param param_n5_motors, param_tcp_server;
	pthread_attr_t attr_n5_motors, attr_tcp_server;
	pthread_t thread_n5_motors, thread_tcp_server;
	int ret;

	/* Lock memory */
	if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
		printf("mlockall failed: %m\n");
		exit(-2);
	}

	/* Initialize pthread attributes (default values) */
	ret = pthread_attr_init(&attr_n5_motors);
	ret = pthread_attr_init(&attr_tcp_server);
	if (ret) {
		printf("init pthread attributes failed\n");
		goto out;
	}

	/* Set a specific stack size  */
	ret = pthread_attr_setstacksize(&attr_n5_motors, PTHREAD_STACK_MIN);
	ret = pthread_attr_setstacksize(&attr_tcp_server, PTHREAD_STACK_MIN);
	if (ret) {
		printf("pthread setstacksize failed\n");
		goto out;
	}

	/* Set scheduler policy and priority of pthread */
	param_n5_motors.sched_priority = 80;
	param_tcp_server.sched_priority = 80;
	ret = pthread_attr_setschedpolicy(&attr_n5_motors, SCHED_FIFO);
	ret = pthread_attr_setschedpolicy(&attr_tcp_server, SCHED_FIFO);
	if (ret) {
		printf("pthread setschedpolicy failed\n");
		goto out;
	}
	ret = pthread_attr_setschedparam(&attr_n5_motors, &param_n5_motors);
	ret = pthread_attr_setschedparam(&attr_tcp_server, &param_tcp_server);
	if (ret) {
		printf("pthread setschedparam failed\n");
		goto out;
	}
	/* Use scheduling parameters of attr */
	ret = pthread_attr_setinheritsched(&attr_n5_motors, PTHREAD_EXPLICIT_SCHED);
	ret = pthread_attr_setinheritsched(&attr_tcp_server, PTHREAD_EXPLICIT_SCHED);
	if (ret) {
		printf("pthread setinheritsched failed\n");
		goto out;
	}

	/* Create pthreads with specified attributes */
	ret = pthread_create(&thread_n5_motors, &attr_n5_motors, n5_motors_func, NULL);
	ret = pthread_create(&thread_tcp_server, &attr_tcp_server, tcp_server_func, NULL);
	if (ret) {
		printf("create pthread failed\n");
		goto out;
	}

	/* Join the thread and wait until it is done */
	ret = pthread_join(thread_n5_motors, NULL);
	if (ret)
		printf("join pthread failed: %m\n");

out:
	return ret;
}