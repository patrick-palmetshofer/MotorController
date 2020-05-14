#ifndef N5_MODBUS
#define N5_MODBUS

#include <modbus/modbus.h>
#include <string>
#include <algorithm>
#include <string>
#include <cstring>
#include <bitset>

namespace N5PosUnit
{
}

struct readPDO {
	std::bitset<16> statusword;
	int32_t posactualvalue;
	int32_t velocityactualvalue;
	int32_t endswitchstatus;
};


class N5Motor {
private:
	typedef struct ObjectIndexes {
		static const uint16_t Device_Type = 0x1000;
		static const uint16_t Error_Register = 0x1001;
		static const uint16_t Pre_defined_Error_Field = 0x1003;
		static const uint16_t COB_ID_Sync = 0x1005;
		static const uint16_t Synchronous_Window_Length = 0x1007;
		static const uint16_t Manufacturer_Device_Name = 0x1008;
		static const uint16_t Manufacturer_Hardware_Version = 0x1009;
		static const uint16_t Manufacturer_Software_Version = 0x100A;
		static const uint16_t Guard_Time = 0x100C;
		static const uint16_t Live_Time_Factor = 0x100D;
		static const uint16_t Store_Parameters = 0x1010;
		static const uint16_t Restore_Default_Parameters = 0x1011;
		static const uint16_t COB_ID_EMCY = 0x1014;
		static const uint16_t Producer_Heartbeat_Time = 0x1017;
		static const uint16_t Identity_Object = 0x1018;
		static const uint16_t Verify_Configuration = 0x1020;
		static const uint16_t Error_Behavior = 0x1029;
		//static const uint16_t Receive_PDO_1_Communication_Parameter = 0x1400;
		//static const uint16_t Receive_PDO_2_Communication_Parameter = 0x1401;
		//static const uint16_t Receive_PDO_3_Communication_Parameter = 0x1402;
		//static const uint16_t Receive_PDO_4_Communication_Parameter = 0x1403;
		//static const uint16_t Receive_PDO_5_Communication_Parameter = 0x1404;
		//static const uint16_t Receive_PDO_6_Communication_Parameter = 0x1405;
		//static const uint16_t Receive_PDO_7_Communication_Parameter = 0x1406;
		//static const uint16_t Receive_PDO_8_Communication_Parameter = 0x1407;
		//static const uint16_t Receive_PDO_1_Mapping_Parameter = 0x1600;
		//static const uint16_t Receive_PDO_2_Mapping_Parameter = 0x1601;
		//static const uint16_t Receive_PDO_3_Mapping_Parameter = 0x1602;
		//static const uint16_t Receive_PDO_4_Mapping_Parameter = 0x1603;
		//static const uint16_t Receive_PDO_5_Mapping_Parameter = 0x1604;
		//static const uint16_t Receive_PDO_6_Mapping_Parameter = 0x1605;
		//static const uint16_t Receive_PDO_7_Mapping_Parameter = 0x1606;
		//static const uint16_t Receive_PDO_8_Mapping_Parameter = 0x1607;
		//static const uint16_t Transmit_PDO_1_Communication_Parameter = 0x1800;
		//static const uint16_t Transmit_PDO_2_Communication_Parameter = 0x1801;
		//static const uint16_t Transmit_PDO_3_Communication_Parameter = 0x1802;
		//static const uint16_t Transmit_PDO_4_Communication_Parameter = 0x1803;
		//static const uint16_t Transmit_PDO_5_Communication_Parameter = 0x1804;
		//static const uint16_t Transmit_PDO_6_Communication_Parameter = 0x1805;
		//static const uint16_t Transmit_PDO_7_Communication_Parameter = 0x1806;
		//static const uint16_t Transmit_PDO_8_Communication_Parameter = 0x1807;
		//static const uint16_t Transmit_PDO_1_Mapping_Parameter = 0x1A00;
		//static const uint16_t Transmit_PDO_2_Mapping_Parameter = 0x1A01;
		//static const uint16_t Transmit_PDO_3_Mapping_Parameter = 0x1A02;
		//static const uint16_t Transmit_PDO_4_Mapping_Parameter = 0x1A03;
		//static const uint16_t Transmit_PDO_5_Mapping_Parameter = 0x1A04;
		//static const uint16_t Transmit_PDO_6_Mapping_Parameter = 0x1A05;
		//static const uint16_t Transmit_PDO_7_Mapping_Parameter = 0x1A06;
		//static const uint16_t Transmit_PDO_8_Mapping_Parameter = 0x1A07;
		static const uint16_t Program_Data = 0x1F50;
		static const uint16_t Program_Control = 0x1F51;
		static const uint16_t Program_Status = 0x1F57;
		static const uint16_t CANopen_Baudrate = 0x2005;
		static const uint16_t CANopen_Config = 0x2007;
		static const uint16_t CANopen_NodeID = 0x2009;
		static const uint16_t IP_Configuration = 0x2010;
		static const uint16_t Static_IPv4_Address = 0x2011;
		static const uint16_t Static_IPv4_Subnet_Mask = 0x2012;
		static const uint16_t Current_IPv4_Address = 0x2014;
		static const uint16_t Current_IPv4_Subnet_Mask = 0x2015;
		static const uint16_t Pole_Pair_Count = 0x2030;
		static const uint16_t Maximum_Current = 0x2031;
		static const uint16_t Maximum_Speed = 0x2032;
		static const uint16_t Plunger_Block = 0x2033;
		static const uint16_t Upper_Voltage_Warning_Level = 0x2034;
		static const uint16_t Lower_Voltage_Warning_Level = 0x2035;
		static const uint16_t Open_Loop_Current_Reduction_Idle_Time = 0x2036;
		static const uint16_t Open_Loop_Current_Reduction_Value_factor = 0x2037;
		static const uint16_t Brake_Controller_Timing = 0x2038;
		static const uint16_t Motor_Currents = 0x2039;
		static const uint16_t Homing_On_Block_Configuration = 0x203A;
		static const uint16_t I2t_Parameters = 0x203B;
		static const uint16_t Torque_Window = 0x203D;
		static const uint16_t Torque_Window_Time = 0x203E;
		static const uint16_t Encoder_Alignment = 0x2050;
		static const uint16_t Encoder_Optimization = 0x2051;
		static const uint16_t Encoder_Resolution = 0x2052;
		static const uint16_t Limit_SwitTolerance_Band = 0x2056;
		static const uint16_t Clock_Direction_Multiplier = 0x2057;
		static const uint16_t Clock_Direction_Divider = 0x2058;
		static const uint16_t Encoder_Configuration = 0x2059;
		static const uint16_t Encoder_Boot_Value = 0x205A;
		static const uint16_t Clock_Direction_Or_Clockwise_Counter_Clockwise_Mode = 0x205B;
		static const uint16_t Compensate_Polepair_Count = 0x2060;
		static const uint16_t Velocity_Numerator = 0x2061;
		static const uint16_t Velocity_Denominator = 0x2062;
		static const uint16_t Acceleration_Numerator = 0x2063;
		static const uint16_t Acceleration_Denominator = 0x2064;
		static const uint16_t Jerk_Numerator = 0x2065;
		static const uint16_t Jerk_Denominator = 0x2066;
		static const uint16_t Bootup_Delay = 0x2084;
		static const uint16_t Fieldbus_Module_Availability = 0x2101;
		static const uint16_t Fieldbus_Module_Control = 0x2102;
		static const uint16_t Fieldbus_Module_Status = 0x2103;
		static const uint16_t NanoJ_Control = 0x2300;
		static const uint16_t NanoJ_Status = 0x2301;
		static const uint16_t NanoJ_Error_Code = 0x2302;
		static const uint16_t Number_Of_Active_User_Program = 0x2303;
		static const uint16_t Table_Of_Available_User_Programs = 0x2304;
		static const uint16_t Uptime_Seconds = 0x230F;
		static const uint16_t NanoJ_Input_Data_Selection = 0x2310;
		static const uint16_t NanoJ_Output_Data_Selection = 0x2320;
		static const uint16_t NanoJ_In_Output_Data_Selection = 0x2330;
		static const uint16_t NanoJ_Inputs = 0x2400;
		static const uint16_t NanoJ_Init_Parameters = 0x2410;
		static const uint16_t NanoJ_Outputs = 0x2500;
		static const uint16_t NanoJ_Debug_Output = 0x2600;
		static const uint16_t Customer_Storage_Area = 0x2701;
		static const uint16_t Bootloader_And_Reboot_Settings = 0x2800;
		static const uint16_t Motor_Drive_Submode_Select = 0x3202;
		static const uint16_t Motor_Drive_Sensor_Display_Open_Loop = 0x320A;
		static const uint16_t Motor_Drive_Sensor_Display_Closed_Loop = 0x320B;
		static const uint16_t Motor_Drive_Parameter_Set = 0x3210;
		static const uint16_t Motor_Drive_Flags = 0x3212;
		static const uint16_t Analog_Inputs = 0x3220;
		static const uint16_t Analogue_Inputs_Control = 0x3221;
		static const uint16_t Analogue_Inputs_Switches = 0x3225;
		static const uint16_t Digital_Inputs_Control = 0x3240;
		static const uint16_t Digital_Input_Routing = 0x3242;
		static const uint16_t Digital_Outputs_Control = 0x3250;
		static const uint16_t Digital_Output_Routing = 0x3252;
		static const uint16_t Read_Analogue_Input = 0x3320;
		static const uint16_t Analogue_Input_Offset = 0x3321;
		static const uint16_t Analogue_Input_Pre_scaling = 0x3322;
		static const uint16_t Modbus_Rx_PDO_Mapping = 0x3502;
		static const uint16_t Modbus_Tx_PDO_Mapping = 0x3602;
		static const uint16_t Following_Error_Option_Code = 0x3700;
		static const uint16_t HW_Information = 0x4012;
		static const uint16_t HW_Configuration = 0x4013;
		static const uint16_t Operating_Conditions = 0x4014;
		static const uint16_t Drive_Serial_Number = 0x4040;
		static const uint16_t Device_Id = 0x4041;
		static const uint16_t Error_Code = 0x603F;
		static const uint16_t Controlword = 0x6040;
		static const uint16_t Statusword = 0x6041;
		static const uint16_t Vl_Target_Velocity = 0x6042;
		static const uint16_t Vl_Velocity_Demand = 0x6043;
		static const uint16_t Vl_Velocity_Actual_Value = 0x6044;
		static const uint16_t Vl_Velocity_Min_Max_Amount = 0x6046;
		static const uint16_t Vl_Velocity_Acceleration = 0x6048;
		static const uint16_t Vl_Velocity_Deceleration = 0x6049;
		static const uint16_t Vl_Velocity_Quick_Stop = 0x604A;
		static const uint16_t Vl_Dimension_Factor = 0x604C;
		static const uint16_t Quick_Stop_Option_Code = 0x605A;
		static const uint16_t Shutdown_Option_Code = 0x605B;
		static const uint16_t Disable_Option_Code = 0x605C;
		static const uint16_t Halt_Option_Code = 0x605D;
		static const uint16_t Fault_Option_Code = 0x605E;
		static const uint16_t Modes_Of_Operation = 0x6060;
		static const uint16_t Modes_Of_Operation_Display = 0x6061;
		static const uint16_t Position_Demand_Value = 0x6062;
		static const uint16_t Position_Actual_Internal_Value = 0x6063;
		static const uint16_t Position_Actual_Value = 0x6064;
		static const uint16_t Following_Error_Window = 0x6065;
		static const uint16_t Following_Error_Time_Out = 0x6066;
		static const uint16_t Position_Window = 0x6067;
		static const uint16_t Position_Window_Time = 0x6068;
		static const uint16_t Velocity_Demand_Value = 0x606B;
		static const uint16_t Velocity_Actual_Value = 0x606C;
		static const uint16_t Velocity_Window = 0x606D;
		static const uint16_t Velocity_Window_Time = 0x606E;
		static const uint16_t Target_Torque = 0x6071;
		static const uint16_t Max_Torque = 0x6072;
		static const uint16_t Torque_Demand = 0x6074;
		static const uint16_t Torque_Actual_Value = 0x6077;
		static const uint16_t Target_Position = 0x607A;
		static const uint16_t Position_Range_Limit = 0x607B;
		static const uint16_t Home_Offset = 0x607C;
		static const uint16_t Software_Position_Limit = 0x607D;
		static const uint16_t Polarity = 0x607E;
		static const uint16_t Profile_Velocity = 0x6081;
		static const uint16_t End_Velocity = 0x6082;
		static const uint16_t Profile_Acceleration = 0x6083;
		static const uint16_t Profile_Deceleration = 0x6084;
		static const uint16_t Quick_Stop_Deceleration = 0x6085;
		static const uint16_t Motion_Profile_Type = 0x6086;
		static const uint16_t Torque_Slope = 0x6087;
		static const uint16_t Position_Encoder_Resolution = 0x608F;
		static const uint16_t Gear_Ratio = 0x6091;
		static const uint16_t Feed_Constant = 0x6092;
		static const uint16_t Homing_Method = 0x6098;
		static const uint16_t Homing_Speed = 0x6099;
		static const uint16_t Homing_Acceleration = 0x609A;
		static const uint16_t Profile_Jerk = 0x60A4;
		static const uint16_t Interpolation_Data_Record = 0x60C1;
		static const uint16_t Interpolation_Time_Period = 0x60C2;
		static const uint16_t Interpolation_Data_Configuration = 0x60C4;
		static const uint16_t Max_Acceleration = 0x60C5;
		static const uint16_t Max_Deceleration = 0x60C6;
		static const uint16_t Positioning_Option_Code = 0x60F2;
		static const uint16_t Following_Error_Actual_Value = 0x60F4;
		static const uint16_t Digital_Inputs = 0x60FD;
		static const uint16_t Digital_Outputs = 0x60FE;
		static const uint16_t Target_Velocity = 0x60FF;
		static const uint16_t Supported_Drive_Modes = 0x6502;
		static const uint16_t IEEE_802_MAC_Address = 0x6503;
		static const uint16_t Http_Drive_Catalogue_Address = 0x6505;
	} ObjectIndexes;

	bool usePDO;
	bool operational;

	const uint8_t MODE_PROFILE_POSITION = 1;
	const uint8_t MODE_SYNC_POSITION = 8;
	const uint8_t MODE_SYNC_VELOCITY = 9;

	modbus_t *mb;
	const std::string controller_IP;
	uint8_t operationMode;
	std::bitset<16> controlword;
	std::bitset<16> statusword;
	int32_t actualPosition;
	int32_t targetPosition;
	int32_t targetVelocity;
	int32_t zeropos;
	int8_t msSyncTime;

	const uint32_t maxvel = 400;
	const int SLEEPRW = 1e5;

	template<typename srctype>
	int writeObject(uint16_t index, uint8_t subindex, srctype &src)
	{
		uint8_t rawsrc[MODBUS_MAX_ADU_LENGTH];
		uint16_t len = sizeof(src);
		std::memcpy(rawsrc, &src, len);
		return modbus_write_canopen(mb, 0x01, index, subindex, len, rawsrc);
	}

	template<typename desttype>
	int readObject(uint16_t index, uint8_t subindex, desttype &dest)
	{
		uint8_t rawdest[MODBUS_MAX_ADU_LENGTH];
		desttype t = 0;
		memcpy(rawdest, &t, sizeof(t));
		uint16_t len = sizeof(dest);
		auto ret = modbus_read_canopen(mb, 0x01, index, subindex, len, rawdest);
		std::memcpy(&dest, rawdest, len);
		return ret;
	}

public:
	N5Motor(std::string controller_IP);
	~N5Motor();

	void performHoming(int32_t offsetenc);

	bool checkFinished();

	void setupEndSwitches();
	struct readPDO readPDO;

	int setUsePDO();
	int setDontUsePDO();

	bool isOperational();

	int quickStop();

	int emergencyShutdown();

	void setupProfilePositionMode();
	void setupProfileVelocityMode();

	int setControlword();
	int setControlword(uint16_t newControlword);
	int setControlword(std::bitset<16> newControlword);

	int setTargetPosition(int pos);
	int writeTargetPosition();

	int writeTargetVelocity();

	int startMoveToPosition();

	bool targetReached();

	int shutdown();

	int readProcessData();

	void configurePDO();

	struct readPDO getPDO();

	struct readPDO getReadPDO();

	int writeProcessData();

	void setupSynchronousPositionMode(int8_t msTimestep);

	void setupSynchronousVelocityMode(int8_t msTimestep);

	int resetOperation();

	int turnOnEnableOperation();

	int startMode();

	int calcSyncVelocity();

	int setFollowingErrorTimeout(uint16_t timeout);

	void setOffset(int32_t offsetenc);

	void startHoming(int32_t offset);

	bool checkHomingFinished();


	int setOperationMode();
	int setOperationMode(uint8_t newOperationMode);
	int getOperationMode();

	std::bitset<16> getStatusword();

	bool getStatusbit(int i);

	int getActualPosition();
	int getFollowingError();
};
//}
#endif
