/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Gonçalo Cabrita on 19/05/2010
*********************************************************************/
#include "cereal_port/CerealPort.h"

// Packets sizes
#define SCI_PACKET_GROUP_0_SIZE						26
#define SCI_PACKET_GROUP_1_SIZE						10
#define SCI_PACKET_GROUP_2_SIZE						6
#define SCI_PACKET_GROUP_3_SIZE						10
#define SCI_PACKET_GROUP_4_SIZE						14
#define SCI_PACKET_GROUP_5_SIZE						12
#define SCI_PACKET_GROUP_6_SIZE						52
#define SCI_PACKET_GROUP_100_SIZE					80
#define SCI_PACKET_GROUP_101_SIZE					28
#define SCI_PACKET_GROUP_106_SIZE					12
#define SCI_PACKET_GROUP_107_SIZE					9
#define SCI_PACKET_BUMPS_DROPS_SIZE					1
#define SCI_PACKET_WALL_SIZE							1
#define SCI_PACKET_CLIFF_LEFT_SIZE					1
#define SCI_PACKET_CLIFF_FRONT_LEFT_SIZE				1
#define SCI_PACKET_CLIFF_FRONT_RIGHT_SIZE			1
#define SCI_PACKET_CLIFF_RIGHT_SIZE					1
#define SCI_PACKET_VIRTUAL_WALL_SIZE					1
#define SCI_PACKET_WHEEL_OVERCURRENTS_SIZE			1
#define SCI_PACKET_DIRT_DETECT_SIZE					1
#define SCI_PACKET_IR_CHAR_OMNI_SIZE					1
#define SCI_PACKET_IR_CHAR_LEFT_SIZE					1
#define SCI_PACKET_IR_CHAR_RIGHT_SIZE				1
#define SCI_PACKET_BUTTONS_SIZE						1
#define SCI_PACKET_DISTANCE_SIZE						2
#define SCI_PACKET_ANGLE_SIZE						2
#define SCI_PACKET_CHARGING_STATE_SIZE				1
#define SCI_PACKET_VOLTAGE_SIZE						2
#define SCI_PACKET_CURRENT_SIZE						2
#define SCI_PACKET_TEMPERATURE_SIZE					1
#define SCI_PACKET_BATTERY_CHARGE_SIZE				2
#define SCI_PACKET_BATTERY_CAPACITY_SIZE				2
#define SCI_PACKET_WALL_SIGNAL_SIZE					2
#define SCI_PACKET_CLIFF_LEFT_SIGNAL_SIZE			2
#define SCI_PACKET_CLIFF_FRONT_LEFT_SIGNAL_SIZE		2
#define SCI_PACKET_CLIFF_FRONT_RIGHT_SIGNAL_SIZE		2
#define SCI_PACKET_CLIFF_RIGHT_SIGNAL_SIZE			2
#define SCI_PACKET_CHARGE_SOURCES_SIZE				1
#define SCI_PACKET_SCI_MODE_SIZE						1
#define SCI_PACKET_SONG_NUMBER_SIZE					1
#define SCI_PACKET_SONG_PLAYING_SIZE					1
#define SCI_PACKET_STREAM_PACKETS_SIZE				1
#define SCI_PACKET_REQ_VELOCITY_SIZE					2
#define SCI_PACKET_REQ_RADIUS_SIZE					2
#define SCI_PACKET_REQ_RIGHT_VELOCITY_SIZE			2
#define SCI_PACKET_REQ_LEFT_VELOCITY_SIZE			2
#define SCI_PACKET_RIGHT_ENCODER_SIZE				2
#define SCI_PACKET_LEFT_ENCODER_SIZE					2
#define SCI_PACKET_LIGHT_BUMPER_SIZE					1
#define SCI_PACKET_LIGHT_BUMPER_LEFT_SIZE			2
#define SCI_PACKET_LIGHT_BUMPER_FRONT_LEFT_SIZE		2
#define SCI_PACKET_LIGHT_BUMPER_CENTER_LEFT_SIZE		2
#define SCI_PACKET_LIGHT_BUMPER_CENTER_RIGHT_SIZE	2
#define SCI_PACKET_LIGHT_BUMPER_FRONT_RIGHT_SIZE		2
#define SCI_PACKET_LIGHT_BUMPER_RIGHT_SIZE			2
#define SCI_PACKET_LEFT_MOTOR_CURRENT_SIZE			2
#define SCI_PACKET_RIGHT_MOTOR_CURRENT_SIZE			2
#define SCI_PACKET_BRUSH_MOTOR_CURRENT_SIZE			2
#define SCI_PACKET_SIDE_BRUSH_MOTOR_CURRENT_SIZE		2
#define SCI_PACKET_STASIS_SIZE						1

// OI Modes
#define SCI_MODE_OFF				0
#define SCI_MODE_PASSIVE			1
#define SCI_MODE_SAFE			2
#define SCI_MODE_FULL			3

// Delay after mode change in ms
#define OI_DELAY_MODECHANGE_MS	20

// Charging states
#define SCI_CHARGING_NO			0
#define SCI_CHARGING_RECOVERY	1
#define SCI_CHARGING_CHARGING	2
#define SCI_CHARGING_TRICKLE		3
#define SCI_CHARGING_WAITING		4
#define SCI_CHARGING_ERROR		5

// IR Characters
#define FORCE_FIELD						161
#define GREEN_BUOY						164
#define GREEN_BUOY_FORCE_FIELD			165
#define RED_BUOY						168
#define RED_BUOY_FORCE_FIELD			169
#define RED_BUOY_GREEN_BUOY				172
#define RED_BUOY_GREEN_BUOY_FORCE_FIELD	173
#define VIRTUAL_WALL					162

// Positions
#define LEFT				0
#define RIGHT				1
#define FRONT_LEFT			2
#define FRONT_RIGHT			3
#define CENTER_LEFT			4
#define CENTER_RIGHT		5
#define OMNI				2
#define MAIN_BRUSH			2
#define SIDE_BRUSH			3

// Buttons
#define BUTTON_CLOCK		7
#define BUTTON_SCHEDULE		6
#define BUTTON_DAY			5
#define BUTTON_HOUR			4
#define BUTTON_MINUTE		3
#define BUTTON_DOCK			2
#define BUTTON_SPOT			1
#define BUTTON_CLEAN		0

// Roomba Dimensions
#define ROOMBA_BUMPER_X_OFFSET		0.050
#define ROOMBA_DIAMETER				0.330
#define ROOMBA_AXLE_LENGTH			0.235

#define ROOMBA_MAX_LIN_VEL_MM_S		500
#define ROOMBA_MAX_ANG_VEL_RAD_S	2  
#define ROOMBA_MAX_RADIUS_MM		2000

//! Roomba max encoder counts
#define ROOMBA_MAX_ENCODER_COUNTS	65535
//! Roomba encoder pulses to meter constant
#define ROOMBA_PULSES_TO_M			0.000445558279992234

#define MAX_PATH 32


#ifndef MIN
#define MIN(a,b) ((a < b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b) ((a > b) ? (a) : (b))
#endif
#ifndef NORMALIZE
#define NORMALIZE(z) atan2(sin(z), cos(z))
#endif

namespace irobot
{
	//! SCI op codes
	/*!
	 * Op codes for commands as specified by the iRobot Serial Command Interface.
	 */
	typedef enum _SCI_Opcode
	{
		// Command opcodes
		SCI_OPCODE_START = 128,
		SCI_OPCODE_BAUD = 129,
		SCI_OPCODE_CONTROL = 130,
		SCI_OPCODE_SAFE = 131,
		SCI_OPCODE_FULL = 132,
		SCI_OPCODE_POWER = 133,
		SCI_OPCODE_SPOT = 134,
		SCI_OPCODE_CLEAN = 135,
		SCI_OPCODE_MAX = 136,
		SCI_OPCODE_DRIVE = 137,
		SCI_OPCODE_MOTORS = 138,
		SCI_OPCODE_LEDS = 139,
		SCI_OPCODE_SONG = 140,
		SCI_OPCODE_PLAY = 141,
		SCI_OPCODE_SENSORS = 142,
		SCI_OPCODE_FORCE_DOCK = 143,
//		SCI_OPCODE_PWM_MOTORS = 144,
//		SCI_OPCODE_DRIVE_DIRECT = 145,
//		SCI_OPCODE_DRIVE_PWM = 146,
//		SCI_OPCODE_STREAM = 148,
//		SCI_OPCODE_QUERY = 149,
//		SCI_OPCODE_PAUSE_RESUME_STREAM = 150,
//		SCI_OPCODE_SCHEDULE_LEDS = 162,
//		SCI_OPCODE_DIGIT_LEDS_RAW = 163,
//		SCI_OPCODE_DIGIT_LEDS_ASCII = 164,
//		SCI_OPCODE_BUTTONS = 165,
//		SCI_OPCODE_SCHEDULE = 167,
//		SCI_OPCODE_SET_DAY_TIME = 168

	} SCI_Opcode;

	//! SCI driving codes
	/*!
	 * Op codes for special driving commands commands as specified by the iRobot Serial Command Interface.
	 */
	typedef enum _SCI_Drive_Special
	{
		SCI_DRIVE_NORMAL = 0,
		SCI_DRIVE_STRAIGHT = 0x8000,
		SCI_DRIVE_CW = -1,
		SCI_DRIVE_CCW = 1
	}SCI_Drive_Special;


	//! SCI packet id
	/*!
	 * Packet ids for sensors as specified by the iRobot Serial Command Interface.
	 */
	typedef enum _SCI_Packet_ID
	{
		// Sensor Packets
		SCI_PACKET_GROUP_0 = 0,			//! SCI packets 7-26
		SCI_PACKET_GROUP_1 = 1,			//! SCI packets 7-16
		SCI_PACKET_GROUP_2 = 2,			//! SCI packets 17-20
		SCI_PACKET_GROUP_3 = 3,			//! SCI packets 21-26
		SCI_PACKET_GROUP_4 = 4,			//! SCI packets 27-34
		SCI_PACKET_GROUP_5 = 5,			//! SCI packets 35-42
		SCI_PACKET_GROUP_6 = 6,			//! SCI packets 7-42
		SCI_PACKET_GROUP_100 = 100,		//! SCI packets 7-58
		SCI_PACKET_GROUP_101 = 101,		//! SCI packets 43-58
		SCI_PACKET_GROUP_106 = 106,		//! SCI packets 46-51
		SCI_PACKET_GROUP_107 = 107,		//! SCI packets 54-58
		SCI_PACKET_BUMPS_DROPS = 7,
		SCI_PACKET_WALL = 8,
		SCI_PACKET_CLIFF_LEFT = 9,
		SCI_PACKET_CLIFF_FRONT_LEFT = 10,
		SCI_PACKET_CLIFF_FRONT_RIGHT = 11,
		SCI_PACKET_CLIFF_RIGHT = 12,
		SCI_PACKET_VIRTUAL_WALL = 13,
		SCI_PACKET_WHEEL_OVERCURRENTS = 14,
		SCI_PACKET_DIRT_DETECT = 15,
		SCI_PACKET_IR_CHAR_OMNI = 17,
		SCI_PACKET_BUTTONS = 18,
		SCI_PACKET_DISTANCE = 19,
		SCI_PACKET_ANGLE = 20,
		SCI_PACKET_CHARGING_STATE = 21,
		SCI_PACKET_VOLTAGE = 22,
		SCI_PACKET_CURRENT = 23,
		SCI_PACKET_TEMPERATURE = 24,
		SCI_PACKET_BATTERY_CHARGE = 25,
		SCI_PACKET_BATTERY_CAPACITY = 26,
		SCI_PACKET_WALL_SIGNAL = 27,
		SCI_PACKET_CLIFF_LEFT_SIGNAL = 28,
		SCI_PACKET_CLIFF_FRONT_LEFT_SIGNAL = 29,
		SCI_PACKET_CLIFF_FRONT_RIGHT_SIGNAL = 30,
		SCI_PACKET_CLIFF_RIGHT_SIGNAL = 31,
		SCI_PACKET_CHARGE_SOURCES = 34,
		SCI_PACKET_SCI_MODE = 35,
		SCI_PACKET_SONG_NUMBER = 36,
		SCI_PACKET_SONG_PLAYING = 37,
		SCI_PACKET_STREAM_PACKETS = 38,
		SCI_PACKET_REQ_VELOCITY = 39,
		SCI_PACKET_REQ_RADIUS = 40,
		SCI_PACKET_REQ_RIGHT_VELOCITY = 41,
		SCI_PACKET_REQ_LEFT_VELOCITY = 42,
		SCI_PACKET_RIGHT_ENCODER = 43,
		SCI_PACKET_LEFT_ENCODER = 44,
		SCI_PACKET_LIGHT_BUMPER = 45,
		SCI_PACKET_LIGHT_BUMPER_LEFT = 46,
		SCI_PACKET_LIGHT_BUMPER_FRONT_LEFT = 47,
		SCI_PACKET_LIGHT_BUMPER_CENTER_LEFT = 48,
		SCI_PACKET_LIGHT_BUMPER_CENTER_RIGHT = 49,
		SCI_PACKET_LIGHT_BUMPER_FRONT_RIGHT = 50,
		SCI_PACKET_LIGHT_BUMPER_RIGHT = 51,
		SCI_PACKET_IR_CHAR_LEFT = 52,
		SCI_PACKET_IR_CHAR_RIGHT = 53,
		SCI_PACKET_LEFT_MOTOR_CURRENT = 54,
		SCI_PACKET_RIGHT_MOTOR_CURRENT = 55,
		SCI_PACKET_BRUSH_MOTOR_CURRENT = 56,
		SCI_PACKET_SIDE_BRUSH_MOTOR_CURRENT = 57,
		SCI_PACKET_STASIS = 58
	
	} SCI_Packet_ID;


	/*! \class OpenInterface OpenInterface.h "inc/OpenInterface.h"
	 *  \brief C++ class implementation of the iRobot SCI.
	 *
	 * This class implements the iRobot Serial Command Interface protocolor as described by iRobot. Based on the Player Roomba driver writen by Brian Gerkey.
	 */
	class OpenInterface
	{
		public:
	
		//! Constructor
		/*!
		 * By default the constructor will set the Roomba to read only the encoder counts (for odometry).
		 *
		 *  \param new_serial_port    Name of the serial port to open.
		 *
		 *  \sa setSensorPackets()
		 */
		OpenInterface(const char * new_serial_port);
		//! Destructor
		~OpenInterface();
	
		//! Open the serial port
		/*!
		 *  \param full_control    Whether to set the Roomba on SCImode full or not.
		 */
		int openSerialPort(bool full_control);
		//! Close the serial port
		int closeSerialPort();
	
		//! Power down the Roomba.
		int powerDown();
	
		//! Set sensor packets
		/*!
		*  Set the sensor packets one wishes to read from the roomba. By default the constructor will set the Roomba to read only the encoder counts (for odometry). 
		*
		*  \param new_sensor_packets  	Array of sensor packets to read.
		*  \param new_num_of_packets  	Number of sensor packets in the array.
		*  \param new_buffer_size		Size of the resulting sensor data reply.
		*
		*  \return 0 if ok, -1 otherwise.
		*/
		int setSensorPackets(SCI_Packet_ID * new_sensor_packets, int new_num_of_packets, size_t new_buffer_size);
		//! Read sensor packets
		/*!
		*  Requested the defined sensor packets from the Roomba. If you need odometry and you requested encoder data you need to call calculateOdometry() afterwords.
		*
		*  \param timeout		Timeout in milliseconds.
		*
		* \sa calculateOdometry()
		*
		*  \return 0 if ok, -1 otherwise.
		*/
//		int getSensorPackets(int timeout);
		
		//! Stream sensor packets. NOT TESTED
		int streamSensorPackets();
		//! Start stream. NOT TESTED
		int startStream();
		//! Stom stream. NOT TESTED
		int stopStream();
	
		//! Calculate Roomba odometry. Call after reading encoder pulses.
		void calculateOdometry();
	
		//! Drive
		/*!
		*  Send velocity commands to Roomba.
		*
		*  \param velocity  	Linear speed.
		*  \param radius    	Turning Diameter.
		*  \param special		Special command (turn in place or go straight), overrides radius
		*
		*  \return 0 if ok, -1 otherwise.
		*/
		int drive(double velocity, double radius, SCI_Drive_Special special = SCI_DRIVE_NORMAL);
		//! Drive direct
		/*!
		*  Send velocity commands to Roomba.
		*
		*  \param velocity  	Left wheel speed.
		*  \param radius  	Right wheel speed.
		*
		*  \return 0 if ok, -1 otherwise.
		*/
//		int driveDirect(int left_speed, int right_speed);
		//! Drive PWM
		/*!
		*  Set the motors pwms. NOT IMPLEMENTED
		*
		*  \param left_pwm  	Left wheel motor pwm.
		*  \param right_pwm  	Right wheel motor pwm.
		*
		*  \return 0 if ok, -1 otherwise.
		*/
//		int drivePWM(int left_pwm, int right_pwm);
	
		//! Set brushes
		/*!
		*  Set the various brushes motors.
		*
		*  \param side_brush  			Side brush on (1) or off (0).
		*  \param vacuum  				Vacuum on (1) or off (0).
		*  \param main_brush 			Main brush on (1) or off (0).
		*  \param side_brush_clockwise 	Wether to rotate the side brush clockwise or not.
		*  \param main_brush_dir 		Main brush direction.
		*
		*  \return 0 if ok, -1 otherwise.
		*/
		int brushes(bool side_brush, bool vacuum, bool main_brush);
		//! Set brushes motors pwms
		/*!
		*  Set the brushes motors pwms. This is very interesting. One could disconnect the motors and plug other actuators that could be controller over pwm on the Roomba.
		*
		*  \param main_brush 	Main brush motor pwm.
		*  \param side_brush  	Side brush motor pwm.
		*  \param vacuum  		Vacuum motor pwm.
		*
		*  \return 0 if ok, -1 otherwise.
		*/
//		int brushesPWM(char main_brush, char side_brush, char vacuum);
	
		//! Set the Roomba in cleaning mode. Returns the SCImode to safe.
		int clean();
		//! Set the Roomba in max cleaning mode. Returns the SCImode to safe.
		int max();
		//! Set the Roomba in spot cleaning mode. Returns the SCImode to safe.
		int spot();
		//! Sends the Roomba to the dock. Returns the SCImode to safe.
		int goDock();
	
		
		//! Set song
		/*!
		*  Record a song on Roombas memory. Songs can be played with playSong().
		*
		*  \param song_number 	Song id (from 0 to 15) so you can play it later.
		*  \param song_length  	Number of notes in the song.
		*  \param notes  		Array of notes.
		*  \param note_lengths  Array of notes length.
		*
		*  \sa playSong()
		*
		*  \return 0 if ok, -1 otherwise.
		*/
		int setSong(unsigned char song_number, unsigned char song_length, unsigned char *notes, unsigned char *note_lengths);
		//! Play song
		/*!
		*  Play a song previously recorded on Roombas memory. You can only play songs stored with setSong().
		*
		*  \param song_number 	Song id (from 0 to 15) so you can play it later.
		*
		*  \sa setSong()
		*
		*  \return 0 if ok, -1 otherwise.
		*/
		int playSong(unsigned char song_number);
	
		//! Set leds
		/*!
		*  Set the leds state on the Roomba.
		*
		*  \param check_robot 		Check robot led.
		*  \param dock		 		Dock led.
		*  \param spot		 		Spot led.
		*  \param debris			Debris led.
		*  \param power_color		Power led color, varies from green (yellow, orange) to red.
		*  \param power_intensity	Power led intensity.
		*
		*  \return 0 if ok, -1 otherwise.
		*/
		int setLeds(unsigned char check_robot, unsigned char dock, unsigned char spot, unsigned char debris, unsigned char power_color, unsigned char power_intensity);
		//! Set scheduling leds
		/*!
		*  Set the leds state on the Roomba.
		*
		*  \param sun 		Sunday, 1 on, 0 off.
		*  \param mon		Monday, 1 on, 0 off.
		*  \param tue		Tuesday, 1 on, 0 off.
		*  \param wed		You get the idea...
		*  \param thu		...
		*  \param fri		Boooooring!
		*  \param sat		Saturday, pfew!
		*  \param colon		Colon on the clock.
		*  \param pm		PM.
		*  \param am		AM.
		*  \param clock		Clock.
		*  \param schedule	Schedule.
		*
		*  \return 0 if ok, -1 otherwise.
		*/
//		int setSchedulingLeds(unsigned char sun, unsigned char mon, unsigned char tue, unsigned char wed, unsigned char thu, unsigned char fri, unsigned char sat, unsigned char colon, unsigned char pm, unsigned char am, unsigned char clock, unsigned char schedule);
		//! Set digit leds
		/*!
		*  Set the digit leds on the Roomba, the ones on the clock. Digits are ordered from left to right on the robot, 3, 2, 1, 0.
		*
		*  \param digit3 		Digit 3
		*  \param digit2		Digit 2
		*  \param digit1		Digit 1
		*  \param digit0		Digit 0
		*
		*  \return 0 if ok, -1 otherwise.
		*/
//		int setDigitLeds(unsigned char digit3, unsigned char digit2, unsigned char digit1, unsigned char digit0);
	
		//! Current operation mode, one of ROOMBA_MODE_'s
		unsigned char SCImode_;
	
		//! Sends the Roomba to the dock. Returns the SCImode to safe.
		void resetOdometry();
		void setOdometry(double new_x, double new_y, double new_yaw);
	
		//! Roomba odometry x
		double odometry_x_;
		//! Roomba odometry y
		double odometry_y_;
		//! Roomba odometry yaw
		double odometry_yaw_;
	
		bool wall_;						//! Wall detected.
		bool virtual_wall_;				//! Virtual wall detected.
		bool cliff_[4];					//! Cliff sensors. Indexes: LEFT FRONT_LEFT FRONT_RIGHT RIGHT
		bool bumper_[2];				//! Bumper sensors. Indexes: LEFT RIGHT
		bool ir_bumper_[6];				//! IR bumper sensors. Indexes: LEFT FRONT_LEFT CENTER_LEFT CENTER_RIGHT FRONT_RIGHT RIGHT
		bool wheel_drop_[2];			//! Wheel drop sensors: Indexes: LEFT RIGHT
		int wall_signal_;				//! Wall signal.
		int cliff_signal_[4];			//! CLiff sensors signal. Indexes: LEFT FRONT_LEFT FRONT_RIGHT RIGHT
		int ir_bumper_signal_[6];		//! IR bumper sensors signal. Indexes: LEFT FRONT_LEFT CENTER_LEFT CENTER_RIGHT FRONT_RIGHT RIGHT
		unsigned char ir_char_[3];		//! IR characters received. Indexes: OMNI LEFT RIGHT
	
		bool buttons_[8];				//! Buttons. Indexes: BUTTON_CLOCK BUTTON_SCHEDULE BUTTON_DAY BUTTON_HOUR BUTTON_MINUTE BUTTON_DOCK BUTTON_SPOT BUTTON_CLEAN
	
		unsigned char dirt_detect_;		//! Dirt detected
	
		int motor_current_[4];			//! Motor current. Indexes: LEFT RIGHT MAIN_BRUSH SIDE_BRUSH
		bool overcurrent_[4];			//! Motor overcurrent. Indexes: LEFT RIGHT MAIN_BRUSH SIDE_BRUSH
	
		unsigned char charging_state_;	//! One of SCI_CHARGING_'s
		bool power_cord_;				//! Whether the Roomba is connected to the power cord or not.
		bool dock_;						//! Whether the Roomba is docked or not.
		float voltage_;					//! Battery voltage in volts.
		float current_;					//! Battery current in amps.
		char temperature_;				//! Battery temperature in C degrees.
		float charge_;					//! Battery charge in Ah.
		float capacity_;				//! Battery capacity in Ah
	
		int stasis_;					//! 1 when the robot is going forward, 0 otherwise

		private:
	
		//! Parse data
		/*!
		*  Data parsing function. Parses data comming from the Roomba.
		*
		*  \param buffer  			Data to be parsed.
		*  \param buffer_length  	Size of the data buffer.
		*
		*  \return 0 if ok, -1 otherwise.
		*/
		int parseSensorPackets(unsigned char * buffer, size_t buffer_length);
	
		int parseBumpersAndWheeldrops(unsigned char * buffer, int index);
		int parseWall(unsigned char * buffer, int index);
		int parseLeftCliff(unsigned char * buffer, int index);
		int parseFrontLeftCliff(unsigned char * buffer, int index);
		int parseFrontRightCliff(unsigned char * buffer, int index);
		int parseRightCliff(unsigned char * buffer, int index);	
		int parseVirtualWall(unsigned char * buffer, int index);
		int parseOvercurrents(unsigned char * buffer, int index);
		int parseDirtDetector(unsigned char * buffer, int index);
		int parseIrOmniChar(unsigned char * buffer, int index);
		int parseButtons(unsigned char * buffer, int index);
		int parseDistance(unsigned char * buffer, int index);
		int parseAngle(unsigned char * buffer, int index);
		int parseChargingState(unsigned char * buffer, int index);
		int parseVoltage(unsigned char * buffer, int index);
		int parseCurrent(unsigned char * buffer, int index);
		int parseTemperature(unsigned char * buffer, int index);
		int parseBatteryCharge(unsigned char * buffer, int index);
		int parseBatteryCapacity(unsigned char * buffer, int index);
		int parseWallSignal(unsigned char * buffer, int index);
		int parseLeftCliffSignal(unsigned char * buffer, int index);
		int parseFrontLeftCliffSignal(unsigned char * buffer, int index);
		int parseFontRightCliffSignal(unsigned char * buffer, int index);
		int parseRightCliffSignal(unsigned char * buffer, int index);
		int parseChargingSource(unsigned char * buffer, int index);
		int parseOiMode(unsigned char * buffer, int index);
		int parseSongNumber(unsigned char * buffer, int index);
		int parseSongPlaying(unsigned char * buffer, int index);
		int parseNumberOfStreamPackets(unsigned char * buffer, int index);
		int parseRequestedVelocity(unsigned char * buffer, int index);
		int parseRequestedRadius(unsigned char * buffer, int index);
		int parseRequestedRightVelocity(unsigned char * buffer, int index);
		int parseRequestedLeftVelocity(unsigned char * buffer, int index);
		int parseRightEncoderCounts(unsigned char * buffer, int index);
		int parseLeftEncoderCounts(unsigned char * buffer, int index);
		int parseLightBumper(unsigned char * buffer, int index);
		int parseLightBumperLeftSignal(unsigned char * buffer, int index);
		int parseLightBumperFrontLeftSignal(unsigned char * buffer, int index);
		int parseLightBumperCenterLeftSignal(unsigned char * buffer, int index);
		int parseLightBumperCenterRightSignal(unsigned char * buffer, int index);
		int parseLightBumperFrontRightSignal(unsigned char * buffer, int index);
		int parseLightBumperRightSignal(unsigned char * buffer, int index);
		int parseIrCharLeft(unsigned char * buffer, int index);
		int parseIrCharRight(unsigned char * buffer, int index);
		int parseLeftMotorCurrent(unsigned char * buffer, int index);
		int parseRightMotorCurrent(unsigned char * buffer, int index);
		int parseMainBrushMotorCurrent(unsigned char * buffer, int index);
		int parseSideBrushMotorCurrent(unsigned char * buffer, int index);
		int parseStasis(unsigned char * buffer, int index);
	
		//! Buffer to signed int
		/*!
		*  Parsing helper function. Converts 2 bytes of data into a signed int value. 
		*
		*  \param buffer  	Data buffer.
		*  \param index  	Position in the buffer where the value is.
		*
		*  \sa buffer2unsigned_int()
		*
		*  \return A signed int value.
		*/
		int buffer2signed_int(unsigned char * buffer, int index);
		//! Buffer to unsigned int
		/*!
		*  Parsing helper function. Converts 2 bytes of data into an unsigned int value. 
		*
		*  \param buffer  	Data buffer.
		*  \param index  	Position in the buffer where the value is.
		*
		*  \sa buffer2signed_int()
		*
		*  \return An unsigned int value.
		*/
		int buffer2unsigned_int(unsigned char * buffer, int index);
	
		//! Start OI
		/*!
		*  Start the OI, change to roomba to a OImode that allows control.
		*
		*  \param full_control    Whether to set the Roomba on OImode full or not.
		*
		*  \return 0 if ok, -1 otherwise.
		*/
		int startOI(bool full_control);
		//! Send OP code
		/*!
		*  Send an OP code to Roomba.
		*
		*  \param code  			OP code to send.
		*
		*  \return 0 if ok, -1 otherwise.
		*/
		int sendOpcode(SCI_Opcode code);
	
		//! Serial port to which the robot is connected
		std::string port_name_;
		//! Cereal port object
		cereal::CerealPort * serial_port_;
	
		//! Stream variable. NOT TESTED
		bool stream_defined_;
		
		//! Number of packets
		int num_of_packets_;
		//! Array of packets
		SCI_Packet_ID * sensor_packets_;
		//! Total size of packets
		size_t packets_size_;
	
		//! Amount of distance travelled since last reading. Not being used, poor resolution. 
		int distance_;
		//! Amount of angle turned since last reading. Not being used, poor resolution. 
		int angle_;
		//! Delta encoder counts.
		int encoder_counts_[2];
		//! Last encoder counts reading. For odometry calculation.
		uint16_t last_encoder_counts_[2];
	};

}

// EOF
