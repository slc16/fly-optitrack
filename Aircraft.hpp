/* Samuel Carbone
    AERO2711 Semester 2 2018
    University of Sydney
*/

/* 
    The aircraft class contains the variables and methods necessary for the control of each aircraft
*/

#ifndef AIRCRAFT_H
#define AIRCRAFT_H

#define _USE_MATH_DEFINES
#include <math.h>
#include "NatNetTypes.h"
#include "PID.hpp"
#include <vector>
#include <string>
#include <iostream>

class Aircraft {

    // Methods and variablbes which can be accessed from outside this class
    public:

		// Aircraft(); // The default constructor
		Aircraft(int id); // Constructor where the streaming ID is passed as a parameter
        ~Aircraft(); // Destructor

        int ID; // Streaming ID

        void inputRbData(sRigidBodyData rb_data, uint64_t CameraMidExposureTimestamp, int32_t iFrame, uint64_t clockFreq); // The rigid body data for each frame is passed into this function.
                                                                                                                        // It then updates the relevant variables
        void generateCommands(); // Main position controller code which calculates the output commands
		void commandToPPM(); // Convert the output commands to a PPM value range
		void setArmState(bool armed); // Set the state of the arm channel
		bool getArmState(); // Get the state of the arm channel
		void writeDataHeader(FILE* fp); // Write the header of the CSV file
		void writeDataLine(FILE* fp); // Write all the data for controller for the current frame to the CSV file
		
        std::vector<double> target; // Position and yaw target
		std::vector<double> posOffset; // Position offset
        int throttleTrim; // Offset from 50% throttle which allows for a hover
        std::vector<int> min_c; // Minimum for pre-commands
        std::vector<int> max_c; // Maximum for pre-commands
        int ppmValues[8]; // These values are sent to the transmitter
        std::vector<int> channelDirections; // Channel reversal
        std::vector<PID> pids; // PID controllers for position and yaw
		int numChannels; // Number of transmitter channels
		double dtMillisec; // Time in milliseconds between the current and previous frame

    // Methods and variables which can only be accessed within this (or derived) classes
    protected:
		
        uint64_t CameraMidExposureTimestamp_prev; // Timestamp for previous frame
		double timeMsFromStart; // Time in ms since the first frame when this controller is run
		double time_0; // The time at the first frame
		int32_t frameNumber; // The current frame number
		int32_t frameNum_0; // The frame number of the first frame passed into this controller
		
        std::vector<double> position; // Cartesian components of the position
        std::vector<double> orient; // Quaternion components of the orientation
        double yaw; // The current yaw
		double yawMinDiff; // The minimum difference between the current yaw and the target
        std::vector<double> error_n; // The error for the position and yaw
        
        double cmd_a[4]; // Commands prior to limiting and transformation
        double cmd_b[4]; // Coordinate transformed commands
        int cmd_c[8]; // Between min and max
        bool firstFrame; // Only set to false once the commands for the first frame have been set
		bool isArmed; // The arm state

};
#endif
