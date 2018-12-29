/* Samuel Carbone
    AERO2711 Semester 2 2018
    University of Sydney
*/

/* 
    The aircraft class contains the variables and methods necessary for the control of each aircraft
*/

#include "Aircraft.hpp"

/*
Aircraft::Aircraft() {
	ID = 0; // Set 0 as the default ID id it is not given in the constructor
	isArmed = false; // Start off disarmed
	numChannels = 8;

	// Reserving memory space for vectors
	target.reserve(4);
	posOffset.reserve(3);
	min_c.reserve(8);
	max_c.reserve(8);
	channelDirections.reserve(8);
	pids.reserve(4);
	position.reserve(3);
	orient.reserve(4);
	error_n.reserve(4);

}
*/

// Constructor which takes the streaming ID as an input
Aircraft::Aircraft(int id): ID(id) {
    firstFrame = true; // The first frame 
	isArmed = false; // Start off disarmed
	numChannels = 8; // Number of transmitter channels

	// Reserving memory space for vectors
	target.reserve(4);
	posOffset.reserve(3);
	min_c.reserve(8);
	max_c.reserve(8);
	channelDirections.reserve(8);
	pids.reserve(4);
	position.reserve(3);
	orient.reserve(4);
	error_n.reserve(4);
}

// Destructor
Aircraft::~Aircraft() {}

// Method to process the frame data for the rigid body
void Aircraft::inputRbData(sRigidBodyData rb_data, uint64_t CameraMidExposureTimestamp, int32_t iFrame, uint64_t clockFreq) {

    // Check if this is the first frame of data, and if so, setup some of the parameters
    if(firstFrame) {

        // Set the current position as the offset so that it is the origin
		posOffset = { rb_data.x, rb_data.y, rb_data.z };

        // Set the previous time value
        CameraMidExposureTimestamp_prev = CameraMidExposureTimestamp;
		
		// Set the initial time value
		time_0 = (static_cast<double>(CameraMidExposureTimestamp) * 1000) / static_cast<double>(clockFreq); // ms

		// Set the initial frame number
		frameNum_0 = iFrame;
    }
	
	// Calculate time from the initial frame
	timeMsFromStart = (static_cast<double>(CameraMidExposureTimestamp) * 1000) / static_cast<double>(clockFreq) - time_0; // ms

    // Calculate the time from the previous frame
    uint64_t dt = CameraMidExposureTimestamp- CameraMidExposureTimestamp_prev; // ticks
	dtMillisec = (static_cast<double>(dt) * 1000) / static_cast<double>(clockFreq); // ms
	CameraMidExposureTimestamp_prev = CameraMidExposureTimestamp;

	// Set the current frame number
	frameNumber = iFrame - frameNum_0;

    // Update position {x, y, z}
	position = { rb_data.x - posOffset[0],
				rb_data.y - posOffset[1],
				rb_data.z - posOffset[2] };


    // Update orientation
	orient = { rb_data.qx, rb_data.qy, rb_data.qz, rb_data.qw };

}

// Generate the aircraft commands. Commands for each channel
void Aircraft::generateCommands() {

	// TODO: change from using yaw to using quaternions directly
    // Calculate the yaw -180 <-> 0 <-> +180
    yaw = -atan2(2*(orient[3]*orient[2] + orient[0]*orient[1]), 1-2*(orient[1]*orient[1] + orient[2]*orient[2])); // calculate euler angle yaw here

    // Calculate which way gives the smallest yaw difference angle
    double yawDiff1 = yaw - target[3]; // target[3] is the yaw target
    double yawDiff2;

    // Calculate the second angle difference
	if (yawDiff1 >= 0) 
		yawDiff2 = yaw - target[3] - 2 * M_PI;
	
	else 
		yawDiff2 = 2 * M_PI + yaw - target[3];

    // Set yawMinDiff to the minimum of the two angle differences
	if (abs(yawDiff1) <= abs(yawDiff2))
        yawMinDiff = yawDiff1;
	else
        yawMinDiff = yawDiff2;

    // Calculate the errors for x, y, z and yaw
	error_n = { 0,0,0,0 };
	
    for (int j = 0; j <= 2; j++)
        error_n[j] = position[j] - target[j];
    
    error_n[3] = yawMinDiff;

    // If only the first frame of data has been received
    if(firstFrame) {

        // Set the initial PID errors
        for(int j = 0; j < 4; j++)
            pids[j].error_prev = error_n[j];

        // Set the channel commands as initial neutral and 0% throttle
        cmd_a[0] = 0; // x
        cmd_a[1] = 0; // y
        cmd_a[2] = -100 - throttleTrim; // z
        cmd_a[3] = 0; // yaw

        firstFrame = false;
    }

    else {

        // Calculate the initial commands
        // cmd_a [0: x, 1: y, 2: x, 3: yaw]
        for (int j = 0; j < 4; j++) {
            cmd_a[j] = pids[j].Calculate(error_n[j], dtMillisec); // Command from PID controller
        }

    }

    // Transform the commands for attitude control mode
    // cmd_b [0: roll, 1: pitch, 2: thrust, 3: yaw]

    /*
            z   
            |  y
            | /
            |/____x

    */

    // Roll command (clockwise viewed from back is +ve)
    cmd_b[0] = cmd_a[0]*cos(yaw) + cmd_a[1]*sin(yaw);

    // Pitch command (nose down +ve)
    cmd_b[1] = cmd_a[1]*cos(yaw) - cmd_a[0]*sin(yaw);

    // Thrust command, corrected for pith/roll tilt
    // correcting using qz quaternion component
    // cmd_b[2] = 1/orient[2] * (cmd_a[2] + throttleTrim);

	// Thrust command
	cmd_b[2] = cmd_a[2] + throttleTrim;

    // Yaw command
    cmd_b[3] = cmd_a[3];

    // Limit the commands between the boundaries
    for (int j = 0; j < 4; j++) {

        cmd_c[j] = (int) (cmd_b[j] < 0 ? cmd_b[j] - 0.5 : cmd_b[j] + 0.5); // convert double to int and use proper rounding
        if (cmd_c[j] > max_c[j]) cmd_c[j] = max_c[j];
        else if (cmd_c[j] < min_c[j]) cmd_c[j] = min_c[j];

    }

    // Rearrange commands to align with tx channels
    int cmd_old = cmd_c[2];
    cmd_c[2] = cmd_c[1]; // Now pitch
    cmd_c[1] = cmd_c[0]; // Now roll
    cmd_c[0] = cmd_old; // Now throttle

    // Command for arming
	cmd_c[4] = 100 - isArmed*200; // 100 isn't armed, -100 is armed
    
    // The rest of the channels are not used
    cmd_c[5] = -100;
    cmd_c[6] = -100;
    cmd_c[7] = -100;

	// Debug: output command values
	// std::cout << cmd_c[0] << std::endl;
	std::cout << cmd_c[1] << std::endl;

}

// Create the string for the CSV file header
void Aircraft::writeDataHeader(FILE* fp) {
	
	std::string header = "frame number, time_at_capture";
	header = header + ", pos_x, target_x, pid_x_Kp, pid_x_Ki, pid_x_Kd, pid_x_P, pid_x_I, pid_x_D, pid_x_output";
	header = header + ", pos_y, target_y, pid_y_Kp, pid_y_Ki, pid_y_Kd, pid_y_P, pid_y_I, pid_y_D, pid_y_output";
	header = header + ", pos_z, target_z, pid_z_Kp, pid_z_Ki, pid_z_Kd, pid_z_P, pid_z_I, pid_z_D, pid_z_output";
	header = header + ", yaw, yaw_target, pid_yaw_Kp, pid_yaw_Ki, pid_yaw_Kd, pid_yaw_P, pid_yaw_I, pid_yaw_D, pid_yaw_output";
	header = header + ", qx, qy, qz, qw";
	header = header + ", chn_1, chn_2, chn_3, chn_4, chn_5, chn_6, chn_7, chn_8";
	header = header + "\n";
	
	if (fp) {
		fprintf(fp, header.c_str());
	}
}

// Write the data for the current time to a file
void Aircraft::writeDataLine(FILE* fp) {
	
	// frame number, time at capture
	fprintf(fp, "%d, %.5f", frameNumber, timeMsFromStart);

	// Record data from each of the position controllers
	for (int i = 0; i < 3; i++) {

		// pos, target, pid_Kp, pid_Ki, pid_Kd, pid_P, pid_I, pid_D, pid_output
		fprintf(fp, ", %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f", position[i], target[i], (pids.at(i)).Kp, (pids.at(i)).Ki, (pids.at(i)).Kd, (pids.at(i)).P, (pids.at(i)).I, (pids.at(i)).D, (pids.at(i)).result);

	}

	// Record data from the yaw controller
	// yaw, yaw_target, pid_yaw_Kp, pid_yaw_Ki, pid_yaw_Kd, pid_yaw_P, pid_yaw_I, pid_yaw_D, pid_yaw_output
	fprintf(fp, ", %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f", yaw, target[3], (pids.at(3)).Kp, (pids.at(3)).Ki, (pids.at(3)).Kd, (pids.at(3)).P, (pids.at(3)).I, (pids.at(3)).D, (pids.at(3)).result);

	// Record orientation data
	fprintf(fp, ", %.5f, %.5f, %.5f, %.5f", orient[0], orient[1], orient[2], orient[3]);

	// Record channel data before scaling has occurred
	fprintf(fp, ", %d, %d, %d, %d, %d, %d, %d, %d", cmd_c[0], cmd_c[1], cmd_c[2], cmd_c[3], cmd_c[4], cmd_c[5], cmd_c[6], cmd_c[7]);

	// Newline
	fprintf(fp, "\n");
}

// Convert the commands to a PPM value range
void Aircraft::commandToPPM() {

	// Map to PPM value range of 1000 to 2000 (100 to 200)
	for (int j = 0; j < numChannels; j++)
		ppmValues[j] = channelDirections[j] * 5 * cmd_c[j] + 1500;
}

// Set the arm state
void Aircraft::setArmState(bool armed) {
	isArmed = armed;
}

// Return the arm state
bool Aircraft::getArmState() {
	return isArmed;
}
