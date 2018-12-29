/* Samuel Carbone
    AERO2711 Semester 2 2018
    University of Sydney
*/

/* This program controls an Eachine QX65 Quadrotor.
    Position and orientation information is received
    from the Motive server via the NatNet SDK.

    This program is based upon SampleClient code available from the NatNet SDK, and reuses parts of this code

    Motive server --> This computer --> arduino --> RC transmitter --> RC aircraft
    This program is agnostic to the actual aircraft used, since it uses an RC transmitter (Spektrum)
    the command generation method of the Aircraft class is configured in this case to control a qx65 quadrotor

    For different aircraft types, this method of the Aircraft can be overridden

*/

// Include the necessary standard libraries
#include <vector> // A managed form of arrays
#include <iostream> // C++ stream style console inputs/outputs
#include <sstream> // C++ stream for converting strings
#define _USE_MATH_DEFINES // Need to define this prior to including <math.h>
#include <math.h> // Used for maths functions and constants
#include <chrono> // Used for timer
#include <inttypes.h> // 
#using <System.dll> 

// Include the NatNet SDK libraries
#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <NatNetClient.h>

// Include the aircraft class
#include "Aircraft.hpp"

// Include the necessary libraries for keyboard inputs
#ifdef _WIN32
    #include <conio.h> // windows only
#else
    #include <unistd.h> // UNIX
    #include <termios.h>
#endif

// Declare functions for connecting and receiving data from the Motive server

// Called when a new server is discovered
void NATNET_CALLCONV ServerDiscoveredCallback(const sNatNetDiscoveredServer* pDiscoveredServer, void* pUserContext);
// Called when a new frame is available
void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData);
// Called when a new message is available
void NATNET_CALLCONV MessageHandler(Verbosity msgType, const char* msg);
// 
int ConnectClient();
//
void PrintDataDescriptions();

// Create a serial port object
// Properties
int baudrate = 115200;
char portName = "COM8"
ref struct Globals {
	static System::IO::Ports::SerialPort^ arduinoSerial = gcnew System::IO::Ports::SerialPort(portName, baudrate);
};

// Note on a convention used within the program:
// g_pClient --> Global Packet Client

// Global variables
NatNetClient* g_pClient = NULL; // The NatNet client object pointer
std::vector<sNatNetDiscoveredServer> g_discoveredServers; // Each detected server is added to this vector array
sNatNetClientConnectParams g_connectParams; // The server connection parameters
char g_discoveredMulticastGroupAddr[kNatNetIpv4AddrStrLenMax] = NATNET_DEFAULT_MULTICAST_ADDRESS; // Character array to hold the multicast address
sServerDescription g_serverDescription;

// Aircraft object with a specified rigid body streaming ID
// To determine the streaming ID for a rigid body, run the program with PrintDataDescriptions
// The output will show the streaming ID for easch detected rigid body
Aircraft qx65(2);

// File pointers for the file the flight data and log messages are written to
FILE* g_messageFile;
FILE* g_dataFile;

// This is additional code for the use of flying in a circle.
// It is not part of the core functionality, and can be replaced depending on which path is to be flown
bool circle = false;
double circle_time = 0;

int main() {
	
    // Setup start

	// Prompt the user for the test disgnation used for the file name names
	std::cout << std::endl << "Please input the test designation (uised for file name generation): ";
	std::string test_desig; // String which holds the input
	std::cin >> test_desig; // Stream the user input to a string
	std::string dataFileName = "data_test_" + test_desig + ".csv"; // Concatenate to create file names
	std::string messageFileName = "log_test_" + test_desig + ".txt";

	// Create the data and message files
	g_dataFile = fopen(dataFileName.c_str(), "w"); // Open the file where raw data is written to
	g_messageFile = fopen(messageFileName.c_str(), "w"); // Open the file where messages are written to

	// PID controllers parameters
	qx65.pids = { PID(18, 0.001, 21000), // x
				PID(18, 0.001, 21000), // y
				PID(200, 0.001, 80000), // z
				PID(100, 0, 10000) }; // yaw

	// Min channel values before conversion to PPM
	qx65.min_c = { -100, -100, -100, -100 };

	// Max channel values before conversion to PPM
	qx65.max_c = { 100, 100, 100, 100 };

	// Set the channel directions (-1 is reversed)
    qx65.channelDirections = {1, 1, 1, 1, 1, 1, 1, 1}; 

	// Add and extra 10 to the throttle (so at 0 it should hover)
    qx65.throttleTrim = 10;

	// Set the target position and yaw
	qx65.target = { 0,0,1,0 }; // {x, y, z, yaw}

	// Write the data file header
	qx65.writeDataHeader(g_dataFile);

    NatNet_SetLogCallback(MessageHandler); // Sets the function which handles NatNet logs

    // Create the NatNet client
    g_pClient = new NatNetClient();

    // Search for active NatNet servers on the network

    // Setup an asynchronous search for servers
    // As servers are discovered, they are appended to the g_discoveredServers vector
    NatNetDiscoveryHandle pOutDiscovery;
    NatNet_CreateAsyncServerDiscovery( &pOutDiscovery, ServerDiscoveredCallback);

    // Search for servers until one server is discovered or until 5000ms has passed
	double timeOutMs = 5000; // ms until timeout
    std::chrono::time_point<std::chrono::system_clock> startTime = std::chrono::system_clock::now();
    std::chrono::time_point<std::chrono::system_clock> currentTime = std::chrono::system_clock::now();
	double timeElapsedMs = (std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime)).count();

	// Print the time remaining
	int printCounter = int(timeOutMs/1000);
	printf("Time remaining (s): ");
	
    while(g_discoveredServers.size() == 0 && timeElapsedMs < timeOutMs) {

        // Update the timer
        currentTime = std::chrono::system_clock::now();
        timeElapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime).count();

		// Print a countdown
		if (int((timeOutMs - timeElapsedMs)/1000) < printCounter) {
			printf("%d, ", printCounter);
			--printCounter;
		}
    }
	printf("\n");

    // Either a server has been discovered, or the timer has timed out
    if(g_discoveredServers.size() > 0) {

        // Since a server has been discovered, select the first in this list as the server to be used
        // A reference is created --> This means that the discoveredServer and g_discoveredServers[0] are the same thing
        const sNatNetDiscoveredServer& discoveredServer = g_discoveredServers[0];

        // The following section is the same as the SampleClient example code
        // Setup the connection parameters discovered server
        if ( discoveredServer.serverDescription.bConnectionInfoValid ) {
            
            // Build the connection parameters.
            #ifdef _WIN32
            _snprintf_s(
            #else
            snprintf(
            #endif
                g_discoveredMulticastGroupAddr, sizeof g_discoveredMulticastGroupAddr,
                "%" PRIu8 ".%" PRIu8".%" PRIu8".%" PRIu8"",
                discoveredServer.serverDescription.ConnectionMulticastAddress[0],
                discoveredServer.serverDescription.ConnectionMulticastAddress[1],
                discoveredServer.serverDescription.ConnectionMulticastAddress[2],
                discoveredServer.serverDescription.ConnectionMulticastAddress[3]
            );

            g_connectParams.connectionType = discoveredServer.serverDescription.ConnectionMulticast ? ConnectionType_Multicast : ConnectionType_Unicast;
            g_connectParams.serverCommandPort = discoveredServer.serverCommandPort;
            g_connectParams.serverDataPort = discoveredServer.serverDescription.ConnectionDataPort;
            g_connectParams.serverAddress = discoveredServer.serverAddress;
            g_connectParams.localAddress = discoveredServer.localAddress;
            g_connectParams.multicastAddress = g_discoveredMulticastGroupAddr;
        
        } else {

            // We're missing some info because it's a legacy server.
            // Guess the defaults and make a best effort attempt to connect.
            g_connectParams.connectionType = ConnectionType_Multicast;
            g_connectParams.serverCommandPort = discoveredServer.serverCommandPort;
            g_connectParams.serverDataPort = 0;
            g_connectParams.serverAddress = discoveredServer.serverAddress;
            g_connectParams.localAddress = discoveredServer.localAddress;
            g_connectParams.multicastAddress = NULL;
        }

        // Now attempt to connect to this server
        // The following code is the same as the SampleClient code
        int iResult;

        // Connect to Motive
        iResult = ConnectClient();

        if (iResult != ErrorCode_OK) {
            printf("Error initializing client.  See log for details.  Exiting\n");
			fprintf(g_messageFile, "Error initializing client.  See log for details.  Exiting\n");
            return 1;
        }

        else {
            printf("Client initialized and ready.\n");
			fprintf(g_messageFile, "Client initialized and ready.\n");
        }

    }

    else {

        // If no servers are discovered, exit the program
		printf("Error: no servers detected\n");
		fprintf(g_messageFile, "Error: no servers detected\n");
        return -1;

    }

    // End the asynchronous search for servers
    NatNet_FreeAsyncServerDiscovery(pOutDiscovery);

    // Print information about the detected rigid bodies
	// Can be useful to figure out which objects are detected, and the rigid body ID
    PrintDataDescriptions();

	// Open the Serial Port
	try {

		Globals::arduinoSerial->Open();
		if (!Globals::arduinoSerial->IsOpen) {
			System::Console::WriteLine("[Error] Not Connected to" + "COM8");
		}

	}
	catch (System::InvalidOperationException^ ex) {
		System::Console::WriteLine("[Error]: " + ex->ToString());
	}
	catch (System::IO::IOException^ ex) {
		System::Console::WriteLine("[Error]: " + ex->ToString());
	}
	catch (System::UnauthorizedAccessException^ ex) {
		System::Console::WriteLine("[Error]: " + ex->ToString());
	}

    // Set the frame callback handler
    // The function DataHandler is called when each new frame is available
    g_pClient->SetFrameReceivedCallback(DataHandler, g_pClient);

    // Setup Done
    // At this point, the server is connected, and frame data is being provided to the callback function in a separate thread
    // We want to pause here using a while loop until the program is finished, and process keyboard inputs
	int c = getch(); // get keyboard input
	bool exit = false;

	while(!exit)
	{	
		if (c == 'q') {

			// Exit the program
			exit = true;

			printf("q pressed: exiting\n");
			fprintf(g_messageFile, "q pressed: exiting\n");

		}

		else if (c == ' ') {

			// Toggle the arm state
			qx65.setArmState(!qx65.getArmState());
			printf("[Action]: %s\n", qx65.getArmState() ? "ARMED" : "DISARMED");
			fprintf(g_messageFile, "[Action]: %s\n", qx65.getArmState() ? "ARMED" : "DISARMED");
			
		}

		else if (c == 'd') {
			
			// Start +x step manoeuver
			qx65.target = { 1,0,1,0 };

			printf("[Action]: +x step manoeuver started\n");
			fprintf(g_messageFile, "[Action]: +x step manoeuver started\n");
			fprintf(g_dataFile, "\n\n\n\n\n"); // Add extra lines to the data file to signify the start of the manoeuver. TODO: refine this
		}

		else if (c == 'a') {

			// Start -x step manoeuver
			qx65.target = { -1,0,1,0 };

			printf("[Action]: -x step manoeuver started\n");
			fprintf(g_messageFile, "[Action]: -x step manoeuver started\n");
			fprintf(g_dataFile, "\n\n\n\n\n"); // Add extra lines to the data file to signify the start of the manoeuver. TODO: refine this
		}

		else if (c == 's') {

			// Reset target position
			qx65.target = { 0,0,0.5,0 };

			printf("[Action]: reset start position\n");
			fprintf(g_messageFile, "[Action]: reset start position\n");
			fprintf(g_dataFile, "\n\n\n\n\n"); // Add extra lines to the data file to signify the start of the manoeuver. TODO: refine this

		}

		else if (c == 'c') {
			circle = true;
		}

		c = getch();

	}

    // Done - clean up.
	if (g_pClient)
	{
		g_pClient->Disconnect();
		delete g_pClient;
		g_pClient = NULL;

	}

    return 0;
}

// Same as the SampleClient example code
// For each server discovered, this function is called
// It will print out details of the server THIS CAN BE REMOVED
// The main purpose of this function is to append pDiscoveredServer to the g_discoveredServers vector
void NATNET_CALLCONV ServerDiscoveredCallback(const sNatNetDiscoveredServer* pDiscoveredServer, void* pUserContext) {

    // This section prints out the information
    char serverHotkey = '.';
    if ( g_discoveredServers.size() < 9 )
    {
        serverHotkey = static_cast<char>('1' + g_discoveredServers.size());
    }

    const char* warning = "";

    if ( pDiscoveredServer->serverDescription.bConnectionInfoValid == false )
    {
        warning = " (WARNING: Legacy server, could not autodetect settings. Auto-connect may not work reliably.)";
    }

    printf( "[%c] %s %d.%d at %s%s\n",
        serverHotkey,
        pDiscoveredServer->serverDescription.szHostApp,
        pDiscoveredServer->serverDescription.HostAppVersion[0],
        pDiscoveredServer->serverDescription.HostAppVersion[1],
        pDiscoveredServer->serverAddress,
        warning );

    // This section appends the server info to g_discoveredServers vector
    g_discoveredServers.push_back( *pDiscoveredServer );

}

// Same as the SampleClient example code
// Establish a NatNet Client connection
// The printing of server information, frame rate and samples per frame has been commented out
int ConnectClient() {
    // Release previous server
    g_pClient->Disconnect();

    // Init Client and connect to NatNet server
    int retCode = g_pClient->Connect( g_connectParams ); // Try to connect to the NatNet server

    // Check the error code
    if (retCode != ErrorCode_OK)
    {
        printf("Unable to connect to server.  Error code: %d. Exiting\n", retCode);
        return ErrorCode_Internal;
    }
    else
    {
        // connection succeeded

        void* pResult;
        int nBytes = 0;
        ErrorCode ret = ErrorCode_OK;

        // Get the server info
        memset( &g_serverDescription, 0, sizeof( g_serverDescription ) );
        ret = g_pClient->GetServerDescription( &g_serverDescription ); // Get a server description
        if ( ret != ErrorCode_OK || ! g_serverDescription.HostPresent )
        {
            printf("Unable to connect to server. Host not present. Exiting.\n");
            return 1;
        }

        /*
        // Print the motive server information
        printf("\n[SampleClient] Server application info:\n");
        printf("Application: %s (ver. %d.%d.%d.%d)\n", g_serverDescription.szHostApp, g_serverDescription.HostAppVersion[0],
            g_serverDescription.HostAppVersion[1], g_serverDescription.HostAppVersion[2], g_serverDescription.HostAppVersion[3]);
        printf("NatNet Version: %d.%d.%d.%d\n", g_serverDescription.NatNetVersion[0], g_serverDescription.NatNetVersion[1],
            g_serverDescription.NatNetVersion[2], g_serverDescription.NatNetVersion[3]);
        printf("Client IP:%s\n", g_connectParams.localAddress );
        printf("Server IP:%s\n", g_connectParams.serverAddress );
        printf("Server Name:%s\n", g_serverDescription.szHostComputerName);
        */

        /*
        // get mocap frame rate
        ret = g_pClient->SendMessageAndWait("FrameRate", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            float fRate = *((float*)pResult);
            printf("Mocap Framerate : %3.2f\n", fRate);
        }
        else
            printf("Error getting frame rate.\n");

        */

       /*
        // get # of analog samples per mocap frame of data
        ret = g_pClient->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            g_analogSamplesPerMocapFrame = *((int*)pResult);
            printf("Analog Samples Per Mocap Frame : %d\n", g_analogSamplesPerMocapFrame);
        }
        else
            printf("Error getting Analog frame rate.\n");
        
        */
    }

    return ErrorCode_OK;
}

// Same as the SampleClient example code
// Display information about detected rigid bodies
// Useful for checking streaming ID
void PrintDataDescriptions() {

	// Retrieve Data Descriptions from Motive
	printf("\n\n[SampleClient] Requesting Data Descriptions...\n");
	sDataDescriptions* pDataDefs = NULL;
	int iResult = g_pClient->GetDataDescriptionList(&pDataDefs);
	if (iResult != ErrorCode_OK || pDataDefs == NULL)
	{
		printf("[SampleClient] Unable to retrieve Data Descriptions.\n");
	}
	else
	{
        printf("[SampleClient] Received %d Data Descriptions:\n", pDataDefs->nDataDescriptions );
        for(int i=0; i < pDataDefs->nDataDescriptions; i++)
        {
            printf("Data Description # %d (type=%d)\n", i, pDataDefs->arrDataDescriptions[i].type);
            
           if(pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody)
            {
                // RigidBody
                sRigidBodyDescription* pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
				
                printf("RigidBody Name : %s\n", pRB->szName);
                printf("RigidBody ID : %d\n", pRB->ID);
                printf("RigidBody Parent ID : %d\n", pRB->parentID);
                printf("Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);

            }
            
            else
            {
                printf("Unknown data type.\n");
                // Unknown
            }
        }      
	}
	
	if (pDataDefs)
	{
		NatNet_FreeDescriptions(pDataDefs);
		pDataDefs = NULL;
	}


}


// This fuction is called each time new frame data is available
// For each detected rigid body, it will pass the data to an aircraft object
void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData) {

    // Number of rigid bodies detected
    int n = data->nRigidBodies;
    // printf("nRB: %d", data->nRigidBodies); // Print the number of rigid bodies

	// Loop for each rigid body detected
	for(int i=0; i < n; i++) {
		
        // Check if it was successfully tracked in this frame
        bool bTrackingValid = data->RigidBodies[i].params & 0x01;
        // printf(", Valid=%d, ", bTrackingValid); // Print whether this tracking was valid

        if (bTrackingValid) {
			
            // If the streaming ID of the rigid body is the ID for the aircraft
			if(data->RigidBodies[i].ID == qx65.ID) {

				// Fly in a circle by changing the target position
				if (circle) { 
					circle_time = circle_time + qx65.dtMillisec;
					double a = 2 * M_PI / 30000; // 30s per circle
					qx65.target = { cos(a*circle_time) - 1, sin(a*circle_time), 1, 0 };
				}
                
                // Pass components of the new frame data to the aircraft
				qx65.inputRbData(data->RigidBodies[i], data->CameraMidExposureTimestamp, data->iFrame, g_serverDescription.HighResClockFrequency);
				
                // Process the new data and calculate the commands
                qx65.generateCommands();

                // Map the commands to a PPM value range
                qx65.commandToPPM(); 

				// Output commands as a character string to arduino via serial
				std::string output = "";
				std::stringstream ss;

                // For each channel value, convert to a string and append to the output string
				for (int i = 0; i < qx65.numChannels; i++) {
					
                    // Temporary string which holds just this channel value
					std::string temp;
					ss << qx65.ppmValues[i];
					ss >> temp;
					ss.str("");
					ss.clear();

					// Add the PPM value to the output string, separated by a whitespace
					output.append(temp);
					output.append(" ");
				}

				// Convert the C++ string to a .NET string
				System::String^ output_2 = gcnew System::String(output.c_str());

		        // Send PPM commands to arduino via serial
				try {
					Globals::arduinoSerial->WriteLine(output_2);
				}

                // If there is an invalid operation exception error, catch it and write to the console
				catch (System::InvalidOperationException^ ex) {
					System::Console::WriteLine("[Error]: " + ex->ToString());
				}

                // Write the data for this aircraft for this frame to a file
				qx65.writeDataLine(g_dataFile);

            }
        }

        else {
            // printf("\n");
        }

	}

}

// MessageHandler receives NatNet error/debug messages
void NATNET_CALLCONV MessageHandler(Verbosity msgType, const char* msg)
{
	// logger.MessageHandler(msgType, msg);
	// Optional: Filter out debug messages
	/* if (msgType < Verbosity_Info)
	{
		return;
	} */

	// printf("\n[NatNetLib]");
	std::string totalMessage;

	switch (msgType)
	{
	case Verbosity_Debug:
		totalMessage.append("[DEBUG]: ");
		break;

	case Verbosity_Info:
		totalMessage.append("[INFO]: ");
		break;
	case Verbosity_Warning:
		totalMessage.append("[WARN]: ");
		break;
	case Verbosity_Error:
		totalMessage.append("[error]: ");
		break;
	default:
		totalMessage.append("[?????]: ");
		break;
	}

	totalMessage.append(msg);

	std::cout << totalMessage << std::endl; // Print out the message to the console
	if (g_messageFile)
		fprintf(g_messageFile, "%s\n", totalMessage.c_str()); // Write the message to the message file
}
