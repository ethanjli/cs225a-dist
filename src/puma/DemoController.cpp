/*
 * DemoController.h
 *
 *  Created on: May 10, 2017
 *      Author: Toki Migimatsu
 */

#include <model/ModelInterface.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "puma/RedisDriver.h"

#include <iostream>
#include <fstream>
#include <string>
#include <signal.h>
#include <cstring>
#include <sys/select.h>
#include <algorithm>

#include <istream>
#include <string>
#include <vector>

using namespace std;

static const std::string kRobotName = "puma";
static const std::string kWorldFile = "resources/puma_demo/world.urdf";
static const std::string kRobotFile = "resources/puma_demo/puma.urdf";
static const HiredisServerInfo kRedisServerInfo = {
	"127.0.0.1",  // hostname
	6379,         // port
	{ 1, 500000 } // timeout = 1.5 seconds
};

static volatile bool g_runloop = true;
static RedisClient *g_redis_client = nullptr;

void stop(int) {
	g_runloop = false;
	if (g_redis_client != nullptr)
		g_redis_client->set(Puma::KEY_CONTROL_MODE, "BREAK");
	cout << "Sending BREAK." << endl;
	exit(1);
}

static void setCtrlCHandler(void (*userCallback)(int)) {
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = userCallback;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);
}

// WORKSPACE HELPERS

double clamp_range(double value, double min, double max) {
	return std::max(min, std::min(max, value));
}

double map_range(double value, double in_min, double in_max, double out_min, double out_max) {
	//value = clamp_range(value, in_min, in_max);
	double out = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	if (out_min > out_max) std::swap(out_min, out_max);
	return clamp_range(out, out_min, out_max);
}

class Calibration {
public:
	std::vector<Eigen::Vector3d> corners;
};

class Workspace {
public:
	double x_min = 0;
	double x_max = 0;
	double y_min = 0;
	double y_max = 0;
	double z_min = 0;
	double z_max = 0;
	Eigen::Vector3d plane_normal = Eigen::Vector3d::Zero();
	Eigen::Vector3d centroid = Eigen::Vector3d::Zero();

	Eigen::Vector3d mapFromPlane(double x, double y, double off_plane_z) {
		double relative_x = x - centroid.x();
		double relative_y = y - centroid.y();
		double plane_relative_z = -1 * (plane_normal.x() * relative_x + plane_normal.y() * relative_y);
		double plane_z = plane_relative_z + centroid.z();
		Eigen::Vector3d plane_point(x, y, plane_z);
		return off_plane_z * plane_normal + plane_point;
	};

	Eigen::Vector3d mapFromHapticDevice(const Eigen::Vector3d& hapticDevicePos) {
		double x = map_range(hapticDevicePos(0), 0.07, -0.05, x_min, x_max);
		double y = map_range(hapticDevicePos(1), -0.11, 0.11, y_min, y_max);
		double off_plane_z = map_range(hapticDevicePos(2), 0.0, 0.1, z_min, z_max);
		return mapFromPlane(x, y, off_plane_z);
	};

	Eigen::Vector3d mapFromHandwritingGenerator(const Eigen::Vector3d& handwritingPos) {
		double x = map_range(handwritingPos(1), -3, 7.5, x_min, x_max);
		double y = map_range(handwritingPos(0), 1, 83, y_min, y_max);
		double off_plane_z = map_range(handwritingPos(2), 0.0, 0.1, z_min, z_max);
		return mapFromPlane(x, y, off_plane_z);
	};
};

Eigen::Vector3d findCentroid(std::vector<Eigen::Vector3d>& points) {
	int num_points = points.size();
	Eigen::Vector3d sum = Eigen::Vector3d::Zero();
	for (Eigen::Vector3d point : points) {
		sum += point;
	}
	return sum / num_points;
}

Eigen::Vector3d fitPlane(std::vector<Eigen::Vector3d>& points) {
	Eigen::Vector3d centroid = findCentroid(points);

	double xx = 0;
	double xy = 0;
	double xz = 0;
	double yy = 0;
	double yz = 0;
	double zz = 0;
	for (Eigen::Vector3d point : points) {
		Eigen::Vector3d residual = point - centroid;
		xx += residual.x() * residual.x();
		xy += residual.x() * residual.y();
		xz += residual.x() * residual.z();
		yy += residual.y() * residual.y();
		yz += residual.y() * residual.z();
		zz += residual.z() * residual.z();
	}
	float det = xx * yy - xy * xy;
	float a = (yz * xy - xz * yy) / det;
	float b = (xz * xy - yz * xx) / det;
	return Eigen::Vector3d(a, b, 1.0);
}

Workspace calibrate_workspace(Calibration calibration) {
	Workspace workspace;
	Eigen::Vector3d plane_normal = fitPlane(calibration.corners);
	if (plane_normal.dot(Eigen::Vector3d(0.0, 0.0, 1.0)) < 0) plane_normal *= -1;
	std::cout << "plane normal: " << plane_normal.transpose() << std::endl;
	workspace.plane_normal = plane_normal;
	workspace.centroid = findCentroid(calibration.corners);
	workspace.x_min = 0.5 * (calibration.corners[2](0) + calibration.corners[3](0));
	workspace.x_max = 0.5 * (calibration.corners[0](0) + calibration.corners[1](0));
	workspace.y_min = 0.5 * (calibration.corners[0](1) + calibration.corners[2](1));
	workspace.y_max = 0.5 * (calibration.corners[1](1) + calibration.corners[3](1));
	workspace.z_min = 0.02;
	workspace.z_max = 0.08;

	for (Eigen::Vector3d point : calibration.corners) {
		std::cout << point.transpose() << "->" << workspace.mapFromPlane(point.x(), point.y(), 0.05).transpose() << std::endl;
	}
	return workspace;
}

// CONTROL HELPERS

void setVelocitySaturation(double max_velocity, RedisClient& redis_client) {
	Eigen::VectorXd v_max(1);
	v_max.fill(max_velocity);
	redis_client.setEigenMatrixString(Puma::KEY_VMAX, v_max);
}

void updateRobot(Model::ModelInterface *robot, RedisClient& redis_client) {
	robot->_q = redis_client.getEigenMatrixString(Puma::KEY_JOINT_POSITIONS);
	robot->_dq = redis_client.getEigenMatrixString(Puma::KEY_JOINT_VELOCITIES);
	robot->updateModel();
}

bool updateUntilInput(Model::ModelInterface *robot, LoopTimer& timer, RedisClient& redis_client) {
	// Update the robot until the user hits Enter on the console
	fd_set read_fds;
	struct timeval tv = {0, 0};
	int retval, len;
	char buff[255] = {0};
	while (g_runloop) {
		// Check stdin
		FD_ZERO(&read_fds);
		FD_SET(0, &read_fds);
		retval = select(1, &read_fds, NULL, NULL, &tv);
		if (retval == -1) { // Error
			perror("select()");
			return false;
		} else if (retval) { // Input is available now
			fgets(buff, sizeof(buff), stdin);
			len = strlen(buff) - 1;
			if (buff[len] == '\n') buff[len] = '\0';
			return true;
		}

		// Update robot
		timer.waitForNextLoop();
		updateRobot(robot, redis_client);
	}
}

bool updateUntilPosition(Model::ModelInterface *robot, LoopTimer& timer, RedisClient& redis_client, Eigen::Vector3d& target_position, double error_threshold, int steady_state_threshold) {
	// Update the robot until the user hits Enter on the console
	Eigen::Vector3d current_position;
	// Send desired position for visualization
	redis_client.setEigenMatrixString(Puma::KEY_EE_POS + "_des", target_position);
	Eigen::Vector3d previous_position = current_position;
	int steady_state_counter = 0;
	while (g_runloop) {
		// Update robot
		timer.waitForNextLoop();
		updateRobot(robot, redis_client);
		robot->position(current_position, "end-effector", Eigen::Vector3d::Zero());
		redis_client.setEigenMatrixDerivedString(Puma::KEY_EE_POS, current_position);
		double error = (current_position - target_position).norm();

		if (current_position == previous_position) {
			++steady_state_counter;
		} else {
			steady_state_counter = 0;
		}
		previous_position = current_position;
		if (error < error_threshold && steady_state_counter >= steady_state_threshold) break;
	}
}

// CONTROL PHASES

void initializeJoints(Model::ModelInterface *robot, LoopTimer& timer, RedisClient& redis_client) {
	// Declare control variables
	Eigen::VectorXd Kp(Puma::DOF);
	Eigen::VectorXd Kv(Puma::DOF);
	Eigen::VectorXd q_des(Puma::DOF);
	const double kToleranceInitQ  = 0.15;  // Joint space initialization tolerance
	const double kToleranceInitDq = 0.1;  // Joint space initialization tolerance

	// If lowering gains, set Kp first. If increasing, set Kv first.
	Kp.fill(200);
	Kv.fill(40);

	// Commands MUST be sent as an atomic Redis transaction with MSET
	q_des.setZero();
	q_des << 0.0, -0.83, 2.99, 0.0, -0.50, 0.0;
	//q_des *= M_PI / 180.0;
	redis_client.mset({
		{Puma::KEY_CONTROL_MODE, "JGOTO"},
		{Puma::KEY_COMMAND_DATA, RedisClient::encodeEigenMatrixString(q_des)},
		{Puma::KEY_KP, RedisClient::encodeEigenMatrixString(Kp)},
		{Puma::KEY_KV, RedisClient::encodeEigenMatrixString(Kv)}
	});

	// Wait for convergence
	while (g_runloop) {
		// Wait for next scheduled loop (controller must run at precise rate)
		timer.waitForNextLoop();
		// Read from Redis current sensor values and update the model
		updateRobot(robot, redis_client);

		// Check for convergence
		if ((robot->_q - q_des).lpNorm<Eigen::Infinity>() < kToleranceInitQ && robot->_dq.norm() < kToleranceInitDq) break;

		std::cout << (robot->_q - q_des).lpNorm<Eigen::Infinity>() << std::endl;
	}
}

Workspace calibratePositions(Model::ModelInterface *robot, LoopTimer& timer, RedisClient& redis_client) {
	redis_client.mset({
		{Puma::KEY_CONTROL_MODE, "FLOAT"}
	});

	Calibration calibration;

	for (int i = 0; i < 4; ++i) { // Calibrate on the 4 corners
		if (i == 0) {
			std::cout << "Move the marker tip to the far left corner away from the computer." << std::endl;
			//calibration.corners.push_back(Eigen::Vector3d(0.767857, 0.507831, -0.208286 - 0.015));

		} else if (i == 1) {
			std::cout << "Move the marker tip to the far right corner away from the computer." << std::endl;
			//calibration.corners.push_back(Eigen::Vector3d(0.78287, -0.0725817, -0.208068 - 0.015));

		} else if (i == 2) {
			std::cout << "Move the marker tip to the near left corner away from the computer." << std::endl;
			//calibration.corners.push_back(Eigen::Vector3d(0.48197, 0.496351, -0.207143));
		} else if (i == 3) {
			std::cout << "Move the marker tip to the near right corner away from the computer." << std::endl;
			//calibration.corners.push_back(Eigen::Vector3d(0.511237, -0.116364, -0.199981));
		}
		
		
		if (!updateUntilInput(robot, timer, redis_client)) {
			std::cerr << "Error getting input from console!" << std::endl;
			stop(0);
		}

		// Print out end-effector pose
		Eigen::Matrix3d rotation;
		robot->rotation(rotation, "end-effector");
		Eigen::Quaterniond quat = Eigen::Quaterniond(rotation);
		std::cout << quat.w() << ", " << quat.x() << ", " << quat.y() << ", " << quat.z() << std::endl;
		Eigen::Vector3d position;
		robot->position(position, "end-effector", Eigen::Vector3d(0.075, 0, 0.02));
		std::cout << position.x() << ", " << position.y() << ", " << position.z() << std::endl;
		/*if (i < 2) {
			position.z() -= 0.015;
		}*/
		calibration.corners.push_back(position);
		
	}

	return calibrate_workspace(calibration);
}

void mirrorHapticDevice(Model::ModelInterface *robot, LoopTimer& timer, RedisClient& redis_client, Workspace workspace) {
	setVelocitySaturation(0.2, redis_client);

	// Declare control variables
	Eigen::VectorXd x_des(Puma::SIZE_OP_SPACE_TASK);
	Eigen::Vector3d ee_pos_des(0.7, 0.4, 0.0);
	
	Eigen::Quaterniond ee_ori_des(0.684764, -0.013204, 0.728463, 0.0163064);
	Eigen::Matrix3d ee_rot_des(ee_ori_des);
	// TODO: make this account for orientation
	Eigen::Vector3d ee_to_marker = Eigen::Vector3d(0.02, 0, -0.075);

	Eigen::VectorXd Kp(Puma::DOF);
	Eigen::VectorXd Kv(Puma::DOF);
	Kp.fill(200);
	Kp(1) = 300;
	Kv.fill(40);
	// Kv(1) = 20;

	// When setting new gains, must change control mode simultaneously.
	x_des << ee_pos_des, ee_ori_des.w(), ee_ori_des.x(), ee_ori_des.y(), ee_ori_des.z();
	redis_client.mset({
		{Puma::KEY_CONTROL_MODE, "GOTO"},
		{Puma::KEY_COMMAND_DATA, RedisClient::encodeEigenMatrixString(x_des)},
		{Puma::KEY_KP, RedisClient::encodeEigenMatrixString(Kp)},
		{Puma::KEY_KV, RedisClient::encodeEigenMatrixString(Kv)}
	});

	Eigen::Vector3d hapticDevicePos;
	// Control loop
	while (g_runloop) {
		// Wait for next scheduled loop (controller must run at precise rate)
		timer.waitForNextLoop();
		// Read from Redis current sensor values and update the model
		updateRobot(robot, redis_client);

		// Read from the haptic device
		redis_client.getEigenMatrixDerivedString("position", hapticDevicePos);

		// Calculate the end-effector target position
		//ee_pos_des = workspace.mapFromHapticDevice(hapticDevicePos);
		ee_pos_des = workspace.mapFromHapticDevice(hapticDevicePos) - ee_to_marker;
		//ee_pos_des = workspace.mapFromHapticDevice(Eigen::Vector3d::Zero());

		// Send desired position for visualization
		
		redis_client.setEigenMatrixString(Puma::KEY_EE_POS + "_des", ee_pos_des);


		// Send command to Puma
		x_des << ee_pos_des, ee_ori_des.w(), ee_ori_des.x(), ee_ori_des.y(), ee_ori_des.z();
		redis_client.setEigenMatrixString(Puma::KEY_COMMAND_DATA, x_des);
	}

}

std::vector< std::vector<std::string> >  loadFromCSV( const std::string& filename )
{
    std::ifstream       file( filename.c_str() );
    std::vector< std::vector<std::string> >   matrix;
    std::vector<std::string>   row;
    std::string                line;
    std::string                cell;

    while( file )
    {
        std::getline(file,line);
        std::stringstream lineStream(line);
        row.clear();

        while( std::getline( lineStream, cell, ',' ) )
            row.push_back( cell );

        if( !row.empty() )
            matrix.push_back( row );
    }

    /*
    for( int i=0; i<int(matrix.size()); i++ )
    {
        for( int j=0; j<int(matrix[i].size()); j++ )
            std::cout << matrix[i][j] << " ";

        std::cout << std::endl;
    }
    */

    return matrix;
}



void autoWrite(Model::ModelInterface *robot, LoopTimer& timer, RedisClient& redis_client, Workspace workspace) {
	setVelocitySaturation(0.15, redis_client);

	std::vector<std::vector<std::string>>  autoWriteCoords; 
	autoWriteCoords = loadFromCSV("/home/group1/Desktop/writernn/writingcoordinates2d.csv");
	cout << autoWriteCoords[0][0];
	int counter = 0;


	// Declare control variables
	Eigen::VectorXd x_des(Puma::SIZE_OP_SPACE_TASK);
	Eigen::Vector3d ee_pos_des(0.7, 0.4, 0.0);
	Eigen::Quaterniond ee_ori_des(0.684764, -0.013204, 0.728463, 0.0163064);
	Eigen::Matrix3d ee_rot_des(ee_ori_des);
	// TODO: make this account for orientation
	Eigen::Vector3d ee_to_marker = Eigen::Vector3d(0.02, 0, -0.062);
	Eigen::Vector3d ee_current_pos(0.0, 0.0, 0.0);

	Eigen::VectorXd Kp(Puma::DOF);
	Eigen::VectorXd Kv(Puma::DOF);
	Kp.fill(200);
	Kp(1) = 300;
	Kv.fill(40);
	// Kv(1) = 20;
	float l2_distance = 0;

	// When setting new gains, must change control mode simultaneously.
	x_des << ee_pos_des, ee_ori_des.w(), ee_ori_des.x(), ee_ori_des.y(), ee_ori_des.z();
	redis_client.mset({
		{Puma::KEY_CONTROL_MODE, "GOTO"},
		{Puma::KEY_COMMAND_DATA, RedisClient::encodeEigenMatrixString(x_des)},
		{Puma::KEY_KP, RedisClient::encodeEigenMatrixString(Kp)},
		{Puma::KEY_KV, RedisClient::encodeEigenMatrixString(Kv)}
	});

	Eigen::Vector3d hapticDevicePos;
	// Control loop
	while (g_runloop) {
		// Wait for next scheduled loop (controller must run at precise rate)
		timer.waitForNextLoop();
		// Read from Redis current sensor values and update the model
		updateRobot(robot, redis_client);

		// Read from the haptic device
		redis_client.getEigenMatrixDerivedString("position", hapticDevicePos);

		float x = std::atof(autoWriteCoords[counter][0].c_str());
		float y = std::atof(autoWriteCoords[counter][1].c_str());
		int moveup = std::atoi(autoWriteCoords[counter][2].c_str());
		Eigen::Vector3d ee_pos_des(x, y, 0);
		ee_pos_des = workspace.mapFromHandwritingGenerator(ee_pos_des) - ee_to_marker;
		cout << "ee updateUntilPosition "<< ee_pos_des.transpose() << endl;

		x_des << ee_pos_des, ee_ori_des.w(), ee_ori_des.x(), ee_ori_des.y(), ee_ori_des.z();
		// Send command to Puma
		redis_client.setEigenMatrixString(Puma::KEY_COMMAND_DATA, x_des);
		updateUntilPosition(robot, timer, redis_client, ee_pos_des, 0.02, 5);

		//puma end effector moves up
		if (moveup == 1) {
			// Send command to Puma
			ee_pos_des[2] = ee_pos_des[2] + 0.02;
			x_des << ee_pos_des, ee_ori_des.w(), ee_ori_des.x(), ee_ori_des.y(), ee_ori_des.z();
			redis_client.setEigenMatrixString(Puma::KEY_COMMAND_DATA, x_des);
			cout << "ee updateUntilPosition "<< ee_pos_des.transpose() << endl;
			updateUntilPosition(robot, timer, redis_client, ee_pos_des, 0.02, 5);
			//cout << "Press enter to write the next character..." << std::endl;
			//updateUntilInput(robot, timer, redis_client);
		}

		// Send desired position for visualization
		redis_client.setEigenMatrixString(Puma::KEY_EE_POS + "_des", ee_pos_des);

		//updateUntilPosition(robot, timer, redis_client, autoWriteCoords[counter] );
		counter = counter + 1; 
		if (counter >= autoWriteCoords.size()) {
			break;
		}
	}

}


int main(int argc, char** argv) {

	cout << "This program is a demo Redis controller for the Puma.1" << endl;

	cout << "Loading URDF world model file: " << kWorldFile << endl;

	// Start redis client
	// Make sure redis-server is running at localhost with default port 6379
	auto redis_client = RedisClient();
	redis_client.serverIs(kRedisServerInfo);

	// Set Ctrl C handler
	setCtrlCHandler(stop);
	g_redis_client = &redis_client;

	// Load robot
	auto robot = new Model::ModelInterface(kRobotFile, Model::rbdl, Model::urdf, false);
	robot->updateModel();

	// Create a loop timer
	// uintmax_t controller_counter = 0;
	const double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	timer.setCtrlCHandler(stop);    // exit while loop on ctrl-c
	timer.initializeTimer(1e6); // 1 ms pause before starting loop

	/***** Hold *****/
	cout << "Starting. HOLD" << endl;
	redis_client.set(Puma::KEY_CONTROL_MODE, "JHOLD");

	/***** Initialize joints *****/
	cout << "Initializing joints. JGOTO" << endl;
	initializeJoints(robot, timer, redis_client);

	/***** Calibrate end-effector *****/
	cout << "Calibrating end-effector. FLOAT" << endl;
	Workspace workspace = calibratePositions(robot, timer, redis_client);

	/***** Mirror haptic device *****/
	cout << "Mirroring haptic device. GOTO" << endl;
	mirrorHapticDevice(robot, timer, redis_client, workspace);

	/***** auto write *****/
	//cout << "autowrite. GOTO" << endl;
	//autoWrite(robot, timer, redis_client, workspace);

	/***** Quit *****/
	redis_client.set(Puma::KEY_CONTROL_MODE, "BREAK");

	return 0;
}


