#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON
#include <stdlib.h>

#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <iostream>
#include <chrono>
#include <fstream>
#include <math.h>
#include <sstream>
#include <cstring>
#include <string>
#include "Waypoints.h"
#include"LateralControl.h"
#include "LongitudinalControl.h"
#include <tchar.h>

using namespace msr::airlib;

bool ChegouNoFinal(const msr::airlib::Pose &pose)
{
	if (pose.position[0] > -3 && pose.position[0] < -1) {
		if (pose.position[1] > -5 && pose.position[1] < 5) {
			return true;
		}
	}
	return false;

}
bool deveSalvarPonto(const msr::airlib::Pose &poseInitial, const msr::airlib::Pose &poseFinal, float intervalo)
{

	float finalXposition = poseFinal.position[0];
	float finalYposition = poseFinal.position[1];
	float initialXposition = poseInitial.position[0];
	float initialYposition = poseInitial.position[1];
	float dist = sqrt((pow((finalXposition - initialXposition), 2) + (pow((finalYposition - initialYposition), 2))));
	if (dist >= intervalo)
		return true;
	return false;
}



void moveInTheTrajectory(msr::airlib::CarRpcLibClient &client, float &acceleration, float &steering) {
	CarApiBase::CarControls controls;
	if (acceleration >= 0)
		controls.throttle = acceleration;
	else
		controls.brake = -acceleration;
	controls.steering = steering;
	client.setCarControls(controls);
}



int main()
{
	_tsetlocale(LC_ALL, _T("portuguese"));

	Waypoints checkpoints, trajectory;
	LateralControl lateral_control(2.3, 1, 4);
	LongitudinalControl velocity_control(1.0, 1.0, 0.01);
	msr::airlib::CarRpcLibClient client;

	int escolhaFeita;
	std::cout << "Escolha uma das opções abaixo:\n";
	std::cout << "[1] Para Salvar manualmente os pontos.\n";
	std::cout << "[2] Para o Carro andar automáticamente.\n";
	std::cin >> escolhaFeita;


	try {
		client.confirmConnection();
		client.reset();

		if (escolhaFeita == 2) {
			checkpoints.LoadWaypoints("Valores.txt");
			client.enableApiControl(true);
		}

		msr::airlib::Pose car_poseInitial;
		car_poseInitial.position[0] = 0;
		car_poseInitial.position[1] = 0;
		msr::airlib::Pose car_poseFinal;

		do {
			auto car_state = client.getCarState();
			auto car_speed = car_state.speed;

			car_poseFinal = car_state.kinematics_estimated.pose;
			if (escolhaFeita == 2) {
				Vector3r pose(car_poseFinal.position[0], car_poseFinal.position[1], VectorMath::yawFromQuaternion(car_poseFinal.orientation));
				double desired_velocity = checkpoints.GetWaypointVelocity(pose);
				float steering = lateral_control.Update(checkpoints, pose, car_speed);
				float acceleration = velocity_control.Update(car_speed, desired_velocity);
				moveInTheTrajectory(client, acceleration, steering);
			}

			if (deveSalvarPonto(car_poseInitial, car_poseFinal, 1.0)) {
				trajectory.AddWaypoints(car_poseFinal.position[0], car_poseFinal.position[1], car_speed);
				car_poseInitial = car_poseFinal;				

			}
		} while (!ChegouNoFinal(car_poseInitial));

		trajectory.SaveWaypoints("Trajetoria.txt");
	}

	catch (rpc::rpc_error&  e) {
		std::string msg = e.get_error().as<std::string>();
		std::cout << "Verifique a exceção lançada pela API do AirSim." << std::endl << msg << std::endl; std::cin.get();
	}
	
	

		return 0;
}