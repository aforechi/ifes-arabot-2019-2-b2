#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <iostream>
#include <chrono>
#include <fstream>
#include <math.h>

std::ofstream Valores("Valores.txt");

using namespace msr::airlib;

bool ChegouNoFinal(const msr::airlib::Pose &pose)
{
	if (pose.position[0] > -5 && pose.position[0] < 5) {
		if (pose.position[1] > 0 && pose.position[1] < 1.0) {
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

void printCarPose(const msr::airlib::Pose &pose, float speed)
{
	std::cout << "x=" << pose.position[0] << " y=" << pose.position[1] << " v=" << speed << std::endl;

}

void saveCarPose(const msr::airlib::Pose &pose, float speed)
{

	Valores << pose.position[0] << ";" << pose.position[1] << ";" << speed << std::endl;

}

void moveForwardAndBackward(msr::airlib::CarRpcLibClient &client)
{
	client.enableApiControl(true);
	CarApiBase::CarControls controls;

	std::cout << "Pressione Enter andar pra frente." << std::endl; std::cin.get();
	controls.throttle = 0.5f;
	controls.steering = 0.0f;
	client.setCarControls(controls);

	std::cout << "Pressione Enter para puxar o freio de mão." << std::endl; std::cin.get();
	controls.handbrake = true;
	client.setCarControls(controls);

	std::cout << "Pressione Enter para fazer uma manobra." << std::endl; std::cin.get();
	controls.handbrake = false;
	controls.throttle = -0.5;
	controls.steering = 1;
	controls.is_manual_gear = true;
	controls.manual_gear = -1;
	client.setCarControls(controls);

	std::cout << "Pressione Enter para parar." << std::endl; std::cin.get();
	client.setCarControls(CarApiBase::CarControls());
}

int main()
{
	std::cout << "Verifique se o arquivo Documentos\\AirSim\\settings.json " <<
		"está configurado para simulador de carros \"SimMode\"=\"Car\". " <<
		"Pressione Enter para continuar." << std::endl;
	std::cin.get();

	msr::airlib::CarRpcLibClient client;
	try {
		client.confirmConnection();
		client.reset();

		msr::airlib::Pose car_poseInitial;
		car_poseInitial.position[0] = 0;
		car_poseInitial.position[1] = 0;
		msr::airlib::Pose car_poseFinal;
		do {
			auto car_state = client.getCarState();
			car_poseFinal = car_state.kinematics_estimated.pose;
			auto car_speed = car_state.speed;
			if (deveSalvarPonto(car_poseInitial, car_poseFinal, 1)) {
				saveCarPose(car_poseInitial, car_speed);
				car_poseInitial = car_poseFinal;
			}
		} while (!ChegouNoFinal(car_poseInitial));

	}
	catch (rpc::rpc_error&  e) {
		std::string msg = e.get_error().as<std::string>();
		std::cout << "Verifique a exceção lançada pela API do AirSim." << std::endl << msg << std::endl; std::cin.get();
	}

	Valores.close();
	return 0;
}