#include "../include/Himm.h"

/**
 * Ctor.
 * @grid Reference to the grid in which the hmmi
 * will read and write values.
 */
Himm::Himm(OccupancyGrid* grid){
	this->_grid = grid;
}

void Himm::ToFile(std::string filename){
	this->_grid->ToFile(filename);
}

Himm::~Himm(){}

/**
 * Set a value to the grid according to the position of the robot
 * @param pose A Pose2D struct representing the position and rotation of the robot
 * @param dist The distance captured by the sensor
 * @param theta The inclination of the laser ray during the reading in degrees
 * @returns The value written to the cell
 */
OGCellType Himm::UpdateLocation(Pose2D pose, OGCellType dist, OGCellType theta){

	// Defines odom_theta (robot odometry) between 0 <-> 2PI
	double odom_theta;
	if(pose.theta < 0)
		odom_theta = (2 * M_PI) - std::abs(pose.theta);
	else 
		odom_theta = pose.theta;

	// Defines HOKUYO laser angle between 0 <-> 2PI
	double calc_theta;
	if(theta < 0)
		calc_theta = (2 * M_PI) - std::abs(theta);
	else
		calc_theta = theta;

	//angle correction (neg to pos rad values)
	double pivot_ang;
	pivot_ang = (HOKUYO_ANGLE_MAX - HOKUYO_ANGLE_MIN) / 2;
	
	if (calc_theta > pivot_ang)
		odom_theta -= calc_theta;
	else
		odom_theta += calc_theta;
	
	// while(odom_theta < 0)
	// 	odom_theta  = (2 * M_PI) - std::abs(odom_theta);

	// OGCellType wDist = cos(odom_theta) * dist; //Hokuyo laser horizontal distance
	// OGCellType hDist = sin(odom_theta) * dist; //Hokuyo laser vertical distance

	OGCellType wDist = cos(calc_theta) * dist; //Hokuyo laser horizontal distance
	OGCellType hDist = sin(calc_theta) * dist; //Hokuyo laser vertical distance	

	int x_coord = (pose.x + wDist) * UNIT_FIX;
	int y_coord = (pose.y + hDist) * UNIT_FIX;

	//cria vetor na origem com o mesmo comprimento do vetor 
	//gerado pelo laser (que tem origem no robô)
	Vector2D vecLaser;
	vecLaser.x = x_coord - pose.x;
	vecLaser.y = y_coord - pose.y;

	//gera o vetor unitário do vetor criado (que tem mesmo comprimento
	//e direção do vetor unitário do laser)
	Vector2D vecUnitary = ~vecLaser;

	//cria o vetor da posição do robô
	Vector2D vecRobot;
	vecRobot.x = pose.x;
	vecRobot.y = pose.y;
	
	//cria um vetor de tamanho unitário com origem
	//na coordenada do robô. Incrementa este vetor 
	//em 1 (comprimento) enquanto ele não atingir 
	//a coordenada encontrada pelo laser, passando
	//pelas coordenadas no caminho entre o robô e o
	//alvo encontrado pelo laser.
	Vector2D vecIncrement;
	vecIncrement = vecRobot; //posev é o centro do robô
	
	while(!vecIncrement < !vecLaser){
		_grid->Set(round(vecIncrement.x), round(vecIncrement.y), -1);
		vecIncrement = (vecIncrement + vecUnitary);
	}

	//set location with target increment
	_grid->Set(x_coord, y_coord, 1);
}

