#pragma once
#include <iostream>
#include <vector>
#include <fstream>
#include "common/Common.hpp"

typedef msr::airlib::Vector3r Vector3r;

class Waypoints
{
private:
	Eigen::MatrixX3f waypoints; //x,y,v
    size_t current;
public:
    Waypoints() : current(0) {
	}
    
    size_t GetCurrentWaypointIndex() const {
        return current;
    }
    
	void AddWaypoints(float x, float y, float v){
        Vector3r point(x,y,v);
        size_t row = waypoints.rows();
        waypoints.conservativeResize(row+1, Eigen::NoChange);
        waypoints.row(row) = point;
	}

	void SaveWaypoints(std::string filename) const {
		std::ofstream file(filename);
        for(int i=0; i<waypoints.rows(); i++)
			file << waypoints(i,0) << ";" << waypoints(i,1) << ";" << waypoints(i,2) << std::endl;
		file.close();

	}
    
	void LoadWaypoints(std::string filename) {
		std::ifstream file(filename);
		std::string linha, coluna;
        float point[3];
		while (!file.eof()) {
			std::getline(file, linha);
			std::istringstream Colunas(linha);
			while (!Colunas.eof()) {
                for (int i=0; i<3; i++) {
                    std::getline(Colunas, coluna, ';');
                    if(coluna!="")
                        point[i] = atof(coluna.c_str());
                }
                AddWaypoints(point[0],point[1],point[2]);
			}
		}
		file.close();
	}
    
    Eigen::MatrixXf TransformWaypointsWithRespectToCar(const Vector3r &pose) const {
        float theta = -pose[2];
        Eigen::Matrix2f transform;
        transform(0,0) = cos(theta);
        transform(0,1) = -sin(theta);
        transform(1,0) = sin(theta);
        transform(1,1) = cos(theta);
        
        auto xy = waypoints.leftCols(2).eval();
        xy.noalias() = xy.rowwise() - pose.transpose().leftCols(2);
        xy.noalias() = (transform * xy.transpose()).transpose();
        return xy;
    }
    
    Eigen::Vector3f GetWaypoint(int index) const {
        if (index >= 0  && index < waypoints.rows())
            return waypoints.row(index).eval();
        else
            throw std::out_of_range("waypoint index out of range");
    }
    
    float GetWaypointVelocity(const Vector3r &pose) {
        auto pts = TransformWaypointsWithRespectToCar(pose);
                
        auto min_dist = pts.rowwise().norm().minCoeff(&current);
        
        return waypoints(current, 2);
    }
};

