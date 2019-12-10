#pragma once

#include "Waypoints.h"
#include "Interp.h"

class LateralControl 
{
private:
	float baseline;
	float steermax;
	float lookahead;
public:

    LateralControl(float bl, float sm, float la) : baseline(bl), steermax(sm), lookahead(la){
        
    }
   
    float Update(Waypoints &checkpoints, const Vector3r &pose, float velocity){
        float steering(0.0f);
        if (velocity > 0.001f)
        {
            auto nearest_waypoint_index = checkpoints.GetCurrentWaypointIndex();
            auto waypoints = checkpoints.TransformWaypointsWithRespectToCar(pose);
            auto x = waypoints.col(0).segment(nearest_waypoint_index,2).eval();
            auto y = waypoints.col(1).segment(nearest_waypoint_index,2).eval();
            auto coeffs = polyfit(x, y, 1);
            float cte = polyval(coeffs, lookahead);
            
            steering = atan(2.0f * baseline * cte / pow(lookahead, 2.0f));
        }
        steering = std::max(-steermax, std::min(steering, steermax));
        return steering;
    }
    
    ~LateralControl(){};
};

