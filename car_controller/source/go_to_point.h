/*
 * go_to_point_controller.h
 *
 *  Created on: Jul 28, 2019
 *      Author: Devin
 */

#ifndef GO_TO_POINT_H_
#define GO_TO_POINT_H_

#include <stdint.h>
#include "inertial_states.h"

typedef struct {
   float x;
   float y;
} Destination_T;

class GoToPointController
{
private:
   float  tol;
   float  kp;
   bool   in_route = false;
   float  heading_sp;
   float  d_sp;
   Pose_T pose_fb = {0};
   float  robot_v;
public:
   void Init(float tolerance, float gain, float travel_s);
   void Execute(void);
   void Update_Destination(Destination_T * dest);
   bool In_Route(void);
   void Set_Travel_Speed(float robot_v);
};

#endif /* GO_TO_POINT_H_ */
