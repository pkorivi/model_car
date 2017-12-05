/*
 * sample_nodelet_class2.h
 *
 *  Created on: 2016/09/18
 *      Author: cryborg21
 */
#ifndef VEHICLE_STATE_NODELET_CLASS2_H_
#define VEHICLE_STATE_NODELET_CLASS2_H_
#include <nodelet/nodelet.h>

namespace fub_motion_planner{
  class Vehicle_state : public nodelet::Nodelet{
    public:
        Vehicle_state();
        ~Vehicle_state();

        virtual void onInit();
  };
} // namespace sample_nodelet_ns

#endif /* SAMPLE_NODELET_CLASS_SRC_SAMPLE_NODELET_CLASS2_H_ */
