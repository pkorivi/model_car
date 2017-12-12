/*
 */
#ifndef MOTION_PLANNER_H_
#define MOTION_PLANNER_H_
#include <nodelet/nodelet.h>
#include "VehicleState.h"
#include "VehiclePath.h"

namespace fub_motion_planner{
  class MotionPlanner : public nodelet::Nodelet{
    public:
        MotionPlanner();
        ~MotionPlanner();
        virtual void onInit();
    protected:
      VehicleState m_vehicle_state;
      VehiclePath m_vehicle_path;
      //motion planner trajectory output
      ros::Publisher m_mp_traj;
      void create_traj(VehicleState current_state);
      /** The callback for the timer that triggers the update.
      */
      void callbackTimer(const ros::TimerEvent&);
      // timer triggering our execution // TODO: use WallTimer?
      ros::Timer m_timer;
  };
} // namespace sample_nodelet_ns

#endif /*  */
