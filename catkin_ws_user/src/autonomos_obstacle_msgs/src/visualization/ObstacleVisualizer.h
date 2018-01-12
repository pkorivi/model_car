#pragma once

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <autonomos_obstacle_msgs/Obstacles.h>

#include <visualization_msgs/MarkerArray.h>



namespace autonomos {
namespace visualization {


/** Node that visualizes autonomos_obstacle_msgs/Obstacles messages with visualization markers.
 **
 ** @ingroup @@
 */
class ObstacleVisualizer : public nodelet::Nodelet
{
public:
	ObstacleVisualizer() {}
	virtual ~ObstacleVisualizer() {}

	virtual void onInit() override;

protected:

	/**
	 * Visualizes the obstacles for debugging purposes. Currently we visualize bounding box (box), contour (line), absolute velocity (arrow), ID (text) and debug information (text).
	 *
	 * @param obstaclesMsg[in] The obstacles to visualize
	 */
	void callback(autonomos_obstacle_msgs::Obstacles const & obstaclesMsg);

	/**
	 * Creates multiple markers for the given obstacle and adds them to mVisualizationMarkers while preserving sort order.
	 *
	 * @param obstacle[in] The obstacle to visualize
	 * @return The range of the markers created
	 */
	std::pair<std::vector<visualization_msgs::Marker>::iterator, std::vector<visualization_msgs::Marker>::iterator>
	createMarkersForObstacle(autonomos_obstacle_msgs::Obstacle const & obstacle);

	bool isActive() const;

private:
	/// subscriber on the obstacles to be visualized
	ros::Subscriber mSubscriber;
	/// publisher for visualization markers
	ros::Publisher mMarkerPub;

	/// array of visualization markers (sorted by ID)
	visualization_msgs::MarkerArray mVisualizationMarkers;
};


}
}
