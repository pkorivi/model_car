#include "MarkerFactory.h"
#include "ObstacleVisualizer.h"

#include <ros/ros.h>
#include <tf/transform_datatypes.h>


namespace autonomos {
namespace visualization {


struct MarkerComparator {
	bool operator()(visualization_msgs::Marker const & m, int i)
	{
		return m.id < i;
	}

	bool operator()(int i, visualization_msgs::Marker const & m)
	{
		return i < m.id;
	}
};


/* ---------------------------------------------------------------------- */
void ObstacleVisualizer::onInit()
{
	mSubscriber = getNodeHandle().subscribe("obstacles", 10, &ObstacleVisualizer::callback, this);
	mMarkerPub = getNodeHandle().advertise<visualization_msgs::MarkerArray>("obstacles_marker_array", 10);
}


/* ---------------------------------------------------------------------- */
void ObstacleVisualizer::callback(autonomos_obstacle_msgs::Obstacles const & obstaclesMsg)
{
	// only process if someone is interested in the output
	if (!isActive()) {
		mVisualizationMarkers.markers.clear();
		return;
	}


	///
	/// set delete state to vanished obstacles
	///
	for (visualization_msgs::Marker & marker : mVisualizationMarkers.markers) {
		// find marker in obstacles by its ID
		auto obstacleIt = std::find_if(obstaclesMsg.obstacles.begin(), obstaclesMsg.obstacles.end(), [& marker](autonomos_obstacle_msgs::Obstacle const & obstacle) {
			return obstacle.id == marker.id;
		});

		// set DELETE action if not found
		if (obstacleIt == obstaclesMsg.obstacles.end()) {
			marker.action = visualization_msgs::Marker::DELETE;
		}
	}


	///
	/// add markers for new obstacles, or update them
	///
	for (autonomos_obstacle_msgs::Obstacle const & obstacle : obstaclesMsg.obstacles) {

		auto matchingMarkers = std::equal_range(mVisualizationMarkers.markers.begin(), mVisualizationMarkers.markers.end(), obstacle.id, MarkerComparator());

		// add new markers, if no correspondence found
		if (matchingMarkers.first == matchingMarkers.second) {
			matchingMarkers = createMarkersForObstacle(obstacle);
		}

		// update all markers corresponding to the obstacle, dependent on their type
		for (auto markerIt = matchingMarkers.first; markerIt != matchingMarkers.second; ++markerIt) {
			visualization_msgs::Marker & marker = *markerIt;

			// general update (independent of the type)
			marker.header = obstacle.header;


			switch (marker.type) {
			case visualization_msgs::Marker::CUBE:
			case visualization_msgs::Marker::SPHERE:
				if (marker.ns.compare("reference_point") == 0) {
					MarkerFactory::updateReferencePointMarker(marker, obstacle);
				}
				else {
					MarkerFactory::updateBoxMarker(marker, obstacle);
				}
				break;
			case visualization_msgs::Marker::LINE_STRIP:
				MarkerFactory::updateContourMarker(marker, obstacle);
				break;
			case visualization_msgs::Marker::TEXT_VIEW_FACING:
				if (marker.ns.compare("debug_text") == 0) {
					MarkerFactory::updateDebugTextMarker(marker, obstacle);
				}
				else if (marker.ns.compare("ID") == 0) {
					MarkerFactory::updateIDMarker(marker, obstacle);
				}
				else {
					ROS_WARN_THROTTLE(1, "Unrecognized marker type for ID: %d.", marker.id);
				}
				break;
			case visualization_msgs::Marker::ARROW:
				MarkerFactory::updateVelocityMarker(marker, obstacle);
				break;
			default:
				ROS_WARN_THROTTLE(1, "Unrecognized marker type for ID: %d.", marker.id);
			}
		}
	}


	///
	/// Send updated markers
	///
	mMarkerPub.publish(mVisualizationMarkers);


	///
	/// cleanup added obstacles for next callback
	///
	for (visualization_msgs::Marker & marker : mVisualizationMarkers.markers) {
		if (marker.action == visualization_msgs::Marker::ADD) {
			marker.action = visualization_msgs::Marker::MODIFY;
		}
	}


	///
	/// cleanup vanished obstacles for next callback
	///
	auto newMarkerEnd = std::remove_if(mVisualizationMarkers.markers.begin(), mVisualizationMarkers.markers.end(), [](visualization_msgs::Marker const & marker) {
		return marker.action == visualization_msgs::Marker::DELETE;
	});
	mVisualizationMarkers.markers.erase(newMarkerEnd, mVisualizationMarkers.markers.end());
}


/* ---------------------------------------------------------------------- */
std::pair<std::vector<visualization_msgs::Marker>::iterator, std::vector<visualization_msgs::Marker>::iterator>
ObstacleVisualizer::createMarkersForObstacle(autonomos_obstacle_msgs::Obstacle const & obstacle)
{
	const size_t initialSize = mVisualizationMarkers.markers.size();


	// bounding box
	visualization_msgs::Marker markerBox;
	MarkerFactory::createBoxMarker(markerBox, obstacle);

	// contour
	visualization_msgs::Marker markerContour;
	MarkerFactory::createContourMarker(markerContour, obstacle);

	// ID
	visualization_msgs::Marker markerID;
	MarkerFactory::createIDMarker(markerID, obstacle);

	// absolute velocity
	visualization_msgs::Marker markerVelocity;
	MarkerFactory::createVelocityMarker(markerVelocity, obstacle);

	// debug text
	visualization_msgs::Marker markerDebugText;
	MarkerFactory::createDebugTextMarker(markerDebugText, obstacle);

	// reference point
	visualization_msgs::Marker markerReferencePoint;
	MarkerFactory::createReferencPointMarker(markerReferencePoint, obstacle);


	// add new markers to message (order is reversed here), keep the sorting by ID
	auto upperBound = std::upper_bound(mVisualizationMarkers.markers.begin(), mVisualizationMarkers.markers.end(), obstacle.id, MarkerComparator());
	upperBound = mVisualizationMarkers.markers.insert(upperBound, markerBox);
	upperBound = mVisualizationMarkers.markers.insert(upperBound, markerContour);
	upperBound = mVisualizationMarkers.markers.insert(upperBound, markerID);
	upperBound = mVisualizationMarkers.markers.insert(upperBound, markerVelocity);
	upperBound = mVisualizationMarkers.markers.insert(upperBound, markerDebugText);
	upperBound = mVisualizationMarkers.markers.insert(upperBound, markerReferencePoint);

	const size_t finalSize = mVisualizationMarkers.markers.size();


	// return range of newly added markers
	std::pair<std::vector<visualization_msgs::Marker>::iterator, std::vector<visualization_msgs::Marker>::iterator> pair;
	pair.first = upperBound;
	pair.second = upperBound + (finalSize - initialSize);
	return pair;
}


/* ---------------------------------------------------------------------- */
bool ObstacleVisualizer::isActive() const
{
	return mMarkerPub.getNumSubscribers() > 0;
}


}
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autonomos::visualization::ObstacleVisualizer, nodelet::Nodelet);
