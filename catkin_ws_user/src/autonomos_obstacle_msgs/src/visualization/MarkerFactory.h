#pragma once

//#include <autonomos_common_types/Units.h>
#include <autonomos_obstacle_msgs/Obstacles.h>

#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include <math.h>


namespace autonomos {
namespace visualization {


/** Static object factory that creates or updates visualization_msgs::Marker objects to reflect the state of their corresponding obstacle.
 **
 ** @ingroup @@
 */
class MarkerFactory
{
public:
	/**
	 * Initializes a marker for the obstacles's predicted bounding box.
	 *
	 * @see MarkerFactory::updateBoxMarker
	 *
	 *	* color indicates the obstacle's class
	 *	* alpha indicates the obstacle's prediction age
	 *	* sphere-type indicates no valid bounding box (zero-values)
	 *
	 * @param marker[out] The visualization marker belonging to the obstacle (expected to be uninitialized)
	 * @param obstacle[in] The obstacle belonging to the visualization marker
	 */
	static void createBoxMarker(visualization_msgs::Marker & marker, autonomos_obstacle_msgs::Obstacle const & obstacle)
	{
		marker.id       = obstacle.id;
		marker.action   = visualization_msgs::Marker::ADD;
		marker.lifetime = ros::Duration(.3);
		marker.type     = visualization_msgs::Marker::CUBE;
	}


	/**
	 * Updates the marker for the obstacles's predicted bounding box.
	 *
	 * @param marker[out] The visualization marker belonging to the obstacle (expected to be initialized with the corresponding create... method)
	 * @param obstacle[in] The obstacle belonging to the visualization marker
	 */
	static void updateBoxMarker(visualization_msgs::Marker & marker, autonomos_obstacle_msgs::Obstacle const & obstacle)
	{
		// set color based on the classification (classification can change over time!)
		switch (obstacle.classification) {
		case autonomos_obstacle_msgs::Obstacle::UNCLASSIFIED:
			marker.ns      = "unclassified (red)";
			marker.color.r = 1.;
			marker.color.g = 0.;
			marker.color.b = 0.;
			break;
		case autonomos_obstacle_msgs::Obstacle::UNKNOWN_SMALL:
			marker.ns      = "unknown small (yellow, small)";
			marker.color.r = 1.;
			marker.color.g = 1.;
			marker.color.b = 0.;
			break;
		case autonomos_obstacle_msgs::Obstacle::UNKNOWN_BIG:
			marker.ns      = "unknown big (yellow, big)";
			marker.color.r = 1.;
			marker.color.g = 1.;
			marker.color.b = 0.;
			break;
		case autonomos_obstacle_msgs::Obstacle::PEDESTRIAN:
			marker.ns      = "pedestrian (blue)";
			marker.color.r = 0.;
			marker.color.g = 0.;
			marker.color.b = 1.;
			break;
		case autonomos_obstacle_msgs::Obstacle::BIKE:
			marker.ns      = "bike (cyan)";
			marker.color.r = 0.;
			marker.color.g = 1.;
			marker.color.b = 1.;
			break;
		case autonomos_obstacle_msgs::Obstacle::CAR:
			marker.ns      = "car (orange, small)";
			marker.color.r = 1.0;
			marker.color.g = 0.5;
			marker.color.b = 0.;
			break;
		case autonomos_obstacle_msgs::Obstacle::TRUCK:
			marker.ns      = "truck (orange, big)";
			marker.color.r = 1.0;
			marker.color.g = 0.5;
			marker.color.b = 0.;
			break;
		case autonomos_obstacle_msgs::Obstacle::ELEVATED:
			marker.ns      = "elevated (grey)";
			marker.color.r = 0.7;
			marker.color.g = 0.7;
			marker.color.b = 0.7;
			break;
		default:
			marker.ns      = "unhandled (white)";
			marker.color.r = 1.;
			marker.color.g = 1.;
			marker.color.b = 1.;
			break;
		}

		// set obstacle position
		/*
		 * The marker's position needs to be the center of its bounding box (set by scale). But the obstacles bounding box center is only
		 * implicitly given by min, max, which are relative to the obstacles pose. Hence, we need to add the rotated offset (between
		 * obstacle position and bounding box center) to the obstacle position.
		 */
		tf::Point boundingBoxMin(obstacle.bounding_box_min.x, obstacle.bounding_box_min.y, obstacle.bounding_box_min.z);
		tf::Point boundingBoxMax(obstacle.bounding_box_max.x, obstacle.bounding_box_max.y, obstacle.bounding_box_max.z);
		tf::Vector3 boundingBoxSize = boundingBoxMax - boundingBoxMin;
		tf::Vector3 obstPositionToBoxCenter = (boundingBoxMin + boundingBoxMax) / 2;
		tf::Quaternion q;
		tf::quaternionMsgToTF(obstacle.odom.pose.pose.orientation, q);
		obstPositionToBoxCenter = tf::quatRotate(q, obstPositionToBoxCenter);
		tf::Point boundingBoxCenter(obstacle.odom.pose.pose.position.x, obstacle.odom.pose.pose.position.y, obstacle.odom.pose.pose.position.z);
		boundingBoxCenter += obstPositionToBoxCenter;

		marker.pose.position.x = boundingBoxCenter.x();
		marker.pose.position.y = boundingBoxCenter.y();
		marker.pose.position.z = boundingBoxCenter.z();


		// set obstacle orientation
		marker.pose.orientation = obstacle.odom.pose.pose.orientation;

		// set obstacle box size
		// sometimes obstacles have a width/length of zero, visualize this by using a different marker type
		if (boundingBoxSize.isZero()) {
			marker.type = visualization_msgs::Marker::SPHERE;
			marker.scale.x = 1;
			marker.scale.y = 1;
			marker.scale.z = 1;
		}
		else {
			marker.type = visualization_msgs::Marker::CUBE;
			marker.scale.x = std::max(0.1, boundingBoxSize.x());
			marker.scale.y = std::max(0.1, boundingBoxSize.y());
			marker.scale.z = std::max(0.1, boundingBoxSize.z());
		}



		// fade the obstacle according to its prediction age
		float predictionAge = (obstacle.header.stamp - obstacle.last_observed).toSec();
		marker.color.a = std::max(0.3, 1.0 - predictionAge / 10.f);

	}


	/**
	 * Initializes a marker for the obstacle's predicted contour.
	 *
	 * @see MarkerFactory::updateContourMarker
	 *
	 * @param marker[out] The visualization marker belonging to the obstacle (expected to be uninitialized)
	 * @param obstacle[in] The obstacle belonging to the visualization marker
	 */
	static void createContourMarker(visualization_msgs::Marker & marker, autonomos_obstacle_msgs::Obstacle const & obstacle)
	{
		marker.id       = obstacle.id;
		marker.type     = visualization_msgs::Marker::LINE_STRIP;
		marker.action   = visualization_msgs::Marker::ADD;
		marker.lifetime = ros::Duration(.3);
		marker.ns       = "contour";

		marker.color.r = 1.;
		marker.color.g = 1.;
		marker.color.b = 1.;
		marker.color.a = 1.;

		marker.scale.x = .1; // width of line segments in m
	}


	/**
	 * Updates the marker for the obstacle's predicted contour.
	 *
	 * @param marker[out] The visualization marker belonging to the obstacle (expected to be initialized with the corresponding create... method)
	 * @param obstacle[in] The obstacle belonging to the visualization marker
	 */
	static void updateContourMarker(visualization_msgs::Marker & marker, autonomos_obstacle_msgs::Obstacle const & obstacle)
	{
		marker.points.clear();

		// add points - geometry_msgs::Point32 -> geometry_msgs::Point
		for (geometry_msgs::Point32 const & point : obstacle.contour_points) {
			geometry_msgs::Point p;
			p.x = point.x;
			p.y = point.y;
			p.z = point.z;
			marker.points.push_back(p);
		}
	}


	/**
	 * Initializes a marker for the obstacle's ID.
	 *
	 * @see MarkerFactory::updateIDMarker
	 *
	 * @param marker[out] The visualization marker belonging to the obstacle (expected to be uninitialized)
	 * @param obstacle[in] The obstacle belonging to the visualization marker
	 */
	static void createIDMarker(visualization_msgs::Marker & marker, autonomos_obstacle_msgs::Obstacle const & obstacle)
	{
		marker.id              = obstacle.id;
		marker.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
		marker.action          = visualization_msgs::Marker::ADD;
		marker.lifetime        = ros::Duration(.3);
		marker.ns              = "ID";
		marker.text            = "[" + std::to_string((int)obstacle.id) + "]";
		marker.scale.z         = .3; // text size
		marker.color.r         = 0.6;
		marker.color.g         = 0.7;
		marker.color.b         = 1.0;
		marker.color.a         = 1.;
	}


	/**
	 * Updates the marker for the obstacle's ID.
	 *
	 * @param marker The visualization marker belonging to the obstacle (expected to be initialized with the corresponding create... method)
	 * @param obstacle The obstacle belonging to the visualization marker
	 */
	static void updateIDMarker(visualization_msgs::Marker & marker, autonomos_obstacle_msgs::Obstacle const & obstacle)
	{
		marker.pose = obstacle.odom.pose.pose;
		marker.pose.position.z += obstacle.bounding_box_max.z + 0.3;
	}


	/**
	 * Initializes a marker for the obstacle's predicted absolute velocity.
	 *
	 * @see MarkerFactory::updateVelocityMarker
	 *
	 *	* color indicates the velocity prediction quality (green=good, red=bad)
	 *	* alpha indicates the velocity prediction quality (visible=good, transparent=bad)
	 *	* arrow length-indicates the absolute velocity
	 *
	 * @param marker[out] The visualization marker belonging to the obstacle (expected to be uninitialized)
	 * @param obstacle[in] The obstacle belonging to the visualization marker
	 */
	static void createVelocityMarker(visualization_msgs::Marker & marker, autonomos_obstacle_msgs::Obstacle const & obstacle)
	{
		marker.id              = obstacle.id;
		marker.type            = visualization_msgs::Marker::ARROW;
		marker.action          = visualization_msgs::Marker::ADD;
		marker.lifetime        = ros::Duration(.3);
		marker.ns              = "velocity prediction";
		marker.pose.position.z = 0.25;
		marker.scale.x         = 1;
		marker.scale.y         = .1;
		marker.scale.z         = .1;
		marker.color.r         = 0.1;
		marker.color.g         = 0.8;
		marker.color.b         = 0.1;
		marker.color.a         = 1.;
	}


	/**
	 * Updates the marker for the obstacle's predicted absolute velocity.
	 *
	 * @param marker[out] The visualization marker belonging to the obstacle (expected to be initialized with the corresponding create... method)
	 * @param obstacle[in] The obstacle belonging to the visualization marker
	 */
	static void updateVelocityMarker(visualization_msgs::Marker & marker, autonomos_obstacle_msgs::Obstacle const & obstacle)
	{
		// hide marker if the absolute velocity is NaN
		if (std::isnan(obstacle.abs_velocity.twist.linear.x) or std::isnan(obstacle.abs_velocity.twist.linear.y)) {
			marker.color.a = 0;
			return;
		}


		// set orientation
		//TODO pkorivi changed Radian to double, radians to 180/M_PI
		double velocityDirection = atan2(obstacle.abs_velocity.twist.linear.y, obstacle.abs_velocity.twist.linear.x) * 180/M_PI;
		tf::Quaternion q;
		//q.setEuler(0, 0, velocityDirection.value());
		q.setEuler(0, 0, velocityDirection);
		marker.pose.orientation.x = q.getX();
		marker.pose.orientation.y = q.getY();
		marker.pose.orientation.z = q.getZ();
		marker.pose.orientation.w = q.getW();

		// set scale
		//MPS velocity = sqrt((obstacle.abs_velocity.twist.linear.x * obstacle.abs_velocity.twist.linear.x) + (obstacle.abs_velocity.twist.linear.y * obstacle.abs_velocity.twist.linear.y)) * meters / seconds;
		//TODO - pkorivi changed MPS to double and meter/seconds to 5/18
		double velocity = sqrt((obstacle.abs_velocity.twist.linear.x * obstacle.abs_velocity.twist.linear.x) + (obstacle.abs_velocity.twist.linear.y * obstacle.abs_velocity.twist.linear.y)) * 5 / 18;
		// scale down the marker, we dont want 30m long arrows for the velocity of 30m/s
		//TODO pkorivi chnaged velocity.value() to velocity
		marker.scale.x = std::max(0.0000001, sqrt(velocity) * 0.5);
		//if (velocity <= 0 * meters / seconds) {
		if (velocity <= 0) {
			marker.color.a = 0.;
			return;
		}

		// set alpha and color by (relative) velocity sigma
		/// This assumes that absolute_velocity_sigma is absolute. Accordingly, the sigma needs to be relative to the velocity (sigma of 1 at speed of 1m/s is far more uncertain than at 100m/s)
		float avgVelocitySigma = 0.5 * (sqrt(sqrt(obstacle.abs_velocity.covariance[0])) + sqrt(sqrt(obstacle.abs_velocity.covariance[7])));
		//float relativeMeanUncertainty = avgVelocitySigma / ((velocity.value() > 1) ? velocity.value() : 1.0);
		//TODO - pkorivi chnaged
		float relativeMeanUncertainty = avgVelocitySigma / ((velocity > 1) ? velocity : 1.0);
		/// This is an empirical und subjective scaling, tuned at a point where the obstacle velocity was not yet used. TODO: update this indicator to your needs.
		double uncertaintyIndication = 2 * relativeMeanUncertainty;
		marker.color.r = std::min(0.8, uncertaintyIndication);
		marker.color.g = std::max(0.2, 1.0 - uncertaintyIndication);
		marker.color.a = std::max(0.2, 1.0 - uncertaintyIndication);

		// set position (attach the arrow to the center front of the obstacle - obstacle are expected to be oriented towards their movement direction)
		tf::quaternionMsgToTF(obstacle.odom.pose.pose.orientation, q);
		tf::Vector3 centerToBoundingBoxFront(obstacle.bounding_box_max.x, (obstacle.bounding_box_max.y + obstacle.bounding_box_min.y) / 2, 0);
		centerToBoundingBoxFront = tf::quatRotate(q, centerToBoundingBoxFront);
		marker.pose.position.x = obstacle.odom.pose.pose.position.x + centerToBoundingBoxFront.x();
		marker.pose.position.y = obstacle.odom.pose.pose.position.y + centerToBoundingBoxFront.y();
		marker.pose.position.z = 0.2;
	}


	/**
	 * Initializes a marker for some of the obstacles's properties.
	 *
	 * @see MarkerFactory::updateDebugTextMarker
	 *
	 * @param marker[out] The visualization marker belonging to the obstacle (expected to be uninitialized)
	 * @param obstacle[in] The obstacle belonging to the visualization marker
	 */
	static void createDebugTextMarker(visualization_msgs::Marker & marker, autonomos_obstacle_msgs::Obstacle const & obstacle)
	{
		marker.id              = obstacle.id;
		marker.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
		marker.action          = visualization_msgs::Marker::ADD;
		marker.lifetime        = ros::Duration(.3);
		marker.ns              = "debug_text";
		marker.scale.z         = .3; // text size
		marker.color.r         = 0.2;
		marker.color.g         = 1.;
		marker.color.b         = 0.2;
		marker.color.a         = 1.;
	}


	/**
	 * Updates the marker for some of the obstacle's properties.
	 *
	 *
	 * @param marker[out] The visualization marker belonging to the obstacle (expected to be initialized with the corresponding create... method)
	 * @param obstacle[in] The obstacle belonging to the visualization marker
	 */
	static void updateDebugTextMarker(visualization_msgs::Marker & marker, autonomos_obstacle_msgs::Obstacle const & obstacle)
	{
		marker.pose = obstacle.odom.pose.pose;
		marker.pose.position.z = 2.6;


		std::ostringstream out;
		out << std::setprecision(3);
		out << "last obs:   "   << (ros::Time::now() - obstacle.last_observed).toSec() << "s ago\n";
		out << "first obs:   "  << (ros::Time::now() - obstacle.first_observed).toSec() << "s ago\n";
		out << "class cert:  "  << obstacle.classification_certainty  << "\n";
		out << "pos var:    ["  << sqrt(obstacle.odom.pose.covariance[0]) << "," << sqrt(obstacle.odom.pose.covariance[7]) << "," << sqrt(obstacle.odom.pose.covariance[14]) << "]\n";
		out << "orient var:  [" << sqrt(obstacle.odom.pose.covariance[21]) << "," << sqrt(obstacle.odom.pose.covariance[28]) << "," << sqrt(obstacle.odom.pose.covariance[35]) << "]\n";
		out << "rel vel:     [" << obstacle.odom.twist.twist.linear.x << "," << obstacle.odom.twist.twist.linear.y << "," << obstacle.odom.twist.twist.linear.z << "]\n";
		out << "abs vel:    ["  << obstacle.abs_velocity.twist.linear.x << "," << obstacle.abs_velocity.twist.linear.y << "," << obstacle.abs_velocity.twist.linear.z << "]\n";
		out << "abs vel var: [" << sqrt(obstacle.abs_velocity.covariance[0]) << "," << sqrt(obstacle.abs_velocity.covariance[7]) << "," << sqrt(obstacle.abs_velocity.covariance[14]) << "]\n";

		marker.text = out.str();
	}


	/**
	 * Initializes a marker for the obstacle's reference point.
	 *
	 * @see MarkerFactory::updateReferencePointMarker
	 *
	 * @param marker[out] The visualization marker belonging to the obstacle (expected to be uninitialized)
	 * @param obstacle[in] The obstacle belonging to the visualization marker
	 */
	static void createReferencPointMarker(visualization_msgs::Marker & marker, autonomos_obstacle_msgs::Obstacle const & obstacle)
	{
		marker.id              = obstacle.id;
		marker.type            = visualization_msgs::Marker::SPHERE;
		marker.action          = visualization_msgs::Marker::ADD;
		marker.lifetime        = ros::Duration(.3);
		marker.ns              = "reference_point";
		marker.scale.x         = .2;
		marker.scale.y         = .2;
		marker.scale.z         = .2;
		marker.color.r         = 1.0;
		marker.color.g         = 0.2;
		marker.color.b         = 0.2;
		marker.color.a         = 1.;
	}


	/**
	 * Updates the marker for the obstacle's reference_point_sigma.
	 *
	 * @param marker[out] The visualization marker belonging to the obstacle (expected to be initialized with the corresponding create... method)
	 * @param obstacle[in] The obstacle belonging to the visualization marker
	 */
	static void updateReferencePointMarker(visualization_msgs::Marker & marker, autonomos_obstacle_msgs::Obstacle const & obstacle)
	{

		// hide marker if the reference point is NaN
		if (std::isnan(obstacle.reference_point.x) or std::isnan(obstacle.reference_point.y)) {
			marker.color.a = 0;
			return;
		}

		marker.pose.position.x = obstacle.reference_point.x;
		marker.pose.position.y = obstacle.reference_point.y;
		marker.pose.position.z = obstacle.reference_point.z;

		// set alpha by reference_point standard deviation
		double avgSigma = (obstacle.reference_point_sigma.x + obstacle.reference_point_sigma.y + obstacle.reference_point_sigma.z) / 3;
		marker.color.a = std::min(1.0, std::max(0.3, 1 - avgSigma / 10));
	}


private:
	MarkerFactory() {}
};

}
}
