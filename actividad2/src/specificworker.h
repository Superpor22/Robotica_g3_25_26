/*
*    Copyright (C) 2025 by G3 {Guadalupe González Santos, Máximo Bueno Martínez & José Antonio Bravo Romero}
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @brief SpecificWorker class implements the main functionality of the component.
 *
 * This class handles the robot’s behavior using a state machine, processes 3D Lidar data,
 * and controls movement according to the environment. It also manages visualization using
 * the AbstractGraphicViewer and provides several behavioral methods such as FORWARD, TURN,
 * FOLLOW_WALL, and SPIRAL.
 *
 * @author G3
 */
#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

// Uncomment the following line to enable automatic period reduction during inactivity
//#define HIBERNATION_ENABLED

#include <expected>
#include <genericworker.h>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <Eigen/Geometry>
#include "hungarian.h"
#include "room_detector.h"


/**
 * @brief Class SpecificWorker implements the core functionality of the component.
 */
class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	/**
	 * @brief Constructor for SpecificWorker.
	 *
	 * Initializes the component, optionally performs startup checks, configures the state machine,
	 * and enables hibernation monitoring if applicable.
	 *
	 * @param configLoader Reference to the configuration loader.
	 * @param tprx Tuple containing the proxies required for robot communication.
	 * @param startup_check Boolean flag indicating whether to perform startup checks.
	 */
	SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);
	/**
	 * @brief Destructor for SpecificWorker.
	 *
	 * Cleans up allocated resources and prints a destruction message.
	 */
	~SpecificWorker();

	struct NominalRoom
	{
		float width; //  mm
		float length;
		Corners corners;
		explicit NominalRoom(const float width_=10000.f, const float length_=5000.f, Corners  corners_ = {}) noexcept :
					width(width_), length(length_), corners(std::move(corners_)){} ;
		Corners transform_corners_to(const Eigen::Affine2d &transform) const  // for room to robot pass the inverse of robot_pose
		{
			Corners transformed_corners;
			for(const auto &[p, _, __] : corners)
			{
				auto ep = Eigen::Vector2d{p.x(), p.y()};
				Eigen::Vector2d tp = transform * ep;
				transformed_corners.emplace_back(QPointF{static_cast<float>(tp.x()), static_cast<float>(tp.y())}, 0.f, 0.f);
			}
			return transformed_corners;
		}
	};
	NominalRoom room{10000.f, 5000.f,
				{{QPointF{-5000.f, -2500.f}, 0.f, 0.f},
					   {QPointF{5000.f, -2500.f}, 0.f, 0.f},
					   {QPointF{5000.f, 2500.f}, 0.f, 0.f},
					   {QPointF{-5000.f, 2500.f}, 0.f, 0.f}}};


public slots:

	/**
	 * @brief Initializes the worker one time.
	 */
	void initialize();

	/**
	 * @brief Main compute loop of the worker.
	 */
	void compute();

	/**
	 * @brief Handles the emergency state loop.
	 */
	void emergency();

	/**
	 * @brief Restores the component from an emergency state.
	 */
	void restore();

    /**
     * @brief Performs startup checks for the component.
     * @return An integer representing the result of the checks.
     */
	int startup_check();

	/**
	 * @brief Slot triggered when a new target is set via mouse click.
	 */
	void new_target_slot(QPointF);

	/**
	*
	*/

private:

	/**
     * @brief Flag indicating whether startup checks are enabled.
     */
	bool startup_check_flag;

	// Graphics-related members
	QRectF dimensions;
	AbstractGraphicViewer *viewer;
	const int ROBOT_LENGTH = 400;
	QGraphicsPolygonItem *robot_polygon;
	QGraphicsPolygonItem *robot_room_draw;

	AbstractGraphicViewer *viewer_room;
	Eigen::Affine2d robot_pose;
	rc::Room_Detector room_detector;
	rc::Hungarian hungarian;

	enum class State {IDLE, FORWARD, TURN, FOLLOW_WALL, SPIRAL};

	std::tuple<State,float,float> state_machine(State state, const RoboCompLidar3D::TPoints& filter_data);

	/**
	 * @brief Filters 3D lidar points to a 2D representation.
	 * @param puntos Collection of 3D lidar points.
	 * @return Optional filtered 2D lidar points.
	 */
	std::optional<RoboCompLidar3D::TPoints> data_filter( const RoboCompLidar3D::TPoints &puntos);

	/**
	 * @brief Enum representing the different robot states.
	 */


	/**
	 * @brief Structure containing lidar-related parameters for movement and thresholds.
	 */
	struct Params
	{
		float STOP_THRESHOLD = 700;
		float LIDAR_RIGHT_SIDE_SECTION = qDegreesToRadians(90);
		float LIDAR_LEFT_SIDE_SECTION = qDegreesToRadians(-90);
		float LIDAR_FRONT_SECTION = qDegreesToRadians(10);
		float ROBOT_LENGTH = 400;
	};
	Params params;

	// Variables for state handling
	int contador_turn = 0;
	float bajada = 0.9f;
	float subida = 50.f;

	// Methods for lidar processing and robot behavior
	void draw_lidar(const RoboCompLidar3D::TPoints &points, QGraphicsScene* scene);
	std::expected<int, std::string> closest_lidar_index_to_given_angle(const  RoboCompLidar3D::TPoints &points, float angle);
	std::tuple<State, float, float> FORWARD_method(const RoboCompLidar3D::TPoints& points);
	std::tuple<State, float, float> TURN_method(const RoboCompLidar3D::TPoints& points);
	std::tuple<State, float, float> FOLLOW_WALL_method(const RoboCompLidar3D::TPoints& points);
	std::tuple<State, float, float> SPIRAL_method(const RoboCompLidar3D::TPoints& points);
	std::optional<RoboCompLidar3D::TPoints> get_min_distance(const RoboCompLidar3D::TPoints& points);

signals:
	//void customSignal();
};

#endif
