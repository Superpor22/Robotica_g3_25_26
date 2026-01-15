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
#include "specificworker.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <ranges>
#include <cppitertools/groupby.hpp>
#include <cppitertools/range.hpp>
#include <expected>
#include <cppitertools/enumerate.hpp>
#include <IceUtil/StringUtil.h>

/**
 * @brief Constructor for the SpecificWorker class.
 *
 * This constructor initializes a new instance of `SpecificWorker` and sets up its internal state,
 * including optional startup checks, state machine configuration, and hibernation monitoring.
 *
 * The initialization process includes the following steps:
 *
 * 1. **Startup Check:**
 *    If `startup_check` is `true`, the constructor calls the `startup_check()` method to perform
 *    any required initial verification before proceeding.
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
#include "specificworker.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <ranges>
#include <cppitertools/groupby.hpp>
#include <cppitertools/range.hpp>
#include <expected>
#include <cppitertools/enumerate.hpp>
#include <IceUtil/StringUtil.h>

/**
 * @brief Constructor for the SpecificWorker class.
 *
 * This constructor initializes a new instance of `SpecificWorker` and sets up its internal state,
 * including optional startup checks, state machine configuration, and hibernation monitoring.
 *
 * The initialization process includes the following steps:
 *
 * 1. **Startup Check:**
 *    If `startup_check` is `true`, the constructor calls the `startup_check()` method to perform
 *    any required initial verification before proceeding.
 *
 * 2. **Hibernation (Optional):**
 *    If hibernation is enabled (`HIBERNATION_ENABLED`) and `startup_check` is `false`,
 *    the `hibernationChecker` timer is started with a 500 ms interval to monitor idle states.
 *
 * 3. **State Machine Configuration:**
 *    The internal `statemachine` is set to use exclusive child states, started, and checked
 *    for errors. If any errors occur during startup, a warning is logged and an exception is thrown.
 *
 * @param configLoader Reference to a `ConfigLoader` object for configuration management.
 * @param tprx Tuple of proxy objects used for robot communication.
 * @param startup_check Boolean flag indicating whether to perform the startup check routine.
 */
SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
	this->startup_check_flag = startup_check;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}
	}
}

/**
 * @brief Destructor for the SpecificWorker class.
 *
 * This method is called when an instance of SpecificWorker is destroyed.
 * It handles cleanup tasks and releases resources allocated during the worker’s lifetime.
 *
 * Currently, it logs a message indicating that the worker is being destroyed.
 *
 * @return void
 */
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

void SpecificWorker::initialize()
{
	std::cout << "Initialize worker" << std::endl;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		///////////// Your code ////////
		// Viewer
		viewer = new AbstractGraphicViewer(this->frame, params.GRID_MAX_DIM);
		auto [r, e] = viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
		robot_draw = r;

		viewer_room = new AbstractGraphicViewer(this->frame_room, params.GRID_MAX_DIM);
		auto [rr, re] = viewer_room->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
		robot_room_draw = rr;
		// draw room in viewer_room
		//habitacion = viewer_room->scene.addRect(nominal_rooms[0].rect(), QPen(Qt::black, 30));
		// show();

		// initialise robot pose
		robot_pose.setIdentity();
		robot_pose.translate(Eigen::Vector2f(0.0,0.0));

		// time series plotter for match error
		TimeSeriesPlotter::Config plotConfig;
		plotConfig.title = "Maximum Match Error Over Time";
		plotConfig.yAxisLabel = "Error (mm)";
		plotConfig.timeWindowSeconds = 15.0; // Show a 15-second window
		plotConfig.autoScaleY = false;       // We will set a fixed range
		plotConfig.yMin = 0;
		plotConfig.yMax = 1000;
		qInfo() << "1";
		time_series_plotter = std::make_unique<TimeSeriesPlotter>(frame_plot_error, plotConfig);
		qInfo() << "2";

		match_error_graph = time_series_plotter->addGraph("", Qt::blue);
	}
}

void SpecificWorker::compute()
{
	qInfo() << "Computing SpecificWorker";

	std::tuple<STATE,float,float> result;
	static auto state = STATE::GOTO_ROOM_CENTER;  // Estado inicial

	// Read data from lidar
	auto filter_data = read_data();

	// filtrar con el filtro de huecos
	filter_data = door_detector.filter_points(filter_data, &viewer->scene);
	draw_lidar(filter_data, &viewer->scene);

	// corners
	const auto &[measured_corners, _] =
		room_detector.compute_corners(filter_data, &viewer->scene);

	float max_match_error = -1;

	if (localised)
	{
		if (const auto res = update_robot_pose(room_index, measured_corners, robot_pose, true); res.has_value())
		{
			robot_pose = res.value().first;
			max_match_error = res.value().second;
			time_series_plotter->addDataPoint(match_error_graph,max_match_error);
		}
	}

	const auto &[st, adv, rot] = state_machine(state, filter_data, measured_corners); // Machine states method
	state = st;
	qInfo() << "Estado actual ---------------" << to_string(state);
	try{ omnirobot_proxy->setSpeedBase(0, adv, rot);}
	catch (const Ice::Exception &e){ std::cout << e << " " << "Conexión con Laser" << std::endl; return;}

	// draw robot in viewer
	robot_room_draw->setPos(robot_pose.translation().x(), robot_pose.translation().y());
	const double angle = qRadiansToDegrees(std::atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0)));
	robot_room_draw->setRotation(angle);

	// // update GUI
	time_series_plotter->update();
	label_state-> setText(to_string(state));
	QString is_localized = "not localised";
	if (localised)
	{
		is_localized = "localised";
	}
	else
	{
		is_localized = "not localised";
	}
	label_localized-> setText(is_localized);
	lcdNumber_room-> display(room_index);

	lcdNumber_adv->display(adv);
	lcdNumber_rot->display(rot);
	lcdNumber_x->display(robot_pose.translation().x());
	lcdNumber_y->display(robot_pose.translation().y());
	lcdNumber_angle->display(angle);
	last_time = std::chrono::high_resolution_clock::now();
}

SpecificWorker::RetVal SpecificWorker::goto_door()
{
	// 1. Puertas detectadas en el frame del robot (LIDAR)
	static QGraphicsLineItem* puerta;
	Doors doors = door_detector.doors();
	if (doors.empty())
	{
		qInfo() << __FUNCTION__ << "No doors detected";
		return {STATE::GOTO_DOOR, 0.f, 0.f};
	}

	// 2. Puerta objetivo en GLOBAL (la nominal que queremos cruzar)
	const Door &nominal_door = nominal_rooms[room_index].doors[current_door];

	const auto target_it = std::ranges::min_element(doors, [nominal_door, this](const auto& a, const auto& b)
	{
	return (a.center() - robot_pose.inverse().cast<float>() * nominal_door.global_center()).norm() <
		(b.center() - robot_pose.inverse().cast<float>() * nominal_door.global_center()).norm();
	});

	Door target_door = *target_it;

	target_door.p1_global = nominal_rooms[room_index].get_projection_of_point_on_closest_wall(robot_pose.cast<float>() * target_door.p1.cast<float>());
	target_door.p2_global = nominal_rooms[room_index].get_projection_of_point_on_closest_wall(robot_pose.cast<float>() * target_door.p2.cast<float>());

	// 5. Centro de la puerta objetivo EN FRAME DEL ROBOT
	Eigen::Vector2f centro = target_door.center();

	// 6. Control
	const float k_rot = 1.0f;
	const float angulo = std::atan2(centro.x(), centro.y());
	const float dist   = centro.norm();

	// 7. Condición de salida
	if (dist < 600.f)
		return {STATE::ORIENT_TO_DOOR, 0.f, 0.f};

	// 8. Velocidades
	const float vrot  = k_rot * angulo;
	const float brake = std::exp(-angulo * angulo / (M_PI / 10.f));
	const float adv   = 500.f * brake;

	return {STATE::GOTO_DOOR, adv, vrot};
}

SpecificWorker::RetVal SpecificWorker::orient_to_door()
{
	const auto doorsy = door_detector.doors();

	if (doorsy.empty()) return {};

	const auto sd = std::ranges::min_element(doorsy, [](const auto &a, const auto &b)
	   {  return std::fabs(a.center_angle()) < std::fabs(b.center_angle());} );

	auto centro = sd->center();

	float k = 0.5f;
	auto angulo = atan2(centro.x(), centro.y());

	if (abs(angulo) < 0.1)
	{
		localised = false;
		return {STATE::CROSS_DOOR, 0.5, 0.0};
	}

	float vrot = k * angulo;

	return {STATE::ORIENT_TO_DOOR, 0.0, vrot};
}

SpecificWorker::RetVal SpecificWorker::cross_door(const RoboCompLidar3D::TPoints& points)
{
	// exit condition
	static int contador = 0;

	contador++;
	if (contador == 40)
	{
		robot_room_draw->hide();
		contador = 0;
		qInfo() << "REMOVING ROOM RECT";
		viewer_room->scene.removeItem(habitacion);

		for (auto puerta : puertas)
		{
			viewer_room->scene.removeItem(puerta);
		}

		nominal_rooms[room_index].doors = door_detector.doors();
		if (!nominal_rooms[room_index].doors.empty())
		{
			const auto &entering_door = nominal_rooms[room_index].doors[current_door]; // door we are entering now
			Eigen::Vector2f door_center = entering_door.global_center(); //
			// Vector from door to origin (0,0) is -door_center
			const float angle = std::atan2(-door_center.x(), -door_center.y());
			// robot_pose now must be translated so it is drawn in the new room correctly
			robot_pose.setIdentity();
			door_center.y() -= 500; // place robot 500 mm inside the room
			robot_pose.translate(door_center);
			robot_pose.rotate(0);
			std::cout << door_center.x() << " " << door_center.y() << " " << angle << std::endl;
		}
		localised = false;

		// Puerta entrada
		//nominal_rooms[door_crossing.entering_room_index].doors[door_crossing.entering_door_index].visited = true;

		// Puerta salida
		// nominal_rooms[door_crossing.leaving_room_index].doors[door_crossing.leaving_door_index].visited = true;

		return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
	}

	// do my thing
	return {STATE::CROSS_DOOR, 500.f, 0.f};
}

void SpecificWorker::choose_next_door(int current_room)
{
	int i = 0;
	for (Door &door : nominal_rooms[current_room].doors)
	{
		if (door.visited == false )
		{
			current_door = i;
			break;
		}
		i ++;
	}
	if (i >= nominal_rooms[current_room].doors.size() )
	{
		for (auto &door : nominal_rooms[current_room].doors)
		{
			door.visited = false;
		}
		current_door = 0;
	}
}

SpecificWorker::RetVal SpecificWorker::TURN_method(const Corners &corners)
{
	// exit condition
	door_crossing.track_entering_door(door_detector.doors());
	ROBOCOMPMNIST::MNISTResult mnist_result;
	try
	{
		mnist_result = mnist_proxy->getNumber();
		qInfo() << " number : " << mnist_result.number;
	}
	catch (const std::exception &e)
	{
		qInfo() << e.what();
	};

	//const auto &[success, current_room, left_right] = image_processor.check_colour_patch_in_image(camera360rgb_proxy, label_img);

	if (mnist_result.number != -1 && mnist_result.center > 800)
	{
		if (mnist_result.number != 0 && mnist_result.number != 1){
			return{STATE::TURN, 0.0f, params.RELOCAL_ROT_SPEED};
		}
		room_index = mnist_result.number;

		const auto m = hungarian.match(corners,nominal_rooms[room_index].corners() );
		if (m.empty())
		{
			qInfo() << __FUNCTION__ << "empty match";
		};
		if (m.size() < 3)
		{
			qInfo() << __FUNCTION__ << "m size < 3";
			return{STATE::TURN, 0.0f, params.RELOCAL_ROT_SPEED};
		}
		const auto max_error_iter = std::ranges::max_element(m, [](const auto &a, const auto &b)
								{ return std::get<2>(a) < std::get<2>(b); });
		if (const auto max_match_error = std::get<2>(*max_error_iter); max_match_error > params.RELOCAL_DONE_MATCH_MAX_ERROR)
		{
			qInfo() << __FUNCTION__ << "match error > " << params.RELOCAL_DONE_MATCH_MAX_ERROR;
			return{STATE::TURN, 0.0f, params.RELOCAL_ROT_SPEED};
		}



		// update robot pose to have a fresh value
		if (const auto res = update_robot_pose(mnist_result.number, corners, robot_pose, false); res.has_value())
		   robot_pose = res.value().first;
		else return{STATE::TURN, 0.0f, params.RELOCAL_ROT_SPEED/2};

		// save doors to nominal_room if not previously visited
		nominal_rooms[room_index].name = image_processor.room_name_from_index(room_index);
		auto doorsy = door_detector.doors();
		if (doorsy.empty()) { qWarning() << __FUNCTION__ << "empty doors"; return{STATE::TURN, 0.0f, params.RELOCAL_ROT_SPEED};}

			for (auto &d : doorsy)
			{
				d.p1_global = nominal_rooms[room_index].get_projection_of_point_on_closest_wall(robot_pose.cast<float>() * d.p1.cast<float>());
				d.p2_global = nominal_rooms[room_index].get_projection_of_point_on_closest_wall(robot_pose.cast<float>() * d.p2.cast<float>());
				puertas.emplace_back(viewer_room->scene.addLine(d.p1_global.x(), d.p1_global.y(), d.p2_global.x(), d.p2_global.y(), QPen(Qt::red, 90)));
			}

			nominal_rooms[room_index].doors = doorsy;

			door_crossing.set_entering_data(room_index, nominal_rooms);

			nominal_rooms[room_index].doors[door_crossing.entering_door_index].connect_to_door = door_crossing.leaving_room_index;

			current_door = (door_crossing.entering_door_index + 1) % nominal_rooms[room_index].doors.size();
			door_crossing.leaving_door_index = current_door;

			// nominal_rooms[door_crossing.entering_room_index].doors[door_crossing.entering_door_index].visited = true;
		 //    //choose door to go
		 //    choose_next_door(room_index); // TODO crear metodo para elegir puerta

			Door d = doorsy[current_door];
			d.p1_global = nominal_rooms[room_index].get_projection_of_point_on_closest_wall(robot_pose.cast<float>() * d.p1.cast<float>());
			d.p2_global = nominal_rooms[room_index].get_projection_of_point_on_closest_wall(robot_pose.cast<float>() * d.p2.cast<float>());

			puertas.emplace_back(viewer_room->scene.addLine(d.p1_global.x(), d.p1_global.y(), d.p2_global.x(), d.p2_global.y(), QPen(Qt::blue,150)));

		habitacion = viewer_room->scene.addRect(nominal_rooms[room_index].rect(), QPen(Qt::black, 30));
		robot_room_draw->show();

		localised = true;
		return {STATE::GOTO_DOOR, 0.0f, 0.0f};  // SUCCESS
	}
	// continue turning
	return {STATE::TURN, 0.0f, params.RELOCAL_ROT_SPEED};
}

SpecificWorker::RetVal SpecificWorker::IDLE_method()
{
	return {};
}

SpecificWorker::RetVal SpecificWorker::goto_room_center(const RoboCompLidar3D::TPoints& points)
{
	auto center = center_estimator.estimate(points);
	if (not center.has_value())
		return{};

	auto dist = center.value().norm();
	// exit condition:
	if (dist < 100.f) return {STATE::TURN,0.f, 0.f};

	// 1. Convertir Vector2d → Vector2f
	Eigen::Vector2f center_f = center.value().cast<float>();

	// 2. Llamar al controlador
	auto [v, w] = robot_controller(center_f);

	// 3. Tracks the position of the just entered door.
	door_crossing.track_entering_door(door_detector.doors());

	// 4. Devolver estado, avance y rotación
	return {STATE::GOTO_ROOM_CENTER, v, w};
}

SpecificWorker::RetVal SpecificWorker::localise(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
	// initialise robot pose at origin. Necessary to reser pose accumulation
	robot_pose.setIdentity();
	robot_pose.translate(Eigen::Vector2f(0.0,0.0));
	localised = false;


	// if error high but not at room centre, go to centering step
	// compute mean of LiDAR points as room center estimate


	if(const auto center = center_estimator.estimate(points); center.has_value())
	{
		if (center.value().norm() > params.RELOCAL_CENTER_EPS )
			return{STATE::GOTO_ROOM_CENTER, 0.0f, 0.0f};


		// If close enough to center -> stop and move to TURN
		if (center.value().norm() < params.RELOCAL_CENTER_EPS )
			return {STATE::TURN, 0.0f, 0.0f};
	}
	qWarning() << __FUNCTION__ << "Not able to estimate room center from walls, continue localising.";
	return {STATE::LOCALISE, 0.0f, 0.0f};
}

std::tuple<SpecificWorker::STATE,float,float> SpecificWorker::state_machine(STATE state, const RoboCompLidar3D::TPoints &filter_data, const Corners &corners)
{
	switch (state)
	{
	case STATE::LOCALISE:
		return{};
		break;
	case STATE::IDLE:
		return IDLE_method();
		break;
	case STATE::GOTO_DOOR:
		return goto_door();
		break;
	case STATE::TURN:
		return TURN_method(corners);
		break;
	case STATE::ORIENT_TO_DOOR:
		return orient_to_door();
		break;
	case STATE::GOTO_ROOM_CENTER:
		return goto_room_center(filter_data);
		break;
	case STATE::CROSS_DOOR:
		return cross_door(filter_data);
		break;
	}
	return {};
}

std::tuple<float, float> SpecificWorker::robot_controller(const Eigen::Vector2f &target)
{
	auto dist = target.norm();
	// exit condition:
	if (dist < 100.f) return {0.f, 0.f};

	// do my thing
	auto theta = std::atan2(target.x(), target.y());
	float rot = 0.5f * theta;
	float angle_break = exp((-theta * theta)/(M_PI/6.f));
	float adv = 1000.f * angle_break;

	return {adv, rot};
}

RoboCompLidar3D::TPoints SpecificWorker::read_data()
{
	// data = read_data("helios");
	RoboCompLidar3D::TData data;
	try { data = lidar3d_proxy->getLidarDataWithThreshold2d("helios", 12000, 2);} 
	catch (const Ice::Exception &e){ std::cout << e << " " << "Conexión con Laser" << std::endl; return{};}

	// filter data from 3D to 2D
	RoboCompLidar3D::TPoints filter_data;
	if (const auto filter_data_= data_filter(data.points); filter_data_.has_value())
		filter_data = filter_data_.value();
	else
	{    qWarning() << "filter_data_.has_value()"; return {};}

	return data.points;
  }
//
// /**
//  * @brief Visualizes lidar data on a QGraphicsScene, highlighting key points and directions.
//  *
//  * This method draws a visual representation of lidar readings on the provided Qt graphics scene.
//  * It renders individual lidar points, highlights important reference points, and adds guiding
//  * lines to help visualize the robot’s perception of its environment.
//  *
//  * The drawing process includes the following steps:
//  *
//  * 1. **Scene Reset:**
//  *    Removes all previously drawn items to refresh the visualization at each iteration.
//  *
//  * 2. **Lidar Points Rendering:**
//  *    Draws each lidar point as a small green square positioned according to its (x, y) coordinates.
//  *
//  * 3. **Frontal Minimum Distance Highlight:**
//  *    Calculates the closest point within the frontal section of the lidar’s field of view.
//  *    - If the point is closer than `params.STOP_THRESHOLD`, it is drawn in **red** (indicating an obstacle).
//  *    - Otherwise, it is drawn in **magenta** (indicating safe distance).
//  *
//  * 4. **Wall Distance Visualization:**
//  *    Identifies the closest points on the left and right sides of the robot (using lidar angle sections).
//  *    The nearest of these two is highlighted with an **orange square**, and a line is drawn from the robot’s
//  *    center to that point to indicate the closest wall or obstacle.
//  *
//  * 5. **Directional Boundaries:**
//  *    Draws two colored lines (blue and red) extending from the robot at angles defined by
//  *    `params.LIDAR_FRONT_SECTION`, representing the boundaries of the frontal field of view.
//  *
//  * The method maintains a static list of drawn items so that they can be efficiently removed
//  * and redrawn in each update cycle, ensuring smooth real-time visualization.
//  *
//  * @param points The collection of lidar points (in 2D) to be visualized.
//  * @param scene Pointer to the QGraphicsScene where lidar data and visualization elements will be drawn.
//  * @return void
//  */
void SpecificWorker::draw_lidar(const  RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
	static std::vector<QGraphicsItem*> items;   // store items so they can be shown between iterations

	// remove all items drawn in the previous iteration
	for(auto i: items)
	{
		scene->removeItem(i);
		delete i;
	}
	items.clear();

	auto color = QColor(Qt::green);
	auto brush = QBrush(QColor(Qt::green));
	for(const auto &p : points)
	{
		auto item = scene->addRect(-50, -50, 100, 100, color, brush);
		item->setPos(p.x, p.y);
		items.push_back(item);
	}

	// --- Dibujar el centro de la habitación ---
	auto center = room_detector.estimate_center_from_walls();
	if(center.has_value())
	{
		auto center_item = scene->addEllipse(-150, -150, 300, 300, QPen(Qt::cyan), QBrush(Qt::cyan));
		center_item->setPos(center.value().x(), center.value().y());
		items.push_back(center_item);

		// opcional: dibujar un texto con coordenadas
		auto text_item = scene->addText(QString("Center\nx=%1 y=%2")
										.arg(center.value().x())
										.arg(center.value().y()));
		text_item->setDefaultTextColor(Qt::cyan);
		text_item->setPos(center.value().x() + 20, center.value().y() + 20);
		items.push_back(text_item);
	}
}

//
// /**
//  * @brief Calculates the index of the closest lidar point to the given angle.
//  *
//  * This method searches through the provided std::list of lidar points and finds the point
//  * whose angle (phi value) is closest to the specified angle. If a matching point is found,
//  * the index of the point in the std::list is returned. If no point is found that matches the condition,
//  * an error message is returned.
//  *
//  * @param points The collection of lidar points to search through.
//  * @param angle The target angle to find the closest matching point.
//  * @return std::expected<int, std::string> containing the index of the closest lidar point if found,
//  * or an error message if no such point exists.
//  */
std::expected<int, std::string> SpecificWorker::closest_lidar_index_to_given_angle(const  RoboCompLidar3D::TPoints &points, float angle)
{
	// search for the point in points whose phi value is closest to angle
	auto res = std::ranges::find_if(points, [angle](auto &a){ return a.phi > angle;});
	if(res != std::end(points))
		return std::distance(std::begin(points), res);
	else
		return std::unexpected("No closest value found in method <closest_lidar_index_to_given_angle>");
}

/**
 * @brief Filters lidar points by angle and returns the point with the minimum distance for each angle group.
 *
 * This method groups the provided lidar points by their angle (phi value) and, for each group,
 * finds the point with the smallest distance (r value). The filtered points, with one selected per angle group,
 * are then returned. If the input points collection is empty, an empty result is returned.
 *
 * @param puntos The collection of lidar points to be processed.
 * @return std::optional<RoboCompLidar3D::TPoints> containing the filtered lidar points where each group
 *         of points with the same angle is represented by the point with the smallest distance, or
 *         an empty result if no points exist.
 */
std::optional<RoboCompLidar3D::TPoints> SpecificWorker::data_filter(const RoboCompLidar3D::TPoints& puntos)
{
	if (puntos.empty()) return {};

	RoboCompLidar3D::TPoints salida; salida.reserve(puntos.size());
	// Agrupar por phi y obtener el mínimo de r por grupo en una línea, usando push_back para almacenar en el vector
	for (auto&& [angle, group] : iter::groupby(puntos, [](const auto& p)
	{
		float factor = std::pow(10.0f, 2);  // Potencia de 10 para mover el punto decimal
		return std::floor(p.phi * factor) / factor;  // Redondear y devolver con la cantidad deseada de decimales
	})) {
		auto min_r = std::min_element(std::begin(group), std::end(group),
			[](const auto& p1, const auto& p2) { return p1.r < p2.r; });
		salida.emplace_back(*min_r);
	}
	return salida;
}


std::optional<std::pair<Eigen::Affine2f, float>> SpecificWorker::update_robot_pose(
	int room_index, const Corners& corners, const Eigen::Affine2f& r_pose, bool transform_corners)
{
	// match corners transforming first nominal corners to robot's frame
	Match match;
	if (transform_corners)
		match = hungarian.match(corners, nominal_rooms[room_index].transform_corners_to(r_pose.inverse()));
	else
		match = hungarian.match(corners, nominal_rooms[room_index].corners());


	if (match.empty() or match.size() < 4)
		return {};


	const auto max_error_iter = std::ranges::max_element(match, [](const auto &a, const auto &b)
	  { return std::get<2>(a) < std::get<2>(b); });


	const auto max_match_error = std::get<2>(*max_error_iter);


	// create matrices W and b for pose estimation
	Eigen::MatrixXd W(match.size() * 2, 3);
	Eigen::VectorXd b(match.size() * 2);
	for (auto &&[i,m]: match | iter::enumerate )
	{
		auto &[meas_c, nom_c, _] = m;
		auto &[p_meas, __, ___] = meas_c;
		auto &[p_nom, ____, _____] = nom_c;

		b(2 * i)     = p_nom.x() - p_meas.x();

		b(2 * i + 1) = p_nom.y() - p_meas.y();
		W.block<1, 3>(2 * i, 0)     << 1.0, 0.0, -p_meas.y();
		W.block<1, 3>(2 * i + 1, 0) << 0.0, 1.0, p_meas.x();
	}

	// estimate new pose with pseudoinverse
	const Eigen::Vector3d r = (W.transpose() * W).inverse() * W.transpose() * b;
	if (r.array().isNaN().any())
	{
		qWarning() << __FUNCTION__ << "NaN values in r ";
		return {};
	}


	auto r_pose_copy = r_pose;
	r_pose_copy.translate(Eigen::Vector2f(r(0), r(1)));
	r_pose_copy.rotate(r[2]);
	return {{r_pose_copy, max_match_error}};
}

///////////////////////////////////////////////////////////////////////////////////////////////

void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
    //emergencyCODE
    //
    //if (SUCCESSFUL) //The componet is safe for continue
    //  emmit goToRestore()
}

//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
    //restoreCODE
    //Restore emergency component

}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}