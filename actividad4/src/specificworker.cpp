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
		habitacion = viewer_room->scene.addRect(nominal_rooms[0].rect(), QPen(Qt::black, 30));
		show();

		// initialise robot pose
		robot_pose.setIdentity();
		robot_pose.translate(Eigen::Vector2d(0.0,0.0));

		// time series plotter for match error
		TimeSeriesPlotter::Config plotConfig;
		plotConfig.title = "Maximum Match Error Over Time";
		plotConfig.yAxisLabel = "Error (mm)";
		plotConfig.timeWindowSeconds = 15.0; // Show a 15-second window
		plotConfig.autoScaleY = false;       // We will set a fixed range
		plotConfig.yMin = 0;
		plotConfig.yMax = 1000;
		time_series_plotter = std::make_unique<TimeSeriesPlotter>(frame_plot_error, plotConfig);
		match_error_graph = time_series_plotter->addGraph("", Qt::blue);

		// stop robot
		move_robot(0, 0, 0);
	}
}

void SpecificWorker::compute()
{
	std::tuple<STATE,float,float> result;
	static auto state = STATE::GOTO_ROOM_CENTER;  // Estado inicial

	// Read data from lidar
	auto filter_data = read_data();

	doors = door_detector.detect(filter_data, &viewer->scene);
	draw_doors(doors, &viewer->scene);
	// filtrar con el filtro de huecos
	filter_data = door_detector.filter_points(filter_data, &viewer->scene);
	draw_lidar(filter_data, &viewer->scene);

	// corners
	const auto &[measured_corners, _] =
		room_detector.compute_corners(filter_data, &viewer->scene);

	auto robot_corners = nominal_rooms[room_index].transform_corners_to(robot_pose.inverse());
	auto match = hungarian.match(measured_corners, robot_corners);

	if (match.empty())
		qDebug() << "No match found";

	// compute max of  match error
	float max_match_error = 99999.f;
	if (not match.empty())
	{
		const auto max_error_iter = std::ranges::max_element(match, [](const auto &a, const auto &b)
			{ return std::get<2>(a) < std::get<2>(b); });
		max_match_error = static_cast<float>(std::get<2>(*max_error_iter));
		time_series_plotter->addDataPoint(0, max_match_error);
		//print_match(match, max_match_error); //debugging
	}

	qInfo() << "Puertas: " << doors.size();
	qInfo() << "MATCH SIZE: " << match.size();
	if (match.size() < 3)
		localised = false;
	else
		localised = true;
   // update robot pose
   if (localised)
   {
		update_robot_pose(measured_corners, match);
   }

	qInfo() << "Estado inicial --------------" << to_string(state);
	const auto &[st, adv, rot] = state_machine(state, filter_data); // Machine states method
	qInfo() << "St -------------------------" << to_string(st);
	state = st;
	qInfo() << "Estado salida ---------------" << to_string(state);
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
	// exit condition
	if (doors.empty()) return {};
	auto rp = robot_pose.translation();

	auto target = robot_pose.inverse() * current_door.center_before(rp, 1000.f, true).cast<double>();

	qInfo() << target.norm() << "------------------" << target.x() << " " << target.y() ;
	// viewer->scene.addEllipse(target.x(), target.y(), 200, 200, QPen(Qt::red), QBrush(Qt::black));
	if ( target.norm() < 500.f ) return {STATE::ORIENT_TO_DOOR, 0.f, 0.f};

	// do my thing
	const auto &[adv, rot] = robot_controller(target.cast<float>());
	return {STATE::GOTO_DOOR, adv*0.4, rot};

}

SpecificWorker::RetVal SpecificWorker::orient_to_door()
{
	static int contador = 0;
	if (doors.empty()) return {};
	// exit condition
	auto rp = robot_pose.translation();
	auto u = doors[0].center();
	auto v = doors[0].p2 - doors[0].p1;

	float dot = u.dot(v);
	float norm_u = u.norm();
	float norm_v = v.norm();

	float cos_angle = dot / (norm_u * norm_v);

	// Clamp por seguridad numérica (cos puede quedar 1.00001 o -1.00001)
	cos_angle = std::clamp(cos_angle, -1.0f, 1.0f);

	auto angle = std::acos(cos_angle);  // en radianes

	if (qRadiansToDegrees(angle) > 70 and qRadiansToDegrees(angle) < 110)
	{
		auto [_, w] = robot_controller(u);
		qInfo() << "rotación:" << w;
		if (w < 0.04f)
			return {STATE::CROSS_DOOR, 0.f, 0.f};
		return {STATE::ORIENT_TO_DOOR, 0.f, w};
	}

	// do my thing
	float signo = (angle > 0.f) ? 1.f : -1.f;
	return {STATE::ORIENT_TO_DOOR, 0.f, 0.3f * signo};
}

SpecificWorker::RetVal SpecificWorker::cross_door(const RoboCompLidar3D::TPoints& points)
{
	// exit condition
	static int contador = 0;

	contador++;
	if (contador == 65)
	{
		contador = 0;
		room_index = (room_index + 1) % 2;
		if (room_index == 0)
		{
			color = Qt::red;
		}
		else
		{
			color = "green";
		}
		viewer_room->scene.removeItem(habitacion);
		for (auto puerta : puertas)
			viewer_room->scene.removeItem(puerta);
		habitacion = viewer_room->scene.addRect(nominal_rooms[room_index].rect(), QPen(Qt::black, 30));
		show();
		return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
	}

	// do my thing
	return {STATE::CROSS_DOOR, 500.f, 0.f};
}

SpecificWorker::RetVal SpecificWorker::TURN_method()
{
	// exit condition
	const auto &[success, turn] = image_processor.check_colour_patch_in_image(camera360rgb_proxy, color, label_img);
	if (success)
	{
		for (auto &d : doors)
		{
			d.p1_global = nominal_rooms[room_index].get_projection_of_point_on_closest_wall((robot_pose * d.p1.cast<double>()).cast<float>());
			d.p2_global = nominal_rooms[room_index].get_projection_of_point_on_closest_wall((robot_pose * d.p2.cast<double>()).cast<float>());
			puertas.emplace_back(viewer_room->scene.addLine(d.p1_global.x(), d.p1_global.y(), d.p2_global.x(), d.p2_global.y(), QPen(Qt::red, 90)));
		}
		nominal_rooms[room_index].doors = doors;
		current_door = doors[0];
		//Comprobar si las 4 esquinas están bien colocadas (con el match())
		return {STATE::GOTO_DOOR, 0.f, 0.f};
	}

	// do my thing
	return {STATE::TURN, 0.f, 0.4f};
}

SpecificWorker::RetVal SpecificWorker::IDLE_method()
{
	return {};
}

SpecificWorker::RetVal SpecificWorker::goto_room_center(const RoboCompLidar3D::TPoints& points)
{
	std::optional<Eigen::Vector2d> center;
	if (center = room_detector.estimate_center_from_walls(); not center.has_value())
		return{};

	auto dist = center.value().norm();
	// exit condition:
	if (dist < 100.f) return {STATE::TURN,0.f, 0.f};

	// Do my thing

	// 1. Convertir Vector2d → Vector2f
	Eigen::Vector2f center_f = center.value().cast<float>();

	// 2. Llamar al controlador
	auto [v, w] = robot_controller(center_f);

	// 3. Devolver estado, avance y rotación
	return {STATE::GOTO_ROOM_CENTER, v, w};
}

std::tuple<SpecificWorker::STATE,float,float> SpecificWorker::state_machine(STATE state, const RoboCompLidar3D::TPoints &filter_data)
{
	switch (state)
	{
	case STATE::IDLE:
		return IDLE_method();
		break;
	case STATE::GOTO_DOOR:
		return goto_door();
		break;
	case STATE::TURN:
		return TURN_method();
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

void SpecificWorker::draw_doors(const Doors& doors, QGraphicsScene* scene)
{
	static std::vector<QGraphicsItem*> items;
	for (const auto i: items)
	{
		scene->removeItem(i);
		delete i;
	}
	items.clear();

	for (const auto &d: doors )
	{
		auto item = scene->addEllipse(-100, -100, 200, 200, QPen(Qt::red), QBrush(Qt::red));
		item->setPos(d.p1.x(), d.p1.y());
		items.emplace_back(item);
		item = scene->addEllipse(-100, -100, 200, 200, QPen(Qt::red), QBrush(Qt::red));
		item->setPos(d.p2.x(), d.p2.y());
		items.emplace_back(item);
		auto line = scene->addLine(d.p1.x(), d.p1.y(), d.p2.x(), d.p2.y(), QPen(Qt::red, 30));
		items.emplace_back(line);
	}

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
	{	qWarning() << "filter_data_.has_value()"; return {};}

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

    // compute and draw minimum distance point in frontal range
    // auto offset_begin = closest_lidar_index_to_given_angle(points, -params.LIDAR_FRONT_SECTION);
    // auto offset_end = closest_lidar_index_to_given_angle(points, params.LIDAR_FRONT_SECTION);
    // if(not offset_begin or not offset_end)
    // { std::cout << offset_begin.error() << " " << offset_end.error() << std::endl; return ;}    // abandon the ship
    // auto min_point = std::min_element(std::begin(points) + offset_begin.value(), std::begin(points) + offset_end.value(), [](auto &a, auto &b)
    // { return a.distance2d < b.distance2d; });
    // QColor dcolor;
    // if(min_point->distance2d < params.STOP_THRESHOLD)
    //     dcolor = QColor(Qt::red);
    // else
    //     dcolor = QColor(Qt::magenta);
    // auto ditem = scene->addRect(-100, -100, 200, 200, dcolor, QBrush(dcolor));
    // ditem->setPos(min_point->x, min_point->y);
    // items.push_back(ditem);

    // compute and draw minimum distance point to wall
    // auto wall_res_right = closest_lidar_index_to_given_angle(points, params.LIDAR_RIGHT_SIDE_SECTION);
    // auto wall_res_left = closest_lidar_index_to_given_angle(points, params.LIDAR_LEFT_SIDE_SECTION);
    // if(not wall_res_right or not wall_res_left)   // abandon the ship
    // {
    //     qWarning() << "No valid lateral readings" << QString::fromStdString(wall_res_right.error()) << QString::fromStdString(wall_res_left.error());
    //     return;
    // }
    // auto right_point = points[wall_res_right.value()];
    // auto left_point = points[wall_res_left.value()];
    // // compare both to get the one with minimum distance
    // auto min_obj = (right_point.distance2d < left_point.distance2d) ? right_point : left_point;
    // auto item = scene->addRect(-100, -100, 200, 200, QColor(QColorConstants::Svg::orange), QBrush(QColor(QColorConstants::Svg::orange)));
    // item->setPos(min_obj.x, min_obj.y);
    // items.push_back(item);
    // // draw a line from the robot to the minimum distance point
    // auto item_line = scene->addLine(QLineF(QPointF(0.f, 0.f), QPointF(min_obj.x, min_obj.y)), QPen(QColorConstants::Svg::orange, 10));
    // items.push_back(item_line);

    // Draw two lines coming out from the robot at angles given by params.LIDAR_OFFSET
    // Calculate the end points of the lines
	// auto res_right = closest_lidar_index_to_given_angle(points, params.LIDAR_FRONT_SECTION);
	// auto res_left = closest_lidar_index_to_given_angle(points, -params.LIDAR_FRONT_SECTION);
 //    if(not res_right or not res_left)
 //    { std::cout << res_right.error() << " " << res_left.error() << std::endl; return ;}
 //    // draw two lines at the edges of the range
 //    float right_line_length = points[res_right.value()].distance2d;
 //    float left_line_length = points[res_left.value()].distance2d;
 //    float angle1 = points[res_left.value()].phi;
 //    float angle2 = points[res_right.value()].phi;
 //    QLineF line_left{QPointF(0.f, 0.f),
 //                     robot_draw->mapToScene(left_line_length * sin(angle1), left_line_length * cos(angle1))};
 //    QLineF line_right{QPointF(0.f, 0.f),
 //                      robot_draw->mapToScene(right_line_length * sin(angle2), right_line_length * cos(angle2))};
 //    QPen left_pen(Qt::blue, 10); // Blue color pen with thickness 3
 //    QPen right_pen(Qt::red, 10); // Blue color pen with thickness 3
 //    auto line1 = scene->addLine(line_left, left_pen);
 //    auto line2 = scene->addLine(line_right, right_pen);
 //    items.push_back(line1);
 //    items.push_back(line2);
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


bool SpecificWorker::update_robot_pose(const Corners& corners, const Match& match)
{
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
	std::cout << r << std::endl;
	qInfo() << "--------------------";

	if (r.array().isNaN().any())
		return {};

	robot_pose.translate(Eigen::Vector2d(r(0), r(1)));
	robot_pose.rotate(r[2]);

	robot_room_draw->setPos(robot_pose.translation().x(), robot_pose.translation().y());
	const double angle = std::atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0));
	robot_room_draw->setRotation(qRadiansToDegrees(angle));

	return false;
}

void SpecificWorker::move_robot(float adv, float rot, float max_match_error)
{
}

/**
 * @brief Determines the robot's next movement state based on the closest lidar point.
 *
 * This method analyzes the provided set of lidar points to determine how the robot should move.
 * It finds the point with the minimum distance (r value) and decides the next movement state
 * based on that distance:
 * - If the closest object is nearer than 800 units, the robot switches to the TURN state.
 * - If the closest object is farther than 1100 units, the robot switches to the SPIRAL state.
 * - Otherwise, the robot remains in the FORWARD state, moving straight ahead.
 *
 * The method also returns associated speed and rotational values depending on the selected state.
 *
 * @param points The collection of lidar points to analyze.
 * @return std::tuple<SpecificWorker::State, float, float> containing:
 *         - The next movement state of the robot (FORWARD, TURN, or SPIRAL).
 *         - The linear velocity value.
 *         - The rotational velocity value.
 */
// std::tuple<SpecificWorker::State, float, float> SpecificWorker::FORWARD_method(const RoboCompLidar3D::TPoints& points)
// {
// 	auto min_dist = std::min_element(std::begin(points), std::end(points),[](const auto& p1, const auto& p2)
// 			{ return p1.r < p2.r; });
//
// 	qInfo() << "Punto actual: " << min_dist->r;
//
// 	if (min_dist->r < 800)  // Objeto cerca → gira
// 	{
// 		qInfo() << "CHANGE FROM FORWARD TO TURN";
// 		return {State::TURN, 0.0f, 0.0f};
// 	}
// 	if (min_dist->r > 1100)
// 	{
// 		bajada = 0.6f;
// 		subida = 1000.f;
// 		qInfo() << "-----------------------------Bajada_FORWARD: " << bajada << " subida: " << subida;
// 		return {State::SPIRAL, 0.0f, 0.0f};
// 	}
//
// 	/// What I do when I stay
// 	return {State::FORWARD, 1000.0f, 0.0f};
//
// }
//
// /**
//  * @brief Controls the robot's turning behavior based on lidar data.
//  *
//  * This method determines how the robot should turn when an obstacle is detected nearby.
//  * It analyzes the provided lidar points to find the closest point (with the minimum distance, r value)
//  * and decides whether to keep turning or transition to another movement state:
//  *
//  * - If no obstacle is closer than 800 units, the robot may stop turning and either:
//  *   - Switch to the FOLLOW_WALL state (with 50% probability), or
//  *   - Switch to the FORWARD state, adjusting its rotation direction based on the angle (phi) of the closest point.
//  * - If an obstacle is still detected within 800 units, the robot continues turning in place.
//  *   The direction of rotation (left or right) depends on whether the closest point's angle (phi) is negative or positive.
//  *
//  * The method also includes a counter (`contador_turn`) to control how long the robot stays in the TURN state.
//  * If the counter exceeds a threshold (15 iterations), the robot performs a stronger turn.
//  *
//  * @param points The collection of lidar points used to determine proximity and turning direction.
//  * @return std::tuple<SpecificWorker::State, float, float> containing:
//  *         - The next movement state of the robot (TURN, FOLLOW_WALL, or FORWARD).
//  *         - The linear velocity value.
//  *         - The rotational velocity value.
//  */

//
// /**
//  * @brief Controls the robot’s behavior while following a wall using lidar data.
//  *
//  * This method adjusts the robot’s movement to maintain a consistent distance from a wall or obstacle.
//  * It analyzes the provided lidar points to find the closest point (with the minimum distance, r value)
//  * and decides how to proceed based on that distance:
//  *
//  * - If the closest point is between 770 and 810 units away, the robot maintains a stable forward motion
//  *   while continuing to follow the wall.
//  * - If the closest point is farther than 810 units, the robot adjusts its trajectory to move closer to the wall:
//  *   - If the closest point’s angle (phi) is negative, it turns slightly right.
//  *   - If the angle (phi) is positive, it turns slightly left.
//  * - If none of these conditions are met, the robot transitions back to the FORWARD state to continue moving straight.
//  *
//  * This behavior helps the robot navigate parallel to obstacles, maintaining an optimal distance for safe movement.
//  *
//  * @param points The collection of lidar points used to measure the robot’s distance from nearby walls.
//  * @return std::tuple<SpecificWorker::State, float, float> containing:
//  *         - The next movement state of the robot (FOLLOW_WALL or FORWARD).
//  *         - The linear velocity value.
//  *         - The rotational velocity value.
//  */
// std::tuple<SpecificWorker::State, float, float> SpecificWorker::FOLLOW_WALL_method(const RoboCompLidar3D::TPoints& points)
// {
//
// 	auto min_dist = std::min_element(std::begin(points), std::end(points),[](const auto& p1, const auto& p2)
// 		{ return p1.r < p2.r; });
//
// 	// Si ya no hay obstáculo cerca, volvemos a FORWARD
// 	if (min_dist->r > 770 and min_dist->r < 810)
// 	{
// 		qInfo() << "CHANGE FROM FOLLOW WALL TO FORWARD";
// 		return {State::FOLLOW_WALL, 1000.0f, 0.0f};  // Podemos avanzar
// 	}
//
// 	qInfo() << "CONTINUE FOLLOWING WALL";
//
// 	// What I do if I Stay
// 	if (min_dist->r > 810)
// 	{
// 		if (min_dist->phi < 0)
// 		{
// 			qInfo() << "FOLLOW WALL 1";
//
// 			return {State::FOLLOW_WALL, 800.0f, -0.4f};
// 		}
// 		else
// 		{
// 			qInfo() << "FOLLOW WALL 2";
//
// 			return {State::FOLLOW_WALL, 800.0f, 0.4f};  // Desplazamiento lateral + rotación
// 		}
// 	}
//
// 	qInfo() << "FOLLOW WALL 3";
// 	return {State::FORWARD, 1000.0f, 0.0f};
// }
//
// /**
//  * @brief Controls the robot’s movement in a spiral pattern based on lidar data.
//  *
//  * This method adjusts the robot’s movement to perform a spiral motion, gradually increasing
//  * its forward speed (`subida`) and decreasing its rotation speed (`bajada`) as long as there
//  * are no obstacles within 800 units. The robot continues to adjust these speeds as it moves in a spiral.
//  * If an obstacle is detected within 800 units, the robot will switch to the FORWARD state and move straight ahead.
//  *
//  * The method also ensures that the `subida` (forward speed) and `bajada` (rotational speed) values
//  * remain within specific limits to control the robot's motion:
//  * - `subida` increases with a fixed increment (`delta_subida`) and is clamped between 0 and 1000.
//  * - `bajada` decreases with a fixed increment (`delta_bajada`) and is clamped between 0 and 1.
//  *
//  * This behavior allows the robot to gradually expand its movement in a spiral shape while avoiding obstacles.
//  *
//  * @param points The collection of lidar points used to measure proximity to obstacles and determine movement.
//  * @return std::tuple<SpecificWorker::State, float, float> containing:
//  *         - The next movement state of the robot (SPIRAL or FORWARD).
//  *         - The forward velocity value (`subida`).
//  *         - The rotational velocity value (`bajada`).
//  */
// std::tuple<SpecificWorker::State, float, float> SpecificWorker::SPIRAL_method(const RoboCompLidar3D::TPoints& points)
// {
//
// 	qInfo() << "-----------------------------Bajada_SPIRAL: " << bajada << " subida: " << subida;
//
// 	if (points.empty())
// 	{
// 		qInfo() << __FUNCTION__ << "No points";
// 		return {State::SPIRAL, 0.0f, 0.0f};
// 	}
// 	auto min_dist = std::min_element(std::begin(points), std::end(points),[](const auto& p1, const auto& p2)
// 			{ return p1.r < p2.r; });
//
// 	constexpr float delta_subida = 3.f;
// 	constexpr float delta_bajada = 0.001f;
//
// 	qInfo() << "Punto actual: " << min_dist->r;
// 	if (min_dist -> r < 800)
// 	{
// 		return {State::FORWARD, 1000.0f, 0.0f};
// 	}
// 	else
// 	{
// 		bajada -= delta_bajada;
// 		subida += delta_subida;
// 		bajada = std::clamp(bajada, 0.f, 1.f);
// 		subida = std::clamp(subida, 0.f, 1000.f);
// 		qInfo() << "-----------------------------Bajada: " << bajada << " subida: " << subida;
// 		return {State::SPIRAL, subida, bajada };
// 	}
//
// 	return {State::FORWARD, 1000.0f, 0.0f};
//
// }

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


/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->correctOdometer(int x, int z, float alpha)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->getBasePose(int x, int z, float alpha)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->getBaseState(RoboCompGenericBase::TBaseState state)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->resetOdometer()
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->setOdometer(RoboCompGenericBase::TBaseState state)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->setOdometerPose(int x, int z, float alpha)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->setSpeedBase(float adv, float rot)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->stopBase()

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompLaser you can call this methods:
// RoboCompLaser::TLaserData this->laser_proxy->getLaserAndBStateData(RoboCompGenericBase::TBaseState bState)
// RoboCompLaser::LaserConfData this->laser_proxy->getLaserConfData()
// RoboCompLaser::TLaserData this->laser_proxy->getLaserData()

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

    // // ===============================
    // // Parámetros del controlador
    // // ===============================
    // const float Kp = 0.1;                      // Ganancia proporcional
    // static float Kd = 0.0f;                       // Ganancia derivativa
    //
    // const float vmax = 300.0f;                     // Velocidad lineal máxima (mm/s)
    //
    // const float sigma_theta = M_PI / 4.0f;         // σθ del freno gaussiano (45º)
    // const float d_stop = 500.0f;                   // Distancia para comenzar frenado (mm)
    // const float k_sigmoid = 10.0f;                 // Steepness de la sigmoide
    //
    // // Guardamos el error angular anterior para el término derivativo
    // static float prev_theta_e = 0.0f;
    // static QElapsedTimer timer;
    // if (!timer.isValid())
    //     timer.start();
    // float dt = timer.restart() / 1000.0f;          // Tiempo en segundos
    //
    //
    // // ===============================
    // // 1. Distancia al objetivo
    // // ===============================
    // float tx = target.x();
    // float ty = target.y();
    // float d = std::sqrt(tx*tx + ty*ty);
    //
    // // ===============================
    // // 2. Error angular
    // // ===============================
    // float theta_e = std::atan2(ty, tx);
    //
    // // Normalizar [-PI, PI]
    // while(theta_e >  M_PI) theta_e -= 2 * M_PI;
    // while(theta_e < -M_PI) theta_e += 2 * M_PI;
    //
    // // ===============================
    // // 3. Derivada del error angular
    // // ===============================
    // float theta_dot = (theta_e - prev_theta_e) / std::max(0.001f, dt);
    // prev_theta_e = theta_e;
    //
    // // ===============================
    // // 4. Control PD de rotación
    // // ===============================
    // float w = Kp * theta_e + Kd * theta_dot;
    //
    // // ===============================
    // // 5. Freno gaussiano (ángulo)
    // // ===============================
    // float f_theta = std::exp(-(theta_e*theta_e) / (2.0f * sigma_theta * sigma_theta));
    //
    // // ===============================
    // // 6. Freno de distancia (sigmoide)
    // // ===============================
    // float f_d = 1.0f / (1.0f + std::exp(k_sigmoid * (d - d_stop)));
    //
    // // ===============================
    // // 7. Velocidad final
    // // ===============================
    // float v = vmax * f_theta * f_d;
    //
    // return std::make_tuple(v, w);