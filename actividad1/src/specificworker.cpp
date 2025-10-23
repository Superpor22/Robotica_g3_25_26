/*
 *    Copyright (C) 2025 by YOUR NAME HERE
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
		
		// Example statemachine:
		/***
		//Your definition for the statesmachine (if you dont want use a execute function, use nullptr)
		states["CustomState"] = std::make_unique<GRAFCETStep>("CustomState", period, 
															std::bind(&SpecificWorker::customLoop, this),  // Cyclic function
															std::bind(&SpecificWorker::customEnter, this), // On-enter function
															std::bind(&SpecificWorker::customExit, this)); // On-exit function

		//Add your definition of transitions (addTransition(originOfSignal, signal, dstState))
		states["CustomState"]->addTransition(states["CustomState"].get(), SIGNAL(entered()), states["OtherState"].get());
		states["Compute"]->addTransition(this, SIGNAL(customSignal()), states["CustomState"].get()); //Define your signal in the .h file under the "Signals" section.

		//Add your custom state
		statemachine.addState(states["CustomState"].get());
		***/

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}
	}
}

SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

void SpecificWorker::initialize()
{
    std::cout << "initialize worker" << std::endl;

	this->dimensions = QRectF(-6000, -3000, 12000, 6000);
	this->viewer = new AbstractGraphicViewer(this->frame, this->dimensions);
	this->resize(900,450);
	viewer->show();
	const auto rob = viewer->add_robot(ROBOT_LENGTH, ROBOT_LENGTH, 0, 190, QColor("Blue"));
	robot_polygon = std::get<0>(rob);

	connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);


    //initializeCODE

    /////////GET PARAMS, OPEND DEVICES....////////
    //int period = configLoader.get<int>("Period.Compute") //NOTE: If you want get period of compute use getPeriod("compute")
    //std::string device = configLoader.get<std::string>("Device.name") 

}

void SpecificWorker::compute()
{

	// Read data from lidar
	// data = read_data("helios");
	RoboCompLidar3D::TData data;
	try { data = lidar3d_proxy->getLidarDataWithThreshold2d("helios", 15000, 1);}
	catch (const Ice::Exception &e){ std::cout << e << " " << "Conexión con Laser" << std::endl; return;}

	// filter data from 3D to 2D
	RoboCompLidar3D::TPoints filter_data;
	if (const auto filter_data_= data_filter(data.points); filter_data_.has_value())
		filter_data = filter_data_.value();
	else
	{	qWarning() << "filter_data_.has_value()"; return;}

	draw_lidar(filter_data, &viewer->scene);

	std::tuple<State,float,float> result;
	static auto state = State::SPIRAL;  // Estado inicial, por ejemplo
	switch (state)
	{
	case State::IDLE:
		//result = IDLE_method();
		break;
	case State::FORWARD:
		result = FORWARD_method(filter_data);
		break;
	case State::TURN:
		result = TURN_method(filter_data);
		break;
	case State::FOLLOW_WALL:
		result = FOLLOW_WALL_method(filter_data);
		break;
	case State::SPIRAL:
		result = SPIRAL_method(filter_data);
		break;
	}

	auto &[st, adv, rot] = result;
	state = st;
	qInfo() << "despues de asdfads" << adv << rot ;
	//state = std::get<0>(result);
	//float adv = std::get<1>(result);
	//float rot = std::get<2>(result);
	try{ omnirobot_proxy->setSpeedBase(0, adv, rot);}
	catch (const Ice::Exception &e){ std::cout << e << " " << "Conexión con Laser" << std::endl; return;}
}

// void SpecificWorker::draw_lidar(const  RoboCompLidar3D::TPoints &points, QGraphicsScene* scene)
// {
// 	static std::vector<QGraphicsItem*> draw_points;
// 	for (const auto &p : draw_points)
// 	{
// 		scene->removeItem(p);
// 		delete p;
// 	}
// 	draw_points.clear();
//
// 	const QColor color("LightGreen");
// 	const QPen pen(color, 10);
// 	//const QBrush brush(color, Qt::SolidPattern);
// 	for (const auto &p : points)
// 	{
// 		const auto dp = scene->addRect(-25, -25, 50, 50, pen);
// 		dp->setPos(p.x, p.y);
// 		draw_points.push_back(dp);   // add to the list of points to be deleted next time
// 	}
// }

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

    // compute and draw minimum distance point in frontal range
    auto offset_begin = closest_lidar_index_to_given_angle(points, -params.LIDAR_FRONT_SECTION);
    auto offset_end = closest_lidar_index_to_given_angle(points, params.LIDAR_FRONT_SECTION);
    if(not offset_begin or not offset_end)
    { std::cout << offset_begin.error() << " " << offset_end.error() << std::endl; return ;}    // abandon the ship
    auto min_point = std::min_element(std::begin(points) + offset_begin.value(), std::begin(points) + offset_end.value(), [](auto &a, auto &b)
    { return a.distance2d < b.distance2d; });
    QColor dcolor;
    if(min_point->distance2d < params.STOP_THRESHOLD)
        dcolor = QColor(Qt::red);
    else
        dcolor = QColor(Qt::magenta);
    auto ditem = scene->addRect(-100, -100, 200, 200, dcolor, QBrush(dcolor));
    ditem->setPos(min_point->x, min_point->y);
    items.push_back(ditem);

    // compute and draw minimum distance point to wall
    auto wall_res_right = closest_lidar_index_to_given_angle(points, params.LIDAR_RIGHT_SIDE_SECTION);
    auto wall_res_left = closest_lidar_index_to_given_angle(points, params.LIDAR_LEFT_SIDE_SECTION);
    if(not wall_res_right or not wall_res_left)   // abandon the ship
    {
        qWarning() << "No valid lateral readings" << QString::fromStdString(wall_res_right.error()) << QString::fromStdString(wall_res_left.error());
        return;
    }
    auto right_point = points[wall_res_right.value()];
    auto left_point = points[wall_res_left.value()];
    // compare both to get the one with minimum distance
    auto min_obj = (right_point.distance2d < left_point.distance2d) ? right_point : left_point;
    auto item = scene->addRect(-100, -100, 200, 200, QColor(QColorConstants::Svg::orange), QBrush(QColor(QColorConstants::Svg::orange)));
    item->setPos(min_obj.x, min_obj.y);
    items.push_back(item);
    // draw a line from the robot to the minimum distance point
    auto item_line = scene->addLine(QLineF(QPointF(0.f, 0.f), QPointF(min_obj.x, min_obj.y)), QPen(QColorConstants::Svg::orange, 10));
    items.push_back(item_line);

    // Draw two lines coming out from the robot at angles given by params.LIDAR_OFFSET
    // Calculate the end points of the lines
auto res_right = closest_lidar_index_to_given_angle(points, params.LIDAR_FRONT_SECTION);
auto res_left = closest_lidar_index_to_given_angle(points, -params.LIDAR_FRONT_SECTION);
    if(not res_right or not res_left)
    { std::cout << res_right.error() << " " << res_left.error() << std::endl; return ;}
    // draw two lines at the edges of the range
    float right_line_length = points[res_right.value()].distance2d;
    float left_line_length = points[res_left.value()].distance2d;
    float angle1 = points[res_left.value()].phi;
    float angle2 = points[res_right.value()].phi;
    QLineF line_left{QPointF(0.f, 0.f),
                     robot_polygon->mapToScene(left_line_length * sin(angle1), left_line_length * cos(angle1))};
    QLineF line_right{QPointF(0.f, 0.f),
                      robot_polygon->mapToScene(right_line_length * sin(angle2), right_line_length * cos(angle2))};
    QPen left_pen(Qt::blue, 10); // Blue color pen with thickness 3
    QPen right_pen(Qt::red, 10); // Blue color pen with thickness 3
    auto line1 = scene->addLine(line_left, left_pen);
    auto line2 = scene->addLine(line_right, right_pen);
    items.push_back(line1);
    items.push_back(line2);
}

/**
 * @brief Calculates the index of the closest lidar point to the given angle.
 *
 * This method searches through the provided std::list of lidar points and finds the point
 * whose angle (phi value) is closest to the specified angle. If a matching point is found,
 * the index of the point in the std::list is returned. If no point is found that matches the condition,
 * an error message is returned.
 *
 * @param points The collection of lidar points to search through.
 * @param angle The target angle to find the closest matching point.
 * @return std::expected<int, std::string> containing the index of the closest lidar point if found,
 * or an error message if no such point exists.
 */
std::expected<int, std::string> SpecificWorker::closest_lidar_index_to_given_angle(const  RoboCompLidar3D::TPoints &points, float angle)
{
	// search for the point in points whose phi value is closest to angle
	auto res = std::ranges::find_if(points, [angle](auto &a){ return a.phi > angle;});
	if(res != std::end(points))
		return std::distance(std::begin(points), res);
	else
		return std::unexpected("No closest value found in method <closest_lidar_index_to_given_angle>");
}


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

std::optional<RoboCompLidar3D::TPoints> SpecificWorker::get_min_distance(const RoboCompLidar3D::TPoints& points)
{
	if (points.empty()) return {};

	RoboCompLidar3D::TPoints salida;
	salida.reserve(points.size());
	for (auto&& [angle, group] : iter::groupby(points, [](const auto& p)
		{float multiplier = std::pow(10.0f, 2); return std::floor(p.phi * multiplier) / multiplier; }))
	{
		auto min = std::min_element(std::begin(group), std::end(group),[](const auto& p1, const auto& p2)
			{ return p1.r < p2.r; });
		//if (min->phi > -std::numbers::pi / 2 && min->phi < std::numbers::pi / 2)
			salida.emplace_back(*min);
	}
	//std::sort(salida.begin(), salida.end(),
	//	[](const auto& a, const auto& b) { return a.r < b.r; });

	return salida;
	// auto res = std::ranges::views::filter(points, [](const auto& p){return p.phi> -M_PI_2 and p.phi < M_PI_2;});
	// return std::ranges::min_element(res,[](const auto &a, const auto &b)
	// 	{ return std::hypot(a.x, a.y) < std::hypot(b.x, b.y); }
	// )->r;
}

std::tuple<SpecificWorker::State, float, float> SpecificWorker::FORWARD_method(const RoboCompLidar3D::TPoints& points)
{
	/// exit condition first
	//const int offset = points.size()/2 -15;
	auto min_dist = std::min_element(std::begin(points), std::end(points),[](const auto& p1, const auto& p2)
			{ return p1.r < p2.r; });

	qInfo() << "Punto actual: " << min_dist->r;

	if (min_dist->r < 800)  // Objeto cerca → gira
	{
		qInfo() << "CHANGE FROM FORWARD TO TURN";
		return {State::TURN, 0.0f, 0.0f};
	}
	if (min_dist->r > 1100)
	{
		bajada = 0.6f;
		subida = 1000.f;
		qInfo() << "-----------------------------Bajada_FORWARD: " << bajada << " subida: " << subida;
		return {State::SPIRAL, 0.0f, 0.0f};
	}

	/// What I do when I stay
	return {State::FORWARD, 1000.0f, 0.0f};

}

std::tuple<SpecificWorker::State, float, float> SpecificWorker::TURN_method(const RoboCompLidar3D::TPoints& points)
{
	// exit condition

	/// exit condition first
	//const int offset = points.size()/2 -15;
	auto min_dist = std::min_element(std::begin(points), std::end(points),[](const auto& p1, const auto& p2)
			{ return p1.r < p2.r; });

	// Si ya no hay obstáculo cerca, volvemos a FORWARD
	if (min_dist->r > 800)
	{
		contador_turn = 0;
		int r = std::rand() % 10;
		qInfo() << "-------------------R: " << r;
		if (r < 5)
		{
			qInfo() << "CHANGE FROM TURN TO FOLLOW WALL";
			//return {State::FORWARD, 1000.0f, 0.0f};  // Podemos avanzar
			return {State::FOLLOW_WALL, 0.0f, 0.0f};
		}
		else
		{
			qInfo() << "CHANGE FROM TURN TO FORWARD";
			//return {State::FORWARD, 1000.0f, 0.0f};
			if (min_dist->phi < 0)
				return {State::FORWARD, 1000.0f, 0.5f};
			else
				return {State::FORWARD, 1000.0f, -0.5f};
		}
	}

	qInfo() << "CONTINUE TURNING";

	//What I do if I Stay
	contador_turn++;
	if (contador_turn > 15)
	{
		return {State::TURN, 0.0f, 1.0f};
	}
	qInfo() << "------------------ Menor ángulo: " << min_dist->phi;
	if (min_dist->phi < 0)
		return {State::TURN, 0.0f, 0.8f};
	else
		return {State::TURN, 0.0f, -0.8f};  // Desplazamiento lateral + rotación

	//return {State::TURN, 0.0f, 0.6f};

}

std::tuple<SpecificWorker::State, float, float> SpecificWorker::FOLLOW_WALL_method(const RoboCompLidar3D::TPoints& points)
{

	auto min_dist = std::min_element(std::begin(points), std::end(points),[](const auto& p1, const auto& p2)
		{ return p1.r < p2.r; });

	// Si ya no hay obstáculo cerca, volvemos a FORWARD
	if (min_dist->r > 770 and min_dist->r < 810)
	{
		qInfo() << "CHANGE FROM FOLLOW WALL TO FORWARD";
		return {State::FOLLOW_WALL, 1000.0f, 0.0f};  // Podemos avanzar
	}

	qInfo() << "CONTINUE FOLLOWING WALL";

	//What I do if I Stay
	if (min_dist->r > 810)
	{
		if (min_dist->phi < 0)
		{
			qInfo() << "FOLLOW WALL 1";

			return {State::FOLLOW_WALL, 800.0f, -0.4f};
		}
		else
		{
			qInfo() << "FOLLOW WALL 2";

			return {State::FOLLOW_WALL, 800.0f, 0.4f};  // Desplazamiento lateral + rotación
		}
	}

	qInfo() << "FOLLOW WALL 3";
	return {State::FORWARD, 1000.0f, 0.0f};
}

std::tuple<SpecificWorker::State, float, float> SpecificWorker::SPIRAL_method(const RoboCompLidar3D::TPoints& points)
{

	qInfo() << "-----------------------------Bajada_SPIRAL: " << bajada << " subida: " << subida;

	/// exit condition first
	//const int offset = points.size()/2 -15;
	auto min_dist = std::min_element(std::begin(points), std::end(points),[](const auto& p1, const auto& p2)
			{ return p1.r < p2.r; });

	constexpr float delta_subida = 3.f;
	constexpr float delta_bajada = 0.001f;

	qInfo() << "Punto actual: " << min_dist->r;
	if (min_dist -> r < 800)
	{
		return {State::FORWARD, 1000.0f, 0.0f};
	}
	else
	{
		bajada -= delta_bajada;
		subida += delta_subida;
		bajada = std::clamp(bajada, 0.f, 1.f);
		subida = std::clamp(subida, 0.f, 1000.f);
		qInfo() << "-----------------------------Bajada: " << bajada << " subida: " << subida;
		return {State::SPIRAL, subida, bajada };
	}

	return {State::FORWARD, 1000.0f, 0.0f};

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

void SpecificWorker::new_target_slot(QPointF punto)
{
	qInfo() << punto;
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


// int i = data.size()/2 -5;
//
// auto data2 = data[i];
//
// for(auto x = data.begin() + i; x < data.end() - i; x++)
// {
// 	std::cout << x->dist << std::endl;
// }
//
// std::cout << "-----------------------------------" << std::endl;