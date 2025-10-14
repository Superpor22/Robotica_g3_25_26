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
	viewer = new AbstractGraphicViewer(this->frame, this->dimensions);
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

	//Lectura de datos del lidar



	try
	{
		// read data
		auto data = lidar3d_proxy->getLidarDataWithThreshold2d("helios", 15000, 1);
		qInfo() << data.points.size();
		auto& puntos = data.points;  // Usamos `puntos` como el contenedor con los datos obtenidos
		const auto filter_data= data_filter(puntos);
		//draw_lidar(filter_data.value(), &viewer->scene);

		std::tuple<State,float,float> result;

		switch (state)
		{
		case State::IDLE:
			//result = IDLE_method();
			break;
		case State::FORWARD:
			result = FORWARD_method(filter_data);
			break;
		case State::TURN:
			result = TURN_method();
			break;
		case State::FOLLOW_WALL:
			//result = FOLLOW_WALL_method();
			break;
		case State::SPIRAL:
			//result = SPIRAL_method();
			break;
		}

		state = std::get<0>(result);
		float adv = std::get<1>(result);
		float rot = std::get<2>(result);


			if (not filter_data.has_value())
			{
				state = State::IDLE;
				adv = 0.0f;
				rot = 0.0f; //return {State::IDLE, 0.0f, 0.0f};
			} else {
				const auto &points = filter_data.value();
				draw_lidar(points, &viewer->scene);

				auto min_dist = get_min_distance(points);
				if (not min_dist.has_value()){qWarning()<< "No hay puntos mínimos"; }//return{};}
				for (auto i: iter::range(10))
					printf("----------------Minima distancia: %f\n", min_dist.value()[i].r);

				if (min_dist.value()[0].r < 700)
				{
					state = State::TURN;
					adv = 0.0f;
					rot = 0.6f;
				}
				else
				{
					state = State::FORWARD;
					adv = 1000.0f;
					rot = 0.0f;
				}
				// 	// Objeto cerca → gira
				// 	//return {State::TURN, 0.0f, 0.6f};
				//
				// 	// Movimiento hacia adelante
				// 	//return {State::FORWARD, 1000.0f, 0.0f};
				// 	state = State::FORWARD;
				// 	adv = 1000.0f;
				// 	rot = 0.0f;
			}

	} catch (const Ice::Exception &e)
	{
		std::cout << e << " " << "Conexión con Laser" << std::endl;
		//std::cout << e << std::endl;
		//return {State::IDLE, 0.0f, 0.0f};
	}


	printf("------------------Estado actual: %d\n", std::get<0>(result));
	//
	// state = std::get<0>(result);
	// float adv = std::get<1>(result);
	// float rot = std::get<2>(result);

	try
	{
		omnirobot_proxy->setSpeedBase(0, adv, rot);  // Asumiendo robot omnidireccional
	}catch (const Ice::Exception &e){}


}

void SpecificWorker::draw_lidar(const  RoboCompLidar3D::TPoints &points, QGraphicsScene* scene)
{
	static std::vector<QGraphicsItem*> draw_points;
	for (const auto &p : draw_points)
	{
		scene->removeItem(p);
		delete p;
	}
	draw_points.clear();

	const QColor color("LightGreen");
	const QPen pen(color, 10);
	//const QBrush brush(color, Qt::SolidPattern);
	for (const auto &p : points)
	{
		const auto dp = scene->addRect(-25, -25, 50, 50, pen);
		dp->setPos(p.x, p.y);
		draw_points.push_back(dp);   // add to the list of points to be deleted next time
	}
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
		//salida.emplace_back(RoboCompLidar3D::TPoint{.x = min_r->x, .y = min_r->y, .phi = min_r->phi});
		float r = std::hypot(min_r->x, min_r->y);
		salida.emplace_back(RoboCompLidar3D::TPoint{.x = min_r->x, .y = min_r->y, .phi = min_r->phi, .r = r});

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
		if (min->phi > -std::numbers::pi / 2 && min->phi < std::numbers::pi / 2)
			salida.emplace_back(*min);
	}
	std::sort(salida.begin(), salida.end(),
		[](const auto& a, const auto& b) { return a.r < b.r; });

	return salida;
	// auto res = std::ranges::views::filter(points, [](const auto& p){return p.phi> -M_PI_2 and p.phi < M_PI_2;});
	// return std::ranges::min_element(res,[](const auto &a, const auto &b)
	// 	{ return std::hypot(a.x, a.y) < std::hypot(b.x, b.y); }
	// )->r;
}

std::tuple<State, float, float> SpecificWorker::FORWARD_method(const auto &points)
{
	try
	{
		//std::min_element(points.begin()+offset, points.end()-offset,[](){});
		// auto data = lidar3d_proxy->getLidarDataWithThreshold2d("helios", 15000, 1);
		// // const auto filtered = data_filter(data.points);
		// if (not filtered.has_value()) return {State::IDLE, 0.0f, 0.0f};
		//
		// const auto &points = filtered.value();
		// draw_lidar(points, &viewer->scene);
		//
		// auto min_dist = get_min_distance(points);
		// if (not min_dist.has_value()){qWarning()<< "No hay puntos mínimos"; return{};}
		//
		// printf("----------------Minima distancia: %f\n", min_dist.value());

		if (min_dist.value() < 400)  // Objeto cerca → gira
			return {State::TURN, 0.0f, 0.6f};

		// Movimiento hacia adelante
		return {State::FORWARD, 1000.0f, 0.0f};
	}
	catch(const Ice::Exception &e)
	{
		std::cout << e << std::endl;
		return {State::IDLE, 0.0f, 0.0f};
	}
}

std::tuple<State, float, float> SpecificWorker::TURN_method()
{
	try
	{
		auto data = lidar3d_proxy->getLidarDataWithThreshold2d("helios", 15000, 1);
		const auto filtered = data_filter(data.points);
		if (!filtered.has_value()) return {State::IDLE, 0.0f, 0.0f};

		const auto &points = filtered.value();
		draw_lidar(points, &viewer->scene);

		auto min_dist = get_min_distance(points);
		if (not min_dist.has_value()){qWarning()<< "No hay puntos mínimos"; return{};}
		qInfo() << "TURN - Distancia mínima: " << min_dist.value();

		// Si ya no hay obstáculo cerca, volvemos a FORWARD
		if (min_dist.value() > 500)
			return {State::FORWARD, 1000.0f, 0.0f};  // Podemos avanzar

		// Si sigue habiendo obstáculo, seguir girando
		return {State::TURN, 0.0f, 0.7f};  // Desplazamiento lateral + rotación
	}
	catch(const Ice::Exception &e)
	{
		std::cout << e << std::endl;
		return {State::IDLE, 0.0f, 0.0f};
	}
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