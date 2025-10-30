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

/**
 * @brief Initializes the graphical environment and sets up the robot visualization.
 *
 * This method is called once at the start of the program to configure the main viewer and
 * prepare the graphical interface for displaying lidar data and robot movement.
 *
 * The initialization process includes the following steps:
 *
 * 1. **Viewer Setup:**
 *    Creates an `AbstractGraphicViewer` instance within the main frame using predefined world dimensions.
 *    The visualization area is defined by a rectangular region centered around the robot’s workspace.
 *
 * 2. **Window Configuration:**
 *    Resizes the main application window to the desired dimensions (900x450) and displays the viewer.
 *
 * 3. **Robot Representation:**
 *    Adds a visual representation of the robot to the scene using a blue polygon.
 *    The robot’s polygon reference (`robot_polygon`) is stored for later updates and transformations.
 *
 * 4. **Event Connection:**
 *    Connects the viewer’s mouse coordinate signal (`new_mouse_coordinates`) to the
 *    `SpecificWorker::new_target_slot` slot, enabling interaction with the viewer via mouse input.
 *
 * This setup ensures that the graphical scene and interactive components are properly initialized
 * before the main control loop (`compute()`) begins execution.
 *
 * @return void
 */
void SpecificWorker::initialize()
{
    std::cout << "initialize worker" << std::endl;

	this->dimensions = QRectF(-6000, -3000, 12000, 6000);
	this->viewer = new AbstractGraphicViewer(this->frame, this->dimensions);
	this->resize(900,450);
	viewer->show();
	const auto rob = viewer->add_robot(ROBOT_LENGTH, ROBOT_LENGTH, 0, 190, QColor("Blue"));
	robot_polygon = std::get<0>(rob);

	viewer_room = new AbstractGraphicViewer(this->frame_room, dimensions);
	auto [rr, re] = viewer_room->add_robot(params.ROBOT_LENGTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
	robot_room_draw = rr;

	// draw room in viewer_room
	viewer_room->scene.addRect(dimensions, QPen(Qt::black, 30));
	viewer_room->show();

	// initialise robot pose
	robot_pose.setIdentity();
	robot_pose.translate(Eigen::Vector2d(0.0,0.0));


	connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);
}


/**
 * @brief Main control loop that reads lidar data, processes it, and determines the robot’s motion state.
 *
 * This method serves as the central control function for the robot’s behavior. It performs the following steps:
 *
 * 1. **Lidar Data Acquisition:**
 *    Retrieves lidar data from the “helios” sensor using a predefined distance and threshold filter.
 *    If the data cannot be obtained due to a connection issue, the method exits gracefully.
 *
 * 2. **Data Filtering:**
 *    Calls `data_filter()` to reduce the 3D lidar data into a filtered 2D representation, keeping only the
 *    closest points per angle group. If filtering fails, a warning is displayed and the method returns.
 *
 * 3. **Visualization:**
 *    Draws the filtered lidar points on the scene viewer for visualization purposes.
 *
 * 4. **Behavior Selection:**
 *    Based on the current robot state, the method calls the corresponding behavior function:
 *    - `FORWARD_method()` — move forward while monitoring obstacles.
 *    - `TURN_method()` — rotate in place to avoid nearby obstacles.
 *    - `FOLLOW_WALL_method()` — follow a wall at a stable distance.
 *    - `SPIRAL_method()` — perform a spiral search when no obstacles are nearby.
 *
 * 5. **Motion Execution:**
 *    The selected behavior function returns a tuple containing the next state, forward velocity,
 *    and rotational velocity. The state is updated, and the velocities are sent to the robot’s motion controller
 *    via the `omnirobot_proxy->setSpeedBase()` method.
 *
 * If any communication errors occur during data retrieval or movement commands, the method logs the error
 * and safely terminates the current iteration.
 *
 * @note The initial state of the robot is set to `SPIRAL`. The state transitions dynamically
 *       based on sensor readings and behavior logic.
 *
 * @return void
 */
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

	//std::tuple<State,float,float> result;
	//static auto state = State::SPIRAL;  // Estado inicial
	//auto &[st, adv, rot] = state_machine(state, filter_data); // Machine states method
	//state = st;
	//try{ omnirobot_proxy->setSpeedBase(0, adv, rot);}
	//catch (const Ice::Exception &e){ std::cout << e << " " << "Conexión con Laser" << std::endl; return;}
}

std::tuple<SpecificWorker::State,float,float> SpecificWorker::state_machine(State state, const RoboCompLidar3D::TPoints &filter_data)
{
	switch (state)
	{
	case State::IDLE:
		//result = IDLE_method();
		break;
	case State::FORWARD:
		return FORWARD_method(filter_data);
		break;
	case State::TURN:
		return TURN_method(filter_data);
		break;
	case State::FOLLOW_WALL:
		return FOLLOW_WALL_method(filter_data);
		break;
	case State::SPIRAL:
		return SPIRAL_method(filter_data);
		break;
	}
	return {};
}

/**
 * @brief Visualizes lidar data on a QGraphicsScene, highlighting key points and directions.
 *
 * This method draws a visual representation of lidar readings on the provided Qt graphics scene.
 * It renders individual lidar points, highlights important reference points, and adds guiding
 * lines to help visualize the robot’s perception of its environment.
 *
 * The drawing process includes the following steps:
 *
 * 1. **Scene Reset:**
 *    Removes all previously drawn items to refresh the visualization at each iteration.
 *
 * 2. **Lidar Points Rendering:**
 *    Draws each lidar point as a small green square positioned according to its (x, y) coordinates.
 *
 * 3. **Frontal Minimum Distance Highlight:**
 *    Calculates the closest point within the frontal section of the lidar’s field of view.
 *    - If the point is closer than `params.STOP_THRESHOLD`, it is drawn in **red** (indicating an obstacle).
 *    - Otherwise, it is drawn in **magenta** (indicating safe distance).
 *
 * 4. **Wall Distance Visualization:**
 *    Identifies the closest points on the left and right sides of the robot (using lidar angle sections).
 *    The nearest of these two is highlighted with an **orange square**, and a line is drawn from the robot’s
 *    center to that point to indicate the closest wall or obstacle.
 *
 * 5. **Directional Boundaries:**
 *    Draws two colored lines (blue and red) extending from the robot at angles defined by
 *    `params.LIDAR_FRONT_SECTION`, representing the boundaries of the frontal field of view.
 *
 * The method maintains a static list of drawn items so that they can be efficiently removed
 * and redrawn in each update cycle, ensuring smooth real-time visualization.
 *
 * @param points The collection of lidar points (in 2D) to be visualized.
 * @param scene Pointer to the QGraphicsScene where lidar data and visualization elements will be drawn.
 * @return void
 */
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
std::tuple<SpecificWorker::State, float, float> SpecificWorker::FORWARD_method(const RoboCompLidar3D::TPoints& points)
{
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

/**
 * @brief Controls the robot's turning behavior based on lidar data.
 *
 * This method determines how the robot should turn when an obstacle is detected nearby.
 * It analyzes the provided lidar points to find the closest point (with the minimum distance, r value)
 * and decides whether to keep turning or transition to another movement state:
 *
 * - If no obstacle is closer than 800 units, the robot may stop turning and either:
 *   - Switch to the FOLLOW_WALL state (with 50% probability), or
 *   - Switch to the FORWARD state, adjusting its rotation direction based on the angle (phi) of the closest point.
 * - If an obstacle is still detected within 800 units, the robot continues turning in place.
 *   The direction of rotation (left or right) depends on whether the closest point's angle (phi) is negative or positive.
 *
 * The method also includes a counter (`contador_turn`) to control how long the robot stays in the TURN state.
 * If the counter exceeds a threshold (15 iterations), the robot performs a stronger turn.
 *
 * @param points The collection of lidar points used to determine proximity and turning direction.
 * @return std::tuple<SpecificWorker::State, float, float> containing:
 *         - The next movement state of the robot (TURN, FOLLOW_WALL, or FORWARD).
 *         - The linear velocity value.
 *         - The rotational velocity value.
 */
std::tuple<SpecificWorker::State, float, float> SpecificWorker::TURN_method(const RoboCompLidar3D::TPoints& points)
{
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
			return {State::FOLLOW_WALL, 0.0f, 0.0f};
		}
		else
		{
			qInfo() << "CHANGE FROM TURN TO FORWARD";
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

}

/**
 * @brief Controls the robot’s behavior while following a wall using lidar data.
 *
 * This method adjusts the robot’s movement to maintain a consistent distance from a wall or obstacle.
 * It analyzes the provided lidar points to find the closest point (with the minimum distance, r value)
 * and decides how to proceed based on that distance:
 *
 * - If the closest point is between 770 and 810 units away, the robot maintains a stable forward motion
 *   while continuing to follow the wall.
 * - If the closest point is farther than 810 units, the robot adjusts its trajectory to move closer to the wall:
 *   - If the closest point’s angle (phi) is negative, it turns slightly right.
 *   - If the angle (phi) is positive, it turns slightly left.
 * - If none of these conditions are met, the robot transitions back to the FORWARD state to continue moving straight.
 *
 * This behavior helps the robot navigate parallel to obstacles, maintaining an optimal distance for safe movement.
 *
 * @param points The collection of lidar points used to measure the robot’s distance from nearby walls.
 * @return std::tuple<SpecificWorker::State, float, float> containing:
 *         - The next movement state of the robot (FOLLOW_WALL or FORWARD).
 *         - The linear velocity value.
 *         - The rotational velocity value.
 */
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

	// What I do if I Stay
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

/**
 * @brief Controls the robot’s movement in a spiral pattern based on lidar data.
 *
 * This method adjusts the robot’s movement to perform a spiral motion, gradually increasing
 * its forward speed (`subida`) and decreasing its rotation speed (`bajada`) as long as there
 * are no obstacles within 800 units. The robot continues to adjust these speeds as it moves in a spiral.
 * If an obstacle is detected within 800 units, the robot will switch to the FORWARD state and move straight ahead.
 *
 * The method also ensures that the `subida` (forward speed) and `bajada` (rotational speed) values
 * remain within specific limits to control the robot's motion:
 * - `subida` increases with a fixed increment (`delta_subida`) and is clamped between 0 and 1000.
 * - `bajada` decreases with a fixed increment (`delta_bajada`) and is clamped between 0 and 1.
 *
 * This behavior allows the robot to gradually expand its movement in a spiral shape while avoiding obstacles.
 *
 * @param points The collection of lidar points used to measure proximity to obstacles and determine movement.
 * @return std::tuple<SpecificWorker::State, float, float> containing:
 *         - The next movement state of the robot (SPIRAL or FORWARD).
 *         - The forward velocity value (`subida`).
 *         - The rotational velocity value (`bajada`).
 */
std::tuple<SpecificWorker::State, float, float> SpecificWorker::SPIRAL_method(const RoboCompLidar3D::TPoints& points)
{

	qInfo() << "-----------------------------Bajada_SPIRAL: " << bajada << " subida: " << subida;

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