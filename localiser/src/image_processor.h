//
// Created by pbustos on 12/11/25.
//

// C++
#pragma once

#include <tuple>
#include <opencv2/opencv.hpp>
#include <QLabel>
#include <QImage>
#include <QPixmap>
#include <cmath>
#include <Camera360RGB.h>

namespace rc
{
    struct ImageProcessor
    {
        // Detect a large red patch in the given BGR image.
        // - img: input image in BGR color order (OpenCV default)
        // - label_img: optional QLabel to update for visualization (can be nullptr)
        // - min_nonzero: minimum number of red pixels required to consider detection valid
        // Returns: (detected, room_index, left_right) where left_right = -1 (left) or 1 (right)

        static std::tuple<bool, int, int> check_colour_patch_in_image(RoboCompCamera360RGB::Camera360RGBPrxPtr proxy,
                                                                      QLabel *label_img = nullptr,
                                                                      int min_nonzero = 1000)
        {
            RoboCompCamera360RGB::TImage img;
            try{ img = proxy->getROI(-1, -1, -1, -1, -1, -1);}
            catch (const Ice::Exception &e){ std::cout << e.what() << " Error reading 360 camera " << std::endl; return {false, -1, 1}; }

            // convert to cv::Mat
            cv::Mat cv_img(img.height, img.width, CV_8UC3, img.image.data());

            // extract a ROI leaving out borders (same as original logic)
            const int left_offset = cv_img.cols / 8;
            const int vert_offset = cv_img.rows / 4;
            const cv::Rect roi(left_offset, vert_offset, cv_img.cols - 2 * left_offset, cv_img.rows - 2 * vert_offset);
            if (roi.width <= 0 || roi.height <= 0) return {false, -1, 1};
            cv_img = cv_img(roi);

            // Convert BGR -> RGB for display
            cv::Mat display_img;
            cv::cvtColor(cv_img, display_img, cv::COLOR_BGR2RGB);

            if (label_img)
            {
                QImage qimg(display_img.data, display_img.cols, display_img.rows, static_cast<int>(display_img.step), QImage::Format_RGB888);
                label_img->setPixmap(QPixmap::fromImage(qimg).scaled(label_img->size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation));
            }

            // Convert BGR -> HSV for color thresholding
            cv::Mat hsv_img;
            cv::cvtColor(cv_img, hsv_img, cv::COLOR_BGR2HSV);

            // Set ranges depending on color
            cv::Mat mask_green, mask_red_1, mask_red_2 = cv::Mat::zeros(hsv_img.size(), CV_8UC1);
            cv::inRange(hsv_img, cv::Scalar(35, 50, 50), cv::Scalar(85, 255, 255), mask_green);

            cv::inRange(hsv_img, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask_red_1);
            cv::inRange(hsv_img, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), mask_red_2);
            cv::Mat mask_red = mask_red_1 | mask_red_2;

            // remove small noise
            cv::morphologyEx(mask_green, mask_green, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));
            cv::morphologyEx(mask_red, mask_red, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));

            const int nonZeroCount_green = cv::countNonZero(mask_green);
            const int nonZeroCount_red = cv::countNonZero(mask_red);
            if (nonZeroCount_green < min_nonzero and nonZeroCount_red < min_nonzero)
                return {false, -1, 1};

            // get larger mask
            cv::Mat mask = (nonZeroCount_red > nonZeroCount_green) ? mask_red : mask_green;
            int room_index = (nonZeroCount_red > nonZeroCount_green) ? 0 : 1;

            // compute moments and center of red patch
            const cv::Moments mu = cv::moments(mask, true);
            if (mu.m00 < 1.0) return {false, -1, 1};

            cv::Point2f bestCenter(static_cast<float>(mu.m10 / mu.m00), static_cast<float>(mu.m01 / mu.m00));

            // decide turning direction: default right (1), left (-1)
            int left_right = 1;
            if (bestCenter.x < (display_img.cols / 2) && bestCenter.x > 0)
                left_right = -1;

            // check center is near middle of image (tolerance)
            const int tolerance = display_img.cols / 10;
            const int left_bound = display_img.cols / 2 - tolerance;
            const int right_bound = display_img.cols / 2 + tolerance;
            if ((bestCenter.x < left_bound) || (bestCenter.x > right_bound))
                return {false, -1, left_right};

            // draw marker on detected center and update label if provided
            cv::circle(display_img, bestCenter, 40, cv::Scalar(0, 255, 0), -1);
            if (label_img)
            {
                QImage qimg(display_img.data, display_img.cols, display_img.rows, static_cast<int>(display_img.step), QImage::Format_RGB888);
                label_img->setPixmap(QPixmap::fromImage(qimg).scaled(label_img->size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation));
            }

            return {true, room_index, left_right};
        }
        static std::string room_name_from_index(int index)
        {
            switch(index)
            {
                case 0: return "RED";
                case 1: return "GREEN";
                case 2: return "BLUE";
                case 3: return "YELLOW";
                default: return "UNKNOWN";
            }
        }
    };
}

// SpecificWorker::RetVal SpecificWorker::turn(const Corners &corners)
// {
//     const auto &[success, room_index, left_right] = image_processor.check_colour_patch_in_image(camera360rgb_proxy, this->label_img);
//     if (success)
//     {
//         current_room = room_index;
//         const auto m = hungarian.match(corners,nominal_rooms[current_room].corners() );
//         if (m.empty())
//         {
//             qInfo() << __FUNCTION__ << "empty match";
//         };
//         if (m.size() < 3)
//         {
//             qInfo() << __FUNCTION__ << "m size < 3";
//             return{STATE::TURN, 0.0f, left_right*params.RELOCAL_ROT_SPEED};
//         }
//         const auto max_error_iter = std::ranges::max_element(m, [](const auto &a, const auto &b)
//                                 { return std::get<2>(a) < std::get<2>(b); });
//         if (const auto max_match_error = std::get<2>(*max_error_iter); max_match_error > params.RELOCAL_DONE_MATCH_MAX_ERROR)
//         {
//             qInfo() << __FUNCTION__ << "match error > " << params.RELOCAL_DONE_MATCH_MAX_ERROR;
//             return{STATE::TURN, 0.0f, left_right*params.RELOCAL_ROT_SPEED};
//         }
//         // update robot pose to have a fresh value
//         update_robot_pose(corners, m);
//
//         ///////////////////////////////////////////////////////////////////////
//         // save doors to nominal_room
//         auto doors = door_detector.doors();
//         if (doors.empty()) { qWarning() << __FUNCTION__ << "empty doors"; return{STATE::TURN, 0.0f, left_right*params.RELOCAL_ROT_SPEED};}
//         for (auto &d : doors)
//         {
//             d.p1_global = nominal_rooms[current_room].get_projection_of_point_on_closest_wall(robot_pose * d.p1);
//             d.p2_global = nominal_rooms[current_room].get_projection_of_point_on_closest_wall(robot_pose * d.p2);
//         }
//         nominal_rooms[current_room].doors = doors;
//         // choose door to go
//         current_door = 0; // TODO: more sophisticated choice
//         // we need to match the current selected nominal door to the successive local doors detected during the approach
//         // select the local door closest to the selected nominal door
//         const auto dn = nominal_rooms[current_room].doors[current_door];
//         const auto ds = door_detector.doors();
//         const auto sd = std::ranges::min_element(ds, [dn, this](const auto &a, const auto &b)
//                 {  return (a.center() - robot_pose.inverse() * dn.center_global()).norm() <
//                           (b.center() - robot_pose.inverse() * dn.center_global()).norm(); });
//         // sd is the closest local door to the selected nominal door. Update nominal door with local values
//         nominal_rooms[current_room].doors[current_door].p1 = sd->p1;
//         nominal_rooms[current_room].doors[current_door].p2 = sd->p2;
//         draw_nominal_room(current_room, &viewer_room->scene);
//         draw_nominal_doors(current_room, current_door, &viewer_room->scene);
//         localised = true;
//         return {STATE::GOTO_DOOR, 0.0f, 0.0f};  // SUCCESS
//     }
//     // continue turning
//     return {STATE::TURN, 0.0f, left_right*params.RELOCAL_ROT_SPEED};
// }

// SpecificWorker::RetVal SpecificWorker::goto_door(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
// {
//     Doors doors;
//     // Exit conditions
//     if ( doors = door_detector.doors(); doors.empty())
//     {
//         qInfo() << __FUNCTION__ << "No doors detected, switching to UPDATE_POSE";
//         return {STATE::GOTO_DOOR, 0.f, 0.f};  // TODO: keep moving for a while?
//     }
//     // select from doors, the one closest to the nominal door
//     Door target_door;
//     if (localised)
//     {
//         qInfo() << __FUNCTION__ << "Localised, selecting door closest to nominal door";
//         const auto dn = nominal_rooms[current_room].doors[current_door];
//         const auto sd = std::ranges::min_element(doors, [dn, this](const auto &a, const auto &b)
//                {  return (a.center() - robot_pose.inverse() * dn.center_global()).norm() <
//                          (b.center() - robot_pose.inverse() * dn.center_global()).norm(); });
//         target_door = *sd;
//     }
//     else  // select the one closest to the robot's heading direction
//     {
//         qInfo() << __FUNCTION__ << "Not localised, selecting door closest to robot heading";
//         const auto sd = std::ranges::min_element(doors, [](const auto &a, const auto &b)
//                {  return abs(a.p1_angle) < abs(b.p1_angle); });
//         target_door = *sd;
//     }
//     qInfo() << target_door.p1.x() << target_door.p1.y();
//
//     // distance to target is less than threshold, stop and switch to ORIENT_TO_DOOR
//     constexpr float offset = 600.f;
//     const auto target = target_door.center_before(robot_pose.translation(), offset);
//     const auto dist_to_door = target.norm();
//
//     // draw target
//     static QGraphicsItem *door_target_draw = nullptr;
//     if (door_target_draw != nullptr)
//         scene->removeItem(door_target_draw);
//     door_target_draw = scene->addEllipse(-50, -50, 100, 100, QPen(Qt::magenta), QBrush(Qt::magenta));
//     door_target_draw->setPos(target.x(), target.y());
//
//    // Exit condition
//     if (dist_to_door < params.DOOR_REACHED_DIST)
//     {
//         qInfo() << __FUNCTION__ << "Door reached at distance " << dist_to_door << ", switching to ORIENT_TO_DOOR";
//         return {STATE::ORIENT_TO_DOOR, 0.f, 0.f};
//     }
//
//     qInfo() << __FUNCTION__ << "moving to door at " << target.x() << "," << target.y() << " dist: " << dist_to_door;
//     const auto &[adv, rot] = robot_controller(target); // go to first detected door
//     return {STATE::GOTO_DOOR, adv, rot};
// }

//connect to room y connect to door nuevos atributos de las puertas
//connect to room-> habitación con la que conecta
//connect to door-> puerta de la siguiente habitación con la que conecta
//ambos atributos son enteros