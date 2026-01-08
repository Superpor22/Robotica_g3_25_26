//
// Created by pbustos on 11/11/25.
//

#include "door_detector.h"
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/combinations.hpp>
#include <QGraphicsItem>


Doors DoorDetector::detect(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    Peaks peaks;
    for (const auto &p: points | iter::sliding_window(2))
    {
        const auto &p1 = p[0];
        const auto &p2 = p[1];
        auto d = abs(p2.distance2d - p1.distance2d);
        const auto &shorter = (p1.distance2d < p2.distance2d) ? p1 : p2;
        if (d > 700)
            peaks.emplace_back(Eigen::Vector2f{shorter.x, shorter.y}, shorter.phi);
    }

    // Non-maximum suppression of peaks: remove peaks closer than 500mm
    Peaks nms_peaks;
    for (const auto &[p, a] : peaks)
        if ( const bool too_close = std::ranges::any_of(nms_peaks, [&p](const auto &p2)
            { return (p - std::get<0>(p2)).norm() < 500.f; }); not too_close)
            nms_peaks.emplace_back(p, a);
    peaks = nms_peaks;

    // doors
    Doors doors;
    for (const auto &c : iter::combinations(peaks, 2))
    {
        const auto &[p1, phi1] = c[0];
        const auto &[p2, phi2] = c[1];
        const float dist = (p1 - p2).norm();
        if (600 < dist and dist < 1200)
        {
            doors.emplace_back(Door(p1, phi1, p2, phi2));
        }
    }
    doors_cache = doors;
    return doors;
}

// Method to use the Doors vector to filter out the LiDAR points that come from a room outside the current one
RoboCompLidar3D::TPoints DoorDetector::filter_points(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    const auto doors = detect(points, scene);
    if(doors.empty()) return points;

    // for each door, check if the distance from the robot to each lidar point is smaller than the distance from the robot to the door
    RoboCompLidar3D::TPoints filtered;
    for(const auto &p : points)
    {
        bool filtered_out = false;

        for(const auto &d : doors)
        {
            const float dist_to_door = d.center().norm();
            const bool angle_wraps = d.p2_angle < d.p1_angle;

            // Determine if point is within the door's angular range
            bool point_in_angular_range;
            if (angle_wraps)
            {
                // If the range wraps around, point is in range if it's > p1_angle OR < p2_angle
                point_in_angular_range = (p.phi > d.p1_angle) or (p.phi < d.p2_angle);
            }
            else
            {
                float offset = qDegreesToRadians(3);
                // Normal case: point is in range if it's between p1_angle and p2_angle
                point_in_angular_range = (p.phi > d.p1_angle - offset) and (p.phi < d.p2_angle + offset);
            }

            if (point_in_angular_range && p.distance2d >= dist_to_door)
            {
                filtered_out = true;
                break;
            }
        }
        if (!filtered_out)
            filtered.emplace_back(p);
    }
    return filtered;
}
