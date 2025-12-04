#pragma once
#include <QPointF>
#include <QRectF>
#include <Eigen/Dense>
#include <vector>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/sliding_window.hpp>

#include "src/common_types.h"

  struct NominalRoom
        {
            float width; //  mm
            float length;
            Doors doors;

            explicit NominalRoom(const float width_=10000.f, const float length_=5000.f, Corners  corners_ = {}) :
                width(width_), length(length_)
            {};
            [[nodiscard]] Corners corners() const
            {
                // compute corners from width and length
                return {
                    {QPointF{-width/2.f, -length/2.f}, 0.f, 0.f},
                    {QPointF{width/2.f, -length/2.f}, 0.f, 0.f},
                    {QPointF{width/2.f, length/2.f}, 0.f, 0.f},
                    {QPointF{-width/2.f, length/2.f}, 0.f, 0.f}
                };
            }
            [[nodiscard]] QRectF rect() const
            {
                return QRectF{-width/2.f, -length/2.f, width, length};
            }
            [[nodiscard]] Corners transform_corners_to(const Eigen::Affine2d &transform) const  // for room to robot pass the inverse of robot_pose
            {
                Corners transformed_corners;
                for(const auto &[p, _, __] : corners())
                {
                    auto ep = Eigen::Vector2d{p.x(), p.y()};
                    Eigen::Vector2d tp = transform * ep;
                    transformed_corners.emplace_back(QPointF{static_cast<float>(tp.x()), static_cast<float>(tp.y())}, 0.f, 0.f);
                }
                return transformed_corners;
            }
            Walls get_walls()
            {
                Walls walls;
                auto cs = corners();
                cs.push_back(cs[0]);
                for (const auto &[i,c] : cs | iter::sliding_window(2) | iter::enumerate)
                {
                    const auto &c1 = c[0];
                    const auto &c2 = c[1];

                    Eigen::Vector2f e1{std::get<0>(c1).x(), std::get<0>(c1).y()};
                    Eigen::Vector2f e2{std::get<0>(c2).x(), std::get<0>(c2).y()};

                    const auto r = Eigen::ParametrizedLine<float,2>::Through(e1, e2);
                    walls.emplace_back(r, i, c1, c2);
                }
                return walls;
            }

            Wall get_closest_wall_to_point(const Eigen::Vector2f &p)
            {
                const auto walls = get_walls();

                auto m = std::ranges::min_element(walls, [p](const auto &w1, const auto &w2)
                {   return std::get<0>(w1).distance(p) < std::get<0>(w2).distance(p); });
                return *m;
            }

            Eigen::Vector2f get_projection_of_point_on_closest_wall(const Eigen::Vector2f &p)
            {
                auto w = get_closest_wall_to_point(p);
                return std::get<0>(w).projection(p);
            }
        };