#include <rrt_planner/collision_detector.h>
#include <costmap_2d/costmap_2d.h>  
#include <cmath>                   

namespace rrt_planner {

    CollisionDetector::CollisionDetector(costmap_2d::Costmap2DROS* costmap) {
        costmap_ = costmap->getCostmap();

        resolution_ = costmap_->getResolution();
        origin_x_ = costmap_->getOriginX();
        origin_y_ = costmap_->getOriginY();
    }

    bool CollisionDetector::inFreeSpace(const double* world_pos) {
        // Convert the world coordinates to map coordinates
        unsigned int mx, my;
        double wx = world_pos[0];  
        double wy = world_pos[1];  

        // Check if the world position is within the costmap bounds
        if (!costmap_->worldToMap(wx, wy, mx, my)) {
            return false;
        }

        // Get the cost at the given map coordinates
        unsigned char cost = costmap_->getCost(mx, my);

        // Check if the cost at this cell is below a threshold indicating it's free space
        if (cost == costmap_2d::FREE_SPACE || cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
            return true; 
        } else {
            return false;
        }
    }

    bool CollisionDetector::obstacleBetween(const double* point_a, const double* point_b) {
        double dist = computeDistance(point_a, point_b);

        if (dist < resolution_) {
            // If distance is less than the resolution, just check the final point
            return (!inFreeSpace(point_b)) ? true : false;

        } else {
            // Break the line into steps based on the resolution
            int num_steps = static_cast<int>(floor(dist / resolution_));

            double point_i[2];
            for (int n = 1; n <= num_steps; n++) {
                // Interpolate points along the line
                point_i[0] = point_a[0] + n * (point_b[0] - point_a[0]) / num_steps;
                point_i[1] = point_a[1] + n * (point_b[1] - point_a[1]) / num_steps;

                // Check if this intermediate point is in free space
                if (!inFreeSpace(point_i)) return true;
            }
            // No obstacle detected between the points
            return false;
        }
    }
};