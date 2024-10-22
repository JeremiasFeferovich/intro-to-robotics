#include <rrt_planner/rrt_planner.h>
#include <cmath>  // For math functions

namespace rrt_planner {

    RRTPlanner::RRTPlanner(costmap_2d::Costmap2DROS *costmap, 
            const rrt_params& params) : params_(params), collision_dect_(costmap) {

        costmap_ = costmap->getCostmap();
        map_width_  = costmap_->getSizeInMetersX();
        map_height_ = costmap_->getSizeInMetersY();

        random_double_x.setRange(-map_width_, map_width_);
        random_double_y.setRange(-map_height_, map_height_);

        nodes_.reserve(params_.max_num_nodes);
    }

    bool RRTPlanner::planPath() {

        // clear everything before planning
        nodes_.clear();

        // Start Node
        createNewNode(start_, -1);

        double *p_rand, *p_new;
        Node nearest_node;

        for (unsigned int k = 1; k <= params_.max_num_nodes; k++) {

            p_rand = sampleRandomPoint();
            nearest_node = nodes_[getNearestNodeId(p_rand)];
            p_new = extendTree(nearest_node.pos, p_rand); // new point and node candidate

            if (!collision_dect_.obstacleBetween(nearest_node.pos, p_new)) {
                createNewNode(p_new, nearest_node.node_id);

            } else {
                continue;
            }

            if(k > params_.min_num_nodes) {
                
                if(computeDistance(p_new, goal_) <= params_.goal_tolerance){
                    return true;
                }
            }
        }

        return false;
    }

    // Find the index of the nearest node to the given point
    int RRTPlanner::getNearestNodeId(const double *point) {
        double min_dist = std::numeric_limits<double>::max();
        int nearest_node_id = -1;

        for (size_t i = 0; i < nodes_.size(); ++i) {
            double dist = computeDistance(nodes_[i].pos, point);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_node_id = i;
            }
        }

        return nearest_node_id;
    }

    // Create a new node with the given position and parent node ID
    void RRTPlanner::createNewNode(const double* pos, int parent_node_id) {
        Node new_node;
        new_node.pos[0] = pos[0];
        new_node.pos[1] = pos[1];
        new_node.node_id = nodes_.size(); // Assign ID as current size of the vector
        new_node.parent_id = parent_node_id;

        nodes_.emplace_back(new_node);
    }

    // Sample a random point within the map boundaries
    double* RRTPlanner::sampleRandomPoint() {
        rand_point_[0] = random_double_x.generate();  // Random x coordinate
        rand_point_[1] = random_double_y.generate();  // Random y coordinate

        return rand_point_;
    }

    // Extend the tree towards the random point by a fixed step size
    double* RRTPlanner::extendTree(const double* point_nearest, const double* point_rand) {
        // Calculate direction vector from nearest point to random point
        double dir_x = point_rand[0] - point_nearest[0];
        double dir_y = point_rand[1] - point_nearest[1];
        double length = std::sqrt(dir_x * dir_x + dir_y * dir_y);

        // Normalize direction and multiply by step size
        dir_x = (dir_x / length) * params_.step;
        dir_y = (dir_y / length) * params_.step;

        // Generate the new candidate point by extending in the direction of the random point
        candidate_point_[0] = point_nearest[0] + dir_x;
        candidate_point_[1] = point_nearest[1] + dir_y;

        return candidate_point_;
    }

    const std::vector<Node>& RRTPlanner::getTree() {
        return nodes_;
    }

    void RRTPlanner::setStart(double *start) {
        start_[0] = start[0];
        start_[1] = start[1];
    }

    void RRTPlanner::setGoal(double *goal) {
        goal_[0] = goal[0];
        goal_[1] = goal[1];
    }

};
