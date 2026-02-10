# pragma once

# include <Eigen/Core>
# include <unordered_map>
# include <deque>
# include <vector>
# include <queue>
#include <unordered_set>

# include "map_utils.h"

struct Vec3Hash {
    std::size_t operator()(const Eigen::Vector3d& v) const {
        int x = std::round(v.x() * 1000.0);
        int y = std::round(v.y() * 1000.0);
        int z = std::round(v.z() * 1000.0);
        return ((size_t)x << 42) ^ ((size_t)y << 21) ^ (size_t)z;
    }
};

struct Vec3Equal {
    bool operator()(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const {
        return (a - b).norm() < 1e-6;
    }
};

// struct AStarNode {
//     Eigen::Vector3d pos;
//     double g = 0, h = 0;
//     Eigen::Vector3d parent_pos = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    
//     bool operator<(const AStarNode& o) const {
//         return g + h > o.g + o.h;
//     }
// };

struct Node {
    Eigen::Vector3d parent_coord;
    Eigen::Vector3d coordinate;
    double g;

    bool operator<(const Node& other) const {
        // 按g值比较（小顶堆需要反向）
        return g > other.g;
    }
};


class AStarSearch{
public:
    int search_mode;
    double step_size;
    int max_iter;
    double max_time;
    double reached_threshold;
    double hfunc_factor;

    std::vector<Eigen::Vector3d> action_space;

    void init_searcher(int, double, int, double, double, double);
    void path_search(
        MapUtils*, 
        std::vector<Eigen::Vector3d>&, 
        std::vector<std::pair<size_t, size_t>>&,
        std::vector<std::vector<Eigen::Vector3d>>&,
        Eigen::Vector3d const&
    );
};