#ifndef A_STAR_PLANNER_H
#define A_STAR_PLANNER_H

#include <functional>
#include <future>
#include <math.h>
#include <memory>
#include <mutex>
#include <vector>

#include <unordered_map>
#include <unordered_set>
#include <queue>

#include "octomap/octomap.h"
#include "octomap/ColorOcTree.h"
#include "nav_msgs/msg/path.hpp"
#include "planner3D.h"

class PathNode {
    public:
        PathNode(float x = 0, float y = 0, float z = 0, PathNode* parent = NULL,
                 float gCost = INFINITY, float fCost = INFINITY) {
            this->position = Coord(x , y, z);
            this->parent = parent;
            this->gCost = gCost;
            this->fCost = fCost;
        }

        PathNode(Coord position, PathNode* parent = NULL,
                 float gCost = INFINITY, float fCost = INFINITY) {
            this->position = position;
            this->parent = parent;
            this->gCost = gCost;
            this->fCost = fCost;
        }

        void setGCost(float cost) {gCost = cost;}
        void setFCost(float cost) {fCost = cost;}
        void setParent(PathNode* parent) {this->parent = parent;}

        Coord getPosition() const {return position;}
        PathNode* getParent() const {return parent;}
        float getGcost() const {return gCost;}
        float getFcost() const {return fCost;}

        bool operator<(const PathNode&  pathNode) const {
            return this->fCost < pathNode.fCost;
        }

        bool operator>(const PathNode&  pathNode) const {
            return this->fCost > pathNode.fCost;
        }

        bool operator==(const PathNode&  pathNode) const {
            return this->position.getX() == pathNode.position.getX() &&
                   this->position.getY() == pathNode.position.getY() &&
                   this->position.getZ() == pathNode.position.getZ();
        }

        bool operator==(const Coord& coord) const {
            return this->position.getX() == coord.getX() &&
                   this->position.getY() == coord.getY() &&
                   this->position.getZ() == coord.getZ();
        }

    private:
        Coord position;
        PathNode* parent;
        float gCost;
        float fCost;
};


class AStartPlanner {
  public:
    AStartPlanner();
    AStartPlanner(const std::shared_ptr<octomap::AbstractOcTree>& octomapPtr);

    nav_msgs::msg::Path find_path(Coord start_pos, Coord goal_pos, int resolution_level = 16);

    std::future<nav_msgs::msg::Path> find_path_async(Coord start_pos,
                                                     Coord goal_pos,
                                                     const bool* cancel_flag,
                                                     int resolution_level = 16);

    void update_map(const std::shared_ptr<octomap::AbstractOcTree>& octomapPtr);

  private:
    float heuristiqueFunc(const Coord& current, const Coord&  goal) const;

    bool isNodeOccupied(Coord pos);

    void addNeighbor(const Coord&  currentKey, const Coord&  goal);

    nav_msgs::msg::Path cancelable_find_path(Coord start_pos,
                                             Coord goal_pos,
                                             const bool* cancel_flag,
                                             int resolution_level = 16);

    Coord posToMap(Coord incord);

    nav_msgs::msg::Path reconstructPath(const PathNode& pathEnd);

    std::priority_queue<PathNode, std::vector<PathNode>, std::greater<PathNode>> nodeToOpen;
    std::unordered_map<Coord, PathNode, Coord::hash> nodeSet;
    std::shared_ptr<octomap::AbstractOcTree> octreePtr;
    std::mutex mapMutex;
    double metric_resolution;
    int resolution_level;
};

#endif