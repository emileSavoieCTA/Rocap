
#include "AStarPlanner.h"




AStartPlanner::AStartPlanner() {
    octreePtr = std::shared_ptr<octomap::AbstractOcTree>(new octomap::ColorOcTree(0.15));
}

AStartPlanner::AStartPlanner(const std::shared_ptr<octomap::AbstractOcTree>& octomapPtr) {
    this->octreePtr = octomapPtr;
}

void AStartPlanner::update_map(const std::shared_ptr<octomap::AbstractOcTree>& octomapPtr) {
        mapMutex.lock();
        this->octreePtr = octomapPtr;
        mapMutex.unlock();
}

float AStartPlanner::heuristiqueFunc(const Coord& current, const Coord&  goal) const {
    float xSquare = std::pow(current.getX()-goal.getX(), 2);
    float ySquare = std::pow(current.getY()-goal.getY(), 2);
    float zSquare = std::pow(current.getZ()-goal.getZ(), 2);

    return std::sqrt(xSquare+ySquare+zSquare);
}

bool AStartPlanner::isNodeOccupied(Coord pos) {
    auto node = ((octomap::ColorOcTree*)octreePtr.get())->search(pos.getX(),
                                                                        pos.getY(),
                                                                        pos.getZ(),
                                                                        16);

    if (node != NULL) {
        return ((octomap::ColorOcTree*)octreePtr.get())->isNodeOccupied(node);
    } else {
        return true;
    }
}

void AStartPlanner::addNeighbor(const Coord&  currentKey,
                const Coord&  goal) {
    PathNode* current = &nodeSet.find(currentKey)->second;
    Coord const pos = current->getPosition();

    for (int x = -1; x < 2; x++) {
        for (int y = -1; y < 2; y++) {
            for (int z = -1; z < 2; z++) {
                const float deltaX =(x*metric_resolution);
                const float deltaY =(y*metric_resolution);
                const float deltaZ =(z*metric_resolution);
                Coord neighborCoord = posToMap(Coord(pos.getX()+deltaX, pos.getY()+deltaY, pos.getZ()+deltaZ));
                if (!isNodeOccupied(neighborCoord)) {
                    float dist = std::sqrt(std::pow((deltaX), 2)+ std::pow((deltaY), 2)+ std::pow((deltaZ), 2));
                    float gCost = current->getGcost()+dist;
                    float fCost = gCost + heuristiqueFunc(neighborCoord, goal);
                    PathNode Neighbor(neighborCoord, current, gCost, fCost);

                    auto neighborNode = nodeSet.find(neighborCoord);

                    if (neighborNode == nodeSet.end()) {
                        nodeSet.insert({neighborCoord, Neighbor});
                        nodeToOpen.push(Neighbor);
                    } else {
                        if (Neighbor < (neighborNode->second)) {
                        neighborNode->second.setGCost(gCost);
                        neighborNode->second.setGCost(fCost);
                        neighborNode->second.setParent(current);
                        }
                    }
                }
            }
        }
    }
}

Coord AStartPlanner::posToMap(Coord incord) {
    auto octree_ = ((octomap::ColorOcTree*)octreePtr.get());

    auto key = octree_->coordToKey(incord.getX(), incord.getY(), incord.getZ(), resolution_level);
    auto pos = octree_->keyToCoord(key);

    // octree_->coordToKey(startPosition,treeDepth);
    return Coord(pos.x(), pos.y(), pos.z());
}

nav_msgs::msg::Path AStartPlanner::reconstructPath(const PathNode& pathEnd) {
    nav_msgs::msg::Path computedPath;

    computedPath.header.frame_id = "map";

    std::vector<geometry_msgs::msg::PoseStamped> reversePath;

    PathNode* parent = pathEnd.getParent();
    while (parent != NULL) {
        geometry_msgs::msg::PoseStamped point;
        point.pose.position.x = parent->getPosition().getX();
        point.pose.position.y = parent->getPosition().getY();
        point.pose.position.z = parent->getPosition().getZ();
        reversePath.push_back(point);
        parent = parent->getParent();
    }

    size_t pathLength = reversePath.size();
    for (int ctr = pathLength-1; ctr >= 0; ctr--) {
        computedPath.poses.push_back(reversePath[ctr]);
    }

    return computedPath;
}

nav_msgs::msg::Path AStartPlanner::cancelable_find_path(Coord start_pos,
                                            Coord goal_pos,
                                            const bool* cancel_flag,
                                            int resolution_level) {
    // locking the map
    std::lock_guard<std::mutex> guard(mapMutex);
    this->resolution_level = resolution_level;
    metric_resolution = octreePtr->getResolution()*(16-resolution_level+1);
    Coord start = posToMap(start_pos);
    Coord goal = posToMap(goal_pos);

    PathNode startNode(start.getX(), start.getY(), start.getZ(), NULL, 0.0, heuristiqueFunc(start, goal));

    // reset container
    nodeToOpen = std::priority_queue<PathNode, std::vector<PathNode>, std::greater<PathNode>>();
    nodeSet = std::unordered_map<Coord, PathNode, Coord::hash>();

    nodeToOpen.push(startNode);
    nodeSet.insert({start, startNode});

    if (isNodeOccupied(goal)) {
        return nav_msgs::msg::Path();
    }

    while (!nodeToOpen.empty() && !(*cancel_flag)) {
        PathNode const current = nodeToOpen.top();

        if (current == goal) {
            return reconstructPath(current);
        }
        addNeighbor(current.getPosition(), goal);
        nodeToOpen.pop();
    }

    // no path found
    return nav_msgs::msg::Path();
}


std::future<nav_msgs::msg::Path> AStartPlanner::find_path_async(Coord start_pos,
                                                    Coord goal_pos,
                                                    const bool* cancel_flag,
                                                    int resolution_level) {
    return std::async(std::launch::async,
                        &AStartPlanner::cancelable_find_path,
                        this,
                        start_pos,
                        goal_pos,
                        cancel_flag,
                        resolution_level);
}

nav_msgs::msg::Path AStartPlanner::find_path(Coord start_pos, Coord goal_pos, int resolution_level) {
    bool cancel_flag = 0;
    return cancelable_find_path(start_pos, goal_pos, &cancel_flag, resolution_level);
}



