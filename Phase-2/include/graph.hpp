#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <iostream>
#include <vector>
#include <map>
#include <nlohmann/json.hpp>
#include <unordered_set>
#include <algorithm>
#include <queue>
#include <limits>

using json = nlohmann::json;


struct Node{
    int id;
    double lat;
    double lon;
    std::unordered_map<std::string,bool> pois;
};

struct Edge{
    int id;
    int from;
    int to;
    double length;
    double avg_time;
    std::vector<double> speed_profile;
    bool oneway;
    std::string road_type;
    bool active;
};

class Graph{
private:
    std::unordered_map<int, Edge*> edges;
    std::vector<Node*> nodes;
    int num_nodes;
    std::vector<std::vector<std::pair<Node*, Edge*>>> adj_list;

    
public:
    
    struct PathResult {
        bool ifPath = false;
        double distance = std::numeric_limits<double>::infinity();
        std::vector<int> nodes;
        std::vector<int> edges;
        bool operator==(const PathResult& other) const {
            return nodes == other.nodes && edges == other.edges;
        }
    };

    Graph(const std::string& filename);
    json ShortestPath(json query);
    PathResult minimumDistance(int src, int dest,
                           const std::unordered_set<int>& forbiddenNodes,
                           const std::unordered_set<int>& forbiddenEdge);

    PathResult minimumTime(int src, int dest,
                            const std::unordered_set<int>& forbiddenNodes,
                            const std::unordered_set<std::string>& forbiddenRoadTypes);

    int getNumNodes(){
        return num_nodes;
    }
    json remove_edge(const json& query);
    json modify_edge(const json& query);
    void add_edge(Edge* edge);
    json knn(const nlohmann::json& query);
    json ksp(const nlohmann::json& query);
    std::vector<PathResult> k_shortest_paths_distance(int source, int target, int k);
};

#endif 