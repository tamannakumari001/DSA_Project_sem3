
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
#include <chrono>
#include <cmath>

using json = nlohmann::json;
using ordered_json = nlohmann::ordered_json;


struct Node{
    int id;
    double lat;
    double lon;
    std::unordered_map<std::string,bool> pois;
    int zone_id;
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
    std::vector<std::vector<double>> spTimes;  
    std::vector<std::vector<int>> spParents; // stores parents for everyNode for every possible dijkstra spanning tree
    std::vector<int> landmarks;

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

    double getEuclidianDistance(int n, int m){
        return sqrt((nodes[n]->lat - nodes[m]->lat)*(nodes[n]->lat - nodes[m]->lat) 
         + (nodes[n]->lon - nodes[m]->lon)*(nodes[n]->lon - nodes[m]->lon));
    }

    
    json ksp(const nlohmann::json& query);
    std::vector<PathResult> k_shortest_paths_distance(int source, int target, int k);
    
    json ksp_heuristic(const nlohmann::json& query);
    PathResult minimumDistanceHeuristic(int src, int dest, std::unordered_map<int, int> &edgeCount, const double alpha, const double overlapThreshold);
    std::vector<PathResult> k_shortest_paths_heuristic(int source, int target, int k, double overlapThreshold);
    double computePenalty(const std::vector<Graph::PathResult> &paths, double overlapThreshold);
    std::vector<PathResult> best_subset(const std::vector<Graph::PathResult> &paths, int k, double overlapThreshold);
    std::vector<std::vector<PathResult>> generate_subsets(int n, int k, const std::vector<Graph::PathResult> &paths);

    json shortest_path_approx(const nlohmann::json& query);
    void precomputation();
    std::vector<double> sssp_from(int src);
    double approx_shortest_distance(int src, int dest, double acceptable_error,
         double time_budget_total, double budget, bool &timeflag, double remainingTime);
    json phase_3(const nlohmann::json& query);
    std::vector<std::vector<double>> getAffinityWeights(const std::vector<Order> &orderNodes, int depotNode, std::unordered_set<int> &impNodes);
    void PartitionGraph(const std::vector<Order> &orderNodes, int depotNode);


};

#endif 