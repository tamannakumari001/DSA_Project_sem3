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
#include <numeric>
#include <unordered_map>
#include <atomic>
#include "utils.hpp"

using json = nlohmann::json;
using ordered_json = nlohmann::ordered_json;


struct Node{
    int id;
    double lat;
    double lon;
    std::unordered_map<std::string,bool> pois;
    int zone_id = -1;
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

    uint64_t idx(uint64_t i, uint64_t j){
        return i*num_nodes + j;
    }

    double getEuclidianDistance(int n, int m){
        const double R = 6371000.0;
        double phi1 = nodes[n]->lat * M_PI/180.0;
        double phi2 = nodes[m]->lat * M_PI/180.0;
        double dphi = (nodes[m]->lat - nodes[n]->lat) * M_PI/180.0;
        double dlambda = (nodes[m]->lon - nodes[n]->lon) * M_PI/180.0;

        double a = std::sin(dphi / 2.0) * std::sin(dphi / 2.0) +
               std::cos(phi1) * std::cos(phi2) *
               std::sin(dlambda / 2.0) * std::sin(dlambda / 2.0);
    
        double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));

        return R * c;
    }

    json remove_edge(const json& query);
    json modify_edge(const json& query);
    void add_edge(Edge* edge);
    json knn(const nlohmann::json& query);
    
    json ksp(const nlohmann::json& query);
    std::vector<PathResult> k_shortest_paths_distance(int source, int target, int k);
    
    json ksp_heuristic(const nlohmann::json& query);
    PathResult minimumDistanceHeuristic(int src, int dest, std::unordered_map<int, int> &edgeCount, const double alpha, const double overlapThreshold, bool ifOnePath);
    std::vector<PathResult> k_shortest_paths_heuristic(int source, int target, int k, double overlapThreshold);
    double computePenalty(const std::vector<Graph::PathResult> &paths, double overlapThreshold);
    std::vector<PathResult> best_subset(const std::vector<Graph::PathResult> &paths, int k, double overlapThreshold);
    std::vector<std::vector<PathResult>> generate_subsets(int n, int k, const std::vector<Graph::PathResult> &paths);

    json shortest_path_approx(const nlohmann::json& query);
    void precomputation();
    std::vector<double> sssp_from(int src);

    double approx_shortest_distance(int src, int dest, double acceptable_error, 
                                       std::chrono::time_point<std::chrono::steady_clock> deadline, 
                                       std::atomic<bool> &global_timeout);

    std::vector<double> sssp(int src) const;

    std::vector<std::vector<double>> penalties;
    std::vector<double> computePenalty2(const std::vector<PathResult> &paths, double overlapThreshold);
    json phase_3(const nlohmann::json& query);
    void PartitionGraph(Del_Graph &G);
    void allocateZonalRiders(Del_Graph &G);

    double calculateZoneMSTWorkLoad(Del_Graph &G, int zone_id);
    std::vector<std::vector<double>> getAffinityWeights(int depotNode, std::unordered_set<int> &impNodes, Del_Graph &G);
    double calcPathScore(int start_node, const std::vector<Stop>& path);

    void allocateZonalRidersPaths(Del_Graph &G);
    void convertScheduledPathsToOutput(Rider* rider, std::vector<Stop>& scheduled_path);
    void allocateGlobalRiders(Del_Graph &G);
    void orderNodesByZone(Del_Graph &G);

    std::pair<double, std::vector<Stop>> FindBestInsertion(const SimDriver& driver, Order* order);
    void AllocateBatch(const std::vector<int>& orders, std::vector<SimDriver>& drivers, Del_Graph &G);
};

#endif 