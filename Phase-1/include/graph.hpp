#include <iostream>
#include <vector>
#include <map>
#include <nlohmann/json.hpp>
#include <unordered_set>

using json = nlohmann::json;


struct Node{
    int id;
    double lat;
    double lon;
    std::vector<std::string> pois;
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
};

class Graph{
private:
    std::vector<Edge*> edges;
    std::vector<Node*> nodes;
    int num_nodes;
    std::vector<std::vector<std::pair<Node*, Edge*>>> adj_list;

    
public:
    
    struct PathResult {
        bool ifPath = false;
        double distance = std::numeric_limits<double>::infinity();
        std::vector<int> nodes;
    };

    Graph(const std::string& filename);
    json ShortestPath(json query);
    PathResult minimumDistance(int src, int dest,
                           const std::unordered_set<int>& forbiddenNodes,
                           const std::unordered_set<std::string>& forbiddenRoadTypes);
    int getNumNodes(){
        return num_nodes;
    }
};