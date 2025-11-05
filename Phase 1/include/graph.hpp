#include <iostream>
#include <vector>
#include <map>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <queue>


using namespace std;

struct Node{
    int id;
    double lat;
    double lon;
    unordered_map<string,bool> pois;
};

struct Edge{
    int id;
    int from;
    int to;
    double length;
    double avg_time;
    vector<double> speed_profile;
    bool oneway;
    string road_type;
    bool active;
};

class Graph{
private:
    unordered_map<int, Edge*> edges;
    vector<Node*> nodes;
    vector<vector<pair<Node*, Edge*>>> adj_list;
    int num_nodes;
public:
    Graph(const string& filename);
    void remove_edge(int edge_id);
    void modify_edge(int edge_id, const nlohmann::json& patch);
    void add_edge(Edge* edge);
    vector<int> knn(const nlohmann::json& query);

};


