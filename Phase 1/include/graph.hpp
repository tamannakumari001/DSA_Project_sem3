#include <iostream>
#include <vector>
#include <map>
#include <nlohmann/json.hpp>


using namespace std;

struct Node{
    int id;
    double lat;
    double lon;
    vector<string> pois;
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
};

class Graph{
private:
    vector<Edge*> edges;
    vector<Node*> nodes;
    vector<vector<pair<Node*, Edge*>>> adj_list;
    int num_nodes;
public:
    Graph(const string& filename);
    ~Graph();
    
};


