#ifndef UTILS_HPP
#define UTILS_HPP

#include "graph.hpp"

struct Order{

    int order_id;
    Node* pickup;
    Node* drop;
    double cost;
    bool intra(){
        return pickup->zone_id == drop->zone_id;
    }

};

struct Rider{

    int rider_id;
    double load;
    std::vector<int> assigned_orders;
    std::vector<int> topologcal_order;
    std::vector<int> final_route;
    bool zonal = false;
    int zone_id;

};


struct Zone{

    int zone_id;
    std::vector<int> nodes;
    std::vector<int> orders;
    std::vector<int> riders;
    double concentration_score;
    
};

struct Candidate{
    double weight;
    int target;
    int zone_id;

    bool operator<(Candidate const& other) const {
        return weight < other.weight;
    }
};

class Del_Graph{

public:    

    std::unordered_map<int , Order*> Orders;
    std::vector<Order*> orderList;
    std::vector<Rider*> Riders;
    std::vector<Zone*> zones;
    std::vector<int> zonal_orders;
    std::vector<int> inter_orders;
    std::vector<int> landmarks;
    int num_orders;
    int num_riders;
    int num_zones;
    int depot;
    double time_taken = 0;

 
};

enum StopType {
    PICKUP,
    DROPOFF
};

struct Stop{
    int node_id;
    StopType type;
    int order_id;
};

struct SimDriver {
    int id;
    int current_node;
    std::vector<Stop> scheduled_path;
};

#endif