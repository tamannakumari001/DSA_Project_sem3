#ifndef UTILS_HPP
#define UTILS_HPP

#include "graph.hpp"

struct Order{

    int order_id;
    Node* pickup;
    Node* drop;
    int zone_pickup;
    int zone_drop;
    double cost;

    bool intra(){
        return zone_pickup == zone_drop;
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
    double concentration_score;
    
};

class Del_Graph{

public:    

    std::unordered_map<int , Order*> Orders;
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

#endif