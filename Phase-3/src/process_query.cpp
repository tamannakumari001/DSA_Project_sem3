#include "../include/graph.hpp" 
#include "../include/utils.hpp"
#include <fstream> 

json Graph::phase_3(const nlohmann::json& query){
    if (!query.contains("orders")){
        throw("Error: query does not contain orders");
    }
    if (!query.contains("fleet")){
        throw("Error: query does not contain fleet");
    }
    if(!query["fleet"].contains("num_delivery_guys")){
        throw "Error: fleet does not contain num_delivery_guys";
    }
    if(!query["fleet"].contains("depot_node")){
        throw "Error: fleet does not contain num_delivery_guys";
    }
    json output;
    Del_Graph G;

    //constructed the subgraph G out of 

    for (auto & order : query["orders"]){
        Order* o = new Order;
        if(!order.contains("order_id")){
            throw "Error: order doesn't contain order_id";
        }
        if(!order.contains("pickup")){
            throw "Error: order doesn't contain pickup";
        }
        if(!order.contains("dropoff")){
            throw "Error: order doesn't contain dropoff";
        }
        o -> order_id = order["order_id"];
        o -> pickup = nodes[order["pickup"]];
        o -> drop = nodes[order["dropoff"]];
        G.Orders[o -> order_id] = o;
    }

    G.num_riders = query["fleet"]["num_delivery_guys"];
    G.depot = query["fleet"]["depot_node"];

    for (int i = 0 ; i < G.num_riders ; i++){
        Rider* r =  new Rider;
        r -> rider_id = i;
        G.Riders.push_back(r);
    }

    //computing zones

    // int num_zones = 0.75 * G.num_riders; //heuristic. Here for now I am assuming a distribution where around 75% of drivers may go zonal;
    // G.num_zones = num_zones;

    
    // for(int i = 1; i < num_zones; i++){
    //     int lastLandmark = G.landmarks[i-1];
    //     std::vector<double> distFromLandmark = sssp_from(lastLandmark);
    //     for(int i =0; i< num_nodes; i++){
    //         min_dist[i] = std::min(distFromLandmark[i], min_dist[i]);
    //     }
        
    //     int farthest = std::max_element(min_dist.begin(), min_dist.end()) - min_dist.begin();
    //     landmarks.push_back(farthest);
    // }
    
}