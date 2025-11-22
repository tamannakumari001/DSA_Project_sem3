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

    PartitionGraph(G);
    orderNodesByZone(G);
    allocateZonalRiders(G);
    allocateZonalRidersPaths(G);
    allocateGlobalRiders(G);    
    
    for(auto & order_pair : G.Orders){
        Order* order = order_pair.second;
        if(order -> cost < 0){
            throw "Error: Some orders couldn't be assigned";
        }
        G.time_taken += order -> cost;
    }

    output["assignments"] = json::array();
    for (auto & rider : G.Riders){
        json rider_output;
        rider_output["driver_id"] = rider -> rider_id;
        rider_output["route"] = rider -> final_route;
        rider_output["order_ids"] = rider -> assigned_orders;
        output["assignments"].push_back(rider_output);
    }

    output["metrics"]["total_delivery_time_s"] = G.time_taken;

    return output;



}