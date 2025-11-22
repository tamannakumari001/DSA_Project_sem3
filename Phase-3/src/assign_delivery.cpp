#include "../include/graph.hpp" 
#include "../include/utils.hpp"



double Graph::calcPathScore(int start_node, const std::vector<Stop>& path){

    if(path.empty()){
        return 0.0;
    }
    double current_time = 0.0;
    double total_score = 0.0;
    int prev_node = start_node;

    for(const auto& stop : path){
        double travel_time = spTimes[prev_node][stop.node_id];
        current_time += travel_time;

        if(stop.type == DROPOFF){
            total_score += current_time;
        }

        prev_node = stop.node_id;
    }

    return total_score;
}

std::pair<double, std::vector<Stop>> Graph::FindBestInsertion(const SimDriver& driver, Order* order){
    double old_score = calcPathScore(driver.current_node, driver.scheduled_path);

    double best_score = std::numeric_limits<double>::max();
    std::vector<Stop> best_path;

    Stop p_stop = {order->pickup->id, PICKUP, order->order_id};
    Stop d_stop = {order->drop->id, DROPOFF, order->order_id};

    int n = driver.scheduled_path.size();

    for(int i = 0; i <= n; i++){

        for(int j = i + 1; j <= n + 1; j++){
            std::vector<Stop> temp_path = driver.scheduled_path;
            temp_path.insert(temp_path.begin() + i, p_stop);
            temp_path.insert(temp_path.begin() + j, d_stop);

            double score = calcPathScore(driver.current_node, temp_path);

            if (score < best_score){
                best_score = score;
                best_path = temp_path;
            }
        }
    }


    return {best_score - old_score, best_path};
}

void Graph::AllocateBatch(const std::vector<int>& orders, std::vector<SimDriver>& drivers, Del_Graph &G){
    for(int j = 0; j < orders.size(); j++){
        double min_increase = std::numeric_limits<double>::max();
        std::vector<Stop> best_path;
        int best_driver = -1;

        for(size_t i = 0; i < drivers.size(); i++){
            auto [increase, new_path] = FindBestInsertion(drivers[i], G.Orders[orders[j]]);

            if(increase < min_increase){
                min_increase = increase;
                best_path = new_path;
                best_driver = i;
            }
        }

        if(best_driver != -1){
            drivers[best_driver].scheduled_path = best_path;
        }
    }
}

void Graph::allocateZonalRidersPaths(Del_Graph &G){

    for(int i = 0; i <G.num_zones; i++){
        std::vector<SimDriver> zone_drivers;
        for(auto & rider_id : G.zones[i].riders){
            SimDriver sd;
            sd.id = rider_id;
            sd.current_node = G.depot;
            zone_drivers.push_back(sd);
        }
        AllocateBatch(G.zones[i].orders, zone_drivers, G);
        for(auto & sd : zone_drivers){
            for(auto & dropPoints : sd.scheduled_path){
                if(dropPoints.type == DROPOFF){
                    G.Orders[dropPoints.order_id]->cost = spTimes[G.depot][dropPoints.node_id];
                }
            }
            // std::cout << "Zonal rider id: " << sd.id << std::endl;
            convertScheduledPathsToOutput(G.Riders[sd.id], sd.scheduled_path);
        }
    }
}

void Graph::allocateGlobalRiders(Del_Graph &G){
    std::vector<SimDriver> global_drivers;
    for(int i = 0; i < G.Riders.size(); i++){
        if(!G.Riders[i]->zonal){
            SimDriver sd;
            sd.id = i;
            sd.current_node = G.depot;
            global_drivers.push_back(sd);
        }
    }
    AllocateBatch(G.inter_orders, global_drivers, G);
    for(auto & sd : global_drivers){
        for(auto & dropPoints : sd.scheduled_path){
            if(dropPoints.type == DROPOFF){
                G.Orders[dropPoints.order_id]->cost = spTimes[G.depot][dropPoints.node_id];
            }
        }
        std::cout << "Global rider id: " << sd.id << std::endl;
        convertScheduledPathsToOutput(G.Riders[sd.id], sd.scheduled_path);
    }
}

void Graph::convertScheduledPathsToOutput(Rider* rider, std::vector<Stop>& scheduled_path){
    rider->final_route.clear();
    rider->assigned_orders.clear();
    int prev_node = -1, current_node;
    for(int i = 0;i<(int)scheduled_path.size()-1;i++){
        if(scheduled_path[i].type == PICKUP){
            rider->assigned_orders.push_back(scheduled_path[i].order_id);
        }
        
        current_node = scheduled_path[scheduled_path.size() - i - 1].node_id;
        if(prev_node == current_node){
            continue;
        }
        while(current_node != scheduled_path[scheduled_path.size() - i - 2].node_id){
            rider->final_route.push_back(current_node);
            current_node = spParents[scheduled_path[scheduled_path.size() - i - 2].node_id][current_node];
        }
        prev_node = current_node;
    }
    rider->final_route.push_back(current_node);
}