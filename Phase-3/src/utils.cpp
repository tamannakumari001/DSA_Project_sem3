#include "../include/graph.hpp" 
#include "../include/utils.hpp"

#define global_to_total_ratio 0.2

std::vector<std::vector<double>> Graph::getAffinityWeights(int depotNode, std::unordered_set<int> &impNodes, Del_Graph &G){
    const int alpha = 100000;
    const double epsilon = 1;
    const double order_glue = 10000000;
    for(auto & order : G.Orders){
        impNodes.insert(order.second->pickup->id);
        impNodes.insert(order.second->drop->id);
    } 
    impNodes.insert(depotNode);
    std::vector<std::vector<double>> affinityWeights(num_nodes,std::vector<double> (num_nodes, 0));
    for(const auto & i : impNodes){
        for(const auto & j : impNodes){ 
            if(i==j){
                continue;
            }else if(spTimes[i][j] > 15000) {
                continue;
            }else {
                double affinity = 1.0*alpha/(spTimes[i][j]*spTimes[i][j] +  epsilon); 
                affinityWeights[i][j] = affinity;
            }
        }
    }
    for(const auto & order: G.Orders){
        int u = order.second->pickup->id;
        int v = order.second->drop->id;
        affinityWeights[u][v] += order_glue;
    }
    return affinityWeights;
}

void Graph::PartitionGraph(Del_Graph &G){
    int depotNode = G.depot;
    int noOfDrivers = G.num_riders;
    std::unordered_set<int> impNodes;
    std::vector<std::vector<double>> affinityWeights = getAffinityWeights(depotNode, impNodes, G);

    int N = impNodes.size();
    
    int d_interzonal = ceil(global_to_total_ratio * noOfDrivers);
    int d_intrazonal = noOfDrivers - d_interzonal;

    int k_fleet = ceil(d_intrazonal/12.0);

    int k_density = ceil(N/40.0);

    int num_zones = std::max(k_fleet, k_density);
    Zone* z1 = new Zone(); 
    std::vector<double> min_dist(num_nodes, std::numeric_limits<double>::infinity());
    G.landmarks.push_back(depotNode);
    z1->zone_id = 0;
    z1->nodes.push_back(depotNode);
    G.zones.push_back(*z1);
    // G.zones[0]->zone_id = 0;
    // G.zones[0]->nodes.push_back(depotNode);
    nodes[depotNode]->zone_id = 0;
    for(int i = 1; i < num_zones; i++){
        int lastLandmark = G.landmarks[i-1];
        for(auto &v : impNodes){
            min_dist[v] = std::min(min_dist[v], spTimes[lastLandmark][v]);
        }
        int farthest = std::max_element(min_dist.begin(), min_dist.end()) - min_dist.begin(); 
        G.landmarks.push_back(farthest);
        nodes[farthest]->zone_id = i;
        Zone* z = new Zone();
        z->zone_id = i;
        z->nodes.push_back(farthest);
        G.zones.push_back(*z);
        // G.zones[i]->zone_id = i;
        // G.zones[i]->nodes.push_back(farthest);
    }

    std::priority_queue<Candidate> pq;

    for(int i = 0; i < num_zones; i++){
        int landmark = G.landmarks[i];

        for(auto &v : impNodes){
            if(i != v){
                pq.push({affinityWeights[landmark][v], v, i});
            }
        }
    }

    while(!pq.empty()){
        Candidate top = pq.top();
        pq.pop();

        int u = top.target;
        int z = top.zone_id;

        if (nodes[u]->zone_id != -1) continue;

        nodes[u]->zone_id = z;
        G.zones[z].nodes.push_back(u); 
        
        for (auto& e : impNodes) {
            if(e == u) continue;
            if (nodes[e]->zone_id == -1) {
                pq.push({affinityWeights[u][e], e, z});
            }
        }
    }
    std::cout << "printing zones haha" << '\n';
    for(int i = 0; i < num_zones; i++){
        for(auto &x : G.zones[i].nodes){
            std::cout << x << " ";
        }
        std::cout << '\n';
    }

    for(auto& v : impNodes){
        if(nodes[v]->zone_id == -1){
            throw "Error: Pickup/Delivery at inaccessible location. Disconnected graph";
        }
    }
}



double Graph::calculateZoneMSTWorkLoad(Del_Graph &G, int zone_id){
    int zone_size = G.zones[zone_id].nodes.size();
    std::vector<double> min_dist(zone_size, std::numeric_limits<double>::infinity());
    std::vector<bool> in_mst(zone_size, false);
    int start_node = G.landmarks[zone_id];
    min_dist[start_node] = 0.0;

    double mst_weight = 0.0;

    for(int i = 0; i < zone_size;i++){
        double curr_min = std::numeric_limits<double>::infinity();
        int u = -1;
        for(int j = 0; j < zone_size;j++){
            int node_id = G.zones[zone_id].nodes[j];
            if(!in_mst[j] && min_dist[j] < curr_min){
                curr_min = min_dist[j];
                u = j;
            }
        }

        in_mst[u] = true;
        mst_weight += min_dist[u];

        for(int j = 0; j < zone_size;j++){
            int v_node_id = G.zones[zone_id].nodes[j];
            if(!in_mst[j] && spTimes[G.zones[zone_id].nodes[u]][v_node_id] < min_dist[j]){
                min_dist[j] = spTimes[G.zones[zone_id].nodes[u]][v_node_id];
            }
        }
    }

    return mst_weight*1.1; //backtracking factor
}

void Graph::orderNodesByZone(Del_Graph &G){
    for(auto & order_pair : G.Orders){
        Order* order = order_pair.second;
        int pickup_zone = order->pickup->zone_id;
        int drop_zone = order->drop->zone_id;
        if(pickup_zone == drop_zone){
            G.zones[pickup_zone].orders.push_back(order->order_id);
            G.zonal_orders.push_back(order->order_id);
        }else{
            G.inter_orders.push_back(order->order_id);
        }
    }
}

void Graph::allocateZonalRiders(Del_Graph &G){
    int global_riders = ceil(G.Riders.size() * global_to_total_ratio);
    int zonal_riders = G.Riders.size() - global_riders;

    double total_system_workload = 0.0;
    std::vector<double> zone_workloads(G.num_zones, 0.0);

    for(int i = 0; i < G.num_zones; i++){
        double zone_workload = calculateZoneMSTWorkLoad(G, i);
        if (zone_workload < 1e-5) zone_workload = 1e-5;
        zone_workloads[i] = zone_workload;
        total_system_workload += zone_workload;
    }

    struct ZoneShare { int zone_id; int assigned; double fraction; };

    std::vector<ZoneShare> shares;
    int assigned_so_far = 0;

    for (int zone_id = 0; zone_id < G.num_zones; ++zone_id) {

        double fraction = 0.0;
        if (total_system_workload > 0.0) {
            fraction = (double)zonal_riders * (zone_workloads[zone_id] / total_system_workload);
        }
        int base = (int)std::floor(fraction);
        shares.push_back({zone_id, base, fraction - base});
        assigned_so_far += base;
    }

    int leftovers = zonal_riders - assigned_so_far;
    std::sort(shares.begin(), shares.end(), [](const ZoneShare& a, const ZoneShare& b) {
        return a.fraction > b.fraction;
    });

    for (int i = 0; i < leftovers && i < (int)shares.size(); ++i) {
        shares[i].assigned++;
    }

    std::vector<int> zone_drivers(G.num_zones, 0);
    for (const auto& s : shares) {
        int final_count = 0;
        if (!G.zones[s.zone_id].nodes.empty()) {
            final_count = std::max(1, s.assigned);
        } else {
            final_count = 0;
        }
        zone_drivers[s.zone_id] = final_count;
    }
    int updated_zonal_riders = 0;
    for (int i = 0; i < G.num_zones; i++) 
        updated_zonal_riders += zone_drivers[i];
    
    for(int i = 0 ; i < updated_zonal_riders ; i++){
        G.Riders[i]->zonal = true;
        for(int j = 0 ; j < G.num_zones ; j++){
            if(zone_drivers[j] > 0){
                G.Riders[i]->zone_id = j;
                std::cout << "adding rider id: "<< i << " to zone_id: " <<j<<std::endl;
                G.zones[j].riders.push_back(i);
                zone_drivers[j]--;
                break;
            }
        }
    }
    for(int i = 0 ; i<(int)G.zones.size(); i++){
        for(auto x : G.zones[i].riders){
            std::cout<< x << " ";
        }
        std::cout << std::endl;
    }
}