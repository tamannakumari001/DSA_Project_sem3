#include "../include/graph.hpp"
#include <fstream>

using json = nlohmann::json;

<<<<<<< HEAD:Phase-1/src/graph.cpp
std::string ROAD_TYPES[5] = {"primary","secondary","tertiary","expressway","local"};

Graph::Graph(const std::string& filename) {
    std::fstream file(filename);
=======
void Graph::add_edge(Edge* edge){
    vector<pair<Node*, Edge*>> list = adj_list[edge->from];
    if (find(list.begin(), list.end(), make_pair(nodes[edge->to], edge)) != list.end()){
        return;
    }
    adj_list[edge->from].emplace_back(nodes[edge->to], edge);
    if (!edge->oneway){
        adj_list[edge->to].emplace_back(nodes[edge->from], edge);
    }
}


Graph::Graph(const string& filename) {
    fstream file(filename);
>>>>>>> 97aaaea141a05269d34926b5bf8bae5c54dcac8a:Phase 1/src/graph.cpp
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }
    json data;
    file >> data;

    num_nodes = data["meta"]["nodes"];
    nodes.resize(num_nodes);
    adj_list.resize(num_nodes);
    for (auto& n : data["nodes"]){
        Node* node = new Node();
        node->id = n["id"];
        node->lat = n["lat"];
        node->lon = n["lon"];
        for (auto& poi : n["pois"]){
            node->pois.push_back(poi);
        }
        nodes[node->id] = node;
    }

    for (auto & e : data["edges"]){
        Edge* edge = new Edge();
        edge->id = e["id"];
        edge->from = e["u"];
        edge->to = e["v"];
        edge->length = e["length"];
        edge->avg_time = e["average_time"];
        for (auto& speed : e["speed_profile"]){
            edge->speed_profile.push_back(speed);
        }
        edge->oneway = e["oneway"];
        edge->road_type = e["road_type"];
        edges[edge->id] = edge;
        add_edge(edge);
    }
    
}

<<<<<<< HEAD:Phase-1/src/graph.cpp
=======

void Graph::remove_edge(int edge_id){
    if (edges.find(edge_id) == edges.end()) {
        return;
    }
    Edge* edge = edges[edge_id];
    int from = edge->from;
    int to = edge->to;

    auto& from_list = adj_list[from];
    for(auto it = from_list.begin(); it != from_list.end(); ++it) {
        if (it->second->id == edge_id) {
            from_list.erase(it);
            break;
        }
    }
    if (!edge->oneway) {
        auto& to_list = adj_list[to];
        for(auto it = to_list.begin(); it != to_list.end(); ++it) {
            if (it->second->id == edge_id) {
                to_list.erase(it);
                break;
            }
        }
    }

    edges.erase(edge_id);
    delete edge;

}

void Graph::modify_edge(int edge_id, const nlohmann::json& patch){
    if (edges.find(edge_id)==edges.end()){
        return;
    }

    Edge* edge = edges[edge_id];
    add_edge(edge);
    

    if (patch.contains("length")){
        edge->length = patch["length"];
    }
    if (patch.contains("average_time")){
        edge->avg_time = patch["average_time"]; 
    }
    if (patch.contains("road_type")){
        edge -> road_type = patch["road_type"];
    }
    if (patch.contains("speed_profile")){
        edge->speed_profile.clear();
        for (auto& speed : patch["speed_profile"]){
            edge->speed_profile.push_back(speed);
        }
    }
    // if (patch.contains("oneway")){
    //     bool prev = edge->oneway;
    //     edge->oneway = patch["oneway"]; 
    //     if (prev == edge->oneway) return;

    //     if (edge->oneway){
    //         auto it = find(adj_list[edge->to].begin() , adj_list[edge -> to].end() , make_pair(nodes[edge->from] , edge));
    //         if (it != adj_list[edge->to].end()){
    //             adj_list[edge -> to].erase(it);
    //         }
        
    //     else{
    //     adj_list[edge->to].emplace_back(nodes[edge->from], edge);
    //     }
    // }

        
    // }



}

vector<int> Graph::knn(const nlohmann::json& query){
    int query_id = query["id"];
    int k = query["k"];
    string poi_type = query["poi"];
    double query_lat = query["query_point"]["lat"];
    double query_lon = query["query_point"]["lon"];
    string metric = query["metric"];

    vector<int> final_answer;


    if (metric == "euclidean"){
        priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> q;

        for (auto& node : nodes){
            if (find(node->pois.begin(), node->pois.end(), poi_type) != node->pois.end()){
                double dist = sqrt(pow(node->lat - query_lat, 2) + pow(node->lon - query_lon, 2));
                q.push({dist, node->id});             
            }
        }

        while (!q.empty() && final_answer.size() < k){
            final_answer.push_back(q.top().second);
            q.pop();
        }
    }

    if (metric == "shortest_path"){
        if(nodes.empty()){
            return {};
        }
        double curr_min = numeric_limits<double>::max();
        int source_id = -1;
        for(auto& node : nodes){
            double dist = sqrt(pow(node->lat - query_lat, 2) + pow(node->lon - query_lon, 2));
            if (dist < curr_min){
                curr_min = dist;
                source_id = node->id;
            }
        }
        int pois_found = 0;
        vector<double> distances(num_nodes, numeric_limits<double>::max());
        vector<bool> processed(num_nodes, false);
        distances[source_id] = 0.0;
        priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> q;
        q.push({0.0, source_id});  

        while (!q.empty()){
            auto [curr_dist, u] = q.top();
            q.pop();

            if (processed[u]) continue;
            processed[u] = true;

            if (find(nodes[u]->pois.begin(), nodes[u]->pois.end(), poi_type) != nodes[u]->pois.end()){
                final_answer.push_back(u);
                pois_found++;
                if (pois_found == k){
                    break;
                }
            }

            for (auto& [neighbor, edge] : adj_list[u]){

                double weight = edge->length;
                if (distances[u] + weight < distances[neighbor->id]){
                    distances[neighbor->id] = distances[u] + weight;
                    q.push({distances[neighbor->id], neighbor->id});
                }
            }
        }
    }

    return final_answer;
}
>>>>>>> 97aaaea141a05269d34926b5bf8bae5c54dcac8a:Phase 1/src/graph.cpp
