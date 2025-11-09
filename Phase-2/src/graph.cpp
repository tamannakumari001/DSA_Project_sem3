

#include "../include/graph.hpp"
#include <fstream>

std::string ROAD_TYPES[5] = {"primary","secondary","tertiary","expressway","local"};
std::string POIS[6] = {"restaurant" , "petrol_station" , "hospital" , "atm" , "hotel" , "pharmacy"};
using json = nlohmann::json;



void Graph::add_edge(Edge* edge){

    adj_list[edge->from].emplace_back(nodes[edge->to], edge);
    if (!edge->oneway){
        adj_list[edge->to].emplace_back(nodes[edge->from], edge);
    }
}


Graph::Graph(const std::string& filename) {
    std::fstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }
    json data;
    file >> data;

    if(!data.contains("meta")){
        throw "Error: Does not contain 'meta'";
    }
    if(!data["meta"].contains("nodes")){
        throw "Error: Input does not contain number of nodes in metadata";
    }
    num_nodes = data["meta"]["nodes"];
    nodes.resize(num_nodes);
    adj_list.resize(num_nodes);

    if(!data.contains("nodes")){
        throw "Error: Input does not contain nodes";
    }
    for (auto& n : data["nodes"]){
        Node* node = new Node();
        node->id = n["id"];
        node->lat = n["lat"];
        node->lon = n["lon"];
        for (auto& poi : POIS){
            node->pois[poi] = false;
        }
        for (auto& poi : n["pois"]){
            node->pois[poi] = true;
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

        if (e.contains("speed_profile")){
            for (auto& speed : e["speed_profile"]){
                edge->speed_profile.push_back(speed);
            }
        }    
        edge->oneway = e["oneway"];
        edge->road_type = e["road_type"];
        edges[edge->id] = edge;
        edge -> active = true;
        add_edge(edge);
    }


    
}




json Graph::remove_edge(const json& query){

    json result;
    result["id"] = query["id"];
    if (!query.contains("edge_id")){
        throw "Error: No edge id given in query";
    }
    int edge_id = query["edge_id"];
    if (edges.find(edge_id)==edges.end()){
        result["done"]= false;
        return result;
    }
    edges[edge_id]->active = false;
    result["done"] = true;
    return result;

}

json Graph::modify_edge(const json& query){

    json result;
    result["id"] = query["id"];


    if (!query.contains("edge_id")){
        throw "Error: No edge id given in query";
    }

    int edge_id = query["edge_id"];

    if (edges.find(edge_id) == edges.end()){
        result["done"]= false;
        return result;
    }

    if(!query.contains("patch")){
        if(!edges[edge_id]->active){
            result["done"] = true;
        }else{
            result["done"] = false;
        }
        edges[edge_id]->active = true;
        return result;
    }

    json patch = query["patch"];
    

    Edge* edge = edges[edge_id];
    edge -> active = true;

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

    result["done"] = true;
    return result;

}

json Graph::knn(const nlohmann::json& query){
    int query_id = query["id"];
    int k = query["k"];
    std::string poi_type = query["poi"];
    double query_lat = query["query_point"]["lat"];
    double query_lon = query["query_point"]["lon"];
    std::string metric = query["metric"];

    std::vector<int> final_answer;

    json output;
    output["id"] = query_id;

    if (!query.contains("k") || !query.contains("poi") || !query.contains("query_point") || !query.contains("metric")){
        output["points"] = final_answer;
        output["nodes"] = {};
        return output;
    }


    if (metric == "euclidean"){
        std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> q;

        for (auto& node : nodes){
            if (node->pois[poi_type]){
                double dist = sqrt(pow(node->lat - query_lat, 2) + pow(node->lon - query_lon, 2));
                q.push({dist, node->id});             
            }
        }

        while (!q.empty() && (int)final_answer.size() < k){
            final_answer.push_back(q.top().second);
            q.pop();
        }
    }

    if (metric == "shortest_path"){
        if(nodes.empty()){
            return {};
        }
        double curr_min = std::numeric_limits<double>::max();
        int source_id = -1;
        for(auto& node : nodes){
            double dist = sqrt(pow(node->lat - query_lat, 2) + pow(node->lon - query_lon, 2));
            if (dist < curr_min){
                curr_min = dist;
                source_id = node->id;
            }
        }
        int pois_found = 0;
        std::vector<double> distances(num_nodes, std::numeric_limits<double>::max());
        std::vector<bool> processed(num_nodes, false);
        distances[source_id] = 0.0;
        std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> q;
        q.push({0.0, source_id});  

        while (!q.empty()){
            auto [curr_dist, u] = q.top();
            q.pop();

            if (processed[u]) continue;
            processed[u] = true;

            if (nodes[u]->pois[poi_type]){
                final_answer.push_back(u);
                pois_found++;
                if (pois_found == k){
                    break;
                }
            }

            for (auto& [neighbor, edge] : adj_list[u]){
                if (!edge->active) continue;
                double weight = edge->length;
                if (distances[u] + weight < distances[neighbor->id]){
                    distances[neighbor->id] = distances[u] + weight;
                    q.push({distances[neighbor->id], neighbor->id});
                }
            }
        }
    }
    output["nodes"] = final_answer;
    return output;
}

