#include "../include/graph.hpp"
#include <fstream>

using json = nlohmann::json;

std::string ROAD_TYPES[5] = {"primary","secondary","tertiary","expressway","local"};

Graph::Graph(const std::string& filename) {
    std::fstream file(filename);
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
        edges.push_back(edge);
        adj_list[edge->from].emplace_back(nodes[edge->to], edge);
        if (!edge->oneway){
            adj_list[edge->to].emplace_back(nodes[edge->from], edge);
        }
    }
    
}

