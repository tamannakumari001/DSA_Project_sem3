#include "graph.cpp" 
#include <unordered_set>
#include <vector>
#include <queue>      
#include <limits>     
#include <map>        
#include <algorithm>  
#include <iostream>   





Graph::PathResult Graph::minimumDistance(int src, int dest,
                           const std::unordered_set<int>& forbiddenNodes,
                           const std::unordered_set<std::string>& forbiddenRoadTypes) {
    
    int num_nodes = getNumNodes();
    if (src >= num_nodes || dest >= num_nodes || src < 0 || dest < 0) {
        return PathResult{};
    }

    
    std::vector<double> dist(num_nodes, std::numeric_limits<double>::infinity());
    
   
    std::map<int, int> prev_node;
    
    std::priority_queue<std::pair<double, int>, 
                        std::vector<std::pair<double, int>>, 
                        std::greater<std::pair<double, int>>> pq;

   
    if (forbiddenNodes.count(src) || forbiddenNodes.count(dest)) {
        return PathResult{};
    }

    
    dist[src] = 0.0;
    pq.push({0.0, src});

    while (!pq.empty()) {
        std::pair<double, int> p = pq.top();
        double d = p.first;
        int u = p.second;
        pq.pop();

        if (d > dist[u]) {
            continue;
        }

        
        if (u == dest) {
            PathResult result;
            result.distance = dist[dest];
            result.ifPath = true;
            
            int curr_node = dest;
            while (true) {
                result.nodes.push_back(curr_node);
                if (curr_node == src) break; 
                curr_node = prev_node[curr_node];
            }
            
            
            std::reverse(result.nodes.begin(), result.nodes.end());
            return result;
        }

        
        for (const auto& edge_rel : adj_list[u]) {
            Node* v_node = edge_rel.first;
            Edge* edge = edge_rel.second;
            int v = v_node->id;

            // Forbidden nodes
            if (forbiddenNodes.count(v)) {
                continue; 
            }
            
            // Forbidden road types
            if (forbiddenRoadTypes.count(edge->road_type)) {
                continue;
            }

            // Oneway condition
            if (edge->oneway && edge->to == u) {
                continue; 
            }

            
            double length = edge->length;
            double newDist = dist[u] + length;

            
            if (newDist < dist[v]) {
                dist[v] = newDist;      
                prev_node[v] = u; 
                pq.push({newDist, v}); 
            }
        }
    }

    
    return PathResult{};
}



json Graph::ShortestPath(json query){

    json output;
    output["id"] = query["id"];

    
    if (!query.contains("source") || !query.contains("destination")) {
        return output;
    }

    int src = query["source"];
    int dest = query["destination"];

   
    std::unordered_set<int> forbiddenNodes;
    if (query.contains("constraints") && query["constraints"].contains("forbidden_nodes")) {
        forbiddenNodes.insert(query["constraints"]["forbidden_nodes"].begin(), query["constraints"]["forbidden_nodes"].end());
    }

    
    std::unordered_set<std::string> forbiddenRoadTypes;
    if (query.contains("constraints") && query["constraints"].contains("forbidden_road_types")) {
        forbiddenRoadTypes.insert(query["constraints"]["forbidden_road_types"].begin(), query["constraints"]["forbidden_road_types"].end());
    }

    PathResult result;

    if (query["mode"] == "distance"){
        result = minimumDistance(src, dest, forbiddenNodes, forbiddenRoadTypes);

        if(result.ifPath){
            output["possible"] = true;
            output["minimum_distance"] = result.distance;
            output["path"] = result.nodes;
        }else{
            output["possible"] = false; 
        }
    } 

    return output;
}
