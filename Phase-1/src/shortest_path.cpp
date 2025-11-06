#include "../include/graph.hpp" 
#include <unordered_set>
#include <vector>
#include <queue>      
#include <limits>     
#include <map>        
#include <algorithm>  
#include <iostream>   


Graph::PathResult Graph::minimumTime(int src, int dest,
                            const std::unordered_set<int>& forbiddenNodes,
                            const std::unordered_set<std::string>& forbiddenRoadTypes, json query){

    int num_nodes = getNumNodes();
    if (src >= num_nodes || dest >= num_nodes || src < 0 || dest < 0){
        return PathResult{};
    }

    std::vector<double> time(num_nodes, std::numeric_limits<double>::infinity());

    std::map<int, int> prev_node;

    std::priority_queue<std::pair<double, int>,
                        std::vector<std::pair<double, int>>,
                        std::greater<std::pair<double, int>>> pq;

    if (forbiddenNodes.count(src) || forbiddenNodes.count(dest)){
        return PathResult{};
    }

    std::vector<bool> visited(num_nodes, false);

    time[src] = 0.0;
    pq.push({0.0, src});

    while(!pq.empty()){
        int u = pq.top().second;
        pq.pop();

        if(visited[u]){
            continue;
        }

        visited[u] = true;

        if (u == dest){
            PathResult result;
            result.distance = time[dest];
            result.ifPath = true;

            int curr_node = u;
            while(true){
                result.nodes.push_back(curr_node);
                if(curr_node == src) break;
                curr_node = prev_node[curr_node];
            }

            std::reverse(result.nodes.begin(), result.nodes.end());
            return result;
        }

        for(const auto& edge_rel : adj_list[u]){
            Node* v_node = edge_rel.first;
            Edge* edge = edge_rel.second;
            int v = v_node->id;

            if(forbiddenNodes.count(v) || forbiddenRoadTypes.count(edge->road_type) || !edge->active){
                continue;
            }

            double t1 = time[u];
            double newTime;
            
            if (edge->speed_profile.empty()){
                t1 = edge->avg_time;
                newTime = time[u] + t1; 
            }else{
                int currentIdx = (int)(time[u]/(15*60)) % 96;
                double distToBeTravelled = edge->length;
                while(true){
                    int multiplier = (int)(t1/(15*60*96)) + 1;
                    double currSpeed;
                    if (currentIdx < 96){
                        currSpeed = edge->speed_profile[currentIdx];
                    }else{
                        currSpeed = edge->speed_profile.front();
                        currentIdx = 0;
                    }
                    if(distToBeTravelled - currSpeed*((currentIdx+1)*15*60*multiplier - t1) > 0){
                        distToBeTravelled -= currSpeed*((currentIdx+1)*15*60*multiplier - t1);
                        t1 = (currentIdx+1)*15*60*multiplier;
                        currentIdx++;
                    }else{
                        t1 += (distToBeTravelled)/currSpeed;
                        break;
                    }
                }
                newTime = t1;
            }

            if(!visited[v] && newTime < time[v]){
                time[v] = newTime;
                prev_node[v] = u;
                pq.push({newTime,v}); 
            }
        }

    }

    return PathResult{};

}


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

    std::vector<bool> visited(num_nodes, false);

    dist[src] = 0.0;
    pq.push({0.0, src});

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        if(visited[u]){
            continue;
        }

        visited[u] = true;

        
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

            // Forbidden nodes and forbidden edges
            if (forbiddenNodes.count(v) || forbiddenRoadTypes.count(edge->road_type) || !edge->active) {
                continue; 
            }
             
            
            double length = edge->length;
            double newDist = dist[u] + length;

            
            if (!visited[v] && newDist < dist[v]) {
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

    
    if (!query.contains("source") || !query.contains("target")) {
        std::cout << "DOES NOT CONTAIN SOURCE OR TARGET HELPP" << std::endl;
        return output;
    }

    int src = query["source"];
    int dest = query["target"];

   
    std::unordered_set<int> forbiddenNodes;
    if (query.contains("constraints") && query["constraints"].contains("forbidden_nodes")) {
        forbiddenNodes.insert(query["constraints"]["forbidden_nodes"].begin(), query["constraints"]["forbidden_nodes"].end());
    }

    
    std::unordered_set<std::string> forbiddenRoadTypes;
    if (query.contains("constraints") && query["constraints"].contains("forbidden_road_types")) {
        forbiddenRoadTypes.insert(query["constraints"]["forbidden_road_types"].begin(), query["constraints"]["forbidden_road_types"].end());
    }

    PathResult result;

    if(!query.contains("mode")){
        std::cout << "CONTAINS NO MODE AHHH" << std::endl;
    }
    if (query["mode"] == "distance"){
        result = minimumDistance(src, dest, forbiddenNodes, forbiddenRoadTypes);

        if(result.ifPath){
            output["possible"] = true;
            output["minimum_distance"] = result.distance;
            output["path"] = result.nodes;
        }else{
            output["possible"] = false; 
        }
    }else{
        result = minimumTime(src, dest, forbiddenNodes, forbiddenRoadTypes, query);

        if(result.ifPath){
            output["possible"] = true;
            output["minimum_time"] = result.distance;
            output["path"] = result.nodes;
        }else{
            output["possible"] = false;
        }
    } 

    return output;
}
