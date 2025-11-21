#include "../include/graph.hpp"

std::vector<double> Graph::sssp_from(int src){
    int n = getNumNodes();
    
    std::vector<double> dist(n, std::numeric_limits<double>::infinity());
    std::vector<bool> visited(n, false);
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;

    dist[src] = 0;
    visited[src] = true;
    pq.push({0.0, src});

    while(!pq.empty()){
        double distance = pq.top().first;
        int currnode = pq.top().second;
        
        if(visited[currnode]){
            continue;
        }
        visited[currnode] = true;

        for (const auto& edge_rel : adj_list[currnode]) {
            Node* v_node = edge_rel.first;
            Edge* edge = edge_rel.second;
            int v = v_node->id;
             
            double length = edge->length;
            double newDist = dist[currnode] + length;

            
            if (!visited[v] && newDist < dist[v]) {
                dist[v] = newDist;
                hDistances[src][v] = newDist;
                if(hDistances[v][src] == getEuclidianDistance(v, src)){
                    hDistances[v][src] = newDist;
                }
                hDistances[currnode][v] = std::min(hDistances[currnode][v], length);
                pq.push({newDist, v}); 
            }
        }
    }

    return dist;
    
}

void Graph::precomputation(){
    for(int i = 0;i < num_nodes; i++){
        for(int j = i; j < num_nodes; j++){
            double dist = getEuclidianDistance(i, j);
            hDistances[i][j] = dist;
            hDistances[j][i] = dist;
        }
    }

    int num_landmarks = std::min(num_nodes, 2500);
    
    landmarks.push_back(0);

    std::vector<double> min_dist(num_nodes, std::numeric_limits<double>::infinity());

    for(int i = 1; i < num_landmarks; i++){
        int lastLandmark = landmarks[i-1];
        std::vector<double> distFromLandmark = sssp_from(lastLandmark);
        for(int i =0; i< num_nodes; i++){
            min_dist[i] = std::min(distFromLandmark[i], min_dist[i]);
        }
        
        int farthest = std::max_element(min_dist.begin(), min_dist.end()) - min_dist.begin();
        landmarks.push_back(farthest);
    }

    for(int i = 0; i<num_nodes; i++){
        for(int j = 0; j < num_nodes; j++){
            if(i == j){
                hDistances[i][j] = 0;
                continue;
            }
            if(hDistances[i][j] != getEuclidianDistance(i, j)){
                continue;
            }
    
            double hvalue = hDistances[i][j];
            for(int k : landmarks){
                double diff = std::fabs(hDistances[k][i] - hDistances[k][j]);
                hvalue = std::max(hvalue, diff);
            }
            hDistances[i][j] = hvalue;
        }
    }

}