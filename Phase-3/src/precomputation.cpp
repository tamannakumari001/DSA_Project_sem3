#include "../include/graph.hpp"

std::vector<double> Graph::sssp_from(int src){
    int n = getNumNodes();
    
    std::vector<double> time_taken(n, std::numeric_limits<double>::infinity());
    std::vector<bool> visited(n, false);
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;

    time_taken[src] = 0;
    pq.push({0.0, src});

    while(!pq.empty()){
        double time = pq.top().first;
        int currnode = pq.top().second;
        pq.pop();
        
        if(visited[currnode]){
            continue;
        }
        visited[currnode] = true;

        for (const auto& edge_rel : adj_list[currnode]) {
            Node* v_node = edge_rel.first;
            Edge* edge = edge_rel.second;
            int v = v_node->id;
             
            double time_length = edge->avg_time;
            double newTime = time_taken[currnode] + time_length;

            
            if (!visited[v] && newTime < time_taken[v]) {
                time_taken[v] = newTime;
                spTimes[src][v] = newTime;
                spParents[src][v] = currnode;
                pq.push({newTime, v}); 
            }
        }
    }    
    return time_taken;
}


void Graph::precomputation(){
    // #pragma omp parallel for schedule(dynamic)
    for(int i = 0;i < num_nodes;i++){
        sssp_from(i);
    }
}