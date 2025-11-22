#include "../include/graph.hpp" 

#define TIME_CHECK_FREQ 127 

json Graph::shortest_path_approx(const json& query) {
    json result;
    result["id"] = query["id"];

    if(!query.contains("time_budget_ms")){ 
        throw "Error: No time_budget_ms";
    }
    if(!query.contains("acceptable_error_pct")){
        throw "Error: No acceptable_error_pct";
    }
    
    double time_budget_ms = query["time_budget_ms"];
    double acceptable_error_pct = query["acceptable_error_pct"];

    if(!query.contains("queries") || !query["queries"].is_array()){
        throw "Error: Invalid queries";
    }

    std::vector<std::pair<int, int>> queries;
    for (const auto& q : query["queries"]) {
        if (q.contains("source") && q.contains("target")) {
            queries.emplace_back(q["source"], q["target"]);
        } else {
            throw "Error: Source/Target not found";
        }
    }

    json distances = json::array();
    int noOfQueries = queries.size();
    std::vector<json> temp_results(noOfQueries);

    auto batch_start = std::chrono::steady_clock::now();
    auto global_deadline = batch_start + std::chrono::milliseconds(static_cast<long>(time_budget_ms/1.1));
    
    std::atomic<bool> global_timeout(false);

    #pragma omp parallel for schedule(dynamic) num_threads(8)
    for(int i = 0; i < noOfQueries; i++){
        
        double approx_distance = approx_shortest_distance(
            queries[i].first, 
            queries[i].second, 
            acceptable_error_pct, 
            global_deadline, 
            global_timeout
        );

        temp_results[i] = {
            {"source", queries[i].first},
            {"target", queries[i].second},
            {"approx_shortest_distance", approx_distance}
        };
    }

    for(auto &x : temp_results){
        distances.push_back(x);
    }
    result["distances"] = distances;
    return result;
}

double Graph::approx_shortest_distance(int src, int dest, double acceptable_error, 
                                       std::chrono::time_point<std::chrono::steady_clock> deadline, 
                                       std::atomic<bool> &global_timeout) {

    double w = 1.0 + (acceptable_error / 100.0);
    
    std::vector<double> dist(num_nodes, std::numeric_limits<double>::infinity());
    std::priority_queue<std::pair<double, int>, 
                        std::vector<std::pair<double, int>>, 
                        std::greater<std::pair<double, int>>> pq;
    
    std::vector<bool> visited(num_nodes, false);
    
    dist[src] = 0.0;
    pq.push({0.0 + w * getEuclidianDistance(src, dest), src});

    int ops_counter = 0;

    while (!pq.empty()) {
        
        
        if ((ops_counter++ & TIME_CHECK_FREQ) == 0) {

            if (global_timeout.load(std::memory_order_relaxed) || std::chrono::steady_clock::now() > deadline) {
                global_timeout.store(true, std::memory_order_relaxed);
    
                int u = pq.top().second; 
                double curr_dist = dist[u];
                
                if (curr_dist == std::numeric_limits<double>::infinity()) return -1.0;

                return curr_dist + getEuclidianDistance(u, dest);
            }
        }

        int u = pq.top().second;
        pq.pop();

        if (visited[u]){
            continue;
        }

        visited[u] = true;

        if (u == dest){
            return dist[dest];
        }

        for (const auto& edge_rel : adj_list[u]) {
            Node* v_node = edge_rel.first;
            Edge* edge = edge_rel.second;
            int v = v_node->id;

            double newDist = dist[u] + edge->length;

            if (!visited[v] && newDist < dist[v]) {
                dist[v] = newDist;
                pq.push({newDist + w * getEuclidianDistance(v,dest), v});
            }
        }
    }

    return -1;
}