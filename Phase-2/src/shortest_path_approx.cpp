#include "../include/graph.hpp" 

json Graph::shortest_path_approx(const json& query) {
    json result;
    result["id"] = query["id"];

    if(!query.contains("time_budget_ms")){
        throw "Error: Query does not contain time_budget_ms";
    }

    if(!query.contains("acceptable_error_pct")){
        throw "Error: Query does not contain acceptable_error_pct";
    }
    
    double time_budget_ms = query["time_budget_ms"];
    double acceptable_error_pct = query["acceptable_error_pct"];

    if(!query.contains("queries")){
        throw "Error: Query does not contain queries";
    }
    std::vector<std::pair<int, int>> queries;
    if (query["queries"].is_array()) {
        for (const auto& q : query["queries"]) {
            if (q.contains("source") && q.contains("target")) {
                int src = q["source"];
                int dst = q["target"];
                queries.emplace_back(src, dst);
            }else {
                throw "Error: Source/Target not found";
            }
        }
    }else{
        throw "Error: queries is not an array";
    }

    auto batch_start = std::chrono::steady_clock::now();

    json distances = json::array();

    int noOfQueries = queries.size();
    for(int i = 0; i < noOfQueries; i++){
        auto query_time_start = std::chrono::steady_clock::now();
        double elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(query_time_start - batch_start).count();
        double remaining = time_budget_ms - elapsed_time;
        
        double fraction_of_queries_not_over = (noOfQueries- i - 1)/(noOfQueries*1.0);
        double extra_factor = (0.9 + (fraction_of_queries_not_over)*0.5 );
        
        double budget = (remaining/(noOfQueries-i)*1.0 ) * extra_factor;
        
        bool timeflag = false;

        double approx_distance = approx_shortest_distance(queries[i].first, queries[i].second, acceptable_error_pct, time_budget_ms, budget, timeflag, remaining);
        distances.push_back({
            {"source", queries[i].first},
            {"target", queries[i].second},
            {"approx_shortest_distance", approx_distance}
        });

        if(timeflag){
            for(int j = i+1; j < noOfQueries;j++){
                distances.push_back({
                    {"source", queries[i].first},
                    {"target", queries[i].second},
                    {"approx_shortest_distance", hDistances[queries[i].first][queries[i].second]}
                });
            }
            return distances;
        }
    }
    return distances;
}


double Graph::approx_shortest_distance(int src, int dest, double acceptable_error, double time_budget_total, double budget, bool &timeflag, double remainingTime){
    auto start = std::chrono::steady_clock::now();
    
    double error_factor = 1.0 + acceptable_error/100;
    double w = 1.0 + (error_factor-1.03) /5.0;

    int num_nodes = getNumNodes();
    
    std::vector<double> dist(num_nodes, std::numeric_limits<double>::infinity());
    
    std::priority_queue<std::pair<double, int>, 
                        std::vector<std::pair<double, int>>, 
                        std::greater<std::pair<double, int>>> pq;
    
    std::vector<bool> visited(num_nodes, false);
    dist[src] = 0.0;
    pq.push({0.0, src});

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        if (visited[u]) {
            continue;
        }

        visited[u] = true;

        if (u == dest) {
            return dist[dest];
        }

        for (const auto& edge_rel : adj_list[u]) {
            Node* v_node = edge_rel.first;
            Edge* edge = edge_rel.second;
            int v = v_node->id;

            double newDist = dist[u] + edge->length;

            auto now = std::chrono::steady_clock::now();
            double elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
            if(remainingTime - elapsed_time < 0.01){
                timeflag = true;
                return dist[u] + hDistances[v][dest];
            }

            if (!visited[v] && newDist < dist[v]) {
                dist[v] = newDist;
                hDistances[src][v] = newDist;
                pq.push({newDist + w*hDistances[v][dest], v});
            }
        }
    }

    return -1;

}
