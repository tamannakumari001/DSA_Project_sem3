#include "../include/graph.hpp" 
#include <fstream> 

json Graph::ksp_heuristic(const json& query) {
    json result;
    result["id"] = query["id"];

    if (!query.contains("source") || !query.contains("target")) {
        throw "Error: Query does not contain source/target";
    }
    if(!query.contains("k")){
        throw "Error: Query does not contain k";
    }
    if(!query.contains("overlap_threshold")){
        throw "Error: Query does not contain overlap_threshold";
    }

    int k = query["k"];
    int source = query["source"];
    int target = query["target"];
    double overlapThreshold = query["overlap_threshold"];

    std::vector<PathResult> paths;
    paths = k_shortest_paths_heuristic(source, target, k, overlapThreshold);
    result["paths"] = json::array();


    for (const auto& path : paths){
        json path_json;
        path_json["path"] = path.nodes;
        path_json["length"] = path.distance;
        result["paths"].push_back(path_json);
    }
    
    return result;
}

Graph::PathResult Graph::minimumDistanceHeuristic(int src, int dest,
                           std::unordered_map<int, int> &edgeCount,
                           const double alpha, const double overlapThreshold, bool ifOnePath) {
    
    int num_nodes = getNumNodes();
    if (src >= num_nodes || dest >= num_nodes || src < 0 || dest < 0) {
        return PathResult();
    }

    const double beta = 0.91;
    
    std::vector<double> dist(num_nodes, std::numeric_limits<double>::infinity());
    std::vector<double> unbiasedDist(num_nodes, std::numeric_limits<double>::infinity());
    
   
    std::vector<int> prev_node(num_nodes, -1);
    std::vector<int> prev_edge(num_nodes, -1);

    
    std::priority_queue<std::pair<double, int>, 
                        std::vector<std::pair<double, int>>, 
                        std::greater<std::pair<double, int>>> pq;

    std::vector<bool> visited(num_nodes, false);
    dist[src] = 0.0;
    unbiasedDist[src] = 0.0;
    pq.push({0.0, src});

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        if (visited[u]) {
            continue;
        }

        visited[u] = true;

        if (u == dest) {
            PathResult result;
            result.distance = unbiasedDist[dest];
            result.ifPath = true;

            int curr_node = u;
            while (true) {
                result.nodes.push_back(curr_node);
                if (curr_node == src) break;
                result.edges.push_back(prev_edge[curr_node]);
                edgeCount[prev_edge[curr_node]] ++;
                curr_node = prev_node[curr_node];
            }
            std::reverse(result.nodes.begin(), result.nodes.end());
            std::reverse(result.edges.begin(), result.edges.end());
            return result;
        }

        for (const auto& edge_rel : adj_list[u]) {
            Node* v_node = edge_rel.first;
            Edge* edge = edge_rel.second;
            int v = v_node->id;

            double costFactor = 1.0;
            if(ifOnePath){
                
                double thresholdFactor = (1-(overlapThreshold/100.0));
                double usagePenalty = alpha*pow(edgeCount[edge->id], beta);
                costFactor = 1.0 + (usagePenalty*thresholdFactor);
            }
            double newDist = dist[u] + edge->length*costFactor;
            double newUnbiasedDist = unbiasedDist[u] + edge->length;

            if (!visited[v] && newDist < dist[v]) {
                dist[v] = newDist;
                unbiasedDist[v] = newUnbiasedDist;
                prev_node[v_node->id] = u;
                prev_edge[v_node->id] = edge->id;
                pq.push({newDist, v});
            }
        }
    }

    return PathResult();
}

std::vector<Graph::PathResult> Graph::k_shortest_paths_heuristic(int source, int target, int k, double overlapThreshold){
    
    const double alpha = 0.49;
    std::vector<PathResult> paths;
    std::unordered_map<int, int> EdgeCount;
    Graph::PathResult curr_path = minimumDistanceHeuristic(source, target, EdgeCount, 0, overlapThreshold, false);

    if (!curr_path.ifPath){
        return paths;
    }

    int extraPaths;
    if(k<5){
        extraPaths = 4;
    }else{
        extraPaths = 3;
    }

    paths.push_back(curr_path);
    
    for(int i = 0; i < k-1; i++){
        Graph::PathResult new_path = minimumDistanceHeuristic(source, target, EdgeCount, alpha, overlapThreshold, true);
        if(!new_path.ifPath){
            return paths;
        }else{
            paths.push_back(new_path);
        }
    }
    for(int i = 0; i < extraPaths; i++){
        Graph::PathResult new_path = minimumDistanceHeuristic(source, target, EdgeCount, alpha, overlapThreshold, true);
        if(!new_path.ifPath){
            if(!i){
                return paths;
            }else{
                break;
            }
        }else{
            paths.push_back(new_path);
        }
    }

    std::vector<Graph::PathResult> shortest_k_paths_heurisitic = best_subset(paths, k, overlapThreshold);

    return shortest_k_paths_heurisitic;

}

std::vector<Graph::PathResult> Graph::best_subset(const std::vector<Graph::PathResult> &paths, int k, double overlapThreshold){
    std::vector<Graph::PathResult> subsetpaths;
    double minimumPenalty = std::numeric_limits<double>::infinity();
    int minIdx = -1;

    if(k<=0 || paths.empty()){
        return {};
    }
    
    int totalNoOfPaths = paths.size();
    if(totalNoOfPaths<=k){
        return paths;
    }

    std::vector<std::vector<Graph::PathResult>> path_Subsets = generate_subsets(paths.size(), k, paths);

    for(int i = 0; i<(int)path_Subsets.size();i++){
        double penalty = computePenalty(path_Subsets[i], overlapThreshold);
        if(minimumPenalty > penalty){
            minIdx = i;
            minimumPenalty = penalty;
        }
    }
    return path_Subsets[minIdx];
}


double Graph::computePenalty(const std::vector<Graph::PathResult> &paths, double overlapThreshold){
    double shortestlen = paths.front().distance;
    int noOfPaths = paths.size();
    
    std::vector<std::unordered_set<int>> edgeSets(noOfPaths);
    for (int i = 0; i < noOfPaths; ++i) {
        edgeSets[i].insert(paths[i].edges.begin(), paths[i].edges.end());
    }

    std::vector<int> overlapCount(noOfPaths, 0);
    std::vector<double> distPenalty(noOfPaths, 0);

    for(int i = 0; i < noOfPaths;i++){
        distPenalty[i] = ((paths[i].distance-shortestlen)*1.0/shortestlen) + 0.1;
    }

    for(int i = 0; i < noOfPaths; i++){
        for(int j = i; j < noOfPaths;j++){
            int common = 0;
            for(const int& e : edgeSets[i]){
                if(edgeSets[j].count(e)){
                    common++;
                }
            }

            double overlapj = (double) common/ paths[i].edges.size();
            double overlapi = (double) common/ paths[j].edges.size();
            

            if(overlapi > overlapThreshold/100){
                overlapCount[i]++;
            }
            if(overlapj > overlapThreshold/100){
                overlapCount[j]++;
            }
            if(i == j){
                overlapCount[i]--;
            }
        }
    }

    double totalPenalty = 0;
    for(int i = 0; i< noOfPaths; i++){
        totalPenalty += overlapCount[i] * distPenalty[i];
    }

    return totalPenalty;
}

std::vector<std::vector<Graph::PathResult>> Graph::generate_subsets(int n, int k, const std::vector<Graph::PathResult> &paths){
    
    std::vector<std::vector<int>> subsets;

    if (k > n || n == 0) return {};

    
    std::vector<int> comb;
    for (int i = 1; i < k; ++i)
        comb.push_back(i); 

    while (true) {
        
        std::vector<int> subset = {0};
        subset.insert(subset.end(), comb.begin(), comb.end());
        subsets.push_back(subset);

    
        int i; 
        for (i = comb.size() - 1; i >= 0; --i) {
            if (comb[i] != i + n - (k - 1)) break;
        }
        if (i < 0) break; 

        comb[i]++;
        for (int j = i + 1; j < (int)comb.size(); ++j)
            comb[j] = comb[j - 1] + 1;
    }

    std::vector<std::vector<Graph::PathResult>> subsets_Paths;
    for(int i = 0; i < (int)subsets.size(); i++){
        std::vector<Graph::PathResult> subset_Path;
        for(const auto& j : subsets[i]){
            subset_Path.push_back(paths[j]);
        }
        subsets_Paths.push_back(subset_Path);
    }
    return subsets_Paths;
}



