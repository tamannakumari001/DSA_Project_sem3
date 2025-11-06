#include "../include/graph.hpp"
#include <fstream>




json Graph::ksp(const json& query) {
    json result;
    result["id"] = query["id"];

    if (!query.contains("source") || !query.contains("target") || !query.contains("k")) {
        result["paths"] = json::array();
        return result;
    }

    int k = query["k"];
    int source = query["source"];
    int target = query["target"];
    std::vector<PathResult> paths;

    if (query["mode"] == "distance"){
        paths = k_shortest_paths_distance(source, target, k);
        result["paths"] = json::array();
        for (const auto& path : paths){
            json path_json;
            path_json["path"] = path.nodes;
            path_json["length"] = path.distance;
            result["paths"].push_back(path_json);
        }
    return result;
    }

    else{
        //pass
    }
}




struct PathHash {
    size_t operator()(const Graph::PathResult& p) const {
        size_t h = 0;

        auto hash_combine = [&](size_t v) {
            h ^= v + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
        };

        for (int node_id : p.nodes)
            hash_combine(std::hash<int>()(node_id));

        for (int edge_id : p.edges)
            hash_combine(std::hash<int>()(edge_id));

        return h;
    }
};


std::vector<Graph::PathResult> Graph::k_shortest_paths_distance(int source, int target, int k){

    std::vector<PathResult> paths;

    Graph::PathResult curr_path = minimumDistance(source, target, {},{});

    if (!curr_path.ifPath){
        return paths;
    }

    paths.push_back(curr_path);
    auto& E = edges;

    struct PathCompare{
        bool operator()(const Graph::PathResult& a, const Graph::PathResult& b) {
            return a.distance > b.distance;
        }
    };

    std::priority_queue<Graph::PathResult, std::vector<Graph::PathResult>, PathCompare> q;
    std::unordered_set<Graph::PathResult, PathHash> unique_paths;

    unique_paths.insert(curr_path);
    
    int count = 1;
    while (count < k && curr_path.ifPath){
        int path_size = curr_path.nodes.size();

        for (int i = 0; i < path_size - 1; ++i){
            std::unordered_set<int> forbiddenNodes;
            std::unordered_set<int> forbiddenEdges;

            for (int j = 0; j < i; ++j){
                forbiddenNodes.insert(curr_path.nodes[j]);
            }

            for (const auto& p : paths) {
                if (p.nodes.size() > i &&
                    std::equal(p.nodes.begin(), p.nodes.begin() + i + 1, curr_path.nodes.begin())) 
                {
                    if (p.edges.size() > i)
                        forbiddenEdges.insert(p.edges[i]);
                }
            }


            Graph::PathResult spur_path = minimumDistance(curr_path.nodes[i], target, forbiddenNodes, forbiddenEdges);

            if (spur_path.ifPath){
                Graph::PathResult total_path;
                total_path.distance = 0.0;

                for (int n = 0; n <= i; ++n){
                    total_path.nodes.push_back(curr_path.nodes[n]);
                    if (n < i){
                        total_path.edges.push_back(curr_path.edges[n]);
                        total_path.distance += E[curr_path.edges[n]]->length;
                    }
                }

                for (size_t m = 1; m < spur_path.nodes.size(); ++m){
                    total_path.nodes.push_back(spur_path.nodes[m]);
                    total_path.edges.push_back(spur_path.edges[m - 1]);
                    total_path.distance += E[spur_path.edges[m - 1]]->length;
                }
                if(unique_paths.insert(total_path).second){
                    q.push(total_path);
                }

            }
        }

        if (q.empty()){
            break;
        }

        curr_path = q.top();
        q.pop();
        paths.push_back(curr_path);
        count++;
    }



    return paths;
}