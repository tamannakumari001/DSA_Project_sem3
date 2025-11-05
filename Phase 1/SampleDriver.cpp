#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
/*
    Add other includes that you require, only write code wherever indicated
*/

#include "src/graph.cpp"

using json = nlohmann::json;

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <graph.json> <queries.json> <output.json>" << std::endl;
        return 1;
    }

    // Read graph from first file
    /*
        Add your graph reading and processing code here
        Initialize any classes and data structures needed for query processing
    */
    Graph graph(argv[1]);

    // Read queries from second file

    fstream file(argv[2]);
    if (!file.is_open()) {
        cerr << "Error opening file: " << argv[2] << endl;
        return;
    }
    nlohmann::json data;
    file >> data;

    for (auto& event : data["events"]){
        string type = event["type"];
        try{
            if (type == "shortest_path"){
                //shortest_path function
            }
            else if (type == "remove_edge"){
                //remove_edge function
                graph.remove_edge(event["edge_id"]);
            }

            else if (type == "modify_edge"){
                //modify_edge function
                int edge_id = event["edge_id"];
                json patch = event["patch"];

                graph.modify_edge(edge_id, patch);
            }

            else if (type == "knn"){
                //knn function
                graph.knn(event);
            }
        }

        catch(...){
            continue;
        }
            
        
    }



    std::ifstream queries_file(argv[2]);
    if (!queries_file.is_open()) {
        std::cerr << "Failed to open " << argv[2] << std::endl;
        return 1;
    }
    json queries_json;
    queries_file >> queries_json;

    std::vector<json> results;

    for (const auto& query : queries_json["events"]) {
        auto start_time = std::chrono::high_resolution_clock::now();

        /*
            Add your query processing code here
            Each query should return a json object which should be printed to sample.json
        */

        // Answer each query replacing the function process_query using 
        // whatever function or class methods that you have implemented
        json result = process_query(query);

        auto end_time = std::chrono::high_resolution_clock::now();
        result["processing_time"] = std::chrono::duration<double, std::milli>(end_time - start_time).count();
        results.push_back(result);
    }

    std::ofstream output_file(argv[3]);
    if (!output_file.is_open()) {
        std::cerr << "Failed to open output.json for writing" << std::endl;
        return 1;
    }

    json output = results;
    output_file << output.dump(4) << std::endl;

    output_file.close();
    return 0;
}
