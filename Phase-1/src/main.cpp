#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
// #include "src/graph.cpp"
// #include "src/shortest_path.cpp"
#include "../include/graph.hpp"
/*
    Add other includes that you require, only write code wherever indicated
*/


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
   std::vector<json> results;

    try
   {
        Graph graph(argv[1]);
  
    

    // Read queries from second file





    std::ifstream queries_file(argv[2]);
    if (!queries_file.is_open()) {
        std::cerr << "Failed to open " << argv[2] << std::endl;
        return 1;
    }
    json queries_json;
    queries_file >> queries_json;

    
    
    for (const auto& query : queries_json["events"]) {
        auto start_time = std::chrono::high_resolution_clock::now();

        /*
            Add your query processing code here
            Each query should return a json object which should be printed to sample.json
        */

        // Answer each query replacing the function process_query using 
        // whatever function or class methods that you have implemented
        json result;
        try
        {
            if (!query.contains("type")){
                std::cout << "DOES NOT CONTAIN TYPE HELPP" << std::endl;
                return -1;
            }
            auto type = query["type"];

            if (type == "remove_edge"){
                //remove_edge function
                result=graph.remove_edge(query);
            }

            else if (type == "modify_edge"){
                //modify_edge function
                result = graph.modify_edge(query);
            }

            else if (type == "knn"){
                //knn function
                result = graph.knn(query);
            }
            else if (type == "shortest_path"){
                result = graph.ShortestPath(query);
            }

        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            std::cerr << "Query type: " << query["type"] << '\n';
            std::cerr << "Query ID: " << query["id"] << '\n';
        }
        // json result = process_query(query);

        auto end_time = std::chrono::high_resolution_clock::now();
        result["processing_time"] = std::chrono::duration<double, std::milli>(end_time - start_time).count();
        results.push_back(result);
    }
    }catch(const std::exception& e){
        std::cerr << e.what() << '\n';
        std::cout << "hiii" << std::endl;
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
