#include "../include/graph.hpp" 
#include "../include/utils.hpp"

std::vector<std::vector<double>> Graph::getAffinityWeights(const std::vector<Order> &orderNodes, int depotNode, std::unordered_set<int> &impNodes){
    const int alpha = 100000;
    const double epsilon = 1;
    const double order_glue = 10000000;
    for(auto & order : orderNodes){
        impNodes.insert(order.pickup->id, order.drop->id);
    } 
    impNodes.insert(depotNode);
    std::vector<std::vector<double>> affinityWeights(num_nodes,std::vector<double> (num_nodes, 0));
    for(const auto & i : impNodes){
        for(const auto & j : impNodes){ 
            if(i==j){
                continue;
            }else if(spTimes[i][j] > 15000) {
                continue;
            }else {
                double affinity = 1.0*alpha/(spTimes[i][j]*spTimes[i][j] +  epsilon); 
                affinityWeights[i][j] = affinity;
            }
        }
    }
    for(const auto & order: orderNodes){
        int u = order.pickup->id;
        int v = order.drop->id;
        affinityWeights[u][v] += order_glue;
    }
    return affinityWeights;
}

void Graph::PartitionGraph(const std::vector<Order> &orderNodes, int depotNode){
    std::unordered_set<int> impNodes;
    std::vector<std::vector<double>> affinityWeights = getAffinityWeights(orderNodes, depotNode, impNodes);

    int N = impNodes.size();
    
    
}