#include <iostream>
#include <vector>
#include <map>

int main()
{
    struct Node{ int id; std::vector<int> links;};

    std::map<int, Node> graph;

    for(int i=0; i<100; i++)
        graph.emplace(std::make_pair(i, Node{i, std::vector<int>()}));

    for(auto &[key, value] : graph)
    {
        int vecinos = rand()%6;
        for(int j=0; j<vecinos; j++)
            value.links.emplace_back( rand()%100);
    }

}