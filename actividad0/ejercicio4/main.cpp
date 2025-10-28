#include <algorithm>
#include <functional>
#include <iostream>
#include <chrono>
#include <ranges>

int main()
{
    std::vector<std::tuple<int, int>> myVector(10000000);

    for (auto& v : myVector)
        v = std::make_tuple(rand() % 10, rand() % 1000);//crea y devuelve la tupla

    auto start = std::chrono::high_resolution_clock::now();

    std::ranges::sort(myVector, [](auto &a, auto &b){return std::get<1>(a) < std::get<1>(b);});

    auto end = std::chrono::high_resolution_clock::now();
    const auto difference = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << difference  << std::endl;

    //for (const auto& v : myVector)
      //  std::cout << std::get<0>(v) << " " << std::get<1>(v) << std::endl;

}