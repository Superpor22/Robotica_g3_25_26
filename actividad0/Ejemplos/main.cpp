#include <iostream>
#include <chrono>
#include <vector>
#include <list>

void funcionPrueba ( std::vector<double> &&v)
{
    const auto p=v;
}


int main()
{

    std::vector<double> v(1000);
    std::vector<std::vector <double>> myVector;

    const auto start = std::chrono::high_resolution_clock::now();

    //funcionPrueba(std::move(myVector));


    for (int i = 0; i < 1000000; i++)
    {
        //myVector.push_back(v);
        myVector.emplace_back(v);
    }

    const auto end = std::chrono::high_resolution_clock::now();

    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    std::cout << ms.count() << " ms" << std::endl;

}