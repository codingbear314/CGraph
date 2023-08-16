#ifndef graph_h
#define graph_h

//#define fasttime  //this will pass checking if the input is vaild.

/**
 * Graph.h & Graph.cpp
 *
 * Version 2.0
 *
 * Upgrade notes
 * 1.0 note:   this code features a graph class that can run Dijkstra's
 * algorithm, BFS, and DFS. 1.2 note:   this code now features a directed graph
 * class. 1.4 note:   enhanced dijkstra's algorithm speed. 1.6 note:   added a
 * fasttime flag, fixed BFS and DFS bugs, and added comments. 1.7 note:   added
 * a sort_graph function. 1.8 note:   added a return option to returun the full
 * list 2.0 note:   modified the way to use the graph, seperated the file to
 * two. Added the Bellman-Ford algorithm. added namespace; GraphTheory
 *
 * Use is at your risk.
 *
 * Code written by Jaewook Jung, 2023
 *
 * visit by blog! bit.ly/codingbear314
 *
 * @version 2.0
 * @author Jaewook Jung
 * @date 2023/08/16
 */

#include <iostream>

#include <queue>
#include <stack>
#include <string>
#include <vector>

#include <algorithm>
#include <climits>
#include <stdexcept>

namespace GraphTheory
{
typedef bool Directiontype;
inline constexpr bool Directed = true;
inline constexpr bool Undirected = false;

template <Directiontype isdirected> class Graph
{
  private:
    std::vector<std::vector<std::pair<int, int>>> adj;
    int size;
    bool ContainsNegativeEdges;

  public:
    Graph(int);

    void add_edge(int, int, int);

    int get_size() const;
    const std::vector<std::pair<int, int>> &get_adj(int) const;
    void print_graph() const;
    void print_adj(int) const;

    void sort_graph(void);

    int dijkstra(int, int, std::vector<int> &) const;
    int dijkstra(int, int) const;
    void dijkstra(int, std::vector<int> &) const;

    int BellmanFord(int, int) const;
    int BellmanFord(int, int, std::vector<int> &) const;
    int BellmanFord(int, std::vector<int> &) const;

    void bfs(int, void (*)(int)) const;
    void dfs(int, void (*)(int)) const;
};
} // namespace GraphTheory
#include "Graph.cpp"
#endif