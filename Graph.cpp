#include "Graph.h"

#ifndef graph2_implementations__
#define graph2_implementations__

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
 * @date 2023/06/06
 */

namespace GraphTheory
{
/**
 * @brief Make an empty graph.
 * @param size the number of nodes in the graph
 */
template <Directiontype isdirected>
Graph<isdirected>::Graph(int size)
    : size(size), adj(size), ContainsNegativeEdges(false)
{
}

/**
 * @brief Add an edge to the graph.
 * @param start the starting vertex
 * @param finish the ending vertex
 * @param weight the weight of the edge
 * @return void
 */
template <Directiontype isdirected>
void Graph<isdirected>::add_edge(int start, int finish, int weight)
{
#ifndef fasttime
    if (!(start < this->size && finish < this->size))
        throw std::invalid_argument("Invalid index");
#endif
    adj[start].push_back(std::make_pair(finish, weight));
    if (isdirected == Undirected)
        adj[finish].push_back(std::make_pair(start, weight));
    if (weight < 0)
        this->ContainsNegativeEdges = true;
    return;
}

/**
 * @brief Gets the size of the graph.
 * @return the size of the graph.
 */
template <Directiontype isdirected> int Graph<isdirected>::get_size() const
{
    return this->size;
}

#ifdef EnableDebugTools
/**
 * @brief Gets the adjacency list of a vertex.
 * @param index the index of the wanted vertex
 * @return adjacency list of a vertex.
 */
template <Directiontype isdirected>
const std::vector<std::pair<int, int>> &
Graph<isdirected>::get_adj(int index) const
{
#ifndef fasttime
    return this->adj.at(index);
#else
    return this->adj[index];
#endif
}

/**
 * @brief Prints the graph for debugging purposes.
 * @return void
 */
template <Directiontype isdirected> void Graph<isdirected>::print_graph() const
{
    for (int i = 0; i < this->size; i++)
    {
        std::cout << i << " : ";
        for (auto j : this->adj[i])
            std::cout << "(" << j.first << ", " << j.second << ") ";
        std::cout << std::endl;
    }
    return;
}

/**
 * @brief Prints adj for debugging purposes.
 * @param index the index of the wanted vertex
 * @return void
 */
template <Directiontype isdirected>
void Graph<isdirected>::print_adj(int index) const
{
    std::cout << index << " : ";
    for (auto j : this->adj[index])
        std::cout << "(" << j.first << ", " << j.second << ") ";
}
#endif

/**
 * @brief Runs an dijkstra's algorithm on the graph.
 * It returns the distance to the finish vertex.
 * @pre The graph should not have any negative edges.
 *
 * @param start the starting vertex
 * @param finish the ending vertex
 * @param path the path to the finish vertex
 * @return the fastest length from start to finish. It will return -1 if it is
 * impossible to reach the node.
 */
template <Directiontype isdirected>
int Graph<isdirected>::dijkstra(int start, int finish,
                                std::vector<int> &path) const
{
    if (this->ContainsNegativeEdges)
        throw std::invalid_argument(
            "You tried to run dijkstra algorithm with negative edges. Use the "
            "Bellman-Ford algorithm.");
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>,
                        std::greater<std::pair<int, int>>>
        pq; // using priority_queue for faster speed
    std::vector<int> dist(
        this->size, INT_MAX); // the best way to come to the node since now
    dist[start] = 0;
    pq.push(std::make_pair(0, start));
    std::vector<std::vector<int>> pathfinder(this->size);
    pathfinder[start].push_back(start);
    while (!pq.empty()) // dijkstra's algorithm
    {
        std::pair<int, int> now = pq.top();
        pq.pop();
        if (dist[now.second] < now.first)
            continue;
        int index = now.second;
        for (auto i : this->adj.at(index))
        {
            if (dist[i.first] > dist[index] + i.second)
            {
                dist[i.first] = dist[index] + i.second;
                pathfinder[i.first] = pathfinder[index];
                pathfinder[i.first].push_back(i.first);
                pq.push(std::make_pair(dist[i.first], i.first));
            }
        }
    }
    path = pathfinder[finish];
#ifndef fasttime
    return dist[finish] == INT_MAX ? -1 : dist[finish];
#else
    return dist[finish];
#endif
}

/**
 * @brief Runs an dijkstra's algorithm on the graph.
 * It returns the distance to the finish vertex.
 *
 * @param start the starting vertex
 * @param finish the ending vertex
 * @return the fastest length from start to finish. It will return -1 if it is
 * impossible to reach the node.
 */
template <Directiontype isdirected>
int Graph<isdirected>::dijkstra(
    int start, int finish) const // without path; function overloading
{
    if (this->ContainsNegativeEdges)
        throw std::invalid_argument(
            "You tried to run dijkstra algorithm with negative edges. Use the "
            "Bellman-Ford algorithm.");
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>,
                        std::greater<std::pair<int, int>>>
        pq;
    std::vector<int> dist(this->size, INT_MAX);
    dist[start] = 0;
    pq.push(std::make_pair(0, start));
    while (!pq.empty())
    {
        std::pair<int, int> k = pq.top();
        pq.pop();
        if (dist[k.second] < k.first)
            continue;
        int index = k.second;
        for (auto i : this->adj.at(index))
        {
            if (dist[i.first] > dist[index] + i.second)
            {
                dist[i.first] = dist[index] + i.second;
                pq.push(std::make_pair(dist[i.first], i.first));
            }
        }
    }
#ifndef fasttime
    return dist[finish] == INT_MAX ? -1 : dist[finish];
#else
    return dist[finish];
#endif
}

/**
 * @brief Runs an dijkstra's algorithm on the graph.
 * It returns the distance to all vertex.
 *
 * @param start the staring vertex
 * @param dist the list that holds all the lengths
 */
template <Directiontype isdirected>
void Graph<isdirected>::dijkstra(int start, std::vector<int> &dist) const
{
    if (this->ContainsNegativeEdges)
        throw std::invalid_argument(
            "You tried to run dijkstra algorithm with negative edges. Use the "
            "Bellman-Ford algorithm.");
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>,
                        std::greater<std::pair<int, int>>>
        pq;
    std::fill(dist.begin(), dist.end(), INT_MAX);
    dist.resize(this->size, INT_MAX);
    dist[start] = 0;
    pq.push(std::make_pair(0, start));
    while (!pq.empty())
    {
        std::pair<int, int> k = pq.top();
        pq.pop();
        if (dist[k.second] < k.first)
            continue;
        int index = k.second;
        for (auto i : this->adj.at(index))
        {
            if (dist[i.first] > dist[index] + i.second)
            {
                dist[i.first] = dist[index] + i.second;
                pq.push(std::make_pair(dist[i.first], i.first));
            }
        }
    }
}

/**
 * @brief Runs Bellman-Ford algorithm on the graph.
 * It returns the distance to the finish vertex.
 *
 * @param start the starting vertex
 * @param finish the ending vertex
 * @return the fastest length from start to finish. It will return INT_MAX if it
 * is impossible to reach the node, and -1*INT_MAX if negative loops form.
 */
template <Directiontype isdirected>
int Graph<isdirected>::BellmanFord(int start, int finish) const
{
    std::vector<int> dist(this->size, INT_MAX);
    dist[start] = 0;
    for (int iteration = 1; iteration <= this->size; iteration++)
    {
        bool DidWeChangedAnything = false;
        for (int node = 0; node < this->size; node++)
        {
            if (dist[node] == INT_MAX)
                continue;
            for (auto i : this->adj.at(node))
            {
                if (dist[i.first] > dist[node] + i.second)
                {
                    dist[i.first] = dist[node] + i.second;
                    DidWeChangedAnything = true;
                }
            }
        }
        if (!DidWeChangedAnything)
            break;
        else if (iteration == this->size)
            return -1 * INT_MAX;
    }
    return dist[finish];
}

/**
 * @brief Runs Bellman-Ford algorithm on the graph.
 * It returns the distance to the finish vertex.
 *
 * @param start the starting vertex
 * @param finish the ending vertex
 * @param path the path to the finish vertex
 * @return the fastest length from start to finish. It will return INT_MAX if it
 * is impossible to reach the node, and -1*INT_MAX if negative loops form.
 */
template <Directiontype isdirected>
int Graph<isdirected>::BellmanFord(int start, int finish,
                                   std::vector<int> &path) const
{
    std::vector<int> dist(this->size, INT_MAX);
    dist[start] = 0;
    std::vector<std::vector<int>> pathfinder(this->size);
    pathfinder[start].push_back(start);
    for (int iteration = 1; iteration <= this->size; iteration++)
    {
        bool DidWeChangedAnything = false;
        for (int node = 0; node < this->size; node++)
        {
            if (dist[node] == INT_MAX)
                continue;
            for (auto i : this->adj.at(node))
            {
                if (dist[i.first] > dist[node] + i.second)
                {
                    dist[i.first] = dist[node] + i.second;
                    pathfinder[i.first] = pathfinder[node];
                    pathfinder[i.first].push_back(i.first);
                    DidWeChangedAnything = true;
                }
            }
        }
        if (!DidWeChangedAnything)
            break;
        else if (iteration == this->size)
            return -1 * INT_MAX;
    }
    path = pathfinder[finish];
    return dist[finish];
}

/**
 * @brief Runs Bellman-Ford algorithm on the graph.
 * It returns the distance to all vertex.
 *
 * @param start the staring vertex
 * @param dist the list that holds all the lengths
 * @return 0 if normal, -1 if negative loops are found
 */
template <Directiontype isdirected>
int Graph<isdirected>::BellmanFord(int start, std::vector<int> &dist) const
{
    std::fill(dist.begin(), dist.end(), INT_MAX);
    dist.resize(this->size, INT_MAX);
    dist[start] = 0;
    for (int iteration = 1; iteration <= this->size; iteration++)
    {
        bool DidWeChangedAnything = false;
        for (int node = 0; node < this->size; node++)
        {
            if (dist[node] == INT_MAX)
                continue;
            for (auto i : this->adj.at(node))
            {
                if (dist[i.first] > dist[node] + i.second)
                {
                    dist[i.first] = dist[node] + i.second;
                    DidWeChangedAnything = true;
                }
            }
        }
        if (!DidWeChangedAnything)
            break;
        else if (iteration == this->size)
            return -1;
    }
    return 0;
}

/**
 * @brief Runs a BFS on the graph, and runs visit function every time it visits
 * a node.
 *
 * @param start the starting vertex
 * @param visit the function to run on every node
 * @return void
 */
[[deprecated("Don\'t use this if you don\'t know exactly what you are doing. BFS and DFS in this library doesn\'t work as "
             "you think.")]] template <Directiontype isdirected>
void Graph<isdirected>::bfs(int start, void (*visit)(int)) const
{
    if (start > this->size)
        throw std::invalid_argument("Invalid starting point");

    std::queue<int> bfs_queue;
    std::vector<bool> visited(this->size, false);
    bfs_queue.push(start);
    while (!bfs_queue.empty())
    {
        int now = bfs_queue.front();
        bfs_queue.pop();
        visit(now);
        visited[now] = true;
        for (auto i : this->adj[now])
            if (!visited[i.first])
            {
                bfs_queue.push(i.first);
                visited[i.first] = true;
            }
    }
    return;
}

/**
 * @brief Runs a DFS on the graph, and runs visit function every time it visits
 * a node.
 *
 * @param start the starting vertex
 * @param visit the function to run on every node
 * @return void
 */
[[deprecated("Don\'t use this if you don\'t know exactly what you are doing. BFS and DFS in this library doesn\'t work as "
             "you think.")]] template <Directiontype isdirected>
void Graph<isdirected>::dfs(int start, void (*visit)(int)) const
{
    if (start > this->size)
        throw std::invalid_argument("Invalid starting point");

    std::stack<int> dfs_stack;
    std::vector<bool> visited(this->size, false);
    dfs_stack.push(start);
    while (!dfs_stack.empty())
    {
        int now = dfs_stack.top();
        dfs_stack.pop();
        visited[now] = true;
        visit(now);
        for (auto i : this->adj[now])
            if (!visited[i.first])
            {
                dfs_stack.push(i.first);
                visited[i.first] = true;
            }
    }
    return;
}
} // namespace GraphTheory

#endif