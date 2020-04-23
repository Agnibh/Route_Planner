#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    RoutePlanner::start_node = &m_Model.FindClosestNode(start_x,start_y);
    RoutePlanner::end_node = &m_Model.FindClosestNode(end_x,end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    float h;
    h = node->distance(*RoutePlanner::end_node);
    return h;
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for(RouteModel::Node *curr:current_node->neighbors)
    {
        if(curr->visited==false)
        {
            curr->parent = current_node;
            curr->g_value = curr->parent->g_value + curr->distance(*current_node);
            curr->h_value = RoutePlanner::CalculateHValue(curr);
            curr->visited = true;
            RoutePlanner::open_list.push_back(curr);
        }
    }

}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    RouteModel::Node *ans = nullptr;
    float sum = std::numeric_limits<float>::max();
    for(RouteModel::Node *curr : RoutePlanner::open_list)
    {
            float comp = curr->g_value + curr->h_value;
            if(comp<sum)
            {
                sum = comp;
                ans = curr;
            }

    }
    open_list.erase(std::remove(open_list.begin(),open_list.end(),ans),open_list.end());
    return ans;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    RouteModel::Node *curr = current_node;
    while(curr != RoutePlanner::start_node)
    {
        distance += curr->distance(*curr->parent);
        path_found.push_back(*curr);
        curr = curr->parent;
    }
    path_found.push_back(*curr);
    std::reverse(path_found.begin(),path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    // TODO: Implement your solution here.
    current_node = RoutePlanner::start_node;
    current_node->g_value = 0;
    current_node->visited = true;
    current_node->h_value = CalculateHValue(current_node);

    AddNeighbors(current_node);
    while(open_list.size()!=0  && current_node != end_node)
    {
        current_node=NextNode();
        AddNeighbors(current_node);
        if(current_node==end_node)
        {
            m_Model.path = ConstructFinalPath(current_node);
            break;
        }
    }
}