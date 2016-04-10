#include <iostream> 
#include <armadillo> 
#include <vector>
#include <map>
#include "matrix.h"

using namespace std; 
using namespace arma; 

class Node { 
  public: vector<Node> neighbors; 
          vec coordinate; 
          Node(vec coordinate) { 
            this->coordinate = coordinate; 
            this->neighbors = {}; 
          } 
          void addNeighbor(Node neighbor) { 
            this->neighbors.push_back(neighbor); 
          } 
          bool operator==(const Node& node) { 
            return all(this->coordinate==node.coordinate); 
          } 
};


