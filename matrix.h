#ifndef MATRIX_H
#define MATRIX_H
#include <armadillo>
#include <QVector>
#include "light.h"

using namespace std;
using namespace arma;


class Node {
  public: vector<Node> neighbors;
          vec coordinate;
          Node(vec coordinate) {
              this->coordinate = coordinate;
              this->neighbors = {};
          }

          Node withNeighbor(Node neighbor) {
            Node newNode(this->coordinate);
            newNode.neighbors.push_back(neighbor);
            return newNode;
          }
          bool operator==(const Node& node) {
            return all(this->coordinate==node.coordinate);
          }

          bool operator<(const Node& node) const {
            vec a = this->coordinate;
            vec b = node.coordinate;
            return a[0] == b[0]? a[1] < b[1] : a[0] < b[0];
          }
};

typedef struct {
  Node* point1;
  Node* point2;
} wall_nodes;

Node* getWallNode(vec near, vec far, double lightRadius);
mat getCoords(QVector<QVector<int> >* board, QList<Light*> lights);
mat getCentroids(mat coords, int num);
vector<vec> getDistVecs(mat centroids, QList<Light*> lights, bool replace_centroids);
vec glmToArma(glm::vec2);
void addEdgesBetween(Node* here, Node* there, vector<wall_nodes> walls);
Node* getWallNode(vec near, vec far, double lightRadius);
#endif
