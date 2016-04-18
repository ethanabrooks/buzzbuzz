#ifndef MATRIX_H
#define MATRIX_H
#include <armadillo>
#include <QVector>
#include <QDebug>
#include "light.h"
#include "wall.h"

using namespace std;
using namespace arma;


class Node {
  public: vector<Node> neighbors;
          vec coordinate;
          double distance = DBL_MAX;

          Node() {
              this->coordinate = {0, 0};
              this->neighbors = {};
          }

          Node(vec coordinate) {
              this->coordinate = coordinate;
              this->neighbors = {};
          }

          Node(glm::vec2 v) {
              this->coordinate = {v.x, v.y};
              this->neighbors = {};
          }

          void addNeighbor(Node neighbor) {
              this->neighbors.push_back(neighbor);
          }

          Node withNeighbor(Node neighbor) {
            Node newNode = *this;
            newNode.neighbors.push_back(neighbor);
            return newNode;
          }

          bool operator==(const Node& node) {
            double e = .01;
            return all(abs(this->coordinate - node.coordinate) < e);
          }

          bool operator<(const Node& node) const {
            vec a = this->coordinate;
            vec b = node.coordinate;
            return a[0] == b[0]? a[1] < b[1] : a[0] < b[0];
          }

          bool hasNeighbor(Node n) {
              return find(neighbors.begin(), neighbors.end(), n) != neighbors.end();
          }

          glm::vec2 glm();
};

typedef map<Node, vector<Node>> graph;
Node getWallNode(vec near, vec far, double lightRadius);
mat getCoords(QVector<QVector<int> >* board, QList<Light*> lights, QList<Wall*> walls);
mat getCentroids(mat coords, int num);
vector<vec> getDistVecs(mat centroids, QList<Light*> lights, bool replace_centroids);
vec glmToArma(glm::vec2);
graph graphBetween(vec here, vec there, QList<Wall*> walls);
Wall getTWall(glm::vec2 w1, glm::vec2 w2);
glm::vec2 armaToGlm(vec v);
glm::vec2 setLength(glm::vec2 v, float length);
ostream& operator<<(ostream& os, const Node& node);
glm::vec2 extend(glm::vec2 near, glm::vec2 far, float offset);
#endif
