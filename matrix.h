#ifndef MATRIX_H
#define MATRIX_H
#include <armadillo>
#include <QVector>
#include "light.h"

using namespace std;
using namespace arma;

class Node;

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
