#ifndef MATRIX_H
#define MATRIX_H
#include <armadillo>
#include <QVector>
#include <QDebug>
#include "light.h"
#include "wall.h"

using namespace std;
using namespace arma;

typedef vec Node;
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
