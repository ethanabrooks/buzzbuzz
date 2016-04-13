#include "light.h"
#include "matrix.h"
#include <armadillo>
#include <QVector>

using namespace std;
using namespace arma;

bool withinLight(glm::vec2 objPos,
                 QList<Light*> lights) {
    for (Light* light : lights) {
        glm::vec2 lightPos = light->getPosition();
        double distance =  sqrt(pow(objPos.x - lightPos.x, 2)
                              + pow(objPos.y - lightPos.y, 2));
        if (distance < light->radius) {
            return true;
        }
    }
    return false;
}

vec glmToArma(glm::vec2 v) {
    return vec({v.x, v.y});
}

mat getCoords(QVector<QVector<int> >* board,
              QList<Light*> lights = QList<Light*>()) {
    vector <double> coords;
    int numMosqs = 0;
    for (int i = 0; i < board->size(); i++) {
        for (int j = 0; j < board->at(i).size(); j++) {
            if ((*board)[i][j] == 1) {
                if (!withinLight(glm::vec2({i, j}), lights)) {
                    numMosqs++;
                    for (auto k: {i, j}) {
                        coords.push_back((float) k);
                    }
                }
            }
            if ((*board)[i][j] == 2) { // frog
                for (auto k: {i, j}) {
                    coords.push_back((float) k);
                }
            }
        }
    }
    return mat (&coords[0], // pointer to data
               2, numMosqs, // shape
               false, // copy_aux_mem
               true); // strict
}

mat getCentroids(mat coords, int num) {
    mat centroids;
    bool success = kmeans(centroids, coords, num,
           random_spread, // initializer
           10, // num rounds
           false); // print progress?
    return centroids;
}



vec getLineParams(vec p1, vec p2) {
  mat line1 = join_horiz(p1, p2).t();
  mat x = line1.col(0);
  mat y = line1.col(1);
  mat augX = join_horiz(x, ones(x.size()));
  return solve(augX, y);
}

mat getIntercept(vec startpoint1, vec endpoint1,
                vec startpoint2, vec endpoint2) {
  vec lineParams1 = getLineParams(startpoint1, endpoint1);
  vec lineParams2 = getLineParams(startpoint2, endpoint2);
  mat params = join_horiz(lineParams1, lineParams2).t();
  vec xParams = params.col(0);
  mat augXParams = join_horiz(-xParams, ones(xParams.size()));
  return solve(augXParams, params.col(1));
}

bool liesBetween(vec linePoint1, vec linePoint2, vec point) {
  double x1 = linePoint1[0];
  double x2 = linePoint2[0];
  double xPoint = point[0];
  return (x1 < xPoint && xPoint < x2) || (x1 > xPoint && xPoint > x2);
}

bool intersects(vec startpoint1, vec endpoint1,
                 vec startpoint2, vec endpoint2) {
  vec intercept = getIntercept(startpoint1, endpoint1, startpoint2, endpoint2);
  return liesBetween(startpoint1, endpoint1, intercept)
      && liesBetween(startpoint2, endpoint2, intercept);
}

bool intersects(vec startpoint, vec endpoint, wall_nodes wall) {
  return intersects(startpoint, endpoint,
      wall.point1->coordinate, wall.point2->coordinate);
}

void addEdgesBetween(Node* here, Node* there, vector<wall_nodes> walls) {
  bool straightShot = true;
  for (wall_nodes wall : walls) {
    if (intersects(here->coordinate, there->coordinate, wall)) {
      straightShot = false;
      addEdgesBetween(here, wall.point1, walls);
      addEdgesBetween(here, wall.point2, walls);
      addEdgesBetween(wall.point1, there, walls);
      addEdgesBetween(wall.point2, there, walls);
    }
  }
  if (straightShot) {
    here->addNeighbor(there);
  }
}

Node* getWallNode(vec near, vec far, double lightRadius) {
  vec offset = normalise(near - far);
  vec position = near + offset;
  return new Node(position);
}

ostream& operator<<(ostream& os, const Node& node)
{
  os << node.coordinate[0] << ", " << node.coordinate[1] << endl;
  for (Node* neighbor : node.neighbors) {
    os << "- " << *neighbor << endl;
  }
  return os;
}
