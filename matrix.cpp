#include "light.h"
#include "matrix.h"
#include "wall.h"
#include <armadillo>
#include <set>
#include <QVector>
#include <QDebug>

using namespace std;
using namespace arma;

int WALL_OFFSET = 20;
double BOARD_SIZE = 500;

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

glm::vec2 armaToGlm(vec v) {
    return glm::vec2(v[0], v[1]);
}

glm::vec2 nodeToGlm(Node n) {
    return armaToGlm(n.coordinate);
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
        }
    }
    return mat (&coords[0], // pointer to data
               2, numMosqs, // shape
               false, // copy_aux_mem
               true); // strict
}

mat getCentroids(mat coords, int num) {
    mat centroids;
    kmeans(centroids, coords, num,
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
  if (det(augX) == 0) {
    throw SingularMatrixException();
  }
  return solve(augX, y);
}

bool liesBetween(vec linePoint1, vec linePoint2, vec point) {
  double x1 = linePoint1[0];
  double x2 = linePoint2[0];
  double xPoint = point[0];
  double e = .01;
  if (x1 == x2) {
    x1 = linePoint1[1];
    x2 = linePoint2[1];
    xPoint = point[1];
  }
  return (x1 + e < xPoint && xPoint + e < x2)
      || (x1 > xPoint + e && xPoint > x2 + e);
}
Node graphBetween(Node here, Node there, QList<Wall*> walls) {
  bool straightShot = true;
  for (Wall* wall : walls) {
    vec wallStart = glmToArma(wall->point1), wallEnd = glmToArma(wall->point2);

    glm:: vec2 glmHere = nodeToGlm(here), glmThere = nodeToGlm(there);
    if (wall->isInvalidMove(glmHere, glmThere)
            && glmHere != wall->point1
            && glmHere != wall->point2
            && glmThere != wall->point1
            && glmThere != wall->point2) {
      straightShot = false;
      if (inBounds(wallStart)) {
          Node wall1 = graphBetween(wallStart, there, walls);
          here = graphBetween(here, wall1, walls);
      }
      if (inBounds(wallEnd)) {
          Node wall2 =graphBetween(wallEnd, there, walls);
          here = graphBetween(here, wall2, walls);
      }
      return here;
    }
  }
  if (straightShot) {
    return here.withNeighbor(there);
  }
}

Wall getTWall(glm::vec2 w1, glm::vec2 w2) {
    vec params = getLineParams(glmToArma(w1), glmToArma(w2));
    double a = getLineParams(glmToArma(w1), glmToArma(w2))[0];
    glm::vec2 offset = armaToGlm(WALL_OFFSET *
                                 normalise(vec({a, -1})));
    return Wall(w1 + offset, w1 - offset);
}

ostream& operator<<(ostream& os, const Node& node)
{
  os << node.coordinate[0] << ", " << node.coordinate[1] << endl;
  for (Node neighbor : node.neighbors) {
    os << "- " << neighbor << endl;
  }
  return os;
}
