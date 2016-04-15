#include "light.h"
#include "matrix.h"
#include <armadillo>
#include <set>
#include <QVector>
#include <QDebug>

using namespace std;
using namespace arma;

mutex mtx;

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

vec getVerticalIntercept(double x, mat params) {
    return vec({x, dot(params, vec({x, 1}))});
}

bool intersects(vec startpoint1, vec endpoint1, vec startpoint2, vec endpoint2) {
  vec lineParams1 = getLineParams(startpoint1, endpoint1);
  vec lineParams2 = getLineParams(startpoint2, endpoint2);

  vec intercept;

  double x1 = startpoint1[0];
  double x2 = startpoint2[0];

  bool line1isVertical = x1 == endpoint1[0];
  bool line2isVertical = x2 == endpoint2[0];

  if (line1isVertical && line2isVertical) {
    return false; // the lines are parallel; they don't intersect;
  }
  if (line1isVertical) {
    intercept = getVerticalIntercept(x1, lineParams2);
  } else if (line2isVertical) {
    intercept = getVerticalIntercept(x2, lineParams1);
  } else {
    mat params = join_horiz(lineParams1, lineParams2).t();
    vec xParams = params.col(0);
    mat augXParams = join_horiz(-xParams, ones(xParams.size()));
    if (det(augXParams) == 0) { // this means that the lines are parallel
      return false;
    }
    intercept = solve(augXParams, params.col(1));
  }
  return liesBetween(startpoint1, endpoint1, intercept)
      && liesBetween(startpoint2, endpoint2, intercept);
}

bool intersects(vec startpoint, vec endpoint, wall_nodes wall) {
  return intersects(startpoint, endpoint,
      wall.point1.coordinate, wall.point2.coordinate);
}

Node graphBetween(Node here, Node there, vector<wall_nodes> walls) {
  bool straightShot = true;
  for (wall_nodes wall : walls) {
    if (intersects(here.coordinate, there.coordinate, wall)) {
      straightShot = false;
      Node wall1 = graphBetween(wall.point1, there, walls);
      Node wall2 =graphBetween(wall.point2, there, walls);
      here = graphBetween(here, wall1, walls);
      return graphBetween(here, wall2, walls);
    }
  }
  if (straightShot) {
    return here.withNeighbor(there);
  }
}

Node getWallNode(vec near, vec far, double lightRadius) {
  vec offset = normalise(near - far);
  vec position = near + offset;
  vector<double> bounds = {0, 500};
  vector<vec> corners;
  for (double xCoord: bounds) {
      for (double yCoord: bounds) {
          corners.push_back(vec({xCoord, yCoord}));
      }
  }
  vector<Pair<vec, vec>> edges;
  for (int i = 0; i < corners.size() + 1; i++) {
      int j = (i + 1) % corners.size();
      edges.push_back(Pair<corners[i], corners[j]>);
  }
  for (Pair<vec, vec> edge :  edges) {
      if (intersects(near, offset, edge.first, edge.second)) {
          position = getIntercept(near, offset, edge.first, edge.second);
      }
  }
  return Node(position);
}

ostream& operator<<(ostream& os, const Node& node)
{
  os << node.coordinate[0] << ", " << node.coordinate[1] << endl;
  for (Node neighbor : node.neighbors) {
    os << "- " << neighbor << endl;
  }
  return os;
}
