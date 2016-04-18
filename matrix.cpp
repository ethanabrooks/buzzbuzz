#include "light.h"
#include "matrix.h"
#include "wall.h"
#include <armadillo>
#include <set>
#include <algorithm>
#include <QVector>
#include <QDebug>

using namespace std;
using namespace arma;

float T_WIDTH = 30;
float WALL_INSET = 5;
float NODE_OFFSET = 5;
float BOARD_SIZE = 500;

ostream& operator<<(ostream& os, const Node& node)
{
  os << node.coordinate[0] << ", " << node.coordinate[1] << endl;
  for (Node neighbor : node.neighbors) {
    vec coord = neighbor.coordinate;
    os << "- " << coord[0] << ", " << coord[1] << endl;
  }
  return os;
}

glm::vec2 setLength(glm::vec2 v, float length) {
    return glm::normalize(v) * length;
}

bool withinLight(glm::vec2 mosquitoPos,
                 QList<Light*> lights,
                 QList<Wall*> walls) {
    for (Light* light : lights) {
        bool wallInWay = false;
        for (Wall* wall : walls) {
            if (wall->isInvalidMove(mosquitoPos, light->getPosition())) {
                wallInWay = true;
            }
        }
        glm::vec2 lightPos = light->getPosition();
        double distance =  sqrt(pow(mosquitoPos.x - lightPos.x, 2)
                              + pow(mosquitoPos.y - lightPos.y, 2));
        if (distance < light->radius && !wallInWay) {
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

glm::vec2 Node::glm() {
    return nodeToGlm(*this);
}

mat getCoords(QVector<QVector<int> >* board,
              QList<Light*> lights,
              QList<Wall*> walls) {
    vector <double> coords;
    int numMosqs = 0;
    for (int i = 0; i < board->size(); i++) {
        for (int j = 0; j < board->at(i).size(); j++) {
            if ((*board)[i][j] == 1) {
                if (!withinLight(glm::vec2({i, j}), lights, walls)) {
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

struct SingularMatrixException : public exception {
  const char* what () const throw () {
    return "Cannot invert a singular matrix";
  }
};

vec getLineParams(vec p1, vec p2) {
  mat line1 = join_horiz(p1, p2).t();
  mat x = line1.col(0);
  mat y = line1.col(1);
  mat augX = join_horiz(x, ones(x.size()));
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

bool inBounds(Node n) {
    return liesBetween(vec{0, 0},
                       vec{BOARD_SIZE, BOARD_SIZE},
                       n.coordinate);
}

Node extend(glm::vec2 near, glm::vec2 far) {
    glm::vec2 offset = setLength(near - far, NODE_OFFSET);
    return Node(near + offset);
}

graph graphBetween(vec here, vec there, QList<Wall*> walls) {
    vector<Node> nodes = {Node(here), Node(there)};
    for (Wall* wall : walls) {
        nodes.push_back(extend(wall->point1, wall->point2));
        nodes.push_back(extend(wall->point2, wall->point1));
    }
//    cout << "Num nodes " << nodes.size() << endl;
    graph neighbors;
    for (Node n : nodes) {
        neighbors[n] = {};
    }
    for (long i = 0; i < nodes.size(); i++) {
        for (long j = 0; j < nodes.size(); j++) {
//            cout << "here" <<  endl << nodes[i] << endl;
//            cout << "there" <<  endl << nodes[j] << endl;
            bool straightShot = true;
            for (Wall* wall : walls) {
                if (wall->isInvalidMove(nodes[i].glm(), nodes[j].glm())
                        || !inBounds(nodes[i])
                        || !inBounds(nodes[j])) {

//                    cout << "Not valid" << endl;
//                    cout << "here" <<  endl << nodes[i].coordinate[0] << " " << 500 - nodes[i].coordinate[1] << endl;
//                    cout << "there" <<  endl << nodes[j].coordinate[0] << " " << 500 - nodes[j].coordinate[1] << endl;
                    straightShot = false;
                }
            }
            if (straightShot) {
                neighbors[nodes[i]].push_back(nodes[j]);
            }
//            cout << "connected? " << straightShot << endl;
        }
    }
//    for (Node n0 : nodes) {
//        cout << endl << "next" << endl << n0.coordinate[0] << '\t' << 500 - n0.coordinate[1] << endl;
//        for (Node n_ : neighbors[n0]) {
//            cout <<  n_.coordinate[0] << "\t"
//                 << 500 - n_.coordinate[1] << endl;
//        }
//    }
    return neighbors;
}



Wall getTWall(glm::vec2 near, glm::vec2 far) {
    vec params = getLineParams(glmToArma(near), glmToArma(far));
    double a = getLineParams(glmToArma(near), glmToArma(far))[0];
    glm::vec2 offset = setLength(glm::vec2(a, -1), T_WIDTH);
    glm::vec2 inset = setLength(far - near, WALL_INSET);
    near = near + inset;
    return Wall(near + offset, near - offset);
}
