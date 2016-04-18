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

float T_WIDTH = 20;
float WALL_INSET = 10;
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

bool inBounds(Node n) {
    double x = n.coordinate[0];
    double y = n.coordinate[1];

    return 0 <= x
           && x <= BOARD_SIZE
           && 0 <= y
           && y <= BOARD_SIZE;
}

graph graphBetween(vec here, vec there, QList<Wall*> walls) {
    vector<Node> nodes = {Node(here), Node(there)};
    for (Wall* wall : walls) {
        Node n1 = Node(wall->point1);
        Node n2 = Node(wall->point2);
        if (inBounds(n1)) {
            nodes.push_back(n1);
        }
        if (inBounds(n2)) {
            nodes.push_back(n2);
        }
    }
    graph neighbors;
    for (Node n : nodes) {
        neighbors[n] = {};
    }
    for (long i = 0; i < nodes.size(); i++) {
        for (long j = 0; j < nodes.size(); j++) {
            bool straightShot = true;
            for (Wall* wall : walls) {
                if (wall->isInvalidMove(nodes[i].glm(), nodes[j].glm())) {

                    straightShot = false;
                }
            }
            if (straightShot) {
                neighbors[nodes[i]].push_back(nodes[j]);
            }
        }
    }
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
