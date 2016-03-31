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

vector<vec> getDistVecs(mat centroids,
                        QList<Light*> lights,
                        bool replace_centroids=false) {
    vector<vec> deltas;
    vector<vec> available;
    centroids.each_col([&](vec& centroidPos){
        available.push_back(centroidPos);
    });
    for (Light* light : lights) {

        // convert glm::vec to vec
        glm::vec2 glmPos = light->getPosition();
        vec lightPos({glmPos.x, glmPos.y});

        // get direction of shortest distance
        vector<vec>::iterator closestCentroid =
          min_element(available.begin(), available.end(),
              [&](vec c1, vec c2){
                return norm(c1 - lightPos) < norm(c2 - lightPos);
              });
        deltas.push_back(*closestCentroid - lightPos);
        if (!replace_centroids) {
            available.erase(closestCentroid );
        }
    }
    return deltas;
}
