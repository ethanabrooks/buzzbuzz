#include <armadillo>
#include <QVector>

using namespace std;
using namespace arma;

mat getCoords(QVector<QVector<int> >* board, QList<Light*> lights);
mat getCentroids(mat coords, int num);
vector<vec> getDistVecs(mat centroids, QList<Light*> lights, bool replace_centroids);
