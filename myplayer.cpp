#include "myplayer.h"
#include "matrix.h"
#include <armadillo>
#include <QDebug>
#include <math.h>
#include <set>
#include <queue>
#include <float.h>

using namespace std;
using namespace arma;


int roundNum = 0;
int numLights = 4;
mat centroids;
vector <vec> velocities;
vec FROG_POS = {250, 250};
vector<vec> POSITIONS = {vec({70, 70}),
                         vec({430, 70}),
                         vec({430, 430}),
                         vec({70, 430})};

float SMOOTHING = 10;
float WALL_OFFSET = 40;
int START_HEAT_SEEKING = 200;
bool DEBUG = true;
vector<Wall> newWalls;

//struct less<vec>{
//   bool operator() (const vec& lhs, const vec& rhs) const {
//       return lhs[0] == rhs[0] ? lhs[1] < rhs[1] : lhs[0] < rhs[0];
//   }
//};


//struct greater<Node>{
//   bool operator() (const Node& lhs, const Node& rhs) const {
//       return distances[lhs.coordinate] > distances[rhs.coordinate];
//   }
//};


double getDistance(Node vertex1, Node vertex2) {
    double xdiff = vertex1[0] - vertex2[0];
    double ydiff = vertex1[1] - vertex2[1];
    double distance;
    distance = pow(xdiff, 2) + pow(ydiff, 2);
    distance = sqrt(distance);
    return distance;
}


Node nextDestination(vector<Node> path) {
    switch (path.size()) {
        case 0: throw "path must not be empty";
        case 1:
            return path[0];
        default:
            return path[1];
    }
}

vector<Node> runDijkstra(Node currentPosition, Node destination, graph allNeighbors) {
   vector<Node> path;
   map<vec, double> distances;
   auto cmp = [](Node lhs, Node rhs) {
       return distances[lhs] < distances[rhs];
   };
   priority_queue<Node, vector<Node>, decltype(cmp)> active(cmp);
   map<Node, vector<Node>> paths;
   currentPosition.distance = 0.0;
   for(pair<Node, vector<Node>> neighbor: allNeighbors) {
       distances[neighbor.first.coordinate] = DBL_MAX;
       paths[neighbor.first.coordinate] = {};
       active.push(neighbor.first);
   }
   distances[currentPosition.coordinate] = 0;
   paths[currentPosition] = {currentPosition};
   while(!active.empty()) {
     Node current = active.top();
     active.pop();
     for(Node neighbor : allNeighbors[current]) {
         map<vec,double>::iterator neighborDist = distances.find(neighbor.coordinate);
         map<vec,double>::iterator currentDist = distances.find(current.coordinate);
         double distThroughCurrent = currentDist->second + getDistance(neighbor, current);
         if(neighborDist->second > distThroughCurrent) {
           distances[neighbor.coordinate] = distThroughCurrent;
           paths[neighbor] = paths[current];
           paths[neighbor].push_back(current);
         }
      }
   }
//   map<Node, vec>::iterator prev = paths.find(destination);
//   path.insert(path.begin(), Node(destination.coordinate));
//   while(prev != paths.find(currentPosition)) {
//       vec p = prev->second;
//       path.insert(path.begin(),Node(p));
//       prev = paths.find(Node(p));
////       pathIt++;
//   }
   vector<Node> path = paths[currentPosition.coordinate];
   cout << "path size " << path.size() << endl;
   return path;
}


double getTotalDistance(vec coordinate1, vec coordinate2, QList<Wall*> walls) {
    graph g = graphBetween(coordinate1, coordinate2, walls);
    double totalDistance = 0;
    Node vertex1(coordinate1), vertex2(coordinate2);
    vector<Node> path = runDijkstra(vertex1, vertex2, g);
    Node prev = vertex1;
    Node next = nextDestination(path);
    for(int i = 2; i < int(path.size()); i++) {
        totalDistance+=getDistance(prev, next);
        prev = next;
        next = path.at(i);
    }
    totalDistance+=getDistance(prev, next);
    return totalDistance;
}


int  k = 1;

vec getDelta(Light* light, vec destination, QList<Wall*> walls) {
    vec lightPos = glmToArma(light->getPosition());
    graph g = graphBetween(lightPos, destination, walls);
    vector<Node> path = runDijkstra(Node(lightPos), Node(destination), g);
    vec delta = normalise(nextDestination(path).coordinate - lightPos);
    if (DEBUG) {
        cout << "lightPos" << endl;
        cout << lightPos << endl;
        cout << endl << "graph" << endl;
        for (Node n : g[Node(lightPos)]) {
            cout << "-" << n << endl;
        }
        cout << endl << "path" << endl;
        for (Node n : path) {
            cout << n << endl;
        }
        cout << "end path" << endl;
        cout << "Light pos " <<  k << " " << lightPos[0] << endl;
        cout << "Light pos " <<  k << " " << lightPos[1] << endl;
        cout << "next dest " <<  k << " " << nextDestination(path).coordinate[0] << endl;
        cout << "next dest " <<  k << " " << nextDestination(path).coordinate[1] << endl;
        cout << "dest " <<  k << " " << destination[0] << endl;
        cout << "dest " <<  k << " " << destination[1] << endl;
        cout << "delta " <<  k << " " << delta[0] << endl;
        cout << "delta " <<  k << " " << delta[1] << endl;
        k++;
        if (k > 4) {k = 1;}
    }
    return delta;
}

vector<vec> getDistVecs(mat centroids,
                        QList<Light*> lights,
                        bool replace_centroids,
                        QList<Wall*> walls) {
    vector<vec> deltas;
    vector<vec> available;
    centroids.each_col([&](vec& centroidPos){
        available.push_back(centroidPos);
    });
    int i = 0;
    for (Light* light : lights) {
        i++;
        // convert glm::vec to vec
        vec lightPos = glmToArma(light->getPosition());

        // get direction of shortest distance
        vector<vec>::iterator closestCentroid =
          min_element(available.begin(), available.end(),
              [&](vec c1, vec c2){
                double toC1 = getDistance(Node(lightPos), Node(c1));
                double toC2 = getDistance(Node(lightPos), Node(c2));
                return toC1 < toC2;
              });
        vec delta = getDelta(light, *closestCentroid, walls);
        deltas.push_back(delta);
        if (!replace_centroids) {
            available.erase(closestCentroid );
        }
    }
    return deltas;
}

MyPlayer::MyPlayer()
{
    this->playerName = "My Player";
}


/*
 * This method is called once at the start of the game.
 * The parameter specifies the number of mosquitoes at board->at(x).at(y), or board[x][y]
 * The return value is the vector specifying the x/y coordinates of the frog.
 * You can access the walls through this object's "walls" field, which is a vector of Wall*
 */
glm::vec2 MyPlayer::initializeFrog(QVector<QVector<int> >* board) {

    /*
     * This places the frog in the center.
     * But you can place it anywhere you like!
     */
    return armaToGlm(FROG_POS);
}

Wall extendWall(Wall w) {
    return Wall(extend(w.point1, w.point2, WALL_OFFSET),
                extend(w.point2, w.point1, WALL_OFFSET));
}



/*
 * Thisd method is called once at the start of the game.
 * You should set the initial position of each of the four lights.
 * The parameter specifies the number of mosquitoes at board->at(x).at(y), or board[x][y]
 * You can change the colors if you'd like!
 * You can access the walls through this object's "walls" field, which is a vector of Wall*
 */
void MyPlayer::initializeLights(QVector<QVector<int> >* board) {
  //TODO: make empty list default argument
  mat coords = getCoords(board, this->lights, this->walls);
  centroids = getCentroids(coords, this->lights.size());

    /*
    // this prints out debugging information about the walls
    qDebug() << "initializeLights: number of walls " << this->walls.size();
    for (int i = 0; i < this->walls.size(); i++) {
        Wall* wall = this->walls.at(i);
        qDebug() << "there is a wall from (" << wall->point1.x << "," << wall->point1.y << ") to (" << wall->point2.x << "," << wall->point2.y << ")" << endl;
    }
    */

    /*
    // this prints out debugging information about the mosquitoes
    for (int x = 0; x < board->size(); x++) {
        for (int y = 0; y < board->at(x).size(); y++) {
            int numMosquitoes = board->at(x).at(y);
            if (numMosquitoes > 0)
              qDebug() << "There are " << numMosquitoes << " mosquitoes at (" << x << ", " << y << ")" << endl;
        }
    }
    */

    vector<QColor> colors = { 
      QColor(255, 255, 255), 
      QColor(0, 255, 255),
      QColor(255, 255, 0),
      QColor(255, 0, 255)
    };

    for (int i = 0; i < this->lights.size(); i++) {
        Light* light = this->lights.at(i);
        light->trailColor = colors[i];

        // position lights at centroids
        vec pos = POSITIONS[i];
        light->setInitialPosition(pos[0], pos[1]);

        velocities.push_back(vec({0, 0}));
    }

    for (Wall* wall : this->walls) {
        Wall t1 = getTWall(wall->point1, wall->point2);
        Wall t2 = getTWall(wall->point2, wall->point1);
        newWalls.push_back(t1);
        newWalls.push_back(t2);
        newWalls.push_back(extendWall(*wall));
    }
    this->walls.clear();
    cout << "newWalls size " << newWalls.size() << endl;
    for (int i = 0; i < int(newWalls.size()) ; i++) {
        this->walls.push_back(&newWalls[i]);
    }
    cout << "walls size " << this->walls.size() << endl;
}

/*
 * This method is called once per "step" in the simulation.
 * You can change the position of each Light by calling its moveTo method with the new x/y coordinate.
 * You may not, however, move a Light by more than one unit, and you may not move it through a wall!
 * The parameter specifies the number of mosquitoes at board->at(x).at(y)
 * You can access the walls through this object's "walls" field, which is a vector of Wall*
 */
void MyPlayer::updateLights(QVector<QVector<int> >* board) {

    roundNum++;
    // coordinates of mosquitos outside light
    mat coords = getCoords(board, this->lights, this->walls);
    vector<vec> deltas;
    int numMosqsToCatch = size(coords)[1];
    int numMosqsToLeave = 50;

    cout << "numMosqsToCatch " << numMosqsToCatch << endl;

    float acceleration = 1 / (SMOOTHING * cbrt(max(numMosqsToCatch - numMosqsToLeave, 0)) + 1);
    if (numMosqsToCatch < numMosqsToLeave) {
        centroids = FROG_POS; // go to the frog
        deltas = getDistVecs(centroids, this->lights,
                                           true, // more than one light per centroid
                                           this->walls);
    } else {
        if (roundNum > START_HEAT_SEEKING) {
            centroids = getCentroids(coords, this->lights.size());
            deltas = getDistVecs(centroids, this->lights,
                                            false, // one light per centroid
                                            this->walls);
        } else {
            vector<vec> destinations = POSITIONS;
            rotate(destinations.begin(), destinations.begin() + 1, destinations.end());
            deltas = vector<vec>();
            for (int i = 0; i < int(this->lights.size()); i++) {
                deltas.push_back(getDelta(this->lights[i], destinations[i], walls));
            }
        }
    }
    for (int i = 0; i < this->lights.length(); i++) {
        // this gets the current position of the light
        glm::vec2 currPos = this->lights.at(i)->getPosition();
        // can't change ligth position more than one unit
        vec velocity = normalise(velocities[i] + acceleration * deltas[i]) / 2;

        for (Wall* wall : this->walls) {
            if (wall->isInvalidMove(currPos, currPos + armaToGlm(velocity))) {
                velocity = velocity * -2;
            }
        }

        velocities[i] = velocity;
        this->lights.at(i)->moveTo(currPos.x+velocity[0],
                                   currPos.y+velocity[1]);


        /*
         * This is a pretty bad solution!
         * For part 1, modify this code so that the lights bring all the mosquitoes
         * to the frog within 5000 steps.
         */

    }
}

