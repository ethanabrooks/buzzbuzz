#include "myplayer.h"
#include "matrix.h"
#include <armadillo>
#include <QDebug>
#include <math.h>
#include <set>

using namespace std;
using namespace arma;

std::vector<Node>& runDijkstra(Node currentPosition, Node destination, graph allNeighbors);

int roundNum = 0;
int numLights = 4;
mat centroids;
vector <vec> velocities;
vec FROG_POS = {250, 250};
vector<vec> POSITIONS = {vec({430, 70}),
                                    vec({70, 430}),
                                    vec({70, 70}),
                                    vec({430, 430})};

float SMOOTHING = 10; //3000;
int NUM_WALLS = 6;
float WALL_OFFSET = 40;//20;
bool heat_seeking = false;
vector<Wall> newWalls;

double getDistance(Node vertex1, Node vertex2) {
    double xdiff = vertex1.coordinate[0] - vertex2.coordinate[0];
    double ydiff = vertex1.coordinate[1] - vertex2.coordinate[1];
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

namespace std
{
    template<> struct less<vec>
    {
       bool operator() (const vec& lhs, const vec& rhs) const
       {
           return lhs[0] < rhs[0];
       }
    };
}


std::vector<Node>& runDijkstra(Node currentPosition, Node destination, graph allNeighbors) {
    std::vector<Node>* path = new std::vector<Node>;
    std::vector<Node>::iterator it = allNeighbors[currentPosition].begin();
    while(it != allNeighbors[currentPosition].end()) {
        if(*it == destination) {
            path->push_back(currentPosition);
            path->push_back(destination);
            return *path;
        }
        it++;
    }
    std::map<vec, double> dist;
    std::map<Node, vec> prev;
    dist[currentPosition.coordinate] = 0.0;
    set<Node> active;
    active.insert(currentPosition);
    while(!active.empty()) {
        Node current = *active.begin();
        if(current == destination) {
            break;
        } else {
            active.erase(active.begin());
            for(Node i : allNeighbors[current]) {
                map<vec,double>::iterator it = dist.begin();
                while(it != dist.end()) {
                    if(it->first[0] == i.coordinate[0] && it->first[1] == i.coordinate[1] ) break;
                    it++;

                }
                map<vec,double>::iterator pre = dist.begin();
                while(pre != dist.end()) {
                    if(pre->first[0] == current.coordinate[0] && pre->first[1] == current.coordinate[1] ) break;
                    pre++;
                }
                if(it == dist.end() || it->second > pre->second + getDistance(i, current)) {
                 dist[i.coordinate] = pre->second + getDistance(i, current);
                 prev[i] = current.coordinate;
                 active.insert(i);
                }
            }
        }
    }
    map<Node, vec>::iterator pathIt = prev.find(destination);
    path->insert(path->begin(), Node(destination.coordinate));
    while(pathIt != prev.find(currentPosition)) {
        vec p = pathIt->second;
        path->insert(path->begin(),Node(p));
        pathIt = prev.find(Node(p));
    }
    return *path;
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
        graph g = graphBetween(lightPos, *closestCentroid, walls);
        vector<Node> path = runDijkstra(Node(lightPos), Node(*closestCentroid), g);
        cout << endl << "path" << endl;
        for (Node n : path) {
            cout << n << endl;
        }
        cout << "end path" << endl;
        vec delta = normalise(nextDestination(path).coordinate - lightPos);
        deltas.push_back(delta);
        cout << "Light pos " << i << " " << lightPos[0] << endl;
        cout << "Light pos " << i << " " << lightPos[1] << endl;
        cout << "next dest " << i << " " << nextDestination(path).coordinate[0] << endl;
        cout << "next dest " << i << " " << nextDestination(path).coordinate[1] << endl;
        cout << "dest " << i << " " << (*closestCentroid)[0] << endl;
        cout << "dest " << i << " " << (*closestCentroid)[1] << endl;
        cout << "delta " << i << " " << delta[0] << endl;
        cout << "delta " << i << " " << delta[1] << endl;
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
        cout << "ROUND NUM " << roundNum << endl;
        if (roundNum > 500) {
            centroids = getCentroids(coords, this->lights.size());
        } else {
            vector<vec> positions = POSITIONS;
            rotate(positions.begin(), positions.begin() + 1, positions.end());
            centroids = mat();
            for (vec pos : positions) {
                centroids = join_horiz(centroids, pos);
                cout << "centroids" << endl;
                cout << centroids << endl;
            }
        }
        deltas = getDistVecs(centroids, this->lights,
                                        false, // one light per centroid
                                        this->walls);
    }
    for (int i = 0; i < this->lights.length(); i++) {
        // this gets the current position of the light
        glm::vec2 currPos = this->lights.at(i)->getPosition();
        // can't change ligth position more than one unit
        vec velocity = normalise(velocities[i] + acceleration * deltas[i]) / 2;
        velocities[i] = velocity;
        cout << "currPos " << i << " " << currPos[0] << endl;
        cout << "currPos " << i << " " << currPos[1] << endl;
        cout << "velocity " << i << " " << velocity[0] << endl;
        cout << "velocity " << i << " " << velocity[1] << endl;


        this->lights.at(i)->moveTo(currPos.x+velocity[0],
                                   currPos.y+velocity[1]);


        /*
         * This is a pretty bad solution!
         * For part 1, modify this code so that the lights bring all the mosquitoes
         * to the frog within 5000 steps.
         */

    }
}

