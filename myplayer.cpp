#include "myplayer.h"
#include "matrix.h"
#include <armadillo>
#include <QDebug>
#include <math.h>
#include <set>

using namespace std;
using namespace arma;

std::vector<Node>& runDijkstra(Node currentPosition, Node destination);

int numLights = 4;
mat centroids;
vector <vec> velocities;
vector <wall_nodes> wallNodesList;


double getDistance(Node vertex1, Node vertex2) {
    double xdiff = vertex1.coordinate[0] - vertex2.coordinate[0];
    double ydiff = vertex1.coordinate[1] - vertex2.coordinate[1];
    double distance;
    distance = pow(xdiff, 2) + pow(ydiff, 2);
    distance = sqrt(distance);
    return distance;
}

double getTotalDistance(Node& vertex1, Node& vertex2) {
    double totalDistance = 0;
    vector<Node>& path = runDijkstra(vertex1, vertex2);
    Node prev = vertex1;
    Node next = path.at(1);
    for(int i = 2; i < path.size(); i++) {
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

std::vector<Node>& runDijkstra(Node currentPosition, Node destination) {
    std::vector<Node>* path = new std::vector<Node>;
    std::vector<Node*>::iterator it = currentPosition.neighbors.begin();
    while(it != currentPosition.neighbors.end()) {
        if(**it == destination) {
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
            for(Node* i : current.neighbors) {
                map<vec,double>::iterator it = dist.begin();
                while(it != dist.end()) {
                    if(it->first[0] == i->coordinate[0] && it->first[1] == i->coordinate[1] ) break;
                    it++;

                }
                map<vec,double>::iterator pre = dist.begin();
                while(pre != dist.end()) {
                    if(pre->first[0] == current.coordinate[0] && pre->first[1] == current.coordinate[1] ) break;
                    pre++;
                }
                if(it == dist.end() || it->second > pre->second + getDistance(*i, current)) {
                 qDebug() << "making it"<<endl;
                 dist[i->coordinate] = pre->second + getDistance(*i, current);
                 prev[*i] = current.coordinate;
                 active.insert(*i);
                }
            }
        }
    }
    map<Node, vec>::iterator pathIt = prev.find(destination);
    path->insert(path->begin(), Node(destination.coordinate));
    while(pathIt != prev.find(currentPosition)) {
        vec p = pathIt->second;
        qDebug() << p[0] << endl;
        path->insert(path->begin(),Node(p));
        //path->insert(path->begin(),*next);
        pathIt = prev.find(Node(p));
    }
    qDebug() << "size: " << path->size() << endl;
    return *path;



}

vec getDelta(vec light, vec centroid) {
  vector<Node> path = runDijkstra(Node(light), Node(centroid));
  return path[0].coordinate - light;
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
        Node lightPos = Node(glmToArma(light->getPosition()));

        // get direction of shortest distance
        vector<vec>::iterator closestCentroid =
          min_element(available.begin(), available.end(),
              [&](vec c1, vec c2){
                Node n1(c1), n2(c2);
                addEdgesBetween(&n1, &lightPos, wallNodesList);
                addEdgesBetween(&n2, &lightPos, wallNodesList);
                return getTotalDistance(lightPos, n1) < getTotalDistance(lightPos, n2);
              });
        Node centroidNode(*closestCentroid);
        addEdgesBetween(&lightPos, &centroidNode, wallNodesList);
        vector<Node> path = runDijkstra(lightPos, centroidNode);
        deltas.push_back(path[0].coordinate - lightPos.coordinate);
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
    return glm::vec2({250.0f, 250.0f});
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
  mat coords = getCoords(board, QList<Light*>());
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
    vector<vec> positions = {vec({75, 75}),
                                    vec({75, 425}),
                                    vec({425, 75}),
                                    vec({425, 425})};
    for (int i = 0; i < this->lights.size(); i++) {
        Light* light = this->lights.at(i);
        light->trailColor = colors[i];

        // position lights at centroids
        vec pos = positions[i];
        light->setInitialPosition(pos[0], pos[1]);

        velocities.push_back(vec({0, 0}));
    }

    for (Wall* wall : this->walls) {
        double lightRadius = this->lights[0]->radius;
        vec p1 = glmToArma(wall->point1);
        vec p2 = glmToArma(wall->point2);
        wall_nodes* nodes = new wall_nodes;
        nodes->point1 = getWallNode(p1, p2, lightRadius);
        nodes->point2 = getWallNode(p2, p1, lightRadius);
        wallNodesList.push_back(*nodes);
    }
}

/*
 * This method is called once per "step" in the simulation.
 * You can change the position of each Light by calling its moveTo method with the new x/y coordinate.
 * You may not, however, move a Light by more than one unit, and you may not move it through a wall!
 * The parameter specifies the number of mosquitoes at board->at(x).at(y)
 * You can access the walls through this object's "walls" field, which is a vector of Wall*
 */
void MyPlayer::updateLights(QVector<QVector<int> >* board) {

    // coordinates of mosquitos outside light
    mat coords = getCoords(board, this->lights);
    vector<vec> deltas;
    int numLights = this->lights.size();
    int numMosqsToCatch = size(coords)[1];

    float smoothing = 3000; //1500;
    float acceleration = 1 / (smoothing * cbrt(numMosqsToCatch) + 1);
    if (numMosqsToCatch < numLights) {
        if (numMosqsToCatch == 0) {
            centroids = vec({250, 250}); // go to the frog
        } else {
            centroids = coords; // go to remaining mosquitos
        }
        deltas = getDistVecs(centroids, this->lights,
                                           true); // more than one light per centroid
    } else {
        centroids = getCentroids(coords, this->lights.size());
        deltas = getDistVecs(centroids, this->lights,
                                           false); // one light per centroid
    }
    for (int i = 0; i < this->lights.length(); i++) {

        // this gets the current position of the light
        glm::vec2 currPos = this->lights.at(i)->getPosition();

        // MEREDITH: THIS IS PROBABLY WHERE YOU WANT TO ADD YOUR CODE:
        // get adjacency list
        Node source = Node(glmToArma(currPos));
        Node dest = Node(centroids.col(i));
        addEdgesBetween(&source, &dest, wallNodesList);
        // AT THIS POINT, source IS THE ROOT OF A GRAPH THAT YOU CAN TRAVERSE WITH DIJKSTRA'S
        // WE ALSO DON'T WANT TO HARD CODE DEST -- BUT WE CAN TALK ABOUT THAT.

        // can't change ligth position more than one unit
        vec velocity = normalise(velocities[i] + acceleration * deltas[i]) / 2;
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

