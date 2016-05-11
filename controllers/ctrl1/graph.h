#include <list>
#include <iostream>

//TODO Val mai mare?
#define SEEK_NODE_DIST 400

/*
  Distantele nu se retin in graf
  ci se calculeaza pe loc
  pe baza coordonatelor nodurilor.
*/

enum dir{
  DIR_NORTH = 0,
  DIR_EAST  = 1,
  DIR_SOUTH = 2,
  DIR_WEST  = 3,
  DIR_NONE  = 4
};


dir oposite_dir(dir direction);

struct Node{

  Node();
  Node(int x, int y);


  long x, y;
  int nr;
  bool open_on_dir[4];
  bool visited_on_dir[4];
  //TODO nu ar fi mai bine cu un map ?
  std::list<Node*> neighbours;
};

class Graph{
public:
  Graph();
  //Intoarce o directie in care sa mearga sau
  //stop in cazulin care nodul curent a fost
  //complet explorat
  Node* on_intersection(int x, int y,dir direction,
                        bool opennorth,bool opensouth,
                        bool openeast, bool openwest);
  Node* get_last_node();
private:
  Node* is_node_arround(int x, int y);
  Node *lastnode;
  std::list<Node> allnodes;
  int nodecount;
};
