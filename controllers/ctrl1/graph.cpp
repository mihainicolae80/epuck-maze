#include "graph.h"


dir oposite_dir(dir direction){

  if(direction == DIR_NORTH)
    return DIR_SOUTH;
  else if(direction == DIR_SOUTH)
    return DIR_NORTH;
  else if(direction == DIR_EAST)
    return DIR_WEST;
  else if(direction == DIR_WEST)
    return DIR_EAST;

return DIR_NONE;
}

Node::Node(){}

Node::Node(int x, int y){
  this->x = x;
  this->y = y;
  open_on_dir[0] =
  open_on_dir[1] =
  open_on_dir[2] =
  open_on_dir[3] = false;

  visited_on_dir[0] =
  visited_on_dir[1] =
  visited_on_dir[2] =
  visited_on_dir[3] = false;
}


Graph::Graph(){
  nodecount = 0;
  lastnode = NULL; 
}

Node* Graph::on_intersection(int x, int y,dir direction,bool opennorth,bool opensouth,bool openeast, bool openwest){
  Node *cnode,newnode;
  std::list<Node*>::iterator it;
  cnode = is_node_arround(x,y);

  std::cout<<"ORIENTATION="<<direction<<std::endl;

  //Daca este o intersectie noua
  if(cnode == NULL){
    std::cout << "Intersectie noua x="<<x<<"y="<<y<< std::endl;
    //Setari nod
    newnode.x = x;
    newnode.y = y;
    newnode.nr = nodecount++;
    newnode.open_on_dir[DIR_NORTH] = opennorth;
    newnode.open_on_dir[DIR_SOUTH] = opensouth;
    newnode.open_on_dir[DIR_EAST]  = openeast;
    newnode.open_on_dir[DIR_WEST]  = openwest;
    newnode.visited_on_dir[DIR_NORTH] = false;
    newnode.visited_on_dir[DIR_SOUTH] = false;
    newnode.visited_on_dir[DIR_WEST]  = false;
    newnode.visited_on_dir[DIR_EAST]  = false;
    newnode.visited_on_dir[oposite_dir(direction)] = true;

    //Adauga Nodul
    allnodes.push_back(newnode);

    std::cout<<"a"<<std::endl;

    if(lastnode != NULL){
      std::cout<<"b"<<std::endl;
      lastnode->neighbours.push_back(&allnodes.back());
      std::cout<<"c"<<std::endl;
      allnodes.back().neighbours.push_back(lastnode);

      lastnode->visited_on_dir[direction] = true;
    }

    cnode = &allnodes.back();
  }
  else{
    std::cout << "Intersectie veche x="<<x<<"y="<<y<< std::endl;
    //Nodul curent a fost vizitat pe o directie
    cnode->visited_on_dir[oposite_dir(direction)] = true;
    if(lastnode != NULL){
      //Nodul anterior a fost vizitat pe o directie
      lastnode->visited_on_dir[direction] = true;

      //Adauga legatura
      //daca nodul curent nu este in lista vecinilor
      //lui lastnode
      bool gasit = false;
      for(it = lastnode->neighbours.begin();
          it != lastnode->neighbours.end();
          it++ ){

          //S-a gasit nodul in lista vecinilor
          if((*it)->nr == lastnode->nr){
            gasit = true;
            break;
          }
      }

      //Daca nodul vechi nu il avea in lista vecinilor
      //pe cel nou
      if(!gasit){
        lastnode->neighbours.push_back(cnode);
      }

      gasit = false;

      //Daca nodul anterior nu este in lista vecinilor nodului curent
      for(it = cnode->neighbours.begin();
          it != cnode->neighbours.end();
          it++ ){

          //S-a gasit nodul in lista vecinilor
          if((*it)->nr == cnode->nr){
            gasit = true;
            break;
          }
      }

      if(!gasit){
        cnode->neighbours.push_back(lastnode);
      }
    }
  }

  lastnode = cnode;

  return cnode;
}

Node* Graph::is_node_arround(int x,int y){
  std::list<Node>::iterator it;

  for(it = allnodes.begin(); it != allnodes.end(); it++){
    if(x > (*it).x - SEEK_NODE_DIST && x < (*it).x + SEEK_NODE_DIST
       && y > (*it).y - SEEK_NODE_DIST && y < (*it).y + SEEK_NODE_DIST){

         return &(*it);
    }
  }
  return NULL;
}
