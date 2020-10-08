#ifndef SEARCH_H
#define SEARCH_H
#include <queue> 
#include <iostream>
#include <limits>
#include <cmath>

class Search
{
  public:
    Search() {}
    virtual ~Search() {}
    virtual std::vector<Vector2d>* search(Environment* env) = 0;
    virtual std::string getName() = 0;

    bool isGoal(Vector2d position, Vector2d goal){
      return (position.x == goal.x && position.y == goal.y);
    }

    double heuristics(Vector2d position, Vector2d goal){
      return (position-goal).abs();
    }

    double costToSuccessor(Vector2d position, Vector2d successor){
      return (position-successor).abs();
    }

    bool successorInList(std::vector<Vector2d>* list, Vector2d successor){
      for(int i=0; i<list->size(); i++){
        if(list->at(i).x==successor.x && list->at(i).y==successor.y){
          return true;
        }
      }
      return false;
    }
    
    void eraseNode(std::vector<Vector2d>* list, Vector2d successor){
      for(int i=0; i<list->size(); i++){
        if(list->at(i).x==successor.x && list->at(i).y==successor.y){
          list->erase(list->begin()+i);
        }
      }
    }

    void newList(std::vector<Vector2d>* costList, Vector2d position, Environment* env, std::vector<Vector2d>* visited){
      Vector2d start(0,0);
      Vector2d goal(800,600);
      Vector2d child;
      Vector2d bestChild;
      for(int i=0; i<env->obstacles.size(); i++){
        double min = costToSuccessor(position, goal);
        for(int j=0; j<env->obstacles[i]->n; j++){
          child=env->obstacles[i]->v[j];
          if(!successorInList(visited, child)){
            if((costToSuccessor(position,child))< min){
              bestChild=env->obstacles[i]->v[j];
              min=costToSuccessor(position, child);
            }
          }
        }
        costList->push_back(bestChild);
      }
    }
};

class GreedySearch : public Search
{
  public:
    std::string getName() { return "greedy"; }
    std::vector<Vector2d>* search(Environment* env)
    {
      std::vector<Vector2d>* path = new std::vector<Vector2d>();
      std::vector<Vector2d>* visited = new std::vector<Vector2d>();
      Vector2d start(0,0);
      Vector2d goal(800,600);
      Vector2d child;
      Vector2d bestChild;
      Vector2d position=start;
      path->push_back(position);
      int cont=0;
      
      while(!isGoal(position,goal)){
      int min=heuristics(start,goal);
      for(int i=0; i<env->obstacles.size(); i++){
        for(int j=0; j<env->obstacles[i]->n; j++){
          child=env->obstacles[i]->v[j];
          if(heuristics(child, goal)< min && !successorInList(visited, child)){
            min=heuristics(child,goal);
            bestChild=env->obstacles[i]->v[j];
          }
        }
      }
      if(min>heuristics(position,goal)){
        path->push_back(goal);
        bestChild=goal;
      }
      visited->push_back(bestChild);
      path->push_back(bestChild);
      position=bestChild;
      cont++;
      }
      return path;
      

    }
};

class UniformCostSearch : public Search
{
  public:
    std::string getName() { return "uniformcost"; }
    std::vector<Vector2d>* search(Environment* env)
    {
      
      std::vector<Vector2d>* path = new std::vector<Vector2d>();
      std::vector<Vector2d>* costList = new std::vector<Vector2d>();
      std::vector<Vector2d>* visited = new std::vector<Vector2d>();
      Vector2d start(0,0);
      Vector2d goal(800,600);
      Vector2d position=start;
      path->push_back(start);
      visited->push_back(start);
      double pathCost = costToSuccessor(position,start);
  
    int cont=0;

    while(cont<48){
      newList(costList, position, env, visited);
      int x=0;
      int min=costToSuccessor(position,goal);
        for(int i=0; i<costList->size();i++){
          if((costToSuccessor(position,costList->at(i)))<min){
            min=costToSuccessor(position,costList->at(i));
            x=i;
          }
        }
        path->push_back(costList->at(x));
        visited->push_back(costList->at(x));
        position=costList->at(x);
        costList->clear();
        cont++;
        pathCost+=min;
    } 
      path->push_back(goal);
      return path;
    }
};


class AStarSearch : public Search
{
  public:
    std::string getName() { return "astar"; }

    std::vector<Vector2d>* search(Environment* env)
    {
      std::vector<Vector2d>* path = new std::vector<Vector2d>();
      std::vector<Vector2d>* costList = new std::vector<Vector2d>();
      std::vector<Vector2d>* visited = new std::vector<Vector2d>();
      Vector2d start(0,0);
      Vector2d goal(800,600);
      Vector2d position=start;
      path->push_back(start);
      visited->push_back(start);
      double pathCost=costToSuccessor(start,start);

    while(!isGoal(position,goal)){
      newList(costList, position, env, visited);
      int x=0;
      int min=costToSuccessor(position,goal);
        for(int i=0; i<costList->size();i++){
          if(costToSuccessor(position,costList->at(i))<min){
            if(heuristics(costList->at(i),goal)<heuristics(position,goal)){
              min=costToSuccessor(position,costList->at(i));
              x=i;
            }
          }
        }
        if(heuristics(position,goal)<heuristics(costList->at(x),goal)){
          path->push_back(goal);
          position=goal;
          break;
        }
        else{
          path->push_back(costList->at(x));
          visited->push_back(costList->at(x));
          position=costList->at(x);
          costList->clear();
        } 
      }
      return path;
    }
};


#endif
