#include <cstdio>
#include <cstdlib>
#include <vector>
#include <ctime>
#include <queue> 

#include "Environment.h"
#include "Search.h"




int main(int argc, char* argv[])
{   
  srand(time(NULL));

  Vector2d start(0,0);
  Vector2d goal(800,600);
    
  printf("Loading environment...\n");
  Environment *env = new Environment(800,600);
  env->readFromFile((char*)"output/environment.txt");
  printf("Loaded an environment with %lu obstacles.\n", env->obstacles.size());

  Search* searches[] = {
    (new GreedySearch()),
    (new UniformCostSearch()),
    (new AStarSearch())
  };

  
  for (int i = 0; i < 3; i++)
  {
    printf("Searching with %s search...\n", searches[i]->getName().c_str());
    Environment::printPath(searches[i]->getName().c_str(), searches[i]->search(env));
  } 

  delete env;
}
