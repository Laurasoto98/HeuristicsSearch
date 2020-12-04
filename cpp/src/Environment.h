#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <cstdio>
#include <cmath>
#include <fstream>

class Vector2d
{
  public:
    double x, y;
    Vector2d()                   : x(0),y(0) {}
    Vector2d(double x, double y) : x(x),y(y) {}
    Vector2d operator+(const Vector2d &other) const
      { return Vector2d(x+other.x, y+other.y); }
    Vector2d operator-(const Vector2d &other) const
      { return Vector2d(x-other.x, y-other.y); }
    Vector2d operator*(double s) const
      { return Vector2d(x*s, y*s); }
    double abs() const
      { return sqrt(x*x + y*y); }
};



class Polygon
{
  public:
    int n;
    Vector2d *v;
    Polygon(int n) : n(n)
    {
      v = new Vector2d[n];
    }
    ~Polygon()
    {
      delete[] v;
    }
    bool lineCollision(Vector2d p1, Vector2d p2, Vector2d p3, Vector2d p4, double tolerance)
    {
      double tmp = (p1.x-p2.x)*(p3.y-p4.y)-(p1.y-p2.y)*(p3.x-p4.x);
      if(tmp == 0)
        return false;
      Vector2d p(((p1.x*p2.y-p1.y*p2.x)*(p3.x-p4.x)
                 -(p1.x-p2.x)*(p3.x*p4.y-p3.y*p4.x))/tmp,
                 ((p1.x*p2.y-p1.y*p2.x)*(p3.y-p4.y)
                 -(p1.y-p2.y)*(p3.x*p4.y-p3.y*p4.x))/tmp);
      double l1 = ((p3.x-p4.x)*(p.x-p4.x)+(p3.y-p4.y)*(p.y-p4.y))
                  /(pow(p3.x-p4.x,2)+pow(p3.y-p4.y,2));
      double l2 = ((p1.x-p2.x)*(p.x-p2.x)+(p1.y-p2.y)*(p.y-p2.y))
                  /(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2));
      return (l1>=0 && l1<=1 && l2>0+tolerance && l2<1-tolerance);
    }
    bool checkCollision(Vector2d p1, Vector2d p2, double tolerance=0.001)
    {
      for(int i=0; i<n; i++)
        if(lineCollision(p1,p2, v[i],v[(i+1)%n], tolerance))
          return true;
      for(int i=0; i<n; i++)
        if(lineCollision(p1,p2, v[i],v[(i+2)%n], tolerance))
          return true;
      return false;
    }
    bool checkCollision(Vector2d p)
    {
      Vector2d center(0,0);
      for(int i=0; i<n; i++)
        center = center + v[i];
      return !checkCollision(p, center*(1.0/n), -0.001);
    }
};



class Environment
{
  //private:
  public:
    std::vector<Polygon*> obstacles;
    double width, height;

    Environment(double width, double height) : width(width),height(height)
    {}
    ~Environment()
    {
      for(unsigned int i=0; i<obstacles.size(); i++)
        delete obstacles[i];
    }

    void readFromFile(char* filename)
    {
      std::ifstream envfile;
      envfile.open(filename);
      int polygons, i, j, vertices;
      envfile >> polygons;
      for (i = 0; i < polygons; i++)
      {
        envfile >> vertices;
        Polygon *p = new Polygon(vertices);
        double x, y;
        for (j = 0; j < vertices; j++)
        {
          envfile >> x;
          envfile >> y;
          p->v[j] = Vector2d(x, y);
        }
        obstacles.push_back(p);
      }
    }

    static void printPath(std::string searchName, std::vector<Vector2d> *path)
    {
      FILE* f = fopen(("output/" + searchName + ".js").c_str(), "w");
      fprintf(f, "window.%s =\n\t[\n", searchName.c_str());
      for (unsigned int i=0; i<path->size(); i++)
      {
        fprintf(f, "\t\t[%f, %f]", path->at(i).x, path->at(i).y);
        if (i < path->size() - 1)
          fprintf(f, ",\n");
        else
          fprintf(f, "\n");
      }
      fprintf(f, "\t];");
      fclose(f);
    }
};

#endif
