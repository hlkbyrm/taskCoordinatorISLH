#ifndef PLACEROBOTS_H
#define PLACEROBOTS_H

#include <cmath>
#include <QTimer>

struct Point{
    double x,y;
    Point(double x,double y):x(x),y(y){}
    Point():x(0),y(0){}

    Point operator+=(Point p){
        this->x += p.x;
        this->y += p.y;
        return *this;
    }
    Point operator-=(Point p){
        this->x -= p.x;
        this->y -= p.y;
        return *this;
    }

    Point operator+(Point& p){
        return Point(this->x + p.x,this->y + p.y);
    }
    Point operator-(Point& p){
        return Point(this->x - p.x,this->y - p.y);
    }
    Point operator*(double& d){
        return Point(this->x * d,this->y * d);
    }

    double length(){
        return sqrt(x*x+y*y);
    }
    Point get_normalized(){
        return Point(x/length(),y/length());
    }
};

class Robot{
public:
    Point center;
    double r;
    int robotID;
    Robot(Point center, double r,int robotID):center(center),r(r),robotID(robotID) {}
    Robot():center(Point(0,0)),r(0.0),robotID(0) {}
    Robot(const Robot &r):center(r.center),r(r.r),robotID(r.robotID) {}
};

class Obstacle{
public:
    Point center;
    double r;
    Obstacle(Point center, double r):center(center),r(r) {}
    Obstacle():center(Point(0,0)),r(0.0) {}
    Obstacle(const Obstacle &o):center(o.center),r(o.r) {}
};

class PlaceRobots{
public:
    QVector<Robot> robots;
    QVector<Obstacle> obstacles;
    double r_out;

private:
    int tryCount;
    int tryLimit;
    int bigTryCount;
    int bigTryLimit;

    double factor_of_OR;
    double factor_of_RR;
    double factor_of_Grav;
    double factor_of_OuterCircle;
    double factor_of_BigCircle;

    Point origin;
    double big_r;

    double dist(Point p1, Point p2){
        return sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
    }

    bool isOK(){
        foreach(Robot r,robots){
            if(dist(origin,r.center) > r_out - r.r)
                return false;

            if(dist(Point(0,0),r.center) > big_r - r.r)
                return false;

            foreach(Robot r2,robots){
                if(r.robotID == r2.robotID) continue;

                if(dist(r.center,r2.center) < r.r + r2.r)
                    return false;
            }
            foreach(Obstacle o,obstacles){
                if(dist(r.center,o.center) < r.r + o.r)
                    return false;
            }
        }
        return true;
    }

    void calcNext(){
        if(tryCount == -1 || tryCount >= tryLimit){
            bigTryCount++;
            for(int i=0;i<robots.size();i++){
                double r = (rand() / (RAND_MAX + 1.0)) * (r_out - robots[i].r);
                double alpha = (rand() / (RAND_MAX + 1.0)) * (2*M_PI);
                robots[i].center = Point(r * sin(alpha) + origin.x, r * cos(alpha) + origin.y);
            }
            tryCount = 0;
        }
        if(bigTryCount > bigTryLimit){
            tryCount = -1;
            bigTryCount = 0;
            r_out += 10;
        }

        for(int i=0;i<robots.size();i++){
            Robot r = robots[i];

            r.center += (origin - r.center).get_normalized() * factor_of_Grav;

            foreach(Obstacle o,obstacles){
                if(dist(o.center,r.center) < o.r + r.r){
                    r.center -= (o.center - r.center).get_normalized() * factor_of_OR;
                }
            }

            foreach(Robot r2,robots){
                if(r.robotID == r2.robotID) continue;

                if(dist(r.center,r2.center) < r.r + r2.r){
                    r.center -= (r2.center - r.center).get_normalized() * factor_of_RR;
                }
            }

            if(dist(origin,r.center) > r_out - r.r){
                r.center += (origin - r.center).get_normalized() * factor_of_OuterCircle;
            }

            if(dist(Point(0,0),r.center) > big_r - r.r){
                r.center += (Point(0,0) - r.center).get_normalized() * factor_of_BigCircle;
            }
            robots[i] = r;
        }

        tryCount++;
    }

    void firstInit(){
        tryCount = -1;
        tryLimit = 100;
        bigTryCount = -1;
        bigTryLimit = 15;

        factor_of_OR = 3.0;
        factor_of_RR = 2.0;
        factor_of_Grav = 0.25;
        factor_of_OuterCircle = 4.0;
        factor_of_BigCircle = 4.0;
    }

public:
    PlaceRobots(QVector<Robot> &r,QVector<Obstacle> &o,Point origin,double r_out,double big_r)
                                        :robots(r),obstacles(o),r_out(r_out),origin(origin),big_r(big_r) {firstInit();}

    void startCalculating(){
        while(!isOK()){
            calcNext();
        }
        tryCount = -1;
        bigTryCount = -1;
        return;
    }
};

#endif // PLACEROBOTS_H
