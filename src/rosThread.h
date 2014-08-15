#include <ros/ros.h>
#include <QTimer>
#include <QVector>
#include <QThread>
#include <QObject>
#include <QTime>
#include <QtCore/QString>
#include <messageDecoderISLH/taskInfo2LeaderMessage.h>
#include <taskObserverISLH/newTaskInfoMessage.h>
#include <taskCoordinatorISLH/cmd2LeadersMessage.h>

/*
enum HandlingState
{
    HS_IDLE = 0,
    HS_WAITING_TASK_RESPONSE_FROM_LEADER = 1,
    HS_WAITING_GOAL_RESPONSE_FROM_LEADER = 2
};
*/

struct poseXY{
    double X;
    double Y;
};

struct robotProp{
    uint robotID;
    QVector <double> resources;
    poseXY pose;
    bool inTaskSite;
    bool inGoalPose;
};

struct coalProp{
    uint coalID;
    QVector <robotProp> coalMembers;
    uint coalLeaderID;
};

// task properties
struct taskProp{
  QString taskUUID;
  uint encounteringTime; // in timestamp - "the time when the task is encountered"
  uint responsibleUnit;  // "who is responsible for the task"
  uint encounteringRobotID;  // "Id of the robot encountering the task"
  uint handlingDuration; // in seconds - "the required time to handle the task"
  uint timedOutDuration; // "the timed-out duration for the task"
  int status; // "status of the task"
              //
  uint startHandlingTime; // in timestamp - "the time when the task starts being handled"
  poseXY pose; // the location of the task
  QVector < double > requiredResources;
  QString requiredResourcesString;
};

class RosThread:public QObject
{
    Q_OBJECT

public:

    RosThread();


private:
     bool shutdown;

     ros::NodeHandle n;

     ros::Publisher cmd2LeadersPub;

     ros::Subscriber messageTaskInfoFromLeaderSub;

     QVector <coalProp> coalList;

     QVector <taskProp> waitingTasks;
     QVector <taskProp> completedTasks;
     QVector <taskProp> timedoutTasks;


     void handleTaskInfoFromLeader(taskObserverISLH::newTaskInfoMessage msg);


     ros::Timer ct;

    // the coordinator checks waitingTasks every taskCheckingPeriod seconds
     void taskCheckingTimerCallback(const ros::TimerEvent&);

     int taskCheckingPeriod; // in seconds



public slots:
     void work();

     void shutdownROS();


signals:
   void rosFinished();
   void rosStarted();
   void rosStartFailed();

};
