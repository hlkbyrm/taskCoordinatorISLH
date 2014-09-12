#include <ros/ros.h>
#include <QTimer>
#include <QVector>
#include <QThread>
#include <QObject>
#include <QTime>
#include <QtCore/QString>
//#include <messageDecoderISLH/taskInfo2LeaderMessage.h>
#include <ISLH_msgs/taskInfoFromLeaderMessage.h>
//#include <taskObserverISLH/newTaskInfoMessage.h>
#include <ISLH_msgs/cmd2LeadersMessage.h>
#include <ISLH_msgs/robotPositions.h>

/*
enum HandlingState
{
    HS_IDLE = 0,
    HS_WAITING_TASK_RESPONSE_FROM_LEADER = 1,
    HS_WAITING_GOAL_RESPONSE_FROM_LEADER = 2
};
*/

enum TaskStatus
{
    TS_NOT_ASSIGNED = 0,
    TS_WAITING = 1,
    TS_HANDLING = 2,
    TS_COMPLETED = 3,
    TS_NOT_COMPLETED = 4
};

enum CoalitionStatus
{
    CS_IDLE = 0,
    CS_WAITING_TASK_RESPONSE_FROM_COORDINATOR = 1,
    CS_SUCCORING = 2,
    CS_HANDLING = 3,
    CS_WAITING_GOAL_POSE = 4
};


enum Leader2CoordinatorInfoMgs
{
    INFO_L2C_INSUFFICIENT_RESOURCE = 1,
    INFO_L2C_START_HANDLING = 2,
    INFO_L2C_START_HANDLING_WITH_TASK_INFO = 3,
    INFO_L2C_TASK_COMPLETED = 4,
    INFO_L2C_SPLITTING = 5,
    INFO_L2C_SPLITTING_AND_LEADER_CHANGED = 6,
    INFO_L2C_WAITING_GOAL_POSE = 7,
    INFO_L2C_WAITING_TASK_SITE_POSE = 8
};

enum Coordinator2LeaderCmdMsgs
{
    CMD_C2L_START_OR_STOP_MISSION = 0,
    CMD_C2L_COALITION_MEMBERS = 1,
    CMD_C2L_LEADER_CHANGE = 2,
    CMD_C2L_NEW_GOAL_POSES = 3,
    CMD_C2L_NEW_TASK_SITE_POSES = 4
};

struct coalValFuncParams{
    double w1;
    double w2;
    double w3;
    double ro;
};

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
    //QString coalUUID;
    QVector <robotProp> coalMembers;
    QVector <double> coalTotalResources;
    uint coalLeaderID;
    int status;
     // CS_IDLE 0 -> moving to the given goal positions while detecting tasks
     // CS_WAITING_TASK_RESPONSE_FROM_COORDINATOR 1 -> waiting for help
     // CS_SUCCORING 2 -> waiting for the robots to reach to their goal positions in the coalition
     // CS_HANDLING 3 -> handling a task
     // CS_WAITING_GOAL_POSE 4 -> waiting for new goal positions
    QString currentTaskUUID; // the taskUUID if the coalition has a task
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

  coalProp engagedCoalition; // the coalition is handling this task
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

     ros::Subscriber messagePoseListSub;

     QVector <coalProp> coalList; // current coalitions

     QVector < QVector <coalProp> > coalListHist; // tracking the evolution of the coalitions

     QVector <taskProp> handlingTasks;
     QVector <taskProp> waitingTasks;
     QVector <taskProp> completedTasks;
     QVector <taskProp> timedoutTasks;

     coalValFuncParams cvfParams; // the parameters w1, w2, w3, adn ro in the coalition value function


     QVector <QVector <QVector <uint> > > generatePartitions(QVector <uint> robotList);

     void manageCoalitions();

     void handlePoseList(ISLH_msgs::robotPositions robotPoseListMsg);

     void handleTaskInfoFromLeader(ISLH_msgs::taskInfoFromLeaderMessage infoMsg);

     void runCFG(QVector <int> wTaskIDList, QVector <int> availCoalIDList);

     double calcCoalValue(QVector <robotProp> coalMmbrs, taskProp wTask);

     int findCoalition(int coalID, int wTaskID, QVector <int> wTaskIDList);

     void mergeCoalitions(int coalID, int mCoalID);

     int findSplittedRobot(int coalID, int taskID);

     void splitCoalition(int splitRobotID, int coalID);

     bool readConfigFile(QString filename);


     ros::Timer ct; // timer for checking waitingTasks

    // the coordinator checks waitingTasks every taskCheckingPeriod seconds
     void taskCheckingTimerCallback(const ros::TimerEvent&);

     int taskCheckingPeriod; // in seconds

     bool startChecking;

public slots:
     void work();

     void shutdownROS();


signals:
   void rosFinished();
   void rosStarted();
   void rosStartFailed();

};
