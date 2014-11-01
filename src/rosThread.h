#include <ros/ros.h>
#include <stdlib.h>
#include <QTimer>
#include <QVector>
#include <QThread>
#include <QObject>
#include <QTime>
#include <QtCore/QString>
#include <algorithm>
#include <ISLH_msgs/taskInfoFromLeaderMessage.h>
#include <ISLH_msgs/cmd2LeadersMessage.h>
#include <ISLH_msgs/robotPositions.h>
#include <ISLH_msgs/taskInfo2MonitorMessage.h>
#include "std_msgs/UInt8.h"
#include "std_msgs/Int8MultiArray.h"
#include "placerobots.h"

/*
enum TaskInfoType
{
    TI_NEW_TASK = 1,
    TI_STATUS_UPDATE = 2
};
*/

enum PoseType
{
    GOAL_POSE = 1,
    TASK_SITE_POSE = 2
};

enum TaskStatus
{
    TS_NOT_ASSIGNED = 0,
    TS_WAITING = 1,
    TS_WAITING_TASK_POSE = 2,
    TS_SUCCORING = 3,
    TS_HANDLING = 4,
    TS_COMPLETED = 5,
    TS_NOT_COMPLETED = 6
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
    CMD_C2L_NEW_TASK_SITE_POSES = 4,
    CMD_C2L_NEW_ALL_TARGET_POSES = 5
};

struct coalValFuncParameters{
    double w1;
    double w2;
    double w3;
};

struct missionParameters{
    bool startMission;
    int numOfRobots;
    int taskCheckingPeriod; // in seconds
    double targetSiteRadius;

    double ro; // workspace radius
};

struct poseXY{
    double X;
    double Y;
};

struct robotProp{
    uint coalID;
    uint robotID;
    QVector <double> resources;
    poseXY pose;
    poseXY goalPose;
    poseXY taskSitePose;
    double radius;
    int inTaskSite;
    int inGoalPose;
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
    int responsibleUnit;  // "who is responsible for the task"
    uint encounteringRobotID;  // "Id of the robot encountering the task"
    uint handlingDuration; // in seconds - "the required time to handle the task"
    uint timeOutDuration; // "the timed-out duration for the task"
    int status; // "status of the task"
    uint startHandlingTime; // in timestamp - "the time when the task starts being handled"
    poseXY pose; // the location of the task
    QVector < double > requiredResources;
    QString requiredResourcesString;
    double taskSiteRadius;

    coalProp engagedCoalition; // the coalition is handling this task
};

class RosThread:public QObject
{
    Q_OBJECT

public:

    RosThread();


private:

    int queueSize;

    int ownRobotID;

    int coordinatorRobotID;

    bool shutdown;

    double distThreshold4GeneratingPoses;

    ros::NodeHandle n;

    ros::Publisher cmd2LeadersPub;

    ros::Publisher leaderIDInfo2MonitorPub;

    ros::Publisher taskInfo2MonitorPub;

    ros::Subscriber messageTaskInfoFromLeaderSub;

    ros::Subscriber messagePoseListSub;

    ros::Subscriber messageStartMissionSub;

    QVector <coalProp> coalList; // current coalitions
    QVector <coalProp> _coalList;

    QVector < QVector <coalProp> > coalListHist; // tracking the evolution of the coalitions

    QVector <robotProp> robotsList; // denoting both robots' position and which robot belongs to which coalition in the current coalition structure

    QVector <taskProp> handlingTasks;
    QVector <taskProp> waitingTasks;
    QVector <taskProp> _waitingTasks;
    QVector <taskProp> completedTasks;
    QVector <taskProp> timedoutTasks;

    coalValFuncParameters cvfParams; // the parameters w1, w2, w3, adn ro in the coalition value function

    missionParameters missionParams;

    QVector <QVector <QVector <uint> > > generatePartitions(QVector <uint> robotList);

    void handleStartMission(std_msgs::UInt8 msg);

    void manageCoalitions();

    void pubTaskInfo2Monitor(taskProp task);

    void sendCmd2Leaders(int cmdType, QVector <int> coalIDList);

    void handlePoseList(ISLH_msgs::robotPositions robotPoseListMsg);

    void handleTaskInfoFromLeader(ISLH_msgs::taskInfoFromLeaderMessage infoMsg);

    void runCFG(QVector <int> wTaskIDList);//, QVector <int> availCoalIDList);

    double calcCoalValue(QVector <robotProp> coalMmbrs, taskProp wTask);

    int findCoalition(int coalID, int wTaskID, QVector <int> wTaskIDList);

    int mergeCoalitions(int coalID, int mCoalID); // return new leader robotID

    int findSplittedRobot(int coalID, int taskID);

    void splitCoalition(int splitRobotID, int coalID);

    QVector <double> calcCoalTotalResources(QVector <robotProp> coalMembers);

    void generatePoses (int coalID, int poseType);
    void generatePoses2(int coalID, int poseType);

    void matchCoalitionWithTarget(int coalID);
    bool is_line_segment_intersects(poseXY l1p1,poseXY l1p2,poseXY l2p1,poseXY l2p2);
    double dist(poseXY p1, poseXY p2);
    QVector<poseXY> matchRobotWithTarget(QVector<robotProp> robots);

    bool readConfigFile(QString filename);

    void writeNumOfIterationsToFile(int totNumOfMain, int totNumOfMerging, int totNumOfSplitting);


    ros::Timer ct; // timer for checking waitingTasks

    // the coordinator checks waitingTasks every taskCheckingPeriod seconds
    void taskCheckingTimerCallback(const ros::TimerEvent&);

    bool startChecking;



public slots:
    void work();

    void shutdownROS();


signals:
    void rosFinished();
    void rosStarted();
    void rosStartFailed();

};
