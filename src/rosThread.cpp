#include "rosThread.h"
#include <QDebug>
#include <qjson/parser.h>
#include <QDir>
#include <QFile>



RosThread::RosThread()
{
    shutdown = false;

    taskCheckingPeriod = 10;
}
void RosThread::work()
{

    if(!ros::ok()){

        emit this->rosFinished();

        return;
    }

    emit rosStarted();



    ct = n.createTimer(ros::Duration(taskCheckingPeriod), &RosThread::taskCheckingTimerCallback, this);

    ct.stop();


    messageTaskInfoFromLeaderSub = n.subscribe("messageDecoder/taskInfoFromLeader",5,&RosThread::handleTaskInfoFromLeader, this);


    cmd2LeadersPub = n.advertise<taskCoordinatorISLH::cmd2LeadersMessage>("taskCoordinatorISLH/cmd2Leaders",5);


 //  messageNewTaskInfo = n.subscribe("taskObserverISLH/newTaskInfo",5,&RosThread::handleNewTask, this);

  //  messageTaskInfo2Leader = n.advertise<messageDecoderISLH::taskInfo2LeaderMessage>("taskHandlerISLH/taskInfo2Leader",5);

 /*
    messageIn = n.subscribe("communicationISLH/messageIn",5,&RosThread::handleIncomingMessage,this);
    messageOut = n.advertise<communicationISLH::outMessage>("messageDecoderISLH/messageOut",5);


    messageTaskInfo2Leader = n.subscribe("taskHandlerISLH/taskInfo2Leader",5,&RosThread::sendTaskInfo2Leader,this);

    messageTaskInfoFromLeader = n.advertise<messageDecoderISLH::cmdFromLeaderMessage>("messageDecoderISLH/cmdFromLeader",5);


    messageCmd2Robots = n.subscribe("coalitionLeaderISLH/cmd2Robots",5,&RosThread::sendCmd2Robots,this);

    messageRaw = n.subscribe("messageRaw",5,&RosThread::handleRawMessage,this);

*/

    ros::Rate loop(10);

    while(ros::ok())
    {


        ros::spinOnce();

        loop.sleep();

    }

    qDebug()<<"I am quitting";

    ros::shutdown();

    emit rosFinished();


}
void RosThread::shutdownROS()
{
    ros::shutdown();
    // shutdown = true;

}
// msg nin tipine dikkat!!!!
//
void RosThread::handleTaskInfoFromLeader(taskObserverISLH::newTaskInfoMessage msg)
{


}
void RosThread::taskCheckingTimerCallback(const ros::TimerEvent&)
{


}
