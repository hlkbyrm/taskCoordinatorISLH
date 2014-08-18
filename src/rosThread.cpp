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

    ros::Rate loop(10);

    while(ros::ok())
    {

        manageCoalitions();

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
}

void RosThread::manageCoalitions()
{

}

// handle incoming task info from a coalition leader
void RosThread::handleTaskInfoFromLeader(messageDecoderISLH::taskInfoFromLeaderMessage infoMsg)
{

    if (infoMsg.infoTypeID == INFO_L2C_INSUFFICIENT_RESOURCE)
    {

        taskProp newTask;

        newTask.encounteringRobotID = infoMsg.senderRobotID;

        newTask.taskUUID = QString::fromStdString(infoMsg.taskUUID);

        newTask.pose.X = infoMsg.posX;

        newTask.pose.Y = infoMsg.posY;

        newTask.encounteringTime = infoMsg.encounteringTime;

        newTask.requiredResourcesString = QString::fromStdString(infoMsg.requiredResources);


        QStringList resourceParts = newTask.requiredResourcesString.split(",",QString::SkipEmptyParts);

        newTask.requiredResources.clear();

        for(int i = 0; i < resourceParts.size();i++)
        {
            newTask.requiredResources.append(resourceParts.at(i).toDouble());
        }

        waitingTasks.append(newTask);

    }
    else if  (infoMsg.infoTypeID == INFO_L2C_START_HANDLING)
    {


/*
        data.append("&");

        data.append(QString::fromStdString(taskInfoMsg.taskUUID));
        */
    }
    else if  (infoMsg.infoTypeID == INFO_L2C_TASK_COMPLETED)
    {

    }
    else if (infoMsg.infoTypeID == INFO_L2C_SPLITTING)
    {
        /*
        data.append("&");

        data.append(QString::fromStdString(taskInfoMsg.extraMsg));
        */
    }

}
void RosThread::taskCheckingTimerCallback(const ros::TimerEvent&)
{


}
