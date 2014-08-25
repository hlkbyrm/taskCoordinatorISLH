#include "rosThread.h"
#include <QDebug>
#include <qjson/parser.h>
#include <QDir>
#include <QFile>



RosThread::RosThread()
{
    shutdown = false;

    taskCheckingPeriod = 10; //in seconds

    startChecking = false;


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

/*
    QVector <uint> robotList;

    robotList.append(1);

    robotList.append(2);

    robotList.append(3);

    robotList.append(4);

    robotList.append(5);

    robotList.append(6);

    robotList.append(7);

    robotList.append(8);

    robotList.append(9);

    robotList.append(10);


    QVector <QVector <QVector <uint> > > partitions;


    qDebug()<< "num: ";

    partitions =  generatePartitions(robotList);

     qDebug()<< "num of partitions: " << partitions.size();

*/
    ros::Rate loop(10);

    while(ros::ok())
    {

        if (startChecking)
        {
            manageCoalitions();

            startChecking = false;
        }

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

    // check whether all the resources of the available coalitions are sufficient for the waiting tasks



}

// handle incoming task info from a coalition leader
void RosThread::handleTaskInfoFromLeader(messageDecoderISLH::taskInfoFromLeaderMessage infoMsg)
{

    if (infoMsg.infoTypeID == INFO_L2C_INSUFFICIENT_RESOURCE)
    {
        taskProp newTask;

        newTask.encounteringRobotID = infoMsg.encounteringRobotID;

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

        for(int i = 0; i < coalList.size();i++)
        {
            if (coalList.at(i).coalLeaderID == infoMsg.senderRobotID)
            {
                coalList[i].status = CS_WAITING_TASK_RESPONSE_FROM_COORDINATOR;
                coalList[i].currentTaskUUID = QString::fromStdString(infoMsg.taskUUID);

                break;
            }
        }

    }
    else if  (infoMsg.infoTypeID == INFO_L2C_START_HANDLING_WITH_TASK_INFO)
    {
        taskProp newTask;

        newTask.encounteringRobotID = infoMsg.encounteringRobotID;

        newTask.taskUUID = QString::fromStdString(infoMsg.taskUUID);

        newTask.pose.X = infoMsg.posX;

        newTask.pose.Y = infoMsg.posY;

        newTask.encounteringTime = infoMsg.encounteringTime;

        newTask.startHandlingTime = infoMsg.startHandlingTime;

        newTask.requiredResourcesString = QString::fromStdString(infoMsg.requiredResources);

        QStringList resourceParts = newTask.requiredResourcesString.split(",",QString::SkipEmptyParts);

        newTask.requiredResources.clear();

        for(int i = 0; i < resourceParts.size();i++)
        {
            newTask.requiredResources.append(resourceParts.at(i).toDouble());
        }

        handlingTasks.append(newTask);

        for(int i = 0; i < coalList.size();i++)
        {
            if (coalList.at(i).coalLeaderID == infoMsg.senderRobotID)
            {
                coalList[i].status = CS_HANDLING;
                coalList[i].currentTaskUUID = QString::fromStdString(infoMsg.taskUUID);

                break;
            }
        }
    }
    else if  (infoMsg.infoTypeID == INFO_L2C_START_HANDLING)
    {

/*
        QString taskUUIDTmp = QString::fromStdString(infoMsg.taskUUID);

        for(int i = 0; i < waitingTasks.size();i++)
        {
            if (QString::compare(waitingTasks.at(i).taskUUID, taskUUIDTmp,Qt::CaseInsensitive) == 0)
            {
                waitingTasks.at(i).startHandlingTime =
                handlingTasks.append(waitingTasks.at(i));

                waitingTasks.remove(i);
            }
        }
/*
        data.append("&");

        data.append(QString::fromStdString(taskInfoMsg.taskUUID));
        */
    }
    else if  (infoMsg.infoTypeID == INFO_L2C_TASK_COMPLETED)
    {
        QString taskUUIDTmp = QString::fromStdString(infoMsg.taskUUID);

        for(int i = 0; i < handlingTasks.size();i++)
        {
            if (QString::compare(handlingTasks.at(i).taskUUID, taskUUIDTmp,Qt::CaseInsensitive) == 0)
            {

                for(int cid = 0; cid < coalList.size();cid++)
                {
                    if (QString::compare(coalList.at(cid).currentTaskUUID, taskUUIDTmp,Qt::CaseInsensitive)==0)
                    {
                        coalList[cid].status = CS_IDLE;

                        break;
                    }
                }

                completedTasks.append(handlingTasks.at(i));

                handlingTasks.remove(i);

                break;
            }
        }
    }
    else if ( (infoMsg.infoTypeID == INFO_L2C_SPLITTING) || (infoMsg.infoTypeID == INFO_L2C_SPLITTING_AND_LEADER_CHANGED) )
    {

        QString splittingRobotIDStr = QString::fromStdString(infoMsg.extraMsg);

        QStringList splittingRobotIDList = splittingRobotIDStr.split(",",QString::SkipEmptyParts);

        int newCoalLeaderID  = 0;
        if (infoMsg.infoTypeID == INFO_L2C_SPLITTING_AND_LEADER_CHANGED)
        {
            // new coalition leader ID
            newCoalLeaderID = splittingRobotIDList.at(splittingRobotIDList.size()-1).toUInt();
            splittingRobotIDList.removeLast();
        }

        qDebug()<<"Number of splitting robots"<<splittingRobotIDList.size();
        qDebug()<<splittingRobotIDList;

        for(int cid = 0; cid < coalList.size();cid++)
        {
            for(int rid = 0; rid < coalList.at(cid).coalMembers.size();rid++)
            {
                for(int srid = 0; srid < splittingRobotIDList.size();srid++)
                {
                    if (splittingRobotIDList.at(srid).toUInt() == coalList.at(cid).coalMembers.at(rid).robotID)
                    {                       
                        robotProp splittedRobot = coalList.at(cid).coalMembers.at(rid);
                        coalProp singletonCoal;
                        //singletonCoal.coalID
                        singletonCoal.coalLeaderID = splittedRobot.robotID;
                        singletonCoal.coalMembers.append(splittedRobot);
                        singletonCoal.status = CS_WAITING_GOAL_POSE;

                        coalList.append(singletonCoal);

                        coalList[cid].coalMembers.remove(rid);                       

                        splittingRobotIDList.removeAt(srid);

                        if (infoMsg.infoTypeID == INFO_L2C_SPLITTING_AND_LEADER_CHANGED)
                        {
                            //Assign new coalition leader ID
                            coalList[cid].coalLeaderID = newCoalLeaderID;
                        }
                    }
                }
            }
        }

    }

}

// set startChecking flag to true
void RosThread::taskCheckingTimerCallback(const ros::TimerEvent&)
{
    startChecking = true;
}


QVector <QVector <QVector <uint> > > RosThread::generatePartitions(QVector <uint> robotList)
{
    QVector <QVector <QVector <uint> > > partitions;

    QVector <QVector <uint> > firstPart;

    QVector <QVector<uint> > TMP;

    bool Cflg = false;  // A set not passed in.

    int N = robotList.size();

    qDebug()<< "num of robots: " << N;

    Cflg = true;

    int K = 0;  // Since user doesn't want only certain partitions.

    //Bell number
    //S = ceil(sum((1:2*N).^N./cumprod(1:2*N))/exp(1));
    double Stemp = 0;
    long cumprod = 1;
    for(int i = 1; i <= 2*N;i++)
    {
        Stemp = Stemp + pow(i,N)/cumprod;
        cumprod = cumprod*(i+1);
    }
    int S = ceil(Stemp/exp(1.0));


    firstPart.append(robotList);
    partitions.append(firstPart);

    int cnt = 2; // Start the while loop counter.

    QVector <uint> NV = QVector <uint> (robotList);

    int NU = 1; // Number of unique indices in current partition.
    QVector <int> stp1(2*N+1,0); // Controls assigning of indices.
    stp1[1] = N;
    QVector <uint> RGF (N+1,1); // Holds the indexes.


    while (cnt<=S)
    {
        int idx1 = N; // Index into RGF.
        int stp2 = RGF.at(idx1); // Works with stp1.

        while (stp1.at(stp2)==1)
        {
            RGF[idx1] = 1;  // Assign value to RGF.
            idx1 = idx1 - 1; // Need to increment idx1 for translation below.
            stp2 = RGF.at(idx1); // And set this guy for stp1 assign below.
        }

        NU = NU + idx1 - N;  // Get provisional number of unique vals.
        stp1[1] = stp1.at(1) + N - idx1;

        if (stp2==NU) // Increment the number of unique elements.
        {
            NU = NU +1;
            stp1[NU] = 0;
        }

        RGF[idx1] = stp2 + 1;  // Increment this position.
        stp1[stp2] = stp1.at(stp2) - 1;  // Translate indices of these two.
        stp1[stp2+1] = stp1.at(stp2+1) + 1; // Re-assign stopper.


        TMP.clear();

        for(int i = 1; i <= NU;i++)
        {
            QVector <uint> tempL;
            for(int j = 1; j <= RGF.size();j++)
            {
                if (RGF.at(j)==i)
                {
                    tempL.append(NV.at(j));
                }
            }

            TMP.append(tempL);
        }

        partitions.append(TMP);

        cnt = cnt + 1;

    }

    return partitions;
}
