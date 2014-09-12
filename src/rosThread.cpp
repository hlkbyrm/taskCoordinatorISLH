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



    QString path = QDir::homePath();
    path.append("/ISL_workspace/src/configISL.json");


    if(!readConfigFile(path)){

        qDebug()<< "Read Config File Failed!!!";

        ros::shutdown();

        emit rosFinished();
    }

    ct = n.createTimer(ros::Duration(taskCheckingPeriod), &RosThread::taskCheckingTimerCallback, this);

    ct.stop();


    messageTaskInfoFromLeaderSub = n.subscribe("messageDecoder/taskInfoFromLeader",5,&RosThread::handleTaskInfoFromLeader, this);

    messagePoseListSub = n.subscribe("localizationISLH/poseList",5,&RosThread::handlePoseList, this);

    cmd2LeadersPub = n.advertise<ISLH_msgs::cmd2LeadersMessage>("taskCoordinatorISLH/cmd2Leaders",5);

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

    if (waitingTasks.size()>0)
    {

        // check whether all the resources of the available coalitions are sufficient at least for one of the waiting tasks

        QVector <int> availCoalIDList;
        int numOfRes = waitingTasks.at(0).requiredResources.size();
        QVector <double> availTotRes;
        availTotRes.resize(numOfRes);

        for(int coalIndx = 0; coalIndx < coalList.size();coalIndx++)
        {
            if ( (coalList.at(coalIndx).status != CS_SUCCORING) && (coalList.at(coalIndx).status != CS_HANDLING) )
            {
                availCoalIDList.append(coalIndx);

                for(int robIndx = 0; robIndx < coalList.at(coalIndx).coalMembers.size();robIndx++)
                {
                    for(int resIndx = 0; resIndx < numOfRes; resIndx++)
                    {
                        availTotRes[resIndx] = availTotRes[resIndx] + coalList.at(coalIndx).coalMembers.at(robIndx).resources.at(resIndx);
                    }
                }
            }
        }

        QVector <int> wTaskIDList;

        for(int wTID = 0; wTID < waitingTasks.size();wTID++)
        {
            int resOK = 0;
            for(int resID = 0; resID < numOfRes; resID++)
            {
                if (waitingTasks.at(wTID).requiredResources.at(resID) <= availTotRes.at(resID))
                {
                    resOK = resOK + 1;
                }
            }

            if (resOK == numOfRes)
            {
                wTaskIDList.append(wTID);
            }
        }

        if (wTaskIDList.size()>0)
        {
            // run the coalition formation game
            runCFG(wTaskIDList, availCoalIDList);
        }

    }
}


void RosThread::runCFG(QVector <int> wTaskIDList, QVector <int> availCoalIDList)
{
    int changeOK = 1;

    while(changeOK==1)
    {
        // ......................................//
        // apply merge operation until no change //
        // ......................................//

        int changeOKM= 1;
        int changeMerge = 0;

        while(changeOKM==1)
        {
            int changeNum = 0;

            for(int wTIndx = 0; wTIndx < wTaskIDList.size(); wTIndx++)
            {
                int wTaskID = wTaskIDList.at(wTIndx);
                int coalID = -1;

                if (waitingTasks.at(wTaskID).status == TS_NOT_ASSIGNED)
                    // there is no coalition associated with this task
                {
                    int maxCoalID = -1;
                    double maxCoalVal = 0.0;
                    int maxCoalIDHasTask = 0;
                    int maxTaskID = -1;

                    for(int coalIndx = 0; coalIndx < availCoalIDList.size(); coalIndx++)
                    {
                        int coalIDTmp = availCoalIDList.at(coalIndx);

                        double coalValTmp = calcCoalValue(coalList.at(coalIDTmp).coalMembers, waitingTasks.at(wTaskID));

                        // check whether the assignment of coalID which is also
                        // responsible for another should increase social welfare

                        int taskIDTmp = -1;
                        double coalValTmpPrev = 0.0;

                        for(int wTIndx2 = 0; wTIndx2 < wTaskIDList.size(); wTIndx2++)
                        {
                            int wTaskID2 = wTaskIDList.at(wTIndx2);

                            if (waitingTasks.at(wTaskID2).status == TS_WAITING)
                            {
                                QString taskUUIDTmp = waitingTasks.at(wTaskID2).taskUUID;

                                for(int coalIndx2 = 0; coalIndx2 < availCoalIDList.size(); coalIndx2++)
                                {
                                    int coalIDTmp2 = availCoalIDList.at(coalIndx2);
                                    if (QString::compare(coalList.at(coalIDTmp2).currentTaskUUID, taskUUIDTmp,Qt::CaseInsensitive) == 0)
                                    {
                                        coalValTmpPrev = calcCoalValue(coalList.at(coalIDTmp).coalMembers, waitingTasks.at(wTaskID2));

                                        taskIDTmp = wTaskID2;

                                        break;
                                    }
                                }
                            }
                        }

                        if ( (maxCoalID == -1) && (coalValTmp > coalValTmpPrev) )
                        {
                            maxCoalID = coalIDTmp;
                            maxCoalVal = coalValTmp;

                            if (taskIDTmp > -1)
                            {
                                maxCoalIDHasTask = 1;
                                maxTaskID = taskIDTmp;
                            }
                        }
                        else if ( (maxCoalID > -1) && (coalValTmp > coalValTmpPrev) && (coalValTmp > maxCoalVal) )
                        {
                            maxCoalID = coalIDTmp;
                            maxCoalVal = coalValTmp;
                            if (taskIDTmp > -1)
                            {
                                maxCoalIDHasTask = 1;
                                maxTaskID = taskIDTmp;
                            }
                        }

                    }

                    if (maxCoalID > -1)
                    {
                        coalID = maxCoalID; // the coalition is responsible for task taskID
                        coalList[coalID].status = CS_WAITING_TASK_RESPONSE_FROM_COORDINATOR;
                        waitingTasks[wTaskID].responsibleUnit = coalList.at(coalID).coalLeaderID;

                        if (maxCoalIDHasTask > 0)
                        {
                            waitingTasks[wTaskID].responsibleUnit = 0;
                        }
                    }

                }
                else
                {
                    for(int coalIndx = 0; coalIndx < coalList.size();coalIndx++)
                    {
                        for(int robIndx = 0; robIndx < coalList.at(coalIndx).coalMembers.size();robIndx++)
                        {
                            if ( coalList.at(coalIndx).coalMembers.at(robIndx).robotID == waitingTasks.at(wTaskID).responsibleUnit)
                            {
                                coalID = coalIndx;
                                break;
                            }

                        }
                    }

                }


                if (coalID > -1)
                {
                    //find an available coalition for merging with coalition coalID
                    //such that merging increases the value of both coalitions
                    int mCoalID = -1;

                    mCoalID = findCoalition(coalID, wTaskID, wTaskIDList);

                    if (mCoalID > -1)
                    {
                        //merge these two coalitions
                        mergeCoalitions(coalID, mCoalID);

                        changeNum = changeNum + 1;

                        changeMerge = 1;
                    }
                }

            }

            if (changeNum == 0)
            {
                changeOKM = 0;
            }
        } // end of while(changeOKM==1)


        // ......................................//
        // apply split operation until no change //
        // ......................................//

        int changeOKS= 1;
        int changeSplit = 0;

        while(changeOKS==1)
        {
            int changeNum = 0;

            for(int wTIndx = 0; wTIndx < wTaskIDList.size(); wTIndx++)
            {
                int wTaskID = wTaskIDList.at(wTIndx);

                if (waitingTasks.at(wTaskID).status != TS_NOT_ASSIGNED)
                {
                    int coalID = -1;
                    for(int coalIndx = 0; coalIndx < coalList.size();coalIndx++)
                    {
                        for(int robIndx = 0; robIndx < coalList.at(coalIndx).coalMembers.size();robIndx++)
                        {
                            if ( coalList.at(coalIndx).coalMembers.at(robIndx).robotID == waitingTasks.at(wTaskID).responsibleUnit)
                            {
                                coalID = coalIndx;
                                break;
                            }

                        }
                    }

                    int splitRobotID = findSplittedRobot(coalID, wTaskID);

                    if (splitRobotID > -1)
                    {
                        // split splitRobotID from the coalition
                        splitCoalition(splitRobotID, coalID);

                        if (splitRobotID==waitingTasks.at(wTaskID).responsibleUnit)
                            // this meand that the splitted robot is coalition leader
                        {
                            /*
                            robotIDs = find(coalIDList==coalID);
                            taskList(taskID, 2) = robotIDs(1);
                            */

                            int minRobotID = 9999999;
                            for(int robIndx = 0; robIndx < coalList.at(coalID).coalMembers.size();robIndx++)
                            {
                                int robotIDTmp = coalList.at(coalID).coalMembers.at(robIndx).robotID;
                                if ( robotIDTmp < minRobotID)
                                {
                                    minRobotID = robotIDTmp;
                                }
                            }

                            coalList[coalID].coalLeaderID = minRobotID;

                            waitingTasks[wTaskID].responsibleUnit = minRobotID;
                        }

                        changeNum = changeNum + 1;

                        changeSplit = 1;
                    }

                }
            }

            if (changeNum==0)
            {
                changeOKS = 0;
            }

        } // end of while(changeOKS==1)

        if ( (changeMerge+changeSplit) == 0 )
        {
            changeOK = 0;
        }

    } // end of while(changeOK==1)

}

// find an available coalition for merging with coalID
int RosThread::findCoalition(int coalID, int wTaskID, QVector <int> wTaskIDList)
{
    double coalVal = calcCoalValue(coalList.at(coalID).coalMembers, waitingTasks.at(wTaskID));

    QVector <int> availCoalIDList;

    for(int coalIndx = 0; coalIndx < coalList.size();coalIndx++)
    {
        if ( (coalList.at(coalIndx).status != CS_SUCCORING) && (coalList.at(coalIndx).status != CS_HANDLING) )
        {
            availCoalIDList.append(coalIndx);
        }
    }


    int mCoalID = -1;

    for (int coalIndx = 0; coalIndx < availCoalIDList.size();coalIndx++)
    {
        int coalIDTmp = availCoalIDList.at(coalIndx);

        if (coalIDTmp != coalID)
        {
            //check whether coalIDTemp is responsible for any task
            int taskIDTmp = -1;
            for(int wTIndx = 0; wTIndx < wTaskIDList.size(); wTIndx++)
            {
                int wTaskIDTmp = wTaskIDList.at(wTIndx);

                    if (waitingTasks.at(wTaskIDTmp).responsibleUnit>0)
                    {
                        for (int coalIndx2 = 0; coalIndx2 < coalList.size();coalIndx2++)
                        {
                            for(int robIndx = 0; robIndx < coalList.at(coalIndx2).coalMembers.size();robIndx++)
                            {
                                if (coalList.at(coalIndx2).coalMembers.at(robIndx).robotID == waitingTasks.at(wTaskIDTmp).responsibleUnit)
                                {
                                    if (coalIndx2 == coalIDTmp)
                                    {
                                        taskIDTmp = wTaskIDTmp;
                                        break;
                                    }
                                }
                            }

                        }
                    }
            }

            double coalValTmp = 0;

            if (taskIDTmp>-1)
            {
                coalValTmp = calcCoalValue(coalList.at(coalIDTmp).coalMembers, waitingTasks.at(taskIDTmp));
            }

            QVector <robotProp> mergedCoalRobotsTmp = coalList.at(coalID).coalMembers;
            for(int robIndx = 0; robIndx < coalList.at(coalIDTmp).coalMembers.size();robIndx++)
            {
                mergedCoalRobotsTmp.append(coalList.at(coalIDTmp).coalMembers.at(robIndx));
            }

            double mergedCoalValTmp = calcCoalValue(mergedCoalRobotsTmp, waitingTasks.at(wTaskID));

            if ( (mergedCoalValTmp>(coalVal+coalValTmp)) && (mergedCoalValTmp>=coalValTmp) &&  (mergedCoalValTmp>=coalVal) )
            {
                coalVal = mergedCoalValTmp;
                mCoalID = coalIDTmp;
            }

        }
    }

    return mCoalID;

}

// Merge the given two coalitions
void RosThread::mergeCoalitions(int coalID, int mCoalID)
{
    int numOfRes = waitingTasks.at(0).requiredResources.size();
    QVector <double> coalTotResourcesTmp;
    for(int resID = 0; resID < numOfRes; resID++)
    {
        double coalIDRes = coalList.at(coalID).coalTotalResources.at(resID);
        double mCoalIDRes = coalList.at(mCoalID).coalTotalResources.at(resID);
        coalTotResourcesTmp.append( coalIDRes + mCoalIDRes);
    }
    coalList[coalID].coalTotalResources = coalTotResourcesTmp;


    // add robots in mCoalID to coalID
    for(int robIndx = 0; robIndx < coalList.at(mCoalID).coalMembers.size();robIndx++)
    {
        coalList[coalID].coalMembers.append( coalList.at(mCoalID).coalMembers.at(robIndx) );
    }

    //merged coalition may be responsible for any task.
    // hence waitingTasks should be changed accordingly.

    if (QString::compare(coalList.at(mCoalID).currentTaskUUID, "NONE", Qt::CaseInsensitive) != 0)
    {
        for(int wTID = 0; wTID < waitingTasks.size();wTID++)
        {
            if (QString::compare(waitingTasks.at(wTID).taskUUID, coalList.at(mCoalID).currentTaskUUID,Qt::CaseInsensitive) == 0)
            {
                waitingTasks[wTID].responsibleUnit = 0;
                break;
            }
        }
    }

    coalList[coalID].status = CS_WAITING_TASK_RESPONSE_FROM_COORDINATOR;

    coalList.remove(mCoalID);

}

//  find a robot to be splitted from the coalition
int RosThread::findSplittedRobot(int coalID, int taskID)
{
    int splitRobotID = -1;

    if ( coalList.at(coalID).coalMembers.size() > 1)
    {
        double coalVal =  calcCoalValue(coalList.at(coalID).coalMembers, waitingTasks.at(taskID));

        // only one robot is removed from the coalition
        for(int robIndx = 0; robIndx < coalList.at(coalID).coalMembers.size();robIndx++)
        {

            QVector <robotProp> coalRobotIDsTmp(coalList.at(coalID).coalMembers);

            coalRobotIDsTmp.remove(robIndx);

            double coalValTmp = calcCoalValue(coalRobotIDsTmp, waitingTasks.at(taskID));

            if (coalValTmp >= coalVal)
            {
                coalVal = coalValTmp;
                splitRobotID = coalList.at(coalID).coalMembers.at(robIndx).robotID;
            }
        }

    }

    return splitRobotID;

}

// split the given robot from the given coalition
void RosThread::splitCoalition(int splitRobotID, int coalID)
{

   coalProp newCoal;
   newCoal.coalLeaderID = splitRobotID;

   QVector <robotProp> newCoalMembers;
   newCoalMembers.append(coalList.at(coalID).coalMembers.at(splitRobotID));

   newCoalMembers[0].inGoalPose = false;
   newCoalMembers[0].inTaskSite = false;

   newCoal.coalTotalResources =  newCoalMembers.at(0).resources;

   newCoal.coalMembers = newCoalMembers;

   newCoal.currentTaskUUID = "NONE";

   newCoal.status = CS_WAITING_GOAL_POSE;


   coalList.append(newCoal);
}

// calculate the value of a given coalition coalMmbrs for a given task
double RosThread::calcCoalValue(QVector <robotProp> coalMmbrs, taskProp wTask)
{

    double coalValue = 0.0;

    int numOfMem = coalMmbrs.size();

    int numOfResource = coalMmbrs.at(0).resources.size();

    double resDiff = 0;
    double excess = 0;

    for(int resID=0; resID<numOfResource; resID++)
    {
        double coalRes = 0.0;
        for(int robID=0; robID<numOfMem; robID++)
        {
            coalRes = coalRes + coalMmbrs.at(robID).resources.at(resID);
        }

        double taskRes =wTask.requiredResources.at(resID);

        if ((taskRes-coalRes)>0)
        {
            resDiff = resDiff  + (taskRes-coalRes)*(taskRes-coalRes);
        }

        excess = excess + (1-coalRes/taskRes)*(1-coalRes/taskRes);
    }


    double taskPoseX = wTask.pose.X;
    double taskPoseY = wTask.pose.Y;
    double dist = 0;
    for(int i=0;i<numOfMem; i++)
    {
        double robPoseX = coalMmbrs.at(i).pose.X;
        double robPoseY = coalMmbrs.at(i).pose.Y;

        dist = dist + sqrt((robPoseX-taskPoseX)*(robPoseX-taskPoseX) + (robPoseY-taskPoseY)*(robPoseY-taskPoseY))/(2*cvfParams.ro);
    }

    coalValue = 1.0/(1 + cvfParams.w1*resDiff +  cvfParams.w2*excess + cvfParams.w3*dist);

    return coalValue;
}

// handle incoming robot poses
void RosThread::handlePoseList(ISLH_msgs::robotPositions robotPoseListMsg)
{

    //
    // ???????????????????????????????
    //
}

// handle incoming task info from a coalition leader
void RosThread::handleTaskInfoFromLeader(ISLH_msgs::taskInfoFromLeaderMessage infoMsg)
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
                        singletonCoal.coalTotalResources = singletonCoal.coalMembers.at(0).resources;

                        coalList.append(singletonCoal);

                        coalList[cid].coalMembers.remove(rid);                       

                        splittingRobotIDList.removeAt(srid);

                        if (infoMsg.infoTypeID == INFO_L2C_SPLITTING_AND_LEADER_CHANGED)
                        {
                            //Assign a new coalition leader ID
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


// Reads the config file
bool RosThread::readConfigFile(QString filename)
{
    QFile file(filename);

    if(!file.exists()) return false;

    if(!file.open(QFile::ReadOnly)) return false;

    QJson::Parser parser;

    bool ok;

    QVariantMap result = parser.parse(&file,&ok).toMap();

    if(!ok){

        file.close();
        qDebug()<<"Fatal reading error";

        return false;
    }
    else
    {
        cvfParams.w1 = result["cvf-w1"].toDouble();
        qDebug()<< " w1 " << cvfParams.w1;


        cvfParams.w2 = result["cvf-w2"].toDouble();
        qDebug()<< " w2 " << cvfParams.w2;

        cvfParams.w3 = result["cvf-w3"].toDouble();
        qDebug()<< " w3 " << cvfParams.w3;

        cvfParams.ro = result["ro"].toDouble();
        qDebug()<< " ro " << cvfParams.ro;
    }
    file.close();
    return true;

}
