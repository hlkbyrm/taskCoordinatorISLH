#include "rosThread.h"
#include <QDebug>
#include <qjson/parser.h>
#include <QDir>
#include <QFile>



RosThread::RosThread()
{
    shutdown = false;

    missionParams.startMission = false;

    missionParams.taskCheckingPeriod = 10; //in seconds

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

    ct = n.createTimer(ros::Duration(missionParams.taskCheckingPeriod), &RosThread::taskCheckingTimerCallback, this);

    ct.stop();


    messageTaskInfoFromLeaderSub = n.subscribe("messageDecoder/taskInfoFromLeader",5,&RosThread::handleTaskInfoFromLeader, this);

    messagePoseListSub = n.subscribe("localizationISLH/poseList",5,&RosThread::handlePoseList, this);

    messageStartMissionSub = n.subscribe("monitoringISLH/startMission", 5,&RosThread::handleStartMission, this);

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

/*
    taskProp newTask;
    newTask.status = TS_NOT_ASSIGNED;
    newTask.encounteringRobotID = 1;
    newTask.responsibleUnit = -1;
    newTask.taskUUID = "a123";
    newTask.pose.X = 10;
    newTask.pose.Y = 20;
    newTask.encounteringTime = 10;
    newTask.requiredResourcesString = "1,2,0,3,1";
    QStringList resourceParts = newTask.requiredResourcesString.split(",",QString::SkipEmptyParts);
    newTask.requiredResources.clear();
    for(int i = 0; i < resourceParts.size();i++)
        newTask.requiredResources.append(resourceParts.at(i).toDouble());
    waitingTasks.append(newTask);



    newTask.encounteringRobotID = 3;
    newTask.taskUUID = "a124";
    newTask.pose.X = 40;
    newTask.pose.Y = 60;
    newTask.encounteringTime = 13;
    newTask.requiredResourcesString = "3,2,1,0,2";
    resourceParts = newTask.requiredResourcesString.split(",",QString::SkipEmptyParts);
    newTask.requiredResources.clear();
    for(int i = 0; i < resourceParts.size();i++)
        newTask.requiredResources.append(resourceParts.at(i).toDouble());
    waitingTasks.append(newTask);



    for(int robID = 0; robID < missionParams.numOfRobots;robID++)
    {
        robotProp newRobot;
        newRobot.coalID = robID;
        newRobot.inGoalPose = 0;
        newRobot.inTaskSite = -1;
        newRobot.robotID = robID + 1;
        newRobot.resources << 1 << 1 << 1 << 1 << 1;
        newRobot.pose.X = 15+robID*5;
        newRobot.pose.Y = 15-robID*3;

        coalProp newCoal;

        newCoal.coalLeaderID = robID+1;
        newCoal.coalMembers.append(newRobot);
        newCoal.status = CS_WAITING_GOAL_POSE;
        newCoal.coalTotalResources = newRobot.resources;
        newCoal.currentTaskUUID = "NONE";

        coalList.append(newCoal);
    }
*/

    ros::Rate loop(10);

    while(ros::ok())
    {

        if (coordinatorRobotID == ownRobotID)
        {
            if (missionParams.startMission)
            {
                if (startChecking)
                {
                    manageCoalitions();

                    startChecking = false;
                }
            }
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

void RosThread::handleStartMission(std_msgs::UInt8 msg)
{
    if (msg.data == 1)
    {
        missionParams.startMission = true;
        ct.start();
    }
    else
    {
        missionParams.startMission = false;
        ct.stop();
    }

    QVector <int> coalIDList;
    for(int coalID=0; coalID < coalList.size(); coalID++)
        coalIDList.append(coalID);

    sendCmd2Leaders(CMD_C2L_START_OR_STOP_MISSION, coalIDList);
}

void RosThread::manageCoalitions()
{

    if (waitingTasks.size()>0)
    {
        // check whether all the resources of the available coalitions are sufficient at least for one of the waiting tasks

        //QVector <int> availCoalIDList;
        int numOfRes = waitingTasks.at(0).requiredResources.size();
        QVector <double> availTotRes;
        availTotRes.resize(numOfRes);

        for(int coalIndx = 0; coalIndx < coalList.size();coalIndx++)
        {
            if ( (coalList.at(coalIndx).status != CS_SUCCORING) && (coalList.at(coalIndx).status != CS_HANDLING) )
            {
                //availCoalIDList.append(coalIndx);

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
            _coalList = QVector <coalProp>(coalList);
            _waitingTasks = QVector <taskProp>(waitingTasks);

            // .................................//
            // run the coalition formation game //
            // .................................//
            runCFG(wTaskIDList);//, availCoalIDList);

            // check whether the merged coalitions have sufficient resources for their associated tasks
            // in turn, send the corresponding messages to the merged coalitions' members
            for(int wTIndx = 0; wTIndx < wTaskIDList.size(); wTIndx++)
            {
                int wTaskID = wTaskIDList.at(wTIndx);

                for(int coalIndx = 0; coalIndx < _coalList.size(); coalIndx++)
                {
                    if ( _waitingTasks.at(wTaskID).taskUUID == _coalList.at(coalIndx).currentTaskUUID)
                    {
                        int resourceOK = 0;

                        for(int resID = 0; resID < numOfRes; resID++)
                        {
                            if (_coalList.at(coalIndx).coalTotalResources.at(resID) >= _waitingTasks.at(wTaskID).requiredResources.at(resID))
                            {
                                resourceOK = resourceOK + 1;
                            }
                        }

                        if (resourceOK == numOfRes)
                        {
                            // ????
                        }
                    }
                }
            }



            // check whether there are splitted coalitions
            // in turn, send the corresponding messages to the splitted robots

            // ???


            // clear temporary variables
            _coalList.clear();
            _waitingTasks.clear();

        }
    }
}


void RosThread::runCFG(QVector <int> wTaskIDList)//, QVector <int> availCoalIDList)
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

                //if (_waitingTasks.at(wTaskID).status == TS_NOT_ASSIGNED)
                if (_waitingTasks.at(wTaskID).responsibleUnit<0)
                    // there is no coalition associated with this task
                {
                    int maxCoalID = -1;
                    double maxCoalVal = 0.0;
                    int maxCoalIDHasTask = 0;
                    int maxTaskID = -1;

                    QVector <int> availCoalIDList;
                    for(int coalIndx = 0; coalIndx < _coalList.size();coalIndx++)
                    {
                        if ( (_coalList.at(coalIndx).status != CS_SUCCORING) && (_coalList.at(coalIndx).status != CS_HANDLING) )
                        {
                            availCoalIDList.append(coalIndx);
                        }
                    }

                    for(int coalIndx = 0; coalIndx < availCoalIDList.size(); coalIndx++)
                    {
                        int coalIDTmp = availCoalIDList.at(coalIndx);

                        double coalValTmp = calcCoalValue(_coalList.at(coalIDTmp).coalMembers, _waitingTasks.at(wTaskID));

                        // check whether the assignment of coalID which is also
                        // responsible for another should increase social welfare

                        int taskIDTmp = -1;
                        double coalValTmpPrev = 0.0;

                        for(int wTIndx2 = 0; wTIndx2 < wTaskIDList.size(); wTIndx2++)
                        {
                            int wTaskID2 = wTaskIDList.at(wTIndx2);

                            if (_waitingTasks.at(wTaskID2).status == TS_WAITING)
                            {
                                QString taskUUIDTmp = _waitingTasks.at(wTaskID2).taskUUID;

                                for(int coalIndx2 = 0; coalIndx2 < availCoalIDList.size(); coalIndx2++)
                                {
                                    int coalIDTmp2 = availCoalIDList.at(coalIndx2);
                                    if (QString::compare(_coalList.at(coalIDTmp2).currentTaskUUID, taskUUIDTmp,Qt::CaseInsensitive) == 0)
                                    {
                                        coalValTmpPrev = calcCoalValue(_coalList.at(coalIDTmp).coalMembers, _waitingTasks.at(wTaskID2));

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
                        _coalList[coalID].status = CS_WAITING_TASK_RESPONSE_FROM_COORDINATOR;
                        _coalList[coalID].currentTaskUUID = _waitingTasks.at(wTaskID).taskUUID;
                        _waitingTasks[wTaskID].responsibleUnit = _coalList.at(coalID).coalLeaderID;                        
                        _waitingTasks[wTaskID].status = TS_WAITING;

                        if (maxCoalIDHasTask > 0)
                        {
                            _waitingTasks[maxTaskID].responsibleUnit = -1;
                        }
                    }

                }
                else
                // the task is associated with a coalition
                {
                    for(int coalIndx = 0; coalIndx < _coalList.size();coalIndx++)
                    {
                        for(int robIndx = 0; robIndx < _coalList.at(coalIndx).coalMembers.size();robIndx++)
                        {
                            if ( _coalList.at(coalIndx).coalMembers.at(robIndx).robotID == _waitingTasks.at(wTaskID).responsibleUnit)
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

                if (_waitingTasks.at(wTaskID).status != TS_NOT_ASSIGNED)
                {
                    int coalID = -1;
                    for(int coalIndx = 0; coalIndx < _coalList.size();coalIndx++)
                    {
                        for(int robIndx = 0; robIndx < _coalList.at(coalIndx).coalMembers.size();robIndx++)
                        {
                            if ( _coalList.at(coalIndx).coalMembers.at(robIndx).robotID == _waitingTasks.at(wTaskID).responsibleUnit)
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

                        if (splitRobotID==_waitingTasks.at(wTaskID).responsibleUnit)
                            // this meand that the splitted robot is coalition leader
                        {
                            int minRobotID = 9999999;
                            for(int robIndx = 0; robIndx < _coalList.at(coalID).coalMembers.size();robIndx++)
                            {
                                int robotIDTmp = _coalList.at(coalID).coalMembers.at(robIndx).robotID;
                                if ( robotIDTmp < minRobotID)
                                {
                                    minRobotID = robotIDTmp;
                                }
                            }

                            _coalList[coalID].coalLeaderID = minRobotID;

                            _waitingTasks[wTaskID].responsibleUnit = minRobotID;
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
    double coalVal = calcCoalValue(_coalList.at(coalID).coalMembers, _waitingTasks.at(wTaskID));

    QVector <int> availCoalIDList;

    for(int coalIndx = 0; coalIndx <_coalList.size();coalIndx++)
    {
        if ( (_coalList.at(coalIndx).status != CS_SUCCORING) && (_coalList.at(coalIndx).status != CS_HANDLING) )
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

                if (_waitingTasks.at(wTaskIDTmp).responsibleUnit>0)
                {
                    for (int coalIndx2 = 0; coalIndx2 < _coalList.size();coalIndx2++)
                    {
                        for(int robIndx = 0; robIndx < _coalList.at(coalIndx2).coalMembers.size();robIndx++)
                        {
                            if (_coalList.at(coalIndx2).coalMembers.at(robIndx).robotID == _waitingTasks.at(wTaskIDTmp).responsibleUnit)
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
                coalValTmp = calcCoalValue(_coalList.at(coalIDTmp).coalMembers, _waitingTasks.at(taskIDTmp));
            }

            QVector <robotProp> mergedCoalRobotsTmp = _coalList.at(coalID).coalMembers;
            for(int robIndx = 0; robIndx < _coalList.at(coalIDTmp).coalMembers.size();robIndx++)
            {
                mergedCoalRobotsTmp.append(_coalList.at(coalIDTmp).coalMembers.at(robIndx));
            }

            double mergedCoalValTmp = calcCoalValue(mergedCoalRobotsTmp, _waitingTasks.at(wTaskID));

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
    int numOfRes = _waitingTasks.at(0).requiredResources.size();
    QVector <double> coalTotResourcesTmp;
    for(int resID = 0; resID < numOfRes; resID++)
    {
        double coalIDRes = _coalList.at(coalID).coalTotalResources.at(resID);
        double mCoalIDRes = _coalList.at(mCoalID).coalTotalResources.at(resID);
        coalTotResourcesTmp.append( coalIDRes + mCoalIDRes);
    }
    _coalList[coalID].coalTotalResources = coalTotResourcesTmp;


    // add robots in mCoalID to coalID
    for(int robIndx = 0; robIndx < _coalList.at(mCoalID).coalMembers.size();robIndx++)
    {
        _coalList[mCoalID].coalMembers[robIndx].coalID = coalID;
        _coalList[coalID].coalMembers.append( _coalList.at(mCoalID).coalMembers.at(robIndx) );        
    }

    //merged coalition may be responsible for any task.
    // hence waitingTasks should be changed accordingly.

    if (QString::compare(_coalList.at(mCoalID).currentTaskUUID, "NONE", Qt::CaseInsensitive) != 0)
    {
        for(int wTID = 0; wTID < _waitingTasks.size();wTID++)
        {
            if (QString::compare(_waitingTasks.at(wTID).taskUUID, _coalList.at(mCoalID).currentTaskUUID,Qt::CaseInsensitive) == 0)
            {
                _waitingTasks[wTID].responsibleUnit = -1;
                break;
            }
        }
    }

    _coalList[coalID].status = CS_WAITING_TASK_RESPONSE_FROM_COORDINATOR;

    _coalList.remove(mCoalID);

}

//  find a robot to be splitted from the coalition
int RosThread::findSplittedRobot(int coalID, int taskID)
{
    int splitRobotID = -1;

    if ( _coalList.at(coalID).coalMembers.size() > 1)
    {
        double coalVal =  calcCoalValue(_coalList.at(coalID).coalMembers, _waitingTasks.at(taskID));

        // only one robot is removed from the coalition
        for(int robIndx = 0; robIndx < _coalList.at(coalID).coalMembers.size();robIndx++)
        {

            QVector <robotProp> coalRobotIDsTmp(_coalList.at(coalID).coalMembers);

            coalRobotIDsTmp.remove(robIndx);

            double coalValTmp = calcCoalValue(coalRobotIDsTmp, _waitingTasks.at(taskID));

            if (coalValTmp >= coalVal)
            {
                coalVal = coalValTmp;
                splitRobotID = _coalList.at(coalID).coalMembers.at(robIndx).robotID;
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
   newCoalMembers.append(_coalList.at(coalID).coalMembers.at(splitRobotID));

   newCoalMembers[0].inGoalPose = -1;
   newCoalMembers[0].inTaskSite = -1;

   newCoal.coalTotalResources =  newCoalMembers.at(0).resources;

   newCoal.coalMembers = newCoalMembers;

   newCoal.currentTaskUUID = "NONE";

   newCoal.status = CS_WAITING_GOAL_POSE;


   _coalList.append(newCoal);
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

        if (taskRes==0)
        {
            taskRes = 0.00001;
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

        dist = dist + sqrt((robPoseX-taskPoseX)*(robPoseX-taskPoseX) + (robPoseY-taskPoseY)*(robPoseY-taskPoseY))/(2*missionParams.ro);
    }

    coalValue = 1.0/(1 + cvfParams.w1*resDiff +  cvfParams.w2*excess + cvfParams.w3*dist);

    return coalValue;
}

// handle incoming robot poses
void RosThread::handlePoseList(ISLH_msgs::robotPositions robotPoseListMsg)
{

    if (robotPoseListMsg.positions.size() != missionParams.numOfRobots)
    {
        qDebug() << "incoming robot positions are not consistent with the number of robots";
    }

    for(int robID=0; robID<robotPoseListMsg.positions.size(); robID++)
    {
        robotsList[robID].pose.X = robotPoseListMsg.positions.at(robID).x;
        robotsList[robID].pose.Y = robotPoseListMsg.positions.at(robID).y;
    }
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

        int newCoalLeaderID  = -1;
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
    else if (infoMsg.infoTypeID == INFO_L2C_WAITING_GOAL_POSE)
    {
        if (infoMsg.extraMsg == "GOALPOSE")
        {
            int leaderRobotID = infoMsg.senderRobotID;
            qDebug()<< "goal pose request from robot " << leaderRobotID;

            int coalID = -1;

            for(int cid = 0; cid < coalList.size();cid++)
            {
                if (coalList.at(cid).coalLeaderID == leaderRobotID)
                {
                    coalID = cid;
                    break;
                }
            }

            if (coalID==-1)
                qDebug()<< "robot "<<leaderRobotID << " is not a coalition leader !!! ";

            generatePoses(coalID, GOAL_POSE);

            sendCmd2Leaders(CMD_C2L_NEW_GOAL_POSES, QVector <int>(coalID));

        }
        else
        {
             qDebug()<< "goal pose request: but extraMsg is not coherent";
        }
    }

}

// set startChecking flag to true
void RosThread::taskCheckingTimerCallback(const ros::TimerEvent&)
{
    startChecking = true;
}

void RosThread::generatePoses(int coalID, int poseType)
{
    if (poseType == GOAL_POSE)
    {
        int numOfMem = coalList.at(coalID).coalMembers.size();

        for(int robIndx=0; robIndx < numOfMem; robIndx++)
        {
            int robotID = coalList.at(coalID).coalMembers.at(robIndx).robotID;

            double robotRadius = coalList.at(coalID).coalMembers.at(robIndx).radius;

            int poseOK = 0;
            while(poseOK==0)
            {
                /* initialize random seed: */
                srand (time(NULL));

                /*
                double robX = -(rand()%(2*missionParams.ro)) + missionParams.ro;
                double robY = -(rand()%(2*missionParams.ro)) + missionParams.ro;
                */

                double robX = ((rand()/ (RAND_MAX + 1.0)) * (2*missionParams.ro)) - missionParams.ro;
                double robY = ((rand()/ (RAND_MAX + 1.0)) * (2*missionParams.ro)) - missionParams.ro;


                if (sqrt(robX*robX + robY*robY) <= (missionParams.ro-robotRadius))
                {
                    int distOK = 1;

                    // check dist btw xy & other robots' pose
                    for(int robIndx2=0; robIndx2 < robotsList.size(); robIndx2++)
                    {
                        if ( (robotsList.at(robIndx2).robotID !=robotID) && (robotsList.at(robIndx2).coalID != coalID) )
                        {
                            if (robotsList.at(robIndx2).inGoalPose != -1)
                                // robIndx2 has an assigned goal pose?
                            {
                                double xTmp = robotsList.at(robIndx2).goalPose.X;
                                double yTmp = robotsList.at(robIndx2).goalPose.Y;

                                double robotRadius2 = robotsList.at(robIndx2).radius;

                                if ( sqrt((robX-xTmp)*(robX-xTmp) + (robY-yTmp)*(robY-yTmp)) <= (robotRadius + robotRadius2) )
                                {
                                    distOK = 0;
                                    break;
                                }
                            }
                        }
                    }

                    // check dist btw xy & its coalition members' pose
                    if (distOK == 1)
                    {
                        for(int robIndx2=0; robIndx2 < robIndx; robIndx2++)
                        {
                            if (robotsList.at(robIndx2).inGoalPose != -1)
                                // robIndx2 has an assigned goal pose?
                            {
                                double xTmp = coalList.at(coalID).coalMembers.at(robIndx2).goalPose.X;
                                double yTmp = coalList.at(coalID).coalMembers.at(robIndx2).goalPose.Y;

                                double robotRadius2 = coalList.at(coalID).coalMembers.at(robIndx2).radius;

                                if ( sqrt((robX-xTmp)*(robX-xTmp) + (robY-yTmp)*(robY-yTmp)) <= (robotRadius + robotRadius2) )
                                {
                                    distOK = 0;
                                    break;
                                }
                            }
                        }
                    }


                    // check dist btw xy & tasks' site pose
                    if (distOK == 1)
                    {
                        for(int wTID = 0; wTID < waitingTasks.size();wTID++)
                        {
                            double xTmp = waitingTasks.at(wTID).pose.X;
                            double yTmp = waitingTasks.at(wTID).pose.Y;

                            if ( sqrt((robX-xTmp)*(robX-xTmp) + (robY-yTmp)*(robY-yTmp)) <= (robotRadius + missionParams.taskSiteRadius) )
                            {
                                distOK = 0;
                                break;
                            }
                        }

                        if (distOK == 1)
                        {
                            for(int hTID = 0; hTID < handlingTasks.size();hTID++)
                            {
                                double xTmp = handlingTasks.at(hTID).pose.X;
                                double yTmp = handlingTasks.at(hTID).pose.Y;

                                if ( sqrt((robX-xTmp)*(robX-xTmp) + (robY-yTmp)*(robY-yTmp)) <= (robotRadius + missionParams.taskSiteRadius) )
                                {
                                    distOK = 0;
                                    break;
                                }
                            }
                        }
                    }


                    if (distOK == 1)
                    {
                        robotsList[robotID-1].goalPose.X = robX;
                        robotsList[robotID-1].goalPose.Y = robY;

                        coalList[coalID].coalMembers[robIndx].goalPose.X = robX;
                        coalList[coalID].coalMembers[robIndx].goalPose.Y = robY;

                        poseOK = 1;
                    }
                }

            } // end of while(poseOK==0)
        }
    }
    else if (poseType == TASK_SITE_POSE)
    {
        // ???
        // ???
        // ???
    }

}


// prepare a command message to the coalition leader
void RosThread::sendCmd2Leaders(int cmdType, QVector <int> coalIDList)
{
    ISLH_msgs::cmd2LeadersMessage msg;

    std::time_t sendingTime = std::time(0);
    msg.sendingTime = sendingTime;


    QString targetPosesStr;

    for(int cIndx=0; cIndx < coalIDList.size(); cIndx++)
    {
        int coalID = coalIDList.at(cIndx);

        msg.leaderRobotID.push_back(coalList.at(coalID).coalLeaderID);

        msg.messageTypeID.push_back(cmdType);

        QString msgStr;
        if (cmdType == CMD_C2L_NEW_GOAL_POSES)
        {
            // msgStr = robotID1,posex1,posey1;robotID2,posex2,posey2;...;robotIDn,posexn,poseyn

            int numOfMem = coalList.at(coalID).coalMembers.size();
            for(int robIndx=0; robIndx < numOfMem; robIndx++)
            {
                msgStr.append(QString::number(coalList.at(coalID).coalMembers.at(robIndx).robotID));
                msgStr.append(",");
                msgStr.append(QString::number(coalList.at(coalID).coalMembers.at(robIndx).goalPose.X));
                msgStr.append(",");
                msgStr.append(QString::number(coalList.at(coalID).coalMembers.at(robIndx).goalPose.Y));

                if (robIndx<(numOfMem-1))
                    msgStr.append(";");
            }

            if (msgStr.size() != 0)
                targetPosesStr.append(msgStr).append(";");
        }
        else if (cmdType == CMD_C2L_START_OR_STOP_MISSION)
        {
            if (missionParams.startMission == true)
            {
                msgStr = "START-MISSION";
            }
            else
            {
                msgStr = "STOP-MISSION";
            }
        }

        msg.message.push_back(msgStr.toStdString());

    }


    if(targetPosesStr.size() != 0){
        targetPosesStr.remove(targetPosesStr.size()-1,1);
        for(int cIndx=0; cIndx < coalList.size(); cIndx++)
        {
            msg.messageTypeID.push_back(CMD_C2L_NEW_ALL_TARGET_POSES);

            msg.leaderRobotID.push_back(coalList.at(cIndx).coalLeaderID);

            msg.message.push_back(targetPosesStr.toStdString());
        }
    }

    cmd2LeadersPub.publish(msg);

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

        missionParams.ro = result["ro"].toDouble();
        qDebug()<< " ro " << missionParams.ro;

        missionParams.numOfRobots = result["numrobots"].toInt();
        qDebug()<< " number of robots " << missionParams.numOfRobots;

        missionParams.taskSiteRadius = result["taskSiteRadius"].toDouble();
        qDebug()<< " taskSiteRadius " << missionParams.taskSiteRadius;

        coordinatorRobotID = result["taskCoordinatorRobotID"].toInt();
        qDebug()<< " coordinatorRobotID " << coordinatorRobotID;

        double radiusTmp = result["robotRadius"].toDouble();


        QString resourceStr = result["resources"].toString();
        QStringList resourceStrList = resourceStr.split(",",QString::SkipEmptyParts);
        QVector <double> resources;
        for(int i = 0; i < resourceStrList.size();i++)
        {
           resources.append(resourceStrList.at(i).toDouble());
        }
        ownRobotID = result["robotID"].toInt();


        // initialize  robotsList
        robotProp robotTmp;

        robotTmp.coalID = ownRobotID-1;
        robotTmp.robotID = ownRobotID;
        robotTmp.inGoalPose = -1;
        robotTmp.inTaskSite = -1;
        robotTmp.pose.X = -1;
        robotTmp.pose.Y = -1;
        robotTmp.goalPose.X = -1;
        robotTmp.goalPose.Y = -1;
        robotTmp.taskSitePose.X = -1;
        robotTmp.taskSitePose.Y = -1;
        robotTmp.radius = radiusTmp;
        robotTmp.resources = resources;

        robotsList.resize(missionParams.numOfRobots);

        robotsList[ownRobotID-1] = robotTmp;

        QVariantMap nestedMap = result["Robots"].toMap();
        foreach (QVariant plugin, nestedMap["Robot"].toList()) {
            QString rNameStr = plugin.toMap()["name"].toString();
            rNameStr.remove("IRobot");
            int rID = rNameStr.toInt();

            QString resourceStr =  plugin.toMap()["resources"].toString();
            QStringList resourceStrList = resourceStr.split(",",QString::SkipEmptyParts);
            QVector <double> resources;
            for(int i = 0; i < resourceStrList.size();i++)
               resources.append(resourceStrList.at(i).toDouble());

            robotTmp.coalID = rID-1;
            robotTmp.robotID = rID;
            robotTmp.resources = resources;

            robotsList[rID-1] = robotTmp;
        }

        //initialize the singleton coalitions
        for(int robID = 0; robID < robotsList.size();robID++)
        {
            coalProp newCoal;

            newCoal.coalLeaderID = robID+1;
            newCoal.coalMembers.append(robotsList.at(robID));
            newCoal.status = CS_WAITING_GOAL_POSE;
            newCoal.coalTotalResources = robotsList.at(robID).resources;
            newCoal.currentTaskUUID = "NONE";

            coalList.append(newCoal);
        }

    }

    file.close();

    return true;

}
