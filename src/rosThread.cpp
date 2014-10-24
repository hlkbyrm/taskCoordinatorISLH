#include "rosThread.h"
#include <QDebug>
#include <qjson/parser.h>
#include <QDir>
#include <QFile>



RosThread::RosThread()
{

    /* initialize random seed: */
    srand (time(NULL));

    shutdown = false;

    missionParams.startMission = false;

    //missionParams.taskCheckingPeriod = 10; //in seconds

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


    messageTaskInfoFromLeaderSub = n.subscribe("messageDecoderISLH/taskInfoFromLeader",queueSize,&RosThread::handleTaskInfoFromLeader, this);

    messagePoseListSub = n.subscribe("localizationISLH/poseList",queueSize,&RosThread::handlePoseList, this);

    messageStartMissionSub = n.subscribe("monitoringISLH/startMission", queueSize,&RosThread::handleStartMission, this);

    cmd2LeadersPub = n.advertise<ISLH_msgs::cmd2LeadersMessage>("taskCoordinatorISLH/cmd2Leaders",queueSize);

    leaderIDInfo2MonitorPub = n.advertise<std_msgs::Int8MultiArray>("taskCoordinatorISLH/leaderIDInfo2Monitor",queueSize);

    taskInfo2MonitorPub =  n.advertise<ISLH_msgs::taskInfo2MonitorMessage>("taskCoordinatorISLH/taskInfo2Monitor",queueSize);

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
    qDebug() << "Start-Stop MSG!";
    if (coordinatorRobotID == ownRobotID)
    {
        if (msg.data == 1)
        {
            missionParams.startMission = true;
            ct.start();

            std_msgs::Int8MultiArray msgLeaderID2Monitor;
            for(int robIndx=0; robIndx < robotsList.size(); robIndx++){
                msgLeaderID2Monitor.data.push_back(coalList.at(robotsList.at(robIndx).coalID).coalLeaderID);
            }
            leaderIDInfo2MonitorPub.publish(msgLeaderID2Monitor);
        }
        else
        {
            missionParams.startMission = false;
            ct.stop();
        }

        QVector <int> coalIDListTmp;
        for(int coalID=0; coalID < coalList.size(); coalID++)
            coalIDListTmp.append(coalID);

        sendCmd2Leaders(CMD_C2L_START_OR_STOP_MISSION, coalIDListTmp);
    }
}

void RosThread::manageCoalitions()
{

    bool waitingTaskAvailable=false;

    for(int wti = 0; wti < waitingTasks.size(); wti++){
        if(waitingTasks.at(wti).status == TS_NOT_ASSIGNED || waitingTasks.at(wti).status == TS_WAITING){

           uint currentTime = QDateTime::currentDateTime().toTime_t();
           if (currentTime - waitingTasks.at(wti).encounteringTime >= waitingTasks.at(wti).timeOutDuration)
           {
                waitingTasks[wti].status = TS_NOT_COMPLETED;

                timedoutTasks.append(waitingTasks.at(wti));

                pubTaskInfo2Monitor(taskProp(waitingTasks.at(wti)));

                waitingTasks.remove(wti);

                wti = wti - 1;
           }
           else
            waitingTaskAvailable = true;
        }
    }

    if (waitingTaskAvailable)
    {
        qDebug() << "managing waiting tasks";
        // check whether all the resources of the available coalitions are sufficient at least for one of the waiting tasks

        //QVector <int> availCoalIDList;
        int numOfRes = waitingTasks.at(0).requiredResources.size();
        QVector <double> availTotRes;
        availTotRes.resize(numOfRes);

        for(int coalIndx = 0; coalIndx < coalList.size();coalIndx++)
        {
            if ( (coalList.at(coalIndx).status != CS_SUCCORING) && (coalList.at(coalIndx).status != CS_HANDLING))
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

        qDebug()<< "tasks to be assigned";
        for(int wTID = 0; wTID < waitingTasks.size();wTID++)
        {
            if(waitingTasks.at(wTID).status == TS_NOT_ASSIGNED || waitingTasks.at(wTID).status == TS_WAITING)
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
                    qDebug()<<"taskUUID: "<<waitingTasks.at(wTID).taskUUID <<" encountering robot: "<<waitingTasks.at(wTID).encounteringRobotID;
                }
            }
        }

        if (wTaskIDList.size()>0)
        {

            qDebug() << "available robots for waiting tasks";

            _coalList = QVector <coalProp>(coalList);
            _waitingTasks = QVector <taskProp>(waitingTasks);

            coalListHist.append(QVector <coalProp>(coalList));

            // .................................//
            // run the coalition formation game //
            // .................................//
            runCFG(wTaskIDList);//, availCoalIDList);

            qDebug() << "runCFG is successful";

            // check whether the merged/established coalitions have sufficient resources for their associated tasks
            // in turn, send the corresponding messages to the established coalitions' members

            qDebug()<<"_coalList after runCFG";
            for(int coalIndx = 0; coalIndx < _coalList.size(); coalIndx++)
            {
                qDebug()<<"coalID: "<<coalIndx << "coal size: "<<_coalList.at(coalIndx).coalMembers.size() << "assigned taskUUID: " <<_coalList.at(coalIndx).currentTaskUUID;

            }

            QVector <int> capableCoalIDListTmp;// the coalitions capable of performing the assigned task

            qDebug()<<"Capable coalIDs";
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
                            // the established coalition "_coalList.at(coalIndx)" has sufficient
                            //  resources for the given waiting task "_waitingTasks.at(wTaskID).taskUUID"
                            capableCoalIDListTmp.append(coalIndx);
                            qDebug()<<"coalID: "<<coalIndx << "coal size: "<<_coalList.at(coalIndx).coalMembers.size() << "assigned taskUUID: " <<_coalList.at(coalIndx).currentTaskUUID;
                        }
                        else
                        {
                            _coalList[coalIndx].currentTaskUUID = "NONE";
                        }

                        break;
                    }
                }
            }

            // determining the new partition "_coalListTmp" considering the result of CGG and the current partition

            // first adding the capable coalitions to  _coalListTmp
            QVector <coalProp> _coalListTmp;
            QVector <int> newCoalIDListTmp(missionParams.numOfRobots, -1);

            for(int i = 0; i < capableCoalIDListTmp.size(); i++)
            {
                int coalID = capableCoalIDListTmp.at(i);
                _coalListTmp.append(_coalList.at(coalID));                

                for(int robIndx = 0; robIndx < _coalList.at(coalID).coalMembers.size(); robIndx++)
                {
                    int robotID = _coalList.at(coalID).coalMembers.at(robIndx).robotID;

                    _coalListTmp[i].coalMembers[robIndx].coalID = i;

                    robotsList[robotID-1].coalID = i;

                    newCoalIDListTmp[robotID-1] = i;
                }
            }


            int newCoalID = capableCoalIDListTmp.size();

            // next, removing the splitted robots (which are currentl the member of one of the capable
            //       coalitions generated by CFG) from the non-capable coalitions
            for(int coalIndx = 0; coalIndx < coalList.size(); coalIndx++)
            {
                QVector <int> coalMembersToBeRemoved;
                for(int robIndx = 0; robIndx < coalList.at(coalIndx).coalMembers.size(); robIndx++)
                {
                    int robotID = coalList.at(coalIndx).coalMembers.at(robIndx).robotID;
                    if (newCoalIDListTmp.at(robotID-1)>-1)
                        coalMembersToBeRemoved.append(robotID);
                }

                if (coalList.at(coalIndx).coalMembers.size()> coalMembersToBeRemoved.size())
                {
                    QVector <robotProp> coalMembersTmp = QVector <robotProp>(coalList.at(coalIndx).coalMembers);
                    for(int i = 0; i < coalMembersToBeRemoved.size(); i++)
                    {
                        for(int robIndx = 0; robIndx<coalMembersTmp.size(); robIndx++)
                        {
                            if (coalMembersTmp.at(robIndx).robotID == coalMembersToBeRemoved.at(i))
                            {
                                coalMembersTmp.remove(robIndx);
                                break;
                            }
                        }
                    }

                    coalProp coalTmp;

                    int minRobotID = 9999;
                    for(int robIndx = 0; robIndx<coalMembersTmp.size(); robIndx++)
                    {
                        if (coalMembersTmp.at(robIndx).robotID < minRobotID)
                            minRobotID = coalMembersTmp.at(robIndx).robotID;

                        coalMembersTmp[robIndx].coalID = newCoalID;

                        robotsList[coalMembersTmp.at(robIndx).robotID-1].coalID = newCoalID;
                    }

                    coalTmp.coalLeaderID = minRobotID;
                    coalTmp.coalMembers  = QVector <robotProp>(coalMembersTmp);
                    coalTmp.coalTotalResources = QVector <double>(calcCoalTotalResources(coalTmp.coalMembers));
                    coalTmp.currentTaskUUID = "NONE";
                    coalTmp.status = CS_WAITING_GOAL_POSE;

                    _coalListTmp.append(coalTmp);

                    newCoalID = newCoalID + 1;
                }
            }



            coalList = QVector <coalProp> (_coalListTmp);

            // generate task site poses for the capable coalitions
            qDebug()<<"Capable coalitions";
            QVector <int> capableCoalIDListTmp2;
            for(int coalID = 0; coalID < capableCoalIDListTmp.size(); coalID++)
            {
                qDebug()<<"coalID "<<coalID;
                generatePoses(coalID, TASK_SITE_POSE);
                coalList[coalID].status = CS_SUCCORING;

                capableCoalIDListTmp2.append(coalID);
            }

            QVector <int> coalIDListTmp;
            for(int coalIndx = 0; coalIndx < coalList.size(); coalIndx++)
            {
                // do not append the unchanged coalitions
                bool isUnchanged = false;
                _coalListTmp = QVector <coalProp>(coalListHist.at(coalListHist.size()-1));
                for(int coalIndx2 = 0; coalIndx2 < _coalListTmp.size(); coalIndx2++)
                {
                    if ( (_coalListTmp.at(coalIndx2).coalLeaderID != coalList.at(coalIndx).coalLeaderID) || (_coalListTmp.at(coalIndx2).status != coalList.at(coalIndx).status) || (_coalListTmp.at(coalIndx2).currentTaskUUID != coalList.at(coalIndx).currentTaskUUID) || (_coalListTmp.at(coalIndx2).coalMembers.size() != coalList.at(coalIndx).coalMembers.size()))
                        continue;

                    bool isMatchedResources = true;
                    for(int resID = 0; resID < coalList.at(coalIndx).coalTotalResources.size(); resID++)
                    {
                        if ( _coalListTmp.at(coalIndx2).coalTotalResources.at(resID) != coalList.at(coalIndx).coalTotalResources.at(resID))
                        {
                            isMatchedResources = false;
                            break;
                        }
                    }
                    if (isMatchedResources == false)
                        break;

                    int numOfMatchedRobots = 0;
                    for(int robIndx = 0; robIndx<coalList.at(coalIndx).coalMembers.size(); robIndx++)
                    {
                        for(int robIndx2 = 0; robIndx2<_coalListTmp.at(coalIndx2).coalMembers.size(); robIndx2++)
                        {
                            if (_coalListTmp.at(coalIndx2).coalMembers.at(robIndx2).robotID == coalList.at(coalIndx).coalMembers.at(robIndx).robotID)
                            {
                                numOfMatchedRobots = numOfMatchedRobots + 1;
                                break;
                            }
                        }
                    }
                    if (numOfMatchedRobots != _coalListTmp.at(coalIndx2).coalMembers.size())
                        break;

                    isUnchanged = true;
                    break;
                }

                if (isUnchanged == false)
                    coalIDListTmp.append(coalIndx);

                qDebug()<<"---- coalID ----" << coalIndx;
                for(int robIndx = 0; robIndx<coalList.at(coalIndx).coalMembers.size(); robIndx++)
                    qDebug()<<"robot "<<coalList.at(coalIndx).coalMembers.at(robIndx).robotID;

            }
            qDebug()<<"end of coals";

            sendCmd2Leaders(CMD_C2L_COALITION_MEMBERS, coalIDListTmp);

            QVector <int> capableOldCoalIDListTmp;
            for(int coalIndx = 0; coalIndx < capableCoalIDListTmp2.size(); coalIndx++)
            {
                bool avail = false;
                for(int coalIndx2 = 0; coalIndx2 < coalIDListTmp.size(); coalIndx2++)
                {
                    if (capableCoalIDListTmp2.at(coalIndx)==coalIDListTmp.at(coalIndx2))
                    {
                        avail = true;
                        break;
                    }
                }

                if (avail == false)
                    capableOldCoalIDListTmp.append(capableCoalIDListTmp2.at(coalIndx));
            }

            sendCmd2Leaders(CMD_C2L_NEW_TASK_SITE_POSES, capableOldCoalIDListTmp);

            qDebug() << "end of managing waiting tasks";

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

        qDebug() << "merge";

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
                            _waitingTasks[maxTaskID].status = TS_NOT_ASSIGNED;
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
                        int newCoalLeaderID = mergeCoalitions(coalID, mCoalID);

                        _waitingTasks[wTaskID].responsibleUnit = newCoalLeaderID;

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

        qDebug() << "split";

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

                    qDebug()<<"coalID "<<coalID << "task status" << _waitingTasks.at(wTaskID).status << "rUnit" << _waitingTasks.at(wTaskID).responsibleUnit; ;

                    int splitRobotID = findSplittedRobot(coalID, wTaskID);

                    if (splitRobotID > -1)
                    {
                        // split splitRobotID from the coalition
                        splitCoalition(splitRobotID, coalID);

                        if (splitRobotID==_waitingTasks.at(wTaskID).responsibleUnit)
                            // this means that the splitted robot is coalition leader
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
            //check whether coalIDTmp is responsible for any task
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

            QVector <robotProp> mergedCoalRobotsTmp = QVector <robotProp> (_coalList.at(coalID).coalMembers);
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
int RosThread::mergeCoalitions(int coalID, int mCoalID)
{
    int numOfRes = _waitingTasks.at(0).requiredResources.size();
    QVector <double> coalTotResourcesTmp;
    for(int resID = 0; resID < numOfRes; resID++)
    {
        double coalIDRes = _coalList.at(coalID).coalTotalResources.at(resID);
        double mCoalIDRes = _coalList.at(mCoalID).coalTotalResources.at(resID);
        coalTotResourcesTmp.append( coalIDRes + mCoalIDRes);
    }
    _coalList[coalID].coalTotalResources = QVector <double> (coalTotResourcesTmp);

    int newLeaderID = -1;

    if (_coalList.at(coalID).coalLeaderID < _coalList.at(mCoalID).coalLeaderID)
    {
         newLeaderID = _coalList.at(coalID).coalLeaderID;
    }
    else
    {
        newLeaderID = _coalList.at(mCoalID).coalLeaderID;
    }




    // add robots in mCoalID to coalID
    for(int robIndx = 0; robIndx < _coalList.at(mCoalID).coalMembers.size();robIndx++)
    {
        _coalList[mCoalID].coalMembers[robIndx].coalID = coalID;
        _coalList[coalID].coalMembers.append( _coalList.at(mCoalID).coalMembers.at(robIndx) );        
    }

    _coalList[coalID].coalLeaderID = newLeaderID;


    //merged coalition may be responsible for any task.
    // hence waitingTasks should be changed accordingly.

    if (QString::compare(_coalList.at(mCoalID).currentTaskUUID, "NONE", Qt::CaseInsensitive) != 0)
    {
        for(int wTID = 0; wTID < _waitingTasks.size();wTID++)
        {
            if (QString::compare(_waitingTasks.at(wTID).taskUUID, _coalList.at(mCoalID).currentTaskUUID,Qt::CaseInsensitive) == 0)
            {
                _waitingTasks[wTID].status = TS_NOT_ASSIGNED;
                _waitingTasks[wTID].responsibleUnit = -1;
                break;
            }
        }
    }

    _coalList[coalID].status = CS_WAITING_TASK_RESPONSE_FROM_COORDINATOR;

    _coalList.remove(mCoalID);

    return newLeaderID;

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

            QVector <robotProp> coalRobotIDsTmp = QVector <robotProp> (_coalList.at(coalID).coalMembers);

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

    // establish a new signleton coalition
   coalProp newCoal;
   newCoal.coalLeaderID = splitRobotID;

   // find the splitted robot's index (robIndx) in coalMembers vector
   int robIndx;
   for(robIndx = 0; robIndx < _coalList.at(coalID).coalMembers.size();robIndx++)
   {
       if (_coalList.at(coalID).coalMembers.at(robIndx).robotID == splitRobotID)
       {
           break;
       }
   }

   QVector <robotProp> newCoalMembers;
   newCoalMembers.append(_coalList.at(coalID).coalMembers.at(robIndx));

   newCoalMembers[0].inGoalPose = -1;
   newCoalMembers[0].inTaskSite = -1;

   newCoal.coalTotalResources =  QVector <double> (newCoalMembers.at(0).resources);

   newCoal.coalMembers = QVector <robotProp> (newCoalMembers);
   newCoal.currentTaskUUID = "NONE";
   newCoal.status = CS_WAITING_GOAL_POSE;

   // add the new singleton coalition to _coalList
   _coalList.append(newCoal);

   // update the total resources of the coalition coalID after splitting
   for(int resID=0; resID<_coalList.at(coalID).coalTotalResources.size();resID++)
   {
       _coalList[coalID].coalTotalResources[resID] = _coalList[coalID].coalTotalResources[resID] - _coalList.at(coalID).coalMembers.at(robIndx).resources.at(resID);
   }

   // remove the splitted member from the coalition coalID
   _coalList[coalID].coalMembers.remove(robIndx);



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


// calculate a given coalition's total resources
QVector <double> RosThread::calcCoalTotalResources(QVector <robotProp> coalMembers)
{

    int numOfResources = coalMembers.at(0).resources.size();
    QVector <double> coalTotalResources;
    coalTotalResources.resize(numOfResources);
    for(int robID=0;robID<coalMembers.size();robID++){
        for(int resID=0;resID<numOfResources;resID++){
            coalTotalResources[resID] += coalMembers.at(robID).resources.at(resID);
        }
    }

    return coalTotalResources;
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

    if ( (infoMsg.infoTypeID == INFO_L2C_INSUFFICIENT_RESOURCE) || (infoMsg.infoTypeID == INFO_L2C_WAITING_TASK_SITE_POSE) )
    {
        taskProp newTask;

        newTask.encounteringRobotID = infoMsg.encounteringRobotID;

        newTask.taskUUID = QString::fromStdString(infoMsg.taskUUID);

        newTask.timeOutDuration = infoMsg.timeOutDuration;

        newTask.handlingDuration = infoMsg.handlingDuration;

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

        if ( infoMsg.infoTypeID == INFO_L2C_INSUFFICIENT_RESOURCE )
        {
            qDebug() << "ins. resource" << infoMsg.senderRobotID << QString::fromStdString(infoMsg.taskUUID);

            newTask.status = TS_NOT_ASSIGNED;
            newTask.responsibleUnit = -1;

            waitingTasks.append(newTask);

            for(int i = 0; i < coalList.size();i++)
            {
                if (coalList.at(i).coalLeaderID == infoMsg.senderRobotID)
                {
                    coalList[i].status = CS_WAITING_TASK_RESPONSE_FROM_COORDINATOR;
                    coalList[i].currentTaskUUID = QString::fromStdString(infoMsg.taskUUID);
                    qDebug()<<"coalition leader was found.";
                    break;
                }
            }
        }
        else // INFO_L2C_WAITING_TASK_SITE_POSE
        {
            qDebug() << "waiting task site pose" << infoMsg.senderRobotID << QString::fromStdString(infoMsg.taskUUID);
            newTask.status = TS_SUCCORING;
            newTask.responsibleUnit = infoMsg.senderRobotID;

            waitingTasks.append(newTask);


            for(int i = 0; i < coalList.size();i++)
            {
                if (coalList.at(i).coalLeaderID == infoMsg.senderRobotID)
                {
                    coalList[i].currentTaskUUID = QString::fromStdString(infoMsg.taskUUID);

                    generatePoses(i, TASK_SITE_POSE);

                    QVector <int> coalIDListTmp;
                    coalIDListTmp.append(i);

                    sendCmd2Leaders(CMD_C2L_NEW_TASK_SITE_POSES, coalIDListTmp);

                    coalList[i].status = CS_SUCCORING;


                    break;
                }
            }

        }

        pubTaskInfo2Monitor(taskProp(waitingTasks.last()));

    }
    else if  (infoMsg.infoTypeID == INFO_L2C_START_HANDLING_WITH_TASK_INFO)
    {
        qDebug() << "start handling w task info" << infoMsg.senderRobotID << QString::fromStdString(infoMsg.taskUUID);
        taskProp newTask;

        newTask.encounteringRobotID = infoMsg.encounteringRobotID;

        newTask.taskUUID = QString::fromStdString(infoMsg.taskUUID);

        newTask.pose.X = infoMsg.posX;

        newTask.pose.Y = infoMsg.posY;

        newTask.encounteringTime = infoMsg.encounteringTime;

        newTask.handlingDuration = infoMsg.handlingDuration;

        newTask.startHandlingTime = infoMsg.startHandlingTime;

        newTask.timeOutDuration = infoMsg.timeOutDuration;

        newTask.requiredResourcesString = QString::fromStdString(infoMsg.requiredResources);

        QStringList resourceParts = newTask.requiredResourcesString.split(",",QString::SkipEmptyParts);

        newTask.requiredResources.clear();

        for(int i = 0; i < resourceParts.size();i++)
        {
            newTask.requiredResources.append(resourceParts.at(i).toDouble());
        }

        int numOfMem = -1;
        int coalID = -1;
        for(int i = 0; i < coalList.size();i++)
        {
            if (coalList.at(i).coalLeaderID == infoMsg.senderRobotID)
            {
                coalID = i;
                numOfMem = coalList.at(i).coalMembers.size();
                break;
            }
        }
        newTask.taskSiteRadius = missionParams.targetSiteRadius*numOfMem;

        newTask.status = TS_HANDLING;

        handlingTasks.append(newTask);

        pubTaskInfo2Monitor(taskProp(newTask));

        if (coalID > -1)
        {
            /*
        for(int i = 0; i < coalList.size();i++)
        {
            if (coalList.at(i).coalLeaderID == infoMsg.senderRobotID)
            {
                */
            coalList[coalID].status = CS_HANDLING;
            coalList[coalID].currentTaskUUID = QString::fromStdString(infoMsg.taskUUID);

            ISLH_msgs::cmd2LeadersMessage msg;

            std::time_t sendingTime = std::time(0);
            msg.sendingTime = sendingTime;

            QString msgStr;
            int numOfMem = coalList.at(coalID).coalMembers.size();
            for(int robIndx=0; robIndx < numOfMem; robIndx++)
            {
                msgStr.append(QString::number(coalList.at(coalID).coalMembers.at(robIndx).robotID));
                msgStr.append(",");
                msgStr.append(QString::number(robotsList[coalList.at(coalID).coalMembers.at(robIndx).robotID-1].pose.X));
                msgStr.append(",");
                msgStr.append(QString::number(robotsList[coalList.at(coalID).coalMembers.at(robIndx).robotID-1].pose.Y));

                if (robIndx<(numOfMem-1))
                    msgStr.append(";");
            }

            for(int cIndx=0; cIndx < coalList.size(); cIndx++)
            {
                msg.messageTypeID.push_back(CMD_C2L_NEW_ALL_TARGET_POSES);

                msg.leaderRobotID.push_back(coalList.at(cIndx).coalLeaderID);

                msg.message.push_back(msgStr.toStdString());
            }

            cmd2LeadersPub.publish(msg);


            //break;
            //}
            //    }
        }
        else
        {
            qDebug()<<"Error: handleTaskInfoFromLeader:INFO_L2C_START_HANDLING_WITH_TASK_INFO: coalID is not available in the coordinator";
        }
    }
    else if  (infoMsg.infoTypeID == INFO_L2C_START_HANDLING)
    {
        qDebug() << "start handling w.o task info" << infoMsg.senderRobotID;

        QString taskUUIDTmp = QString::fromStdString(infoMsg.taskUUID);

        for(int i = 0; i < waitingTasks.size();i++)
        {
            if (QString::compare(waitingTasks.at(i).taskUUID, taskUUIDTmp,Qt::CaseInsensitive) == 0)
            {
                waitingTasks[i].startHandlingTime = infoMsg.startHandlingTime;
                waitingTasks[i].responsibleUnit = infoMsg.senderRobotID;
                waitingTasks[i].status = TS_HANDLING;

                pubTaskInfo2Monitor(taskProp(waitingTasks.at(i)));

                handlingTasks.append(waitingTasks.at(i));

                waitingTasks.remove(i);

                break;
            }
        }

/*
        for(int i = 0; i < coalList.size();i++)
        {
            if (coalList.at(i).coalLeaderID == infoMsg.senderRobotID)
            {
                ISLH_msgs::cmd2LeadersMessage msg;

                std::time_t sendingTime = std::time(0);
                msg.sendingTime = sendingTime;

                QString msgStr;
                int numOfMem = coalList.at(i).coalMembers.size();
                for(int robIndx=0; robIndx < numOfMem; robIndx++)
                {
                    msgStr.append(QString::number(coalList.at(i).coalMembers.at(robIndx).robotID));
                    msgStr.append(",");
                    msgStr.append(QString::number(robotsList[coalList.at(i).coalMembers.at(robIndx).robotID-1].pose.X));
                    msgStr.append(",");
                    msgStr.append(QString::number(robotsList[coalList.at(i).coalMembers.at(robIndx).robotID-1].pose.Y));

                    if (robIndx<(numOfMem-1))
                        msgStr.append(";");

                    for(int cIndx=0; cIndx < coalList.size(); cIndx++)
                    {
                        msg.messageTypeID.push_back(CMD_C2L_NEW_ALL_TARGET_POSES);

                        msg.leaderRobotID.push_back(coalList.at(cIndx).coalLeaderID);

                        msg.message.push_back(msgStr.toStdString());
                    }

                    cmd2LeadersPub.publish(msg);
                }
                break;
            }
        }
        */

    }
    else if  (infoMsg.infoTypeID == INFO_L2C_TASK_COMPLETED)
    {
        qDebug() << "task completed" << infoMsg.senderRobotID << QString::fromStdString(infoMsg.taskUUID);
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

                handlingTasks[i].status = TS_COMPLETED;

                pubTaskInfo2Monitor(taskProp(handlingTasks.at(i)));

                completedTasks.append(handlingTasks.at(i));

                handlingTasks.remove(i);

                break;
            }
        }
    }
    else if ( (infoMsg.infoTypeID == INFO_L2C_SPLITTING) || (infoMsg.infoTypeID == INFO_L2C_SPLITTING_AND_LEADER_CHANGED) )
    {
        coalListHist.append(QVector <coalProp>(coalList));

        qDebug() << "splitting" << infoMsg.senderRobotID;

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
                        qDebug()<<"splitting part1";
                        robotProp splittedRobot = coalList.at(cid).coalMembers.at(rid);
                        coalProp singletonCoal;
                        singletonCoal.coalLeaderID = splittedRobot.robotID;
                        singletonCoal.coalMembers.append(splittedRobot);
                        singletonCoal.status = CS_WAITING_GOAL_POSE;                        
                        singletonCoal.coalTotalResources = singletonCoal.coalMembers.at(0).resources;

                        qDebug()<<"splitting part2";

                        coalList.append(singletonCoal);
                        coalList[cid].coalMembers.remove(rid);
                        coalList[cid].coalTotalResources = QVector <double>(calcCoalTotalResources(coalList.at(cid).coalMembers));

                        qDebug()<<"splitting part3";
                        robotsList[splittedRobot.robotID-1].coalID = coalList.size()-1;

                        qDebug()<<"splitting part4";
                        splittingRobotIDList.removeAt(srid);

                        if (infoMsg.infoTypeID == INFO_L2C_SPLITTING_AND_LEADER_CHANGED)
                        {
                            qDebug()<<"splitting part5";
                            //Assign a new coalition leader ID
                            coalList[cid].coalLeaderID = newCoalLeaderID;                                                       
                        }
                    }
                }
            }
        }

        qDebug()<<"splitting part6";
        std_msgs::Int8MultiArray msgLeaderID2Monitor;
        for(int robIndx=0; robIndx < robotsList.size(); robIndx++){
            msgLeaderID2Monitor.data.push_back(coalList.at(robotsList.at(robIndx).coalID).coalLeaderID);
        }
       leaderIDInfo2MonitorPub.publish(msgLeaderID2Monitor);

    }
    else if (infoMsg.infoTypeID == INFO_L2C_WAITING_GOAL_POSE)
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
        {
            qDebug()<< "robot "<<leaderRobotID << " is no longer a coalition leader. Hence I am ignoring the goal pose request. ";
        }
        else
        {

            generatePoses(coalID, GOAL_POSE);

            QVector <int> coalIDListTmp;
            coalIDListTmp.append(coalID);

            sendCmd2Leaders(CMD_C2L_NEW_GOAL_POSES, coalIDListTmp);
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

        while(true){
            bool isAllDone = true;
            int radiusIncreaseCounter = 0;
            double goalX;
            double goalY;
            goalX = ((rand()/ (RAND_MAX + 1.0)) * (2*missionParams.ro)) - missionParams.ro;
            goalY = ((rand()/ (RAND_MAX + 1.0)) * (2*missionParams.ro)) - missionParams.ro;

            for(int goalSiteRadius = missionParams.targetSiteRadius * numOfMem;true;goalSiteRadius += 10)
            {
                if(++radiusIncreaseCounter > 10){
                    isAllDone = false;
                    break;
                }
                bool done = true;
                int counter = 0;
                for(int robIndx=0; robIndx < numOfMem; robIndx++)
                {
                    int robotID = coalList.at(coalID).coalMembers.at(robIndx).robotID;

                    //double robotRadius = coalList.at(coalID).coalMembers.at(robIndx).radius;
                    double robotRadius = robotsList.at(coalList.at(coalID).coalMembers.at(robIndx).robotID-1).radius;
                    double robX,robY;
                    bool doneInside = true;

                    int poseOK = 0;
                    while(poseOK==0)
                    {
                        if((++counter % 100) == 0){
                            doneInside = false;
                            break;
                        }
                        robX = goalX + ((rand()/ (RAND_MAX + 1.0)) * (2*goalSiteRadius)) - goalSiteRadius;
                        robY = goalY + ((rand()/ (RAND_MAX + 1.0)) * (2*goalSiteRadius)) - goalSiteRadius;

                        if (sqrt(robX*robX + robY*robY) <= (missionParams.ro-3*robotRadius))
                        {
                            int distOK = 1;

                            // check dist btw xy & other robots' goal pose
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

                                        if ( sqrt((robX-xTmp)*(robX-xTmp) + (robY-yTmp)*(robY-yTmp)) <= (robotRadius + robotRadius2 + distThreshold4GeneratingPoses) )
                                        {
                                            distOK = 0;
                                            break;
                                        }
                                    }
                                }
                            }

                            // check dist btw xy & its coalition members' goal pose
                            if (distOK == 1)
                            {
                                for(int robIndx2=0; robIndx2 < robIndx; robIndx2++)
                                {
                                    if (robotsList.at(robIndx2).inGoalPose != -1)
                                        // robIndx2 has an assigned goal pose?
                                    {
                                        double xTmp = coalList.at(coalID).coalMembers.at(robIndx2).goalPose.X;
                                        double yTmp = coalList.at(coalID).coalMembers.at(robIndx2).goalPose.Y;

                                        //double robotRadius2 = coalList.at(coalID).coalMembers.at(robIndx2).radius;
                                        double robotRadius2 = robotsList.at(coalList.at(coalID).coalMembers.at(robIndx2).robotID-1).radius;

                                        if ( sqrt((robX-xTmp)*(robX-xTmp) + (robY-yTmp)*(robY-yTmp)) <= (robotRadius + robotRadius2 + distThreshold4GeneratingPoses) )
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

                                    if ( sqrt((robX-xTmp)*(robX-xTmp) + (robY-yTmp)*(robY-yTmp)) <= (robotRadius + waitingTasks.at(wTID).taskSiteRadius) )
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

                                        if ( sqrt((robX-xTmp)*(robX-xTmp) + (robY-yTmp)*(robY-yTmp)) <= (robotRadius + handlingTasks.at(hTID).taskSiteRadius) )
                                        {
                                            distOK = 0;
                                            break;
                                        }
                                    }
                                }
                            }


                            if (distOK == 1)
                                poseOK = 1;
                        }
                    } // end of while(poseOK==0)
                    if(!doneInside){
                        int x = counter / 10;
                        while(x != 0){
                            x/=10;
                            robIndx--;
                        }
                        robIndx--;
                        if(robIndx < 0) { done = false; break; }
                        continue;
                    }
                    counter -= counter%100;

                    robotsList[robotID-1].goalPose.X = robX;
                    robotsList[robotID-1].goalPose.Y = robY;

                    robotsList[robotID-1].inTaskSite = -1;
                    robotsList[robotID-1].inGoalPose = 0;

                    coalList[coalID].coalMembers[robIndx].goalPose.X = robX;
                    coalList[coalID].coalMembers[robIndx].goalPose.Y = robY;
                    coalList[coalID].coalMembers[robIndx].inTaskSite = -1;
                    coalList[coalID].coalMembers[robIndx].inGoalPose = 0;
                }
                if(done) break;
            }
            if(isAllDone) break;
        }
    }
    else if (poseType == TASK_SITE_POSE)
    {
        int taskID = -1;
        for(int wti = 0; wti < waitingTasks.size();wti++)
        {
            if (QString::compare(waitingTasks.at(wti).taskUUID, coalList.at(coalID).currentTaskUUID,Qt::CaseInsensitive) == 0)
            {
               taskID = wti;
               break;
            }
        }

        double taskPoseX = waitingTasks.at(taskID).pose.X;
        double taskPoseY = waitingTasks.at(taskID).pose.Y;


        int numOfMem = coalList.at(coalID).coalMembers.size();

        // check whether other robots' goal poses are inside the task site
        for(int robIndx2=0; robIndx2 < robotsList.size(); robIndx2++)
        {
            if (robotsList.at(robIndx2).coalID != coalID)
            {
                if (robotsList.at(robIndx2).inGoalPose != -1)
                    // robIndx2 has an assigned goal pose?
                {
                    double xTmp = robotsList.at(robIndx2).goalPose.X;
                    double yTmp = robotsList.at(robIndx2).goalPose.Y;

                    double robotRadius2 = robotsList.at(robIndx2).radius;

                    if ( sqrt((taskPoseX-xTmp)*(taskPoseX-xTmp) + (taskPoseY-yTmp)*(taskPoseY-yTmp)) <= missionParams.targetSiteRadius * numOfMem + robotRadius2)
                    {
                        generatePoses(robotsList.at(robIndx2).coalID,GOAL_POSE);

                        QVector<int> coalIDListTmp;
                        coalIDListTmp.append(robotsList.at(robIndx2).coalID);

                        sendCmd2Leaders(CMD_C2L_NEW_GOAL_POSES, coalIDListTmp);

                        break;
                    }
                }
            }
        }

        for(waitingTasks[taskID].taskSiteRadius = missionParams.targetSiteRadius * numOfMem;true;waitingTasks[taskID].taskSiteRadius += 10){
            bool done = true;
            int counter = 0;
            for(int robIndx=0; robIndx < numOfMem; robIndx++)
            {
                int robotID = coalList.at(coalID).coalMembers.at(robIndx).robotID;

                double robotRadius = robotsList.at(robotID-1).radius;//coalList.at(coalID).coalMembers.at(robIndx).radius;

                int poseOK = 0;
                double robX;
                double robY;
                bool doneInside = true;
                while(poseOK == 0)
                {
                    if((++counter % 100) == 0){
                        doneInside = false;
                        break;
                    }
                    robX = taskPoseX + ((rand()/ (RAND_MAX + 1.0)) * (2*(waitingTasks.at(taskID).taskSiteRadius - robotRadius))) - (waitingTasks.at(taskID).taskSiteRadius - robotRadius);
                    robY = taskPoseY + ((rand()/ (RAND_MAX + 1.0)) * (2*(waitingTasks.at(taskID).taskSiteRadius - robotRadius))) - (waitingTasks.at(taskID).taskSiteRadius - robotRadius);

                    if (sqrt(robX*robX + robY*robY) <= (missionParams.ro-robotRadius))
                    {
                        int distOK = 1;

                        /// check dist btw xy & other robots' pose
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

                                    if ( sqrt((robX-xTmp)*(robX-xTmp) + (robY-yTmp)*(robY-yTmp)) <= (robotRadius + robotRadius2 + distThreshold4GeneratingPoses) )
                                    {
                                        distOK = 0;
                                        break;
                                    }
                                }
                            }
                        }

                        /// check with other robots in the same coalision
                        if (distOK == 1)
                        {
                            for(int _robIndx=0; _robIndx < robIndx; _robIndx++){
                                poseXY oldRobotPose = coalList.at(coalID).coalMembers.at(_robIndx).taskSitePose;
                                //poseXY thisRobotPose = coalList.at(coalID).coalMembers.at(robIndx).taskSitePose;

                                double robotRadius2 = robotsList.at(_robIndx).radius;
                                double dist = sqrt((oldRobotPose.X - robX)*(oldRobotPose.X - robX) + (oldRobotPose.Y - robY)*(oldRobotPose.Y - robY));
                                if(dist < robotRadius + robotRadius2 + distThreshold4GeneratingPoses){
                                    distOK = 0;
                                    break;
                                }
                            }
                        }


                        /// check dist btw xy & tasks' site pose
                        if (distOK == 1)
                        {
                            for(int wTID = 0; wTID < waitingTasks.size();wTID++)
                            {
                                if(wTID == taskID) continue;

                                double xTmp = waitingTasks.at(wTID).pose.X;
                                double yTmp = waitingTasks.at(wTID).pose.Y;

                                if ( sqrt((robX-xTmp)*(robX-xTmp) + (robY-yTmp)*(robY-yTmp)) <= (robotRadius + waitingTasks.at(wTID).taskSiteRadius) )
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

                                    if ( sqrt((robX-xTmp)*(robX-xTmp) + (robY-yTmp)*(robY-yTmp)) <= (robotRadius + handlingTasks.at(hTID).taskSiteRadius) )
                                    {
                                        distOK = 0;
                                        break;
                                    }
                                }
                            }
                        }


                        if (distOK == 1)
                            poseOK = 1;
                    }

                }
                if(!doneInside){
                    int x = counter / 10;
                    while(x != 0){
                        x/=10;
                        robIndx--;
                    }
                    robIndx--;
                    if(robIndx < 0) { done = false; break; }
                    continue;
                }
                counter -= counter%100;
                coalList[coalID].coalMembers[robIndx].taskSitePose.X = robX;
                coalList[coalID].coalMembers[robIndx].taskSitePose.Y = robY;
                coalList[coalID].coalMembers[robIndx].inGoalPose = -1;
                robotsList[robotID - 1].taskSitePose.X = robX;
                robotsList[robotID - 1].taskSitePose.Y = robY;
                robotsList[robotID - 1].inGoalPose = -1;
            }
            if(done) break;
        }
        pubTaskInfo2Monitor(taskProp(waitingTasks.at(taskID)));
    }

}

void RosThread::generatePoses2(int coalID, int poseType){
    if(poseType == GOAL_POSE){
        int numOfMem = coalList.at(coalID).coalMembers.size();

        QVector<Robot> robots;
        foreach(robotProp r,coalList.at(coalID).coalMembers){
            double radius = robotsList[r.robotID - 1].radius;

            robots.push_back(Robot(Point(0,0),radius + distThreshold4GeneratingPoses / 2,r.robotID));
        }

        QVector<Obstacle> obstacles;
        // other robots' goal pose as obstacle
        foreach(robotProp r,robotsList){
            if(r.inGoalPose == -1) continue;

            bool flag = false;
            foreach(robotProp _r,coalList.at(coalID).coalMembers){
                if(r.robotID == _r.robotID){
                    flag = true;
                    break;;
                }
            }
            if(flag) continue;

            obstacles.push_back(Obstacle(Point(r.goalPose.X,r.goalPose.Y), r.radius + distThreshold4GeneratingPoses / 2));
        }

        // tasks' site pose as obstacle
        foreach(taskProp t,handlingTasks)
            obstacles.push_back(Obstacle(Point(t.pose.X,t.pose.Y), t.taskSiteRadius + distThreshold4GeneratingPoses / 2));
        foreach(taskProp t,waitingTasks)
            obstacles.push_back(Obstacle(Point(t.pose.X,t.pose.Y), t.taskSiteRadius + distThreshold4GeneratingPoses / 2));

        // set an origin point for goal poses
        bool isOK = false;
        Point p(0,0);
        while(!isOK){
            isOK = true;

            double radius = rand() / (RAND_MAX + 1.0) * (missionParams.ro - distThreshold4GeneratingPoses);
            double alpha = rand() / (RAND_MAX + 1.0) * (2 * M_PI);
            p = Point(radius * cos(alpha),radius * sin(alpha));

            foreach(taskProp t,handlingTasks){
                if(sqrt((t.pose.X - p.x) * (t.pose.X - p.x) + (t.pose.Y - p.y) * (t.pose.Y - p.y)) < t.taskSiteRadius){
                    isOK = false;
                    break;
                }
            }
        }

        PlaceRobots pl(robots,obstacles,p,numOfMem*missionParams.targetSiteRadius,missionParams.ro);
        pl.startCalculating();

        for(int robIndx = 0;robIndx<coalList.at(coalID).coalMembers.size();robIndx++){
            Robot closest;
            robotProp r = robotsList[coalList.at(coalID).coalMembers.at(robIndx).robotID - 1];

            double minDist = missionParams.ro * 999;
            foreach(Robot _r,pl.robots){
                if(_r.robotID == -1) continue;

                double dist = sqrt((_r.center.x - r.pose.X) * (_r.center.x - r.pose.X) +
                                                (_r.center.y - r.pose.Y) * (_r.center.y - r.pose.Y));
                if(dist < minDist){
                    closest = _r;
                    minDist = dist;
                }
            }

            for(int i=0;i<pl.robots.size();i++){
                if(pl.robots[i].robotID == closest.robotID){
                    pl.robots[i].robotID = -1;
                    break;
                }
            }

            robotsList[r.robotID-1].goalPose.X = closest.center.x;
            robotsList[r.robotID-1].goalPose.Y = closest.center.y;
            robotsList[r.robotID-1].inTaskSite = -1;
            robotsList[r.robotID-1].inGoalPose = 0;
            coalList[coalID].coalMembers[robIndx].goalPose.X = closest.center.x;
            coalList[coalID].coalMembers[robIndx].goalPose.Y = closest.center.y;
            coalList[coalID].coalMembers[robIndx].inTaskSite = -1;
            coalList[coalID].coalMembers[robIndx].inGoalPose = 0;
        }
    }
    else if(poseType == TASK_SITE_POSE){
        int taskID = -1;
        for(int wti = 0; wti < waitingTasks.size();wti++)
        {
            if (QString::compare(waitingTasks.at(wti).taskUUID, coalList.at(coalID).currentTaskUUID,Qt::CaseInsensitive) == 0)
            {
               taskID = wti;
               break;
            }
        }

        int numOfMem = coalList.at(coalID).coalMembers.size();

        QVector<Robot> robots;
        foreach(robotProp r,coalList.at(coalID).coalMembers){
            double radius = robotsList[r.robotID - 1].radius;

            robots.push_back(Robot(Point(0,0),radius + distThreshold4GeneratingPoses / 2,r.robotID));
        }

        QVector<Obstacle> obstacles;
        // tasks' site pose as obstacle
        foreach(taskProp t,handlingTasks)
            obstacles.push_back(Obstacle(Point(t.pose.X,t.pose.Y), t.taskSiteRadius + distThreshold4GeneratingPoses / 2));
        foreach(taskProp t,waitingTasks)
            obstacles.push_back(Obstacle(Point(t.pose.X,t.pose.Y), t.taskSiteRadius + distThreshold4GeneratingPoses / 2));

        // set an origin point for goal poses
        Point p(waitingTasks.at(taskID).pose.X,waitingTasks.at(taskID).pose.Y);

        PlaceRobots pl(robots,obstacles,p,numOfMem*missionParams.targetSiteRadius,missionParams.ro);
        pl.startCalculating();

        for(int robIndx = 0;robIndx<coalList.at(coalID).coalMembers.size();robIndx++){
            Robot closest;
            robotProp r = robotsList[coalList.at(coalID).coalMembers.at(robIndx).robotID - 1];

            double minDist = missionParams.ro * 999;
            foreach(Robot _r,pl.robots){
                if(_r.robotID == -1) continue;

                double dist = sqrt((_r.center.x - r.pose.X) * (_r.center.x - r.pose.X) +
                                                (_r.center.y - r.pose.Y) * (_r.center.y - r.pose.Y));
                if(dist < minDist){
                    closest = _r;
                    minDist = dist;
                }
            }

            for(int i=0;i<pl.robots.size();i++){
                if(pl.robots[i].robotID == closest.robotID){
                    pl.robots[i].robotID = -1;
                    break;
                }
            }

            robotsList[r.robotID-1].taskSitePose.X = closest.center.x;
            robotsList[r.robotID-1].taskSitePose.Y = closest.center.y;
            robotsList[r.robotID-1].inTaskSite = 0;
            robotsList[r.robotID-1].inGoalPose = -1;
            coalList[coalID].coalMembers[robIndx].taskSitePose.X = closest.center.x;
            coalList[coalID].coalMembers[robIndx].taskSitePose.Y = closest.center.y;
            coalList[coalID].coalMembers[robIndx].inTaskSite = 0;
            coalList[coalID].coalMembers[robIndx].inGoalPose = -1;
        }
    }
}


// prepare a command message to the coalition leader
void RosThread::sendCmd2Leaders(int cmdType, QVector <int> coalIDList)
{
    qDebug()<< "sendCmd2Leaders-> cmdType: "<<cmdType;

    ISLH_msgs::cmd2LeadersMessage msg;

    std::time_t sendingTime = std::time(0);
    msg.sendingTime = sendingTime;


    bool isLeaderChanged = false;

    // send the CMD_C2L_LEADER_CHANGE message to the robots which are no longer coalition leader
    if(cmdType == CMD_C2L_COALITION_MEMBERS){
        QVector <coalProp> coalListPrev = QVector <coalProp> (coalListHist.last());
        for(int coalPrevIndx = 0; coalPrevIndx <coalListPrev.size();coalPrevIndx++){
            int leaderID =  coalListPrev.at(coalPrevIndx).coalLeaderID;
            bool isLeader = false;
            for(int coalIndx=0; coalIndx < coalList.size(); coalIndx++){
                if (leaderID == coalList.at(coalIndx).coalLeaderID){
                    isLeader = true;
                    break;
                }
            }

            if (isLeader == false){
                msg.messageTypeID.push_back(CMD_C2L_LEADER_CHANGE);

                msg.leaderRobotID.push_back(leaderID);

                msg.message.push_back("1");
            }
        }


        for(int i=0;i<robotsList.size();i++){
            bool flag = false;
            for(int coalPrevIndx = 0; coalPrevIndx <coalListPrev.size();coalPrevIndx++){
                if(coalList[robotsList[i].coalID].coalLeaderID == coalListPrev[coalPrevIndx].coalLeaderID){
                    for(int j = 0;j < coalListPrev[coalPrevIndx].coalMembers.size(); j++){
                        if(coalListPrev[coalPrevIndx].coalMembers[j].robotID == robotsList[i].robotID){
                            flag = true;
                            break;
                        }
                    }
                    break;
                }
            }
            if(!flag){
                isLeaderChanged = true;
                break;
            }
        }
    }


    // if the coalition leaders are changed, send  the coalition leader ID info of each robot to the monitor
    if (isLeaderChanged){
        std_msgs::Int8MultiArray msgLeaderID2Monitor;

        for(int robIndx=0; robIndx < robotsList.size(); robIndx++){
            msgLeaderID2Monitor.data.push_back(coalList.at(robotsList.at(robIndx).coalID).coalLeaderID);
        }

       leaderIDInfo2MonitorPub.publish(msgLeaderID2Monitor);
    }




    QString targetPosesStr;

    for(int cIndx=0; cIndx < coalIDList.size(); cIndx++)
    {
        int coalID = coalIDList.at(cIndx);

        qDebug() <<"coalID: "<< coalID <<"coalLeaderID: "<< coalList.at(coalID).coalLeaderID;

        msg.leaderRobotID.push_back(coalList.at(coalID).coalLeaderID);

        msg.messageTypeID.push_back(cmdType);

        QString msgStr;
        if (cmdType == CMD_C2L_NEW_GOAL_POSES || cmdType == CMD_C2L_NEW_TASK_SITE_POSES)
        {
            // msgStr = robotID1,posex1,posey1;robotID2,posex2,posey2;...;robotIDn,posexn,poseyn

            int numOfMem = coalList.at(coalID).coalMembers.size();
            for(int robIndx=0; robIndx < numOfMem; robIndx++)
            {
                msgStr.append(QString::number(coalList.at(coalID).coalMembers.at(robIndx).robotID));
                msgStr.append(",");
                if (cmdType == CMD_C2L_NEW_GOAL_POSES)
                    msgStr.append(QString::number(coalList.at(coalID).coalMembers.at(robIndx).goalPose.X));
                else
                    msgStr.append(QString::number(coalList.at(coalID).coalMembers.at(robIndx).taskSitePose.X));

                msgStr.append(",");

                if (cmdType == CMD_C2L_NEW_GOAL_POSES)
                    msgStr.append(QString::number(coalList.at(coalID).coalMembers.at(robIndx).goalPose.Y));
                else
                    msgStr.append(QString::number(coalList.at(coalID).coalMembers.at(robIndx).taskSitePose.Y));

                if (robIndx<(numOfMem-1))
                    msgStr.append(";");
            }

            if (msgStr.size() != 0)
                targetPosesStr.append(msgStr).append(";");
        }
        else if (cmdType == CMD_C2L_NEW_ALL_TARGET_POSES)
        {
            // msgStr = robotID1,posex1,posey1;robotID2,posex2,posey2;...;robotIDn,posexn,poseyn

            int numOfMem = coalList.at(coalID).coalMembers.size();
            for(int robIndx=0; robIndx < numOfMem; robIndx++)
            {
                msgStr.append(QString::number(coalList.at(coalID).coalMembers.at(robIndx).robotID));
                msgStr.append(",");
                msgStr.append(QString::number(coalList.at(coalID).coalMembers.at(robIndx).taskSitePose.X));
                msgStr.append(",");
                msgStr.append(QString::number(coalList.at(coalID).coalMembers.at(robIndx).taskSitePose.Y));

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
        else if (cmdType == CMD_C2L_COALITION_MEMBERS)
        {

            // msgStr = numberOfMembers:robotID1;res1,res2,..,resn;posex,posey;goalOrTaskSite, targetx,targety:robotID2;res1,res2,...,resn;posex,posey;goalOrTaskSite,targetx,targety

            // number of members in the coalition
            msgStr.append(QString::number(coalList.at(coalID).coalMembers.size()));

            // send also coalition members' info to the leader
            //robotID1;res1,res2,..,resn;posex,posey: ...
            for(int i = 0; i < coalList.at(coalID).coalMembers.size();i++)
            {
                int robID = coalList.at(coalID).coalMembers.at(i).robotID;

                msgStr.append(":");

                msgStr.append(QString::number(robID));
                msgStr.append(";");
                for(int j=0; j<coalList.at(coalID).coalMembers.at(i).resources.size();j++)
                {
                    msgStr.append(QString::number(coalList.at(coalID).coalMembers.at(i).resources.at(j)));
                    if (j<coalList.at(coalID).coalMembers.at(i).resources.size()-1)
                        msgStr.append(",");
                }
                msgStr.append(";"); // member pose
                msgStr.append(QString::number(coalList.at(coalID).coalMembers.at(i).pose.X));
                msgStr.append(",");
                msgStr.append(QString::number(coalList.at(coalID).coalMembers.at(i).pose.Y));
                msgStr.append(";");

                if (coalList.at(coalID).status == CS_SUCCORING){
                    msgStr.append("t"); // task site pose
                    msgStr.append(",");
                    msgStr.append(QString::number(coalList.at(coalID).coalMembers.at(i).taskSitePose.X));
                    msgStr.append(",");
                    msgStr.append(QString::number(coalList.at(coalID).coalMembers.at(i).taskSitePose.Y));
                }
                else{
                    msgStr.append("g"); // goal pose
                    msgStr.append(",");
                    msgStr.append(QString::number(coalList.at(coalID).coalMembers.at(i).goalPose.X));
                    msgStr.append(",");
                    msgStr.append(QString::number(coalList.at(coalID).coalMembers.at(i).goalPose.Y));
                }
            }


            if (QString::compare(coalList.at(coalID).currentTaskUUID, "NONE", Qt::CaseInsensitive) != 0)
            {
                // if the currentTaskUUID is not "NONE, which means the coalition is going to handle a task, send the leader the task to be handled
                int wti;
                for(wti = 0; wti < waitingTasks.size();wti++)
                {
                    if ( QString::compare(waitingTasks.at(wti).taskUUID, coalList.at(coalID).currentTaskUUID, Qt::CaseInsensitive) == 0)
                    {
                      break;
                    }

                }
                msgStr.append(":");
                // add the task properties to the message
                msgStr.append(waitingTasks.at(wti).taskUUID);
                msgStr.append(";");
                msgStr.append(QString::number(waitingTasks.at(wti).pose.X));
                msgStr.append(";");
                msgStr.append(QString::number(waitingTasks.at(wti).pose.Y));
                msgStr.append(";");
                msgStr.append(QString::number(waitingTasks.at(wti).encounteringRobotID));
                msgStr.append(";");
                msgStr.append(QString::number(waitingTasks.at(wti).handlingDuration));
                msgStr.append(";");
                msgStr.append(QString::number(waitingTasks.at(wti).timeOutDuration));
                msgStr.append(";");
                msgStr.append(waitingTasks.at(wti).requiredResourcesString);
                msgStr.append(";");
                msgStr.append(QString::number(waitingTasks.at(wti).encounteringTime));


                // set the startHandlingTime
                //waitingTasks[wti].startHandlingTime = QDateTime::currentDateTime().toTime_t();
                // add the task to handlingTasks
                //handlingTasks.append(waitingTasks.at(wti));

                waitingTasks[wti].status = TS_SUCCORING;
                waitingTasks[wti].responsibleUnit = coalList.at(coalID).coalLeaderID;

                pubTaskInfo2Monitor(taskProp(waitingTasks.at(wti)));

                // remove the task from waitingTasks
                //waitingTasks.remove(wti);
            }

        }


        msg.message.push_back(msgStr.toStdString());

    }

    if(targetPosesStr.size() > 0){
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

// publish task info to monitoringISLH
void RosThread::pubTaskInfo2Monitor(taskProp task)
{    
    ISLH_msgs::taskInfo2MonitorMessage taskInfoMsg;

    taskInfoMsg.encounteringRobotID = task.encounteringRobotID;
    taskInfoMsg.encounteringTime = task.encounteringTime;
    taskInfoMsg.handlingDuration = task.handlingDuration;
    taskInfoMsg.posXY.x = task.pose.X;
    taskInfoMsg.posXY.y = task.pose.Y;
    taskInfoMsg.responsibleUnit = task.responsibleUnit;
    taskInfoMsg.startHandlingTime = task.startHandlingTime;
    taskInfoMsg.status = task.status;
    taskInfoMsg.taskResource = task.requiredResourcesString.toStdString();
    taskInfoMsg.taskUUID = task.taskUUID.toStdString();
    taskInfoMsg.timeOutDuration = task.timeOutDuration;
    taskInfoMsg.taskSiteRadius = task.taskSiteRadius;

    taskInfo2MonitorPub.publish(taskInfoMsg);
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
        queueSize = result["queueSize"].toInt();
        qDebug()<< " queueSize " << queueSize;

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

        missionParams.targetSiteRadius = result["targetSiteRadius"].toDouble();
        qDebug()<< " targetSiteRadius " << missionParams.targetSiteRadius;

        missionParams.taskCheckingPeriod = result["taskCheckingPeriod"].toDouble();
        qDebug()<< " taskCheckingPeriod " << missionParams.taskCheckingPeriod;

        coordinatorRobotID = result["taskCoordinatorRobotID"].toInt();
        qDebug()<< " coordinatorRobotID " << coordinatorRobotID;

        distThreshold4GeneratingPoses = result["distanceThreshold4GeneratingPoses"].toDouble();
        qDebug()<< " distThreshold4GeneratingPoses " << distThreshold4GeneratingPoses;

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
            //rNameStr.remove("IRobot");
            int rID = rNameStr.toInt();

            QString resourceStr =  plugin.toMap()["resources"].toString();
            QStringList resourceStrList = resourceStr.split(",",QString::SkipEmptyParts);
            QVector <double> resources;
            for(int i = 0; i < resourceStrList.size();i++)
               resources.append(resourceStrList.at(i).toDouble());

            robotTmp.coalID = rID-1;
            robotTmp.robotID = rID;
            robotTmp.resources = resources;

            robotTmp.inGoalPose = -1;
            robotTmp.inTaskSite = -1;

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
