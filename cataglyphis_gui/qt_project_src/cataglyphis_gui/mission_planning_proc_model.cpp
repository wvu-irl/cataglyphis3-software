/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2016, WVU Interactive Robotics Laboratory
*                       https://web.statler.wvu.edu/~irl/
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "mission_planning_proc_model.h"

mission_planning_proc_model::mission_planning_proc_model(QObject *parent):
    QStandardItemModel(parent)
{
    setupTable();
}

mission_planning_proc_model::mission_planning_proc_model(int row, int column, QObject *parent):
    QStandardItemModel(row, column, parent)
{
    setupTable();
}

void mission_planning_proc_model::addAllColumns()
{
    once = true;
    addProcStateColumn(&lastMissionInfo);
    addProcBoolColumn(&lastMissionInfo.procsBeingExecuted,mission_planning_enums::executing+1);
    addProcBoolColumn(&lastMissionInfo.procsToExecute,mission_planning_enums::to_be_executed+1);
    addProcBoolColumn(&lastMissionInfo.procsToInterrupt,mission_planning_enums::interrupted+1);
    addProcBoolColumn(&lastMissionInfo.procsToResume,mission_planning_enums::resume+1);
}

void mission_planning_proc_model::on_confirm_changes()
{

}

void mission_planning_proc_model::on_discard_changes()
{

}

void mission_planning_proc_model::on_mission_planning_info_callback(const messages::MissionPlanningInfo info)
{
    ROS_DEBUG("mission planning callback");
    lastMissionInfo = info;
    refreshTable();
}

void mission_planning_proc_model::addProcBoolColumn(std::vector<uint8_t> *data, int column/*mission_planning_enums::PROC_COLUMN_BOOL_T column*/)
{
    for(unsigned int row = 0; row < data->size(); row++)
    {
        QModelIndex index = this->index(row, column, QModelIndex());
//        ROS_WARN("info- procstates %d", info->procStates.at(row));
        this->setData(index, data->at(row)!=0, QT_MISSION_DATA_ROLE);
    }
}

void mission_planning_proc_model::addProcStateColumn(messages::MissionPlanningInfo *info)
{
    //ROS_WARN("Adding new states");
    for(unsigned int row = 0; row < info->procStates.size(); row++)
    {
        QModelIndex index = this->index(row, 0, QModelIndex());
//        ROS_WARN("info- procstates %d", info->procStates.at(row));
        this->setData(index, info->procStates.at(row), QT_MISSION_DATA_ROLE);
    }
}

std::vector<uint8_t> mission_planning_proc_model::formBoolVectorFromColumn(int column)
{
    std::vector<uint8_t> returnVec(lastMissionInfo.procsToExecute.size());
    for(int row = 0; hasIndex(row, column); row++)
    {
        returnVec.push_back(0);
    }
    for(int row = 0; hasIndex(row, column); row++)
    {
        returnVec.at(row)=(index(row,column).data(QT_MISSION_DATA_ROLE)!=0);
    }
    return returnVec;
}

std::vector<int32_t> mission_planning_proc_model::formIntVectorFromColumn(int column)
{
    std::vector<int32_t> returnVec(lastMissionInfo.procStates.size());
    for(int row = 0; hasIndex(row, column); row++)
    {
        returnVec.push_back(0);
    }
    for(int row = 0; hasIndex(row, column); row++)
    {
        returnVec.at(row) = (index(row,column).data(QT_MISSION_DATA_ROLE)!=0);
    }
    return returnVec;
}

void mission_planning_proc_model::setReadOnly(bool readOnly)
{
    int column = 0;
    for(int row = 0; hasIndex(row, column); row++)
    {
        for(column = 0; hasIndex(row, column); column++)
        {
            this->setData(index(row,column, QModelIndex()), readOnly, QT_READ_ONLY_ROLE);
        }
        column = 0;
    }
}
