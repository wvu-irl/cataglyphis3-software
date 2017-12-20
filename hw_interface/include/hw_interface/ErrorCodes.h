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

#ifndef __ErrorCodes_H_
#define __ErrorCodes_H_

#define RQ_INVALID_HANDLE        -1
#define RQ_SUCCESS               0
#define RQ_ERR_OPEN_PORT         1
#define RQ_ERR_NOT_CONNECTED     2
#define RQ_ERR_TRANSMIT_FAILED   3
#define	RQ_ERR_SERIAL_IO         4
#define	RQ_ERR_SERIAL_RECEIVE    5
#define RQ_INVALID_RESPONSE      6
#define RQ_UNRECOGNIZED_DEVICE   7
#define RQ_UNRECOGNIZED_VERSION  8
#define RQ_INVALID_CONFIG_ITEM   9
#define RQ_INVALID_OPER_ITEM     10
#define RQ_INVALID_COMMAND_ITEM  11
#define RQ_INDEX_OUT_RANGE       12
#define RQ_SET_CONFIG_FAILED     13
#define RQ_GET_CONFIG_FAILED     14
#define RQ_GET_VALUE_FAILED      15
#define RQ_SET_COMMAND_FAILED    16

#endif
