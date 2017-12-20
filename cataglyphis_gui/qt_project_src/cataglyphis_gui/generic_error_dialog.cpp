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

#include "generic_error_dialog.h"
#include "ui_generic_error_dialog_form.h"

#define OBJ_FUNCTION_CALL
#ifdef OBJ_FUNCTION_CALL
#define FUNCTION_CALL(_obj, args...) _obj->setupUi(args)
#else
#define FUNCTION_CALL(_obj)
#endif

generic_error_dialog::generic_error_dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::generic_error_dialog_form)
{
    FUNCTION_CALL(ui, this);
}

generic_error_dialog::generic_error_dialog(const QString &header, const QString &footer, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::generic_error_dialog_form)
{
    ui->setupUi(this);
    ui->header->setPlainText(header);
    ui->footer->setPlainText(footer);
    this->exec();
}

void generic_error_dialog::setHeaderText(const QString &headerText)
{
    ui->header->setPlainText(headerText);
}

void generic_error_dialog::setBodyText(const QString &bodyText)
{
    ui->footer->setPlainText(bodyText);
}

generic_error_dialog::~generic_error_dialog()
{
    delete ui;
}
