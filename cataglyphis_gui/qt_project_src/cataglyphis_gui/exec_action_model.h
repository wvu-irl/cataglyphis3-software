#ifndef EXEC_ACTION_MODEL_H
#define EXEC_ACTION_MODEL_H

#include <QObject>
#include <QStandardItemModel>

class exec_action_model : public QStandardItemModel
{
public:
    exec_action_model() {}
    exec_action_model(int rows, int columns, QObject * parent = 0):
            QStandardItemModel(rows, columns, parent){}

    void addExecAction();
    void clearTable(){this->clear();};
};

#endif // EXEC_ACTION_MODEL_H
