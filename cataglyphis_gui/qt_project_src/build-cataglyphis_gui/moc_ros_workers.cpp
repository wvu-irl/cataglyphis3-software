/****************************************************************************
** Meta object code from reading C++ file 'ros_workers.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../cataglyphis_gui/ros_workers.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ros_workers.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_ros_workers_t {
    QByteArrayData data[10];
    char stringdata0[163];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ros_workers_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ros_workers_t qt_meta_stringdata_ros_workers = {
    {
QT_MOC_LITERAL(0, 0, 11), // "ros_workers"
QT_MOC_LITERAL(1, 12, 17), // "nav_init_returned"
QT_MOC_LITERAL(2, 30, 0), // ""
QT_MOC_LITERAL(3, 31, 9), // "sucessful"
QT_MOC_LITERAL(4, 41, 21), // "bias_removal_returned"
QT_MOC_LITERAL(5, 63, 26), // "messages::NavFilterControl"
QT_MOC_LITERAL(6, 90, 11), // "navResponse"
QT_MOC_LITERAL(7, 102, 24), // "run_bias_removal_service"
QT_MOC_LITERAL(8, 127, 20), // "run_nav_init_service"
QT_MOC_LITERAL(9, 148, 14) // "serviceRequest"

    },
    "ros_workers\0nav_init_returned\0\0sucessful\0"
    "bias_removal_returned\0messages::NavFilterControl\0"
    "navResponse\0run_bias_removal_service\0"
    "run_nav_init_service\0serviceRequest"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ros_workers[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x06 /* Public */,
       4,    1,   37,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       7,    0,   40,    2, 0x0a /* Public */,
       8,    1,   41,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, 0x80000000 | 5,    6,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 5,    9,

       0        // eod
};

void ros_workers::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ros_workers *_t = static_cast<ros_workers *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->nav_init_returned((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->bias_removal_returned((*reinterpret_cast< const messages::NavFilterControl(*)>(_a[1]))); break;
        case 2: _t->run_bias_removal_service(); break;
        case 3: _t->run_nav_init_service((*reinterpret_cast< const messages::NavFilterControl(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (ros_workers::*_t)(bool );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ros_workers::nav_init_returned)) {
                *result = 0;
            }
        }
        {
            typedef void (ros_workers::*_t)(const messages::NavFilterControl );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ros_workers::bias_removal_returned)) {
                *result = 1;
            }
        }
    }
}

const QMetaObject ros_workers::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_ros_workers.data,
      qt_meta_data_ros_workers,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *ros_workers::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ros_workers::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_ros_workers.stringdata0))
        return static_cast<void*>(const_cast< ros_workers*>(this));
    return QObject::qt_metacast(_clname);
}

int ros_workers::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void ros_workers::nav_init_returned(bool _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void ros_workers::bias_removal_returned(const messages::NavFilterControl _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_END_MOC_NAMESPACE
