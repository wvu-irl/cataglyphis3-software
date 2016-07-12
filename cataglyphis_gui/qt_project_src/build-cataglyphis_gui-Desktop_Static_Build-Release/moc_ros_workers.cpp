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
    QByteArrayData data[20];
    char stringdata0[385];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ros_workers_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ros_workers_t qt_meta_stringdata_ros_workers = {
    {
QT_MOC_LITERAL(0, 0, 11), // "ros_workers"
QT_MOC_LITERAL(1, 12, 20), // "nav_service_returned"
QT_MOC_LITERAL(2, 33, 0), // ""
QT_MOC_LITERAL(3, 34, 26), // "messages::NavFilterControl"
QT_MOC_LITERAL(4, 61, 11), // "navResponse"
QT_MOC_LITERAL(5, 73, 12), // "wasSucessful"
QT_MOC_LITERAL(6, 86, 8), // "callerID"
QT_MOC_LITERAL(7, 95, 17), // "nav_init_returned"
QT_MOC_LITERAL(8, 113, 21), // "bias_removal_returned"
QT_MOC_LITERAL(9, 135, 31), // "dead_reckoning_service_returned"
QT_MOC_LITERAL(10, 167, 17), // "nav_info_callback"
QT_MOC_LITERAL(11, 185, 22), // "messages::NavFilterOut"
QT_MOC_LITERAL(12, 208, 7), // "navInfo"
QT_MOC_LITERAL(13, 216, 15), // "run_nav_service"
QT_MOC_LITERAL(14, 232, 14), // "serviceRequest"
QT_MOC_LITERAL(15, 247, 24), // "run_bias_removal_service"
QT_MOC_LITERAL(16, 272, 32), // "run_start_dead_reckoning_service"
QT_MOC_LITERAL(17, 305, 20), // "run_nav_init_service"
QT_MOC_LITERAL(18, 326, 29), // "run_nav_info_subscriber_start"
QT_MOC_LITERAL(19, 356, 28) // "run_nav_info_subscriber_stop"

    },
    "ros_workers\0nav_service_returned\0\0"
    "messages::NavFilterControl\0navResponse\0"
    "wasSucessful\0callerID\0nav_init_returned\0"
    "bias_removal_returned\0"
    "dead_reckoning_service_returned\0"
    "nav_info_callback\0messages::NavFilterOut\0"
    "navInfo\0run_nav_service\0serviceRequest\0"
    "run_bias_removal_service\0"
    "run_start_dead_reckoning_service\0"
    "run_nav_init_service\0run_nav_info_subscriber_start\0"
    "run_nav_info_subscriber_stop"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ros_workers[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    3,   69,    2, 0x06 /* Public */,
       7,    2,   76,    2, 0x06 /* Public */,
       8,    2,   81,    2, 0x06 /* Public */,
       9,    2,   86,    2, 0x06 /* Public */,
      10,    1,   91,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      13,    2,   94,    2, 0x0a /* Public */,
      15,    0,   99,    2, 0x0a /* Public */,
      16,    0,  100,    2, 0x0a /* Public */,
      17,    1,  101,    2, 0x0a /* Public */,
      18,    0,  104,    2, 0x0a /* Public */,
      19,    0,  105,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3, QMetaType::Bool, QMetaType::Int,    4,    5,    6,
    QMetaType::Void, 0x80000000 | 3, QMetaType::Bool,    4,    5,
    QMetaType::Void, 0x80000000 | 3, QMetaType::Bool,    4,    5,
    QMetaType::Void, 0x80000000 | 3, QMetaType::Bool,    4,    5,
    QMetaType::Void, 0x80000000 | 11,   12,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3, QMetaType::Int,   14,    6,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 3,   14,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void ros_workers::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ros_workers *_t = static_cast<ros_workers *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->nav_service_returned((*reinterpret_cast< const messages::NavFilterControl(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2])),(*reinterpret_cast< const int(*)>(_a[3]))); break;
        case 1: _t->nav_init_returned((*reinterpret_cast< const messages::NavFilterControl(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 2: _t->bias_removal_returned((*reinterpret_cast< const messages::NavFilterControl(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 3: _t->dead_reckoning_service_returned((*reinterpret_cast< const messages::NavFilterControl(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 4: _t->nav_info_callback((*reinterpret_cast< const messages::NavFilterOut(*)>(_a[1]))); break;
        case 5: _t->run_nav_service((*reinterpret_cast< messages::NavFilterControl(*)>(_a[1])),(*reinterpret_cast< const int(*)>(_a[2]))); break;
        case 6: _t->run_bias_removal_service(); break;
        case 7: _t->run_start_dead_reckoning_service(); break;
        case 8: _t->run_nav_init_service((*reinterpret_cast< messages::NavFilterControl(*)>(_a[1]))); break;
        case 9: _t->run_nav_info_subscriber_start(); break;
        case 10: _t->run_nav_info_subscriber_stop(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (ros_workers::*_t)(const messages::NavFilterControl , bool , const int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ros_workers::nav_service_returned)) {
                *result = 0;
            }
        }
        {
            typedef void (ros_workers::*_t)(const messages::NavFilterControl , bool );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ros_workers::nav_init_returned)) {
                *result = 1;
            }
        }
        {
            typedef void (ros_workers::*_t)(const messages::NavFilterControl , bool );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ros_workers::bias_removal_returned)) {
                *result = 2;
            }
        }
        {
            typedef void (ros_workers::*_t)(const messages::NavFilterControl , bool );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ros_workers::dead_reckoning_service_returned)) {
                *result = 3;
            }
        }
        {
            typedef void (ros_workers::*_t)(const messages::NavFilterOut );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ros_workers::nav_info_callback)) {
                *result = 4;
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
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 11)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 11;
    }
    return _id;
}

// SIGNAL 0
void ros_workers::nav_service_returned(const messages::NavFilterControl _t1, bool _t2, const int _t3)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void ros_workers::nav_init_returned(const messages::NavFilterControl _t1, bool _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void ros_workers::bias_removal_returned(const messages::NavFilterControl _t1, bool _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void ros_workers::dead_reckoning_service_returned(const messages::NavFilterControl _t1, bool _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void ros_workers::nav_info_callback(const messages::NavFilterOut _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}
QT_END_MOC_NAMESPACE
