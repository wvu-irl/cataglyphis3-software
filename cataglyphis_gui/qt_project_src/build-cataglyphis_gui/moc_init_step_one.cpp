/****************************************************************************
** Meta object code from reading C++ file 'init_step_one.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../cataglyphis_gui/init_step_one.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'init_step_one.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_init_step_one_t {
    QByteArrayData data[10];
    char stringdata0[170];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_init_step_one_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_init_step_one_t qt_meta_stringdata_init_step_one = {
    {
QT_MOC_LITERAL(0, 0, 13), // "init_step_one"
QT_MOC_LITERAL(1, 14, 15), // "init_nav_filter"
QT_MOC_LITERAL(2, 30, 0), // ""
QT_MOC_LITERAL(3, 31, 26), // "messages::NavFilterControl"
QT_MOC_LITERAL(4, 58, 7), // "navInit"
QT_MOC_LITERAL(5, 66, 17), // "step_one_finished"
QT_MOC_LITERAL(6, 84, 20), // "when_nav_init_return"
QT_MOC_LITERAL(7, 105, 9), // "sucessful"
QT_MOC_LITERAL(8, 115, 27), // "on_skip_init_button_clicked"
QT_MOC_LITERAL(9, 143, 26) // "on_continue_button_clicked"

    },
    "init_step_one\0init_nav_filter\0\0"
    "messages::NavFilterControl\0navInit\0"
    "step_one_finished\0when_nav_init_return\0"
    "sucessful\0on_skip_init_button_clicked\0"
    "on_continue_button_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_init_step_one[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   39,    2, 0x06 /* Public */,
       5,    0,   42,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       6,    1,   43,    2, 0x0a /* Public */,
       8,    0,   46,    2, 0x08 /* Private */,
       9,    0,   47,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,    7,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void init_step_one::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        init_step_one *_t = static_cast<init_step_one *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->init_nav_filter((*reinterpret_cast< const messages::NavFilterControl(*)>(_a[1]))); break;
        case 1: _t->step_one_finished(); break;
        case 2: _t->when_nav_init_return((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->on_skip_init_button_clicked(); break;
        case 4: _t->on_continue_button_clicked(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (init_step_one::*_t)(const messages::NavFilterControl );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&init_step_one::init_nav_filter)) {
                *result = 0;
            }
        }
        {
            typedef void (init_step_one::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&init_step_one::step_one_finished)) {
                *result = 1;
            }
        }
    }
}

const QMetaObject init_step_one::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_init_step_one.data,
      qt_meta_data_init_step_one,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *init_step_one::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *init_step_one::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_init_step_one.stringdata0))
        return static_cast<void*>(const_cast< init_step_one*>(this));
    return QWidget::qt_metacast(_clname);
}

int init_step_one::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void init_step_one::init_nav_filter(const messages::NavFilterControl _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void init_step_one::step_one_finished()
{
    QMetaObject::activate(this, &staticMetaObject, 1, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
