/****************************************************************************
** Meta object code from reading C++ file 'cataglyphis_startup_form_main.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../cataglyphis_gui/cataglyphis_startup_form_main.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'cataglyphis_startup_form_main.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_cataglyphis_startup_form_main_t {
    QByteArrayData data[8];
    char stringdata0[165];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_cataglyphis_startup_form_main_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_cataglyphis_startup_form_main_t qt_meta_stringdata_cataglyphis_startup_form_main = {
    {
QT_MOC_LITERAL(0, 0, 29), // "cataglyphis_startup_form_main"
QT_MOC_LITERAL(1, 30, 17), // "step_one_returned"
QT_MOC_LITERAL(2, 48, 0), // ""
QT_MOC_LITERAL(3, 49, 17), // "step_two_returned"
QT_MOC_LITERAL(4, 67, 26), // "on_start_up_button_clicked"
QT_MOC_LITERAL(5, 94, 33), // "on_reboot_recovery_button_cli..."
QT_MOC_LITERAL(6, 128, 30), // "on_input_tabber_currentChanged"
QT_MOC_LITERAL(7, 159, 5) // "index"

    },
    "cataglyphis_startup_form_main\0"
    "step_one_returned\0\0step_two_returned\0"
    "on_start_up_button_clicked\0"
    "on_reboot_recovery_button_clicked\0"
    "on_input_tabber_currentChanged\0index"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_cataglyphis_startup_form_main[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   39,    2, 0x0a /* Public */,
       3,    0,   40,    2, 0x0a /* Public */,
       4,    0,   41,    2, 0x08 /* Private */,
       5,    0,   42,    2, 0x08 /* Private */,
       6,    1,   43,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    7,

       0        // eod
};

void cataglyphis_startup_form_main::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        cataglyphis_startup_form_main *_t = static_cast<cataglyphis_startup_form_main *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->step_one_returned(); break;
        case 1: _t->step_two_returned(); break;
        case 2: _t->on_start_up_button_clicked(); break;
        case 3: _t->on_reboot_recovery_button_clicked(); break;
        case 4: _t->on_input_tabber_currentChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject cataglyphis_startup_form_main::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_cataglyphis_startup_form_main.data,
      qt_meta_data_cataglyphis_startup_form_main,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *cataglyphis_startup_form_main::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *cataglyphis_startup_form_main::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_cataglyphis_startup_form_main.stringdata0))
        return static_cast<void*>(const_cast< cataglyphis_startup_form_main*>(this));
    return QWidget::qt_metacast(_clname);
}

int cataglyphis_startup_form_main::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
QT_END_MOC_NAMESPACE
