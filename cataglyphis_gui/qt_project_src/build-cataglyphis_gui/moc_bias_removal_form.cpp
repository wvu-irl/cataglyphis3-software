/****************************************************************************
** Meta object code from reading C++ file 'bias_removal_form.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../cataglyphis_gui/bias_removal_form.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'bias_removal_form.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_bias_removal_form_t {
    QByteArrayData data[9];
    char stringdata0[209];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_bias_removal_form_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_bias_removal_form_t qt_meta_stringdata_bias_removal_form = {
    {
QT_MOC_LITERAL(0, 0, 17), // "bias_removal_form"
QT_MOC_LITERAL(1, 18, 18), // "start_bias_removal"
QT_MOC_LITERAL(2, 37, 0), // ""
QT_MOC_LITERAL(3, 38, 21), // "bias_removal_finished"
QT_MOC_LITERAL(4, 60, 27), // "update_bias_removal_display"
QT_MOC_LITERAL(5, 88, 26), // "messages::NavFilterControl"
QT_MOC_LITERAL(6, 115, 15), // "serviceResponse"
QT_MOC_LITERAL(7, 131, 38), // "on_begin_dead_reckoning_butto..."
QT_MOC_LITERAL(8, 170, 38) // "on_perform_bias_removal_butto..."

    },
    "bias_removal_form\0start_bias_removal\0"
    "\0bias_removal_finished\0"
    "update_bias_removal_display\0"
    "messages::NavFilterControl\0serviceResponse\0"
    "on_begin_dead_reckoning_button_clicked\0"
    "on_perform_bias_removal_button_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_bias_removal_form[] = {

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
       1,    0,   39,    2, 0x06 /* Public */,
       3,    0,   40,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       4,    1,   41,    2, 0x0a /* Public */,
       7,    0,   44,    2, 0x08 /* Private */,
       8,    0,   45,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 5,    6,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void bias_removal_form::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        bias_removal_form *_t = static_cast<bias_removal_form *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->start_bias_removal(); break;
        case 1: _t->bias_removal_finished(); break;
        case 2: _t->update_bias_removal_display((*reinterpret_cast< messages::NavFilterControl(*)>(_a[1]))); break;
        case 3: _t->on_begin_dead_reckoning_button_clicked(); break;
        case 4: _t->on_perform_bias_removal_button_clicked(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (bias_removal_form::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&bias_removal_form::start_bias_removal)) {
                *result = 0;
            }
        }
        {
            typedef void (bias_removal_form::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&bias_removal_form::bias_removal_finished)) {
                *result = 1;
            }
        }
    }
}

const QMetaObject bias_removal_form::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_bias_removal_form.data,
      qt_meta_data_bias_removal_form,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *bias_removal_form::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *bias_removal_form::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_bias_removal_form.stringdata0))
        return static_cast<void*>(const_cast< bias_removal_form*>(this));
    return QWidget::qt_metacast(_clname);
}

int bias_removal_form::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
void bias_removal_form::start_bias_removal()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void bias_removal_form::bias_removal_finished()
{
    QMetaObject::activate(this, &staticMetaObject, 1, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
