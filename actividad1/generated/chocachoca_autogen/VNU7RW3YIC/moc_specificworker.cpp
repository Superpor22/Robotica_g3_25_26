/****************************************************************************
** Meta object code from reading C++ file 'specificworker.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.4.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../src/specificworker.h"
#include <QtGui/qtextcursor.h>
#include <QScreen>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'specificworker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.4.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
namespace {
struct qt_meta_stringdata_SpecificWorker_t {
    uint offsetsAndSizes[36];
    char stringdata0[15];
    char stringdata1[11];
    char stringdata2[1];
    char stringdata3[8];
    char stringdata4[10];
    char stringdata5[8];
    char stringdata6[14];
    char stringdata7[16];
    char stringdata8[11];
    char stringdata9[25];
    char stringdata10[7];
    char stringdata11[16];
    char stringdata12[6];
    char stringdata13[15];
    char stringdata14[30];
    char stringdata15[12];
    char stringdata16[17];
    char stringdata17[21];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_SpecificWorker_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_SpecificWorker_t qt_meta_stringdata_SpecificWorker = {
    {
        QT_MOC_LITERAL(0, 14),  // "SpecificWorker"
        QT_MOC_LITERAL(15, 10),  // "initialize"
        QT_MOC_LITERAL(26, 0),  // ""
        QT_MOC_LITERAL(27, 7),  // "compute"
        QT_MOC_LITERAL(35, 9),  // "emergency"
        QT_MOC_LITERAL(45, 7),  // "restore"
        QT_MOC_LITERAL(53, 13),  // "startup_check"
        QT_MOC_LITERAL(67, 15),  // "new_target_slot"
        QT_MOC_LITERAL(83, 10),  // "draw_lidar"
        QT_MOC_LITERAL(94, 24),  // "RoboCompLidar3D::TPoints"
        QT_MOC_LITERAL(119, 6),  // "points"
        QT_MOC_LITERAL(126, 15),  // "QGraphicsScene*"
        QT_MOC_LITERAL(142, 5),  // "scene"
        QT_MOC_LITERAL(148, 14),  // "FORWARD_method"
        QT_MOC_LITERAL(163, 29),  // "std::tuple<State,float,float>"
        QT_MOC_LITERAL(193, 11),  // "TURN_method"
        QT_MOC_LITERAL(205, 16),  // "get_min_distance"
        QT_MOC_LITERAL(222, 20)   // "std::optional<float>"
    },
    "SpecificWorker",
    "initialize",
    "",
    "compute",
    "emergency",
    "restore",
    "startup_check",
    "new_target_slot",
    "draw_lidar",
    "RoboCompLidar3D::TPoints",
    "points",
    "QGraphicsScene*",
    "scene",
    "FORWARD_method",
    "std::tuple<State,float,float>",
    "TURN_method",
    "get_min_distance",
    "std::optional<float>"
};
#undef QT_MOC_LITERAL
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_SpecificWorker[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,   74,    2, 0x0a,    1 /* Public */,
       3,    0,   75,    2, 0x0a,    2 /* Public */,
       4,    0,   76,    2, 0x0a,    3 /* Public */,
       5,    0,   77,    2, 0x0a,    4 /* Public */,
       6,    0,   78,    2, 0x0a,    5 /* Public */,
       7,    1,   79,    2, 0x0a,    6 /* Public */,
       8,    2,   82,    2, 0x0a,    8 /* Public */,
      13,    0,   87,    2, 0x0a,   11 /* Public */,
      15,    0,   88,    2, 0x0a,   12 /* Public */,
      16,    1,   89,    2, 0x0a,   13 /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Int,
    QMetaType::Void, QMetaType::QPointF,    2,
    QMetaType::Void, 0x80000000 | 9, 0x80000000 | 11,   10,   12,
    0x80000000 | 14,
    0x80000000 | 14,
    0x80000000 | 17, 0x80000000 | 9,   10,

       0        // eod
};

Q_CONSTINIT const QMetaObject SpecificWorker::staticMetaObject = { {
    QMetaObject::SuperData::link<GenericWorker::staticMetaObject>(),
    qt_meta_stringdata_SpecificWorker.offsetsAndSizes,
    qt_meta_data_SpecificWorker,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_SpecificWorker_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<SpecificWorker, std::true_type>,
        // method 'initialize'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'compute'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'emergency'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'restore'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'startup_check'
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'new_target_slot'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<QPointF, std::false_type>,
        // method 'draw_lidar'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const RoboCompLidar3D::TPoints &, std::false_type>,
        QtPrivate::TypeAndForceComplete<QGraphicsScene *, std::false_type>,
        // method 'FORWARD_method'
        QtPrivate::TypeAndForceComplete<std::tuple<State,float,float>, std::false_type>,
        // method 'TURN_method'
        QtPrivate::TypeAndForceComplete<std::tuple<State,float,float>, std::false_type>,
        // method 'get_min_distance'
        QtPrivate::TypeAndForceComplete<std::optional<float>, std::false_type>,
        QtPrivate::TypeAndForceComplete<const RoboCompLidar3D::TPoints &, std::false_type>
    >,
    nullptr
} };

void SpecificWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<SpecificWorker *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->initialize(); break;
        case 1: _t->compute(); break;
        case 2: _t->emergency(); break;
        case 3: _t->restore(); break;
        case 4: { int _r = _t->startup_check();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = std::move(_r); }  break;
        case 5: _t->new_target_slot((*reinterpret_cast< std::add_pointer_t<QPointF>>(_a[1]))); break;
        case 6: _t->draw_lidar((*reinterpret_cast< std::add_pointer_t<RoboCompLidar3D::TPoints>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QGraphicsScene*>>(_a[2]))); break;
        case 7: { std::tuple<State,float,float> _r = _t->FORWARD_method();
            if (_a[0]) *reinterpret_cast< std::tuple<State,float,float>*>(_a[0]) = std::move(_r); }  break;
        case 8: { std::tuple<State,float,float> _r = _t->TURN_method();
            if (_a[0]) *reinterpret_cast< std::tuple<State,float,float>*>(_a[0]) = std::move(_r); }  break;
        case 9: { std::optional<float> _r = _t->get_min_distance((*reinterpret_cast< std::add_pointer_t<RoboCompLidar3D::TPoints>>(_a[1])));
            if (_a[0]) *reinterpret_cast< std::optional<float>*>(_a[0]) = std::move(_r); }  break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
        case 6:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 1:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< QGraphicsScene* >(); break;
            }
            break;
        }
    }
}

const QMetaObject *SpecificWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SpecificWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SpecificWorker.stringdata0))
        return static_cast<void*>(this);
    return GenericWorker::qt_metacast(_clname);
}

int SpecificWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = GenericWorker::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
