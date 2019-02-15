/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../src/mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[58];
    char stringdata0[903];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 7), // "getfile"
QT_MOC_LITERAL(2, 19, 0), // ""
QT_MOC_LITERAL(3, 20, 14), // "window_initial"
QT_MOC_LITERAL(4, 35, 37), // "Rectangular2SphericalCoordina..."
QT_MOC_LITERAL(5, 73, 13), // "vector<float>"
QT_MOC_LITERAL(6, 87, 1), // "x"
QT_MOC_LITERAL(7, 89, 1), // "y"
QT_MOC_LITERAL(8, 91, 1), // "z"
QT_MOC_LITERAL(9, 93, 37), // "Spherical2RectangularCoordina..."
QT_MOC_LITERAL(10, 131, 6), // "radius"
QT_MOC_LITERAL(11, 138, 5), // "theta"
QT_MOC_LITERAL(12, 144, 3), // "phi"
QT_MOC_LITERAL(13, 148, 12), // "point_length"
QT_MOC_LITERAL(14, 161, 19), // "getpointcloudcenter"
QT_MOC_LITERAL(15, 181, 14), // "vector<double>"
QT_MOC_LITERAL(16, 196, 35), // "pcl::PointCloud<pcl::PointXYZ..."
QT_MOC_LITERAL(17, 232, 5), // "cloud"
QT_MOC_LITERAL(18, 238, 20), // "initial_cloud_object"
QT_MOC_LITERAL(19, 259, 12), // "cloud_object"
QT_MOC_LITERAL(20, 272, 13), // "circle_sample"
QT_MOC_LITERAL(21, 286, 5), // "long*"
QT_MOC_LITERAL(22, 292, 10), // "judge_find"
QT_MOC_LITERAL(23, 303, 16), // "std::vector<int>"
QT_MOC_LITERAL(24, 320, 7), // "indices"
QT_MOC_LITERAL(25, 328, 5), // "value"
QT_MOC_LITERAL(26, 334, 18), // "getpointcloudbound"
QT_MOC_LITERAL(27, 353, 18), // "pointCloud_cluster"
QT_MOC_LITERAL(28, 372, 20), // "vector<cloud_object>"
QT_MOC_LITERAL(29, 393, 27), // "statistical_outlier_removal"
QT_MOC_LITERAL(30, 421, 20), // "PointcloudDownsample"
QT_MOC_LITERAL(31, 442, 24), // "pointcloudheadingcorrect"
QT_MOC_LITERAL(32, 467, 19), // "cloud_target_screen"
QT_MOC_LITERAL(33, 487, 17), // "cloud_radius_crop"
QT_MOC_LITERAL(34, 505, 13), // "pcl::PointXYZ"
QT_MOC_LITERAL(35, 519, 11), // "searchPoint"
QT_MOC_LITERAL(36, 531, 7), // "PCL_ICP"
QT_MOC_LITERAL(37, 539, 8), // "cloud_in"
QT_MOC_LITERAL(38, 548, 9), // "cloud_out"
QT_MOC_LITERAL(39, 558, 39), // "target_pointcloud_direction_s..."
QT_MOC_LITERAL(40, 598, 11), // "cloud_model"
QT_MOC_LITERAL(41, 610, 17), // "pointcloud_vector"
QT_MOC_LITERAL(42, 628, 1), // "i"
QT_MOC_LITERAL(43, 630, 9), // "add_cloud"
QT_MOC_LITERAL(44, 640, 5), // "isrun"
QT_MOC_LITERAL(45, 646, 17), // "pclcloud2polydata"
QT_MOC_LITERAL(46, 664, 28), // "vtkSmartPointer<vtkPolyData>"
QT_MOC_LITERAL(47, 693, 4), // "test"
QT_MOC_LITERAL(48, 698, 25), // "pointCloud_cluster_simple"
QT_MOC_LITERAL(49, 724, 26), // "get_cloud_object_direction"
QT_MOC_LITERAL(50, 751, 6), // "PointT"
QT_MOC_LITERAL(51, 758, 2), // "co"
QT_MOC_LITERAL(52, 761, 31), // "targetcloud_waitfortrack_update"
QT_MOC_LITERAL(53, 793, 24), // "targetcloud_waitfortrack"
QT_MOC_LITERAL(54, 818, 28), // "targetcloud_waitfortrack_new"
QT_MOC_LITERAL(55, 847, 19), // "pointcloud_tansform"
QT_MOC_LITERAL(56, 867, 28), // "pcl::PointCloud<PointT>::Ptr"
QT_MOC_LITERAL(57, 896, 6) // "theta_"

    },
    "MainWindow\0getfile\0\0window_initial\0"
    "Rectangular2SphericalCoordinateSystem\0"
    "vector<float>\0x\0y\0z\0"
    "Spherical2RectangularCoordinateSystem\0"
    "radius\0theta\0phi\0point_length\0"
    "getpointcloudcenter\0vector<double>\0"
    "pcl::PointCloud<pcl::PointXYZ>::Ptr\0"
    "cloud\0initial_cloud_object\0cloud_object\0"
    "circle_sample\0long*\0judge_find\0"
    "std::vector<int>\0indices\0value\0"
    "getpointcloudbound\0pointCloud_cluster\0"
    "vector<cloud_object>\0statistical_outlier_removal\0"
    "PointcloudDownsample\0pointcloudheadingcorrect\0"
    "cloud_target_screen\0cloud_radius_crop\0"
    "pcl::PointXYZ\0searchPoint\0PCL_ICP\0"
    "cloud_in\0cloud_out\0"
    "target_pointcloud_direction_singlecloud\0"
    "cloud_model\0pointcloud_vector\0i\0"
    "add_cloud\0isrun\0pclcloud2polydata\0"
    "vtkSmartPointer<vtkPolyData>\0test\0"
    "pointCloud_cluster_simple\0"
    "get_cloud_object_direction\0PointT\0co\0"
    "targetcloud_waitfortrack_update\0"
    "targetcloud_waitfortrack\0"
    "targetcloud_waitfortrack_new\0"
    "pointcloud_tansform\0pcl::PointCloud<PointT>::Ptr\0"
    "theta_"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      25,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,  139,    2, 0x08 /* Private */,
       3,    0,  140,    2, 0x08 /* Private */,
       4,    3,  141,    2, 0x08 /* Private */,
       9,    3,  148,    2, 0x08 /* Private */,
      13,    3,  155,    2, 0x08 /* Private */,
      14,    1,  162,    2, 0x08 /* Private */,
      18,    1,  165,    2, 0x08 /* Private */,
      20,    1,  168,    2, 0x08 /* Private */,
      22,    2,  171,    2, 0x08 /* Private */,
      26,    1,  176,    2, 0x08 /* Private */,
      27,    1,  179,    2, 0x08 /* Private */,
      29,    1,  182,    2, 0x08 /* Private */,
      30,    1,  185,    2, 0x08 /* Private */,
      31,    1,  188,    2, 0x08 /* Private */,
      32,    1,  191,    2, 0x08 /* Private */,
      33,    3,  194,    2, 0x08 /* Private */,
      36,    2,  201,    2, 0x08 /* Private */,
      39,    3,  206,    2, 0x08 /* Private */,
      43,    2,  213,    2, 0x08 /* Private */,
      45,    1,  218,    2, 0x08 /* Private */,
      47,    0,  221,    2, 0x08 /* Private */,
      48,    1,  222,    2, 0x08 /* Private */,
      49,    1,  225,    2, 0x08 /* Private */,
      52,    2,  228,    2, 0x08 /* Private */,
      55,    2,  233,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    0x80000000 | 5, QMetaType::Float, QMetaType::Float, QMetaType::Float,    6,    7,    8,
    0x80000000 | 5, QMetaType::Float, QMetaType::Float, QMetaType::Float,   10,   11,   12,
    QMetaType::Float, QMetaType::Float, QMetaType::Float, QMetaType::Float,    6,    7,    8,
    0x80000000 | 15, 0x80000000 | 16,   17,
    0x80000000 | 19, 0x80000000 | 16,   17,
    0x80000000 | 21, 0x80000000 | 16,   17,
    QMetaType::Bool, 0x80000000 | 23, QMetaType::Int,   24,   25,
    0x80000000 | 5, 0x80000000 | 16,   17,
    0x80000000 | 28, 0x80000000 | 16,   17,
    0x80000000 | 16, 0x80000000 | 16,   17,
    0x80000000 | 16, 0x80000000 | 16,   17,
    0x80000000 | 16, 0x80000000 | 16,   17,
    QMetaType::Void, 0x80000000 | 16,   17,
    0x80000000 | 16, 0x80000000 | 16, 0x80000000 | 34, QMetaType::Float,   17,   35,   10,
    0x80000000 | 5, 0x80000000 | 16, 0x80000000 | 16,   37,   38,
    QMetaType::Void, 0x80000000 | 19, 0x80000000 | 28, QMetaType::Int,   40,   41,   42,
    QMetaType::Void, 0x80000000 | 16, QMetaType::Bool,   17,   44,
    0x80000000 | 46, 0x80000000 | 16,   17,
    QMetaType::Void,
    0x80000000 | 28, 0x80000000 | 16,   17,
    0x80000000 | 50, 0x80000000 | 19,   51,
    0x80000000 | 28, 0x80000000 | 28, 0x80000000 | 28,   53,   54,
    0x80000000 | 56, 0x80000000 | 56, QMetaType::Float,   17,   57,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainWindow *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->getfile(); break;
        case 1: _t->window_initial(); break;
        case 2: { vector<float> _r = _t->Rectangular2SphericalCoordinateSystem((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2])),(*reinterpret_cast< float(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< vector<float>*>(_a[0]) = std::move(_r); }  break;
        case 3: { vector<float> _r = _t->Spherical2RectangularCoordinateSystem((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2])),(*reinterpret_cast< float(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< vector<float>*>(_a[0]) = std::move(_r); }  break;
        case 4: { float _r = _t->point_length((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2])),(*reinterpret_cast< float(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< float*>(_a[0]) = std::move(_r); }  break;
        case 5: { vector<double> _r = _t->getpointcloudcenter((*reinterpret_cast< pcl::PointCloud<pcl::PointXYZ>::Ptr(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< vector<double>*>(_a[0]) = std::move(_r); }  break;
        case 6: { cloud_object _r = _t->initial_cloud_object((*reinterpret_cast< pcl::PointCloud<pcl::PointXYZ>::Ptr(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< cloud_object*>(_a[0]) = std::move(_r); }  break;
        case 7: { long* _r = _t->circle_sample((*reinterpret_cast< pcl::PointCloud<pcl::PointXYZ>::Ptr(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< long**>(_a[0]) = std::move(_r); }  break;
        case 8: { bool _r = _t->judge_find((*reinterpret_cast< std::vector<int>(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 9: { vector<float> _r = _t->getpointcloudbound((*reinterpret_cast< pcl::PointCloud<pcl::PointXYZ>::Ptr(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< vector<float>*>(_a[0]) = std::move(_r); }  break;
        case 10: { vector<cloud_object> _r = _t->pointCloud_cluster((*reinterpret_cast< pcl::PointCloud<pcl::PointXYZ>::Ptr(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< vector<cloud_object>*>(_a[0]) = std::move(_r); }  break;
        case 11: { pcl::PointCloud<pcl::PointXYZ>::Ptr _r = _t->statistical_outlier_removal((*reinterpret_cast< pcl::PointCloud<pcl::PointXYZ>::Ptr(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< pcl::PointCloud<pcl::PointXYZ>::Ptr*>(_a[0]) = std::move(_r); }  break;
        case 12: { pcl::PointCloud<pcl::PointXYZ>::Ptr _r = _t->PointcloudDownsample((*reinterpret_cast< pcl::PointCloud<pcl::PointXYZ>::Ptr(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< pcl::PointCloud<pcl::PointXYZ>::Ptr*>(_a[0]) = std::move(_r); }  break;
        case 13: { pcl::PointCloud<pcl::PointXYZ>::Ptr _r = _t->pointcloudheadingcorrect((*reinterpret_cast< pcl::PointCloud<pcl::PointXYZ>::Ptr(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< pcl::PointCloud<pcl::PointXYZ>::Ptr*>(_a[0]) = std::move(_r); }  break;
        case 14: _t->cloud_target_screen((*reinterpret_cast< pcl::PointCloud<pcl::PointXYZ>::Ptr(*)>(_a[1]))); break;
        case 15: { pcl::PointCloud<pcl::PointXYZ>::Ptr _r = _t->cloud_radius_crop((*reinterpret_cast< pcl::PointCloud<pcl::PointXYZ>::Ptr(*)>(_a[1])),(*reinterpret_cast< pcl::PointXYZ(*)>(_a[2])),(*reinterpret_cast< float(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< pcl::PointCloud<pcl::PointXYZ>::Ptr*>(_a[0]) = std::move(_r); }  break;
        case 16: { vector<float> _r = _t->PCL_ICP((*reinterpret_cast< pcl::PointCloud<pcl::PointXYZ>::Ptr(*)>(_a[1])),(*reinterpret_cast< pcl::PointCloud<pcl::PointXYZ>::Ptr(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< vector<float>*>(_a[0]) = std::move(_r); }  break;
        case 17: _t->target_pointcloud_direction_singlecloud((*reinterpret_cast< cloud_object(*)>(_a[1])),(*reinterpret_cast< vector<cloud_object>(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3]))); break;
        case 18: _t->add_cloud((*reinterpret_cast< pcl::PointCloud<pcl::PointXYZ>::Ptr(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 19: { vtkSmartPointer<vtkPolyData> _r = _t->pclcloud2polydata((*reinterpret_cast< pcl::PointCloud<pcl::PointXYZ>::Ptr(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< vtkSmartPointer<vtkPolyData>*>(_a[0]) = std::move(_r); }  break;
        case 20: _t->test(); break;
        case 21: { vector<cloud_object> _r = _t->pointCloud_cluster_simple((*reinterpret_cast< pcl::PointCloud<pcl::PointXYZ>::Ptr(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< vector<cloud_object>*>(_a[0]) = std::move(_r); }  break;
        case 22: { PointT _r = _t->get_cloud_object_direction((*reinterpret_cast< cloud_object(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< PointT*>(_a[0]) = std::move(_r); }  break;
        case 23: { vector<cloud_object> _r = _t->targetcloud_waitfortrack_update((*reinterpret_cast< vector<cloud_object>(*)>(_a[1])),(*reinterpret_cast< vector<cloud_object>(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< vector<cloud_object>*>(_a[0]) = std::move(_r); }  break;
        case 24: { pcl::PointCloud<PointT>::Ptr _r = _t->pointcloud_tansform((*reinterpret_cast< pcl::PointCloud<PointT>::Ptr(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< pcl::PointCloud<PointT>::Ptr*>(_a[0]) = std::move(_r); }  break;
        default: ;
        }
    }
}

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow.data,
      qt_meta_data_MainWindow,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 25)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 25;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 25)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 25;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
