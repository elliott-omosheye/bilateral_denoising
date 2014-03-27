#ifndef MODEL_HH
#define MODEL_HH

#include <iostream>
#include <fstream>
#include <QString>
#include <QVector>
#include <QPainter>
#include <QGraphicsItem>
#include "Scene.hh"
#include "MyMesh.hh"
#include <QVector3D>
#include <eigen3/Eigen/Dense>
#include <nanoflann.hpp>

//#include <flann/io/hdf5.h>

using namespace Qt;
using namespace OpenMesh;
using namespace Eigen;

template <typename M>
class QtModelT : public QGraphicsItem
{
public:
    typedef M MyMesh;
    typedef std::vector<std::vector< std::pair< size_t, double > > > MapTable;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    QRectF boundingRect() const;
    void calcNormals();

public:
     M mesh;
     M groundTruth;

    QtModelT(M& m);
    ~QtModelT();
    void render();
    void updateColour();
    void updateRotation(QVector3D& rotationVec);
    void updateHorizontal(float x);
    void updateVertical(float x);
    void applyTransformations();
    M* getMesh(){ return &mesh; }
    PointMatrix buildMatrix();
    PointMatrix buildSampledMatrix();
    PointMatrix buildNormalMatrix();
    int getNoVerticies();
    void updateTransformations(Matrix<double, 3, 3>& R, double x, double y, double z);
    void addNoise(double sigma);
    void mergeColours(QtModelT<M>* m2);
    void bilateralFiltering(double sigc, double sigs);
    void extendedBilateralFiltering(double sigc, double sigs);
    void nearestNeighbours(double radius, MapTable* resultTable);
    //void renderBackBuffer();
    void getDistFromGroundTruth();
    float gt_distance;
    float FaceNormalErrorCalc();

private:
    QVector3D modelRotation;
    QColor modelColor;
    GLfloat vertical;
    GLfloat horizontal;
    GLfloat depth;
    const float deg2Rad;
    float calcMeshAreaGT();
    float calcMeshArea();
    float faceArea(typename M::ConstFaceVertexIter fvIt);
    float pointFaceDist(typename M::ConstFaceVertexIter f, Point p);
    float sign(float i);
    bool  pointInsideTraingle(Point *a, Point *b, Point *c, Point p);


};

#if defined(OM_INCLUDE_TEMPLATES) && !defined(MODEL_CC)
#  define SCENE_TEMPLATES
#  include "QtModelT.cc"
#endif

#endif
