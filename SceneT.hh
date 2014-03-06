#ifndef SCENET_HH
#define SCENET_HH

#include <QGraphicsScene>
#include <QLabel>
#include <QTime>
#include <QtOpenGL>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include "QtModelT.hh"
#include <OpenMesh/Core/IO/Options.hh>
#include <OpenMesh/Core/Utils/GenProg.hh>
#include <OpenMesh/Core/Utils/color_cast.hh>
#include <OpenMesh/Core/Mesh/Attributes.hh>
#include <nanoflann.hpp>


using namespace Qt;
using namespace OpenMesh;
using namespace Eigen;

template <typename M>
class SceneT : public QGraphicsScene
{
public:
  typedef M MyMesh;
  std::vector<QtModelT<M>*> models;

public:
    SceneT();
    void drawBackground(QPainter *painter, const QRectF &rect);
    void drawForeground(QPainter *painter, const QRectF &rect);

protected:
    void runICPglobalMeshes(int iterations, bool sampling);
    void loadMesh(const QString filePath);
    void applyTransformations();
    void runICP(int iterations);
    void runTwoMesh(const QString m1, const QString m2);
    void runNoiseICP();
    void runSamplingICP();
    void loadGlobal(const QString m1, const QString m2, const QString m3);

protected:
    QWidget *m_modelButton;
    QWidget *m_applyButton;
    QWidget *m_icpButton;
    QWidget *m_icpButton5;
    QWidget *m_icpButton10;
    QWidget *m_icpButton50;
    QWidget *m_icpButton100;
    QWidget *m_icpButtonSampling;
    QWidget *m_icpButtonNoise;
    QWidget *m_twoMeshButton;
    QWidget *m_sameMeshButton;
    QWidget *m_globalButton;
    void wheelEvent(QGraphicsSceneWheelEvent * wheelEvent);
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    void keyPressEvent( QKeyEvent* event);
    bool runICPTwoMeshes(QtModelT<M>* m1, QtModelT<M>*  m2, int iterations);
    bool samplingICP(QtModelT<M>* m1, QtModelT<M>*  m2, int iterations);

private:
    QDialog *createDialog(const QString &windowTitle) const;
    int whichRadioButton();

    MyMesh m_mymesh;
    QColor m_backgroundColor;
    OpenMesh::IO::Options _options;

    QTime m_time;
    int m_mouseEventTime;

    float m_distance;
    float m_vertical;
    float m_horizontal;
    float radius;
    QVector3D m_rotation;

    QGraphicsRectItem *m_lightItem;
    QGroupBox* groupBox;
    QRadioButton* radio1;
    QRadioButton* radio2;
    QRadioButton* radio3;
    QRadioButton* radio4;
    QRadioButton* radio5;
    QRadioButton* radio6;
    const float TANSLATE_SPEED;
    double getMedian(Matrix<double, Dynamic, 1> mat);
    void getBaryCenteredPoints(PointMatrix &matHat, const PointMatrix &mat);
    void generateA(Matrix<double, 3, 3>  &A, const PointMatrix &m1Hat, const PointMatrix &m2Hat);
    void updateColour();

};

#if defined(OM_INCLUDE_TEMPLATES) && !defined(SCENE_CC)
#  define SCENE_TEMPLATES
#  include "SceneT.cc"
#endif

#endif
