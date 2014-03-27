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
    void loadMesh(const QString filePath);

protected:
    QWidget *m_modelButton;
    QWidget *removeModelButton;
    QWidget *m_ex1Button;
    QWidget *m_ex2Button;
    QWidget *m_ex3Button;
    QDoubleSpinBox *noiseSpinBox;
    QWidget *applyNoiseButton;
    QWidget *updateNormalsButton;
    QSpinBox *bilateralFilteringSpinBox;
    QDoubleSpinBox *radiusSpinBox;
    QDoubleSpinBox *standardDeviationSpinBox;
    QWidget *bilateralFilteringButton;
    QWidget *extendedBilateralFilteringButton;
    void wheelEvent(QGraphicsSceneWheelEvent * wheelEvent);
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    void keyPressEvent( QKeyEvent* event);
    void applyNoise();
    void updateNormals();
    void applyBilateralFiltering();
    void applyExtendedBilateralFiltering();

private:
    QDialog *createDialog(const QString &windowTitle) const;
    int whichRadioButton();
    void setDefaultMaterial();
    void setDefaultLight();
    void updateGTDistances();

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
    const float TANSLATE_SPEED;

    QWidget *meshes;
    QGroupBox* groupBox;
    QRadioButton* radio1;
    QRadioButton* radio2;
    QRadioButton* radio3;
    QRadioButton* radio4;
    QRadioButton* radio5;
    QRadioButton* radio6;

};

#if defined(OM_INCLUDE_TEMPLATES) && !defined(SCENE_CC)
#  define SCENE_TEMPLATES
#  include "SceneT.cc"
#endif

#endif
