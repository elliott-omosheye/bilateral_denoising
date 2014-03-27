#ifndef SCENE_CC
#define SCENE_CC

#include <iostream>
#include "Scene.hh"
#include <QVector3D>
#include "GLUT/glut.h"
#include <QtGui>
#include <QtOpenGL>
#include <QDoubleSpinBox>
#include <OpenMesh/Core/Utils/vector_cast.hh>
#include <OpenMesh/Tools/Utils/Timer.hh>

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif


template <typename M>
QDialog
*SceneT<M>::createDialog(const QString &windowTitle) const
{
  QDialog *dialog = new QDialog(0, Qt::CustomizeWindowHint | Qt::WindowTitleHint);

  dialog->setWindowOpacity(0.8);
  dialog->setWindowTitle(windowTitle);
  dialog->setLayout(new QVBoxLayout);

  return dialog;
}

template <typename M>
SceneT<M>::SceneT()
: m_backgroundColor(0.0f, 0.0f, 0.0f)
, m_distance(4.5f)
, m_vertical(-0.1f)
, m_horizontal(0.0f)
, TANSLATE_SPEED(0.01f)
{

  //QWidget *examples = createDialog(tr("Examples"));
  //m_ex1Button = new QPushButton(tr("Example 1 (ToDo)"));
  //examples->layout()->addWidget(m_ex1Button );
  //m_ex2Button= new QPushButton(tr("Example 2 (ToDo)"));
  //examples->layout()->addWidget(m_ex2Button);
  //m_ex3Button = new QPushButton(tr("Example 3 (ToDo)"));
  //examples->layout()->addWidget(m_ex3Button );

  QWidget *controls = createDialog(tr("Controls"));
  
  groupBox = new QGroupBox(tr("Select Mesh"));
  radio1 = new QRadioButton(tr("All"));
  radio2 = new QRadioButton(tr("M1"));
  radio3 = new QRadioButton(tr("M2"));
  radio4 = new QRadioButton(tr("M3"));
  radio5 = new QRadioButton(tr("M4"));
  radio6 = new QRadioButton(tr("M5"));
  groupBox->setHidden(true);
  groupBox->setFocusPolicy(Qt::StrongFocus);
  radio3->setHidden(true);
  radio4->setHidden(true);
  radio5->setHidden(true);
  radio6->setHidden(true);

  radio1->setChecked(true);
  QVBoxLayout *vbox = new QVBoxLayout;
  vbox->addWidget(radio1);
  vbox->addWidget(radio2);
  vbox->addWidget(radio3);
  vbox->addWidget(radio4);
  vbox->addWidget(radio5);
  vbox->addWidget(radio6);

  vbox->addStretch(1);
  groupBox->setLayout(vbox);
  controls->layout()->addWidget(groupBox);

  m_modelButton = new QPushButton(tr("Load model"));
  controls->layout()->addWidget(m_modelButton);

  removeModelButton = new QPushButton(tr("Remove model"));
  controls->layout()->addWidget(removeModelButton);
  removeModelButton->setHidden(true);
  
  noiseSpinBox = new QDoubleSpinBox();
  noiseSpinBox->setFocusPolicy(Qt::StrongFocus);
  noiseSpinBox->setMinimum(0.001);
  noiseSpinBox->setMaximum(1.00);
  noiseSpinBox->setSingleStep(0.001);
  //noiseSpinBox->setMinimumWidth(200);
  noiseSpinBox->setDecimals(3);
  noiseSpinBox->setPrefix("Noise: ");
  controls->layout()->addWidget(noiseSpinBox);
  noiseSpinBox->setHidden(true);

  applyNoiseButton = new QPushButton(tr("Apply Noise"));
  controls->layout()->addWidget(applyNoiseButton);
  applyNoiseButton->setHidden(true);

  updateNormalsButton = new QPushButton(tr("Update Normals"));
  controls->layout()->addWidget(updateNormalsButton);
  updateNormalsButton->setHidden(true);

  bilateralFilteringSpinBox = new QSpinBox();
  bilateralFilteringSpinBox->setMinimum(1);
  bilateralFilteringSpinBox->setMaximum(20);
  bilateralFilteringSpinBox->setPrefix("Iterations: ");
  controls->layout()->addWidget(bilateralFilteringSpinBox);
  bilateralFilteringSpinBox->setHidden(true);

  radiusSpinBox  = new QDoubleSpinBox();
  radiusSpinBox ->setMinimum(0.001);
  radiusSpinBox ->setMaximum(10.00);
  radiusSpinBox->setSingleStep(0.001);
  radiusSpinBox->setDecimals(4);

  radiusSpinBox ->setPrefix("Radius: ");
  controls->layout()->addWidget(radiusSpinBox );
  radiusSpinBox ->setHidden(true);

  standardDeviationSpinBox  = new QDoubleSpinBox();
  standardDeviationSpinBox ->setMinimum(0.001);
  standardDeviationSpinBox ->setMaximum(1.000);
  standardDeviationSpinBox->setSingleStep(0.001);
  standardDeviationSpinBox->setDecimals(4);
  standardDeviationSpinBox ->setPrefix("SD: ");
  controls->layout()->addWidget(standardDeviationSpinBox );
  standardDeviationSpinBox ->setHidden(true);


  bilateralFilteringButton = new QPushButton(tr("Apply Filter"));
  controls->layout()->addWidget(bilateralFilteringButton);
  bilateralFilteringButton->setHidden(true);
  
  extendedBilateralFilteringButton = new QPushButton(tr("Apply Extended Filter"));
  controls->layout()->addWidget(extendedBilateralFilteringButton);
  extendedBilateralFilteringButton->setHidden(true);

  //meshes = createDialog(tr("Meshes"));
  //meshes->setHidden(true);

 
  //QWidget *widgets[] = { meshes, controls, examples  };
  QWidget *widgets[] = { controls };

  for (uint i = 0; i < sizeof(widgets) / sizeof(*widgets); ++i) {
    QGraphicsProxyWidget *proxy = new QGraphicsProxyWidget(0, Qt::Dialog);
    proxy->setWidget(widgets[i]);
    addItem(proxy);
  }

  QPointF pos(10, 10);
  foreach (QGraphicsItem *item, items()) {
    item->setFlag(QGraphicsItem::ItemIsMovable);
    item->setCacheMode(QGraphicsItem::DeviceCoordinateCache);
    const QRectF rect = item->boundingRect();
    item->setPos(pos.x() - rect.x(), pos.y() - rect.y());
    pos += QPointF(0, 10 + rect.height());
  }

}

template <typename M>
void
SceneT<M>::setDefaultMaterial(void)
{
  GLfloat mat_a[] = {0.1, 0.1, 0.1, 1.0};
  GLfloat mat_d[] = {0.7, 0.7, 0.5, 1.0};
  GLfloat mat_s[] = {1.0, 1.0, 1.0, 1.0};
  GLfloat shine[] = {120.0};
  
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT,   mat_a);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   mat_d);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  mat_s);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shine);
}


//----------------------------------------------------------------------------

template <typename M>
void
SceneT<M>::setDefaultLight(void)
{
  GLfloat pos1[] = { 0.1,  0.1, -0.02, 0.0};
  GLfloat pos2[] = {-0.1,  0.1, -0.02, 0.0};
  GLfloat pos3[] = { 0.0,  0.0,  0.1,  0.0};
  GLfloat col1[] = { 0.7,  0.7,  0.8,  1.0};
  GLfloat col2[] = { 0.8,  0.7,  0.7,  1.0};
  GLfloat col3[] = { 1.0,  1.0,  1.0,  1.0};
 
  glEnable(GL_LIGHT0);    
  glLightfv(GL_LIGHT0,GL_POSITION, pos1);
  glLightfv(GL_LIGHT0,GL_DIFFUSE,  col1);
  glLightfv(GL_LIGHT0,GL_SPECULAR, col1);
  
  glEnable(GL_LIGHT1);  
  glLightfv(GL_LIGHT1,GL_POSITION, pos2);
  glLightfv(GL_LIGHT1,GL_DIFFUSE,  col2);
  glLightfv(GL_LIGHT1,GL_SPECULAR, col2);
  
  glEnable(GL_LIGHT2);  
  glLightfv(GL_LIGHT2,GL_POSITION, pos3);
  glLightfv(GL_LIGHT2,GL_DIFFUSE,  col3);
  glLightfv(GL_LIGHT2,GL_SPECULAR, col3);
}



template <typename M>
void
SceneT<M>::applyNoise()
{
    std::cout << "Apply Noise" << "\n";
    const int radioId = whichRadioButton();
    if(radioId  == 1)
    {
      for(typename std::vector<QtModelT<M>*>::size_type i = 0; i != models.size(); i++) {
        models[i]->addNoise(noiseSpinBox->value());
      }
    }
    else
    {
      models[radioId-2]->addNoise(noiseSpinBox->value());
    }
  updateGTDistances();
}

template <typename M>
void
SceneT<M>::drawForeground(QPainter *painter, const QRectF &rect)
{
  //std::cout << "Draw Foreground" << "\n";
  painter->beginNativePainting();

  if(models.size() > 0){
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();

    glLoadIdentity();
    gluPerspective(70, width() / height(), 0.01, 1000);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    setDefaultMaterial();

    setDefaultLight();  
    glLoadIdentity();

    glEnable(GL_LIGHTING);
    glShadeModel(GL_FLAT);
    //glutSolidTeapot(0.5);

    glTranslatef(m_horizontal, m_vertical, -m_distance);
    glRotatef(m_rotation.x(), 1, 0, 0);
    glRotatef(m_rotation.y(), 0, 1, 0);
    glRotatef(m_rotation.z(), 0, 0, 1);

    glEnable(GL_MULTISAMPLE);


    //const int radioId = whichRadioButton();
    //if (radioId == 1)
    //{

      for(typename std::vector<QtModelT<M>*>::size_type i = 0; i != models.size(); i++) {
        models[i]->render();
      }

    //}
    //else
    //{
      //glDisable(GL_LIGHTING);
      //glDisable(GL_TEXTURE_2D);
      //models[radioId-2]->renderBackBuffer();

    //}

    glDisable(GL_MULTISAMPLE);

    glPopMatrix();

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
  }

  painter->endNativePainting();
}

template <typename M>
void
SceneT<M>::drawBackground(QPainter *painter, const QRectF &)
{
  if (painter->paintEngine()->type() != QPaintEngine::OpenGL
      && painter->paintEngine()->type() != QPaintEngine::OpenGL2)
  {
    qWarning("OpenGLScene: drawBackground needs a QGLWidget to be set as viewport on the graphics view");
    return;
  }
  painter->beginNativePainting();
  glClearColor(m_backgroundColor.redF(), m_backgroundColor.greenF(), m_backgroundColor.blueF(), 1.0f);
  //glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  painter->endNativePainting();
}

template <typename M>
void
SceneT<M>::loadMesh(const QString filePath)
{
  if (filePath.isEmpty())
    return;

  std::cout << filePath.toStdString() << "\n";
  m_modelButton->setEnabled(false);
  QApplication::setOverrideCursor(Qt::BusyCursor);
  if(OpenMesh::IO::read_mesh(m_mymesh, filePath.toStdString(), _options))
  {

    models.push_back(new QtModelT<M>(m_mymesh));

    //if(!models.back()->hasColour())
      //models.back()->updateColour(models.size());

    updateGTDistances();
    switch(models.size())
    {
      case 1:
        removeModelButton->setHidden(false);
        //meshes->setHidden(false);
        noiseSpinBox->setHidden(false);
        applyNoiseButton->setHidden(false);
        groupBox->setHidden(false);
        updateNormalsButton->setHidden(false);
        bilateralFilteringSpinBox->setHidden(false);
        bilateralFilteringButton->setHidden(false);
        extendedBilateralFilteringButton->setHidden(false);
        radiusSpinBox->setHidden(false);
        standardDeviationSpinBox->setHidden(false);
        break;
      case 2:
        radio3->setHidden(false);
        break;
      case 3:
        radio4->setHidden(false);
        break;
      case 4:
        radio5->setHidden(false);
        break;
      case 5:
        radio6->setHidden(false);
        break;
    }

    //std::clog << m_mymesh.n_vertices() << " vertices, "
    //<< m_mymesh.n_edges()    << " edge, "
    //<< m_mymesh.n_faces()    << " faces\n";
  }
  else
  {
    std::cout << "Error Loading Mesh" << "\n";
  }
  m_modelButton->setEnabled(true);
  QApplication::restoreOverrideCursor();

}


template <typename M>
void
SceneT<M>::wheelEvent(QGraphicsSceneWheelEvent *event)
{

  QGraphicsScene::wheelEvent(event);
  if (event->isAccepted())
    return;

  m_distance *= qPow(1.2, -event->delta() / 120.);
  //std::cout << m_distance << "\n";
  event->accept();
  update();
}

template <typename M>
void
SceneT<M>::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
  QGraphicsScene::mouseMoveEvent(event);
  if (event->isAccepted())
    return;
  if (event->buttons() & Qt::LeftButton) {
    const QPointF delta = event->scenePos() - event->lastScenePos();
    QVector3D angularImpulse = QVector3D(delta.y(), delta.x(), 0) * 0.1;
    
    const int radioId = whichRadioButton();
    if(radioId  == 1){
      m_rotation += angularImpulse;
    }
    else
    {
      models[radioId-2]->updateRotation(angularImpulse);
    }
    event->accept();
    update();
  }
}

template <typename M>
void
SceneT<M>::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
  QGraphicsScene::mousePressEvent(event);
  if (event->isAccepted())
    return;
  m_mouseEventTime = m_time.elapsed();



  event->accept();
}

template <typename M>
void
SceneT<M>::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
  QGraphicsScene::mouseReleaseEvent(event);
  if (event->isAccepted())
    return;
  const int delta = m_time.elapsed() - m_mouseEventTime;
  event->accept();
  update();
}

template <typename M>
void
SceneT<M>::keyPressEvent( QKeyEvent* event)
{
  const int radioId = whichRadioButton();
  if(radioId  == 1)
  {
    switch(event->key())
    {
      case Key_Up:
        m_vertical += TANSLATE_SPEED;
        break;
      case Key_Down:
        m_vertical -= TANSLATE_SPEED;
        break;
      case Key_Right:
        m_horizontal += TANSLATE_SPEED;
        break;
      case Key_Left:
        m_horizontal -= TANSLATE_SPEED;
        break;
    }
  }
  else
  {
    switch(event->key())
    {
      case Key_Up:
        models[radioId-2]->updateVertical(TANSLATE_SPEED);
        break;
      case Key_Down:
        models[radioId-2]->updateVertical(-TANSLATE_SPEED);
        break;
      case Key_Right:
        models[radioId-2]->updateHorizontal(TANSLATE_SPEED);
        break;
      case Key_Left:
        models[radioId-2]->updateHorizontal(-TANSLATE_SPEED);
        break;
    }
  }
  event->accept();
  update();
}

template <typename M>
int
SceneT<M>::whichRadioButton()
{
  if(radio1->isChecked())
    return 1;
  else if(radio2->isChecked())
    return 2;
  else if(radio3->isChecked())
    return 3;
  else if(radio4->isChecked())
    return 4;
  else if(radio5->isChecked())
    return 5;
  else if(radio6->isChecked())
    return 6;
  else
    return 0;
}

template <typename M>
void
SceneT<M>::updateNormals()
{
  const int radioId = whichRadioButton();
  if(radioId  == 1)
  {
    for(typename std::vector<QtModelT<M>*>::size_type i = 0; i != models.size(); i++) {
        models[i]->calcNormals();
    }
  }
  else
  {
    models[radioId-2]->calcNormals();
  }
  updateGTDistances();
}

template <typename M>
void
SceneT<M>::applyBilateralFiltering()
{
  const int radioId = whichRadioButton();
  for(int i = 0; i < bilateralFilteringSpinBox->value(); i++)
  {
    
    if(radioId  == 1)
    {
      for(typename std::vector<QtModelT<M>*>::size_type i = 0; i != models.size(); i++) {
          models[i]->bilateralFiltering(radiusSpinBox->value(),standardDeviationSpinBox->value());
      }
    }
    else
    {
      //standardDeviationSpinBox->value()
      //radiusSpinBox->value()
      models[radioId-2]->bilateralFiltering(radiusSpinBox->value(),standardDeviationSpinBox->value());
    }

  }
  updateGTDistances();
}


template <typename M>
void
SceneT<M>::applyExtendedBilateralFiltering()
{
  const int radioId = whichRadioButton();
  for(int i = 0; i < bilateralFilteringSpinBox->value(); i++)
  {
    
    if(radioId  == 1)
    {
      for(typename std::vector<QtModelT<M>*>::size_type i = 0; i != models.size(); i++) {
          models[i]->extendedBilateralFiltering(radiusSpinBox->value(),standardDeviationSpinBox->value());
      }
    }
    else
    {
      //standardDeviationSpinBox->value()
      //radiusSpinBox->value()
      models[radioId-2]->extendedBilateralFiltering(radiusSpinBox->value(),standardDeviationSpinBox->value());
    }

  }
  updateGTDistances();
}

template <typename M>
void
SceneT<M>::updateGTDistances()
{
  if (models.size() > 0)
  {
    std::stringstream sstm;
    sstm << "M1 (" << models[0]->gt_distance << ", " << models[0]->FaceNormalErrorCalc() << ")";
    radio2->setText(QString::fromStdString(sstm.str()));
  }
  if (models.size() > 1)
  {
    std::stringstream sstm;
    sstm << "M2 (" << models[1]->gt_distance << ")";
    radio2->setText(QString::fromStdString(sstm.str()));
  }
  if (models.size() > 2)
  {
    std::stringstream sstm;
    sstm << "M3 (" << models[2]->gt_distance << ")";
    radio2->setText(QString::fromStdString(sstm.str()));
  }
  if (models.size() > 3)
  {
    std::stringstream sstm;
    sstm << "M4 (" << models[3]->gt_distance << ")";
    radio2->setText(QString::fromStdString(sstm.str()));
  }
  if (models.size() > 4)
  {
    std::stringstream sstm;
    sstm << "M5 (" << models[4]->gt_distance << ")";
    radio2->setText(QString::fromStdString(sstm.str()));
  }

}

#endif
