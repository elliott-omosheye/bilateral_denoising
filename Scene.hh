#ifndef SCENE_HH
#define SCENE_HH

#include <OpenMesh/Core/IO/MeshIO.hh>

#include <SceneT.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Tools/Utils/getopt.h>
#include "MyMesh.hh"

using namespace Qt;
using namespace OpenMesh;



class Scene : public SceneT<MyMesh>
{
  Q_OBJECT
public:
    /// default constructor
    Scene() : SceneT<MyMesh>()
    {
      connect(m_modelButton, SIGNAL(clicked()), this, SLOT(selectMesh()));
      connect(m_applyButton, SIGNAL(clicked()), this, SLOT(applyTransformationsSlot()));
      connect(m_icpButton, SIGNAL(clicked()), this, SLOT(runICPSlot()));
      connect(m_icpButton5, SIGNAL(clicked()), this, SLOT(runICPSlot5()));
      connect(m_icpButton10, SIGNAL(clicked()), this, SLOT(runICPSlot10()));
      connect(m_icpButton50, SIGNAL(clicked()), this, SLOT(runICPSlot50()));
      connect(m_icpButton100, SIGNAL(clicked()), this, SLOT(runICPSlot100()));
      connect(m_twoMeshButton, SIGNAL(clicked()), this, SLOT(twoMeshSlot()));
      connect(m_sameMeshButton, SIGNAL(clicked()), this, SLOT(sameMeshSlot()));
      connect(m_globalButton, SIGNAL(clicked()), this, SLOT(globalSlot()));

      connect(m_icpButtonSampling, SIGNAL(clicked()), this, SLOT(samplingSlot()));
      connect(m_icpButtonNoise, SIGNAL(clicked()), this, SLOT(noiseSlot()));
    }

public slots:
    void selectMesh()
    {
      loadMesh(QFileDialog::getOpenFileName(0, tr("Choose mesh"), QString(), QLatin1String("*.ply")));
    }
    void applyTransformationsSlot()
    {
      applyTransformations();
    }
    void runICPSlot()
    {
      runICP(1);
    }
    void runICPSlot5()
    {
      runICP(5);
    }
    void runICPSlot10()
    {
      runICP(10);
    }
    void runICPSlot50()
    {
      runICP(50);
    }
    void runICPSlot100()
    {
      runICP(100);
    }
    void noiseSlot()
    {
      runNoiseICP();
    }
    void samplingSlot()
    {
      runSamplingICP();
    }
    void twoMeshSlot()
    {
      runTwoMesh("./bun000.ply", "./bun045.ply");
    }
   void sameMeshSlot()
    {
      runTwoMesh("./bun000.ply", "./bun000.ply");
    }
    void globalSlot()
    {
      loadGlobal("./bun000.ply", "./bun045.ply", "./bun315.ply");
    }

};
#endif
