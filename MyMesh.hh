#ifndef MyMesh_HH
#define MyMesh_HH
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <eigen3/Eigen/Dense>

struct MyTraits : public OpenMesh::DefaultTraits
{
  typedef OpenMesh::Vec3f Color;
  VertexAttributes( OpenMesh::Attributes::Normal |
                    OpenMesh::Attributes::Color );

  FaceAttributes( OpenMesh::Attributes::Normal |
                    OpenMesh::Attributes::Color );



  HalfedgeAttributes(OpenMesh::Attributes::PrevHalfedge);
};

typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits>  MyMesh;
//typedef MyMesh::ConstFaceVertexIter FVI;
typedef MyMesh::Point Point;
typedef Eigen::MatrixX3d PointMatrix;



#endif
