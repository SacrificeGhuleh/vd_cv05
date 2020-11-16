//
// Created by richardzvonek on 11/16/20.
//
#include "wsurface.h"

#include <vtkPoints.h>
#include <vtkTriangle.h>
#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <opencv2/viz/widget_accessor.hpp>

namespace cv {
  namespace viz {
  
  
    WSurface::WSurface(const std::vector<Triangle> &triangles, const viz::Color &color) {
      // Create a triangle
      vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
      vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
      for(int i = 0; i < triangles.size() * 3; i+=3){
        const Point3f pt1 = triangles.at(i/3).v1;
        const Point3f pt2 = triangles.at(i/3).v2;
        const Point3f pt3 = triangles.at(i/3).v3;
        
        points->InsertNextPoint(pt1.x, pt1.y, pt1.z);
        points->InsertNextPoint(pt2.x, pt2.y, pt2.z);
        points->InsertNextPoint(pt3.x, pt3.y, pt3.z);
        vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();
        triangle->GetPointIds()->SetId(0, i+0);
        triangle->GetPointIds()->SetId(1, i+1);
        triangle->GetPointIds()->SetId(2, i+2);
  
        cells->InsertNextCell(triangle);
  
      }
      // Create a polydata object
      vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
      // Add the geometry and topology to the polydata
      polyData->SetPoints(points);
      polyData->SetPolys(cells);
      // Create mapper and actor
      vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
#if VTK_MAJOR_VERSION <= 5
      mapper->SetInput(polyData);
#else
      mapper->SetInputData(polyData);
#endif
      vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
      actor->SetMapper(mapper);
      // Store this actor in the widget in order that visualizer can access it
      viz::WidgetAccessor::setProp(*this, actor);
      // Set the color of the widget. This has to be called after WidgetAccessor.
      setColor(color);
    }
  }
}