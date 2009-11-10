#include <cstdio>

#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkCellArray.h>
#include <vtkIdList.h>
#include <vtkDelaunay3D.h>
#include <vtkUnstructuredGrid.h>
#include <vtkSmartPointer.h>
#include <vtkOBBTree.h>

using namespace std;

int main(int argc, char **argv)
{
	// First read the example pointcloud
	vtkSmartPointer< vtkPolyDataReader > reader = vtkSmartPointer< vtkPolyDataReader >::New();
	reader->SetFileName("example_pointcloud.vtk");
	reader->Update();

	// Create a regular grid with a defined spacing inside the boundaries of the pointcloud
	// Later we'll check which of the gridpoints are inside the convex hull of the pointcloud
	vtkSmartPointer< vtkPoints > reggrid=vtkSmartPointer< vtkPoints >::New();

	double pointtoinsert[3], bounds[6];

	reader->GetOutput()->GetBounds(bounds); // boundaries of the pointcloud;

	double gridspacing=15.0; // choose whatever you like

	// filling the regular grid
	double x=bounds[0], y=bounds[2], z=bounds[4];
	while (x<bounds[1]){
		pointtoinsert[0]=x;
		while (y<bounds[3]){
			pointtoinsert[1]=y;
			while (z<bounds[5]){
				pointtoinsert[2]=z;
				reggrid->InsertNextPoint(pointtoinsert);
				z=z+gridspacing;
			}
			z=bounds[4];
			y=y+gridspacing;
		}
		z=bounds[4];
		y=bounds[2];
		x=x+gridspacing;
	}

	// Create the convex hull of the pointcloud
	vtkSmartPointer< vtkDelaunay3D > delau = vtkSmartPointer< vtkDelaunay3D >::New();
	delau->SetInput(reader->GetOutput());
	delau->Update();

	// Check which points of the regular grid are inside the convex hull
	int subId, pointno, nopoints=reggrid->GetNumberOfPoints();
	double pcoords[3];
	double *test;
	double *weights = new double[delau->GetOutput()->GetMaxCellSize()];
	vtkIdType cellId;
	int insidepointsvtk=0, intersections=0;
	for (pointno=0;pointno<nopoints;pointno++)
	    {
		test=reggrid->GetPoint(pointno);
	    cellId = delau->GetOutput()->FindCell(test, NULL, 0, 0.0, subId, pcoords, weights);
	     if (cellId>=0){
	    	 cout << "Point #" << pointno << " (" << test[0] << "," << test[1] << "," << test[2] << ") is in cell " << cellId << endl;
	    	 insidepointsvtk++;
	     }
	    }
	cout << endl;
	cout << "Total number of regular gridpoints: " << nopoints << endl;
	cout << "Total number of points inside the convex hull: " << insidepointsvtk << endl;

	return 1;
}
