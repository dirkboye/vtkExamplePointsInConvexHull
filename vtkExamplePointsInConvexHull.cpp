/*================================================================================

vtkExamplePointsInConvexHull.cpp

Copyright (c) 2009, Dirk Boye
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    * 	Redistributions of source code must retain the above copyright notice,
		this list of conditions and the following disclaimer.
    * 	Redistributions in binary form must reproduce the above copyright notice,
		this list of conditions and the following disclaimer in the documentation
		and/or other materials provided with the distribution.
    *	Neither the name of Dirk Boye nor the names of its contributors
		may be used to endorse or promote products derived from this software
		without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

==================================================================================*/

#include <cstdio>

#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkCellArray.h>
#include <vtkIdList.h>
#include <vtkDelaunay3D.h>
#include <vtkUnstructuredGrid.h>
#include <vtkSmartPointer.h>

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
