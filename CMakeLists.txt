PROJECT (vtkExamplePointsInConvexHull)

CMAKE_MINIMUM_REQUIRED (VERSION 2.4.0)

FIND_PACKAGE (VTK REQUIRED)
INCLUDE(${USE_VTK_FILE})

SET(vtkExamplePointsInConvexHull_SRCS
    vtkExamplePointsInConvexHull.cpp
    )
    

INCLUDE_DIRECTORIES(${vtkExamplePointsInConvexHull_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} )

ADD_EXECUTABLE( example ${vtkExamplePointsInConvexHull_SRCS})

TARGET_LINK_LIBRARIES ( example
  vtkIO vtkGraphics
  )

