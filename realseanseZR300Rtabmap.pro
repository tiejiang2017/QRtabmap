#-------------------------------------------------
#
# Project created by QtCreator 2017-06-21T09:45:36
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport svg

TARGET = realseanseZR300Rtabmap
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES +=\
    rtabmap/app/src/main.cpp \
    rtabmap/corelib/src/BayesFilter.cpp \
    rtabmap/corelib/src/Camera.cpp \
    rtabmap/corelib/src/CameraModel.cpp \
    rtabmap/corelib/src/CameraRGB.cpp \
    rtabmap/corelib/src/CameraRGBD.cpp \
    rtabmap/corelib/src/CameraStereo.cpp \
    rtabmap/corelib/src/CameraThread.cpp \
    rtabmap/corelib/src/Compression.cpp \
    rtabmap/corelib/src/DBDriver.cpp \
    rtabmap/corelib/src/DBDriverSqlite3.cpp \
    rtabmap/corelib/src/DBReader.cpp \
    rtabmap/corelib/src/EpipolarGeometry.cpp \
    rtabmap/corelib/src/Features2d.cpp \
    rtabmap/corelib/src/FlannIndex.cpp \
    rtabmap/corelib/src/GainCompensator.cpp \
    rtabmap/corelib/src/GeodeticCoords.cpp \
    rtabmap/corelib/src/Graph.cpp \
    rtabmap/corelib/src/Link.cpp \
    rtabmap/corelib/src/Memory.cpp \
    rtabmap/corelib/src/OccupancyGrid.cpp \
    rtabmap/corelib/src/Odometry.cpp \
    rtabmap/corelib/src/OdometryF2F.cpp \
    rtabmap/corelib/src/OdometryF2M.cpp \
    rtabmap/corelib/src/OdometryMono.cpp \
    rtabmap/corelib/src/OdometryThread.cpp \
    rtabmap/corelib/src/Optimizer.cpp \
    rtabmap/corelib/src/OptimizerCVSBA.cpp \
    rtabmap/corelib/src/OptimizerG2O.cpp \
    rtabmap/corelib/src/OptimizerGTSAM.cpp \
    rtabmap/corelib/src/OptimizerTORO.cpp \
    rtabmap/corelib/src/Parameters.cpp \
    rtabmap/corelib/src/Registration.cpp \
    rtabmap/corelib/src/RegistrationIcp.cpp \
    rtabmap/corelib/src/RegistrationVis.cpp \
    rtabmap/corelib/src/Rtabmap.cpp \
    rtabmap/corelib/src/RtabmapThread.cpp \
    rtabmap/corelib/src/SensorData.cpp \
    rtabmap/corelib/src/Signature.cpp \
    rtabmap/corelib/src/Statistics.cpp \
    rtabmap/corelib/src/Stereo.cpp \
    rtabmap/corelib/src/StereoCameraModel.cpp \
    rtabmap/corelib/src/StereoDense.cpp \
    rtabmap/corelib/src/Transform.cpp \
    rtabmap/corelib/src/util2d.cpp \
    rtabmap/corelib/src/util3d.cpp \
    rtabmap/corelib/src/util3d_correspondences.cpp \
    rtabmap/corelib/src/util3d_features.cpp \
    rtabmap/corelib/src/util3d_filtering.cpp \
    rtabmap/corelib/src/util3d_mapping.cpp \
    rtabmap/corelib/src/util3d_motion_estimation.cpp \
    rtabmap/corelib/src/util3d_registration.cpp \
    rtabmap/corelib/src/util3d_surface.cpp \
    rtabmap/corelib/src/util3d_transforms.cpp \
    rtabmap/corelib/src/VisualWord.cpp \
    rtabmap/corelib/src/VWDictionary.cpp \
    rtabmap/corelib/src/ConvertUTF.c \
    rtabmap/corelib/src/clams/discrete_depth_distortion_model.cpp \
    rtabmap/corelib/src/clams/discrete_depth_distortion_model_helpers.cpp \
    rtabmap/corelib/src/clams/frame_projector.cpp \
    rtabmap/corelib/src/clams/slam_calibrator.cpp \
    rtabmap/corelib/src/opencv/Orb.cpp \
    rtabmap/corelib/src/opencv/solvepnp.cpp \
    rtabmap/corelib/src/rtflann/ext/lz4.c \
    rtabmap/corelib/src/rtflann/ext/lz4hc.c \
    rtabmap/corelib/src/sqlite3/sqlite3.c \
    rtabmap/corelib/src/toro3d/posegraph2.cpp \
    rtabmap/corelib/src/toro3d/posegraph3.cpp \
    rtabmap/corelib/src/toro3d/treeoptimizer2.cpp \
    rtabmap/corelib/src/toro3d/treeoptimizer3.cpp \
    rtabmap/corelib/src/toro3d/treeoptimizer3_iteration.cpp \
    rtabmap/corelib/src/vertigo/g2o/edge_se2MaxMixture.cpp \
    rtabmap/corelib/src/vertigo/g2o/edge_se2Switchable.cpp \
    rtabmap/corelib/src/vertigo/g2o/edge_se3Switchable.cpp \
    rtabmap/corelib/src/vertigo/g2o/edge_switchPrior.cpp \
    rtabmap/corelib/src/vertigo/g2o/types_g2o_robust.cpp \
    rtabmap/corelib/src/vertigo/g2o/vertex_switchLinear.cpp \
    rtabmap/guilib/src/AboutDialog.cpp \
    rtabmap/guilib/src/CalibrationDialog.cpp \
    rtabmap/guilib/src/CameraViewer.cpp \
    rtabmap/guilib/src/CloudViewer.cpp \
    rtabmap/guilib/src/ConsoleWidget.cpp \
    rtabmap/guilib/src/CreateSimpleCalibrationDialog.cpp \
    rtabmap/guilib/src/DatabaseViewer.cpp \
    rtabmap/guilib/src/DataRecorder.cpp \
    rtabmap/guilib/src/DepthCalibrationDialog.cpp \
    rtabmap/guilib/src/EditDepthArea.cpp \
    rtabmap/guilib/src/ExportCloudsDialog.cpp \
    rtabmap/guilib/src/ExportDialog.cpp \
    rtabmap/guilib/src/ExportScansDialog.cpp \
    rtabmap/guilib/src/GraphViewer.cpp \
    rtabmap/guilib/src/ImageView.cpp \
    rtabmap/guilib/src/KeypointItem.cpp \
    rtabmap/guilib/src/LoopClosureViewer.cpp \
    rtabmap/guilib/src/MainWindow.cpp \
    rtabmap/guilib/src/MapVisibilityWidget.cpp \
    rtabmap/guilib/src/OdometryViewer.cpp \
    rtabmap/guilib/src/ParametersToolBox.cpp \
    rtabmap/guilib/src/PdfPlot.cpp \
    rtabmap/guilib/src/PostProcessingDialog.cpp \
    rtabmap/guilib/src/PreferencesDialog.cpp \
    rtabmap/guilib/src/ProgressDialog.cpp \
    rtabmap/guilib/src/StatsToolBox.cpp \
    rtabmap/guilib/src/3rdParty/QMultiComboBox.cpp \
    rtabmap/guilib/src/opencv/vtkImageMatSource.cpp \
    rtabmap/guilib/src/utilite/UPlot.cpp \
    rtabmap/utilite/src/UConversion.cpp \
    rtabmap/utilite/src/UDirectory.cpp \
    rtabmap/utilite/src/UEventsHandler.cpp \
    rtabmap/utilite/src/UEventsManager.cpp \
    rtabmap/utilite/src/UEventsSender.cpp \
    rtabmap/utilite/src/UFile.cpp \
    rtabmap/utilite/src/ULogger.cpp \
    rtabmap/utilite/src/UProcessInfo.cpp \
    rtabmap/utilite/src/UThread.cpp \
    rtabmap/utilite/src/UTimer.cpp \
    rtabmap/utilite/src/UVariant.cpp

HEADERS  += \
    rtabmap/corelib/src/ConvertUTF.h \
    rtabmap/corelib/src/DBDriverSqlite3.h \
    rtabmap/corelib/src/SimpleIni.h \
    rtabmap/corelib/src/clams/eigen_extensions/eigen_extensions.h \
    rtabmap/corelib/src/opencv/Orb.h \
    rtabmap/corelib/src/opencv/solvepnp.h \
    rtabmap/corelib/src/pcl18/surface/organized_fast_mesh.h \
    rtabmap/corelib/src/pcl18/surface/texture_mapping.h \
    rtabmap/corelib/src/pcl18/surface/impl/organized_fast_mesh.hpp \
    rtabmap/corelib/src/pcl18/surface/impl/texture_mapping.hpp \
    rtabmap/corelib/src/rtflann/config.h \
    rtabmap/corelib/src/rtflann/defines.h \
    rtabmap/corelib/src/rtflann/flann.hpp \
    rtabmap/corelib/src/rtflann/general.h \
    rtabmap/corelib/src/rtflann/algorithms/all_indices.h \
    rtabmap/corelib/src/rtflann/algorithms/autotuned_index.h \
    rtabmap/corelib/src/rtflann/algorithms/center_chooser.h \
    rtabmap/corelib/src/rtflann/algorithms/composite_index.h \
    rtabmap/corelib/src/rtflann/algorithms/dist.h \
    rtabmap/corelib/src/rtflann/algorithms/hierarchical_clustering_index.h \
    rtabmap/corelib/src/rtflann/algorithms/kdtree_index.h \
    rtabmap/corelib/src/rtflann/algorithms/kdtree_single_index.h \
    rtabmap/corelib/src/rtflann/algorithms/kmeans_index.h \
    rtabmap/corelib/src/rtflann/algorithms/linear_index.h \
    rtabmap/corelib/src/rtflann/algorithms/lsh_index.h \
    rtabmap/corelib/src/rtflann/algorithms/nn_index.h \
    rtabmap/corelib/src/rtflann/ext/lz4.h \
    rtabmap/corelib/src/rtflann/ext/lz4hc.h \
    rtabmap/corelib/src/rtflann/nn/ground_truth.h \
    rtabmap/corelib/src/rtflann/nn/index_testing.h \
    rtabmap/corelib/src/rtflann/nn/simplex_downhill.h \
    rtabmap/corelib/src/rtflann/util/allocator.h \
    rtabmap/corelib/src/rtflann/util/any.h \
    rtabmap/corelib/src/rtflann/util/dynamic_bitset.h \
    rtabmap/corelib/src/rtflann/util/heap.h \
    rtabmap/corelib/src/rtflann/util/logger.h \
    rtabmap/corelib/src/rtflann/util/lsh_table.h \
    rtabmap/corelib/src/rtflann/util/matrix.h \
    rtabmap/corelib/src/rtflann/util/object_factory.h \
    rtabmap/corelib/src/rtflann/util/params.h \
    rtabmap/corelib/src/rtflann/util/random.h \
    rtabmap/corelib/src/rtflann/util/result_set.h \
    rtabmap/corelib/src/rtflann/util/sampling.h \
    rtabmap/corelib/src/rtflann/util/saving.h \
    rtabmap/corelib/src/rtflann/util/serialization.h \
    rtabmap/corelib/src/rtflann/util/timer.h \
    rtabmap/corelib/src/sqlite3/sqlite3.h \
    rtabmap/corelib/src/sqlite3/sqlite3ext.h \
    rtabmap/corelib/src/toro3d/dmatrix.hh \
    rtabmap/corelib/src/toro3d/dmatrix.hxx \
    rtabmap/corelib/src/toro3d/posegraph.hh \
    rtabmap/corelib/src/toro3d/posegraph.hxx \
    rtabmap/corelib/src/toro3d/posegraph2.hh \
    rtabmap/corelib/src/toro3d/posegraph3.hh \
    rtabmap/corelib/src/toro3d/transformation2.hh \
    rtabmap/corelib/src/toro3d/transformation3.hh \
    rtabmap/corelib/src/toro3d/transformation3.hxx \
    rtabmap/corelib/src/toro3d/treeoptimizer2.hh \
    rtabmap/corelib/src/toro3d/treeoptimizer3.hh \
    rtabmap/corelib/src/vertigo/g2o/edge_se2MaxMixture.h \
    rtabmap/corelib/src/vertigo/g2o/edge_se2Switchable.h \
    rtabmap/corelib/src/vertigo/g2o/edge_se3Switchable.h \
    rtabmap/corelib/src/vertigo/g2o/edge_switchPrior.h \
    rtabmap/corelib/src/vertigo/g2o/vertex_switchLinear.h \
    rtabmap/corelib/src/vertigo/gtsam/betweenFactorMaxMix.h \
    rtabmap/corelib/src/vertigo/gtsam/betweenFactorSwitchable.h \
    rtabmap/corelib/src/vertigo/gtsam/switchVariableLinear.h \
    rtabmap/corelib/src/vertigo/gtsam/switchVariableSigmoid.h \
    rtabmap/guilib/src/AboutDialog.h \
    rtabmap/guilib/src/CreateSimpleCalibrationDialog.h \
    rtabmap/guilib/src/DepthCalibrationDialog.h \
    rtabmap/guilib/src/EditDepthArea.h \
    rtabmap/guilib/src/ExportCloudsDialog.h \
    rtabmap/guilib/src/ExportDialog.h \
    rtabmap/guilib/src/ExportScansDialog.h \
    rtabmap/guilib/src/MapVisibilityWidget.h \
    rtabmap/guilib/src/ParametersToolBox.h \
    rtabmap/guilib/src/PostProcessingDialog.h \
    rtabmap/guilib/src/TexturingState.h \
    rtabmap/guilib/src/3rdParty/QMultiComboBox.h \
    rtabmap/guilib/src/opencv/vtkImageMatSource.h \
    rtabmap/app/src/ObjDeletionHandler.h \
    rtabmap/guilib/include/rtabmap/gui/CalibrationDialog.h \
    rtabmap/guilib/include/rtabmap/gui/CameraViewer.h \
    rtabmap/guilib/include/rtabmap/gui/CloudViewer.h \
    rtabmap/guilib/include/rtabmap/gui/ConsoleWidget.h \
    rtabmap/guilib/include/rtabmap/gui/DatabaseViewer.h \
    rtabmap/guilib/include/rtabmap/gui/DataRecorder.h \
    rtabmap/guilib/include/rtabmap/gui/GraphViewer.h \
    rtabmap/guilib/include/rtabmap/gui/ImageView.h \
    rtabmap/guilib/include/rtabmap/gui/KeypointItem.h \
    rtabmap/guilib/include/rtabmap/gui/LoopClosureViewer.h \
    rtabmap/guilib/include/rtabmap/gui/MainWindow.h \
    rtabmap/guilib/include/rtabmap/gui/OdometryViewer.h \
    rtabmap/guilib/include/rtabmap/gui/PdfPlot.h \
    rtabmap/guilib/include/rtabmap/gui/PreferencesDialog.h \
    rtabmap/guilib/include/rtabmap/gui/ProgressDialog.h \
    rtabmap/guilib/include/rtabmap/gui/RtabmapGuiExp.h \
    rtabmap/guilib/include/rtabmap/gui/StatsToolBox.h \
    rtabmap/guilib/include/rtabmap/utilite/UCv2Qt.h \
    rtabmap/guilib/include/rtabmap/utilite/UImageView.h \
    rtabmap/guilib/include/rtabmap/utilite/UPlot.h

FORMS    += \
    rtabmap/guilib/src/ui/aboutDialog.ui \
    rtabmap/guilib/src/ui/calibrationDialog.ui \
    rtabmap/guilib/src/ui/consoleWidget.ui \
    rtabmap/guilib/src/ui/createSimpleCalibrationDialog.ui \
    rtabmap/guilib/src/ui/DatabaseViewer.ui \
    rtabmap/guilib/src/ui/depthCalibrationDialog.ui \
    rtabmap/guilib/src/ui/exportCloudsDialog.ui \
    rtabmap/guilib/src/ui/exportDialog.ui \
    rtabmap/guilib/src/ui/exportScansDialog.ui \
    rtabmap/guilib/src/ui/loopClosureViewer.ui \
    rtabmap/guilib/src/ui/mainWindow.ui \
    rtabmap/guilib/src/ui/postProcessingDialog.ui \
    rtabmap/guilib/src/ui/preferencesDialog.ui
INCLUDEPATH += ../rtabmap/utilite/include \
               ../rtabmap/corelib/include \
                ../rtabmap/guilib/include \
                /usr/local/include/pcl-1.8 \
                /usr/include/eigen3 \
                /usr/include/ni \
                ../rtabmap/corelib/src \
                /usr/include/suitesparse \
                /usr/local/include/vtk-7.1 \
                ../rtabmap/guilib/src \
                /home/xtx/workspace/OpenNI2/Include


LIBS += -lz -ldl /usr/lib/x86_64-linux-gnu/libopencv_*.so -ldc1394 -lgomp  /usr/local/lib/libvtk*  /usr/local/lib/libg2o*.so -lcxsparse
LIBS += /usr/local/lib/libpcl*.so  /usr/local/lib/libboost*.so /home/xtx/workspace/OpenNI2/Bin/x64-Release/libOpenNI2.so /usr/lib/libOpenNI.so.0
LIBS += -llz4 /usr/lib/librealsense*.so -lcholmod /usr/local/lib/libflann_cpp_s.a
LIBS += /usr/local/lib/libpcl_people.so -lz -lrealsense -lrealsense_slam -lrealsense_image -lSP_Core -ltracker -lcpu_tsdf


#DEFINES +=RTABMAP_OPENNI2
DISTFILES +=-lSM -lICE -lX11 -lXext -lXt -lm \
    rtabmap/app/src/RTABMap.ico \
    rtabmap/app/src/RTABMap.icns \
    rtabmap/app/src/RTABMap.rc

RESOURCES += \
    rtabmap/guilib/src/GuiLib.qrc
