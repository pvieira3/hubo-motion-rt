ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include -I/usr/include/eigen3 -I$(top_srcdir)/src/dart_Lite/src/ -L/home/hubo/Downloads/hubo-motion-rt -L/usr/local/lib
AM_CFLAGS = -std=gnu99 -L/usr/local/lib

EXTRA_DIST = tables/control.table

init_ddir = /etc/init.d
init_d_SCRIPTS = scripts/tech

includedir = /usr/local/include

# installed under $prefix/include/
kinematicsdir = $(includedir)/kinematics
kinematics_HEADERS = src/dart_Lite/src/kinematics/BodyNode.h \
src/dart_Lite/src/kinematics/Dof.h \
src/dart_Lite/src/kinematics/Joint.h \
src/dart_Lite/src/kinematics/Marker.h \
src/dart_Lite/src/kinematics/Shape.h \
src/dart_Lite/src/kinematics/Skeleton.h \
src/dart_Lite/src/kinematics/Transformation.h \
src/dart_Lite/src/kinematics/TrfmRotateAxis.h \
src/dart_Lite/src/kinematics/TrfmRotateEuler.h \
src/dart_Lite/src/kinematics/TrfmRotateExpmap.h \
src/dart_Lite/src/kinematics/TrfmRotateQuat.h \
src/dart_Lite/src/kinematics/TrfmTranslate.h


robotics_parser_urdf_parserdir = $(includedir)/robotics/parser/urdf_parser
robotics_parser_urdf_parser_HEADERS = src/dart_Lite/src/robotics/parser/urdf_parser/urdf_parser.h

robotics_parser_urdfdom_headers_urdf_exceptiondir = $(includedir)/robotics/parser/urdfdom_headers/urdf_exception
robotics_parser_urdfdom_headers_urdf_exception_HEADERS = src/dart_Lite/src/robotics/parser/urdfdom_headers/urdf_exception/exception.h

robotics_parser_urdfdom_headers_urdf_modeldir = $(includedir)/robotics/parser/urdfdom_headers/urdf_model
robotics_parser_urdfdom_headers_urdf_model_HEADERS = src/dart_Lite/src/robotics/parser/urdfdom_headers/urdf_model/color.h \
src/dart_Lite/src/robotics/parser/urdfdom_headers/urdf_model/joint.h \
src/dart_Lite/src/robotics/parser/urdfdom_headers/urdf_model/link.h \
src/dart_Lite/src/robotics/parser/urdfdom_headers/urdf_model/model.h \
src/dart_Lite/src/robotics/parser/urdfdom_headers/urdf_model/pose.h \
src/dart_Lite/src/robotics/parser/urdfdom_headers/urdf_model/twist.h

robotics_parser_urdfdom_headers_urdf_model_statedir = $(includedir)/robotics/parser/urdfdom_headers/urdf_model_state
robotics_parser_urdfdom_headers_urdf_model_state_HEADERS = src/dart_Lite/src/robotics/parser/urdfdom_headers/urdf_model_state/model_state.h

robotics_parser_urdfdom_headers_urdf_sensordir = $(includedir)/robotics/parser/urdfdom_headers/urdf_sensor
robotics_parser_urdfdom_headers_urdf_sensor_HEADERS = src/dart_Lite/src/robotics/parser/urdfdom_headers/urdf_sensor/sensor.h

robotics_parser_urdfdom_headers_urdf_worlddir = $(includedir)/robotics/parser/urdfdom_headers/urdf_world
robotics_parser_urdfdom_headers_urdf_world_HEADERS = src/dart_Lite/src/robotics/parser/urdfdom_headers/urdf_world/world.h

robotics_parser_dart_parserdir = $(includedir)/robotics/parser/dart_parser
robotics_parser_dart_parser_HEADERS = src/dart_Lite/src/robotics/parser/dart_parser/DartLoader.h
 
utilsdir = $(includedir)/utils
utils_HEADERS = src/dart_Lite/src/utils/EigenHelper.h \
src/dart_Lite/src/utils/LoadOpengl.h \
src/dart_Lite/src/utils/Misc.h \
src/dart_Lite/src/utils/Paths.h \
src/dart_Lite/src/utils/Timer.h \
src/dart_Lite/src/utils/UtilsCode.h \
src/dart_Lite/src/utils/UtilsMath.h \
src/dart_Lite/src/utils/UtilsRotation.h

dynamicsdir = $(includedir)/dynamics
dynamics_HEADERS = src/dart_Lite/src/dynamics/BodyNodeDynamics.h src/dart_Lite/src/dynamics/SkeletonDynamics.h

include_HEADERS = include/Hubo_Tech.h include/control-daemon.h include/daemonizer.h include/manip.h

bin_PROGRAMS = control-daemon tech-console proto-manip-daemon oneLeg moveLimbs

hubotechconfdir=/etc/hubo-ach
hubotechconf_DATA = tables/control.table

control_daemon_SOURCES = src/control-daemon.c
control_daemon_LDADD = -lm -lach -lc libdaemonizer.la -lhuboparams

tech_console_SOURCES = src/tech-console.cpp
tech_console_LDADD = -lm -lach -lreadline libhubotech.la libdaemonizer.la libdartlite.la -lhuboparams

proto_manip_daemon_SOURCES = src/proto-manip-daemon.cpp
proto_manip_daemon_LDADD = -lm -lach libhubotech.la libdaemonizer.la libdartlite.la -lhuboparams

oneLeg_SOURCES = ../../hubo-misc/test-code/src/oneLeg.cpp
oneLeg_LDADD = -lm -lach libhubotech.la libdaemonizer.la libdartlite.la -lhuboparams

moveLimbs_SOURCES = ../../hubo-misc/test-code/src/moveLimbs.cpp
moveLimbs_LDADD = -lm -lach libhubotech.la libdaemonizer.la libdartlite.la -lhuboparams

hubo_techdir = /usr/lib
hubo_tech_LTLIBRARIES = libhubotech.la libdaemonizer.la libdartlite.la
libdaemonizer_la_SOURCES = src/daemonizer.c
libdaemonizer_la_LDFLAGS = -module -avoid-version -shared
libdartlite_la_SOURCES = src/dart_Lite/src/dynamics/BodyNodeDynamics.cpp \
src/dart_Lite/src/dynamics/SkeletonDynamics.cpp \
src/dart_Lite/src/kinematics/BodyNode.cpp \
src/dart_Lite/src/kinematics/Dof.cpp \
src/dart_Lite/src/kinematics/Joint.cpp \
src/dart_Lite/src/kinematics/Marker.cpp\
src/dart_Lite/src/kinematics/Shape.cpp\
src/dart_Lite/src/kinematics/Skeleton.cpp\
src/dart_Lite/src/kinematics/Transformation.cpp\
src/dart_Lite/src/kinematics/TrfmRotateAxis.cpp\
src/dart_Lite/src/kinematics/TrfmRotateEuler.cpp\
src/dart_Lite/src/kinematics/TrfmRotateExpmap.cpp\
src/dart_Lite/src/kinematics/TrfmRotateQuat.cpp\
src/dart_Lite/src/kinematics/TrfmTranslate.cpp\
src/dart_Lite/src/robotics/parser/urdf_parser/joint.cpp\
src/dart_Lite/src/robotics/parser/urdf_parser/link.cpp\
src/dart_Lite/src/robotics/parser/urdf_parser/model.cpp\
src/dart_Lite/src/robotics/parser/urdf_parser/pose.cpp\
src/dart_Lite/src/robotics/parser/urdf_parser/twist.cpp \
src/dart_Lite/src/robotics/parser/urdf_parser/urdf_model_state.cpp \
src/dart_Lite/src/robotics/parser/urdf_parser/urdf_sensor.cpp \
src/dart_Lite/src/robotics/parser/urdf_parser/world.cpp \
src/dart_Lite/src/robotics/parser/dart_parser/DartLoader.cpp \
src/dart_Lite/src/robotics/parser/dart_parser/LoaderUtils.cpp \
src/dart_Lite/src/robotics/parser/dart_parser/ToDartUtils.cpp \
src/dart_Lite/src/utils/Timer.cpp \
src/dart_Lite/src/utils/UtilsCode.cpp \
src/dart_Lite/src/utils/UtilsRotation.cpp
libdartlite_la_LDFLAGS = -module -avoid-version -shared -ltinyxml
libhubotech_la_SOURCES = src/Hubo_Tech.cpp
libhubotech_la_LDFLAGS = -module -avoid-version -shared
#libhubotech_la_LIBADD = -lm -lach -lhuboparams
