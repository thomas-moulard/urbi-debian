# Files to ship.
EXTRA_DIST +=					\
  $(call ls_files,@{system/ros}/doc/*)		\
  $(call ls_files,@{system/ros}/scripts/*)	\
  $(call ls_files,@{system/ros}/tests/*)

if BUILD_ros
module_LTLIBRARIES += urbi/ros.la

urbi_ros_la_SOURCES =				\
  algorithm/md5/md5-impl.hh			\
  algorithm/md5/md5-impl.cc			\
  system/ros/src/common-msg.hh			\
  system/ros/src/common-msg.hxx			\
  system/ros/src/common-msg.cc			\
  system/ros/src/ros-overload.hh		\
  system/ros/src/ros-overload.hxx		\
  system/ros/src/ros-overload.cc		\
  system/ros/src/ros-type-node.hh		\
  system/ros/src/ros-type-node.hxx		\
  system/ros/src/ros-type-node.cc		\
  system/ros/src/service.hh			\
  system/ros/src/service.hxx			\
  system/ros/src/service.cc			\
  system/ros/src/service-msg.hh			\
  system/ros/src/service-msg.hxx		\
  system/ros/src/service-msg.cc			\
  system/ros/src/tools.hh			\
  system/ros/src/tools.hxx			\
  system/ros/src/tools.cc			\
  system/ros/src/topic-msg.hh			\
  system/ros/src/topic-msg.hxx			\
  system/ros/src/topic-msg.cc			\
  system/ros/src/topic.hh			\
  system/ros/src/topic.hxx			\
  system/ros/src/topic.cc

urbi_ros_la_LIBADD =				\
  $(AM_LIBADD)					\
  $(ros_LIBADD)					\
  $(URBI_SDK_LIBS)

urbi_ros_la_LDFLAGS =				\
  -module -avoid-version			\
  $(ros_LDFLAGS)				\
  $(AM_LDFLAGS) $(URBI_SDK_LDFLAGS)

urbi_ros_la_CPPFLAGS =				\
  $(AM_CPPFLAGS)				\
  $(ros_CPPFLAGS)				\
  $(URBI_SDK_CPPFLAGS)

# Compile our ROS packages used by the test suite.
all: ros-packages
ROS_PACKAGE_PATH = $$ROS_PACKAGE_PATH:$(abs_builddir)/system/ros/tests
ROS_PACKAGES = uservices utopics
ros-packages:
## It seems that the ROS build system does not support vpath builds,
## or at least, not easily.  So copy the src tree in the build tree,
## and compile there.  Beware that in distcheck, the source tree is
## read-only: restore proper permissions.
	mkdir -p system/ros/tests
	rsync -av $(modules_srcdir)/system/ros/tests/rosstack system/ros/tests
	chmod -R u+wX system/ros/tests
	export ROS_PACKAGE_PATH=$(ROS_PACKAGE_PATH);			\
	for d in $(ROS_PACKAGES);					\
	do								\
	  $(MAKE) $(AM_MAKEFLAGS) -C system/ros/tests/rosstack/$$d;	\
	done

endif BUILD_ros

clean-local:
	for d in $(ROS_PACKAGES);					  \
	do								  \
	  $(MAKE) $(AM_MAKEFLAGS) -C system/ros/tests/rosstack/$$d clean; \
	done

.PHONY: ros-packages
