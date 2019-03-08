execute_process(COMMAND "/home/rachel/rawhide/rawhide_ws/build/sawyer_pykdl/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/rachel/rawhide/rawhide_ws/build/sawyer_pykdl/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
