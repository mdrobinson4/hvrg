#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/softrobot/mark/hvrg/catkin_ws/src/imu_pp"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/softrobot/mark/hvrg/catkin_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/softrobot/mark/hvrg/catkin_ws/install/lib/python2.7/dist-packages:/home/softrobot/mark/hvrg/catkin_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/softrobot/mark/hvrg/catkin_ws/build" \
    "/usr/bin/python2" \
    "/home/softrobot/mark/hvrg/catkin_ws/src/imu_pp/setup.py" \
    build --build-base "/home/softrobot/mark/hvrg/catkin_ws/build/imu_pp" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/softrobot/mark/hvrg/catkin_ws/install" --install-scripts="/home/softrobot/mark/hvrg/catkin_ws/install/bin"
