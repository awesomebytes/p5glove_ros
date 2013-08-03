p5glove_ros
===========

We will work on resurrect the driver for the p5glove and pass it's data via ROS messages.

The ROS package here builds and has a rosdep for Ubuntu 12.04 defined.





-----------
Start
What we got until now:

How to compile old p5 glove driver:
Info: http://www.noisybox.net/computers/p5glove/

Driver: http://www.noisybox.net/computers/p5glove/libp5glove_svn_20050206.tar.gz

sudo apt-get install automake1.9
aclocal
autoheader
automake --add-missing
autoconf
./configure
