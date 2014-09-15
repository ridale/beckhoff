beckhoff CX-1000
========
Linux kernel level driver to allow access to the I/O of the Beckhoff CX-1100 power supply used with the Beckhoff CX-1000 embedded PC. This allows access to the K-Bus and IP-Bus of the Beckhoff system.

This repo is imported from my original SourceForge repo, https://sourceforge.net/projects/beckhoffdriver/

While writing the example code I found a nasty bug in the kernel driver code the call to the ioctl IOCTL_BHKLDRV_BOARDMAP does not check the input and can be overflowed causing kernel problems when you try to read. Easily fixed in the driver with a check of the input data.

The driver should be ported to the 3.0 kernel at some stage or preferably added as a bitbake recipe to a yocto build.
