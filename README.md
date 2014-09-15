beckhoff CX-1000
========

Linux kernel level driver to allow access to the I/O of the Beckhoff CX-1100 power supply used with the Beckhoff CX-1000 embedded PC. This allows access to the K-Bus and IP-Bus of the Beckhoff system.

This repo is imported from my original SourceForge repo, https://sourceforge.net/projects/beckhoffdriver/

The kernel level driver has a serious bug that allows access past the memory limits of the dual port ram of the CX-1100.

The driver should be ported to the 3.0 kernel at some stage or preferably added as a bitbake recipe to a yocto build.
