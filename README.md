# meta-tests
OE meta layer for test applications

## Preparation

Integrate the meta-tests layer into your Yocto environment.


## File Descriptor Communication between Processes

### Overview

Simple test application for passing a file descriptor (opened file or ION file descriptor) between processes using socket. In order to test passing the ION buffer, ION support is required on your target.


### Customization

By default the test is performed on a file descriptor from an allocated ION buffer. To perform the test with an opened file descriptor, enable the SHARE_OPENED_FILE definition in the client and server source code. On the target, create the file by running:

```
dd if=/dev/random of=/tmp/anyfile.bin count=1 bs=16
```
