#
# This file was derived from the 'Hello World!' example recipe in the
# Yocto Project Development Manual.
#
SUMMARY = "FD Tests"
SECTION = "tests"
LICENSE = "BSD-2-Clause"
LIC_FILES_CHKSUM = "file://LICENSE;md5=08950a6ae0da875ca581a843039f2a8b"

SRC_URI = "file://server.c \
           file://client.c"

S = "${WORKDIR}"

TARGET_CC_ARCH += "${LDFLAGS}"

do_compile() {
         ${CC} server.c -o fd-tests_server
         ${CC} client.c -o fd-tests_client
}

do_install() {
         install -d ${D}${bindir}
         install -m 0755 fd-tests_server ${D}${bindir}
         install -m 0755 fd-tests_client ${D}${bindir}
}

