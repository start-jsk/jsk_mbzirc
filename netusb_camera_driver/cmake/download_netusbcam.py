#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import cStringIO
import logging
import os
import platform
import shutil
import subprocess
import sys
import urllib2
import zipfile

logging.basicConfig(level=logging.INFO)
log = logging.getLogger(os.path.basename(__file__))

# global variables
VERSION="1.39"
if platform.machine() == "x86_64":
    ARCH="amd64"
elif platform.machine() == "i386":
    ARCH="i386"
else:
    log.fatal("architecture {arch} is not supported.".format(arch=platform.machine()))
    sys.exit(1)
DRIVER_URL="http://www.mvision.co.jp/DL/net3/3iCube_Linux_{version}.zip".format(version=VERSION)
DEB_PATH="3iCube_Linux_{version}/03_Driver/netusbcam_{version}-1_{arch}_libudev.deb".format(version=VERSION,
                                                                                            arch=ARCH)
SHARED_LIB_PATH="usr/lib/libNETUSBCAM.so"
INCLUDE_DIR_PATH="usr/include"
LIBRARY_DESTINATION = sys.argv[1]
INCLUDE_DIR_DESTINATION = sys.argv[2]

log.info("Downloading driver")
res = urllib2.urlopen(DRIVER_URL)
if res.code != 200:
    log.fatal("failed to download from {url}: return code: {code}, msg: {msg}".format(url=res.url,
                                                                                          code=res.code,
                                                                                          msg=res.msg))
    sys.exit(1)

log.info("Unarchiving tarball")
with zipfile.ZipFile(cStringIO.StringIO(res.read())) as z:
    z.extractall()
subprocess.check_call(['dpkg', '--extract', DEB_PATH, '.'])

log.info("Copying library")
src_dir = os.path.abspath(os.path.dirname(SHARED_LIB_PATH))
dst_dir = os.path.abspath(os.path.dirname(LIBRARY_DESTINATION))
for src_file in os.listdir(src_dir):
    src_path = os.path.join(src_dir, src_file)
    dst_path = os.path.join(dst_dir, src_file)
    if os.path.islink(src_path):
        link_dst = os.readlink(src_path)
        os.symlink(link_dst, dst_path)
    else:
        shutil.copy(src_path, dst_path)
    log.info("    %s => %s" % (src_path, dst_path))

log.info("Copying header files")
for f in os.listdir(INCLUDE_DIR_PATH):
    if f.endswith(".h") or f.endswith(".hpp") or f.endswith(".hh"):
        if not os.path.exists(INCLUDE_DIR_DESTINATION):
            log.info("    make directory: %s" % INCLUDE_DIR_DESTINATION)
            os.makedirs(INCLUDE_DIR_DESTINATION)
        src_path = os.path.join(INCLUDE_DIR_PATH, f)
        dst_path = os.path.join(INCLUDE_DIR_DESTINATION, f)
        shutil.copy(src_path, dst_path)
        log.info("    %s => %s" % (src_path, dst_path))

