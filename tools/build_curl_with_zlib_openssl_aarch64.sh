#!/usr/bin/zsh
set -e

# prepare cmake toolchain file
cat > toolchain-edge.cmake << EOF
SET(CMAKE_SYSTEM_NAME Linux)

SET(BIN_DIR /home/liuyk/work/edge/make_tool/aarch64/gcc-linaro-6.2.1-2016.11-x86_64_aarch64-linux-gnu/bin)
SET(CROSS_PREFIX ${BIN_DIR}/aarch64-linux-gnu-)
SET(CMAKE_C_COMPILER  ${BIN_DIR}/aarch64-linux-gnu-gcc)
SET(CMAKE_CXX_COMPILER ${BIN_DIR}/aarch64-linux-gnu-g++)
SET(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
EOF

# Download and extract
wget https://zlib.net/zlib-1.2.12.tar.gz && tar zxf zlib-1.2.12.tar.gz
wget https://github.com/openssl/openssl/archive/refs/tags/openssl-3.0.5.tar.gz && tar zxf openssl-3.0.5.tar.gz
wget https://github.com/curl/curl/releases/download/curl-7_84_0/curl-7.84.0.tar.bz2 && tar jxf curl-7.84.0.tar.bz2

# build zlib
pushd zlib-1.2.12
mkdir build install
pushd build
cmake -DCMAKE_TOOLCHAIN_FILE=/home/liuyk/work/edge/curl/toolchain-edge.cmake -DCMAKE_INSTALL_PREFIX=../install ..
make -j && make install
popd
# delete shared libs
pushd install/lib
rm -f *so*
popd
popd

# build openssl
pushd openssl-openssl-3.0.5
mkdir install
./Configure -fPIC no-asm no-shared linux-aarch64 \
	--cross-compile-prefix=/home/liuyk/work/edge/make_tool/aarch64/gcc-linaro-6.2.1-2016.11-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu- --prefix=/home/liuyk/work/edge/curl/openssl-openssl-3.0.5/install \
	--with-zlib-include=/home/liuyk/work/edge/curl/zlib-1.2.12/install/include \
	--with-zlib-lib=/home/liuyk/work/edge/curl/zlib-1.2.12/install/lib
make -j && make install
popd

# build curl
pushd curl-7.84.0
mkdir install
export PATH=/home/liuyk/work/edge/make_tool/aarch64/gcc-linaro-6.2.1-2016.11-x86_64_aarch64-linux-gnu/bin:$PATH
export AR=aarch64-linux-gnu-ar
export AS=aarch64-linux-gnu-as
export LD=aarch64-linux-gnu-ld
export RANLIB=aarch64-linux-gnu-ranlib
export CC=aarch64-linux-gnu-gcc
export NM=aarch64-linux-gnu-gcc-nm
./configure --target=aarch64-linux-gnu \
	--host=aarch64-linux-gnu \
    --prefix=/home/liuyk/work/edge/curl/curl-7.84.0/install \
    --with-ssl=/home/liuyk/work/edge/curl/openssl-openssl-3.0.5/install \
    --with-zlib=/home/liuyk/work/edge/curl/zlib-1.2.12/install
make -j && make install
readelf -d ./install/lib/libcurl.so.4.8.0
popd

