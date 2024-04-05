#
# Copyright (c) 2024. Created on 4/5/24 12:21 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
# geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
# the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
# systems and multi-sensor fusion.
#

# update submodules
echo "----------------------------------------------------"
echo "update submodules remotely, it may take some time..."
echo "----------------------------------------------------"
git submodule update --init --recursive
git submodule update --remote --recursive
if [ $? -ne 0 ]; then
    echo "--------------------------------------------"
    echo "error occurs when updating submodules, exit!"
    echo "--------------------------------------------"
    exit
fi

# shellcheck disable=SC2046
RVIER_ROOT_PATH=$(cd $(dirname $0) || exit; pwd)
echo "the root path of 'RIs-Calib': ${RVIER_ROOT_PATH}"

# build tiny-viewer
echo "----------------------------------"
echo "build thirdparty: 'tiny-viewer'..."
echo "----------------------------------"

mkdir ${RVIER_ROOT_PATH}/thirdparty/ctraj/thirdparty/tiny-viewer-build
# shellcheck disable=SC2164
cd "${RVIER_ROOT_PATH}"/thirdparty/ctraj/thirdparty/tiny-viewer-build

cmake ../tiny-viewer
echo current path: $PWD
echo "-----------------------------"
echo "start making 'tiny-viewer'..."
echo "-----------------------------"
make -j8

# build ctraj
echo "----------------------------"
echo "build thirdparty: 'ctraj'..."
echo "----------------------------"

mkdir ${RVIER_ROOT_PATH}/thirdparty/ctraj-build
# shellcheck disable=SC2164
cd "${RVIER_ROOT_PATH}"/thirdparty/ctraj-build

cmake ../ctraj
echo current path: $PWD
echo "-----------------------"
echo "start making 'ctraj'..."
echo "-----------------------"
make -j8