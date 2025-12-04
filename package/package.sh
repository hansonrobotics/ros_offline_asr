#!/usr/bin/env bash
##
## Copyright (c) 2025 Hanson Robotics.
##
## This file is part of Hanson AI.
## See https://www.hansonrobotics.com/hanson-ai for further info.
##
## Licensed under the MIT License.
## See LICENSE file in the project root for full license information.
##

package() {
    local reponame=ros_offline_asr

    mkdir -p $BASEDIR/src
    rsync -r --delete \
        --exclude ".git" \
        --exclude "package" \
        $BASEDIR/../ $BASEDIR/src/$reponame

    get_version $1
    source_ros
    catkin_make_isolated --directory $BASEDIR --install --install-space $BASEDIR/install -DCMAKE_BUILD_TYPE=Release

    local name=head-ros-offline-asr
    local desc="ROS Offline Speech recognition"
    local url="https://api.github.com/repos/hansonrobotics/$reponame/releases"
    
    #pushd "$BASEDIR/src/$reponame" >/dev/null
    #./download_models.sh
    #popd >/dev/null

    fpm -C "${BASEDIR}" -s dir -t deb -n "${name}" -v "${version#v}" --vendor "${VENDOR}" \
        --url "${url}" --description "${desc}" ${ms} \
        --deb-no-default-config-files \
        -p $BASEDIR/${name}_VERSION_ARCH.deb \
        install/share=${HR_ROS_PREFIX}/ \
        install/lib=${HR_ROS_PREFIX}/

    cleanup_ros_package_build $BASEDIR
}

if [[ $(readlink -f ${BASH_SOURCE[0]}) == $(readlink -f $0) ]]; then
    BASEDIR=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
    source $BASEDIR/common.sh
    set -e
    package $1
fi
