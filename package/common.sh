##
## Copyright (c) 2025 Hanson Robotics.
##
## This file is part of Hanson AI.
## See https://www.hansonrobotics.com/hanson-ai for further info.
##
## Licensed under the MIT License.
## See LICENSE file in the project root for full license information.
##

env() {
    export HR_PREFIX=/opt/hansonrobotics
    export HR_BIN_PREFIX=$HR_PREFIX/bin
    export HR_ROS_PREFIX=${HR_PREFIX}/ros
    export HR_DATA_PREFIX=$HR_PREFIX/data
    export URL_PREFIX=https://github.com/hansonrobotics
    export VENDOR="Hanson Robotics"
}

install_deps() {
    if ! hash gem >/dev/null 2>&1; then
        echo "Installing ruby-full"
        sudo apt-get install ruby-full
    fi

    if ! hash fpm >/dev/null 2>&1; then
        gem install fpm
        gem install deb-s3
    fi

    if ! hash chrpath >/dev/null 2>&1; then
        echo "Installing chrpath"
        sudo apt-get install chrpath
    fi

    if ! hash autoconf >/dev/null 2>&1; then
        echo "Installing autoconf"
        sudo apt-get install autoconf
    fi

    if ! hash jq >/dev/null 2>&1; then
        echo "Installing jq"
        sudo apt-get install jq
    fi

    if [[ ! -f /usr/local/go/bin/go ]]; then
        echo "Installing go"
        wget https://dl.google.com/go/go1.14.2.linux-amd64.tar.gz -O /tmp/go1.14.2.linux-amd64.tar.gz
        sudo tar -C /usr/local -xzf /tmp/go1.14.2.linux-amd64.tar.gz
    fi

    export PATH=/usr/local/go/bin:$PATH
}

COLOR_INFO='\033[32m'
COLOR_WARN='\033[33m'
COLOR_ERROR='\033[31m'
COLOR_RESET='\033[0m'
info() {
    printf "${COLOR_INFO}[INFO] ${1}${COLOR_RESET}\n" >&2
}
warn() {
    printf "${COLOR_WARN}[WARN] ${1}${COLOR_RESET}\n" >&2
}
error() {
    printf "${COLOR_ERROR}[ERROR] ${1}${COLOR_RESET}\n" >&2
}

source_ros() {
    local ros_dists=(noetic melodic kinetic indigo)
    for ros_dist in ${ros_dists[@]}; do
        if [[ -e /opt/ros/$ros_dist/setup.bash ]]; then
            info "ROS distribution $ros_dist"
            source /opt/ros/$ros_dist/setup.bash
            return
        fi
    done
}

add_control_scripts() {
    local root_dir=${1:-${PACKAGE_DIR}/control}
    local preinst="${root_dir}/preinst.sh"
    local postinst="${root_dir}/postinst.sh"
    local prerm="${root_dir}/prerm.sh"
    local postrm="${root_dir}/postrm.sh"

    local ms=""
    [[ -f ${preinst} ]] && ms="$ms --before-install ${preinst}"
    [[ -f ${postinst} ]] && ms="$ms --after-install ${postinst}"
    [[ -f ${prerm} ]] && ms="$ms --before-remove ${prerm}"
    [[ -f ${postrm} ]] && ms="$ms --after-remove ${postrm}"

    if [[ -z $ms ]]; then
        echo "Empty maintainer scripts"
        return 1
    fi
    echo $ms
}

cleanup_ros_package_build() {
    # clean up
    pushd $1 >/dev/null
    rm -r src build_isolated devel_isolated .catkin_workspace install
    popd >/dev/null
}

get_version() {
    local date=$(date +%Y%m%d%H%M%S)
    local version_file=$BASEDIR/src/$reponame/version
    local tag=$(git describe --tags --candidates=0)
    if [[ -f $version_file ]]; then
        version=$(head -n 1 $version_file)
        # if 1 is present or the latest tag equals to version
        if [[ $1 != 1 && ${tag#v} != $version ]]; then
            version=${version}-${date}
        fi
    else
        version=$date
    fi
}

env
install_deps
