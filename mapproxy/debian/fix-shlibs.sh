#!/bin/bash

function fix_focal() {
    # change to base distro version
    sed -i 's/4.2.0+melown/4.2.0+dfsg-5/g' $1
}

function fix() {
    case ${DEB_RELEASE} in
        focal) fix_focal $1;;
    esac
}

for file in debian/*.substvars; do
    fix ${file}
done
