#!/bin/sh

[ -d build ] && rm -rf build
sphinx-autobuild source build
