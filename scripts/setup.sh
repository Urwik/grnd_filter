#!/bin/bash

script_dir="$(dirname "$0")"
parent_dir="$(dirname "$script_dir")"
parent_dir="$(realpath "$parent_dir")"
build_dir="$parent_dir/build"
echo "Addint to PATH build dir located in: $build_dir"
export PATH=$PATH:$build_dir
