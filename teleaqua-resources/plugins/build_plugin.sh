#!/bin/bash

# build_plugins_simple.sh

set -e

echo "编译Gazebo插件..."

failed=0

for plugin in bidir_motor_model hydrodynamics hybrid_air_propeller_model; do
    echo "处理: $plugin"

    if [[ ! -d "$plugin" ]]; then
        echo "✗ $plugin 不存在"
        failed=1
        continue
    fi

    pushd "$plugin" > /dev/null

    rm -rf build
    mkdir build
    pushd build > /dev/null

    if cmake .. && make -j$(nproc); then
        echo "✓ $plugin 成功"
    else
        echo "✗ $plugin 失败"
        failed=1
    fi

    popd > /dev/null
    popd > /dev/null
done

if [[ "$failed" -ne 0 ]]; then
    echo "插件编译失败，请先修复上面的错误。"
    exit 1
fi

echo "完成!"
