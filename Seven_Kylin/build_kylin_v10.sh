#!/bin/bash
# 麒麟V10控制台程序一键编译&安装脚本
# 使用方式：sudo ./build_kylin_v10.sh

set -e # 出错立即退出


echo -e "\n===== 🔍 系统环境检查 ====="
# 检查是否为麒麟V10
if grep -q "Kylin" /etc/os-release; then
    echo -e "✅ 当前系统：银河麒麟V10"
else
    echo -e "⚠️ 警告：非麒麟V10系统，编译结果可能无法运行！"
    read -p "是否继续编译？(y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# 检查架构
ARCH=$(uname -m)
echo -e "✅ 当前架构：$ARCH"

# ========== 安装依赖（已修复麒麟V10桌面版） ==========
echo -e "\n===== 📦 安装编译依赖 ====="
sudo apt install -y gcc g++ cmake make libeigen3-dev libjsoncpp-dev pkg-config

# ========== 编译构建 ==========
echo -e "\n===== 🔨 开始编译（Debug 模式） ====="
rm -rf build && mkdir -p build && cd build

# 关键修改：Debug + 运行时搜索当前目录
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    #-DCMAKE_INSTALL_PREFIX="../install" \
    #-DCMAKE_INSTALL_RPATH="\$ORIGIN"

make -j$(nproc)

# ========== 安装程序（安装到本地 install 目录） ==========
echo -e "\n===== 🚀 安装程序 ====="
make install

# ========== 验证结果 ==========
echo -e "\n===== ✅ 编译完成 ====="
echo "✅ 可执行文件：install/bin/seven-cli"
echo "✅ 依赖库也会被安装到：install/bin/"
echo "💡 直接运行：cd install/bin && ./seven-cli"

cd ..
echo -e "\n===== 📢 编译完成 ====="
