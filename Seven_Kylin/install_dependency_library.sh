set -e

echo -e "\n===== 🔍 系统环境检查 ====="

if grep -q "Kylin" /etc/os-release; then
    echo -e " 当前系统：银河麒麟V10"
else
    echo -e " 警告：非麒麟V10系统，编译结果可能无法运行！"
    read -p "是否继续编译？(y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

ARCH=$(uname -m)
echo -e " 当前架构：$ARCH"

echo -e "\n===== 📦 安装编译依赖 ====="
sudo apt install -y gcc g++ cmake make libeigen3-dev libjsoncpp-dev pkg-config

echo -e "\n===== 安装完成 ====="