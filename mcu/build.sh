#!/bin/bash

pwd=`pwd`
while getopts v:t: flag
do
    case "${flag}" in
        v) version=${OPTARG};;
        t) target=${OPTARG};;
    esac
done
echo "Version: $version";
echo "Build: $target";

if [ -z "$version" ]
then
      echo "\$version is empty"
      exit 1
fi

if [ -z "$target" ]
then
      echo "\$target is empty"
      exit 1
fi
cp -rvf $pwd/board/${target}/config/config $pwd/.config
make clean
mkdir out
cp -rvf $pwd/board/${target}/cmd/compile_time_request.c $pwd/out/compile_time_request.c
make -j

if [ $? -ne 0 ]; then
    echo "Build failed"
    exit 1
fi

#加密文件添加头的C程序编译
if [ ! -f "$pwd/scripts/add_magic.c" ];then
    echo "##############add_magic.c not exist##############"
    exit 1
fi
gcc -o $pwd/out/add_magic $pwd/scripts/add_magic.c -lcrypto
if [ $? -ne 0 ]; then
    echo " gcc -o $pwd/out/add_magic $pwd/scripts/add_magic.c -lcrypto error"
    exit 1
fi
#获取版本号
if [[ $version =~ ([0-9]+)\.([0-9]+)\.([0-9]+) ]]; then
    major_version=${BASH_REMATCH[1]}
    minor_version=${BASH_REMATCH[2]}
    patch_version=${BASH_REMATCH[3]}
    echo "Major version: $major_version, Minor version: $minor_version, Patch version: $patch_version"
else
    echo "The format of version is not x.y.z"
fi

#获取机型
if [ -n "$target" ]; then
    if [ "$target" == "e100" ]; then
        $pwd/out/add_magic $pwd/out/klipper.bin $pwd/out/upgrade_"$target"_"$version".bin 20 24 01 26 $major_version $minor_version $patch_version 01 01 00 00 00
    else
        $pwd/out/add_magic $pwd/out/klipper.bin $pwd/out/upgrade_"$target"_"$version".bin 20 24 01 26 $major_version $minor_version $patch_version 255 01 00 00 00
    fi
else
    $pwd/out/add_magic $pwd/out/klipper.bin $pwd/out/upgrade_"$target"_"$version".bin 20 24 01 26 $major_version $minor_version $patch_version 255 01 00 00 00
fi

cp -rvf $pwd/board/${target}/bootloader/bootloader.bin $pwd/out/bootloader.bin
# 获取原始文件的大小
original_size=$(stat -c%s $pwd/out/bootloader.bin)

# 计算需要添加的大小
append_size=$((32*1024 - original_size))

# 生成一个包含0xff的文件
printf "\xff%.0s" $(seq 1 $append_size) > $pwd/out/append_bootloader_file.bin

# 将这个文件追加到原始文件的末尾
cat $pwd/out/append_bootloader_file.bin >> $pwd/out/bootloader.bin

# 将bootlaode完整包和原始bin文件连接起来，生成新的bin文件
cat $pwd/out/bootloader.bin $pwd/out/upgrade_"$target"_"$version".bin > $pwd/out/upgrade_"$target"_"$version"_full_pack.bin
