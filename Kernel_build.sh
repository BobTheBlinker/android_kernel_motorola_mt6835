#!/bin/bash

my_top_dir=$PWD

LOG_FILE=$my_top_dir/KernelBuild.log

exec > >(tee -a "$LOG_FILE") 2>&1

if [ -f "$LOG_FILE" ]; then
    rm $LOG_FILE
fi 



KERNEL_DIR=$my_top_dir/kernel-5.15

REL_KERNEL_OUT="$my_top_dir/out_krn/target/product/mgk_64_entry_level_k515/obj/KERNEL_OBJ"

kernel_out_dir="$my_top_dir/out_krn/target/product/mgk_64_entry_level_k515/obj/KERNEL_OBJ/kernel-5.15"

MODULES_STAGING_DIR=$my_top_dir/out_krn/target/product/mgk_64_entry_level_k515/obj/KERNEL_OBJ/staging

KERNEL_ZIMAGE_OUT="$kernel_out_dir/arch/arm64/boot/Image.gz"

TARGET_KERNEL_CONFIG="$kernel_out_dir/.config"

PATH=${my_top_dir}/prebuilts/build-tools/path/linux-x86:${my_top_dir}/prebuilts/build-tools/linux-x86/bin:$my_top_dir/prebuilts/clang/host/linux-x86/clang-r450784d/bin:$my_top_dir/kernel/prebuilts/kernel-build-tools/linux-x86/bin:$kernel_out_dir/host_tools:$PATH

export CLANG_TRIPLE= CROSS_COMPILE=aarch64-linux-gnu- CROSS_COMPILE_COMPAT= CROSS_COMPILE_ARM32= ARCH=arm64 SUBARCH= MAKE_GOALS=all HOSTCC=clang HOSTCXX=clang++ CC=clang CPP=clang-cpp LLD=ld.lld AR=llvm-ar RANLIB=llvm-ranlib OBJCOPY=llvm-objcopy OBJDUMP=llvm-objdump SIZE=llvm-size READELF=llvm-readelf STRIP=strip

export C_INCLUDE_PATH=$my_top_dir/prebuilts/clang/host/linux-x86/clang-r450784d/lib64/clang/14.0.6/include:/usr/local/include:/usr/include/x86_64-linux-gnu:/usr/include:$my_top_dir/kernel/prebuilts/kernel-build-tools/linux-x86/include:$C_INCLUDE_PATH
export LIBRARY_PATH=$my_top_dir/prebuilts/clang/host/linux-x86/clang-r450784d/lib64/clang/14.0.6/lib:/usr/local/lib:/usr/lib:$my_top_dir/kernel/prebuilts/kernel-build-tools/linux-x86/lib64:$LIBRARY_PATH
export LD_LIBRARY_PATH=$my_top_dir/prebuilts/clang/host/linux-x86/clang-r450784d/lib64/clang/14.0.6/lib:/usr/local/lib:/usr/lib:$my_top_dir/kernel/prebuilts/kernel-build-tools/linux-x86/lib64:$LD_LIBRARY_PATH

mkdir -vp $kernel_out_dir

cd $my_top_dir

python kernel-5.15/scripts/gen_build_config.py --kernel-defconfig mgk_64_k515_defconfig --kernel-defconfig-overlays "" --kernel-build-config-overlays "" -m user -o ${REL_KERNEL_OUT}/build.config

cp -p kernel-5.15/arch/arm64/configs/mgk_64_k515_defconfig ${REL_KERNEL_OUT}/mgk_64_k515.config

cd kernel-5.15

make LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=${kernel_out_dir}/scripts/dtc/dtc O=$kernel_out_dir gki_defconfig ../../../../out_krn/target/product/mgk_64_entry_level_k515/obj/KERNEL_OBJ/mgk_64_k515.config entry_level.config userdebug.config

cd $kernel_out_dir

make O=$kernel_out_dir LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=${kernel_out_dir}/scripts/dtc/dtc all Image.lz4

make -j16 O=$kernel_out_dir LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=${kernel_out_dir}/scripts/dtc/dtc INSTALL_MOD_PATH=$MODULES_STAGING_DIR modules_install

cd -

mkdir -p ${REL_KERNEL_OUT}/vendor/mediatek/kernel_modules/met_drv_v3

make -C ../vendor/mediatek/kernel_modules/met_drv_v3 M=../vendor/mediatek/kernel_modules/met_drv_v3 KERNEL_SRC=$KERNEL_DIR O=${kernel_out_dir} LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=${kernel_out_dir}/scripts/dtc/dtc

make -C ../vendor/mediatek/kernel_modules/met_drv_v3 M=../vendor/mediatek/kernel_modules/met_drv_v3 KERNEL_SRC=$KERNEL_DIR O=${kernel_out_dir} LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=${kernel_out_dir}/scripts/dtc/dtc INSTALL_MOD_PATH=$MODULES_STAGING_DIR INSTALL_MOD_DIR=extra/../vendor/mediatek/kernel_modules/met_drv_v3 INSTALL_HDR_PATH=${REL_KERNEL_OUT}/kernel_uapi_headers/usr modules_install

mkdir -p ${REL_KERNEL_OUT}/vendor/mediatek/kernel_modules/connectivity/common

make -C ../vendor/mediatek/kernel_modules/connectivity/common M=../vendor/mediatek/kernel_modules/connectivity/common KERNEL_SRC=$KERNEL_DIR O=${kernel_out_dir} LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=${kernel_out_dir}/scripts/dtc/dtc

make -C ../vendor/mediatek/kernel_modules/connectivity/common M=../vendor/mediatek/kernel_modules/connectivity/common KERNEL_SRC=$KERNEL_DIR O=${kernel_out_dir} LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=${kernel_out_dir}/scripts/dtc/dtc INSTALL_MOD_PATH=$MODULES_STAGING_DIR INSTALL_MOD_DIR=extra/../vendor/mediatek/kernel_modules/connectivity/common INSTALL_HDR_PATH=${REL_KERNEL_OUT}/kernel_uapi_headers/usr modules_install

mkdir -p ${REL_KERNEL_OUT}/vendor/mediatek/kernel_modules/connectivity/conninfra

make -C ../vendor/mediatek/kernel_modules/connectivity/conninfra M=../vendor/mediatek/kernel_modules/connectivity/conninfra KERNEL_SRC=$KERNEL_DIR O=${kernel_out_dir} LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=${kernel_out_dir}/scripts/dtc/dtc

make -C ../vendor/mediatek/kernel_modules/connectivity/conninfra M=../vendor/mediatek/kernel_modules/connectivity/conninfra KERNEL_SRC=$KERNEL_DIR O=${kernel_out_dir} LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=${kernel_out_dir}/scripts/dtc/dtc INSTALL_MOD_PATH=$MODULES_STAGING_DIR INSTALL_MOD_DIR=extra/../vendor/mediatek/kernel_modules/connectivity/conninfra INSTALL_HDR_PATH=${REL_KERNEL_OUT}/kernel_uapi_headers/usr modules_install

mkdir -p ${REL_KERNEL_OUT}/vendor/mediatek/kernel_modules/connectivity/connfem

make -C ../vendor/mediatek/kernel_modules/connectivity/connfem M=../vendor/mediatek/kernel_modules/connectivity/connfem KERNEL_SRC=$KERNEL_DIR O=${kernel_out_dir} LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=${kernel_out_dir}/scripts/dtc/dtc

make -C ../vendor/mediatek/kernel_modules/connectivity/connfem M=../vendor/mediatek/kernel_modules/connectivity/connfem KERNEL_SRC=$KERNEL_DIR O=${kernel_out_dir} LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=${kernel_out_dir}/scripts/dtc/dtc INSTALL_MOD_PATH=$MODULES_STAGING_DIR INSTALL_MOD_DIR=extra/../vendor/mediatek/kernel_modules/connectivity/connfem INSTALL_HDR_PATH=${REL_KERNEL_OUT}/kernel_uapi_headers/usr modules_install

mkdir -p ${REL_KERNEL_OUT}/vendor/mediatek/kernel_modules/connectivity/gps/gps_stp

make -C ../vendor/mediatek/kernel_modules/connectivity/gps/gps_stp M=../vendor/mediatek/kernel_modules/connectivity/gps/gps_stp KERNEL_SRC=$KERNEL_DIR O=${kernel_out_dir} LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=${kernel_out_dir}/scripts/dtc/dtc

make -C ../vendor/mediatek/kernel_modules/connectivity/gps/gps_stp M=../vendor/mediatek/kernel_modules/connectivity/gps/gps_stp KERNEL_SRC=$KERNEL_DIR O=${kernel_out_dir} LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=${kernel_out_dir}/scripts/dtc/dtc INSTALL_MOD_PATH=$MODULES_STAGING_DIR INSTALL_MOD_DIR=extra/../vendor/mediatek/kernel_modules/connectivity/gps/gps_stp INSTALL_HDR_PATH=${REL_KERNEL_OUT}/kernel_uapi_headers/usr modules_install

mkdir -p ${REL_KERNEL_OUT}/vendor/mediatek/kernel_modules/connectivity/gps/data_link/plat/v010

make -C ../vendor/mediatek/kernel_modules/connectivity/gps/data_link/plat/v010 M=../vendor/mediatek/kernel_modules/connectivity/gps/data_link/plat/v010 KERNEL_SRC=$KERNEL_DIR O=${kernel_out_dir} LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=${kernel_out_dir}/scripts/dtc/dtc

make -C ../vendor/mediatek/kernel_modules/connectivity/gps/data_link/plat/v010 M=../vendor/mediatek/kernel_modules/connectivity/gps/data_link/plat/v010 KERNEL_SRC=$KERNEL_DIR O=${kernel_out_dir} LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=${kernel_out_dir}/scripts/dtc/dtc INSTALL_MOD_PATH=$MODULES_STAGING_DIR INSTALL_MOD_DIR=extra/../vendor/mediatek/kernel_modules/connectivity/gps/data_link/plat/v010 INSTALL_HDR_PATH=${REL_KERNEL_OUT}/kernel_uapi_headers/usr modules_install

mkdir -p ${REL_KERNEL_OUT}/vendor/mediatek/kernel_modules/connectivity/gps/data_link/plat/v030

make -C ../vendor/mediatek/kernel_modules/connectivity/gps/data_link/plat/v030 M=../vendor/mediatek/kernel_modules/connectivity/gps/data_link/plat/v030 KERNEL_SRC=$KERNEL_DIR O=${kernel_out_dir} LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=${kernel_out_dir}/scripts/dtc/dtc

make -C ../vendor/mediatek/kernel_modules/connectivity/gps/data_link/plat/v030 M=../vendor/mediatek/kernel_modules/connectivity/gps/data_link/plat/v030 KERNEL_SRC=$KERNEL_DIR O=${kernel_out_dir} LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=${kernel_out_dir}/scripts/dtc/dtc INSTALL_MOD_PATH=$MODULES_STAGING_DIR INSTALL_MOD_DIR=extra/../vendor/mediatek/kernel_modules/connectivity/gps/data_link/plat/v030 INSTALL_HDR_PATH=${REL_KERNEL_OUT}/kernel_uapi_headers/usr modules_install

mkdir -p ${REL_KERNEL_OUT}/vendor/mediatek/kernel_modules/connectivity/gps/data_link/plat/v050

make -C ../vendor/mediatek/kernel_modules/connectivity/gps/data_link/plat/v050 M=../vendor/mediatek/kernel_modules/connectivity/gps/data_link/plat/v050 KERNEL_SRC=$KERNEL_DIR O=${kernel_out_dir} LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=${kernel_out_dir}/scripts/dtc/dtc

make -C ../vendor/mediatek/kernel_modules/connectivity/gps/data_link/plat/v050 M=../vendor/mediatek/kernel_modules/connectivity/gps/data_link/plat/v050 KERNEL_SRC=$KERNEL_DIR O=${kernel_out_dir} LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=${kernel_out_dir}/scripts/dtc/dtc INSTALL_MOD_PATH=$MODULES_STAGING_DIR INSTALL_MOD_DIR=extra/../vendor/mediatek/kernel_modules/connectivity/gps/data_link/plat/v050 INSTALL_HDR_PATH=${REL_KERNEL_OUT}/kernel_uapi_headers/usr modules_install

mkdir -p ${REL_KERNEL_OUT}/vendor/mediatek/kernel_modules/connectivity/gps/gps_pwr

make -C ../vendor/mediatek/kernel_modules/connectivity/gps/gps_pwr M=../vendor/mediatek/kernel_modules/connectivity/gps/gps_pwr KERNEL_SRC=$KERNEL_DIR O=${kernel_out_dir} LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=${kernel_out_dir}/scripts/dtc/dtc

make -C ../vendor/mediatek/kernel_modules/connectivity/gps/gps_pwr M=../vendor/mediatek/kernel_modules/connectivity/gps/gps_pwr KERNEL_SRC=$KERNEL_DIR O=${kernel_out_dir} LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=${kernel_out_dir}/scripts/dtc/dtc INSTALL_MOD_PATH=$MODULES_STAGING_DIR INSTALL_MOD_DIR=extra/../vendor/mediatek/kernel_modules/connectivity/gps/gps_pwr INSTALL_HDR_PATH=${REL_KERNEL_OUT}/kernel_uapi_headers/usr modules_install

mkdir -p ${REL_KERNEL_OUT}/vendor/mediatek/kernel_modules/connectivity/gps/gps_scp

make -C ../vendor/mediatek/kernel_modules/connectivity/gps/gps_scp M=../vendor/mediatek/kernel_modules/connectivity/gps/gps_scp KERNEL_SRC=$KERNEL_DIR O=${kernel_out_dir} LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=${kernel_out_dir}/scripts/dtc/dtc

make -C ../vendor/mediatek/kernel_modules/connectivity/gps/gps_scp M=../vendor/mediatek/kernel_modules/connectivity/gps/gps_scp KERNEL_SRC=$KERNEL_DIR O=${kernel_out_dir} LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=${kernel_out_dir}/scripts/dtc/dtc INSTALL_MOD_PATH=$MODULES_STAGING_DIR INSTALL_MOD_DIR=extra/../vendor/mediatek/kernel_modules/connectivity/gps/gps_scp INSTALL_HDR_PATH=${REL_KERNEL_OUT}/kernel_uapi_headers/usr modules_install

mkdir -p ${REL_KERNEL_OUT}/vendor/mediatek/kernel_modules/gpu

make -C ../vendor/mediatek/kernel_modules/gpu M=../vendor/mediatek/kernel_modules/gpu KERNEL_SRC=$KERNEL_DIR O=${kernel_out_dir} LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=${kernel_out_dir}/scripts/dtc/dtc

make -C ../vendor/mediatek/kernel_modules/gpu M=../vendor/mediatek/kernel_modules/gpu KERNEL_SRC=$KERNEL_DIR O=${kernel_out_dir} LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=${kernel_out_dir}/scripts/dtc/dtc INSTALL_MOD_PATH=$MODULES_STAGING_DIR INSTALL_MOD_DIR=extra/../vendor/mediatek/kernel_modules/gpu INSTALL_HDR_PATH=${REL_KERNEL_OUT}/kernel_uapi_headers/usr modules_install

mkdir -p ${REL_KERNEL_OUT}/vendor/mediatek/kernel_modules/connectivity/fmradio

make -C ../vendor/mediatek/kernel_modules/connectivity/fmradio M=../vendor/mediatek/kernel_modules/connectivity/fmradio KERNEL_SRC=$KERNEL_DIR O=${kernel_out_dir} LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=${kernel_out_dir}/scripts/dtc/dtc

make -C ../vendor/mediatek/kernel_modules/connectivity/fmradio M=../vendor/mediatek/kernel_modules/connectivity/fmradio KERNEL_SRC=$KERNEL_DIR O=${kernel_out_dir} LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=${kernel_out_dir}/scripts/dtc/dtc INSTALL_MOD_PATH=$MODULES_STAGING_DIR INSTALL_MOD_DIR=extra/../vendor/mediatek/kernel_modules/connectivity/fmradio INSTALL_HDR_PATH=${REL_KERNEL_OUT}/kernel_uapi_headers/usr modules_install
