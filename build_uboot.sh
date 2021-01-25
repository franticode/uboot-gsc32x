export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabihf-

make clean
make distclean

if [ "$1" = "plus" ]
then
	#plus
	echo "****************** build imx6q plus ******************"
	cd board/freescale/mx6sabresd
	#cp mx6qp.cfg_1G mx6qp.cfg
	#cp mx6qp.cfg_2G mx6qp.cfg
	cd -

	make mx6qpsabresdandroid_config
else
	#6q
	echo "****************** build imx6q ******************"
	#make mx6qsabresdandroid_config
	cd board/freescale/mx6sabresd
	#cp mx6q_4x_mt41j128.cfg_1G mx6q_4x_mt41j128.cfg
	#cp mx6q_4x_mt41j128.cfg_2G mx6q_4x_mt41j128.cfg
	cd -

	make mx6qsabresd_defconfig
fi

make -j6 -s
