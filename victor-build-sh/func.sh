
#
# common function
# 

checkconf() {
	if [ ! -f $OUTPUT/.config ]; then
		echo "Kernel configuration is not exist !"
		echo "You must first do '$0 conf'."
		exit 1
	fi
}

setconf() {
	echo "Configure default configuration ..."
	$BUILDSH O=$OUTPUT $CONF
}

makeimage() {
	echo "Build kernel Image ..."
	checkconf
	$BUILDSH O=$OUTPUT Image
}

makemodules() {
	echo "Build kernel module drivers ..."
	checkconf
	$BUILDSH O=$OUTPUT modules
}

makedtbs() {
	echo "Build device tree ..."
	checkconf
	$BUILDSH O=$OUTPUT dtbs
}

do_modules_install() {
	echo "Install kernel module drivers ..."
	checkconf
	$BUILDSH O=$OUTPUT modules_install
}

do_others() {
	echo "Do $@ ..."
	checkconf
	$BUILDSH O=$OUTPUT $@
}

usage() {
	echo "Usage: $0 options"
	echo "\tconf\tset to default kernel configuration"
	echo "\timage\tbuild kernel Image"
	echo "\tmodules\tbuild kernel module drivers"
	echo "\tmodules_install\tinstall kernel module drivers"
	echo "\tdistclean\tremove all output and build"
	echo "\tdo something input"
	echo "\tall\tdo all above"
	echo "\trebuildall\tremove all and rebuild all"
	echo "\thelp\tshow this message"
}

for i in $@ ;do
	case "$i" in
	conf)
		setconf
		;;
	image)
		makeimage
		;;
	dtbs)
		makedtbs
		;;
	modules)
		makemodules
        	;;
	modules_install)
		do_modules_install
		;;
	all)
		setconf
		makeimage
		makedtbs
		makemodules
		do_modules_install
		;;
	rebuildall)
		rm -fr $OUTPUT
		setconf
		makeimage
		makemodules
		do_modules_install
		;;
	distclean)
		echo "Remove all output and build !"
		rm -fr $OUTPUT
		;;
	help)
		usage
		;;
	*)
		if [ x"$@" != "" ]; then
			do_others $@
		else
			echo "Error option !!!"
			usage
		fi
		;;
	esac
done
