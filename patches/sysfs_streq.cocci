// Look for occasions where sysfs_streq would be better than trying to match /0 /n

/*
 spatch --sp-file patches/sysfs_streq.cocci \
	./arch/sparc/kernel/led.c \
	./arch/frv/kernel/sysctl.c \
	./drivers/misc/enclosure.c \
	./arch/powerpc/kernel/ibmebus.c \
	./drivers/staging/speakup/kobjects.c \
	./drivers/usb/gadget/configfs.c \
	./drivers/net/ethernet/chelsio/cxgb4/cxgb4_debugfs.c \
	./net/batman-adv/sysfs.c
*/

/* Look for new line conversions */
@@
char * buff;
identifier len;
statement s;
@@
* if (buff[len - 1] == '\n')
*     buff[len - 1] = '\0';


