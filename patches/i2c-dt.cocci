// Remove all I2C Device Tables, when there is an OF table present

// Ensure that we are in a driver with OF device mappings
@ of_dt_present @
@@
#include <linux/of.h>

@ of_dev_id_present @
identifier arr;
@@
struct of_device_id arr[] = { ... };

// Remove the i2c_device_id array

@ dev_id depends on of_dev_id_present @
identifier arr;
@@
- struct i2c_device_id arr[] = { ... };


// Now remove the MODULE_DEVICE_TABLE entry
@ depends on dev_id @
declarer name MODULE_DEVICE_TABLE;
identifier i2c;
identifier dev_id.arr;
@@
- MODULE_DEVICE_TABLE(i2c, arr);


// Remove the i2c_device_id reference
@ depends on dev_id @
identifier drv;
identifier dev_id.arr;
@@
static struct i2c_driver drv = {
	...,
-	.id_table	= arr,
	...,
};


// Temporarily rename the probe to our temp .probe2

@ driver depends on dev_id @
identifier drv;
identifier probefunc;
@@
static struct i2c_driver drv = {
	...,
-	.probe 		= probefunc,
+	.probe2 	= probefunc,
	...,
};


// Convert the probe function

@ depends on driver @
identifier driver.probefunc;
identifier client;
identifier id;
@@
static int probefunc(
	struct i2c_client *client,
-	const struct i2c_device_id *id
	)
	{ ... }