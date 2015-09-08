// Remove all I2C Device Tables, when there is an OF table present
//
// Aim to rename the probe function, and adapt its prototype, if:
// - The driver supports an OF table already
// - The driver is not currently using the 'id' variable in the probe

//
// First identify that we are in a suitable file, and assumptions met
//

// C1 : Identify the presence of the OF device id table

@ of_dev_id_present @
identifier arr;
@@
struct of_device_id arr[] = { ... };

// C2 : Identify the i2c_device_id array

@ dev_id depends on of_dev_id_present @
identifier arr;
@@
struct i2c_device_id arr[] = { ... };


// C3 : Identify our Probe function

@ driver depends on dev_id @
identifier drv;
identifier probefunc;
@@
struct i2c_driver drv = {
	...,
	.probe 		= probefunc,
	...,
};

// C4 : Detect if id is used in the probe function

@ probe_id_unused depends on driver @
identifier driver.probefunc;
identifier client;
identifier id;
@@
static int probefunc( struct i2c_client *client, const struct i2c_device_id *id)
 {
 ...
 when != id
 }


//
// Then, only if requirements are met, start making adjustments
//

@ script:python depends on driver && !probe_id_unused @
@@
print("Probe function uses the ID parameter")
cocci.exit()


// A : Convert the probe function

@ depends on probe_id_unused @
identifier driver.probefunc;
identifier client;
identifier id;
@@
static int probefunc(
	struct i2c_client *client
-	, const struct i2c_device_id *id
	)
	{ ... }

// B : Temporarily rename the probe to our temp .probe_new

@ depends on probe_id_unused @
identifier drv;
identifier driver.probefunc;
@@
struct i2c_driver drv = {
	...,
-	.probe 		= probefunc,
+	.probe_new 	= probefunc,
	...,
};

// C : Remove the i2c_device_id reference

@ depends on probe_id_unused @
identifier drv;
identifier dev_id.arr;
@@
struct i2c_driver drv = {
	...,
-	.id_table	= arr,
	...,
};

// D : Remove the i2c_device_id array

@ depends on probe_id_unused @
identifier arr;
@@
- struct i2c_device_id arr[] = { ... };

// E : Now remove the MODULE_DEVICE_TABLE entry

@ depends on probe_id_unused @
declarer name MODULE_DEVICE_TABLE;
identifier i2c;
identifier dev_id.arr;
@@
- MODULE_DEVICE_TABLE(i2c, arr);


