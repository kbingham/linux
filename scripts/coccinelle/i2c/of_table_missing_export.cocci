// Look for I2C drivers without an exported of_device_id table
//

// C1 : Identify the i2c_device_id array

@ i2c_dev_id @
identifier arr;
@@
struct i2c_device_id arr[] = { ... };

// C2 : For now, we only want to match on I2C drivers

@ dev_id depends on i2c_dev_id @
identifier arr;
@@
struct of_device_id arr[] = { ... };

// C2 : Check if we already export the MODULE_DEVICE_TABLE

@ of_dev_table depends on dev_id @
declarer name MODULE_DEVICE_TABLE;
identifier of;
identifier dev_id.arr;
@@
 MODULE_DEVICE_TABLE(of, arr);


// A1: Export it!

@ add_mod_dev_table depends on !of_dev_table @
identifier dev_id.arr;
@@
struct of_device_id arr[] = { ... };
+ MODULE_DEVICE_TABLE(of, arr);
