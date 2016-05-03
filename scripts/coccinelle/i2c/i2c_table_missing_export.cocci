// Look for I2C drivers without an exported i2c_device_id table,
// and export it using the MODULE_DEVICE_TABLE();
//
// Usage:
// spatch --sp-file scripts/coccinelle/i2c/i2c_table_missing_export.cocci . --in-place

// C1 : Identify the i2c_device_id array

@ dev_id @
identifier arr;
@@
struct i2c_device_id arr[] = { ... };

// C2 : Check if we already export the MODULE_DEVICE_TABLE

@ i2c_dev_table depends on dev_id @
declarer name MODULE_DEVICE_TABLE;
identifier i2c;
identifier dev_id.arr;
@@
 MODULE_DEVICE_TABLE(i2c, arr);


// A1: Export it!

@ add_mod_dev_table depends on !i2c_dev_table @
identifier dev_id.arr;
@@
struct i2c_device_id arr[] = { ... };
+ MODULE_DEVICE_TABLE(i2c, arr);
