// C1 : Identify the driver structure

@ struct_drv @
identifier drv;
@@
struct i2c_driver drv = { ... };


@ init depends on struct_drv @
declarer name module_init;
identifier init_func;
@@
 module_init(init_func);

@ exit depends on struct_drv @
declarer name module_exit;
identifier exit_func;
@@
 module_exit(exit_func);

@ find_init depends on struct_drv @
identifier struct_drv.drv;
identifier init.init_func;
@@
int init_func(void)
{
       return i2c_add_driver(&drv);
}


@ find_exit depends on find_init @
identifier struct_drv.drv;
identifier exit.exit_func;
@@
void exit_func(void)
{
       i2c_del_driver(&drv);
}

// Actions : Remove entries

@ rm_init depends on find_exit @
identifier struct_drv.drv;
identifier init.init_func;
@@
-int init_func(void)
-{
-       return i2c_add_driver(&drv);
-}


@ rm_exit depends on find_exit @
identifier struct_drv.drv;
identifier exit.exit_func;
@@
-void exit_func(void)
-{
-       i2c_del_driver(&drv);
-}

@ rm_mod_init depends on rm_init @
identifier init.init_func;
@@
- module_init(init_func);


@ rm_mod_exit depends on rm_exit @
identifier exit.exit_func;
identifier struct_drv.drv;
@@
- module_exit(exit_func);

