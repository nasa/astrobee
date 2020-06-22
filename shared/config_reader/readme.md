\page config_reader Config Reader

This library assists in reading values from Lua config files. It also provides a way to check if files have been changed so that a program can reload the config values if desired.

## Usage
To use this library, specify the config files needed by using the add files function. After all the files have been added, call the read files function to load the variables into Lua.

For the purpose of this config reader, Lua is made up of four basic types: booleans, numbers, strings, and tables. Use the getter functions along with the variable names to read these types out of Lua. The getter functions will return true if the value was successfully read or it will return false if the variable name doesn't exist or if the wrong type getter was used. For instance, if a value is of type int and the GetBool function is used, the function will fail. A check value exists function is provide so that a programmer can check to see if a value exists before trying to read it.

A table class is provide to help read data out of Lua tables. Lua tables can be either an array (meaning the value in the table can be extracted using an index), a map (meaning the value in the table can be extracted using a variable name), or both. The get size function can be used to get the size of a table that is an array or the size of the array portion of a table.

If a timer is setup to call the check file updated function, the config reader will periodically check to see if the files added to it have been modified. Be sure to pass a reload function to the check file updated function so that your reload function will be called when a file is modified. Also, you must call the read files function in the reload function so that the values can be reloaded into Lua.
