This folder contains the systemd service units and associated environment
variable file that collectively enable the flight software to be
launched automatically as an Ubuntu Linux service on boot. Flight software
can then be controlled using the systemctl user-space application.

# To install the systemd service units

  scp astrobee.env astrobee@llp:/res
  scp *.service /usr/lib/systemd/system
  ssh astrobee@llp
  > sudo systemctl daemon-reload
  > sudo systemctl enable fsw
  > sudo systemctl start fsw

# The "wants" relationship between service units

                                                          
                  +-----> mlp.service ----> network.target
                  |                             |         
            fsw.service-> ros.service ----------+         
               |  |                                       
               |  +-----> eps.service ----------+         
               |                                |         
               +--------> pmc.service ------> res.mount   
                                                          

Here is what each of the services do:

* mlp.service - blocks until the MLP can be pinged (timeout 60s)
* eps.service - queries the EPS for mode and writes if off-nominal to /res/mode
* pmc.service - queries the PMC to see if the propulation modules are not found
* ros.service - starts rosmaster in --core mode (no logging)
* fsw.service - starts flight software in the existing ROS code