This folder contains the systemd service units and associated environment
variable file that collectively enable the smart dock application to be
launched automatically as an Ubuntu Linux service on boot. It can then be
controlled using the systemctl user-space application.

# To install the systemd service units

  scp astrobee.env astrobee@dock:/res
  scp *.service /usr/lib/systemd/system
  ssh astrobee@llp
  > sudo systemctl daemon-reload
  > sudo systemctl enable astrobee
  > sudo systemctl start astrobee

# The "wants" relationship between dock service units

                                                          
    dock.service ----> network.target
                                                          

Here is what each of the services do:

* dock.service - starts the dock service after the network comes up