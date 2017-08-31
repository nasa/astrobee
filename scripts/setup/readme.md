# Temporary instructions to build a VM

```
vagrant up
vagrant ssh

# is a reboot always required?
sudo reboot
vagrant ssh
# reboot again to finally get the shared folders?

# bring manually the custom debs for now
cd /vagrant
mkdir arsdebs
cd arsdebs
  # something like: scp spheresgoat:debs/\*amd64.\* .

# add extra repos
sudo ./scripts/platform/desktop/add_ros_repository.sh
sudo ./scripts/platform/desktop/add_local_repository.sh

# install dependencies
sudo ./scripts/platform/desktop/install_desktop_16_04_packages.sh

```
